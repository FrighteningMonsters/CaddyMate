from flask import Flask, jsonify, send_from_directory, request
from flask_cors import CORS
import sqlite3
import os
import json
import heapq
import threading
import atexit
from collections import deque
from voice_to_text import VoiceToText
from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler

try:
    from PIL import Image
    HAS_PIL = True
except ImportError:
    HAS_PIL = False

app = Flask(__name__)
CORS(app)

DB_PATH = 'data/caddymate_store.db'
LAYOUT_PATH = 'store_layout.json'
SLAM_PGM_PATH = 'lobby_final.pgm'
SLAM_YAML_PATH = 'lobby_final.yaml'
SLAM_OUTPUT_PNG = 'lobby_map.png'
ROS_CONFIG_PATH = 'ros_config.json'
BASE_DIR = os.path.dirname(os.path.abspath(__file__))
VOICE_MODEL_PATH = os.path.join(BASE_DIR, 'resources', 'vosk-model-small-en-us-0.15')
VOICE_USE_GRAMMAR = os.getenv('VOICE_USE_GRAMMAR', '1').strip().lower() not in {'0', 'false', 'no', 'off'}
_raw_voice_device = os.getenv('VOICE_DEVICE', '').strip()
if _raw_voice_device == '':
    VOICE_DEVICE = None
elif _raw_voice_device.lstrip('-').isdigit():
    VOICE_DEVICE = int(_raw_voice_device)
else:
    VOICE_DEVICE = _raw_voice_device

# Cache for pathfinding grid
_grid_cache = {
    'blocked_cells': None,
    'columns': None,
    'rows': None,
    'world_width': None,
    'world_height': None,
    'grid_resolution': None,
    'shelves': None,
}


class DynamixelMotorController:
    ADDR_TORQUE_ENABLE = 64
    ADDR_GOAL_VELOCITY = 104
    ADDR_PROFILE_ACCEL = 108
    ADDR_OPERATING_MODE = 11
    OPERATING_MODE_VELOCITY = 1
    PROTOCOL_VERSION = 2.0

    def __init__(
        self,
        device_name,
        baudrate=57600,
        dxl_id=1,
        speed_up=-256,
        speed_down=256,
        profile_accel=30,
    ):
        self.device_name = device_name
        self.baudrate = baudrate
        self.dxl_id = dxl_id
        self.speed_up = speed_up
        self.speed_down = speed_down
        self.profile_accel = profile_accel
        self._lock = threading.Lock()
        self._connected = False
        self._closed = False
        self._last_direction = None
        self._last_mode = 'MANUAL'
        self._port_handler = PortHandler(self.device_name)
        self._packet_handler = PacketHandler(self.PROTOCOL_VERSION)

    def _ensure_connection(self):
        if self._closed:
            raise RuntimeError('Motor controller is closed.')
        if self._connected:
            return

        if not self._port_handler.openPort():
            raise RuntimeError(f'Could not open Dynamixel port {self.device_name}.')

        if not self._port_handler.setBaudRate(self.baudrate):
            self._port_handler.closePort()
            raise RuntimeError(f'Could not set Dynamixel baudrate {self.baudrate}.')
        self._write1(self.ADDR_TORQUE_ENABLE, 0)
        self._write1(self.ADDR_OPERATING_MODE, self.OPERATING_MODE_VELOCITY)
        self._write4(self.ADDR_PROFILE_ACCEL, self.profile_accel)
        self._write1(self.ADDR_TORQUE_ENABLE, 1)
        self._connected = True
        print(f'Connected to Dynamixel on {self.device_name}')

    def _write1(self, address, value):
        dxl_comm_result, dxl_error = self._packet_handler.write1ByteTxRx(
            self._port_handler,
            self.dxl_id,
            address,
            int(value),
        )
        if dxl_comm_result != COMM_SUCCESS:
            raise RuntimeError(self._packet_handler.getTxRxResult(dxl_comm_result))
        if dxl_error != 0:
            raise RuntimeError(self._packet_handler.getRxPacketError(dxl_error))

    def _write4(self, address, value):
        write_value = int(value)
        if write_value < 0:
            write_value = (1 << 32) + write_value
        dxl_comm_result, dxl_error = self._packet_handler.write4ByteTxRx(
            self._port_handler,
            self.dxl_id,
            address,
            write_value,
        )
        if dxl_comm_result != COMM_SUCCESS:
            raise RuntimeError(self._packet_handler.getTxRxResult(dxl_comm_result))
        if dxl_error != 0:
            raise RuntimeError(self._packet_handler.getRxPacketError(dxl_error))

    def set_velocity(self, velocity):
        with self._lock:
            self._ensure_connection()
            self._write4(self.ADDR_GOAL_VELOCITY, velocity)

    def send_direction(self, direction):
        direction_upper = direction.upper()
        if direction_upper not in {'UP', 'DOWN'}:
            raise ValueError('Direction must be either "up" or "down"')

        velocity = self.speed_up if direction_upper == 'UP' else self.speed_down
        self.set_velocity(velocity)
        self._last_direction = direction_upper

    def stop(self):
        self.set_velocity(0)
        self._last_direction = None

    def send_mode(self, mode_command):
        if mode_command not in {'MANUAL', 'LOAD', 'UNLOAD'}:
            raise ValueError('Invalid mode command.')
        self._last_mode = mode_command

    def close(self):
        with self._lock:
            if self._closed:
                return

            if self._connected:
                try:
                    self._write4(self.ADDR_GOAL_VELOCITY, 0)
                except Exception:
                    pass
                try:
                    self._write1(self.ADDR_TORQUE_ENABLE, 0)
                except Exception:
                    pass
                try:
                    self._port_handler.closePort()
                except Exception:
                    pass

            self._connected = False
            self._closed = True

    @property
    def last_direction(self):
        return self._last_direction

    @property
    def last_mode(self):
        return self._last_mode


def resolve_dynamixel_port():
    configured_port = os.getenv('DYNAMIXEL_PORT', '').strip()
    if configured_port:
        return configured_port

    for candidate in ('/dev/ttyUSB0', '/dev/ttyUSB1'):
        if os.path.exists(candidate):
            return candidate

    return 'COM13'


DYNAMIXEL_PORT = resolve_dynamixel_port()
motor_controller = DynamixelMotorController(device_name=DYNAMIXEL_PORT)


def shutdown_motor_controller():
    try:
        motor_controller.close()
    except Exception as error:
        print(f'Failed to close motor controller cleanly: {error}')


atexit.register(shutdown_motor_controller)
voice_to_text = VoiceToText(
    model_path=VOICE_MODEL_PATH,
    db_path=DB_PATH,
    device=VOICE_DEVICE,
    use_grammar=VOICE_USE_GRAMMAR,
)


def parse_point(raw_value):
    if not isinstance(raw_value, dict):
        return None

    try:
        x = float(raw_value.get('x'))
        y = float(raw_value.get('y'))
    except (TypeError, ValueError):
        return None

    return {'x': x, 'y': y}


def point_in_polygon(point_x, point_y, polygon):
    inside = False
    j = len(polygon) - 1

    for i in range(len(polygon)):
        xi, yi = polygon[i]
        xj, yj = polygon[j]

        intersects = ((yi > point_y) != (yj > point_y)) and (
            point_x < (xj - xi) * (point_y - yi) / ((yj - yi) or 1e-12) + xi
        )

        if intersects:
            inside = not inside

        j = i

    return inside


def load_normalized_layout(padding=1.0):
    with open(LAYOUT_PATH, 'r', encoding='utf-8') as layout_file:
        layout = json.load(layout_file)

    shelves = layout.get('shelves', [])

    min_x = float('inf')
    min_y = float('inf')
    max_x = float('-inf')
    max_y = float('-inf')

    for shelf in shelves:
        for x, y in shelf.get('polygon', []):
            min_x = min(min_x, x)
            min_y = min(min_y, y)
            max_x = max(max_x, x)
            max_y = max(max_y, y)

    if min_x == float('inf'):
        min_x = 0
        min_y = 0
        max_x = layout.get('world', {}).get('width', 50)
        max_y = layout.get('world', {}).get('height', 40)

    world_width = (max_x - min_x) + padding * 2
    world_height = (max_y - min_y) + padding * 2
    offset_x = padding - min_x
    offset_y = padding - min_y

    normalized_shelves = []
    for shelf in shelves:
        normalized_polygon = [
            [x + offset_x, y + offset_y] for x, y in shelf.get('polygon', [])
        ]
        normalized_shelves.append(normalized_polygon)

    return {
        'world_width': world_width,
        'world_height': world_height,
        'shelves': normalized_shelves,
    }


def point_to_cell(point, grid_resolution):
    return (
        int(point['x'] / grid_resolution),
        int(point['y'] / grid_resolution),
    )


def cell_center(cell_x, cell_y, grid_resolution):
    return {
        'x': (cell_x + 0.5) * grid_resolution,
        'y': (cell_y + 0.5) * grid_resolution,
    }


def find_nearest_free_cell(start_cell, blocked_cells, columns, rows):
    sx, sy = start_cell
    if 0 <= sx < columns and 0 <= sy < rows and start_cell not in blocked_cells:
        return start_cell

    queue = deque([start_cell])
    visited = {start_cell}
    neighbors = [(1, 0), (-1, 0), (0, 1), (0, -1)]

    while queue:
        cx, cy = queue.popleft()

        for dx, dy in neighbors:
            nx, ny = cx + dx, cy + dy
            cell = (nx, ny)

            if cell in visited:
                continue

            visited.add(cell)

            if not (0 <= nx < columns and 0 <= ny < rows):
                continue

            if cell not in blocked_cells:
                return cell

            queue.append(cell)

    return None


def reconstruct_cell_path(came_from, end_cell):
    path = [end_cell]
    current = end_cell

    while current in came_from:
        current = came_from[current]
        path.append(current)

    path.reverse()
    return path


def simplify_points(points):
    if len(points) < 3:
        return points

    simplified = [points[0]]

    for index in range(1, len(points) - 1):
        prev_point = simplified[-1]
        current_point = points[index]
        next_point = points[index + 1]

        same_x = abs(prev_point['x'] - current_point['x']) < 1e-6 and abs(current_point['x'] - next_point['x']) < 1e-6
        same_y = abs(prev_point['y'] - current_point['y']) < 1e-6 and abs(current_point['y'] - next_point['y']) < 1e-6

        if same_x or same_y:
            continue

        simplified.append(current_point)

    simplified.append(points[-1])
    return simplified


def initialize_grid_cache(grid_resolution=1.0):
    """Pre-compute and cache the pathfinding grid to avoid recalculating on every request."""
    layout = load_normalized_layout()
    world_width = layout['world_width']
    world_height = layout['world_height']
    shelves = layout['shelves']

    columns = max(1, int(world_width / grid_resolution) + 1)
    rows = max(1, int(world_height / grid_resolution) + 1)

    blocked_cells = set()
    for y in range(rows):
        center_y = (y + 0.5) * grid_resolution
        for x in range(columns):
            center_x = (x + 0.5) * grid_resolution
            for polygon in shelves:
                if point_in_polygon(center_x, center_y, polygon):
                    blocked_cells.add((x, y))
                    break

    _grid_cache['blocked_cells'] = blocked_cells
    _grid_cache['columns'] = columns
    _grid_cache['rows'] = rows
    _grid_cache['world_width'] = world_width
    _grid_cache['world_height'] = world_height
    _grid_cache['grid_resolution'] = grid_resolution
    _grid_cache['shelves'] = shelves

    print(f"Grid cache initialized: {columns}x{rows} cells, {len(blocked_cells)} blocked")


def find_path(start, end, grid_resolution=1.0):
    # Initialize cache if not already done
    if _grid_cache['blocked_cells'] is None or _grid_cache['grid_resolution'] != grid_resolution:
        initialize_grid_cache(grid_resolution)

    world_width = _grid_cache['world_width']
    world_height = _grid_cache['world_height']
    columns = _grid_cache['columns']
    rows = _grid_cache['rows']
    blocked_cells = _grid_cache['blocked_cells']

    if not (0 <= start['x'] <= world_width and 0 <= start['y'] <= world_height):
        return None
    if not (0 <= end['x'] <= world_width and 0 <= end['y'] <= world_height):
        return None

    start_cell = point_to_cell(start, grid_resolution)
    end_cell = point_to_cell(end, grid_resolution)

    start_cell = find_nearest_free_cell(start_cell, blocked_cells, columns, rows)
    end_cell = find_nearest_free_cell(end_cell, blocked_cells, columns, rows)

    if not start_cell or not end_cell:
        return None

    # A* with direction tracking to minimize turns
    open_heap = []
    # State: (cell, direction) where direction is None for start or (dx, dy)
    heapq.heappush(open_heap, (0, start_cell, None))
    
    came_from = {}  # (cell, direction) -> (prev_cell, prev_direction)
    g_score = {}  # (cell, direction) -> cost
    
    # Start has no direction, cost 0
    g_score[(start_cell, None)] = 0

    def heuristic(cell):
        return abs(cell[0] - end_cell[0]) + abs(cell[1] - end_cell[1])

    neighbors = [(1, 0), (-1, 0), (0, 1), (0, -1)]
    TURN_PENALTY = 0.5  # Cost penalty for changing direction

    while open_heap:
        _, current, current_dir = heapq.heappop(open_heap)

        if current == end_cell:
            # Found the goal - reconstruct path
            cell_path = []
            state = (current, current_dir)
            while state in came_from:
                cell_path.append(state[0])
                state = came_from[state]
            cell_path.append(start_cell)
            cell_path.reverse()

            points = [cell_center(cx, cy, grid_resolution) for cx, cy in cell_path]

            if points:
                points[0] = {'x': start['x'], 'y': start['y']}
                points[-1] = {'x': end['x'], 'y': end['y']}

            return {
                'points': simplify_points(points),
                'grid_resolution': grid_resolution,
                'world_width': world_width,
                'world_height': world_height,
            }

        for dx, dy in neighbors:
            nx, ny = current[0] + dx, current[1] + dy
            neighbor = (nx, ny)

            if not (0 <= nx < columns and 0 <= ny < rows):
                continue
            if neighbor in blocked_cells:
                continue

            # Base movement cost
            move_cost = 1.0
            
            # Add turn penalty if changing direction
            if current_dir is not None and current_dir != (dx, dy):
                move_cost += TURN_PENALTY

            current_state = (current, current_dir)
            neighbor_state = (neighbor, (dx, dy))
            
            tentative_g = g_score.get(current_state, float('inf')) + move_cost
            
            if tentative_g < g_score.get(neighbor_state, float('inf')):
                came_from[neighbor_state] = current_state
                g_score[neighbor_state] = tentative_g
                f_score = tentative_g + heuristic(neighbor)
                heapq.heappush(open_heap, (f_score, neighbor, (dx, dy)))

    return None

def get_db_connection():
    conn = sqlite3.connect(DB_PATH)
    conn.row_factory = sqlite3.Row
    return conn


def json_no_store(payload):
    resp = jsonify(payload)
    resp.headers['Cache-Control'] = 'no-store, max-age=0'
    resp.headers['Pragma'] = 'no-cache'
    return resp

@app.route('/')
def index():
    return send_from_directory('.', 'home.html')

@app.route('/<path:path>')
def serve_file(path):
    return send_from_directory('.', path)

@app.route('/api/categories')
def get_categories():
    conn = get_db_connection()
    cursor = conn.cursor()
    cursor.execute('SELECT * FROM categories ORDER BY name')
    categories = [dict(row) for row in cursor.fetchall()]
    conn.close()
    return json_no_store(categories)

@app.route('/api/items/<int:category_id>')
def get_items_by_category(category_id):
    conn = get_db_connection()
    cursor = conn.cursor()
    cursor.execute('SELECT * FROM items WHERE category_id = ? ORDER BY name', (category_id,))
    items = [dict(row) for row in cursor.fetchall()]
    conn.close()
    return json_no_store(items)

@app.route('/api/items')
def get_all_items():
    conn = get_db_connection()
    cursor = conn.cursor()
    cursor.execute('SELECT * FROM items ORDER BY name')
    items = [dict(row) for row in cursor.fetchall()]
    conn.close()
    return json_no_store(items)

@app.route('/api/path', methods=['POST'])
def get_path():
    payload = request.get_json(silent=True) or {}
    start = parse_point(payload.get('start'))
    end = parse_point(payload.get('end'))

    if not start or not end:
        return jsonify({'error': 'Invalid start/end payload. Expected {start:{x,y}, end:{x,y}}'}), 400

    path_result = find_path(start, end, grid_resolution=1.0)
    if not path_result:
        return jsonify({'error': 'No path found'}), 404

    return jsonify({
        'points': path_result['points'],
        'meta': {
            'gridResolution': path_result['grid_resolution'],
            'worldWidth': path_result['world_width'],
            'worldHeight': path_result['world_height'],
        },
    })


def convert_slam_pgm_to_png():
    """Convert lobby_final.pgm to styled lobby_map.png for UI display."""
    if not HAS_PIL or not os.path.isfile(SLAM_PGM_PATH):
        return
    try:
        img = Image.open(SLAM_PGM_PATH).convert('L')
        pixels = img.load()
        w, h = img.size
        out = Image.new('RGBA', (w, h))
        out_pixels = out.load()
        for y in range(h):
            for x in range(w):
                v = pixels[x, y]
                if v >= 205:
                    out_pixels[x, y] = (248, 250, 252, 255)
                elif v <= 50:
                    out_pixels[x, y] = (30, 41, 59, 255)
                else:
                    out_pixels[x, y] = (148, 163, 184, 255)
        out.save(SLAM_OUTPUT_PNG)
        print(f"SLAM map converted: {SLAM_OUTPUT_PNG}")
    except Exception as e:
        print(f"SLAM map conversion failed: {e}")


def load_slam_map_info():
    """Load map metadata from lobby_final.yaml."""
    if not os.path.isfile(SLAM_YAML_PATH):
        return None
    try:
        with open(SLAM_YAML_PATH, 'r', encoding='utf-8') as f:
            data = f.read()
        resolution = 0.05
        origin_x, origin_y = -7.75, -6.35
        for line in data.splitlines():
            line = line.strip()
            if line.startswith('resolution:'):
                resolution = float(line.split(':', 1)[1].strip())
            elif line.startswith('origin:'):
                rest = line.split(':', 1)[1].strip().strip('[]')
                parts = rest.split(',')
                if len(parts) >= 2:
                    origin_x = float(parts[0].strip())
                    origin_y = float(parts[1].strip())
        if os.path.isfile(SLAM_PGM_PATH) and HAS_PIL:
            with Image.open(SLAM_PGM_PATH) as img:
                w, h = img.size
        else:
            w, h = 372, 278
        return {
            'resolution': resolution,
            'origin_x': origin_x,
            'origin_y': origin_y,
            'width_px': w,
            'height_px': h,
            'world_width_m': w * resolution,
            'world_height_m': h * resolution,
        }
    except Exception:
        return None


@app.route('/api/map_info')
def get_map_info():
    info = load_slam_map_info()
    if info is None:
        return jsonify({'error': 'SLAM map not available'}), 404
    return jsonify(info)


@app.route('/api/ros_config')
def get_ros_config():
    if not os.path.isfile(ROS_CONFIG_PATH):
        return jsonify({'rosbridge_host': 'localhost', 'rosbridge_port': 9090})
    try:
        with open(ROS_CONFIG_PATH, 'r', encoding='utf-8') as f:
            cfg = json.load(f)
        return jsonify({
            'rosbridge_host': cfg.get('rosbridge_host', 'localhost'),
            'rosbridge_port': cfg.get('rosbridge_port', 9090),
        })
    except Exception:
        return jsonify({'rosbridge_host': 'localhost', 'rosbridge_port': 9090})


def build_voice_status_payload():
    payload = voice_to_text.get_status()
    payload['available'] = voice_to_text.is_available()
    availability_error = voice_to_text.availability_error()
    if availability_error and not payload.get('last_error'):
        payload['last_error'] = availability_error
    return payload


@app.route('/api/voice/status')
def get_voice_status():
    return jsonify(build_voice_status_payload())


@app.route('/api/voice/start', methods=['POST'])
def start_voice_input():
    availability_error = voice_to_text.availability_error()
    if availability_error:
        payload = build_voice_status_payload()
        payload['error'] = availability_error
        return jsonify(payload), 503

    started = voice_to_text.start()
    payload = build_voice_status_payload()

    if not started:
        payload['error'] = payload.get('last_error') or 'Failed to start local speech recognition.'
        return jsonify(payload), 500

    payload['state'] = 'running'
    return jsonify(payload)


@app.route('/api/voice/stop', methods=['POST'])
def stop_voice_input():
    try:
        final_text = voice_to_text.stop()
        payload = build_voice_status_payload()
        payload['state'] = 'stopped'
        payload['final_text'] = final_text or payload.get('final_text', '')
        return jsonify(payload)
    except Exception as error:
        return jsonify({'error': f'Failed to stop local speech recognition: {error}'}), 500


@app.route('/api/motor/start', methods=['POST'])
def start_motor():
    payload = request.get_json(silent=True) or {}
    direction = (payload.get('direction') or '').strip().lower()

    if direction not in {'up', 'down'}:
        return jsonify({'error': 'Invalid direction. Use "up" or "down".'}), 400

    try:
        motor_controller.send_direction(direction)
        return jsonify({
            'status': 'running', 
            'direction': direction,
            'current_mode': motor_controller.last_mode
        })
    except Exception as error:
        return jsonify({'error': f'Communication error: {error}'}), 500


@app.route('/api/motor/stop', methods=['POST'])
def stop_motor():
    try:
        motor_controller.stop()
        return jsonify({'status': 'stopped'})
    except Exception as error:
        return jsonify({'error': f'Failed to stop motor: {error}'}), 500


@app.route('/api/motor/mode', methods=['POST'])
def set_motor_mode():
    payload = request.get_json(silent=True) or {}
    mode = (payload.get('mode') or '').strip().lower()

    mode_command_map = {
        'manual': 'MANUAL',
        'loading': 'LOAD',
        'unloading': 'UNLOAD',
    }

    mode_command = mode_command_map.get(mode)
    if mode_command is None:
        return jsonify({'error': 'Invalid mode. Use "manual", "loading", or "unloading".'}), 400

    try:
        motor_controller.send_mode(mode_command)
        return jsonify({'status': 'ok', 'mode': mode})
    except ValueError as error:
        return jsonify({'error': str(error)}), 400
    except Exception as error:
        return jsonify({'error': f'Failed to set motor mode: {error}'}), 500


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5000)
    args = parser.parse_args()
    convert_slam_pgm_to_png()
    print("Initializing pathfinding grid cache...")
    initialize_grid_cache(grid_resolution=1.0)
    print(f"Starting server on port {args.port}...")
    app.run(debug=False, host='0.0.0.0', port=args.port)
