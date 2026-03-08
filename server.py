from flask import Flask, jsonify, send_from_directory, request
from flask_cors import CORS
import sqlite3
import os
import json
import heapq
from collections import deque

app = Flask(__name__)
CORS(app)

DB_PATH = 'data/caddymate_store.db'
LAYOUT_PATH = 'store_layout.json'


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


def find_path(start, end, grid_resolution=0.5):
    layout = load_normalized_layout()
    world_width = layout['world_width']
    world_height = layout['world_height']
    shelves = layout['shelves']

    if not (0 <= start['x'] <= world_width and 0 <= start['y'] <= world_height):
        return None
    if not (0 <= end['x'] <= world_width and 0 <= end['y'] <= world_height):
        return None

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

    start_cell = point_to_cell(start, grid_resolution)
    end_cell = point_to_cell(end, grid_resolution)

    start_cell = find_nearest_free_cell(start_cell, blocked_cells, columns, rows)
    end_cell = find_nearest_free_cell(end_cell, blocked_cells, columns, rows)

    if not start_cell or not end_cell:
        return None

    open_heap = []
    heapq.heappush(open_heap, (0, start_cell))
    came_from = {}
    g_score = {start_cell: 0}

    def heuristic(cell):
        return abs(cell[0] - end_cell[0]) + abs(cell[1] - end_cell[1])

    neighbors = [(1, 0), (-1, 0), (0, 1), (0, -1)]

    while open_heap:
        _, current = heapq.heappop(open_heap)

        if current == end_cell:
            cell_path = reconstruct_cell_path(came_from, end_cell)
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

            tentative_g = g_score[current] + 1
            if tentative_g >= g_score.get(neighbor, float('inf')):
                continue

            came_from[neighbor] = current
            g_score[neighbor] = tentative_g
            f_score = tentative_g + heuristic(neighbor)
            heapq.heappush(open_heap, (f_score, neighbor))

    return None

def get_db_connection():
    conn = sqlite3.connect(DB_PATH)
    conn.row_factory = sqlite3.Row
    return conn

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
    return jsonify(categories)

@app.route('/api/items/<int:category_id>')
def get_items_by_category(category_id):
    conn = get_db_connection()
    cursor = conn.cursor()
    cursor.execute('SELECT * FROM items WHERE category_id = ? ORDER BY name', (category_id,))
    items = [dict(row) for row in cursor.fetchall()]
    conn.close()
    return jsonify(items)


@app.route('/api/path', methods=['POST'])
def get_path():
    payload = request.get_json(silent=True) or {}
    start = parse_point(payload.get('start'))
    end = parse_point(payload.get('end'))

    if not start or not end:
        return jsonify({'error': 'Invalid start/end payload. Expected {start:{x,y}, end:{x,y}}'}), 400

    path_result = find_path(start, end, grid_resolution=0.5)
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

if __name__ == '__main__':
    app.run(debug=False, host='0.0.0.0', port=5000)
