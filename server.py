from flask import Flask, jsonify, send_from_directory, request
from flask_cors import CORS
import sqlite3
import os
import json
import heapq
import threading
import atexit
import re
from collections import deque
from voice_to_text import VoiceToText
from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler
import time

try:
    import termios
    HAS_TERMIOS = True
except ImportError:
    termios = None
    HAS_TERMIOS = False

try:
    import serial
    HAS_SERIAL = True
except ImportError:
    HAS_SERIAL = False

try:
    from PIL import Image
    HAS_PIL = True
except ImportError:
    HAS_PIL = False

app = Flask(__name__)
CORS(app)

DB_PATH = 'data/caddymate_store.db'
LAYOUT_PATH = 'store_layout.json'
SLAM_PGM_PATH = 'lab_final.pgm'
SLAM_YAML_PATH = 'lab_final.yaml'
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

# Cache for SLAM map pathfinding (pixel-based A*)
_slam_path_cache = {
    'obstacle_map': None,  # 2D list[row][col]: True = obstacle
    'cost_map': None,      # 2D list[row][col]: float, INF = impassable
    'width': None,
    'height': None,
}


class DynamixelMotorController:
    ADDR_TORQUE_ENABLE = 64
    ADDR_GOAL_VELOCITY = 104
    ADDR_PROFILE_ACCEL = 108
    ADDR_OPERATING_MODE = 11
    ADDR_PRESENT_POSITION = 132
    SINGLE_TURN_TICKS = 4096
    OPERATING_MODE_VELOCITY = 1
    PROTOCOL_VERSION = 2.0
    SOFTWARE_LIMIT_LOOP_SECONDS = 0.05

    def __init__(
        self,
        device_name,
        baudrate=57600,
        dxl_id=1,
        speed_up=-256,
        speed_down=256,
        profile_accel=30,
        top_to_bottom_ticks=12000,
        down_increases_position=True,
        swap_direction_commands=False,
    ):
        self.device_name = device_name
        self.baudrate = baudrate
        self.dxl_id = dxl_id
        self.speed_up = speed_up
        self.speed_down = speed_down
        self.profile_accel = profile_accel
        self.top_to_bottom_ticks = max(0, int(top_to_bottom_ticks))
        self.down_increases_position = bool(down_increases_position)
        self.swap_direction_commands = bool(swap_direction_commands)
        self._lock = threading.Lock()
        self._connected = False
        self._closed = False
        self._last_direction = None
        self._last_mode = 'MANUAL'
        self._top_position = None
        self._top_position_raw = None
        self._bottom_position = None
        self._soft_min_limit = None
        self._soft_max_limit = None
        self._software_limit_stop_count = 0
        self._limit_stop_event = threading.Event()
        self._limit_thread = None
        self._port_handler = PortHandler(self.device_name)
        self._packet_handler = PacketHandler(self.PROTOCOL_VERSION)

    def _flush(self):

        if not HAS_TERMIOS:
            if hasattr(self._port_handler, 'ser') and self._port_handler.ser is not None:
                try:
                    self._port_handler.ser.reset_input_buffer()
                    self._port_handler.ser.reset_output_buffer()
                except Exception:
                    pass
            return

        if hasattr(self._port_handler, 'fd') and self._port_handler.fd is not None:
            termios.tcflush(self._port_handler.fd, termios.TCIOFLUSH)

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

        # 1. Clear the buffer of any initial noise
        self._flush()

        # 2. Disable Torque first. 
        # If the Pi crashed earlier, Torque might still be 1.
        try:
            self._write1(self.ADDR_TORQUE_ENABLE, 0)
        except:
            pass # Ignore if it was already off
        
        time.sleep(0.1) # Give the motor a moment to process

        # 3. Set Operating Mode (Velocity)
        self._write1(self.ADDR_OPERATING_MODE, self.OPERATING_MODE_VELOCITY)
        
        # 4. Set Profile Accel
        self._write4(self.ADDR_PROFILE_ACCEL, self.profile_accel)
        
        # 5. Re-enable Torque
        self._write1(self.ADDR_TORQUE_ENABLE, 1)

        # Capture top at startup and set software-only limits for velocity mode.
        self._configure_software_limits_from_top()
        self._start_limit_watcher()
        
        self._connected = True
        print(f'Connected to Dynamixel on {self.device_name}')
        print(
            f'Software limits configured from top={self._top_position} '
            f'-> bottom={self._bottom_position} '
            f'(soft_min={self._soft_min_limit}, soft_max={self._soft_max_limit})'
        )
        print(
            'Direction mapping: '
            f"DOWN {'increases' if self.down_increases_position else 'decreases'} position"
        )
        print(
            'Command mapping: '
            f"swap_directions={self.swap_direction_commands}"
        )

    def _write1(self, address, value):
        self._flush() # Clean before every write
        dxl_comm_result, dxl_error = self._packet_handler.write1ByteTxRx(
            self._port_handler, self.dxl_id, address, int(value)
        )
        if dxl_comm_result != COMM_SUCCESS:
            raise RuntimeError(self._packet_handler.getTxRxResult(dxl_comm_result))
        if dxl_error != 0:
            # If we get "Address not available", it usually means Torque was ON
            raise RuntimeError(f"Motor Error at Addr {address}: {self._packet_handler.getRxPacketError(dxl_error)}")

    def _write4(self, address, value):
        self._flush() # Clean before every write
        write_value = int(value)
        if write_value < 0:
            write_value = (1 << 32) + write_value
        dxl_comm_result, dxl_error = self._packet_handler.write4ByteTxRx(
            self._port_handler, self.dxl_id, address, write_value
        )
        if dxl_comm_result != COMM_SUCCESS:
            raise RuntimeError(self._packet_handler.getTxRxResult(dxl_comm_result))
        if dxl_error != 0:
            raise RuntimeError(f"Motor Error at Addr {address}: {self._packet_handler.getRxPacketError(dxl_error)}")

    def _read4(self, address):
        self._flush() # Clean before every read
        read_value, dxl_comm_result, dxl_error = self._packet_handler.read4ByteTxRx(
            self._port_handler, self.dxl_id, address
        )
        if dxl_comm_result != COMM_SUCCESS:
            raise RuntimeError(self._packet_handler.getTxRxResult(dxl_comm_result))
        if dxl_error != 0:
            raise RuntimeError(f"Motor Error at Addr {address}: {self._packet_handler.getRxPacketError(dxl_error)}")
        return int(read_value)

    def _to_signed_32(self, value):
        if value & (1 << 31):
            return value - (1 << 32)
        return value

    def _read_present_position(self):
        raw_value = self._read4(self.ADDR_PRESENT_POSITION)
        return self._to_signed_32(raw_value)

    def _configure_software_limits_from_top(self):
        top_position_raw = self._read_present_position()
        top_single_turn = top_position_raw % self.SINGLE_TURN_TICKS

        bottom_offset = self.top_to_bottom_ticks
        bottom_sign = 1 if self.down_increases_position else -1

        # Software limits in velocity mode should use continuous raw position.
        bottom_position = top_position_raw + (bottom_sign * bottom_offset)
        min_limit = min(top_position_raw, bottom_position)
        max_limit = max(top_position_raw, bottom_position)

        self._top_position = top_position_raw
        self._top_position_raw = top_position_raw
        self._bottom_position = bottom_position
        self._soft_min_limit = min_limit
        self._soft_max_limit = max_limit

    def _at_soft_limit_for_direction(self, direction, current_position):
        if self._soft_min_limit is None or self._soft_max_limit is None:
            return False

        if direction == 'UP':
            if self.down_increases_position:
                return current_position <= self._soft_min_limit
            return current_position >= self._soft_max_limit

        if direction == 'DOWN':
            if self.down_increases_position:
                return current_position >= self._soft_max_limit
            return current_position <= self._soft_min_limit

        return False

    def _map_requested_direction(self, direction):
        if not self.swap_direction_commands:
            return direction
        return 'DOWN' if direction == 'UP' else 'UP'

    def _stop_without_lock(self):
        self._write4(self.ADDR_GOAL_VELOCITY, 0)
        self._last_direction = None

    def _software_limit_loop(self):
        while not self._limit_stop_event.is_set():
            try:
                with self._lock:
                    if self._closed or not self._connected or self._last_direction is None:
                        pass
                    else:
                        current_position = self._read_present_position()
                        if self._at_soft_limit_for_direction(self._last_direction, current_position):
                            self._stop_without_lock()
                            self._software_limit_stop_count += 1
                            print(
                                f'Software limit stop: direction={self._last_direction}, '
                                f'current={current_position}, '
                                f'min={self._soft_min_limit}, max={self._soft_max_limit}'
                            )
            except Exception as error:
                print(f'Software limit watchdog error: {error}')

            self._limit_stop_event.wait(self.SOFTWARE_LIMIT_LOOP_SECONDS)

    def _start_limit_watcher(self):
        if self._limit_thread is not None and self._limit_thread.is_alive():
            return
        self._limit_stop_event.clear()
        self._limit_thread = threading.Thread(target=self._software_limit_loop, daemon=True)
        self._limit_thread.start()

    def set_velocity(self, velocity):
        with self._lock:
            self._ensure_connection()
            self._write4(self.ADDR_GOAL_VELOCITY, velocity)

    def send_direction(self, direction):
        direction_upper = direction.upper()
        if direction_upper not in {'UP', 'DOWN'}:
            raise ValueError('Direction must be either "up" or "down"')

        effective_direction = self._map_requested_direction(direction_upper)

        with self._lock:
            self._ensure_connection()
            current_position = self._read_present_position()
            if self._at_soft_limit_for_direction(effective_direction, current_position):
                self._stop_without_lock()
                raise RuntimeError(
                    f'Software limit reached for {direction_upper} '
                    f'(effective={effective_direction}, '
                    f'(current={current_position}, min={self._soft_min_limit}, max={self._soft_max_limit})'
                )

            velocity = self.speed_up if effective_direction == 'UP' else self.speed_down
            self._write4(self.ADDR_GOAL_VELOCITY, velocity)
            self._last_direction = effective_direction

    def stop(self):
        with self._lock:
            self._ensure_connection()
            self._stop_without_lock()

    def send_mode(self, mode_command):
        if mode_command not in {'MANUAL', 'LOAD', 'UNLOAD'}:
            raise ValueError('Invalid mode command.')
        self._last_mode = mode_command

    def close(self):
        limit_thread = None
        with self._lock:
            if self._closed:
                return

            if self._connected:
                self._limit_stop_event.set()
                limit_thread = self._limit_thread
                self._limit_thread = None
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

        if limit_thread is not None:
            limit_thread.join(timeout=1.0)

    @property
    def last_direction(self):
        return self._last_direction

    @property
    def last_mode(self):
        return self._last_mode

    def get_limit_state(self):
        return {
            'top_position': self._top_position,
            'top_position_raw': self._top_position_raw,
            'bottom_position': self._bottom_position,
            'soft_min_limit': self._soft_min_limit,
            'soft_max_limit': self._soft_max_limit,
            'software_limit_stop_count': self._software_limit_stop_count,
            'top_to_bottom_ticks': self.top_to_bottom_ticks,
            'down_increases_position': self.down_increases_position,
            'swap_direction_commands': self.swap_direction_commands,
        }

    def read_position_state(self):
        with self._lock:
            self._ensure_connection()
            current_position_raw = self._read_present_position()
            current_position = current_position_raw
            current_position_single_turn = current_position_raw % self.SINGLE_TURN_TICKS
            top_position = self._top_position

        offset_from_top = None
        if top_position is not None:
            offset_from_top = current_position - top_position

        return {
            'current_position': current_position,
            'current_position_raw': current_position_raw,
            'current_position_single_turn': current_position_single_turn,
            'top_position': top_position,
            'top_position_raw': self._top_position_raw,
            'offset_from_top': offset_from_top,
            'soft_min_limit': self._soft_min_limit,
            'soft_max_limit': self._soft_max_limit,
            'software_limit_stop_count': self._software_limit_stop_count,
        }


def resolve_dynamixel_port():
    configured_port = os.getenv('DYNAMIXEL_PORT', '').strip()
    if configured_port:
        return configured_port

    for candidate in ('/dev/ttyUSB0', '/dev/ttyUSB1'):
        if os.path.exists(candidate):
            return candidate

    return 'COM13'


DYNAMIXEL_PORT = resolve_dynamixel_port()
MOTOR_TOP_TO_BOTTOM_TICKS = int(os.getenv('MOTOR_TOP_TO_BOTTOM_TICKS', '12000'))
MOTOR_DOWN_INCREASES_POSITION = os.getenv('MOTOR_DOWN_INCREASES_POSITION', '1').strip().lower() not in {'0', 'false', 'no', 'off'}
MOTOR_SWAP_DIRECTION_COMMANDS = os.getenv('MOTOR_SWAP_DIRECTION_COMMANDS', '0').strip().lower() not in {'0', 'false', 'no', 'off'}
MOTOR_OFFSET_DEBUG_PRINT = os.getenv('MOTOR_OFFSET_DEBUG_PRINT', '1').strip().lower() not in {'0', 'false', 'no', 'off'}
MOTOR_OFFSET_DEBUG_INTERVAL_SECONDS = float(os.getenv('MOTOR_OFFSET_DEBUG_INTERVAL_SECONDS', '0.5'))
motor_controller = DynamixelMotorController(
    device_name=DYNAMIXEL_PORT,
    top_to_bottom_ticks=MOTOR_TOP_TO_BOTTOM_TICKS,
    down_increases_position=MOTOR_DOWN_INCREASES_POSITION,
    swap_direction_commands=MOTOR_SWAP_DIRECTION_COMMANDS,
)


class MotorOffsetCalibrationPrinter:
    def __init__(self, motor_ctrl, interval_seconds=0.5):
        self._motor = motor_ctrl
        self._interval = max(0.1, float(interval_seconds))
        self._stop_event = threading.Event()
        self._thread = None

    def _run_loop(self):
        while not self._stop_event.is_set():
            try:
                state = self._motor.read_position_state()
                print(
                    '[Motor Cal] '
                    f"current={state['current_position']} "
                    f"top={state['top_position']} "
                    f"offset_from_top={state['offset_from_top']} "
                    f"soft_min={state['soft_min_limit']} "
                    f"soft_max={state['soft_max_limit']} "
                    f"soft_stops={state['software_limit_stop_count']}"
                )
            except Exception as error:
                print(f'[Motor Cal] read failed: {error}')

            self._stop_event.wait(self._interval)

    def start(self):
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)


motor_offset_calibration_printer = MotorOffsetCalibrationPrinter(
    motor_controller,
    interval_seconds=MOTOR_OFFSET_DEBUG_INTERVAL_SECONDS,
)

if MOTOR_OFFSET_DEBUG_PRINT:
    motor_offset_calibration_printer.start()


class MotorAutomationController:
    SENSOR_COUNT = 4
    NO_OBJECT_DISTANCE_CM = 44.0
    IGNORE_ABOVE_CM = 45.0
    LOOP_INTERVAL_SECONDS = 0.1
    LOAD_CONFIRMATION_COUNT = 5
    READ_MATCH_TOLERANCE_CM = 1.0

    def __init__(self, motor_ctrl):
        self._motor = motor_ctrl
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread = None
        self._last_auto_direction = None
        self._state = {
            'sensor_cm': [self.NO_OBJECT_DISTANCE_CM] * self.SENSOR_COUNT,
            'last_read_cm': [None] * self.SENSOR_COUNT,
            'same_read_count': [0] * self.SENSOR_COUNT,
            'updated_at': None,
        }

    def _normalize_sensor_value(self, raw_value):
        try:
            value = float(raw_value)
        except (TypeError, ValueError):
            return self.NO_OBJECT_DISTANCE_CM

        if value > self.IGNORE_ABOVE_CM:
            return self.NO_OBJECT_DISTANCE_CM
        if value < 0:
            return self.NO_OBJECT_DISTANCE_CM
        return value

    def update_sensors(self, sensor_values):
        updates_by_index = {}

        if isinstance(sensor_values, dict):
            for raw_index, value in sensor_values.items():
                try:
                    index = int(raw_index)
                except (TypeError, ValueError):
                    continue
                if 0 <= index < self.SENSOR_COUNT and value is not None:
                    updates_by_index[index] = self._normalize_sensor_value(value)
        elif isinstance(sensor_values, list):
            if len(sensor_values) > self.SENSOR_COUNT:
                raise ValueError(f'Expected up to {self.SENSOR_COUNT} sensor readings.')
            for index, value in enumerate(sensor_values):
                if value is None:
                    continue
                updates_by_index[index] = self._normalize_sensor_value(value)
        else:
            raise ValueError('Sensor payload must be a list or object.')

        if not updates_by_index:
            raise ValueError('No valid sensor readings were provided.')

        with self._lock:
            next_values = list(self._state['sensor_cm'])
            next_last_read = list(self._state['last_read_cm'])
            next_same_count = list(self._state['same_read_count'])

            for index, value in updates_by_index.items():
                last_value = next_last_read[index]
                if last_value is not None and abs(value - last_value) <= self.READ_MATCH_TOLERANCE_CM:
                    next_same_count[index] += 1
                else:
                    next_same_count[index] = 1

                next_values[index] = value
                next_last_read[index] = value

            self._state['sensor_cm'] = next_values
            self._state['last_read_cm'] = next_last_read
            self._state['same_read_count'] = next_same_count
            self._state['updated_at'] = time.time()

    def _sensor_detected(self, sensor_values):
        # Any sensor reading below the no-object baseline means an object is present.
        return any(value < self.NO_OBJECT_DISTANCE_CM for value in sensor_values)

    def _sensor_confirmed_for_load(self, sensor_values, same_read_count):
        for index, value in enumerate(sensor_values):
            if value < self.NO_OBJECT_DISTANCE_CM and same_read_count[index] >= self.LOAD_CONFIRMATION_COUNT:
                return True
        return False

    def _compute_direction(self, mode_command, sensor_values, same_read_count):
        detected = self._sensor_detected(sensor_values)
        confirmed_for_load = self._sensor_confirmed_for_load(sensor_values, same_read_count)

        if mode_command == 'LOAD':
            return 'DOWN' if confirmed_for_load else None
        if mode_command == 'UNLOAD':
            return 'UP' if not detected else None
        return None

    def _run_loop(self):
        while not self._stop_event.is_set():
            with self._lock:
                sensor_values = list(self._state['sensor_cm'])
                same_read_count = list(self._state['same_read_count'])

            mode_command = self._motor.last_mode
            desired_direction = self._compute_direction(mode_command, sensor_values, same_read_count)
            should_force_stop = (
                mode_command in {'LOAD', 'UNLOAD'}
                and desired_direction is None
                and self._motor.last_direction is not None
            )

            if desired_direction != self._last_auto_direction or should_force_stop:
                try:
                    if desired_direction is None:
                        self._motor.stop()
                    else:
                        self._motor.send_direction(desired_direction)
                except Exception as error:
                    print(f'Auto motor control error: {error}')
                finally:
                    self._last_auto_direction = desired_direction

            self._stop_event.wait(self.LOOP_INTERVAL_SECONDS)

    def start(self):
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)

    def get_state(self):
        with self._lock:
            return {
                'sensor_cm': list(self._state['sensor_cm']),
                'same_read_count': list(self._state['same_read_count']),
                'updated_at': self._state['updated_at'],
                'mode': self._motor.last_mode,
                'auto_direction': self._last_auto_direction,
            }


motor_automation = MotorAutomationController(motor_controller)
motor_automation.start()


class ArduinoUltrasonicSerialReader:
    SENSOR_COUNT = 4
    DEFAULT_BAUDRATE = 115200
    SENSOR_PATTERN = re.compile(r'S(\d+)\s*:\s*(-?\d+(?:\.\d+)?)')

    def __init__(self, motor_auto_ctrl, port=None, baudrate=None):
        self._motor_auto_ctrl = motor_auto_ctrl
        self._port = port or self._resolve_default_port()
        self._baudrate = int(baudrate or self.DEFAULT_BAUDRATE)
        self._lock = threading.Lock()
        self._stop_event = threading.Event()
        self._thread = None
        self._serial_conn = None
        self._state = {
            'enabled': HAS_SERIAL,
            'port': self._port,
            'baudrate': self._baudrate,
            'connected': False,
            'last_line': None,
            'last_update_at': None,
            'parse_errors': 0,
            'last_error': None,
        }

    def _resolve_default_port(self):
        configured_port = os.getenv('ULTRASONIC_SERIAL_PORT', '').strip()
        if configured_port:
            return configured_port

        for candidate in ('/dev/ttyACM0', '/dev/ttyACM1', '/dev/serial0'):
            if os.path.exists(candidate):
                return candidate

        return 'COM14'

    def _set_state(self, **updates):
        with self._lock:
            self._state.update(updates)

    def _parse_sensor_line(self, line):
        matches = self.SENSOR_PATTERN.findall(line)
        if not matches:
            return None

        by_index = {}
        for raw_index, raw_value in matches:
            index = int(raw_index)
            if index < 0 or index >= self.SENSOR_COUNT:
                continue
            by_index[index] = float(raw_value)

        if not by_index:
            return None

        return by_index

    def _open_serial(self):
        return serial.Serial(self._port, self._baudrate, timeout=1)

    def _close_serial(self):
        if self._serial_conn is None:
            return
        try:
            self._serial_conn.close()
        except Exception:
            pass
        finally:
            self._serial_conn = None

    def _run_loop(self):
        if not HAS_SERIAL:
            self._set_state(last_error='pyserial is not installed.')
            return

        while not self._stop_event.is_set():
            try:
                if self._serial_conn is None:
                    self._serial_conn = self._open_serial()
                    self._set_state(connected=True, last_error=None)

                raw_line = self._serial_conn.readline()
                if not raw_line:
                    continue

                line = raw_line.decode('utf-8', errors='replace').strip()
                if not line:
                    continue

                parsed_values = self._parse_sensor_line(line)
                if parsed_values is None:
                    with self._lock:
                        self._state['parse_errors'] += 1
                        self._state['last_line'] = line
                    continue

                self._motor_auto_ctrl.update_sensors(parsed_values)
                self._set_state(last_line=line, last_update_at=time.time())

            except Exception as error:
                self._set_state(connected=False, last_error=str(error))
                self._close_serial()
                self._stop_event.wait(1.0)

        self._set_state(connected=False)
        self._close_serial()

    def start(self):
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(target=self._run_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        self._close_serial()

    def get_state(self):
        with self._lock:
            return dict(self._state)


ultrasonic_serial_reader = ArduinoUltrasonicSerialReader(motor_automation)
ultrasonic_serial_reader.start()


def shutdown_motor_controller():
    try:
        motor_offset_calibration_printer.stop()
    except Exception as error:
        print(f'Failed to stop motor calibration printer cleanly: {error}')

    try:
        ultrasonic_serial_reader.stop()
    except Exception as error:
        print(f'Failed to stop ultrasonic serial reader cleanly: {error}')

    try:
        motor_automation.stop()
    except Exception as error:
        print(f'Failed to stop motor automation cleanly: {error}')

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


def initialize_slam_path_cache():
    """Load lobby_map.png and build obstacle + cost maps for SLAM-based A* pathfinding.

    Three stages:
      1. Binarize pixels  → obstacle_map[row][col]
      2. BFS multi-source distance transform → dist_map[row][col] (distance to nearest obstacle)
      3. Cost map → cost = 1.0 + WALL_WEIGHT / (dist + 1)  (far from walls = cheaper)
    """
    global _slam_path_cache
    if not HAS_PIL or not os.path.isfile(SLAM_OUTPUT_PNG):
        print("SLAM path cache: map PNG not available, skipping.")
        return

    try:
        img = Image.open(SLAM_OUTPUT_PNG).convert('RGBA')
        w, h = img.size
        pixels = img.load()

        # --- Stage 1: Binarize ---
        # lobby_map.png colours set by convert_slam_pgm_to_png():
        #   free     (248, 250, 252)  v >= 205
        #   obstacle (30,  41,  59)   v <= 50
        #   unknown  (148, 163, 184)  otherwise  → treat as obstacle (conservative)
        obstacle_map = [[True] * w for _ in range(h)]
        for row in range(h):
            for col in range(w):
                r, g, b, _ = pixels[col, row]
                if r >= 240 and g >= 240 and b >= 240:
                    obstacle_map[row][col] = False

        # --- Stage 2: BFS Distance Transform ---
        # Seed every obstacle pixel with dist=0, then expand outward.
        INF = float('inf')
        dist_map = [[INF] * w for _ in range(h)]
        queue = deque()
        for row in range(h):
            for col in range(w):
                if obstacle_map[row][col]:
                    dist_map[row][col] = 0
                    queue.append((row, col))

        _nbrs4 = [(-1, 0), (1, 0), (0, -1), (0, 1)]
        while queue:
            r, c = queue.popleft()
            nd = dist_map[r][c] + 1
            for dr, dc in _nbrs4:
                nr, nc = r + dr, c + dc
                if 0 <= nr < h and 0 <= nc < w and nd < dist_map[nr][nc]:
                    dist_map[nr][nc] = nd
                    queue.append((nr, nc))

        # --- Stage 3: Cost Map ---
        WALL_WEIGHT = 8.0
        cost_map = [[INF] * w for _ in range(h)]
        for row in range(h):
            for col in range(w):
                if not obstacle_map[row][col]:
                    cost_map[row][col] = 1.0 + WALL_WEIGHT / (dist_map[row][col] + 1.0)

        _slam_path_cache['obstacle_map'] = obstacle_map
        _slam_path_cache['cost_map'] = cost_map
        _slam_path_cache['width'] = w
        _slam_path_cache['height'] = h
        print(f"SLAM path cache initialized: {w}x{h} px")

    except Exception as e:
        print(f"SLAM path cache initialization failed: {e}")


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


def _slam_nearest_free(col, row):
    """BFS from (col, row) to find the nearest free pixel in the SLAM obstacle map."""
    obs = _slam_path_cache['obstacle_map']
    w = _slam_path_cache['width']
    h = _slam_path_cache['height']
    if not obs[row][col]:
        return col, row
    visited = {(col, row)}
    q = deque([(col, row)])
    while q:
        c, r = q.popleft()
        for dc, dr in ((-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)):
            nc, nr = c + dc, r + dr
            if 0 <= nc < w and 0 <= nr < h and (nc, nr) not in visited:
                if not obs[nr][nc]:
                    return nc, nr
                visited.add((nc, nr))
                q.append((nc, nr))
    return None


def _douglas_peucker(points, epsilon):
    """Recursively simplify a polyline using the Douglas-Peucker algorithm."""
    if len(points) < 3:
        return points
    start, end = points[0], points[-1]
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    seg_len = (dx*dx + dy*dy) ** 0.5

    max_dist = 0.0
    max_idx = 0
    for i in range(1, len(points) - 1):
        if seg_len == 0:
            d = ((points[i][0] - start[0])**2 + (points[i][1] - start[1])**2) ** 0.5
        else:
            # Perpendicular distance from point to line
            d = abs(dy * points[i][0] - dx * points[i][1] + end[0]*start[1] - end[1]*start[0]) / seg_len
        if d > max_dist:
            max_dist = d
            max_idx = i

    if max_dist > epsilon:
        left  = _douglas_peucker(points[:max_idx + 1], epsilon)
        right = _douglas_peucker(points[max_idx:], epsilon)
        return left[:-1] + right
    return [start, end]


def slam_find_path(start_col, start_row, goal_col, goal_row):
    """A* on the SLAM cost map (pixel grid, 8-directional).

    Returns a list of (col, row) pixel tuples, or None if no path found.
    The cost map penalises pixels close to walls so the path follows
    corridor centres.
    """
    if _slam_path_cache['cost_map'] is None:
        return None

    cost_map = _slam_path_cache['cost_map']
    w = _slam_path_cache['width']
    h = _slam_path_cache['height']
    INF = float('inf')

    # Snap start/goal to nearest free cell if they land on an obstacle
    sc = _slam_nearest_free(start_col, start_row)
    gc = _slam_nearest_free(goal_col, goal_row)
    if sc is None or gc is None:
        return None
    start_col, start_row = sc
    goal_col, goal_row   = gc

    start = (start_col, start_row)
    goal  = (goal_col,  goal_row)

    if start == goal:
        return [start]

    # 8-directional neighbours: (dcol, drow, base_move_cost)
    DIRS = [
        (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
        (-1, -1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (1, 1, 1.414),
    ]

    def heuristic(col, row):
        return ((col - goal_col)**2 + (row - goal_row)**2) ** 0.5

    g_score = {start: 0.0}
    came_from = {}
    open_heap = [(heuristic(*start), 0.0, start)]

    while open_heap:
        f, g, current = heapq.heappop(open_heap)
        if current == goal:
            # Reconstruct path
            path = []
            node = current
            while node in came_from:
                path.append(node)
                node = came_from[node]
            path.append(start)
            path.reverse()
            return path

        if g > g_score.get(current, INF):
            continue

        cc, cr = current
        for dc, dr, move_base in DIRS:
            nc, nr = cc + dc, cr + dr
            if not (0 <= nc < w and 0 <= nr < h):
                continue
            cell_cost = cost_map[nr][nc]
            if cell_cost == INF:
                continue
            # Anti-corner-cutting: diagonal move requires both axis-neighbours free
            if dc != 0 and dr != 0:
                if cost_map[cr][cc + dc] == INF or cost_map[cr + dr][cc] == INF:
                    continue
            tentative_g = g + move_base * cell_cost
            neighbour = (nc, nr)
            if tentative_g < g_score.get(neighbour, INF):
                g_score[neighbour] = tentative_g
                came_from[neighbour] = current
                heapq.heappush(open_heap, (tentative_g + heuristic(nc, nr), tentative_g, neighbour))

    return None


@app.route('/api/slam_preview_path')
def get_slam_preview_path():
    """Return a pre-computed navigation preview path using the SLAM map.

    Query params (all ROS-frame floats):
      sx, sy  – robot start position
      gx, gy  – goal position
    Response JSON: { "path": [[x_ros, y_ros], ...], "point_count": N }
    """
    if _slam_path_cache['cost_map'] is None:
        return jsonify({'error': 'SLAM map not ready'}), 503

    try:
        sx = float(request.args['sx'])
        sy = float(request.args['sy'])
        gx = float(request.args['gx'])
        gy = float(request.args['gy'])
    except (KeyError, ValueError):
        return jsonify({'error': 'Missing or invalid sx/sy/gx/gy params'}), 400

    map_info = load_slam_map_info()
    if map_info is None:
        return jsonify({'error': 'Map metadata unavailable'}), 503

    res  = map_info['resolution']
    ox   = map_info['origin_x']
    oy   = map_info['origin_y']
    h_px = map_info['height_px']

    def ros_to_px(xr, yr):
        col = int(round((xr - ox) / res))
        row = int(round((h_px - 1) - (yr - oy) / res))
        return col, row

    def px_to_ros(col, row):
        xr = col * res + ox
        yr = (h_px - 1 - row) * res + oy
        return xr, yr

    start_col, start_row = ros_to_px(sx, sy)
    goal_col,  goal_row  = ros_to_px(gx, gy)

    pixel_path = slam_find_path(start_col, start_row, goal_col, goal_row)
    if not pixel_path:
        return jsonify({'error': 'No path found'}), 404

    # Simplify with Douglas-Peucker (epsilon = 3 pixels)
    simplified = _douglas_peucker(pixel_path, epsilon=3.0)

    # Convert back to ROS coordinates
    ros_path = [list(px_to_ros(c, r)) for c, r in simplified]

    return jsonify({'path': ros_path, 'point_count': len(ros_path)})


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


@app.route('/api/motor/sensors', methods=['POST'])
def update_motor_sensors():
    payload = request.get_json(silent=True) or {}
    sensor_values = payload.get('sensor_cm')

    # Accept either an ordered list in sensor_cm or indexed keys like sensor_0..sensor_3 / sensor_1..sensor_4.
    if sensor_values is None:
        sensor_values = {}
        for one_based in range(1, MotorAutomationController.SENSOR_COUNT + 1):
            key = f'sensor_{one_based}'
            if key in payload:
                sensor_values[one_based - 1] = payload.get(key)

        for zero_based in range(MotorAutomationController.SENSOR_COUNT):
            key = f'sensor_{zero_based}'
            if key in payload:
                sensor_values[zero_based] = payload.get(key)

    try:
        motor_automation.update_sensors(sensor_values)
        return jsonify({'status': 'ok', 'state': motor_automation.get_state()})
    except ValueError as error:
        return jsonify({'error': str(error)}), 400
    except Exception as error:
        return jsonify({'error': f'Failed to update sensor values: {error}'}), 500


@app.route('/api/motor/sensors')
def get_motor_sensors_state():
    return jsonify(motor_automation.get_state())


@app.route('/api/motor/ultrasonic')
def get_ultrasonic_reader_state():
    return jsonify(ultrasonic_serial_reader.get_state())


if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5000)
    args = parser.parse_args()
    convert_slam_pgm_to_png()
    print("Initializing pathfinding grid cache...")
    initialize_grid_cache(grid_resolution=1.0)
    print("Initializing SLAM path cache...")
    initialize_slam_path_cache()
    print(f"Starting server on port {args.port}...")
    app.run(debug=False, host='0.0.0.0', port=args.port)
