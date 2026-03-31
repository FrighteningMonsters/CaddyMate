# CaddyMate GUI

A touch-friendly GUI for a 7-inch display designed for elderly users. Built on the original HTML/Flask shopping UI, with TurtleBot3 + ROS2 Jazzy + Nav2 integration for supermarket navigation.

## Setup

1. Install Python dependencies:
```bash
pip install -r requirements.txt
```

2. Run once to create/regenerate the database with ROS coordinates for TurtleBot navigation:
```bash
python data/Database_Creator.py
```

## Hardware Wiring (Pi → Arduino Uno via I2C)

The Raspberry Pi controls the Dynamixel motor indirectly via the Arduino Uno over I2C. The Uno runs the DynamixelShield and receives velocity commands from the Pi.

### Pin Connections

| Raspberry Pi        | Arduino Uno | Purpose   |
|---------------------|-------------|-----------|
| Pin 3 (GPIO2 / SDA) | A4          | I2C Data  |
| Pin 5 (GPIO3 / SCL) | A5          | I2C Clock |
| Pin 6 (GND)         | GND         | Ground    |

### DynamixelShield Switch

The DynamixelShield has a slide switch that must be set correctly:

- **UPLOAD** — connects Serial to USB; required when flashing a new sketch
- **DXL** — connects Serial to the Dynamixel motor; required during normal operation

Always switch back to **DXL** after uploading, or the motor will not respond.

### I2C Commands

The Pi sends raw UTF-8 bytes to I2C address `0x08`:

| Command | Effect              |
|---------|---------------------|
| `UP`    | Spin forward (200)  |
| `DOWN`  | Spin reverse (-200) |
| `STOP`  | Stop motor (0)      |

## Running the Application

1. Start the Flask server:
```bash
python server.py
```

2. Open your browser and navigate to:
```
http://localhost:5000
```

The server will:
- Serve the HTML pages
- Provide API endpoints for categories and items from the SQLite database
- Convert SLAM map (PGM) to PNG for the map page
- Serve `map_info` and `ros_config` for TurtleBot integration

## Pages

- **home.html** - Main menu with Browse Categories and Search Items buttons
- **motor.html** - Dedicated hold-to-run up/down motor controls
- **categories.html** - Displays all product categories from the database
- **search.html** - Search for items and navigate directly to the map
- **items.html** - Displays items for a selected category; items with ROS coordinates link to the map with target pre-selected
- **map.html** - SLAM map view with robot position, Nav2 path, and Navigate button (when connected to TurtleBot)

## API Endpoints

- `GET /api/categories` - Get all categories
- `GET /api/items` - Get all items for search
- `GET /api/items/<category_id>` - Get items for a specific category
- `GET /api/map_info` - Get SLAM map metadata (resolution, origin, dimensions)
- `GET /api/ros_config` - Get rosbridge host and port for WebSocket connection
- `POST /api/path` - Compute path between two points (abstract layout grid; used internally)
- `GET /api/slam_preview_path?sx=&sy=&gx=&gy=` - Compute A* preview path on the SLAM pixel map; returns ROS-frame waypoints

## Database

The application uses SQLite database located at `data/caddymate_store.db` with the following structure:

### Tables:
- **categories** (id, name)
- **items** (id, name, category_id, aisle, aisle_position, x_ros, y_ros, yaw_ros)

The `x_ros`, `y_ros`, `yaw_ros` columns store ROS map-frame coordinates for TurtleBot navigation. Items with these values show a Navigate button on the map page.

### Items with navigation targets (current)

Only the following items have ROS coordinates and show a target on the map page.
Coordinates are calibrated for the **lab SLAM map** (`lab_final.pgm`, origin `[-3.45, -4.01]`, resolution `0.05 m/px`):

| Item       | x (m) | y (m) | yaw (rad) |
|------------|------:|------:|----------:|
| Apples     | 4.4   | 4.0   | 0.0       |
| Whole milk | 3.5   | 11.0  | 0.0       |
| Bananas    | 2.6   | -0.6  | 0.0       |

To add or update coordinates, edit `item_ros_coords` in `data/Database_Creator.py` and re-run the script.

## TurtleBot Integration

When `lab_final.pgm` and `lab_final.yaml` (SLAM map) are in the project root, the map page loads the SLAM map and connects to ROS via rosbridge.

### Prerequisites

1. **rosbridge** running on the Dice Machine:
```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090
```

2. **ros_config.json** in project root with `rosbridge_host` and `rosbridge_port` (`10.0.0.1`, `9090`)

### Map Page Flow

1. `GET /api/map_info` → SLAM mode enabled; `lobby_map.png` (converted from `lab_final.pgm`) displayed as background
2. WebSocket connection to rosbridge → `/amcl_pose` (robot position), `/plan` (Nav2 path), `/navigate_to_pose/_action/status` (arrival/failure)
3. Green marker = target item (items with `x_ros`, `y_ros` only)
4. **Preview path** (orange dashed line) — drawn automatically as soon as ROS connects and robot position is known, using server-side A* on the SLAM pixel map; no Navigate press required
   - Path follows corridor centres (wall-penalised cost map via BFS distance transform)
   - Trims in sync with every `/amcl_pose` update; re-plans if robot deviates > 20 px from path
   - Hidden when Navigate is pressed; restored from current position on failure/cancel
5. **Navigate** button → enables motor, publishes `/goal_pose`, monitors status
6. On arrival (status SUCCEEDED) or failure (CANCELED/ABORTED) → overlay shown; motor released
7. **Stop** button cancels navigation and releases motor
8. **Manual drive arrival** — if Navigate is not pressed, robot position is checked against the target on every `/amcl_pose` update; entering within **0.7 m** triggers the same "Item Found" overlay and auto-exit (detection armed 2 s after first position fix to prevent instant trigger on page load)

### Network Topology (Demo Setup)

| Device        | Role                          |
|---------------|-------------------------------|
| Raspberry Pi  | Flask server, UI hosting      |
| Dice Machine  | ROS2, Nav2, rosbridge :9090   |
| TurtleBot     | LiDAR, motors, ROS2 DDS      |

Browser → `ws://<Dice Machine IP>:9090` (rosbridge) → ROS2 DDS → TurtleBot

### UI State Indicators

- **Disconnected** – No rosbridge connection; Navigate disabled
- **Connection lost** – Banner when WebSocket drops
- **Position unavailable** – No `/amcl_pose` for 5+ seconds
- **Robot map not ready** – `map_info` fetch failed; placeholder shown

