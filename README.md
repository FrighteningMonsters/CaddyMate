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
- `POST /api/path` - Compute path between two points (used by server-side pathfinding; map page uses Nav2 `/plan` when connected)

## Database

The application uses SQLite database located at `data/caddymate_store.db` with the following structure:

### Tables:
- **categories** (id, name)
- **items** (id, name, category_id, aisle, aisle_position, x_ros, y_ros, yaw_ros)

The `x_ros`, `y_ros`, `yaw_ros` columns store ROS map-frame coordinates for TurtleBot navigation. Items with these values show a Navigate button on the map page.

### Items with navigation targets (current)

Only the following items have ROS coordinates and show a target on the map page:

| Item           | x (m) | y (m) | yaw (rad) |
|----------------|------:|------:|----------:|
| Apples         | -4.0  | 2.0   | 0.0       |
| White bread    | -1.5  | 4.5   | 0.0       |
| Whole milk     | 1.0   | 5.5   | 1.57      |
| Chicken breast| 3.5   | 4.0   | 3.14      |
| Frozen pizza   | 5.0   | 2.5   | 0.0       |
| White rice     | 6.5   | 0.5   | 1.57      |
| Crisps         | 5.0   | -1.5  | 3.14      |
| Still water    | 3.0   | -3.0  | 0.0       |
| Red wine       | 1.0   | -2.5  | 0.0       |
| Ale            | 1.5   | -2.0  | 0.0       |
| Toilet paper   | -1.5  | -2.0  | 1.57      |
| Paracetamol    | -3.5  | -1.0  | 0.0       |
| Shampoo        | -5.0  | 1.0   | 1.57      |
| Nappies        | -3.0  | 4.0   | 0.0       |
| Dog food       | 7.5   | 3.5   | 3.14      |
| Hummus         | 0.0   | 0.5   | 0.0       |

To add or update coordinates, edit `item_ros_coords` in `data/Database_Creator.py` and re-run the script.

## TurtleBot Integration

When `lobby_final.pgm` and `lobby_final.yaml` (SLAM map) are in the project root, the map page loads the SLAM map and connects to ROS via rosbridge.

### Prerequisites

1. **rosbridge** running on the Dice Machine:
```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml port:=9090
```

2. **ros_config.json** in project root with `rosbridge_host` and `rosbridge_port` (`10.0.0.1`, `9090`)

### Map Page Flow

1. `GET /api/map_info` → SLAM mode enabled; `lobby_map.png` displayed as background
2. WebSocket connection to rosbridge → `/amcl_pose` (robot position), `/plan` (Nav2 path), `/navigate_to_pose/_action/status` (arrival/failure)
3. Green marker = target item (items with `x_ros`, `y_ros` only)
4. **Navigate** button → enables motor, publishes `/goal_pose`, monitors status
5. On arrival (status SUCCEEDED) or failure (CANCELED/ABORTED) → overlay shown; motor released
6. **Stop** button cancels navigation and releases motor

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

