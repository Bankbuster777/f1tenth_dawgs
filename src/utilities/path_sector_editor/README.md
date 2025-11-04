# Path Sector Editor

Interactive matplotlib-based tool for sector-wise path tuning on F1TENTH racing tracks.

## Features

- **Interactive Sector Definition**: Click-based sector creation on track visualization
- **Real-time Path Modification**:
  - Velocity scaling (multiply by factor)
  - Velocity offset (add constant)
  - Lateral offset (d in Frenet frame)
- **matplotlib GUI**: Full interactive visualization with controls
- **CSV Import/Export**: Load and save modified paths in standard format (x, y, v, kappa)
- **ROS2 Integration**: Publishes modified path to `/global_centerline` topic

## Installation

```bash
cd ~/f1tenth_dawgs
colcon build --packages-select path_sector_editor
source install/setup.bash
```

## Usage

### Launch with default configuration:

```bash
ros2 launch path_sector_editor path_sector_editor.launch.py
```

### Launch with custom CSV file:

```bash
ros2 launch path_sector_editor path_sector_editor.launch.py \
    csv_file:=/path/to/your/waypoints.csv
```

### Run node directly:

```bash
ros2 run path_sector_editor path_sector_editor_node \
    --ros-args \
    -p csv_file_path:=/path/to/your/waypoints.csv
```

## GUI Instructions

### Defining Sectors:

1. Click **"New Sector"** button
2. Click **start point** on the track
3. Click **end point** on the track
4. Enter **sector name** in the text box
5. Press **Enter** to create the sector

### Tuning Sectors:

1. **Select a sector** by clicking on it in the map
2. Adjust the sliders:
   - **Vel Scale**: Multiply velocity (0.1 - 2.0)
   - **Vel Offset**: Add to velocity (-2.0 - 2.0 m/s)
   - **D Offset**: Lateral shift (-1.0 - 1.0 m, + = right)
3. Click **"Apply Changes"** to apply modifications

### Other Controls:

- **Delete Sector**: Remove selected sector
- **Reset All**: Revert all modifications
- **Save CSV**: Export modified path to CSV file
- **Clear Sectors**: Remove all sectors

## Configuration

Edit `config/path_sector_editor.yaml`:

```yaml
path_sector_editor_node:
  ros__parameters:
    csv_file_path: "/path/to/your/waypoints.csv"
    global_path_topic: "/global_centerline"
    frame_id: "map"
    publish_rate: 1.0
```

## CSV Format

Input CSV must follow the format:
```
x,y,v,kappa
-5.558,0.345,5.360,0.509
-5.476,0.288,5.432,0.489
...
```

## Architecture

### Key Components:

1. **SectorManager**: CSV loading/saving, sector management, path modification
2. **PathSectorEditorNode**: ROS2 node, matplotlib GUI, event handling

## License

MIT License
