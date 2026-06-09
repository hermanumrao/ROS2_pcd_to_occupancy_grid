# ROS2_pcd_to_occupancy_grid
This package is used to derive an occupancy grid from any given PCD/PCL. 
It converts a ros2 `sensor_msgs::msg::PointCloud2` which is publishing data in form of PCL2 to a 2d map `nav_msgs::msg::OccupancyGrid`.

# ROS2 PCD to Occupancy Grid

A ROS 2 package that converts a 3D point cloud (`sensor_msgs/msg/PointCloud2`) into a 2D occupancy grid (`nav_msgs/msg/OccupancyGrid`). It ships with two tools:

1. **`mapper`** — a C++ node that subscribes to a live or replayed point cloud topic, slices it by height, and publishes a 2D occupancy grid.
2. **`map_gui`** — a Python/PyQt5 GUI node that lets you interactively translate and rotate the saved point cloud before saving both the aligned 3D cloud (`.pcd`) and the 2D map image (`.pgm`).

---

## Table of Contents

- [Overview](#overview)
- [Package Structure](#package-structure)
- [Dependencies](#dependencies)
- [Installation](#installation)
- [How It Works](#how-it-works)
  - [Part 1 — C++ Mapper Node](#part-1--c-mapper-node)
  - [Part 2 — Python Map GUI Node](#part-2--python-map-gui-node)
- [ROS Topics](#ros-topics)
- [Configuration Parameters](#configuration-parameters)
  - [Mapper Node (`params.yaml`)](#mapper-node-paramsyaml)
- [Launch](#launch)
  - [Launching the Mapper](#launching-the-mapper)
  - [Running the GUI](#running-the-gui)
- [Output Files](#output-files)
- [Typical Workflow](#typical-workflow)
- [Troubleshooting](#troubleshooting)

---

## Overview

The package is designed for the common robotics workflow where you have already built a 3D map with a SLAM system (e.g. RTAB-Map, LIO-SAM, FAST-LIO) and need a clean flat 2D occupancy grid for Nav2 path planning or for offline map editing.

```
[3D SLAM Map]
      |
      | sensor_msgs/PointCloud2  →  /saved_map
      ↓
  [mapper node]  ──────────────────────────────→  /map2d  (nav_msgs/OccupancyGrid)
      |
      |  (optional)
      ↓
  [map_gui node]
      ├─ Interactive 6-DOF transform (tx, ty, tz, roll, pitch, yaw)
      ├─ Preview via /aligned_map
      └─ Save → ~/maps/aligned.pcd  +  ~/maps/aligned2d.pgm
```

---

## Package Structure

```
ROS2_pcd_to_occupancy_grid/
└── map_3d_to_2d/
    ├── CMakeLists.txt
    ├── package.xml
    ├── config/
    │   └── params.yaml          # Tunable parameters for the mapper node
    ├── include/
    │   └── map_3d_to_2d/
    │       └── mapper.hpp
    ├── launch/
    │   └── mapper.launch.py     # Launch file for the C++ mapper node
    ├── scripts/
    │   └── map_gui.py           # Python PyQt5 GUI for post-processing
    └── src/
        └── mapper.cpp           # C++ mapper node implementation
```

---

## Dependencies

### ROS 2 Packages
- `rclcpp`
- `sensor_msgs`
- `nav_msgs`
- `pcl_conversions`
- `pcl_ros`
- `tf2`
- `tf2_ros`

### System Libraries
- [PCL (Point Cloud Library)](https://pointclouds.org/)
- OpenCV (`cv2`) — used by the GUI for saving `.pgm` files
- PyQt5 — used by the GUI

### Python (GUI only)
- `rclpy`
- `sensor_msgs_py`
- `numpy`
- `opencv-python`
- `PyQt5`

Install Python dependencies if not already present:

```bash
pip install numpy opencv-python PyQt5
# or via apt:
sudo apt install python3-pyqt5 python3-opencv python3-numpy
```

---

## Installation

```bash
# Navigate to your ROS 2 workspace source directory
cd ~/ros2_ws/src

# Clone the repository
git clone https://github.com/hermanumrao/ROS2_pcd_to_occupancy_grid.git

# Install ROS dependencies
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build
colcon build --packages-select map_3d_to_2d

# Source the workspace
source install/setup.bash
```

---

## How It Works

### Part 1 — C++ Mapper Node

**Source:** `src/mapper.cpp`

The `mapper` executable is a ROS 2 node (node name: `map_3d_to_2d`) that:

1. **Subscribes** to `/saved_map` (`sensor_msgs/msg/PointCloud2`).
2. **Filters** the incoming cloud by height using a Z-axis passthrough filter, keeping only points between `z_min` and `z_max`. This removes the floor, ceiling, and out-of-range noise.
3. **Initialises the map bounds** on the first message. It computes the X/Y extents of the filtered cloud and adds a configurable `padding` border. Map width and height (in cells) are derived automatically from these extents and the `resolution`.
4. **Projects** surviving 3D points onto the XY plane. Each point is binned into a 2D grid cell. A running count of how many points land in each cell is maintained.
5. **Thresholds** — any cell whose point count reaches or exceeds `occupancy_threshold` is marked as **occupied** (value `100`). All other cells are left as **unknown** (value `-1`). There is no explicit "free" marking; the node does not perform ray-casting.
6. **Publishes** the resulting `nav_msgs/msg/OccupancyGrid` on `/map2d` every time a new point cloud arrives.

> **Note:** Map bounds are locked after the first received message. If the cloud changes significantly between messages, restart the node.

---

### Part 2 — Python Map GUI Node

**Source:** `scripts/map_gui.py`

The `map_gui` node is a PyQt5 GUI application that runs alongside the mapper. It provides interactive 6-DOF (degrees of freedom) control to realign the source point cloud before the final map is saved. This is useful when the SLAM map has a slight tilt, orientation mismatch, or needs to be repositioned.

The GUI window exposes six spin-box controls:

| Control | Axis | Range | Step |
|---------|------|-------|------|
| **X** | Translation along X | −100 m to +100 m | 0.1 m |
| **Y** | Translation along Y | −100 m to +100 m | 0.1 m |
| **Z** | Translation along Z | −100 m to +100 m | 0.1 m |
| **Roll** | Rotation around X | −π to +π rad | 0.01 rad |
| **Pitch** | Rotation around Y | −π to +π rad | 0.01 rad |
| **Yaw** | Rotation around Z | −π to +π rad | 0.01 rad |

Rotation order is **Z × Y × X** (yaw applied first, then pitch, then roll).

**Buttons:**

- **Apply Transform** — applies the current rotation + translation to the stored original cloud and re-publishes on `/aligned_map`. The mapper node will pick this up and regenerate `/map2d`.
- **Reset** — resets all sliders to zero and re-publishes the original unmodified cloud.
- **Save Maps** — writes two files to `~/maps/`:
  - `aligned.pcd` — the transformed 3D point cloud in ASCII PCD format.
  - `aligned2d.pgm` — the latest `/map2d` occupancy grid rendered as a grayscale PGM image (occupied = `0` / black, free = `254` / white, unknown = `205` / grey).

> **Important:** The GUI stores the original cloud on the **first** `/saved_map` message it receives and always transforms from that baseline. Re-publishing a new cloud from a bag will **not** update the stored original; restart the node to load a fresh cloud.

---

## ROS Topics

### Subscribed

| Topic | Type | Description |
|-------|------|-------------|
| `/saved_map` | `sensor_msgs/msg/PointCloud2` | Input 3D point cloud (from SLAM map or bag replay) |
| `/map2d` | `nav_msgs/msg/OccupancyGrid` | 2D map consumed by the GUI for saving (published by the mapper) |

### Published

| Topic | Type | Publisher | Description |
|-------|------|-----------|-------------|
| `/map2d` | `nav_msgs/msg/OccupancyGrid` | `mapper` node | The generated 2D occupancy grid |
| `/aligned_map` | `sensor_msgs/msg/PointCloud2` | `map_gui` node | Transformed cloud preview |

---

## Configuration Parameters

### Mapper Node (`params.yaml`)

Located at `map_3d_to_2d/config/params.yaml`. Loaded automatically by the launch file.

```yaml
map_3d_to_2d:
  ros__parameters:
    resolution: 0.05
    width: 400
    height: 400
    z_min: -0.5
    z_max: 1.5
    occupancy_threshold: 2
    frame_id: "map"
```

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `resolution` | `double` | `0.05` | Size of each map cell in metres. Smaller values produce finer maps but increase memory usage. Typical range: `0.02` – `0.10` m. |
| `z_min` | `double` | `-0.5` | Lower bound of the Z-axis passthrough filter (metres). Points below this height are discarded. Set slightly above the floor plane. |
| `z_max` | `double` | `1.5` | Upper bound of the Z-axis passthrough filter (metres). Points above this height are discarded. Set below the ceiling or LiDAR blind spot. |
| `occupancy_threshold` | `int` | `2` | Minimum number of 3D points that must project into a cell for it to be marked **occupied** (`100`). Increasing this value reduces noise at the cost of potentially missing thin obstacles. |
| `frame_id` | `string` | `"map"` | The `frame_id` written into the published `OccupancyGrid` header. Must match the frame used by your navigation stack (usually `"map"`). |
| `padding` | `double` | `1.0` | Extra border added around the auto-computed map extents, in metres. Prevents points near the edge from being clipped. *(Not exposed in `params.yaml` by default — add it manually if needed.)* |

> **Note on `width` and `height`:** The values listed in `params.yaml` are not actually used by the mapper node. Map dimensions are computed automatically from the point cloud extents and `resolution` on the first message. They are left in the file as reference/documentation only.

To override any parameter at launch time without editing the YAML:

```bash
ros2 launch map_3d_to_2d mapper.launch.py \
  --ros-args -p resolution:=0.03 -p z_min:=-0.3 -p z_max:=2.0
```

---

## Launch

### Launching the Mapper

The provided launch file starts the C++ mapper node and loads `config/params.yaml` automatically.

```bash
ros2 launch map_3d_to_2d mapper.launch.py
```

If you are replaying a ROS 2 bag that contains the point cloud on `/saved_map`, play it in a separate terminal:

```bash
ros2 bag play <your_bag_directory>
```

To visualise the output occupancy grid in RViz:

```bash
rviz2
# Add a Map display → topic: /map2d
# Add a PointCloud2 display → topic: /saved_map
```

### Running the GUI

The GUI is a standalone Python script that must be run **after** the mapper is already running (it subscribes to both `/saved_map` and `/map2d`):

```bash
ros2 run map_3d_to_2d map_gui.py
```

> The GUI requires a display (X11 / Wayland). It will not run headlessly.

---

## Output Files

Both output files are written to `~/maps/` (created automatically if it does not exist).

| File | Format | Description |
|------|--------|-------------|
| `~/maps/aligned.pcd` | ASCII PCD v0.7 | The transformed 3D point cloud with fields `x y z`. Can be re-loaded into any SLAM or visualisation tool that accepts PCD files. |
| `~/maps/aligned2d.pgm` | PGM (8-bit grayscale) | The 2D occupancy grid rendered as an image. Pixel values: `0` = occupied (black), `254` = free (white), `205` = unknown (grey). Compatible with Nav2's `map_server`. |

To use `aligned2d.pgm` with Nav2, create a matching YAML file:

```yaml
# aligned2d.yaml
image: aligned2d.pgm
resolution: 0.05          # must match the resolution param used during conversion
origin: [0.0, 0.0, 0.0]  # adjust to match map origin if needed
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

Then serve it with:

```bash
ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=~/maps/aligned2d.yaml
```

---

## Typical Workflow

1. **Run your 3D SLAM pipeline** (e.g. FAST-LIO, LIO-SAM) and save the resulting map as a `PointCloud2` bag or publish it on `/saved_map`.

2. **Launch the mapper node:**
   ```bash
   ros2 launch map_3d_to_2d mapper.launch.py
   ```

3. **Play your bag** (if using a pre-recorded map):
   ```bash
   ros2 bag play <bag_directory>
   ```

4. **Verify the 2D map** looks correct in RViz by subscribing to `/map2d`.

5. **(Optional) Open the GUI** if the map needs alignment:
   ```bash
   ros2 run map_3d_to_2d map_gui.py
   ```
   - Adjust **Roll / Pitch** to level the map if the LiDAR was mounted at an angle.
   - Adjust **Yaw** to rotate the map to the desired orientation.
   - Adjust **X / Y / Z** to reposition the origin.
   - Click **Apply Transform** to preview the result on `/aligned_map` and `/map2d`.
   - Click **Save Maps** when satisfied.

6. **Use the saved files** with Nav2 or any other 2D navigation stack.

---

## Troubleshooting

**Filtered cloud is empty**
The warning `Filtered cloud is empty` means no points survived the Z passthrough. Widen the `z_min` / `z_max` range in `params.yaml` to match the actual Z extent of your cloud.

**Map looks very sparse / too many unknown cells**
Lower `occupancy_threshold` to `1`. A value of `1` marks any cell hit by even a single point as occupied.

**Map is too noisy (many isolated occupied cells)**
Raise `occupancy_threshold` to `3` or higher to require more points before a cell is marked occupied.

**Map resolution is too low / coarse**
Decrease `resolution` (e.g. `0.02`). Note that halving the resolution quadruples the number of cells.

**GUI does not show any cloud**
Ensure the mapper is running and a message has been published on `/saved_map` before starting the GUI. The GUI captures the original cloud only from the first message it receives.

**`aligned2d.pgm` appears flipped or rotated**
Use the **Yaw** control in the GUI to rotate the cloud before saving. If the map appears upside-down, try setting **Roll** to `±π`.
