# PGM to SDF Converter

A ROS2 package for converting 2D occupancy grid maps to Gazebo Classic simulation worlds. Simply point it at your map file, and it generates a ready-to-use SDF world with walls from it.

**Works with both clean CAD maps and noisy SLAM-generated maps** - no configuration needed.

## Table of Contents

- [Quick Start](#quick-start)
- [What It Does](#what-it-does)
- [Installation](#installation)
- [Usage Examples](#usage-examples)
- [Configuration Options](#configuration-options)
- [Troubleshooting](#troubleshooting)
- [Technical Details](#technical-details)

## Quick Start

```bash
# Install dependencies if you haven't already
pip install pyyaml numpy scipy

# Clone this repository into your ROS2 workspace
cd ~/ros2_ws/src
git clone https://github.com/robotics-upo/pgm_to_sdf_converter.git

# Build the package
cd ~/ros2_ws
colcon build --packages-select pgm_to_sdf_converter
source install/setup.bash

# Convert your map
ros2 run pgm_to_sdf_converter pgm_to_sdf map.yaml

# View in Gazebo
gazebo map.sdf
```

## What It Does

The converter turns your 2D occupancy map into a 3D simulation world by:

1. **Analyzing** your map to determine its quality (clean or noisy)
2. **Cleaning** the map if needed (removes noise, fills gaps in walls)
3. **Extracting** wall segments from the cleaned map
4. **Optimizing** by merging adjacent walls to reduce object count
5. **Generating** a complete Gazebo world file

**Key benefits:**

- Zero configuration required
- Handles both clean and noisy maps automatically
- Creates optimized worlds with fewer objects
- Preserves all map details

## Installation

### Requirements

- ROS2 (tested on Humble)
- Python 3.8 or newer
- Python packages: PyYAML, NumPy, SciPy

### Build Steps

```bash
# Navigate to your ROS2 workspace
cd ~/ros2_ws/src

# Place this package here
git clone https://github.com/robotics-upo/pgm_to_sdf_converter.git

# Install Python dependencies
pip install pyyaml numpy scipy

# Build
cd ~/ros2_ws
colcon build --packages-select pgm_to_sdf_converter
source install/setup.bash
```

### Test with Example Maps

```bash
# Clean map example
ros2 run pgm_to_sdf_converter pgm_to_sdf examples/bookstore.yaml
gazebo examples/bookstore.sdf

# SLAM map example
ros2 run pgm_to_sdf_converter pgm_to_sdf examples/submap_0.yaml
gazebo examples/submap_0.sdf
```

## Usage Examples

### Basic Usage

```bash
# Convert map.yaml to map.sdf
ros2 run pgm_to_sdf_converter pgm_to_sdf map.yaml # or map (without extension)

# Specify output file
ros2 run pgm_to_sdf_converter pgm_to_sdf map -o my_world.sdf
```

### Common Customizations

```bash
# Change wall height (default: 2.5 meters)
ros2 run pgm_to_sdf_converter pgm_to_sdf map --wall-height 3.0

# Filter short wall segments (useful for removing clutter)
ros2 run pgm_to_sdf_converter pgm_to_sdf map --min-wall-length 1.0

# Save the cleaned map for inspection
ros2 run pgm_to_sdf_converter pgm_to_sdf map --keep-cleaned-pgm

# Custom world name
ros2 run pgm_to_sdf_converter pgm_to_sdf map --world-name my_warehouse
```

### Testing with example maps

```bash
# Bookstore 
ros2 run pgm_to_sdf_converter pgm_to_sdf ~/ros2_ws/src/pgm_to_sdf_converter/examples/bookstore.yaml
gazebo ~/ros2_ws/src/pgm_to_sdf_converter/examples/bookstore.sdf   

# Small house 
ros2 run pgm_to_sdf_converter pgm_to_sdf ~/ros2_ws/src/pgm_to_sdf_converter/examples/small_house.yaml
gazebo ~/ros2_ws/src/pgm_to_sdf_converter/examples/small_house.sdf
```

### Using the Python API

```python
from pgm_to_sdf_converter.converter_node import PGMToSDFConverter

converter = PGMToSDFConverter()
sdf_content = converter.convert(
    yaml_path='map.yaml',
    output_path='output.sdf',
    wall_height=2.5,
    min_wall_size=1,
    min_wall_length=0.5,
    enable_merging=True,
    merge_tolerance=0.05,
    keep_cleaned_pgm=False,
    world_name='my_world'
)
```

## Configuration Options

| Option | Default | Description |
|--------|---------|-------------|
| `-o, --output` | [input].sdf | Output file path |
| `--wall-height` | 2.5 | Wall height in meters |
| `--min-wall-length` | 0.5 | Filter walls shorter than this (meters) |
| `--min-wall-size` | 1 | Filter walls smaller than this (pixels) |
| `--merge-tolerance` | 0.05 | Maximum gap between walls to merge (meters) |
| `--keep-cleaned-pgm` | False | Save the cleaned map file |
| `--no-merge` | False | Disable wall merging optimization |
| `--world-name` | occupancy_map_world | World name in the SDF file |

**Note:** You typically don't need to change these defaults.

## Input Format

Your map needs two files:

1. **PGM file** - The image (darker pixels = walls)
2. **YAML file** - Map settings

Example YAML file:

```yaml
image: map.pgm
resolution: 0.05          # Meters per pixel
origin: [0.0, 0.0, 0.0]  # Map position [x, y, rotation]
negate: 0                 # Auto-detected if incorrect
occupied_thresh: 0.65
free_thresh: 0.196
```

The converter automatically detects if the wall colors are inverted.

## Troubleshooting

### Too many small wall pieces

**Increase the minimum wall length:**

```bash
ros2 run pgm_to_sdf_converter pgm_to_sdf map --min-wall-length 1.0
```

This filters out wall segments shorter than 1 meter.

### Want to inspect the cleaned map

**Save the cleaned version:**

```bash
ros2 run pgm_to_sdf_converter pgm_to_sdf map --keep-cleaned-pgm
```

The cleaned PGM file will be saved next to your original map.

### Command not found error

**Make sure you sourced the workspace:**

```bash
source ~/ros2_ws/install/setup.bash
```

## Technical Details

### How Map Cleaning Works

The converter automatically analyzes your map and applies appropriate cleaning:

**Clean Maps** (CAD-generated)

- Smooths rough edges on walls
- Fills small gaps in fragmented walls
- Keeps all features intact (furniture, small objects remain)

**Noisy Maps** (SLAM-generated)

- Applies stronger noise removal
- Fills larger gaps between wall sections
- Removes small noise artifacts
- Uses multi-pass filtering for heavy noise

**Detection Method:**

- Analyzes how many small disconnected pieces exist
- Measures how fragmented the walls are
- Detects rough/textured wall boundaries
- Classifies as CLEAN, MODERATE, or HEAVY noise

### How Wall Extraction Works

**Two-Pass Scanning:**

1. Scans each row left-to-right to find horizontal wall segments
2. Scans each column top-to-bottom to find vertical wall segments
3. Wall thickness comes from your map's resolution

**Why this approach:**

- More efficient than pixel-by-pixel object creation
- Handles both clean and noisy maps
- Optimized for axis-aligned walls but works with any geometry

### How Optimization Works

**Segment Merging:**

- Combines adjacent wall pieces that line up perfectly
- Only merges walls on consecutive rows/columns with identical length
- Reduces object count by 60-70% typically
- Improves Gazebo simulation performance

### Limitations

- **Rectangular segment representation** - The converter represents all walls as horizontal and vertical rectangles. Angled or curved walls are approximated by multiple small segments following the pixel pattern. This works fine but creates more objects for non-axis-aligned walls.

- **Very fine details** - Maps with extremely high resolution (< 0.02m per pixel) may produce many segments even after optimization, though the merging reduction helps significantly.

- **Complex geometries** - Rooms with many small alcoves, furniture arrangements, or irregular shapes will naturally create more wall segments to accurately represent the space.

The system handles any map geometry - it extracts whatever occupied pixels exist. Most standard robotics maps (both clean CAD and noisy SLAM) work efficiently out of the box.

## License

MIT License - Copyright (c) 2025 Service Robotics Lab

See [LICENSE](LICENSE) file for full details.
