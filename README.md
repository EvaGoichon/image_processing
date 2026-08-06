# Image Processing Package

ROS2 package for IR image processing and camera degradation simulation for RGB-IR-Depth (RID) SLAM systems.

## Features

- **Active IR Processor**: IR enhancement via percentile-based dynamic-range clipping and edge-preserving bilateral filtering, with mode-dependent tone mapping (`WFOV`: linear percentile clip; `NFOV`: logarithmic compression). Supports an optional fixed-range mode where the normalization range is computed once and held fixed for the sequence.
- **Passive IR Processor**: Basic IR processing with gamma correction.
- **Occlusion / Degradation Node**: Simulates sensor degradation (dimming, spatial occlusion patterns, noise, blur) on a selected image stream, for robustness evaluation.

This package is developed as a general-purpose toolkit for multimodal RGB-IR-Depth sensing and is used across several related works, including the `stella_vslam_ros_RID` SLAM pipeline. Not every feature is necessarily exercised by every downstream project.

> **Note:** When used with `stella_vslam_ros_RID`, these nodes are not run standalone — they are launched automatically as part of its launch files (e.g. `RID_CD-MaJ_WFOV.py`), with parameters (`mode_ir`, `use_fixed_range`, `dimming`) set as launch arguments. The standalone `ros2 run` commands below remain valid for manual testing or use in other projects.

## Installation

```bash
cd ~/ros2_ws/src
git clone --recursive https://github.com/EvaGoichon/image_processing.git
cd ~/ros2_ws
colcon build --packages-select image_processing --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

## Quick Start

### IR Processing

```bash
# Active IR processor (WFOV mode, fixed-range normalization)
ros2 run image_processing active_ir_processor --ros-args \
  -p mode_ir:=WFOV -p use_fixed_range:=true \
  -p input_topic:=/ir/image_raw -p output_topic:=/ir_processed/image_raw

# Active IR processor (NFOV mode)
ros2 run image_processing active_ir_processor --ros-args -p mode_ir:=NFOV

# Passive IR processor (legacy)
ros2 run image_processing passive_ir_processor
```

### Degradation Simulation

```bash
# RGB dimming (50%)
ros2 run image_processing occlusion -i rgb -m dimming -r 50

# IR-side degradation
ros2 run image_processing occlusion -i ir -m dimming -r 50
```

## Occlusion Node Options

```bash
ros2 run image_processing occlusion -i TYPE -m MODE -r RATE
```

**Options:**
- `-i`: Target stream (`rgb` | `ir`)
- `-m`: Degradation pattern — `dimming`, `left`, `right`, `center`, `noise`, `blur`. All patterns are implemented and produce the expected image transformation; `dimming` is the pattern used in the SLAM robustness evaluation of the associated paper, the others are available for general-purpose use.
- `-r`: Intensity, 0–100

## Topics

### Active IR Processor
- Input: configurable via `input_topic` parameter (default `/ir/image_raw`, 16-bit)
- Output: configurable via `output_topic` parameter (default `/ir_processed/image_raw`, 8-bit mono)
- Key parameters: `mode_ir` (`WFOV` | `NFOV`), `use_fixed_range` (bool)

### Passive IR Processor
- Input: `/ir/image_raw` (16-bit)
- Output: `/camera/image_raw` (8-bit)

### Occlusion Node
**Subscribed:**
- `/rgb/image_raw`
- `/ir_processed/image_raw`
- `/depth/image_raw`
- `/depth_to_rgb/image_raw`

**Published:**
- `/camera/color/image_raw`
- `/camera/ir/image_raw`
- `/camera/depth_ir/image_raw`
- `/camera/depth/image_raw`

## Dependencies

- ROS2 (Humble/Iron)
- OpenCV 4.x
- cv_bridge
- sensor_msgs
- image_transport

## Author

Eva Goichon - [GitHub](https://github.com/EvaGoichon)

## Citation

```bibtex
@software{image_processing_ros2,
  author = {Eva Goichon},
  title = {Image Processing Package for ROS2},
  year = {2025},
  url = {https://github.com/EvaGoichon/image_processing}
}
```
