# Image Processing Package

ROS2 package for IR image processing and camera failure simulation for RGBIRD SLAM systems.

## Features

- **Active IR Processor**: Advanced IR enhancement with percentile clipping and bilateral filtering
- **Passive IR Processor**: Basic IR processing with gamma correction
- **Camera Failure Simulator**: Test SLAM robustness with simulated sensor failures

## Installation

```bash
cd ~/ros2_ws/src
git clone https://github.com/EvaGoichon/image_processing.git
cd ~/ros2_ws
colcon build --packages-select image_processing
source install/setup.bash
```

## Quick Start

### IR Processing

```bash
# Active IR processor 
ros2 run image_processing active_ir_processor

# Passive IR processor 
ros2 run image_processing passive_ir_processor
```

### Failure Simulation

```bash
# RGB camera occlusion (50% center)
ros2 run image_processing occlusion -i rgb -r 50 -m center

# IR sensor failure (affects all IR/depth streams)
ros2 run image_processing occlusion -i ir -r 70 -m center

# Dimming (30% darker)
ros2 run image_processing occlusion -i rgb -r 30 -m dimming
```

## Occlusion Options

```bash
ros2 run image_processing occlusion -i TYPE -r RATE -m MODE
```

**Options:**
- `-i`: Camera type (`rgb` | `ir`)
- `-r`: Intensity 0-100%
- `-m`: Pattern (`left` | `right` | `center` | `noise` | `blur` | `dimming`)

**Examples:**
```bash
# Left side occlusion
ros2 run image_processing occlusion -i rgb -r 40 -m left

# Gaussian noise
ros2 run image_processing occlusion -i rgb -r 60 -m noise

# Complete IR sensor failure
ros2 run image_processing occlusion -i ir -r 100 -m center
```

## Topics

### Active IR Processor
- Input: `/ir/image_raw` (16-bit)
- Output: `/ir_processed/image_raw` (8-bit)

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
