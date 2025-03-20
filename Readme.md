# IMU Tools for ROS 2

## Overview
This package provides tools for IMU (Inertial Measurement Unit) data filtering and processing in ROS 2. It includes multiple filtering methods:

1. **imu_complementary_filter** – Implements a complementary filter for IMU data fusion.
2. **imu_filter_madgwick** – Implements Madgwick's filter for IMU data fusion.
3. **Butterworth Filters** – Implements Butterworth filters for IMU data processing.
4. **Mean and Median Filters** – Implements mean and median filters for noise reduction.
5. **Low-Pass, Band-Pass, and High-Pass Filters** – Provides various filtering options for IMU data.

## Package Contents
### imu_complementary_filter
- Implements a complementary filter to estimate orientation from IMU data.
- Configuration: `config/filter_config.yaml`
- Launch file: `launch/complementary_filter.launch.py`

### imu_filter_madgwick
- Uses Madgwick’s algorithm to estimate orientation.
- Configuration: `config/imu_filter.yaml`
- Launch files:
  - `launch/imu_filter.launch.py`
  - `launch/imu_filter_component.launch.py`

### Butterworth Filters
- **butterworth_filter_border_coeff.py** – Handles Butterworth filter border coefficient calculations.
- **butterworth_filter_type_coeff.py** – Defines different Butterworth filter types and their coefficients.
- Implements Butterworth filters of 1st, 2nd, 3rd, and 4th order.
- Includes Butterworth low-pass, band-pass, and high-pass filters.

### Mean and Median Filters
- Implements mean and median filters for IMU data smoothing and noise reduction.

### Other Files
- **filter_node.py** – ROS 2 node implementation for applying filters to IMU data.
- **filters.py** – Collection of various filtering functions.
- **msgs_conversion.py** – Utilities for converting IMU messages.
- **`__init__.py`** – Initializes the filter package.

## Installation
Ensure you have ROS 2 installed, then clone the repository and build the package:

```bash
cd ~/ros2_ws/src
git clone https://github.com/ros-perception/imu_tools.git
cd ~/ros2_ws
colcon build --packages-select imu_tools
source install/setup.bash
```

## Usage
To launch the complementary filter:
```bash
ros2 launch imu_complementary_filter complementary_filter.launch.py
```

To launch the Madgwick filter:
```bash
ros2 launch imu_filter_madgwick imu_filter.launch.py
```

## License
This package is licensed under BSD and GPLv3. See the `LICENSE` files for details.

## Contributors
Maintained by the ROS Perception Group.

