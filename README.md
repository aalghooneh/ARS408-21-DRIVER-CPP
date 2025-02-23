# ARS408-21-DRIVER-CPP

## Overview
Provides a ROS driver for the Continental ARS408-21 radar. Decodes CAN frames into ROS messages for object and cluster information.

## Features
- Publishes decoded radar data to ROS topics
- Converts radar targets to `PointCloud2` messages
- Supports both cluster and object detection modes

## Dependencies
- ROS (tested with Melodic/Noetic)
- can_msgs, geometry_msgs, nav_msgs, roscpp, rospy, std_msgs, message_filters, message_generation
- Boost (program_options)
- PCL (for point cloud processing)
- Eigen

## Installation
1. Clone this repository into a catkin workspace.  
2. Run:
   ```bash
   cd ~/catkin_ws
   catkin_make
   source devel/setup.bash
   ```

## Usage
- Launch with:
  ```bash
  roslaunch decode_radars launch_radar_clustering.launch
  ```
- Set command-line arguments (e.g. “--mode cluster --id 2”) to switch modes and identifiers.

## Contribution
Contributions are welcome! Please open an issue or submit a pull request.

## License
Distributed under the [MIT License](LICENSE).
