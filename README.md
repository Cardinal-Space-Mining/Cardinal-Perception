<!-- # Cardinal Perception -->
![Cardinal Perception](doc/cardinal-perception.png)

Cardinal Perception is CSM's perception package used as a base for all advanced robot autonomy. It is structured as a multi-stage pipeline, consisting of **odometry**, **fiducial-based localization**, **mapping**, **traversiblity estimation**, and **path-planning** components.

## Overview
![architecture overview](doc/cardinal-perception-v050-overview.svg)

*This diagram is slightly out of date!*

See the [architecture documentation](doc/architecture.md) for more information on individual pipeline stages.

## Build
1. Install [ROS2](https://docs.ros.org/en/jazzy/Installation.html) if necessary

2. Use rosdep to install ROS package dependencies
    - Initialize rosdep if necessary:
        ```bash
        sudo rosdep init
        ```
    - Update and install:
        ```bash
        rosdep update
        rosdep install --ignore-src --from-paths . -r -y
        ```

3. Install apt dependencies (should have already been resolved by rosdep)
    ```bash
    sudo apt update
    sudo apt-get install libpcl-dev libopencv-dev
    ```

4. Build with colcon
    ```bash
    colcon build --symlink-install <--executor parallel> <--event-handlers console_direct+> <--cmake-args=-DCMAKE_EXPORT_COMPILE_COMMANDS:BOOL=ON>
    source install/setup.bash
    ```
    Additionally, there are various compile-time configurations which are exposed as CMake options. These are all listed in the [config generator template](cmake/config.hpp.in).

*The project has been verified to build and function on ROS2 Humble (Ubuntu 22.04), Jazzy (Ubuntu 24.04) and Kilted (Ubuntu 24.04), on both x86-64 and aarch64 architectures, as well as WSL.*

## Usage
### Prerequisites
**To run Cardinal Perception, you will need (REQUIRED):**
- A `sensor_msgs::msg::PointCloud2` topic providing a 3D LiDAR scan.
- A correctly configured `config/perception.yaml` file (or equivalent) - see the [related documentation](doc/config.md) for information on parameters.

**Optionally, you may also need:**
- A `sensor_msgs::msg::Imu` topic providing IMU samples. This can help stabilize the odometry system, especially when an orientation estimate is directly usable as apart of each sample.
- A set of `/tf` of `/tf_static` transforms provided by `robot_state_publisher`. This is necessary when the coordinate frame of the LiDAR scan is different from the coordinate frame of the IMU, or if you want to compute odometry for a frame different than that of the LiDAR scan.
- A customized launch configuration to support your specific robot setup.

**Finally, to use the AprilTag detector for global estimates, you will need:**
- A set of `sensor_msgs::msg::Image` and accompanying `sensor_msgs::msg::CameraInfo` topic for each camera to be used.
- A launch configuration file describing the behavior of the fiducial-tag system. See the provided [config file](config/perception.json) for an example.

### Running
The following nodes are built, which can all be run individually or in a launch system:
- `perception_node` - The core perception package
- `tag_detection_node` - The apriltag detector
- `pplan_client_node` - A service caller that can interface with foxglove studio cursor clicks

However, to fully configure the perception system, you will need to build and source the [launch_utils](https://github.com/Cardinal-Space-Mining/launch-utils) package. All three packages can then be configured an run using the [JSON config](config/perception.json) and following launch command:
```bash
ros2 launch cardinal_perception perception.launch.py <launch args...>
```
**_For an advanced usage example, see [this repo](https://github.com/Cardinal-Space-Mining/lance-2025)._**

## VSCode
The build script exports compile commands which can help VSCode's C/C++ extension resolve correct syntax highlighting. To ensure this is working, paste the following code into the `c_cpp_properties.json` file (under .vscode directory in a workspace):
```json
{
    "configurations": [
        {
            "name": "Linux",
            "includePath": [
                "${workspaceFolder}/**"
            ],
            "defines": [],
            "compilerPath": "/usr/bin/gcc",
            "intelliSenseMode": "linux-gcc-x64",
            "cStandard": "c17",
            "cppStandard": "c++17",
            "compileCommands": [
                "build/compile_commands.json"
            ]
        }
    ],
    "version": 4
}
```
__*Last updated: 9/13/25*__

