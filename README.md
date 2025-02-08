<!-- # Cardinal Perception -->
![Cardinal Perception](doc/cardinal-perception.png)

This package currently comprises localization (lidar odometry + fiducial rebiasing), terrain mapping, traversability evaluation (in progress), and path generation (also in progress) components as used in CSM's autonomy solution (2024 and onward). The system has been designed around our hardware setup, but theoretically is compatible with varying configurations. See the architecture section for more info on sensor inputs and their role in the pipeline.

## Overview
![architecture overview](doc/cardinal-perception-v050-overview.svg)

**Cardinal Perception is currently split into two [ROS] nodes - a core node which accomplishes the vast majority of functionality, as well as a helper node used for detecting AprilTags and calculating pose information. The primary node is architected to act as a pipeline - comprising stages which accomlish localization; terrain mapping; traversibility generation; and trajectory generation tasks, respectively. This architecture supports a failry simple multithreading paradigm as well as minimizes latency for crucial stages (localization) while decoupling later stages such that they don't bottleneck the system as a whole. For more information on each stage as well as I/O and performance considerations, see the [architecture documentation](doc/architecture.md).**

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

3. Install apt dependencies
    ```bash
    sudo apt update
    sudo apt-get install libpcl-dev libopencv-dev
    ```

4. Build with colcon
    ```bash
    colcon build --symlink-install <--executor parallel> <--event-handlers console_direct+> <--cmake-args=-DCMAKE_EXPORT_COMPILE_COMMANDS:BOOL=ON>
    source install/setup.bash
    ```

*This package has been verified on and developed for ROS2 Humble (with Ubuntu 22.04) as well as ROS2 Jazzy (with Ubuntu 24.04). WSL has not been tested but theoretically should work.*

## Usage
**To run Cardinal Perception, you will need (REQUIRED):**
- A `sensor_msgs::msg::PointCloud2` topic providing a 3D LiDAR scan.
- A correctly configured `config/perception.yaml` file (or equivalent).

**Optionally, you may also need:**
- A `sensor_msgs::msg::Imu` topic providing IMU samples. This can help stabilize the odometry system, especially when an orientation estimate is directly usable as apart of each sample.
- A set of `/tf` of `/tf_static` transforms provided by `robot_state_publisher`. This is necessary when the coordinate frame of the LiDAR scan is different from the coordinate frame of the IMU, or if you want to compute odometry for a frame different than that of the LiDAR scan.
- A customized launchfile

## VSCode
If intellisense is not working properly, ensure the CMake and C++ extensions are installed, and the C/C++ configuration is correct. This configuration can be edited by clicking the current configuration name in the bottom-right corner of the window, and editing the JSON file. Under the configuration you plan to use, make sure the following line is present:
```json
"configurationProvider": "ms-vscode.cmake-tools"
```
This tells the C/C++ extension to use CMake as a configuration source for include directories, thus allowing ROS libraries to be correctly used for intellisense. Below is a complete, functional config file for Linux for reference:
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
            "configurationProvider": "ms-vscode.cmake-tools"
        }
    ],
    "version": 4
}
```
__*Last updated: 1/25/25*__

