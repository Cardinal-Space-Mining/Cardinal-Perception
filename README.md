<!-- # Cardinal Perception -->
![Cardinal Perception](doc/cardinal-perception.png)

This package currently comprises localization (lidar odometry + fiducial rebiasing), terrain mapping, traversability evaluation (in progress), and path generation (also in progress) components as used in CSM's autonomy solution (2024 and onward). The system has been designed around our hardware setup, but theoretically is compatible with varying configurations. See the architecture section for more info on sensor inputs and their role in the pipeline.

## Architecture
![architecture overview](doc/cardinal-perception-v050-overview.svg)
**Cardinal Perception is currently split into two [ROS] nodes - a core node which accomplishes the vast majority of functionality, as well as a helper node used for detecting AprilTags and calculating pose information. The primary node is architected to act as a pipeline - including stages accomplishing localization; terrain mapping; traversibility generation; and trajectory generation processes, respectively. This architecture provides effective utilization of multi-core CPUs as well as minimizes latency for crucial stages (localization) while decoupling later stages such that they don't bottleneck the system as a whole.**

### Localization
The localization solution consists of two components: LiDAR odometry and fiducial detection for global alignment. The LiDAR odometry is based off of [direct_lidar_odometry](https://github.com/vectr-ucla/direct_lidar_odometry) (DLO) and utilizes a scan-to-scan and scan-to-map based odometry solution, with optional IMU input to seed registration. Fiducial detection is not required but can come in one of two forms - AprilTag detections or LiDAR-reflector detection (this is custom). Fusion between local and global measurements is currently only done on a simple transform-offset basis, utilizing the `map` and `odom` transform-tree frames (meaning that without fiducial detection, odometry is implicitly treated as the full localization solution).

**Localization subsribes to the following ROS2 topics:**
1. A `sensor_msgs/msg/PointCloud2` topic as configured by the parameter `scan_topic` (default is `scan`). This can be any pointcloud and is not restricted to being organized or needing have timestamp data, etc..
2. A `sensor_msgs/msg/Imu` topic as configured by the parameter `imu_topic` (default is `imu`). *This is optional and IMU usage can be disabled by setting the `dlo.imu.use` parameter to `false`.* IMU data must be in the same time-base as LiDAR scans, since these are timestamp matched internally.
3. A `cardinal_perception/msg/TagsTransform` topic which defaults to `"/tags_detections"` in both nodes. This is only relevant when using the AprilTag detector node to provide global measurements, which can be parameter-disabled by setting `use_tag_detections` to `false` or compile-time disabled by using setting the `USE_TAG_DETECTION_PIPELINE` macro to `0`.

### Terrain Mapping
Terrain mapping attempts to model the real world utilizing a set of cartesian points. The map itself is stored as a pointcloud, and indexed using a specialized octree which automatically voxelizes all points in the same leaf and allows for direct deletion of points/leaves. Point-deletion is calculating using the custom-developed "KDTree Frustum Collision" (KFC) mapping model, which provides (more or less) realtime mapping using just LiDAR scans. This stage does not have any additonal ROS subscriptions but works directly off of buffers which are reused from the localization stage, as well as the most recent localization results.

### Traversability Generation
(Not implemented yet!)

### Trajectory Generation
(Not implemented yet!)

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

