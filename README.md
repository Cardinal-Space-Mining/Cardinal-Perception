<!-- # Cardinal Perception -->
![Cardinal Perception](doc/cardinal-perception.png)

Cardinal Perception is CSM's ROS2 perception package which comprises all the necessary prerequisites for advanced autonomy; most notably localization, path-planning and terrain analysis. It has been designed around using the SICK MultiScan136 3D-LiDAR as its only input, although is highly configrable and likely to support other LiDAR/IMU setups.

# Setup
> [!IMPORTANT]
> **CSM team members:** Cardinal Perception is usually included as a submodule in larger robot code projects, where build/install procedures are integrated into a larger system/script. **YOU SHOULD NOT NEED TO CLONE/USE THIS REPO ON ITS OWN!**

> [!NOTE]
> Cardinal Perception has been verified to build and function on **ROS2 Humble (Ubuntu 22.04)**, **Jazzy (Ubuntu 24.04)** and **Kilted (Ubuntu 24.04)**, on both **x86-64** and **aarch64** architectures, as well as **WSL**.

1. Install [ROS2](https://docs.ros.org/en/jazzy/Installation.html) if necessary

2. Setup your workspace and clone required repos
    - Create directories:
        ```bash
        mkdir ros-ws && cd ros-ws
        mkdir src && cd src
        ```
    - Clone repos:
        ```bash
        git clone https://github.com/Cardinal-Space-Mining/Cardinal-Perception -b main cardinal-perception
        git clone https://github.com/Cardinal-Space-Mining/launch-utils -b main launch-utils
        git clone https://github.com/Cardinal-Space-Mining/csm-metrics -b main csm-metrics
        ```
    - Navigate back to your workspace directory for the following steps:
        ```bash
        cd ..
        ```

3. Use rosdep to install ROS package dependencies
    - Initialize rosdep if necessary:
        ```bash
        sudo rosdep init
        ```
    - Update and install:
        ```bash
        rosdep update
        rosdep install --ignore-src --from-paths ./src -r -y
        ```

4. Install apt dependencies (should have already been resolved by rosdep)
    ```bash
    sudo apt update
    sudo apt-get install libpcl-dev libopencv-dev
    ```

5. Build with colcon
    ```bash
    colcon build \
        --symlink-install \
        --event-handlers console_direct+ \
        --cmake-args=-DCMAKE_EXPORT_COMPILE_COMMANDS:BOOL=ON
    source install/setup.bash
    ```
    > [!TIP]
    > There are various compile-time configurations which are exposed as CMake options. These are all listed in the [config generator template](cmake/config.hpp.in).

# Usage
The best way to run Cardinal Perception is by using the included launchfile, which utilizes [launch-utils](https://github.com/Cardinal-Perception/launch-utils) to setup everything using the included [JSON config](config/perception.json):
```bash
ros2 launch cardinal_perception perception.launch.py
```
> [!IMPORTANT]
> Review the following sections to ensure ROS2 and your config file are setup properly!

## ROS Prerequisites
Cardinal Perception requires the following in order to run:
1. A `sensor_msgs::msg::PointCloud2` topic providing a 3D LiDAR scan. This can be live or from a bag.
2. A proper transform definition, usually published on `/tf` and `/tf_static` by `robot_state_publisher`. The launch-utils package provides a way to bundle the config for this and automatically launch this node - see the [relevant docs](https://github.com/Cardinal-Space-Mining/launch-utils?tab=readme-ov-file#usage) for more information.

The following are not required but are useful in some situations:
- A `sensor_msgs::msg::Imu` topic providing IMU samples. This is used by the LIO system to pre-align scans and can drastically improve localization quality when in featureless environments or if scan message reliability is decreased. Some LiDARs contain integrated IMUs in which case this is a freebe.
- A set of `sensor_msgs::msg::Image` and accompanying `sensor_msgs::msg::CameraInfo` topic for any number of cameras. These can be used to enable the optional AprilTag alignment pipeline.

## Configuration
As already alluded, Cardinal Perception uses the [launch-utils](https://github.com/Cardinal-Perception/launch-utils) package to handle configuration and launching. This is due to the fact that many other ROS2 nodes must be run alongside Cardinal Perception in order to accomplish anything useful. By default, the included launch system uses [this file](config/perception.json) to configure and launch everything. This is merely a starting point which includes defaults for all the main parameters that can be used by Cardinal Perception, as well a barebones robot setup for the various boilerplate ROS2 nodes.

The primary config sections used by Cardinal Perception are:
- `perception`: Configures the main perception node.
- `tag_detection`: Configures the AprilTag detector node.
- `pplan_client`: Configures (enables/disables) the path planning client node.

Due to the large number of paramters, up-to-date descriptions on what each one does have not been documented here. Old documentation can be referenced [here](doc/config.md) for some parameters that haven't changed in a while, and more specific docs about this may be included in the future. Fortunately, many configs can be left unchanged and the most crucial ones are quite straightforward in what they do.

> [!TIP]
> To better understand the config file layout, as well as boilerplate config sections, check out the [relevant docs](https://github.com/Cardinal-Space-Mining/launch-utils?tab=readme-ov-file#how-it-works).

> [!TIP]
> The example config is a cut down version of the primarly config used by CSM's LANCE robot(s). The full config can be found [here](https://github.com/Cardinal-Space-Mining/lance-2025/blob/main/lance/config/lance.json).

# Development

## VSCode
The provided build command exports compile commands which can help VSCode's C/C++ extension resolve correct syntax highlighting. To ensure this is working, paste the following code into the `c_cpp_properties.json` file (under .vscode directory in a workspace):
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

__*Last updated on 1/19/26*__

