# Cardinal Perception

This package currently comprises localization (lidar odometry + apriltag rebias) and mapping components as used in CSM's autonomy solution (2024 and onward). The system has been designed around our hardware setup, but theoretically is compatible with varying configurations. See the architecture section for more info on sensor inputs and their role in the pipeline.

## Architecture
TODO

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
__*Last updated: 11/28/24*__

