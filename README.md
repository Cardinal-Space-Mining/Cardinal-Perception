# Cardinal Perception

## Build

* Install rosdep dependencies
```bash
sudo rosdep init
rosdep update
rosdep install --ignore-src --from-paths . -r -y
```
* Install apt dependencies
```bash
sudo apt update
sudo apt-get install libpcl-dev libopencv-dev [ros-jazzy-xacro]
```
* Build and install GTSAM
```bash
(navigate to a different directory)
git clone https://github.com/borglab/gtsam
cd gtsam && git checkout 4.2
mkdir build && cd build
cmake .. -DGTSAM_BUILD_EXAMPLES_ALWAYS=OFF \
         -DGTSAM_BUILD_TESTS=OFF \
         -DGTSAM_WITH_TBB=OFF \
         -DGTSAM_USE_SYSTEM_EIGEN=ON \
         -DGTSAM_BUILD_WITH_MARCH_NATIVE=OFF
make -j$(nproc)
sudo make install
```
* Build
```
colcon build --symlink-install [--event-handlers console_direct+] [--executor parallel] [--cmake-args=-DCMAKE_EXPORT_COMPILE_COMMANDS:BOOL=ON]
source install/setup.bash
```

*See the first troubleshooting step as it is a common error on new Ubuntu installations...*

## Troubleshooting

* If the node fails to run due an inability to find the GTSAM libraries, you may need to add the install path to the `LD_LIBRARY_PATH` environment variable:
    * The default install path is `/usr/local/lib/`, which can be appended to the `LD_LIBRARY_PATH` entry in `/etc/environment`. If it does not exist, the entry can be added. Below is an example:
    ```
    PATH="..."
    LD_LIBRARY_PATH="...,/usr/local/lib"
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
__*Last updated: 9/20/24*__

