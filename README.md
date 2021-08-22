# Sensor Fusion Self-Driving Car Course

<img src="media/ObstacleDetectionFPS.gif" width="700" height="400" />

### Welcome to the Sensor Fusion course for self-driving cars.

## Local Installation

### Prerequisites

The project was developed on Windows 10 with C++17 and PCL v1.9.1 but should work on other platforms.

#### Windows 10

* [Visual Studio 2019 Community](https://visualstudio.microsoft.com/downloads/) (with C++17 supporting MSVC compiler)
* vcpkg syn'ed to commit one-prior to c7cd618 (see below)
* PCL 1.7+
* (Optional) [CMake](https://cmake.org/download/). You may use the one that comes with Visual Studio
* (Optional) Visual Studio Code

#### Ubuntu 18.04

* C++ v17
* PCL v1.7+
* gcc v7+ (or any C++17 supporting compiler)
 

#### WINDOWS Installation

For other platforms see the original [README](https://github.com/udacity/SFND_Lidar_Obstacle_Detection.git)

##### Install via vcpkg

1. Clone vcpkg depot

   ```sh
   git clone https://github.com/Microsoft/vcpkg.git
   git checkout c7cd618^^
   cd vcpkg
   bootstrap-vcpkg.bat
   vcpkg integrate install
   vcpkg install pcl
   ```
(The checkout command is actually `git checkout c7cd618^`, but the "`^`" character needs to be escaped on Windows). This particular checkin breaks VTK version so PCL will not be installed correctly if using a later `vcpkg`.

2. Clone this github repo

   ```shell
   cd ~
   git clone https://github.com/fierval/SFND_Lidar_Obstacle_Detection.git
   ```

3. To build and run from the command line (make sure cmake is in your path):

   ```shell
   cd ~/SFND_Lidar_Obstacle_Detection
   mkdir build && cd build
   cmake .. -DCMAKE_TOOLCHAIN_FILE=<vcpkg-git-repo-root>\scripts\buildsystems\vcpkg.cmake  
   cmake --build c:/git/udacity/LidarDetection/build --config Debug --target ALL_BUILD -j 18 --
   environment.exe
   ```

   Or build and run with Visual Studio or Visual Studio Code

## Project output (YouTube Video)

[![YouTube Video](https://img.youtube.com/vi/07pfecuDdKE/0.jpg)](https://www.youtube.com/watch?v=07pfecuDdKE)
