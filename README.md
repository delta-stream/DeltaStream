# DeltaStream: 2D-Inferred Delta Encoding for Live Volumetric Video Streaming

<p align="center">
  <img src="https://github.com/user-attachments/assets/b8cb1ed1-82bb-48f7-b439-09991be9dc59" />
</p>

### OS Support
DeltaStream supports the following operating systems:
- Ubuntu 20.04

## Getting Started

### Prerequisites
`Open3D` and `Cereal` are required to build the project. 
The following steps will guide you through the installation of these dependencies.

#### Build Tools
Install system packages needed build the project.
```shell
sudo apt-get update
sudo apt-get install -y \
    build-essential \
    apt-transport-https \
    g++ git wget curl \
    lsb-release zip unzip tar \
    software-properties-common \
    pkg-config autoconf automake libtool \
    libc++-dev libc++abi-dev libsystemd-dev \
    nasm libudev-dev libglu1-mesa-dev
```

Install CMake version 3.29 or higher if it's not installed already.
```shell
# Install the latest CMake version
sudo apt-get remove --purge --auto-remove cmake
wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | sudo tee /etc/apt/trusted.gpg.d/kitware.gpg >/dev/null
sudo apt-add-repository "deb https://apt.kitware.com/ubuntu/ $(lsb_release -cs) main"
sudo apt-get update
sudo apt-get install kitware-archive-keyring -y
sudo rm /etc/apt/trusted.gpg.d/kitware.gpg
sudo apt-get update
sudo apt-get install cmake -y
```
If `sudo apt-get update` results in an error, try running the following command:
```shell
# Replace the 1A127079A92F09ED with the key in the error message
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys 1A127079A92F09ED
```

Install OpenCV as a system package.
```shell
sudo apt-get install libopencv-dev -y
```

Install Intel RealSense SDK as a system package.
```shell
sudo mkdir -p /etc/apt/keyrings
sudo curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null
echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
      sudo tee /etc/apt/sources.list.d/librealsense.list
sudo apt-get update
sudo apt-get install -y librealsense2-dev
```

#### Manually Installed Dependencies
```shell
cd /path/to/deltastream
mkdir libs
cd libs

# Open3D
wget https://github.com/isl-org/Open3D/releases/download/v0.18.0/open3d-devel-linux-x86_64-cxx11-abi-0.18.0.tar.xz
tar -xf open3d-devel-linux-x86_64-cxx11-abi-0.18.0.tar.xz
mv open3d-devel-linux-x86_64-cxx11-abi-0.18.0 open3d
rm open3d-devel-linux-x86_64-cxx11-abi-0.18.0.tar.xz

# Cereal
wget https://github.com/USCiLab/cereal/archive/refs/tags/v1.3.2.tar.gz
tar -xf v1.3.2.tar.gz
mv cereal-1.3.2/include/cereal ./
rm -rf cereal-1.3.2 v1.3.2.tar.gz
```

DeltaStream uses [vcpkg](https://vcpkg.io/en/index.html) to manage most of its dependencies.
The following steps will guide you through the installation of `vcpkg` and the required dependencies.

(Credits: [Tutorial: Install and use packages with CMake](https://learn.microsoft.com/en-us/vcpkg/get_started/get-started?pivots=shell-bash)

```shell 
# Navigate to the directory where you want to install vcpkg (e.g., your home directory)
cd /path/to/desired/vcpkg/install/directory
git clone https://github.com/microsoft/vcpkg.git
cd vcpkg && ./bootstrap-vcpkg.sh
export VCPKG_ROOT=/path/to/vcpkg # Replace /path/to/vcpkg with the path to the vcpkg directory
export PATH=$VCPKG_ROOT:$PATH
```
> The commands to set environment variables is only valid for the current terminal session. 
> To make the changes permanent, add the above two lines to your shell's configuration file (e.g., ~/.bashrc, ~/.zshrc)

### Build and Run
```shell
cd /path/to/deltastream
mkdir build && mkdir output
cd build
cmake .. -DCMAKE_TOOLCHAIN_FILE=$VCPKG_ROOT/scripts/buildsystems/vcpkg.cmake
cmake --build . --config Release
```

Run the server.
```shell
./server <port>
```

Run the client.
```shell
./client <server_ip> <server_port>
```

### Docker Quickstart
Coming soon


### Parameter Configuration

Update the following parameters in the `config.h` file according to your settings:

1. **StreamMode**: Choose one of the following modes:
   - `LiveScan3D`
   - `MetaStream`
   - `DeltaStream`
2. **num_cameras**: Specify the number of cameras being used. We used four cameras.
3. **depth_filters_per_cam**: Set the maximum depth for each camera.

Note that the server must be connected to Intel RealSense cameras.

You can also test with your own dataset by setting `img_mode=true`.

Update the following parameters in the `client.cpp` file according to your settings:

1. **transformation_matrix**: Define the transformation matrix for each camera to align multiple cameras into the world coordinate system.


### Client
We tested the client on [Intel Core i7-9750H @ 2.60GHz](https://www.cpubenchmark.net/cpu.php?id=3425&cpu=Intel+Core+i7-9750H+%40+2.60GHz), which has lower compuatational capabilities compared to [Apple Vision Pro](https://www.apple.com/apple-vision-pro/specs/) [M2 Chip](https://www.cpubenchmark.net/cpu.php?cpu=Apple+M2+8+Core+3500+MHz&id=4922).
