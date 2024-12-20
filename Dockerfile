FROM --platform=linux/amd64 ubuntu:22.04

# Set the environment variables to prevent interactive prompts
ENV DEBIAN_FRONTEND=noninteractive

# Update the package manager and install prerequisites
RUN apt-get update

RUN apt-get install --fix-missing -y \
    build-essential \
    apt-transport-https \
    g++ \
    git \
    wget \
    curl \
    lsb-release \
    zip unzip tar \
    software-properties-common \
    pkg-config \
    autoconf automake libtool \
    libc++-dev \
    libc++abi-dev libsystemd-dev


# Install CMake
RUN apt-get remove --purge --auto-remove cmake
RUN wget -O - https://apt.kitware.com/keys/kitware-archive-latest.asc 2>/dev/null | gpg --dearmor - | tee /etc/apt/trusted.gpg.d/kitware.gpg >/dev/null
RUN apt-add-repository "deb https://apt.kitware.com/ubuntu/ $(lsb_release -cs) main"
RUN apt-get update
RUN apt-get install kitware-archive-keyring -y
RUN rm /etc/apt/trusted.gpg.d/kitware.gpg
RUN apt-get update
RUN apt-get install cmake -y


# Install Intel RealSense
RUN mkdir -p /etc/apt/keyrings
RUN curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | tee /etc/apt/keyrings/librealsense.pgp > /dev/null

RUN echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo `lsb_release -cs` main" | \
      tee /etc/apt/sources.list.d/librealsense.list
RUN apt-get update

#RUN apt-get install -y \
#    librealsense2-dkms \
#    librealsense2-utils \
#    librealsense2-dev
RUN uname -r
# RUN apt-get install -y librealsense2-dkms
RUN apt-get install -y librealsense2-utils
RUN apt-get install -y librealsense2-dev


# Install FFMPEG
# https://trac.ffmpeg.org/wiki/CompilationGuide/Ubuntu
RUN apt-get install ffmpeg libavcodec-dev libavformat-dev libavutil-dev libswscale-dev -y
RUN apt-get -y install \
  libass-dev \
  libfreetype6-dev \
  libgnutls28-dev \
  libmp3lame-dev \
  libsdl2-dev \
  libtool \
  libva-dev \
  libvdpau-dev \
  libvorbis-dev \
  libxcb1-dev \
  libxcb-shm0-dev \
  libxcb-xfixes0-dev \
  meson \
  ninja-build \
  texinfo \
  yasm \
  zlib1g-dev \
  libunistring-dev libaom-dev libdav1d-dev \
  nasm \
  libx264-dev

RUN apt-get install -y libmlpack-dev


# Install vcpkg
# RUN rm -rf /opt/vcpkg
RUN git clone https://github.com/microsoft/vcpkg.git /opt/vcpkg && \
    /opt/vcpkg/bootstrap-vcpkg.sh -disableMetrics

# Set VCPKG_ROOT environment variable
ENV VCPKG_ROOT /opt/vcpkg


RUN /opt/vcpkg/vcpkg install xtensor
RUN /opt/vcpkg/vcpkg install boost-asio
RUN /opt/vcpkg/vcpkg install boost-lockfree
RUN /opt/vcpkg/vcpkg install ffmpeg
# RUN /opt/vcpkg/vcpkg install mlpack
RUN /opt/vcpkg/vcpkg install draco

RUN apt-get install libopencv-dev -y

# RUN /opt/vcpkg/vcpkg install opencv
#RUN /opt/vcpkg/vcpkg install realsense2
#RUN /opt/vcpkg/vcpkg install draco opencv xtensor ffmpeg boost-asio boost-lockfree realsense2
#RUN /opt/vcpkg/vcpkg install ffmpeg boost-asio boost-lockfree realsense2

# Set the default vcpkg triplet
ENV VCPKG_DEFAULT_TRIPLET x64-linux


WORKDIR /app
COPY . .


# Build the project
RUN mkdir build && mkdir output && cd build && \
    cmake .. -DCMAKE_TOOLCHAIN_FILE=/opt/vcpkg/scripts/buildsystems/vcpkg.cmake && \
    cmake --build .


# RUN mkdir -p build && cd build && \
#    cmake .. && \
#    cmake --build .

# ENV IMG_CAM1_COLOR_IMAGES_FOLDER='/app/images/multicamera/experiment_20241130_2308/cam_D415_851112061420/color'
# ENV IMG_CAM1_DEPTH_IMAGES_FOLDER='/app/images/multicamera/experiment_20241130_2308/cam_D415_851112061420/depth'
# ENV IMG_CAM2_COLOR_IMAGES_FOLDER='/app/images/multicamera/experiment_20241130_2308/cam_D435I_845112071364/color'
# ENV IMG_CAM2_DEPTH_IMAGES_FOLDER='/app/images/multicamera/experiment_20241130_2308/cam_D435I_845112071364/depth'
# ENV IMG_CAM3_COLOR_IMAGES_FOLDER='/app/images/multicamera/experiment_20241130_2308/cam_D435I_849312070097/color'
# ENV IMG_CAM3_DEPTH_IMAGES_FOLDER='/app/images/multicamera/experiment_20241130_2308/cam_D435I_849312070097/depth'
# ENV IMG_CAM4_COLOR_IMAGES_FOLDER='/app/images/multicamera/experiment_20241130_2308/cam_D455_234322305663/color'
# ENV IMG_CAM4_DEPTH_IMAGES_FOLDER='/app/images/multicamera/experiment_20241130_2308/cam_D455_234322305663/depth'

ENV IMG_CAM1_COLOR_IMAGES_FOLDER='/app/images/multicamera/experiment_20241130_2319/cam_D415_851112061420/color'
ENV IMG_CAM1_DEPTH_IMAGES_FOLDER='/app/images/multicamera/experiment_20241130_2319/cam_D415_851112061420/depth'
ENV IMG_CAM2_COLOR_IMAGES_FOLDER='/app/images/multicamera/experiment_20241130_2319/cam_D435I_845112071364/color'
ENV IMG_CAM2_DEPTH_IMAGES_FOLDER='/app/images/multicamera/experiment_20241130_2319/cam_D435I_845112071364/depth'
ENV IMG_CAM3_COLOR_IMAGES_FOLDER='/app/images/multicamera/experiment_20241130_2319/cam_D435I_849312070097/color'
ENV IMG_CAM3_DEPTH_IMAGES_FOLDER='/app/images/multicamera/experiment_20241130_2319/cam_D435I_849312070097/depth'
ENV IMG_CAM4_COLOR_IMAGES_FOLDER='/app/images/multicamera/experiment_20241130_2319/cam_D455_234322305663/color'
ENV IMG_CAM4_DEPTH_IMAGES_FOLDER='/app/images/multicamera/experiment_20241130_2319/cam_D455_234322305663/depth'
ENV BENCHMARK_OUTPUT_FOLDER='/app/output'

ENTRYPOINT ["./build/volume_pipeline", "12345"]
