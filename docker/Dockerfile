# MiniVIO Dockerfile
# Ubuntu 20.04 + C++14 + OpenCV 4.2.0 + Ceres 2.1.0 + Pangolin

FROM ubuntu:20.04

# Avoid interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Install basic build tools and dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    wget \
    unzip \
    pkg-config \
    # GDB for debugging
    gdb \
    # Eigen3
    libeigen3-dev \
    # OpenCV 4.2 (Ubuntu 20.04 default)
    libopencv-dev \
    # Boost
    libboost-all-dev \
    # Ceres dependencies
    libgoogle-glog-dev \
    libgflags-dev \
    libatlas-base-dev \
    libsuitesparse-dev \
    # Pangolin dependencies
    libglew-dev \
    libgl1-mesa-dev \
    libwayland-dev \
    libxkbcommon-dev \
    wayland-protocols \
    libegl1-mesa-dev \
    libpng-dev \
    libjpeg-dev \
    libavcodec-dev \
    libavutil-dev \
    libavformat-dev \
    libswscale-dev \
    # --- Fix runtime GUI/GL warnings ---
    # Mesa software renderer (fixes libGL errors when no GPU)
    mesa-utils \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    # X11 display support
    xvfb \
    x11-utils \
    # GTK and locale (fixes Gtk-WARNING, canberra-gtk-module)
    libcanberra-gtk-module \
    libcanberra-gtk3-module \
    dbus-x11 \
    at-spi2-core \
    # Locale support
    locales \
    && rm -rf /var/lib/apt/lists/*

# Generate and set locale (fixes "Locale not supported by C library")
RUN locale-gen en_US.UTF-8
ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Suppress accessibility bus warning when no dbus session is available
ENV NO_AT_BRIDGE=1

# Use Mesa software rendering (fixes libGL nouveau driver errors)
ENV LIBGL_ALWAYS_SOFTWARE=1
ENV MESA_GL_VERSION_OVERRIDE=3.3

# Build and install Ceres Solver 2.1.0 (requires C++14)
WORKDIR /tmp
RUN git clone --depth 1 --branch 2.1.0 https://github.com/ceres-solver/ceres-solver.git && \
    cd ceres-solver && \
    mkdir build && cd build && \
    cmake .. -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF -DCMAKE_CXX_STANDARD=14 && \
    make -j4 && \
    make install && \
    cd /tmp && rm -rf ceres-solver

# Build and install Pangolin v0.6
WORKDIR /tmp
RUN git clone --depth 1 --branch v0.6 https://github.com/stevenlovegrove/Pangolin.git && \
    cd Pangolin && \
    mkdir build && cd build && \
    cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF && \
    make -j4 && \
    make install && \
    cd /tmp && rm -rf Pangolin

# Update shared library cache
RUN ldconfig

# Copy project source code
WORKDIR /workspace/MiniVIO
COPY . /workspace/MiniVIO/

# Build the project (C++14 is set in CMakeLists.txt)
# RUN mkdir -p build && cd build && \
#     cmake .. -DCMAKE_BUILD_TYPE=Release && \
#     make -j$(nproc)

# Set working directory to bin for running executables
WORKDIR /workspace/MiniVIO/bin

# Default command
CMD ["/bin/bash"]
