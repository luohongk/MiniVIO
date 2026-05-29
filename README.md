<!--
 * @Author: Hongkun Luo
 * @Description: MiniVIO — A Minimal, Robust and Easy-to-Learn Visual-Inertial Odometry
 *
 * Hongkun Luo
-->

<h2 align="center">MiniVIO: A Minimal, Robust and Easy-to-Learn<br>Visual-Inertial Odometry</h2>

<h3 align="center">
  <a href="https://luohongkun.top/scholar/">Hongkun Luo</a>
</h3>

<p align="center">
  <a href="README.md">中文</a> | <b>English</b>
</p>

<p align="center">
    <a href="https://github.com/HKUST-Aerial-Robotics/VINS-Mono">
        <img src="https://img.shields.io/badge/Based_on-VINS--Fusion-red" />
    </a>
    <a href="https://github.com/HeYijia/VINS-Course">
        <img src="https://img.shields.io/badge/Refactored_from-VINS--Course-orange" />
    </a>
    <a href="https://cmake.org/">
        <img src="https://img.shields.io/badge/C++-14-blue" />
    </a>
    <a href="https://www.docker.com/">
        <img src="https://img.shields.io/badge/Docker-Ready-2496ED" />
    </a>
    <a href="https://google.github.io/styleguide/cppguide.html">
        <img src="https://img.shields.io/badge/Code_Style-Google_C++-green" />
    </a>
    <a href="https://www.gnu.org/licenses/gpl-3.0.html">
        <img src="https://img.shields.io/badge/License-GPL3.0-yellow.svg" />
    </a>
</p>

<div align=center><img src="image/README/1780056529346.png" width=100%></div>

---

## 🎬 Demo

<div align=center><img src="image/图片1-1.png" width=100%></div>

<div align=center><img src="image/minivio.gif" width=100%></div>

---

## 🔍 Overview

**MiniVIO** is a **minimal, single-step debuggable Visual-Inertial Odometry (VIO)** system designed for **learning and research**. Built on top of [VINS-Course](https://github.com/HeYijia/VINS-Course) by Yijia He, MiniVIO keeps the lightweight ROS-free architecture, and further adds **robustness fixes**, **engineering refactoring**, and **Docker-based deployment**.

### 💡 Why open-source MiniVIO?

| Pain point                                                   | How MiniVIO solves it                                                                                            |
| ------------------------------------------------------------ | ---------------------------------------------------------------------------------------------------------------- |
| VINS-Mono / VINS-Fusion are heavy and tightly coupled to ROS | **ROS-free**, pure C++ implementation — easy to step through and study                                    |
| VINS-Course back-end crashes on noisy data                   | Fixes for**bad features / negative inverse depth / NaN-Inf / ill-conditioned matrices** and other failures |
| Painful build environment setup                              | Provides a**one-click Docker** image, ready to run in 10 minutes                                           |
| Inconsistent code style and high reading barrier             | Refactored under**Google C++ Style** — clean naming, decoupled modules                                    |

### 🛡️ Robustness Fixes (Core Improvements)

MiniVIO completely resolves the following fatal issues in the VINS back-end optimizer:

- ✅ Crashes / freezes caused by bad features and negative inverse depth
- ✅ Pose update halts caused by NaN / Inf values
- ✅ Permanent prior pollution caused by ill-conditioned matrices
- ✅ LM solver divergence and runaway

> 💬 MiniVIO is intentionally lightweight and very approachable — a great entry point for anyone who wants to truly understand how a VIO system works internally.

---

## 🐳 Quick Start (Docker, Recommended)

### 📋 Prerequisites

- Docker
- An X11 server (for Pangolin visualization)

### 🏗️ 1. Build the Docker Image

The build takes about **10 minutes**:

```bash
git clone https://github.com/luohongk/MiniVIO
cd MiniVIO
docker build -f docker/Dockerfile -t minivio:latest .
```

### 🚀 2. Launch the Container

```bash
xhost +local:root && \
docker run -it \
  --gpus all \
  --network=host \
  --privileged \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY=$DISPLAY \
  -e HOME=/root \
  -v <YOUR_MINIVIO_PATH>:/root/workspace/MiniVIO \
  -v <YOUR_DATA_PATH>:/root/data \
  --name MiniVIO_work \
  -w /root/workspace/MiniVIO \
  minivio:latest
```

> Replace `<YOUR_MINIVIO_PATH>` with the path to your local MiniVIO repo, and `<YOUR_DATA_PATH>` with your dataset directory.

For example:

```bash
xhost +local:root && \
docker run -it \
  --gpus all \
  --network=host \
  --privileged \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -e DISPLAY=$DISPLAY \
  -e HOME=/root \
  -v /home/lhk/workspace/MiniVIO/:/root/workspace/MiniVIO \
  -v /home/lhk/data:/root/data \
  --name MiniVIO_work \
  -w /root/workspace/MiniVIO \
  minivio:latest
```

### 🔨 3. Build Inside the Container

```bash
cd ~/workspace/MiniVIO
mkdir -p build && cd build

# Release mode (recommended)
cmake .. -DCMAKE_BUILD_TYPE=Release

# Debug mode (for single-step debugging)
# cmake .. -DCMAKE_BUILD_TYPE=Debug

make -j8
```

### ▶️ 4. Run

Download the example dataset: [V1_01_easy.zip](https://github.com/luohongk/MiniVIO/releases/download/example/V1_01_easy.zip)

Edit `config/euroc_config.yaml` and update the dataset paths to your own:

```yaml
# common parameters
imu_topic:    "/imu0"
image_topic:  "/cam0/image_raw"
output_path:  "/home/lhk/output"

# dataset parameters
imu_data_file:   "V101_imu0.txt"   # IMU data file (relative to config path)
image_data_file: "V101_cam0.txt"   # image timestamp file (relative to config path)
image_subdir:    "cam0/data/"      # image subdirectory (relative to data path)
```

Launch:

```bash
cd build/bin
./run_euroc ~/data/slam_data/V2_02_medium/mav0/ ../../config/
```

> Replace `~/data/slam_data/V2_02_medium/mav0/` with the path to your own EuRoC dataset.

Full demo video: [image/5月29日.mp4](image/5月29日.mp4)

---

## 🔧 Build Without Docker

### 📋 Dependencies

| Dependency             | Version        | Notes                                                                              |
| ---------------------- | -------------- | ---------------------------------------------------------------------------------- |
| **Ubuntu**       | 20.04 (64-bit) | 18.04 / 22.04 also work; OpenCV must be built from source                          |
| **Pangolin**     | 0.6            | [GitHub](https://github.com/stevenlovegrove/Pangolin)                                 |
| **OpenCV**       | 4.2.0          | Installable via `apt` on Ubuntu 20.04                                            |
| **Eigen**        | -              | `sudo apt install libeigen3-dev`                                                 |
| **Ceres Solver** | 2.1.0          | Used for SfM in initialization;[GitHub](https://github.com/ceres-solver/ceres-solver) |

### 🛠️ 1. Install Dependencies

**Pangolin 0.6:**

```bash
git clone --depth 1 --branch v0.6 https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin && mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=OFF -DBUILD_TESTS=OFF
make -j4 && sudo make install
```

**OpenCV 4.2.0 (Ubuntu 20.04):**

```bash
sudo apt-get install libopencv-dev
```

> Ubuntu 18.04 / 22.04 users should build from the [official OpenCV repo](https://github.com/opencv/opencv).

**Eigen:**

```bash
sudo apt-get install libeigen3-dev
```

**Ceres 2.1.0:**

```bash
git clone --depth 1 --branch 2.1.0 https://github.com/ceres-solver/ceres-solver.git
cd ceres-solver && mkdir build && cd build
cmake .. -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF -DCMAKE_CXX_STANDARD=14
make -j4 && sudo make install
```

### 🏗️ 2. Build MiniVIO

```bash
git clone https://github.com/luohongk/MiniVIO
cd MiniVIO
mkdir -p build && cd build

# Release mode
cmake .. -DCMAKE_BUILD_TYPE=Release

# Debug mode (optional)
# cmake .. -DCMAKE_BUILD_TYPE=Debug

make -j8
```

---

## ▶️ Run Examples

### 1️⃣ Curve Fitting — Verify the Solver

```bash
cd build/bin
./curve_fitting
```

### 2️⃣ MiniVIO on the EuRoC Dataset

```bash
cd build/bin
./run_euroc ~/data/cuvslam_data/V2_02_medium/mav0/ ../../config/
```

---

## 📊 Accuracy Evaluation

Compare against EuRoC ground truth using [EVO](https://github.com/MichaelGrupp/evo):

```bash
evo_ape euroc euroc_mh05_groundtruth.csv pose_output.txt -a -p
```

---

## 📄 License

MiniVIO is released under the [GNU General Public License v3.0](https://www.gnu.org/licenses/gpl-3.0.html).

Reliability of the codebase is still being improved. For technical questions or collaboration:

Project contact: **Hongkun Luo** — `luohongkun0715@gmail.com`

---

## 🙏 Acknowledgements

MiniVIO stands on the shoulders of these outstanding open-source projects:

- [VINS-Mono](https://github.com/HKUST-Aerial-Robotics/VINS-Mono) — The classic visual-inertial SLAM work from Prof. Shaojie Shen's group at HKUST
- [VINS-Course](https://github.com/HeYijia/VINS-Course) — Yijia He's ROS-free educational version, the direct foundation of this project
- [Ceres Solver](http://ceres-solver.org/) — Non-linear optimization library
- [Pangolin](https://github.com/stevenlovegrove/Pangolin) — Visualization library

Special thanks to the authors above for making SLAM learning so much more accessible to the community.
