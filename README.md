
<p align="center">
  <b>Step-by-step guide for Jetson Orin users: Setting up Intel RealSense SDK and ROS2 Wrapper (PointCloud/Depth) on JetPack 6 + Ubuntu 22.04 + ROS2 Humble</b><br>
  <a href="https://github.com/IntelRealSense/realsense-ros/releases">Latest release notes</a><br>
  <small>Guide maintained by <a href="https://github.com/NamaiBest">NamaiBest</a></small>
</p>
<hr>

[![humble][humble-badge]][humble]
[![ubuntu22][ubuntu22-badge]][ubuntu22]

---

## Table of contents
  * [ROS1 and ROS2 legacy](#ros1-and-ros2-legacy)
  * [Installation (Jetson Orin, JetPack 6, Ubuntu 22.04, ROS2 Humble)](#installation-jetson-orin-jetpack-6-ubuntu-2204-ros2-humble)
  * [Key notes on building from source](#key-notes-on-building-from-source)
  * [Implementation: Running and visualizing point & depth cloud](#implementation-running-and-visualizing-point--depth-cloud)
  * [Next steps](#next-steps)

<hr>

# ROS1 and ROS2 Legacy

<details>
  <summary>
    Intel RealSense ROS1 Wrapper
  </summary>
    Intel Realsense ROS1 Wrapper is not supported anymore<br>
    For ROS1 wrapper, go to <a href="https://github.com/IntelRealSense/realsense-ros/tree/ros1-legacy">ros1-legacy</a> branch
</details>

<details>
   <summary>
     Moving from <a href="https://github.com/IntelRealSense/realsense-ros/tree/ros2-legacy">ros2-legacy</a> to ros2-master
  </summary>
  (See upstream docs for parameter changes and migration)
</details>

---

<details open>
  <summary><b>Installation (Jetson Orin, JetPack 6, Ubuntu 22.04, ROS2 Humble)</b></summary>

This guide was written for new Jetson Orin users (JetPack 6, Ubuntu 22.04) who want to quickly get Intel RealSense SDK and the ROS2 Wrapper working for point cloud and depth streaming.  
It is based on my hands-on experience if you follow the steps in order, you should get a working camera and ROS topics!  
For more board details and compatibility, see [this issue](https://github.com/IntelRealSense/realsense-ros/issues/3364).

---

### Step 1: Install the ROS2 distribution

Follow the official ROS2 Humble installation instructions for Ubuntu 22.04:

- [ROS2 Humble Installation (Debian packages)](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

**Quick steps:**
```bash
# Setup sources
sudo apt update && sudo apt install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble desktop
sudo apt update
sudo apt install -y ros-humble-desktop

# Source ROS2 environment
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Install essential ROS2 tools
sudo apt install -y python3-colcon-common-extensions python3-rosdep
```

---

### Step 2: Install Intel® RealSense™ SDK 2.0 (librealsense)

**For Jetson, use ONLY the JetsonHacksNano [`installLibrealsense.sh`](https://github.com/JetsonHacksNano/installLibrealsense) script (no kernel patching required!).**  
> **Original article:** https://wp.me/p7ZgI9-34j  
> **LibRealSense SDK:** https://github.com/IntelRealSense/librealsense

This will install the official Intel librealsense2 (D400, T265, SR300 support) via the Intel APT repository.

#### 1. Clone the helper scripts

```bash
git clone https://github.com/JetsonHacksNano/installLibrealsense.git
cd installLibrealsense
```

#### 2. Install librealsense using the install script (**RECOMMENDED, what I did**)

```bash
# Run the install script (no kernel patching required)
./installLibrealsense.sh
```
**Do NOT run the `buildLibrealsense.sh` script unless you have a very advanced use-case.**
- This script installs the official Intel® RealSense™ SDK (librealsense2) from the Intel APT repository (i.e., using apt-get install).
- You do **not** have to patch modules or kernels.
- This will install the required libraries and set up udev rules.
- **All necessary USB permissions (udev rules) are also set up automatically by this script, no extra steps required.**
- If you cannot access the camera as a normal user after installation, simply unplug/replug the camera or `sudo reboot` your Jetson.
- This is the path I personally took and recommend for all new Jetson users!

#### 3. Verify camera connection

After installation, launch the RealSense viewer tool:

```bash
realsense-viewer
```

- The camera should appear and show a live feed.
- **Only proceed to the next step after verifying your camera is detected and working here.**

---

### Step 3: Install Intel® RealSense™ ROS2 Wrapper

To avoid version mismatches, **build the ROS2 wrapper from source** (especially on Jetson).

**Install from source**

```bash
# 1. Create a ROS2 workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src/

# 2. Clone the latest ROS2 Intel® RealSense™ wrapper
git clone https://github.com/IntelRealSense/realsense-ros.git -b ros2-master
cd ~/ros2_ws

# 3. Install dependencies
sudo apt-get install python3-rosdep -y
sudo rosdep init # "sudo rosdep init --include-eol-distros" for Foxy and earlier
rosdep update    # "sudo rosdep update --include-eol-distros" for Foxy and earlier
rosdep install -i --from-path src --rosdistro $ROS_DISTRO --skip-keys=librealsense2 -y

# 4. Build
colcon build

# 5. Source environment
ROS_DISTRO=humble  # set your ROS_DISTRO: iron, humble, foxy, etc.
source /opt/ros/$ROS_DISTRO/setup.bash
cd ~/ros2_ws
. install/local_setup.bash
```
</details>

---

# Key notes on building from source

- **Consolidation:**  
  If you build **either** librealsense **or** the ROS wrapper from source, you should **build both from source** within the same environment/workspace.  
  Mixing a source build and a binary install can cause runtime issues, version mismatches, or missing features.
- This guide ensures both SDK and wrapper are managed in one place for robust Jetson support (especially with JetPack 6, Ubuntu 22.04, and ROS2 Humble).

---

# Implementation: Running and visualizing point & depth cloud

To provide a working and reliable example (since upstream examples are often broken on Jetson), this guide uses a **custom configuration YAML** and a refined workflow.

## 1. How to run the RealSense node with your configuration

After cloning this repo (or your fork) and adding your custom YAML (example: `realsense_config.yaml`) to the `realsense2_camera/launch` directory, do:

```bash
ros2 launch realsense2_camera rs_launch.py config_file:=src/realsense-ros/realsense2_camera/launch/realsense_config.yaml
```

- This launches the RealSense node using all parameters defined in your YAML file.
- The YAML contains stable/recommended settings for point cloud and depth w.r.t. Jetson Orin.

## 2. Visualize in RViz2

Open a **new terminal**, source your workspace, and launch RViz2:

```bash
ros2 run rviz2 rviz2
```

### RViz2 setup

1. Under **Global Options** > **Fixed Frame**, set:  
   ```
   camera_link
   ```
2. **To view the depth cloud:**  
   - Click “Add” > select `DepthCloud` (or `Image`)
   - For depth map:  
     - **Topic:** `/camera/camera/aligned_depth_to_color/image_raw`
     - For color overlay: **Topic:** `/camera/camera/color/image_raw`

3. **To view the point cloud:**  
   - Click “Add” > select `PointCloud2`
   - **Topic:** `/camera/camera/depth/color/points`

> **Tip:** Save your RViz2 config (`File > Save Config As`) so displays and topics auto-load next time.

---

## Troubleshooting

- **If you experience lag or crashes when running the point cloud:**
  - Try lowering resolution or framerate (e.g., `640x480x30`).
  - Make sure you are not running other heavy processes.
  - Monitor Jetson system resources with `tegrastats` or `htop`.
  - USB 3 connection is required for full performance.

- **No data in RViz:**
  - Check that topics are being published (`ros2 topic list` and `ros2 topic echo <topic>`).
  - Ensure you have selected the correct topics in RViz.

---

# Next steps

You are now ready to use the RealSense ROS2 nodes on Jetson Orin with JetPack 6.  
**You should now be able to stream depth and point cloud data, and visualize them in RViz2!**

---
[humble-badge]: https://img.shields.io/badge/-HUMBLE-orange?style=flat-square&logo=ros
[humble]: https://docs.ros.org/en/humble/index.html
[ubuntu22-badge]: https://img.shields.io/badge/-UBUNTU%2022%2E04-blue?style=flat-square&logo=ubuntu&logoColor=white
[ubuntu22]: https://releases.ubuntu.com/jammy/