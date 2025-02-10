# Kocox2 codes

# TortoiseBot ROS2

A complete guide to set up and operate your TortoiseBot with ROS2 Humble/Galactic.

## Prerequisites

- Ubuntu 22.04 (ROS2 Humble) or Ubuntu 20.04 (ROS2 Galactic)
- ROS2 Humble or Galactic installed
- SD card with TortoiseBot Image (optional)

## Installation

### 1. System Setup

If using Ubuntu 22.04, install ROS2 Humble.
If using Ubuntu 20.04, install ROS2 Galactic.

### 2. Create ROS2 Workspace

```bash
source /opt/ros/galactic/setup.bash  # or humble for Ubuntu 22.04
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/
colcon build
source ~/ros2_ws/install/setup.bash
```

### 3. Install Dependencies

```bash
sudo apt install \
  ros-$ROS_DISTRO-joint-state-publisher \
  ros-$ROS_DISTRO-robot-state-publisher \
  ros-$ROS_DISTRO-cartographer \
  ros-$ROS_DISTRO-cartographer-ros \
  ros-$ROS_DISTRO-gazebo-plugins \
  ros-$ROS_DISTRO-teleop-twist-keyboard \
  ros-$ROS_DISTRO-teleop-twist-joy \
  ros-$ROS_DISTRO-xacro \
  ros-$ROS_DISTRO-nav2* \
  ros-$ROS_DISTRO-urdf
```

### 4. Clone Repository

```bash
cd ~/ros2_ws/src
git clone -b ros2-galactic https://github.com/rigbetellabs/tortoisebot.git
cd ~/ros2_ws/
colcon build
source /opt/ros/galactic/setup.bash
source ~/ros2_ws/install/setup.bash
```

## Usage

### Simulation

1. Start the simulation environment:
```bash
ros2 launch tortoisebot_bringup bringup.launch.py use_sim_time:=True
```

2. For teleoperation:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

3. For autonomous navigation with SLAM:
```bash
ros2 launch tortoisebot_bringup autobringup.launch.py use_sim_time:=True exploration:=True
```

### Real Robot Setup

#### Network Configuration

1. Locate the SD card's "writable" folder
2. Navigate to "/etc/netplan/"
3. Edit "50-cloud-init.yaml":
```yaml
wifis:
  wlan0:
    optional: true
    access-points:
      "your_ssid_name":
        password: "your_password"
    dhcp4: true
```

#### Connecting to the Robot

1. SSH into the robot:
```bash
ssh tortoisebot@<robot_ip>
# Default password: raspberry
```

2. On the robot, launch the base nodes:
```bash
source /opt/ros/galactic/setup.bash
ros2 launch tortoisebot_bringup bringup.launch.py use_sim_time:=False
```

3. On your PC, for visualization:
```bash
ros2 launch tortoisebot_description rviz.launch.py
```

### SLAM and Navigation

1. Start mapping:
```bash
ros2 launch tortoisebot_bringup autobringup.launch.py use_sim_time:=False exploration:=True
```

2. Save the map:
```bash
ros2 run nav2_map_server map_saver_cli -f /path_to_map/name_of_map_file.yaml
```

3. Load existing map and navigate:
```bash
ros2 launch tortoisebot_bringup autobringup.launch.py use_sim_time:=False exploration:=False map:=/path_to_map/map_file_name.yaml
```

## Multi-Robot Setup

For multiple robots, set different domain IDs:
```bash
export ROS_DOMAIN_ID=<unique_number>
```

## Tips
- Always source ROS2 and workspace in new terminals:
  ```bash
  source /opt/ros/galactic/setup.bash
  source ~/ros2_ws/install/setup.bash
  ```
- When loading a saved map, ensure the robot is placed at approximately the same position as during mapping
- For the real robot, make sure to use `use_sim_time:=False`
