# Kocox2 : A 2 wheeled differential drive robot with a lidar 

## Prerequisites

- Ubuntu 22.04 
- ros2 Humble installed

## Installation

### 1. System Setup

Ubuntu 22.04 : install ros2 Humble. 

### 2. Create ros2 Workspace

```bash
source /opt/ros/humble/setup.bash  
mkdir -p ~/kocox2_ws/src
cd ~/kocox2_ws/
colcon build
source ~/kocox2_ws/install/setup.bash
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
cd ~/kocox2_ws/src
git clone https://github.com/Kou-shik2004/KocoX2
cd ~/kocox2_ws/
colcon build
source /opt/ros/humble/setup.bash
source ~/kocox2_ws/install/setup.bash
```

## Usage

### Simulation

![simulation](KocoX2/sim)

1. Start the simulation environment:
```bash
ros2 launch kocox2_bringup bringup.launch.py use_sim_time:=True
```

2. For teleoperation:
```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

3. For autonomous navigation with SLAM:
```bash
ros2 launch kocox2_bringup autobringup.launch.py use_sim_time:=True exploration:=True
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
ssh kocox2@<robot_ip>    # Default password: raspberry
```

2. On the robot, launch the base nodes:
```bash
source /opt/ros/humble/setup.bash
kocox2 launch kocox2_bringup bringup.launch.py use_sim_time:=False
```

3. On your PC, for visualization:
```bash
kocox2 launch kocox2_description rviz.launch.py
```

### SLAM and Navigation

1. Start mapping:
```bash
kocox2 launch kocox2_bringup autobringup.launch.py use_sim_time:=False exploration:=True
```

2. Save the map:
```bash
kocox2 run nav2_map_server map_saver_cli -f /path_to_map/name_of_map_file.yaml
```

3. Load existing map and navigate:
```bash
kocox2 launch kocox2_bringup autobringup.launch.py use_sim_time:=False exploration:=False map:=/path_to_map/map_file_name.yaml
```

## Multi-Robot Setup

For multiple robots, set different domain IDs:
```bash
export ROS_DOMAIN_ID=<unique_number>
```

## Tips
- Always source kocox2 and workspace in new terminals:
  ```bash
  source /opt/ros/humble/setup.bash
  source ~/kocox2_ws/install/setup.bash
  ```
- When loading a saved map, ensure the robot is placed at approximately the same position as during mapping
- For the real robot, make sure to use `use_sim_time:=False`