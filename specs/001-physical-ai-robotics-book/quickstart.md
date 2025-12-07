## Quickstart Guide: Physical AI & Humanoid Robotics

This guide provides a concise, step-by-step approach to get your development environment set up and run a foundational example from the book.

### 1. Prerequisites

*   Operating System: Ubuntu 20.04+ (recommended)
*   Python 3.11
*   NVIDIA GPU with Compute Capability 6.0+ (for Isaac Sim)
*   At least 8GB RAM (16GB+ recommended)

### 2. Environment Setup

#### Install ROS 2 Humble Hawksbill:
```bash
# Set up locale
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Set up sources
sudo apt update && sudo apt install curl gnupg
curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key | sudo apt-key add -
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2
sudo apt update
sudo apt install ros-humble-desktop
sudo apt install python3-colcon-common-extensions python3-rosdep python3-vcstool
sudo rosdep init
rosdep update

# Source the setup script
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

#### Install Gazebo:
```bash
sudo apt install gazebo
sudo apt install ros-humble-gazebo-ros-pkgs ros-humble-gazebo-plugins
```

#### Install Isaac Sim (if you have an NVIDIA GPU):
```bash
# Install Python 3.10 (required for Isaac Sim)
sudo apt install python3.10 python3.10-venv python3.10-dev

# Create virtual environment
python3.10 -m venv isaac_venv
source isaac_venv/bin/activate

# Install Isaac Sim
pip install omni-isaac-gym-py
```

### 3. First Robotic Simulation

#### Using ROS 2 and Gazebo:
1. Create a workspace:
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

2. Create a simple robot URDF file (`~/ros2_ws/src/my_robot.urdf`):
```xml
<?xml version="1.0"?>
<robot name="simple_robot">
  <link name="base_link">
    <visual>
      <geometry>
        <cylinder length="0.2" radius="0.2"/>
      </geometry>
      <material name="blue">
        <color rgba="0 0 1 0.8"/>
      </material>
    </visual>
    <collision>
      <geometry>
        <cylinder length="0.2" radius="0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="1.0"/>
      <inertia ixx="0.1" ixy="0.0" ixz="0.0" iyy="0.1" iyz="0.0" izz="0.1"/>
    </inertial>
  </link>
</robot>
```

3. Launch Gazebo with your robot:
```bash
cd ~/ros2_ws
# Source ROS 2
source /opt/ros/humble/setup.bash

# Launch Gazebo
gz sim -r -v 4 --render-engine ogre ~/ros2_ws/src/my_robot.urdf
```

### 4. Basic Code Example

Create a simple Python script to control a robot in simulation:

```python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class SimpleController(Node):
    def __init__(self):
        super().__init__('simple_controller')
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)

    def timer_callback(self):
        msg = Twist()
        msg.linear.x = 0.5  # Move forward at 0.5 m/s
        msg.angular.z = 0.2  # Turn at 0.2 rad/s
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    controller = SimpleController()
    rclpy.spin(controller)
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 5. Verification Steps

1. Check that ROS 2 is properly installed:
```bash
ros2 topic list
```

2. Verify Gazebo runs correctly:
```bash
gz sim --version
```

3. Test basic ROS 2 communication:
```bash
# Terminal 1
ros2 topic echo /chatter std_msgs/String

# Terminal 2
ros2 topic pub /chatter std_msgs/String "data: Hello from quickstart"
```

### Next Steps

After completing this quickstart:
1. Read Module 1 to understand Physical AI and embodied intelligence concepts
2. Work through Module 2 to master ROS 2 and simulation environments
3. Explore the examples in the `docs/module*/examples/` directories
4. Progress to more advanced topics in the book as your understanding grows