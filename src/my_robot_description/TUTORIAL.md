# 📚 Complete Tutorial: Creating a Multi-Segment Robot in ROS 2 Jazzy

**From Zero to Robot Visualization**

This tutorial walks you through creating a Double Pendulum Arm robot from scratch using ROS 2 Jazzy, URDF/Xacro, and RViz.

---

## 📋 Table of Contents

1. [Prerequisites](#1-prerequisites)
2. [Understanding the Concepts](#2-understanding-the-concepts)
3. [Workspace and Package Setup](#3-workspace-and-package-setup)
4. [Creating the Robot Description (URDF)](#4-creating-the-robot-description-urdf)
5. [Creating the Launch File](#5-creating-the-launch-file)
6. [Updating setup.py](#6-updating-setuppy)
7. [Creating RViz Configuration](#7-creating-rviz-configuration)
8. [Building and Running](#8-building-and-running)
9. [Inspecting the ROS 2 System](#9-inspecting-the-ros-2-system)
10. [Relaunching Later](#10-relaunching-later)
11. [Troubleshooting](#11-troubleshooting)

---

## 1. Prerequisites

### 1.1 System Requirements

- **Operating System:** Ubuntu 24.04 LTS (Noble Numbat)
- **ROS 2 Distribution:** Jazzy Jalisco
- **Python:** 3.10 or higher

### 1.2 Install ROS 2 Jazzy

If you haven't installed ROS 2 Jazzy yet, follow the official installation guide:
https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html

### 1.3 Verify ROS 2 Installation

Open a terminal and run:

```bash
source /opt/ros/jazzy/setup.bash
ros2 --version
```

You should see the ROS 2 version number (e.g., `ros2 0.10.x`).

### 1.4 Install Required Packages

```bash
sudo apt update
sudo apt install ros-jazzy-joint-state-publisher-gui ros-jazzy-xacro ros-jazzy-robot-state-publisher
```

**What these packages do:**
- `joint-state-publisher-gui`: Provides a GUI with sliders to control joint angles
- `xacro`: XML macro language for creating modular URDF files
- `robot-state-publisher`: Publishes robot transforms (TF) based on joint states

---

## 2. Understanding the Concepts

### 2.1 What is URDF?

**URDF (Unified Robot Description Format)** is an XML format for representing a robot model. It defines:
- **Links**: The rigid bodies of the robot (e.g., arms, wheels, sensors)
- **Joints**: The connections between links that allow movement

### 2.2 What is Xacro?

**Xacro (XML Macros)** is an extension of URDF that adds:
- Variables and parameters
- Mathematical expressions
- Macros for reusable components
- Conditional statements

### 2.3 Our Robot Structure

We'll build a double pendulum arm:

```
                    ┌─────────┐
                    │  WORLD  │  (Fixed reference frame)
                    └────┬────┘
                         │ (fixed joint)
                    ┌────┴────┐
                    │ SPHERE 1│  (Red - Base)
                    └────┬────┘
                         │ joint_1 (continuous - rotates around Y axis)
                    ┌────┴────┐
                    │CYLINDER1│  (Blue - First arm)
                    └────┬────┘
                         │ (fixed joint)
                    ┌────┴────┐
                    │ SPHERE 2│  (Green - Middle)
                    └────┬────┘
                         │ joint_2 (continuous - rotates around Y axis)
                    ┌────┴────┐
                    │CYLINDER2│  (White - Second arm)
                    └─────────┘
```

### 2.4 Joint Types in URDF

| Type | Description |
|------|-------------|
| `fixed` | No movement allowed |
| `revolute` | Rotates with angle limits |
| `continuous` | Rotates without limits (our choice) |
| `prismatic` | Slides along an axis |
| `floating` | 6 degrees of freedom |
| `planar` | Movement in a plane |

---

## 3. Workspace and Package Setup

### 3.1 Source ROS 2

**⚠️ CRUCIAL:** You must source ROS 2 in EVERY new terminal!

```bash
source /opt/ros/jazzy/setup.bash
```

💡 **Pro Tip:** Add this to your `~/.bashrc` to auto-source:
```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```

### 3.2 Create the Workspace

```bash
# Create workspace directory structure
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

**Understanding the structure:**
- `ros2_ws/` - Your workspace root
- `ros2_ws/src/` - Where your packages live
- `ros2_ws/build/` - Build artifacts (created during build)
- `ros2_ws/install/` - Installed packages (created during build)
- `ros2_ws/log/` - Build logs (created during build)

### 3.3 Create the Package

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_robot_description --dependencies rclpy xacro
```

**Command breakdown:**
- `ros2 pkg create` - Create a new package
- `--build-type ament_python` - Python package (vs. `ament_cmake` for C++)
- `my_robot_description` - Package name
- `--dependencies rclpy xacro` - Package dependencies

### 3.4 Create Organization Folders

```bash
cd my_robot_description
mkdir urdf launch rviz
```

**Folder purposes:**
- `urdf/` - Robot description files (.xacro, .urdf)
- `launch/` - Launch files to start nodes
- `rviz/` - RViz configuration files

---

## 4. Creating the Robot Description (URDF)

### 4.1 Create the Xacro File

Create the file `urdf/robot.urdf.xacro`:

```bash
nano urdf/robot.urdf.xacro
```

### 4.2 Add the Robot Description

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="double_arm_robot">

    <!-- ============================================ -->
    <!-- MATERIAL DEFINITIONS                         -->
    <!-- ============================================ -->
    <!-- Define colors we'll use for visual elements -->
    <material name="blue"><color rgba="0 0 0.8 1"/></material>
    <material name="red"><color rgba="0.8 0 0 1"/></material>
    <material name="green"><color rgba="0 0.8 0 1"/></material>
    <material name="white"><color rgba="1 1 1 1"/></material>

    <!-- ============================================ -->
    <!-- WORLD LINK (Fixed Reference Frame)          -->
    <!-- ============================================ -->
    <!-- The world link is an empty reference frame -->
    <link name="world"/>

    <!-- ============================================ -->
    <!-- SPHERE 1 (Base Joint)                       -->
    <!-- ============================================ -->
    <!-- Fixed joint connecting world to first sphere -->
    <joint name="world_to_sphere1" type="fixed">
        <parent link="world"/>
        <child link="sphere_link_1"/>
        <origin xyz="0 0 0.1" rpy="0 0 0"/>
        <!-- xyz: position relative to parent (10cm up) -->
        <!-- rpy: rotation (roll, pitch, yaw) in radians -->
    </joint>

    <!-- First sphere - the base of our pendulum -->
    <link name="sphere_link_1">
        <visual>
            <geometry><sphere radius="0.025"/></geometry>
            <material name="red"/>
        </visual>
    </link>

    <!-- ============================================ -->
    <!-- CYLINDER 1 (First Arm Segment)              -->
    <!-- ============================================ -->
    <!-- Continuous joint - allows unlimited rotation -->
    <joint name="joint_1" type="continuous">
        <parent link="sphere_link_1"/>
        <child link="cylinder_link_1"/>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <axis xyz="0 1 0"/>
        <!-- Axis: rotation around Y axis -->
    </joint>

    <!-- First arm segment -->
    <link name="cylinder_link_1">
        <visual>
            <geometry><cylinder radius="0.02" length="0.1"/></geometry>
            <!-- Move cylinder so its top is at origin -->
            <origin xyz="0 0 -0.05" rpy="0 0 0"/>
            <material name="blue"/>
        </visual>
    </link>

    <!-- ============================================ -->
    <!-- SPHERE 2 (Middle Joint)                     -->
    <!-- ============================================ -->
    <!-- Fixed joint at the bottom of cylinder 1 -->
    <joint name="cylinder_1_to_sphere_2" type="fixed">
        <parent link="cylinder_link_1"/>
        <child link="sphere_link_2"/>
        <origin xyz="0 0 -0.1" rpy="0 0 0"/>
    </joint>

    <!-- Middle sphere connecting the two arms -->
    <link name="sphere_link_2">
        <visual>
            <geometry><sphere radius="0.025"/></geometry>
            <material name="green"/>
        </visual>
    </link>

    <!-- ============================================ -->
    <!-- CYLINDER 2 (Second Arm Segment)             -->
    <!-- ============================================ -->
    <!-- Second continuous joint -->
    <joint name="joint_2" type="continuous">
        <parent link="sphere_link_2"/>
        <child link="cylinder_link_2"/>
        <origin xyz="0 0 0" rpy="0 0 0"/>
        <axis xyz="0 1 0"/>
    </joint>

    <!-- Second arm segment -->
    <link name="cylinder_link_2">
        <visual>
            <geometry><cylinder radius="0.02" length="0.1"/></geometry>
            <origin xyz="0 0 -0.05" rpy="0 0 0"/>
            <material name="white"/>
        </visual>
    </link>

</robot>
```

### 4.3 Understanding the Coordinate System

```
        Z (up)
        │
        │
        │
        └───────── Y (left)
       /
      /
     X (forward)
```

- **xyz**: Position offset from parent [x, y, z] in meters
- **rpy**: Rotation [roll, pitch, yaw] in radians
  - Roll: rotation around X axis
  - Pitch: rotation around Y axis
  - Yaw: rotation around Z axis

---

## 5. Creating the Launch File

### 5.1 Create the Launch File

Create `launch/display.launch.py`:

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    # Package name and path
    pkg_name = 'my_robot_description'
    pkg_path = get_package_share_directory(pkg_name)

    # Process the Xacro file to get URDF XML
    xacro_file = os.path.join(pkg_path, 'urdf', 'robot.urdf.xacro')
    robot_description_config = xacro.process_file(xacro_file)

    # Path to RViz config file
    rviz_config_path = os.path.join(pkg_path, 'rviz', 'view_robot.rviz')

    return LaunchDescription([
        # Node 1: Robot State Publisher
        # Publishes the robot model and transforms (TF)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description_config.toxml()}]
        ),
        
        # Node 2: Joint State Publisher GUI
        # Provides sliders to control joint angles
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui'
        ),
        
        # Node 3: RViz2
        # 3D visualization tool
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_path] if os.path.exists(rviz_config_path) else [],
            output='screen'
        )
    ])
```

### 5.2 Understanding the Nodes

| Node | Purpose |
|------|---------|
| `robot_state_publisher` | Converts joint states to TF transforms |
| `joint_state_publisher_gui` | GUI to set joint angles |
| `rviz2` | 3D visualization |

---

## 6. Updating setup.py

### 6.1 Modify setup.py

Edit `setup.py` to include our resource files:

```python
import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'my_robot_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Add these lines to include our files:
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Double Pendulum Arm Robot Description Package',
    license='MIT',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [],
    },
)
```

### 6.2 Understanding data_files

The `data_files` list tells Python where to install our files:
- Launch files → `share/my_robot_description/launch/`
- URDF files → `share/my_robot_description/urdf/`
- RViz files → `share/my_robot_description/rviz/`

---

## 7. Creating RViz Configuration

### 7.1 Create the RViz Config

Create `rviz/view_robot.rviz`:

```yaml
Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: rviz_common/Views
    Name: Views
Visualization Manager:
  Class: ""
  Displays:
    - Class: rviz_default_plugins/Grid
      Name: Grid
      Enabled: true
      Value: true
    - Alpha: 1
      Class: rviz_default_plugins/RobotModel
      Description Source: Topic
      Description Topic:
        Depth: 5
        Durability Policy: Volatile
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /robot_description
      Enabled: true
      Name: RobotModel
      Visual Enabled: true
  Global Options:
    Background Color: 48; 48; 48
    Fixed Frame: world
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/Interact
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
  Value: true
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 0.5
      Enable Stereo Rendering:
        Value: false
      Focal Point:
        X: 0
        Y: 0
        Z: 0
      Focal Shape Fixed Size: true
      Focal Shape Size: 0.05
      Invert Z Axis: false
      Name: Current View
      Near Clip Distance: 0.01
      Pitch: 0.5
      Target Frame: <Fixed Frame>
      Value: Orbit (rviz)
      Yaw: 0.5
    Saved: ~
Window Geometry:
  Height: 800
  Width: 1200
  X: 100
  Y: 100
```

### 7.2 Key RViz Settings

| Setting | Value | Purpose |
|---------|-------|---------|
| Fixed Frame | `world` | Reference frame for visualization |
| RobotModel | Enabled | Shows the robot |
| Description Topic | `/robot_description` | Where to get URDF |

---

## 8. Building and Running

### 8.1 Build the Workspace

```bash
# Navigate to workspace root
cd ~/ros2_ws

# Source ROS 2 (if not already done)
source /opt/ros/jazzy/setup.bash

# Build with symlink install (for faster development)
colcon build --symlink-install
```

**Build options explained:**
- `--symlink-install`: Creates symlinks instead of copying files (faster iteration)

### 8.2 Source the Workspace

```bash
source install/setup.bash
```

**⚠️ Important:** You must source after EVERY build!

### 8.3 Launch the Robot

```bash
ros2 launch my_robot_description display.launch.py
```

### 8.4 Expected Result

You should see:
1. **RViz2** window with the robot visualization
2. **Joint State Publisher GUI** popup with sliders
3. The robot: Red sphere → Blue cylinder → Green sphere → White cylinder

Use the sliders to control `joint_1` and `joint_2`!

---

## 9. Inspecting the ROS 2 System

Open a **new terminal** (keep the robot running) and source the workspace:

```bash
source ~/ros2_ws/install/setup.bash
```

### 9.1 List Active Nodes

```bash
ros2 node list
```

**Expected output:**
```
/robot_state_publisher
/joint_state_publisher_gui
/rviz2
```

### 9.2 List Active Topics

```bash
ros2 topic list
```

**Key topics:**
- `/joint_states` - Real-time joint angles
- `/robot_description` - The URDF XML
- `/tf` - Coordinate transforms
- `/tf_static` - Static transforms

### 9.3 Monitor Joint States

```bash
ros2 topic echo /joint_states
```

Move the sliders and watch the values change!

**Example output:**
```yaml
header:
  stamp:
    sec: 1234567890
    nanosec: 123456789
  frame_id: ''
name:
- joint_1
- joint_2
position:
- 0.5
- -0.3
velocity: []
effort: []
```

### 9.4 View the TF Tree

```bash
ros2 run tf2_tools view_frames
```

This generates a PDF showing the transform tree.

---

## 10. Relaunching Later

After closing terminals or restarting your computer:

```bash
# 1. Navigate to workspace
cd ~/ros2_ws

# 2. Source ROS 2
source /opt/ros/jazzy/setup.bash

# 3. Source your workspace
source install/setup.bash

# 4. Launch
ros2 launch my_robot_description display.launch.py
```

**Note:** You only need to rebuild (`colcon build`) if you:
- Add new files
- Modify `setup.py` or `package.xml`
- Change non-symlinked files

---

## 11. Troubleshooting

### Problem: "Package not found"

**Solution:** Make sure you've sourced both ROS 2 and your workspace:
```bash
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash
```

### Problem: Robot doesn't appear in RViz

**Solutions:**
1. Check **Fixed Frame** is set to `world`
2. Click **Add** → **RobotModel**
3. Set **Description Topic** to `/robot_description`

### Problem: "command not found: ros2"

**Solution:** Source ROS 2:
```bash
source /opt/ros/jazzy/setup.bash
```

### Problem: Build fails

**Solutions:**
1. Install missing dependencies:
   ```bash
   rosdep install --from-paths src --ignore-src -r -y
   ```
2. Check for syntax errors in Python/XML files

### Problem: Changes not reflected

**Solution:** Rebuild and re-source:
```bash
colcon build --symlink-install
source install/setup.bash
```

---

## 🎉 Congratulations!

You've successfully created a multi-segment robot in ROS 2 Jazzy! 

**What you learned:**
- ✅ ROS 2 workspace structure
- ✅ Creating Python packages
- ✅ URDF/Xacro robot description
- ✅ Launch files
- ✅ RViz visualization
- ✅ Joint control with GUI

**Next steps:**
- Add collision geometry for simulation
- Add inertial properties for physics
- Create custom controllers
- Integrate with Gazebo simulation

---

*Happy Robot Building! 🤖*
