# UR5 Gazebo Simulation Setup (Linux)

**Complete guide for running realistic UR5 robot simulation in Gazebo without ROS.**

> 🐧 **Linux Only**: This guide is for Ubuntu/Debian Linux systems. Gazebo 11 is deprecated on macOS.  
> 🍎 **macOS Users**: Use `mjpython ur5_simulation.py` for interactive 3D visualization.  
> 🪟 **Windows Users**: Use WSL2 + Linux instructions, or stick with Matplotlib.

---

## 🎯 Why Gazebo? (For Linux Users)

- **Industry Standard**: Used by NASA, Boston Dynamics, and robotics labs worldwide
- **Realistic Physics**: High-fidelity collision detection, friction, and dynamics
- **Beautiful Rendering**: Professional lighting, shadows, and materials
- **ROS Integration**: Optional ROS support for advanced robotics workflows
- **Sensor Simulation**: Cameras, LiDAR, IMU, force/torque sensors
- **Community**: Largest robotics simulation community

**For macOS/Windows users**: MuJoCo and Matplotlib are excellent alternatives that are easier to set up.

## 📋 Prerequisites

### Ubuntu/Debian Linux (Recommended)

```bash
# Install Gazebo Classic 11
sudo apt update
sudo apt install gazebo11 libgazebo11-dev

# Verify installation
gazebo --version  # Should show version 11.x.x

# Install Python dependencies
pip install pygazebo protobuf numpy
```

**Tested on:**
- Ubuntu 20.04 LTS (Focal)
- Ubuntu 22.04 LTS (Jammy)
- Debian 11 (Bullseye)

### macOS

**⚠️ Gazebo 11 is deprecated and won't build on macOS 14+**

**Use these alternatives instead:**

1. **MuJoCo Interactive Viewer (Recommended for macOS):**
   ```bash
   pip install mujoco numpy
   mjpython ur5_simulation.py
   ```
   - Fast, interactive 3D graphics
   - Better macOS integration
   - Already installed if you have MuJoCo

2. **Matplotlib Animation (Always works):**
   ```bash
   pip install matplotlib pillow numpy
   python ur5_animation.py
   ```
   - Creates GIFs and images
   - No system dependencies
   - Perfect for presentations

3. **Docker (If you really need Gazebo):**
   ```bash
   # Requires Docker Desktop and XQuartz
   docker pull osrf/ros:noetic-desktop-full
   
   # Run Gazebo in container with display forwarding
   xhost + localhost
   docker run -it --rm \
     -e DISPLAY=host.docker.internal:0 \
     -v $(pwd):/workspace \
     osrf/ros:noetic-desktop-full \
     gazebo
   ```

### Windows

Gazebo Classic doesn't officially support Windows. **Recommended options:**

1. **WSL2** (Windows Subsystem for Linux) - Best option:
   ```bash
   # In WSL2 Ubuntu
   sudo apt install gazebo11 libgazebo11-dev
   pip install pygazebo protobuf numpy
   ```

2. **Docker** with Linux container

3. **Use MuJoCo/Matplotlib** (no setup needed):
   ```bash
   python ur5_animation.py  # Works on all platforms
   ```

## 🚀 Quick Start (Linux)

```bash
# 1. Install Gazebo (one-time setup)
sudo apt install gazebo11 libgazebo11-dev

# 2. Install Python packages
pip install pygazebo protobuf numpy

# 3. Run the simulation
python ur5_gazebo_simulation.py

# The script will:
# - Start Gazebo with lighting and environment
# - Spawn the UR5 robot model
# - Move through predefined waypoints
# - Display real-time joint states
```

**What you'll see:**
- Professional 3D environment with realistic lighting
- UR5 robot with color-coded links
- Smooth motion through trajectory
- Real-time physics simulation
- Interactive camera controls (click and drag)

## 📁 Required Files

| File | Description |
|------|-------------|
| `ur5_robot.urdf` | Robot model (URDF format) |
| `ur5_world.sdf` | Simulation environment (lighting, ground, table) |
| `ur5_gazebo_simulation.py` | Python control script |

## 🎮 Features

### Realistic Physics
- Accurate mass and inertia properties
- Joint limits and dynamics
- Collision detection
- Gravity and friction

### Beautiful Visualization
- Directional sun lighting with shadows
- Multiple point lights for illumination
- Color-coded robot links (Blue/Grey/Orange)
- Wooden work table
- Grid ground plane

### Python Control (No ROS!)
- Joint position control
- State monitoring (positions, velocities)
- Trajectory execution
- Async/await interface

## 🔧 How It Works

### Architecture

```
┌─────────────────────────────────────────────┐
│        Gazebo Simulation Server             │
│  (Physics engine, rendering, sensors)       │
└─────────────┬───────────────────────────────┘
              │ TCP/IP
              │ Protobuf messages
┌─────────────▼───────────────────────────────┐
│         PyGazebo (Python Client)            │
│  - Connects to Gazebo without ROS           │
│  - Sends joint commands                     │
│  - Receives robot state                     │
└─────────────┬───────────────────────────────┘
              │
┌─────────────▼───────────────────────────────┐
│    ur5_gazebo_simulation.py (Your Code)     │
│  - Define waypoints                         │
│  - Control robot motion                     │
│  - Monitor state                            │
└─────────────────────────────────────────────┘
```

### Message Flow

1. **Startup**:
   - Script starts Gazebo server
   - Spawns UR5 robot from URDF
   - Connects via PyGazebo

2. **Control Loop**:
   - Python sends joint position commands
   - Gazebo simulates physics
   - PyGazebo receives state updates
   - Display updated in real-time

## 🎨 Customization

### Modify Robot Appearance

Edit `ur5_robot.urdf`:

```xml
<material name="custom_color">
  <color rgba="1.0 0.0 0.0 1"/>  <!-- Red -->
</material>
```

### Change Environment

Edit `ur5_world.sdf`:

```xml
<!-- Add a box obstacle -->
<model name="obstacle">
  <static>true</static>
  <pose>0.5 0 0.5 0 0 0</pose>
  <link name="link">
    <visual name="visual">
      <geometry>
        <box>
          <size>0.2 0.2 0.2</size>
        </box>
      </geometry>
    </visual>
  </link>
</model>
```

### Control Different Trajectories

Modify `ur5_gazebo_simulation.py`:

```python
# Define custom waypoints
waypoints = [
    np.array([0, 0, 0, 0, 0, 0]),           # Home
    np.array([1.57, 0, 0, 0, 0, 0]),        # 90° shoulder pan
    np.array([1.57, -1.57, 0, 0, 0, 0]),    # Lift shoulder
    # Add more waypoints...
]

await sim.set_joint_positions(waypoint, duration=2.0)
```

## 🐛 Troubleshooting

### "Gazebo not found"

**Linux:**
```bash
sudo apt install gazebo11
which gazebo  # Verify installation
```

**macOS:**
```bash
brew install gazebo11
gazebo --version  # Check version
```

### "Could not spawn robot"

1. Check URDF file exists: `ls -l ur5_robot.urdf`
2. Validate URDF syntax:
   ```bash
   check_urdf ur5_robot.urdf  # If available
   ```
3. Try manual spawn:
   ```bash
   gazebo ur5_world.sdf &
   gz model -f ur5_robot.urdf -m ur5
   ```

### "Connection refused" / PyGazebo errors

1. Verify Gazebo is running:
   ```bash
   ps aux | grep gazebo
   ```

2. Check Gazebo master:
   ```bash
   gz topic -l  # Should list topics
   ```

3. Reinstall pygazebo:
   ```bash
   pip uninstall pygazebo
   pip install pygazebo
   ```

### Robot doesn't move

1. Check joint names match in URDF and Python script
2. Verify joint controllers are active
3. Increase force/effort in joint commands
4. Check terminal for error messages

### Poor performance / lag

1. Reduce physics update rate in `ur5_world.sdf`:
   ```xml
   <real_time_update_rate>500</real_time_update_rate>
   ```

2. Disable shadows:
   ```xml
   <shadows>false</shadows>
   ```

3. Close other heavy applications

## 📊 Comparison: Gazebo vs MuJoCo vs Matplotlib

| Feature | Gazebo (Linux) | MuJoCo (All) | Matplotlib (All) |
|---------|----------------|--------------|------------------|
| **Platforms** | Linux only | macOS, Linux, Windows | All platforms |
| **Realism** | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐ | ⭐⭐ |
| **Speed** | ⭐⭐⭐ | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐ |
| **Graphics** | ⭐⭐⭐⭐⭐ | ⭐⭐⭐ | ⭐⭐⭐ |
| **Setup Difficulty** | ⭐⭐ (Linux), ❌ (macOS) | ⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ |
| **Lighting/Shadows** | ✅ Professional | ❌ Basic | ❌ No |
| **ROS Integration** | ✅ Excellent | ⚠️ Manual | ❌ No |
| **Real-time Control** | ✅ Yes | ✅ Yes | ❌ No |
| **Community** | ⭐⭐⭐⭐⭐ Largest | ⭐⭐⭐⭐ Growing | ⭐⭐⭐ Mature |
| **Best For** | Research, ROS projects | ML, fast sim | Quick viz |

**When to use Gazebo (Linux):**
- ✅ Professional presentations and demos
- ✅ ROS-based robotics projects
- ✅ Multi-robot scenarios
- ✅ Sensor simulation (cameras, LiDAR)
- ✅ Publishing research papers
- ✅ Industry-standard validation

**When to use alternatives:**
- 🍎 **macOS**: Use MuJoCo (`mjpython ur5_simulation.py`)
- 🚀 **Speed**: Use MuJoCo (10x faster for learning)
- 📊 **Figures**: Use Matplotlib (publication-ready)
- 🪟 **Windows**: Use Matplotlib or WSL2

## 🎓 Next Steps

### Add Sensors

```xml
<!-- In URDF, add camera to end effector -->
<gazebo reference="wrist_3_link">
  <sensor type="camera" name="camera1">
    <update_rate>30.0</update_rate>
    <camera name="head">
      <horizontal_fov>1.3962634</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
      </image>
    </camera>
  </sensor>
</gazebo>
```

### Integrate with Planning Libraries

```python
# Use MoveIt! (with ROS)
# Or implement custom inverse kinematics:
from UR5 import UR5Kinematics  # Our computed dynamics

ik = UR5Kinematics()
joint_angles = ik.inverse(target_position)
await sim.set_joint_positions(joint_angles)
```

### Record Data

```python
# Log joint states and end-effector positions
trajectory_log = []

for waypoint in waypoints:
    await sim.set_joint_positions(waypoint)
    trajectory_log.append({
        'time': time.time(),
        'joints': sim.current_positions.copy(),
        'ee_pos': compute_forward_kinematics(sim.current_positions)
    })

# Save to file
np.save('trajectory.npy', trajectory_log)
```

## 📚 Resources

- **Gazebo Tutorials**: http://gazebosim.org/tutorials
- **URDF Documentation**: http://wiki.ros.org/urdf
- **PyGazebo GitHub**: https://github.com/jpieper/pygazebo
- **UR5 Documentation**: https://www.universal-robots.com/

## 🆘 Getting Help

If you encounter issues:

1. Check Gazebo logs: `~/.gazebo/server-*.log`
2. Test basic Gazebo: `gazebo` (should open GUI)
3. Verify URDF: `gz sdf -p ur5_robot.urdf`
4. Run dependency check in script: `python ur5_gazebo_simulation.py`

---

## 🆘 Platform-Specific Help

### Linux Users

**Issue: "Gazebo not found"**
```bash
# Check if Gazebo is in PATH
which gazebo

# If not found, reinstall
sudo apt update
sudo apt install gazebo11 libgazebo11-dev

# Verify
gazebo --version
```

**Issue: "Could not spawn robot"**
```bash
# Validate URDF
check_urdf ur5_robot.urdf  # Install: sudo apt install liburdfdom-tools

# Manual spawn test
gazebo ur5_world.sdf &
sleep 5
gz model -f ur5_robot.urdf -m ur5
```

**Issue: Poor performance**
- Close other applications
- Reduce physics rate in `ur5_world.sdf`
- Disable shadows: Edit world file, set `<shadows>false</shadows>`

### macOS Users

**You cannot use Gazebo 11 on modern macOS. Use these instead:**

**Best option - MuJoCo:**
```bash
pip install mujoco numpy
mjpython ur5_simulation.py
```

**Easiest option - Matplotlib:**
```bash
pip install matplotlib numpy
python ur5_animation.py
```

**Advanced option - Docker:**
```bash
# Install Docker Desktop and XQuartz first
brew install --cask xquartz docker

# Start XQuartz and allow connections
xhost + localhost

# Run Gazebo in Docker
docker run -it --rm \
  -e DISPLAY=host.docker.internal:0 \
  -v $(pwd):/workspace \
  -w /workspace \
  osrf/ros:noetic-desktop-full \
  bash -c "source /opt/ros/noetic/setup.bash && python3 ur5_gazebo_simulation.py"
```

### Windows Users

**Option 1: WSL2 (Best for Gazebo)**
```bash
# In WSL2 Ubuntu
sudo apt install gazebo11 libgazebo11-dev
pip install pygazebo protobuf numpy
python ur5_gazebo_simulation.py
```

**Option 2: Native Windows (No Gazebo)**
```bash
# Use Matplotlib instead
pip install matplotlib numpy
python ur5_animation.py
```

---

**Still stuck?** 

**For any platform**, the matplotlib animation always works:
```bash
pip install matplotlib numpy sympy
python UR5.py  # Compute dynamics
python ur5_animation.py  # Create visualization
```

This creates `ur5_animation.gif` with beautiful 3D visualization - perfect for papers and presentations!
