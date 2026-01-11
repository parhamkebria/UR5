# Running Gazebo in Docker on macOS

Since Gazebo won't build natively on modern macOS due to dependency conflicts, Docker provides a complete working solution with ROS2 + Gazebo.

## ✅ Prerequisites Installation

### 1. Install Docker Desktop

```bash
brew install --cask docker
```

**After installation:**
1. Open Docker Desktop from Applications
2. Wait for it to start (whale icon appears in menu bar)
3. Accept the license agreement

### 2. XQuartz (Already Installed ✓)

XQuartz is already installed on your system and ready for X11 forwarding.

---

## 🚀 Quick Start

### One-Command Launch

```bash
./run_gazebo_docker.sh
```

This script will:
1. ✅ Check Docker is running
2. ✅ Start XQuartz if needed
3. ✅ Configure X11 forwarding
4. ✅ Pull ROS2 + Gazebo image (first time, ~2GB)
5. ✅ Launch interactive container with UR5 files

### What You Get

Inside the container, you have:
- **ROS2 Jazzy** (latest LTS)
- **Gazebo Classic** (ros-gazebo)
- **New Gazebo** (gz-sim)
- **All your UR5 files** mounted at `/workspace`

---

## 📦 Manual Docker Commands

### Launch Gazebo Classic

```bash
# Start container
docker run -it --rm \
  -e DISPLAY=host.docker.internal:0 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $(pwd):/workspace \
  -w /workspace \
  osrf/ros:jazzy-desktop-full \
  bash

# Inside container, source ROS2
source /opt/ros/jazzy/setup.bash

# Launch Gazebo
gazebo
```

### Launch with UR5

```bash
# Inside container
gazebo ur5_world.sdf
```

### Use New Gazebo (gz-sim)

```bash
# Inside container
gz sim ur5_robot.sdf
```

### Spawn UR5 via ROS2

```bash
# Terminal 1: Start Gazebo server
gazebo --verbose

# Terminal 2: Spawn robot
ros2 run gazebo_ros spawn_entity.py \
  -entity ur5 \
  -file ur5_robot.urdf \
  -x 0 -y 0 -z 0.1
```

---

## 🔧 XQuartz Configuration

### First-Time Setup

1. **Start XQuartz:**
   ```bash
   open -a XQuartz
   ```

2. **Configure Security:**
   - Open XQuartz → Preferences → Security
   - ✅ Enable "Allow connections from network clients"
   - Restart XQuartz

3. **Allow localhost:**
   ```bash
   xhost + localhost
   ```

### Troubleshooting Display

If Gazebo window doesn't appear:

```bash
# Check DISPLAY variable
echo $DISPLAY  # Should show :0 or similar

# Restart XQuartz
killall Xquartz
open -a XQuartz
sleep 3
xhost + localhost

# Try again
./run_gazebo_docker.sh
```

---

## 📊 Docker Image Info

### Image Details

- **Name:** `osrf/ros:jazzy-desktop-full`
- **Size:** ~2.5 GB
- **Includes:** ROS2 Jazzy + Gazebo + GUI tools
- **Base:** Ubuntu 22.04

### Managing Images

```bash
# Pull image manually
docker pull osrf/ros:jazzy-desktop-full

# List Docker images
docker images

# Remove image (if needed)
docker rmi osrf/ros:jazzy-desktop-full

# See running containers
docker ps

# Stop container
docker stop ur5_gazebo
```

---

## 💡 Usage Examples

### Example 1: Test Gazebo GUI

```bash
./run_gazebo_docker.sh

# Inside container:
gazebo
# You should see Gazebo GUI with empty world
```

### Example 2: Load UR5 World

```bash
./run_gazebo_docker.sh

# Inside container:
gazebo ur5_world.sdf
# UR5 robot appears with lighting and table
```

### Example 3: Python Control

Create a file `test_ros_control.py`:

```python
import subprocess
import time

# Start gazebo in background
proc = subprocess.Popen(['gazebo', '--verbose'])
time.sleep(5)

# Spawn UR5
subprocess.run([
    'ros2', 'run', 'gazebo_ros', 'spawn_entity.py',
    '-entity', 'ur5',
    '-file', 'ur5_robot.urdf',
    '-x', '0', '-y', '0', '-z', '0.1'
])

# Keep running
proc.wait()
```

```bash
./run_gazebo_docker.sh

# Inside container:
python3 test_ros_control.py
```

---

## ⚡ Performance Tips

### 1. Allocate More Resources

Docker Desktop → Settings → Resources:
- **CPUs:** 4-6 cores
- **Memory:** 6-8 GB
- **Disk:** 40 GB

### 2. Reduce Graphics Quality

```bash
# Inside container, set environment
export LIBGL_ALWAYS_SOFTWARE=1
gazebo
```

### 3. Use Headless Mode

```bash
# Run simulation without GUI (fastest)
gzserver ur5_world.sdf

# In another terminal, query state
gz topic -l
gz topic -e /gazebo/default/pose/info
```

---

## 🐛 Troubleshooting

### Issue: "Cannot connect to X server"

**Solution:**
```bash
# Restart XQuartz
killall Xquartz
open -a XQuartz
sleep 3

# Re-allow connections
xhost + localhost

# Check DISPLAY
echo $DISPLAY
```

### Issue: "Docker daemon not running"

**Solution:**
1. Open Docker Desktop app
2. Wait for whale icon in menu bar
3. Run script again

### Issue: Slow performance

**Solutions:**
1. Close other apps
2. Increase Docker resources (see Performance Tips)
3. Use headless mode: `gzserver` instead of `gazebo`

### Issue: UR5 model not loading

**Solution:**
```bash
# Inside container, check files
ls -la /workspace/*.sdf
ls -la /workspace/*.urdf

# Validate URDF/SDF
check_urdf ur5_robot.urdf
```

---

## 📋 Comparison with Other Options

| Feature | Docker + Gazebo | MuJoCo (mjpython) | Matplotlib |
|---------|----------------|-------------------|------------|
| **Setup Time** | 15 min (first time) | 0 min (working) | 0 min (working) |
| **Graphics Quality** | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐ | ⭐⭐⭐ |
| **Performance** | ⭐⭐ (overhead) | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐ |
| **ROS2 Support** | ✅ Full | ❌ Manual | ❌ No |
| **Complexity** | Medium | Low | Low |
| **Best For** | ROS2 projects | Interactive sim | Figures/papers |

---

## 🎯 When to Use Docker

**Use Docker + Gazebo if:**
- ✅ You need ROS2 integration
- ✅ You need Gazebo-specific plugins
- ✅ You're testing code for Linux deployment
- ✅ You need sensor simulations (camera, LiDAR)
- ✅ You need multi-robot scenarios

**Use MuJoCo if:**
- ✅ You want fast, interactive 3D visualization
- ✅ You don't need ROS2
- ✅ You want better macOS performance
- ✅ You're doing machine learning / control experiments

**Use Matplotlib if:**
- ✅ You need publication-ready figures
- ✅ You want to create animations/GIFs
- ✅ You don't need real-time interaction

---

## 🚀 Next Steps

1. **Test basic Gazebo:**
   ```bash
   ./run_gazebo_docker.sh
   # Then inside: gazebo
   ```

2. **Load UR5:**
   ```bash
   # Inside container:
   gazebo ur5_world.sdf
   ```

3. **Explore ROS2:**
   ```bash
   # Inside container:
   ros2 topic list
   ros2 node list
   ```

4. **Develop your project:**
   - Edit files on macOS (in UR5 folder)
   - Run in Docker (files auto-sync via volume mount)
   - Best of both worlds!

---

## 💾 Persistence

**Note:** The container is ephemeral (`--rm` flag), but:
- ✅ Your files in `/workspace` persist (mounted from macOS)
- ❌ Installed packages inside container are lost on exit

**To persist packages:**

Create a Dockerfile:
```dockerfile
FROM osrf/ros:jazzy-desktop-full

RUN apt-get update && apt-get install -y \
    python3-pip \
    vim \
    && rm -rf /var/lib/apt/lists/*

RUN pip3 install numpy scipy matplotlib

WORKDIR /workspace
```

Build:
```bash
docker build -t ur5-gazebo .
```

Use:
```bash
docker run -it --rm \
  -e DISPLAY=host.docker.internal:0 \
  -v $(pwd):/workspace \
  ur5-gazebo bash
```

---

**For most UR5 work, MuJoCo + Matplotlib is faster and easier. Use Docker when you specifically need ROS2 or Gazebo features!**
