# Quick Start: Gazebo in Docker

**Complete setup in 3 steps!**

## Step 1: Start Docker Desktop

```bash
open -a Docker
```

Wait for the whale icon to appear in your menu bar (~30 seconds).

## Step 2: Configure XQuartz

```bash
open -a XQuartz
# Wait 3 seconds, then:
xhost + localhost
```

## Step 3: Launch Gazebo

```bash
cd ~/UR5
./run_gazebo_docker.sh
```

---

## What Happens Next

1. **First time:** Docker downloads ROS2+Gazebo image (~2.5GB, 5-10 min)
2. **Container starts:** You'll see a bash prompt
3. **Run Gazebo:** Type `gazebo` and press Enter
4. **GUI appears:** Gazebo window opens via X11

---

## Quick Commands (Inside Container)

```bash
# Test empty world
gazebo

# Load UR5
gazebo ur5_world.sdf

# New Gazebo
gz sim ur5_robot.sdf

# ROS2 topics
source /opt/ros/jazzy/setup.bash
ros2 topic list

# Exit
exit  # or Ctrl+D
```

---

## Troubleshooting

**"Cannot connect to Docker daemon"**
→ Open Docker Desktop app and wait for it to start

**"Cannot open display"**
→ Run: `open -a XQuartz && sleep 3 && xhost + localhost`

**Slow performance**
→ Docker Desktop → Settings → Resources → Increase CPUs/Memory

---

## Full Documentation

See [DOCKER_GAZEBO.md](DOCKER_GAZEBO.md) for complete guide.
