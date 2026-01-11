#!/bin/bash
#
# Run Gazebo with UR5 robot in Docker
# This script sets up X11 forwarding and launches Gazebo in a container
#

set -e

echo "=========================================="
echo "  UR5 Gazebo Docker Launcher"
echo "=========================================="
echo ""

# Check if Docker is running
if ! docker info > /dev/null 2>&1; then
    echo "❌ Error: Docker is not running!"
    echo ""
    echo "Please start Docker Desktop:"
    echo "  1. Open Applications folder"
    echo "  2. Launch Docker.app"
    echo "  3. Wait for Docker to start (whale icon in menu bar)"
    echo "  4. Run this script again"
    exit 1
fi

# Check if XQuartz is running
if ! pgrep -x "Xquartz" > /dev/null; then
    echo "⚠️  XQuartz is not running. Starting it now..."
    open -a XQuartz
    echo "   Waiting for XQuartz to start..."
    sleep 3
fi

# Allow localhost connections
echo "🔧 Configuring X11 forwarding..."
xhost + localhost > /dev/null 2>&1 || true

# Set display
export DISPLAY=host.docker.internal:0

echo "✅ Setup complete!"
echo ""
echo "🐳 Launching Gazebo in Docker..."
echo "   This will:"
echo "   - Pull ROS2 + Gazebo image (first time only, ~2GB)"
echo "   - Start Gazebo with the UR5 robot"
echo "   - Enable GUI via X11 forwarding"
echo ""

# Run Docker container with Gazebo
docker run -it --rm \
  --name ur5_gazebo \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v "$(pwd)":/workspace \
  -w /workspace \
  osrf/ros:jazzy-desktop-full \
  bash -c "
    echo '=========================================='
    echo 'ROS2 + Gazebo Environment'
    echo '=========================================='
    echo ''
    echo 'Available commands:'
    echo '  gazebo                  - Launch empty Gazebo world'
    echo '  gazebo ur5_world.sdf    - Launch with UR5 world'
    echo '  ros2 launch gazebo_ros gazebo.launch.py - ROS2 Gazebo launch'
    echo '  gz sim ur5_robot.sdf    - Launch UR5 with new Gazebo (gz)'
    echo ''
    echo 'Your UR5 files are in /workspace'
    echo 'Press Ctrl+D or type exit to quit'
    echo ''
    
    # Source ROS2
    source /opt/ros/jazzy/setup.bash
    
    # Drop into interactive shell
    bash
"

echo ""
echo "👋 Gazebo session ended"
