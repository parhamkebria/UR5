#!/bin/bash
set -e

echo "🚀 Starting Gazebo with X11 forwarding..."
echo ""

# 1. Configure XQuartz for network connections
echo "1️⃣ Configuring XQuartz..."
defaults write org.xquartz.X11 nolisten_tcp -bool false
defaults write org.xquartz.X11 no_auth -bool false

# 2. Restart XQuartz
echo "2️⃣ Restarting XQuartz..."
killall XQuartz 2>/dev/null || true
open -a XQuartz
echo "   Waiting for XQuartz to start..."
sleep 8

# 3. Get Mac IP address
echo "3️⃣ Getting Mac IP address..."
IP=$(ifconfig en0 | grep "inet " | awk '{print $2}')
if [ -z "$IP" ]; then
    echo "❌ Could not find IP on en0, trying en1..."
    IP=$(ifconfig en1 | grep "inet " | awk '{print $2}')
fi

if [ -z "$IP" ]; then
    echo "❌ Could not find IP address"
    exit 1
fi

echo "   Your Mac IP: $IP"

# 4. Allow X11 connections
echo "4️⃣ Configuring X11 access control..."
export DISPLAY=:0
xhost + $IP
xhost + localhost

# 5. Stop any running container
echo "5️⃣ Cleaning up old containers..."
docker stop ur5_gazebo 2>/dev/null || true
docker rm ur5_gazebo 2>/dev/null || true

# 6. Start container
echo "6️⃣ Starting Docker container..."
echo "   DISPLAY=$IP:0"
echo ""

docker run -it --rm \
  --name ur5_gazebo \
  -e DISPLAY=$IP:0 \
  -v /Users/parhamkebria/UR5:/root/ur5_ws \
  osrf/ros:jazzy-desktop-full \
  bash -c "
    echo '🔧 Installing Gazebo...'
    apt-get update -qq && apt-get install -y -qq ros-jazzy-ros-gz ros-jazzy-gz-sim-vendor > /dev/null 2>&1
    
    echo '✅ Gazebo installed!'
    echo ''
    echo '📦 Available commands:'
    echo '  gz sim                        - Launch empty Gazebo world'
    echo '  gz sim Gazebo/ur5_world.sdf   - Launch UR5 robot'
    echo '  ros2 topic list               - List ROS2 topics'
    echo ''
    
    # Source ROS2 environment
    source /opt/ros/jazzy/setup.bash
    cd /root/ur5_ws
    
    # Start bash
    exec bash
  "
