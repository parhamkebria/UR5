#!/bin/bash
set -e

echo "🚀 Starting Gazebo with VNC (better graphics support)..."
echo ""

# Stop any running container
docker stop ur5_gazebo_vnc 2>/dev/null || true
docker rm ur5_gazebo_vnc 2>/dev/null || true

# Start container with VNC
echo "Starting container with VNC server..."
docker run -d --rm \
  --name ur5_gazebo_vnc \
  -p 5900:5900 \
  -p 6080:6080 \
  -v /Users/parhamkebria/UR5:/root/ur5_ws \
  --shm-size=512m \
  osrf/ros:jazzy-desktop-full \
  bash -c "
    apt-get update -qq
    apt-get install -y -qq \
      ros-jazzy-ros-gz \
      ros-jazzy-gz-sim-vendor \
      x11vnc \
      xvfb \
      fluxbox \
      websockify \
      mesa-utils \
      libgl1-mesa-dri
    
    # Start X virtual framebuffer
    Xvfb :99 -screen 0 1920x1080x24 &
    export DISPLAY=:99
    sleep 2
    
    # Start window manager
    fluxbox &
    sleep 1
    
    # Start VNC server
    x11vnc -display :99 -forever -shared -rfbport 5900 &
    
    # Start noVNC web interface
    /usr/share/novnc/utils/novnc_proxy --vnc localhost:5900 --listen 6080 &
    
    # Keep container running
    tail -f /dev/null
  "

echo ""
echo "✅ Container started!"
echo ""
echo "🌐 Access Gazebo in your browser:"
echo "   http://localhost:6080/vnc.html"
echo ""
echo "📱 Or use VNC client:"
echo "   vnc://localhost:5900"
echo ""
echo "To run Gazebo, open another terminal and run:"
echo "   docker exec -it ur5_gazebo_vnc bash"
echo "   source /opt/ros/jazzy/setup.bash"
echo "   gz sim Gazebo/ur5_world.sdf"
echo ""
