# Running Gazebo in Docker on macOS

Since Gazebo 11 won't build natively on macOS, you can run it in a Docker container.

## Prerequisites

```bash
# Install Docker Desktop
brew install --cask docker

# Install XQuartz (for GUI forwarding)
brew install --cask xquartz
```

## Setup X11 Forwarding

1. **Start XQuartz:**
   ```bash
   open -a XQuartz
   ```

2. **Configure XQuartz:**
   - Go to XQuartz → Preferences → Security
   - Enable "Allow connections from network clients"
   - Restart XQuartz

3. **Allow localhost connections:**
   ```bash
   xhost + localhost
   ```

## Run Gazebo in Docker

### Quick Test

```bash
# Pull the ROS Noetic image (includes Gazebo 11)
docker pull osrf/ros:noetic-desktop-full

# Run Gazebo GUI
docker run -it --rm \
  -e DISPLAY=host.docker.internal:0 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $(pwd):/workspace \
  -w /workspace \
  osrf/ros:noetic-desktop-full \
  gazebo
```

### Run UR5 Simulation

```bash
# Create a Docker helper script
cat > run_gazebo_docker.sh << 'EOF'
#!/bin/bash
docker run -it --rm \
  -e DISPLAY=host.docker.internal:0 \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v $(pwd):/workspace \
  -w /workspace \
  osrf/ros:noetic-desktop-full \
  bash -c "
    apt-get update && apt-get install -y python3-pip
    pip3 install pygazebo protobuf numpy
    python3 ur5_gazebo_simulation.py
  "
EOF

chmod +x run_gazebo_docker.sh
./run_gazebo_docker.sh
```

## Limitations

- **Performance**: Docker adds overhead, may be slower
- **Setup Complexity**: Requires X11 forwarding
- **GPU**: Limited GPU acceleration in Docker on macOS

## Verdict

**For macOS, MuJoCo is better than Docker + Gazebo:**

| Feature | MuJoCo Native | Gazebo Docker |
|---------|---------------|---------------|
| Setup | Easy | Complex |
| Performance | Fast | Slow |
| Graphics | Good | Good |
| Reliability | High | Medium |

**Recommendation:** Stick with `mjpython ur5_simulation.py` on macOS!
