# Installing New Gazebo (Jetty) on macOS from Source

This guide shows how to install **Gazebo Jetty** (the modern Gazebo, not deprecated Classic 11) on macOS from source.

> ✅ **This works on macOS Ventura, Sonoma, and Sequoia**  
> 🎯 **Recommended for serious robotics work on macOS**  
> ⚠️ **Takes ~1-2 hours to compile**

---

## Why New Gazebo vs Gazebo Classic?

| Feature | Gazebo Jetty (New) | Gazebo Classic 11 |
|---------|-------------------|-------------------|
| macOS Support | ✅ Yes (from source) | ❌ Deprecated |
| Performance | ⭐⭐⭐⭐⭐ | ⭐⭐⭐ |
| Graphics | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐ |
| Active Development | ✅ Yes | ❌ End-of-life |
| Python Bindings | ✅ Modern | ⚠️ pygazebo |

---

## Prerequisites

### 1. Install Xcode Command Line Tools

```bash
xcode-select --install
```

Requires **Xcode 14+** on macOS Ventura or later.

### 2. Install Homebrew

```bash
# If not already installed
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
```

### 3. Add OSRF Tap

```bash
brew tap osrf/simulation
brew update
```

---

## Installation Steps

### Step 1: Install Dependencies

```bash
# Install XQuartz (required for Ogre rendering)
brew install --cask xquartz

# Install all build dependencies
brew install assimp boost bullet cli11 cmake cppzmq dartsim doxygen \
  eigen fcl ffmpeg flann freeimage freetype gdal gflags google-benchmark \
  ipopt jsoncpp libccd libyaml libzzip libzip nlopt ode open-scene-graph \
  ossp-uuid ogre1.9 ogre2.3 pkg-config protobuf qt@6 qwt rapidjson ruby \
  tbb tinyxml2 urdfdom zeromq python3
```

**This will take 15-30 minutes.**

### Step 2: Set Up Python Tools

```bash
# Create virtual environment for build tools
python3 -m venv $HOME/vcs_colcon_installation
source $HOME/vcs_colcon_installation/bin/activate

# Install vcstool and colcon
pip3 install vcstool colcon-common-extensions
```

### Step 3: Get Gazebo Sources

```bash
# Create workspace
mkdir -p ~/workspace/src
cd ~/workspace/src

# Download collection file
curl -OL https://raw.githubusercontent.com/gazebo-tooling/gazebodistro/master/collection-jetty.yaml

# Clone all repositories
vcs import < collection-jetty.yaml
```

### Step 4: Build Gazebo

```bash
cd ~/workspace/

# Check what will be built
colcon graph

# Build everything (takes 30-60 minutes)
# For Apple Silicon (M1/M2/M3):
colcon build --cmake-args \
  -DCMAKE_MACOSX_RPATH=FALSE \
  -DCMAKE_INSTALL_NAME_DIR=$(pwd)/install/lib \
  -DBUILD_TESTING=OFF \
  --merge-install

# For Intel Macs:
colcon build --cmake-args -DBUILD_TESTING=OFF --merge-install
```

**Compilation time:**
- Apple Silicon: ~30-45 minutes
- Intel Mac: ~45-90 minutes

### Step 5: Set Up Environment

Add to your `~/.zshrc` (or `~/.bashrc`):

```bash
# Gazebo Jetty
source $HOME/workspace/install/setup.zsh

# Add to PATH
export PATH="$HOME/workspace/install/bin:$PATH"
```

Reload shell:
```bash
source ~/.zshrc
```

---

## Testing Installation

```bash
# Test Gazebo version
gz sim --version

# Launch empty world (GUI)
gz sim shapes.sdf

# Launch headless
gz sim shapes.sdf -s

# Launch just GUI (connect to running server)
gz sim -g
```

---

## Using with UR5

The new Gazebo uses SDF format (not URDF directly), but can convert:

```bash
# Convert URDF to SDF
gz sdf -p ur5_robot.urdf > ur5_robot.sdf

# Launch with UR5
gz sim ur5_robot.sdf
```

---

## Python Integration

The new Gazebo has different Python bindings. We'll need to adapt the script:

```python
# Old way (pygazebo - for Classic):
from pygazebo import Manager

# New way (gz-python for Jetty):
from gz.msgs10.model_pb2 import Model
from gz.transport13 import Node
```

---

## Troubleshooting

### Build Errors

**"Cannot find Qt6"**
```bash
export CMAKE_PREFIX_PATH=/opt/homebrew/opt/qt@6:$CMAKE_PREFIX_PATH
colcon build --merge-install
```

**"Ogre not found"**
```bash
brew reinstall ogre2.3
```

### Runtime Errors

**"Display not found"**
```bash
# Make sure XQuartz is running
open -a XQuartz

# Set DISPLAY
export DISPLAY=:0
```

**"Library not loaded"**
```bash
# Re-source the workspace
source ~/workspace/install/setup.zsh
```

---

## Comparison: Build Options

| Method | Time | Difficulty | Best For |
|--------|------|------------|----------|
| **Source (This)** | 1-2 hrs | Medium | Latest features |
| **MuJoCo** | 5 min | Easy | Quick start |
| **Matplotlib** | 1 min | Easiest | Visualization |
| **Docker** | 30 min | Hard | Classic Gazebo |

---

## Daily Usage

After installation, just source the workspace in each new terminal:

```bash
source ~/workspace/install/setup.zsh

# Now you can use Gazebo
gz sim shapes.sdf
```

Or add the source line to your `~/.zshrc` to make it automatic.

---

## Next Steps

1. **Verify installation**: `gz sim --version`
2. **Test GUI**: `gz sim shapes.sdf`
3. **Load UR5**: We'll need to adapt the Python script for new Gazebo API
4. **Read docs**: https://gazebosim.org/docs/latest/getstarted/

---

## Should You Do This?

**Yes, if:**
- ✅ You need latest Gazebo features
- ✅ You're doing serious robotics research
- ✅ You want ROS 2 integration
- ✅ You have 1-2 hours for compilation

**No, if:**
- ❌ You want something quick
- ❌ You just need visualization → Use `python ur5_animation.py`
- ❌ You want interactive 3D → Use `mjpython ur5_simulation.py`

For most users, **MuJoCo + Matplotlib is faster and easier**. But if you need full Gazebo, this guide works!
