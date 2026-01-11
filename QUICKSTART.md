# UR5 Robot Simulation - Quick Start

## 🚀 Get Running in 2 Minutes

```bash
# Install
pip install mujoco numpy sympy matplotlib

# Run
python UR5.py           # Compute dynamics
python ur5_animation.py # Create visualization
```

Done! Check `ur5_animation.gif` and `ur5_configurations.png`

## ✅ Available Options

### Option 1: Matplotlib Animation ⭐ RECOMMENDED
**Always works, creates visual output files**

```bash
python ur5_animation.py
```

**What it does:**
- ✅ Simulates robot motion through waypoints
- ✅ Creates 3D visualization
- ✅ Saves animation as `ur5_animation.gif`
- ✅ Saves static poses as `ur5_configurations.png`
- ✅ Opens interactive matplotlib window

### Option 2: No-Viewer Mode
**For servers without displays or batch processing**

```bash
python ur5_simulation.py no-viewer
```

**What it does:**
- ✅ Runs physics simulation
- ✅ Prints dynamics (Mass matrix, Jacobian)
- ✅ Shows joint positions and end-effector location
- ❌ No visualization (terminal output only)

### Option 3: Interactive Launcher

```bash
python run_simulation.py
```

**Menu options:**
1. Matplotlib Animation (recommended)
2. MuJoCo Interactive Viewer (may not open window)
3. No Viewer Mode (headless)

## 📁 What Each File Does

| File | Purpose | Output |
|------|---------|--------|
| `UR5.m` | Original MATLAB implementation | Reference |
| `UR5.py` | Compute analytical dynamics | `.pkl`, `.txt` files |
| `ur5_animation.py` | **3D animation ⭐ USE THIS** | `.gif`, `.png` |
| `ur5_simulation.py` | MuJoCo headless simulation | Terminal output |
| `run_simulation.py` | Interactive menu launcher | Varies |
| `ur5_model.xml` | MuJoCo robot model definition | Used by simulations |

## Quick Examples

### Run Complete Demo
```bash
# 1. Compute dynamics
python UR5.py

# 2. Create visualization
python ur5_animation.py

# 3. View outputs
ls -lh *.gif *.png *.txt *.pkl
```

### Extract Dynamics Data
```python
import pickle
import numpy as np

# Load computed dynamics
with open('UR5.pkl', 'rb') as f:
    data = pickle.load(f)
    
T = data['T']        # Forward kinematics
M = data['M']        # Mass matrix
C = data['C']        # Coriolis matrix
G = data['G']        # Gravity vector
J = data['Jacobi']   # Jacobian
```

### Run Simulation Programmatically
```python
from ur5_animation import UR5Animator
import numpy as np

# Create simulator
sim = UR5Animator()

# Define waypoints
waypoints = [
    np.array([0, -np.pi/4, np.pi/2, -np.pi/4, -np.pi/2, 0]),
    np.array([np.pi/4, -np.pi/3, np.pi/3, -np.pi/6, -np.pi/2, 0])
]

# Simulate
trajectory = sim.simulate_trajectory(waypoints)

# Access data
for frame in trajectory:
    print(f"Time: {frame['time']:.3f}, EE: {frame['ee_pos']}")
```

## 🔧 Troubleshooting

| Issue | Solution |
|-------|----------|
| "No display found" | Use `python ur5_animation.py` or add `no-viewer` flag |
| "MuJoCo viewer error" | On **macOS**: use `mjpython ur5_simulation.py` instead of `python` |
| "Module not found" | `pip install mujoco numpy sympy matplotlib pillow` |
| Animation window frozen | Close manually, check for output `.gif`/`.png` files |
| UR5.py takes forever | Normal - symbolic computation takes 30-120 seconds |
| PyBullet won't install | Not needed - use MuJoCo and matplotlib instead |

**💡 macOS MuJoCo Viewer:** Use `mjpython ur5_simulation.py` instead of `python` to enable the interactive 3D viewer.

## 📤 Output Files

After running the tools:

| File | Size | Description |
|------|------|-------------|
| `UR5.pkl` | ~173 KB | Symbolic dynamics (M, C, G, Jacobian) |
| `UR5M.txt` | ~421 KB | Mass matrix (text format) |
| `UR5C.txt` | ~11 MB | Coriolis matrix (large, symbolic) |
| `UR5G.txt` | ~2.7 KB | Gravity vector |
| `UR5J.txt` | ~4.7 KB | Jacobian matrix |
| `UR5T.txt` | ~3.4 KB | Forward kinematics |
| `ur5_animation.gif` | Varies | Animated motion |
| `ur5_configurations.png` | Varies | Static poses |

## 🎯 Decision Guide

**I want to...**

→ **See the robot move** → `python ur5_animation.py`  
→ **Get dynamics equations** → `python UR5.py` then load `UR5.pkl`  
→ **Run without graphics** → `python ur5_simulation.py no-viewer`  
→ **Not sure what to do** → `python run_simulation.py`

## ⚠️ Known Limitations

- MuJoCo interactive viewer doesn't work on all systems (use matplotlib instead)
- PyBullet compilation fails on modern macOS (not needed)
- Coriolis matrix file is very large (~11 MB) due to symbolic complexity
- Symbolic computation in UR5.py takes time (~1-2 minutes)
