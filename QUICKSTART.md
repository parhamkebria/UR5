# UR5 Robot Simulation - Quick Start

## The Problem with 3D Viewers

MuJoCo's interactive viewer requires a display environment and may not work in all setups (SSH sessions, headless servers, some macOS configurations). 

## ✅ Working Solutions

### Option 1: Matplotlib Animation (RECOMMENDED)
**Always works, creates visual output files**

```bash
python ur5_animation.py
```

This will:
- ✅ Simulate robot motion through waypoints
- ✅ Create 3D visualization
- ✅ Save animation as `ur5_animation.gif`
- ✅ Save static configurations as `ur5_configurations.png`
- ✅ Display interactive matplotlib window

### Option 2: No-Viewer Mode (Headless)
**For servers without displays**

```bash
python ur5_simulation.py no-viewer
```

This will:
- ✅ Run physics simulation
- ✅ Print dynamics (Mass matrix, Jacobian)
- ✅ Show joint positions and end-effector location
- ❌ No visualization

### Option 3: PyBullet (Alternative Physics Engine)
**Different physics engine with built-in viewer**

```bash
pip install pybullet
python ur5_simulation_pybullet.py
```

### Option 4: Interactive Launcher

```bash
python run_simulation.py
```

Choose from menu:
1. Matplotlib Animation (recommended)
2. MuJoCo Interactive Viewer (may not work)
3. No Viewer Mode
4. PyBullet Simulation

## What Each File Does

| File | Purpose |
|------|---------|
| `UR5.py` | Compute analytical dynamics (M, C, G matrices) |
| `ur5_animation.py` | **Matplotlib 3D animation (USE THIS)** |
| `ur5_simulation.py` | MuJoCo simulation (viewer may not work) |
| `ur5_simulation_pybullet.py` | PyBullet alternative |
| `run_simulation.py` | Interactive launcher |
| `ur5_model.xml` | MuJoCo robot model |

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

## Troubleshooting

**"No display found"** → Use `ur5_animation.py` (matplotlib) or `ur5_simulation.py no-viewer`

**"MuJoCo viewer error"** → Use `ur5_animation.py` instead

**"Module not found"** → 
```bash
pip install mujoco numpy matplotlib pillow
```

**Animation window doesn't show** → Close it manually if frozen, check for `ur5_animation.gif` and `ur5_configurations.png` files

## Output Files

After running simulations:

- `UR5.pkl` - Symbolic dynamics (M, C, G, Jacobian)
- `UR5T.txt`, `UR5M.txt`, `UR5C.txt`, `UR5G.txt`, `UR5J.txt` - Text format
- `ur5_animation.gif` - Animated robot motion
- `ur5_configurations.png` - Static robot poses

## Summary

**Want visualization?** → Use `python ur5_animation.py`

**Want data/dynamics?** → Use `python UR5.py` then load `UR5.pkl`

**No display available?** → Use `python ur5_simulation.py no-viewer`
