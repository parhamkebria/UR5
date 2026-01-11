# UR5 Robot Simulation - Complete Guide

This package provides comprehensive analytical dynamics computation and physics-based simulation of the UR5 robot arm.

## 📦 Installation

### Complete Installation
```bash
pip install mujoco numpy sympy matplotlib pillow
```

### Minimal (dynamics only)
```bash
pip install numpy sympy
```

### Recommended (simulation + visualization)
```bash
pip install mujoco numpy matplotlib
```

## 📁 File Overview

| File | Type | Purpose |
|------|------|---------|
| `UR5.m` | MATLAB | Original implementation |
| `UR5.py` | Python | Symbolic dynamics computation |
| `ur5_model.xml` | XML | MuJoCo robot model |
| `ur5_animation.py` | Python | 3D visualization ⭐ |
| `ur5_simulation.py` | Python | MuJoCo headless simulation |
| `run_simulation.py` | Python | Interactive launcher |

## Usage

### 1. Compute Analytical Dynamics

Run the symbolic dynamics computation:

```bash
python UR5.py
```

This computes:
- Forward kinematics transformation matrix (T)
- End-effector Jacobian
- Mass/inertia matrix (M)
- Coriolis matrix (C)
- Gravity vector (G)

Results are saved to `UR5.pkl` and text files.

### 2. Run 3D Visualization

**Recommended** - Always works, creates output files:

```bash
python ur5_animation.py
```

**What happens:**
- Simulates robot moving through 5 waypoints
- Creates animated GIF (`ur5_animation.gif`)
- Saves static configurations (`ur5_configurations.png`)
- Opens interactive matplotlib window

### 3. Run Headless Simulation

For servers or batch processing:

```bash
python ur5_simulation.py no-viewer
```

**Output:**
- Joint positions and velocities
- End-effector position
- Mass matrix (6×6)
- Jacobian matrix (6×6)
- Matrix properties (symmetry, positive definiteness)
🎯 Features

### Analytical Model (UR5.py)
✅ Based on DH parameters from UR5 specifications  
✅ Symbolic computation of full dynamics (M, C, G)  
✅ Forward kinematics transformation matrices  
✅ 6×6 Jacobian computation  
✅ Exports to Python pickle and text files  
✅ Compatible with original MATLAB implementation  

### Matplotlib Visualization (ur5_animation.py) ⭐
✅ 3D animated robot motion  
✅ Trajectory plotting  
✅ Multiple waypoint demonstration  
✅ Saves GIF animation  
✅ Saves static configuration images  
✅ Works on all systems (no display required)  

### MuJoCo Simulation (ur5_simulation.py)
✅ Accurate physics simulation  
✅ PD position control  
✅ Trajectory following  
✅ Mass matrix extraction  
✅ Jacobian computation at any configuration  
✅ Headless mode for servers  

## 📐 Robot Model Details

**DH Parameters:**
| Link | α | a (m) | d (m) | θ |
|------|---|-------|-------|---|
| 1 | 0 | 0 | 0.08916 | q₁ |
| 2 | π/2 | 0 | 0 | q₂ |
| 3 | 0 | 0.425 | 0 | q₃ |
| 4 | 0 | 0.39225 | 0.10915 | q₄ |
| 5 | -π/2 | 0 | 0.09456 | q₅ |
| 6 | π/2 | 0 | 0.0823 | q₆ |

**Mass Properties:**
- Link masses: [3.7, 8.393, 2.275, 1.219, 1.219, 0.1879] kg
- Total robot mass: ~18.4 kg
- Payload capacity: 5 kg

**Workspace:**
- Total reach: ~850 mm
- Joint ranges: ±360° (most joints)
- Shoulder-Elbow: 425 mm
- Elbow-Wrist: 392.25 mm

## 💻 Code Examples

### Create and Run Animation

```python
from ur5_animation import UR5Animator
import numpy as np

# Create animator
animator = UR5Animator()

# Define waypoints
waypoints = [
    np.array([0, -np.pi/4, np.pi/2, -np.pi/4, -np.pi/2, 0]),
    np.array([np.pi/4, -np.pi/3, np.pi/3, -np.pi/6, -np.pi/2, 0])
]

# Simulate and record
trajectory = animator.simulate_trajectory(waypoints, steps_per_waypoint=500)

# Access trajectory data
for frame in trajectory:
    print(f"Time: {frame['time']:.3f}, EE: {frame['ee_pos']}")
```

### Load and Use Dynamics Data

```python
import pickle
import numpy as np
from sympy import symbols

# Load computed dynamics
with open('UR5.pkl', 'rb') as f:
    data = pickle.load(f)

T = data['T']        # Forward kinematics
M = data['M']        # Mass matrix  
C = data['C']        # Coriolis matrix
G = data['G']        # Gravity vector
J = data['Jacobi']   # Jacobian

# Define joint variables
q_1, q_2, q_3, q_4, q_5, q_6 = symbols('q_1 q_2 q_3 q_4 q_5 q_6')
g = symbols('g')

# Substitute numerical values
q_vals = {
    q_1: 0, q_2: -np.pi/4, q_3: np.pi/2,
  # Extract Mass Matrix from MuJoCo

```python
import mujoco
import numpy as np
from ur5_simulation import UR5Simulation

sim = UR5Simulation()

# Set configuration
q = np.array([0.1, -0.5, 0.8, -0.3, -1.2, 0.2])
sim.reset(q)

# Extract mass matrix
M = np.zeros((sim.model.nv, sim.model.nv))
mujoco.mj_fullM(sim.model, M, sim.data.qM)
M_robot = M[:6, :6]

print(f"Mass matrix:\n{M_robot}")
print(f"Symmetric: {np.allclose(M_robot, M_robot.T)}")
print(f"Positive definite: {np.all(np.linalg.eigvals(M_robot) > 0)}")
```

### Compute Jacobian

```python
from ur5_simulation import UR5Simulation
import numpy as np

sim = UR5Simulation()
q = np.array([0, -np.pi/4, np.pi/2, -np.pi/4, -np.pi/2, 0])
sim.reset(q)

# Get Jacobian
J = sim.get_jacobian()  # 6×6 matrix
print(f"Jacobian:\n{J}")

# Check manipulability
det_J = np.linalg.det(J)
print(f"Manipulability measure: {abs(det_J):.6f}")
```

## ✅ Validation

Compare analytical model (UR5.py) with simulation (MuJoCo):

```python
import pickle
import numpy as np
from sympy import symbols
from ur5_simulation import UR5Simulation
import mujoco

# 1. Load analytical dynamics
with open('UR5.pkl', 'rb') as f:
    analytical = pickle.load(f)

# 2. Set same configuration
q_test = [0.1, -0.5, 0.8, -0.3, -1.2, 0.2]

# 3. Get numerical mass matrix from analytical model
q_1, q_2, q_3, q_4, q_5, q_6 = symbols('q_1 q_2 q_3 q_4 q_5 q_6')
subs_dict = {q_1: q_test[0], q_2: q_test[1], q_3: q_test[2],
             q_4: q_test[3], q_5: q_test[4], q_6: q_test[5]}
M_analytical = np.array(analytical['M'].subs(subs_dict)).astype(np.float64)

# 4. Get mass matrix from MuJoCo
sim = UR5Simulation()
sim.reset(np.array(q_test))
M_mujoco = np.zeros((6, 6))
mujoco.mj_fullM(sim.model, M_mujoco, sim.data.qM)

# 5. Compare
difference = np.abs(M_analytical - M_mujoco)
print(f"Max difference: {np.max(difference):.6f}")
print(f"Relative error: {np.max(difference / M_analytical):.2%}")
```

## 🔧 Troubleshooting

| Issue | Cause | Solution |
|-------|-------|----------|
| "Module not found" | Missing dependencies | `pip install mujoco numpy sympy matplotlib` |
| No display/viewer | SSH/headless system | Use `python ur5_animation.py` |
| UR5.py slow | Symbolic computation | Normal - wait 1-2 minutes |
| Animation frozen | Window not responding | Close manually, check output files |
| Import errors | Wrong environment | Check with `which python` and activate correct env |

## 📚 References

- [Universal Robots UR5 Specifications](https://www.universal-robots.com/products/ur5-robot/)
- [MuJoCo Documentation](https://mujoco.readthedocs.io)
- [SymPy Documentation](https://docs.sympy.org)
- Denavit-Hartenberg Parameters Convention
- Featherstone, R. (2008). Rigid Body Dynamics Algorithms

## 🚫 Known Issues

- **MuJoCo interactive viewer** doesn't work on all systems → Use matplotlib animation
- **PyBullet** won't compile on modern macOS → Not needed, removed from project  
- **Coriolis matrix** file is very large (~11 MB) → This is normal for symbolic form
- **Long computation time** for UR5.py → Symbolic manipulation is computationally intensive
# Simulate
for i in range(1000):
    sim.step()
    if i % 100 == 0:
        state = sim.get_state()
        print(f"Step {i}: q = {state['q']}")
```

## Troubleshooting

**MuJoCo Installation Issues:**
- On macOS: `brew install mujoco`
- Or use PyBullet alternative

**ImportError: No module named 'mujoco':**
```bash
pip install --upgrade mujoco
```

**Simulation runs too fast:**
- Adjust `timestep` in ur5_model.xml (MuJoCo)
- Adjust `time.sleep()` in simulation script

**Robot falls or behaves strangely:**
- Check joint limits in XML file
- Adjust PD gains (kp, kd) in simulation script
- Verify initial configuration is valid

## References

- Universal Robots UR5 specifications
- MuJoCo documentation: https://mujoco.readthedocs.io
- PyBullet documentation: https://pybullet.org
