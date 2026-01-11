# UR5 Robot Simulation

This package contains analytical dynamics computation and physics-based simulation of the UR5 robot arm.

## Files

- **UR5.py** - Analytical dynamics computation using symbolic math (SymPy)
- **ur5_model.xml** - MuJoCo robot model definition
- **ur5_simulation.py** - MuJoCo-based simulation with interactive visualization
- **ur5_simulation_pybullet.py** - PyBullet-based simulation (alternative)

## Installation

### For MuJoCo Simulation (Recommended)

```bash
pip install mujoco numpy
```

### For PyBullet Simulation (Alternative)

```bash
pip install pybullet numpy
```

### For Analytical Dynamics

```bash
pip install sympy numpy
```

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

### 2. Run MuJoCo Simulation

Interactive simulation with 3D visualization:

```bash
python ur5_simulation.py
```

**Controls:**
- Mouse drag to rotate view
- Scroll to zoom
- The robot will automatically move through waypoints
- Press ESC to exit

**Non-interactive mode:**

```bash
python ur5_simulation.py no-viewer
```

### 3. Run PyBullet Simulation

Alternative physics engine:

```bash
python ur5_simulation_pybullet.py
```

## Features

### Analytical Model (UR5.py)
- Based on DH parameters from UR5 specifications
- Symbolic computation of full dynamics
- Exports equations for use in control algorithms
- Compatible with MATLAB implementation

### MuJoCo Simulation (ur5_simulation.py)
- Real-time physics simulation
- PD position control
- Trajectory following
- Jacobian computation
- Mass matrix extraction
- Interactive 3D visualization

### PyBullet Simulation (ur5_simulation_pybullet.py)
- Alternative to MuJoCo with simpler installation
- Built-in collision detection
- Real-time rendering
- PD control with force limits

## Robot Specifications

**UR5 DH Parameters:**
- 6 DOF revolute joints
- Total reach: ~850mm
- Payload: 5 kg
- Joint ranges: ±360° (most joints)

**Link parameters:**
- Shoulder to elbow: 425mm
- Elbow to wrist: 392.25mm
- Mass: ~18.4 kg total

## Example Code

### Set Target Position

```python
from ur5_simulation import UR5Simulation
import numpy as np

sim = UR5Simulation()
q_target = np.array([0, -np.pi/4, np.pi/2, -np.pi/4, -np.pi/2, 0])
sim.set_target_positions(q_target)

for _ in range(1000):
    sim.step()
    state = sim.get_state()
    print(f"EE position: {state['ee_pos']}")
```

### Compute Jacobian

```python
sim = UR5Simulation()
J = sim.get_jacobian()  # 6x6 Jacobian matrix
print(f"Jacobian:\n{J}")
```

### Extract Mass Matrix

```python
import mujoco
import numpy as np

sim = UR5Simulation()
M = np.zeros((sim.model.nv, sim.model.nv))
mujoco.mj_fullM(sim.model, M, sim.data.qM)
print(f"Mass matrix:\n{M[:6, :6]}")
```

## Comparison with Analytical Model

You can validate the MuJoCo simulation against the analytical model from UR5.py:

```python
# 1. Set same joint configuration in both
# 2. Extract M, C, G from MuJoCo
# 3. Substitute q values in SymPy expressions from UR5.py
# 4. Compare numerical values
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
