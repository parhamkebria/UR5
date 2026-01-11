"""
UR5 Robot Simulation using MuJoCo

This script simulates the UR5 robot arm with interactive control and visualization.

Usage:
    macOS:         mjpython ur5_simulation.py
    Linux/Windows: python ur5_simulation.py
    Headless:      python ur5_simulation.py no-viewer

Note: On macOS, the MuJoCo viewer requires 'mjpython' instead of 'python'.
"""

import numpy as np
import mujoco
import mujoco.viewer
import time
import os

class UR5Simulation:
    def __init__(self, model_path='ur5_model.xml'):
        """Initialize the UR5 simulation."""
        if not os.path.exists(model_path):
            raise FileNotFoundError(f"Model file not found: {model_path}")
        
        # Load model and create data
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        
        # Get joint and actuator info
        self.n_joints = 6
        self.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        
        # Control parameters
        self.control_mode = 'position'  # 'position', 'velocity', or 'torque'
        self.target_positions = np.zeros(6)
        self.kp = np.array([100, 100, 100, 50, 50, 50])  # Position gains
        self.kd = np.array([10, 10, 10, 5, 5, 5])  # Derivative gains
        
        print("UR5 Simulation initialized!")
        print(f"Number of joints: {self.n_joints}")
        print(f"Number of actuators: {self.model.nu}")
        
    def reset(self, q_init=None):
        """Reset simulation to initial state."""
        mujoco.mj_resetData(self.model, self.data)
        
        if q_init is not None:
            self.data.qpos[:self.n_joints] = q_init
            self.target_positions = q_init.copy()
        
        mujoco.mj_forward(self.model, self.data)
        
    def set_target_positions(self, q_target):
        """Set target joint positions."""
        self.target_positions = np.array(q_target)
        
    def pd_control(self):
        """Compute PD control torques."""
        q = self.data.qpos[:self.n_joints]
        dq = self.data.qvel[:self.n_joints]
        
        # PD control law
        error = self.target_positions - q
        torque = self.kp * error - self.kd * dq
        
        return torque
    
    def step(self, control=None):
        """Execute one simulation step."""
        if control is None:
            if self.control_mode == 'position':
                control = self.pd_control()
            else:
                control = np.zeros(self.n_joints)
        
        self.data.ctrl[:] = control
        mujoco.mj_step(self.model, self.data)
        
    def get_state(self):
        """Get current robot state."""
        return {
            'q': self.data.qpos[:self.n_joints].copy(),
            'dq': self.data.qvel[:self.n_joints].copy(),
            'tau': self.data.ctrl[:self.n_joints].copy(),
            'ee_pos': self.data.sensor('ee_pos').data.copy(),
            'ee_quat': self.data.sensor('ee_quat').data.copy(),
            'time': self.data.time
        }
    
    def get_jacobian(self, site_name='ee_site'):
        """Compute Jacobian matrix at end-effector."""
        jacp = np.zeros((3, self.model.nv))
        jacr = np.zeros((3, self.model.nv))
        
        site_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, site_name)
        mujoco.mj_jacSite(self.model, self.data, jacp, jacr, site_id)
        
        return np.vstack([jacp[:, :self.n_joints], jacr[:, :self.n_joints]])
    
    def print_state(self):
        """Print current state information."""
        state = self.get_state()
        print(f"\n=== Time: {state['time']:.3f} s ===")
        print("Joint positions (rad):")
        for i, name in enumerate(self.joint_names):
            print(f"  {name}: {state['q'][i]:.4f}")
        print(f"End-effector position: {state['ee_pos']}")
        

def run_interactive_simulation():
    """Run interactive simulation with viewer."""
    # Create simulation
    sim = UR5Simulation()
    
    # Reset to home position
    q_home = np.array([0, -np.pi/4, np.pi/2, -np.pi/4, -np.pi/2, 0])
    sim.reset(q_home)
    
    print("\n" + "="*60)
    print("UR5 Interactive Simulation")
    print("="*60)
    print("Controls:")
    print("  - Use viewer controls to rotate/zoom camera")
    print("  - Robot will move through predefined waypoints")
    print("  - Press ESC to exit")
    print("="*60 + "\n")
    
    # Define trajectory waypoints
    waypoints = [
        np.array([0, -np.pi/4, np.pi/2, -np.pi/4, -np.pi/2, 0]),
        np.array([np.pi/4, -np.pi/3, np.pi/3, -np.pi/6, -np.pi/2, 0]),
        np.array([np.pi/2, -np.pi/4, np.pi/4, 0, -np.pi/2, np.pi/4]),
        np.array([0, -np.pi/6, np.pi/3, -np.pi/6, -np.pi/2, 0]),
        np.array([0, -np.pi/4, np.pi/2, -np.pi/4, -np.pi/2, 0]),
    ]
    
    waypoint_idx = 0
    waypoint_hold_time = 3.0  # seconds
    last_waypoint_time = 0
    
    # Launch viewer
    with mujoco.viewer.launch_passive(sim.model, sim.data) as viewer:
        viewer.cam.distance = 2.5
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -20
        
        # Simulation loop
        while viewer.is_running():
            step_start = time.time()
            
            # Update waypoint
            if sim.data.time - last_waypoint_time > waypoint_hold_time:
                waypoint_idx = (waypoint_idx + 1) % len(waypoints)
                sim.set_target_positions(waypoints[waypoint_idx])
                last_waypoint_time = sim.data.time
                print(f"\nMoving to waypoint {waypoint_idx + 1}/{len(waypoints)}")
                sim.print_state()
            
            # Step simulation
            sim.step()
            
            # Sync viewer
            viewer.sync()
            
            # Maintain real-time
            time_until_next_step = sim.model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)


def run_trajectory_example():
    """Run a simple trajectory without visualization."""
    print("Running trajectory example without viewer...")
    
    sim = UR5Simulation()
    
    # Reset to home
    q_home = np.array([0, -np.pi/4, np.pi/2, -np.pi/4, -np.pi/2, 0])
    sim.reset(q_home)
    
    # Define target
    q_target = np.array([np.pi/4, -np.pi/3, np.pi/3, -np.pi/6, -np.pi/2, 0])
    sim.set_target_positions(q_target)
    
    print("Initial state:")
    sim.print_state()
    
    # Simulate for 3 seconds
    duration = 3.0
    steps = int(duration / sim.model.opt.timestep)
    
    print(f"\nSimulating {duration}s motion to target...")
    
    for i in range(steps):
        sim.step()
        
        if i % 500 == 0:  # Print every 500 steps
            print(f"Step {i}/{steps}")
    
    print("\nFinal state:")
    sim.print_state()
    
    # Compute Jacobian at final position
    J = sim.get_jacobian()
    print("\nEnd-effector Jacobian (6x6):")
    print(J)


def demo_dynamics_validation():
    """Demonstrate how to extract and validate dynamics."""
    print("\n" + "="*60)
    print("Dynamics Validation Demo")
    print("="*60)
    
    sim = UR5Simulation()
    q_test = np.array([0.1, -0.5, 0.8, -0.3, -1.2, 0.2])
    sim.reset(q_test)
    
    state = sim.get_state()
    
    # Extract mass matrix from MuJoCo
    M = np.zeros((sim.model.nv, sim.model.nv))
    mujoco.mj_fullM(sim.model, M, sim.data.qM)
    M_joints = M[:6, :6]
    
    print("\nMass Matrix M(q) from MuJoCo:")
    print(M_joints)
    print(f"\nMass matrix properties:")
    print(f"  - Symmetric: {np.allclose(M_joints, M_joints.T)}")
    print(f"  - Positive definite: {np.all(np.linalg.eigvals(M_joints) > 0)}")
    print(f"  - Condition number: {np.linalg.cond(M_joints):.2f}")
    
    # Compute Jacobian
    J = sim.get_jacobian()
    print(f"\nJacobian shape: {J.shape}")
    print(f"Jacobian condition number: {np.linalg.cond(J):.2f}")
    
    # Compare with analytical model from UR5.py
    print("\nNote: To validate against UR5.py analytical model,")
    print("substitute q values and compare M, C, G matrices.")


if __name__ == "__main__":
    import sys
    
    print("UR5 MuJoCo Simulation")
    print("="*60)
    
    # Check if MuJoCo viewer is available
    try:
        if len(sys.argv) > 1 and sys.argv[1] == 'no-viewer':
            run_trajectory_example()
            demo_dynamics_validation()
        else:
            print("\nAttempting to launch interactive 3D viewer...")
            print("Note: If viewer doesn't open, use 'python ur5_animation.py' instead")
            print("      or run with 'python ur5_simulation.py no-viewer'\n")
            run_interactive_simulation()
    except KeyboardInterrupt:
        print("\nSimulation interrupted by user")
    except Exception as e:
        print(f"\n⚠️  Interactive viewer error: {e}")
        print("\nThe MuJoCo viewer requires a display environment.")
        print("Alternatives:")
        print("  1. Run: python ur5_animation.py (matplotlib-based visualization)")
        print("  2. Run: python ur5_simulation.py no-viewer (no graphics)")
        print("\nFalling back to non-interactive mode...")
        run_trajectory_example()
        demo_dynamics_validation()
