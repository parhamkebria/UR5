"""
UR5 Robot Simulation using PyBullet (Alternative)
This script provides an alternative simulation using PyBullet if MuJoCo is not available.
"""

import numpy as np
import pybullet as p
import pybullet_data
import time
import os

class UR5SimulationPyBullet:
    def __init__(self, use_gui=True):
        """Initialize PyBullet simulation."""
        # Connect to PyBullet
        if use_gui:
            self.client = p.connect(p.GUI)
            p.configureDebugVisualizer(p.COV_ENABLE_GUI, 0)
            p.resetDebugVisualizerCamera(
                cameraDistance=2.5,
                cameraYaw=50,
                cameraPitch=-35,
                cameraTargetPosition=[0, 0, 0.5]
            )
        else:
            self.client = p.connect(p.DIRECT)
        
        # Set up environment
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(0.001)
        
        # Load ground plane
        self.plane_id = p.loadURDF("plane.urdf")
        
        # Load UR5 robot from URDF (with realistic meshes)
        self.robot_id = p.loadURDF("ur5_pybullet.urdf",
                                    basePosition=[0, 0, 0.1],
                                    useFixedBase=True)
        
        # Get joint information
        self.n_joints = 0
        self.joint_indices = []
        for i in range(p.getNumJoints(self.robot_id)):
            info = p.getJointInfo(self.robot_id, i)
            if info[2] == p.JOINT_REVOLUTE:  # Only revolute joints
                self.joint_indices.append(i)
                self.n_joints += 1
        
        self.joint_names = [
            'shoulder_pan', 'shoulder_lift', 'elbow',
            'wrist_1', 'wrist_2', 'wrist_3'
        ]
        
        # Control parameters
        self.kp = np.array([500, 500, 500, 100, 100, 100])
        self.kd = np.array([50, 50, 50, 10, 10, 10])
        self.max_force = np.array([150, 150, 150, 28, 28, 28])
        
        print("UR5 PyBullet Simulation initialized!")
    
    def reset(self, q_init=None):
        """Reset robot to initial configuration."""
        if q_init is None:
            q_init = np.zeros(6)
        
        for i, q in enumerate(q_init):
            p.resetJointState(self.robot_id, self.joint_indices[i], q)
    
    def set_target_positions(self, q_target, kp=None, kd=None, max_force=None):
        """Set target joint positions with PD control."""
        if kp is None:
            kp = self.kp
        if kd is None:
            kd = self.kd
        if max_force is None:
            max_force = self.max_force
        
        p.setJointMotorControlArray(
            bodyUniqueId=self.robot_id,
            jointIndices=self.joint_indices,
            controlMode=p.POSITION_CONTROL,
            targetPositions=q_target,
            positionGains=kp,
            velocityGains=kd,
            forces=max_force
        )
    
    def get_state(self):
        """Get current robot state."""
        joint_states = p.getJointStates(self.robot_id, self.joint_indices)
        q = np.array([state[0] for state in joint_states])
        dq = np.array([state[1] for state in joint_states])
        tau = np.array([state[3] for state in joint_states])
        
        # Get end-effector pose
        link_state = p.getLinkState(self.robot_id, self.n_joints - 1, computeForwardKinematics=True)
        ee_pos = np.array(link_state[4])
        ee_quat = np.array(link_state[5])
        
        return {
            'q': q,
            'dq': dq,
            'tau': tau,
            'ee_pos': ee_pos,
            'ee_quat': ee_quat
        }
    
    def step(self):
        """Step the simulation."""
        p.stepSimulation()
    
    def print_state(self):
        """Print current state."""
        state = self.get_state()
        print("\nJoint positions (rad):")
        for i, name in enumerate(self.joint_names):
            print(f"  {name}: {state['q'][i]:.4f}")
        print(f"End-effector position: {state['ee_pos']}")
    
    def close(self):
        """Close simulation."""
        p.disconnect()


def run_pybullet_simulation():
    """Run PyBullet simulation."""
    print("="*60)
    print("UR5 PyBullet Simulation")
    print("="*60)
    
    # Create simulation
    sim = UR5SimulationPyBullet(use_gui=True)
    
    # Reset to home position
    q_home = np.array([0, -np.pi/4, np.pi/2, -np.pi/4, -np.pi/2, 0])
    sim.reset(q_home)
    
    # Let the robot settle before starting control
    for _ in range(100):
        p.stepSimulation()
        time.sleep(0.001)
    
    print("\nRobot will move through predefined waypoints")
    print("Close the window to exit\n")
    
    # Define waypoints
    waypoints = [
        np.array([0, -np.pi/4, np.pi/2, -np.pi/4, -np.pi/2, 0]),
        np.array([np.pi/4, -np.pi/3, np.pi/3, -np.pi/6, -np.pi/2, 0]),
        np.array([np.pi/2, -np.pi/4, np.pi/4, 0, -np.pi/2, np.pi/4]),
        np.array([0, -np.pi/6, np.pi/3, -np.pi/6, -np.pi/2, 0]),
        np.array([0, -np.pi/4, np.pi/2, -np.pi/4, -np.pi/2, 0]),
    ]
    
    waypoint_idx = 0
    steps_per_waypoint = 3000
    step_count = 0
    
    try:
        while True:
            # Update target
            if step_count % steps_per_waypoint == 0:
                waypoint_idx = (waypoint_idx + 1) % len(waypoints)
                sim.set_target_positions(waypoints[waypoint_idx])
                print(f"\nMoving to waypoint {waypoint_idx + 1}/{len(waypoints)}")
                sim.print_state()
            
            # Step simulation
            sim.step()
            time.sleep(0.001)
            step_count += 1
            
    except KeyboardInterrupt:
        print("\nSimulation interrupted")
    finally:
        sim.close()


if __name__ == "__main__":
    run_pybullet_simulation()
