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
        
        # Joint information
        self.n_joints = 6
        
        # Create UR5 robot from scratch using DH parameters
        self.robot_id = self.create_ur5_robot()
        self.joint_indices = list(range(self.n_joints))
        self.joint_names = [
            'shoulder_pan', 'shoulder_lift', 'elbow',
            'wrist_1', 'wrist_2', 'wrist_3'
        ]
        
        # Control parameters
        self.kp = np.array([500, 500, 500, 100, 100, 100])
        self.kd = np.array([50, 50, 50, 10, 10, 10])
        self.max_force = np.array([150, 150, 150, 28, 28, 28])
        
        print("UR5 PyBullet Simulation initialized!")
        
    def create_ur5_robot(self):
        """Create UR5 robot using primitive shapes based on DH parameters."""
        base_position = [0, 0, 0.1]
        base_orientation = p.getQuaternionFromEuler([0, 0, 0])
        
        # Visual and collision shapes for each link
        visual_shapes = []
        collision_shapes = []
        link_masses = [3.7, 8.393, 2.275, 1.219, 1.219, 0.1879]
        link_positions = []
        link_orientations = []
        link_inertial_frame_positions = []
        link_inertial_frame_orientations = []
        joint_types = []
        joint_axes = []
        parent_indices = []
        
        # DH parameters
        d = [0.08916, 0, 0, 0.10915, 0.09456, 0.0823]
        a = [0, 0, 0.425, 0.39225, 0, 0]
        alpha = [0, np.pi/2, 0, 0, -np.pi/2, np.pi/2]
        
        # Link 1: Shoulder
        visual_shapes.append(p.createVisualShape(
            shapeType=p.GEOM_CYLINDER,
            radius=0.06,
            length=0.1,
            rgbaColor=[0.2, 0.2, 0.8, 1]
        ))
        collision_shapes.append(p.createCollisionShape(
            shapeType=p.GEOM_CYLINDER,
            radius=0.06,
            height=0.1
        ))
        link_positions.append([0, 0, d[0]])
        link_orientations.append(p.getQuaternionFromEuler([0, 0, 0]))
        link_inertial_frame_positions.append([0, 0, 0])
        link_inertial_frame_orientations.append([0, 0, 0, 1])
        joint_types.append(p.JOINT_REVOLUTE)
        joint_axes.append([0, 0, 1])
        parent_indices.append(0)  # Parent is base
        
        # Link 2: Upper arm
        visual_shapes.append(p.createVisualShape(
            shapeType=p.GEOM_CAPSULE,
            radius=0.05,
            length=0.425,
            rgbaColor=[0.8, 0.2, 0.2, 1]
        ))
        collision_shapes.append(p.createCollisionShape(
            shapeType=p.GEOM_CAPSULE,
            radius=0.05,
            height=0.425
        ))
        link_positions.append([0, 0.13585, 0])
        link_orientations.append(p.getQuaternionFromEuler([0, np.pi/2, 0]))
        link_inertial_frame_positions.append([0, 0, 0])
        link_inertial_frame_orientations.append([0, 0, 0, 1])
        joint_types.append(p.JOINT_REVOLUTE)
        joint_axes.append([0, 1, 0])
        parent_indices.append(0)  # Parent is link 1 (index 0 in link list)
        
        # Link 3: Forearm
        visual_shapes.append(p.createVisualShape(
            shapeType=p.GEOM_CAPSULE,
            radius=0.045,
            length=0.39225,
            rgbaColor=[0.2, 0.8, 0.2, 1]
        ))
        collision_shapes.append(p.createCollisionShape(
            shapeType=p.GEOM_CAPSULE,
            radius=0.045,
            height=0.39225
        ))
        link_positions.append([0.425, 0, 0])
        link_orientations.append(p.getQuaternionFromEuler([0, 0, 0]))
        link_inertial_frame_positions.append([0, 0, 0])
        link_inertial_frame_orientations.append([0, 0, 0, 1])
        joint_types.append(p.JOINT_REVOLUTE)
        joint_axes.append([0, 1, 0])
        parent_indices.append(1)  # Parent is link 2
        
        # Link 4: Wrist 1
        visual_shapes.append(p.createVisualShape(
            shapeType=p.GEOM_CYLINDER,
            radius=0.035,
            length=0.08,
            rgbaColor=[0.2, 0.2, 0.8, 1]
        ))
        collision_shapes.append(p.createCollisionShape(
            shapeType=p.GEOM_CYLINDER,
            radius=0.035,
            height=0.08
        ))
        link_positions.append([0.39225, 0, 0.10915])
        link_orientations.append(p.getQuaternionFromEuler([0, 0, 0]))
        link_inertial_frame_positions.append([0, 0, 0])
        link_inertial_frame_orientations.append([0, 0, 0, 1])
        joint_types.append(p.JOINT_REVOLUTE)
        joint_axes.append([0, 0, 1])
        parent_indices.append(2)  # Parent is link 3
        
        # Link 5: Wrist 2
        visual_shapes.append(p.createVisualShape(
            shapeType=p.GEOM_CYLINDER,
            radius=0.035,
            length=0.08,
            rgbaColor=[0.8, 0.2, 0.2, 1]
        ))
        collision_shapes.append(p.createCollisionShape(
            shapeType=p.GEOM_CYLINDER,
            radius=0.035,
            height=0.08
        ))
        link_positions.append([0, -0.09456, 0])
        link_orientations.append(p.getQuaternionFromEuler([np.pi/2, 0, 0]))
        link_inertial_frame_positions.append([0, 0, 0])
        link_inertial_frame_orientations.append([0, 0, 0, 1])
        joint_types.append(p.JOINT_REVOLUTE)
        joint_axes.append([0, 1, 0])
        parent_indices.append(3)  # Parent is link 4
        
        # Link 6: Wrist 3
        visual_shapes.append(p.createVisualShape(
            shapeType=p.GEOM_CYLINDER,
            radius=0.03,
            length=0.06,
            rgbaColor=[0.2, 0.8, 0.2, 1]
        ))
        collision_shapes.append(p.createCollisionShape(
            shapeType=p.GEOM_CYLINDER,
            radius=0.03,
            height=0.06
        ))
        link_positions.append([0, 0, 0.0823])
        link_orientations.append(p.getQuaternionFromEuler([0, 0, 0]))
        link_inertial_frame_positions.append([0, 0, 0])
        link_inertial_frame_orientations.append([0, 0, 0, 1])
        joint_types.append(p.JOINT_REVOLUTE)
        joint_axes.append([0, 0, 1])
        parent_indices.append(4)  # Parent is link 5
        
        # Create base
        base_visual = p.createVisualShape(
            shapeType=p.GEOM_CYLINDER,
            radius=0.08,
            length=0.1,
            rgbaColor=[0.5, 0.5, 0.5, 1]
        )
        base_collision = p.createCollisionShape(
            shapeType=p.GEOM_CYLINDER,
            radius=0.08,
            height=0.1
        )
        
        # Create multi-body
        robot_id = p.createMultiBody(
            baseMass=4.0,
            baseCollisionShapeIndex=base_collision,
            baseVisualShapeIndex=base_visual,
            basePosition=base_position,
            baseOrientation=base_orientation,
            linkMasses=link_masses,
            linkCollisionShapeIndices=collision_shapes,
            linkVisualShapeIndices=visual_shapes,
            linkPositions=link_positions,
            linkOrientations=link_orientations,
            linkInertialFramePositions=link_inertial_frame_positions,
            linkInertialFrameOrientations=link_inertial_frame_orientations,
            linkParentIndices=parent_indices,
            linkJointTypes=joint_types,
            linkJointAxis=joint_axes
        )
        
        # Set joint damping
        for i in range(self.n_joints):
            p.changeDynamics(robot_id, i, linearDamping=0.04, angularDamping=0.04)
            # Disable collisions between adjacent links
            if i > 0:
                p.setCollisionFilterPair(robot_id, robot_id, i-1, i, 0)
        
        return robot_id
    
    def reset(self, q_init=None):
        """Reset robot to initial configuration."""
        if q_init is None:
            q_init = np.zeros(6)
        
        for i, q in enumerate(q_init):
            p.resetJointState(self.robot_id, i, q)
    
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
