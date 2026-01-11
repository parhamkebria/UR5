"""
UR5 Robot Simulation using PyBullet with URDF
Stable version with realistic motion
"""
import numpy as np
import pybullet as p
import pybullet_data
import time

# Connect to PyBullet
p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_GUI, 1)
p.resetDebugVisualizerCamera(
    cameraDistance=2.0,
    cameraYaw=45,
    cameraPitch=-30,
    cameraTargetPosition=[0, 0, 0.5]
)

# Set up environment
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -9.81)
p.setTimeStep(1/240.0)

# Load ground plane
plane_id = p.loadURDF("plane.urdf")

# Load UR5 robot
robot_id = p.loadURDF("ur5_pybullet.urdf", 
                      basePosition=[0, 0, 0.1],
                      useFixedBase=True)

# Get joint information
n_joints = p.getNumJoints(robot_id)
joint_indices = []
joint_names = []

for i in range(n_joints):
    info = p.getJointInfo(robot_id, i)
    if info[2] == p.JOINT_REVOLUTE:  # Only revolute joints
        joint_indices.append(i)
        joint_names.append(info[1].decode('utf-8'))

print(f"Found {len(joint_indices)} controllable joints:")
for i, name in enumerate(joint_names):
    print(f"  Joint {i}: {name}")

# Initial position
q_home = [0, -1.57, 1.57, -1.57, -1.57, 0]

# Reset to home
for i, idx in enumerate(joint_indices):
    p.resetJointState(robot_id, idx, q_home[i])

# Enable force/torque sensor
for idx in joint_indices:
    p.enableJointForceTorqueSensor(robot_id, idx, True)

# Define waypoints
waypoints = [
    [0, -1.57, 1.57, -1.57, -1.57, 0],           # Home
    [0.5, -1.2, 1.8, -2.2, -1.57, 0],            # Reach forward
    [1.0, -1.4, 2.0, -2.0, -1.57, 0.5],          # Reach side
    [-0.5, -1.0, 1.5, -2.0, -1.57, -0.5],        # Other side
    [0, -1.57, 1.57, -1.57, -1.57, 0],           # Back to home
]

waypoint_idx = 0
target_q = waypoints[waypoint_idx]

# Control gains
kp = 50.0
kd = 10.0
max_force = 100.0

print("\n" + "="*60)
print("UR5 PyBullet Simulation - URDF Version")
print("="*60)
print("\nRobot will cycle through waypoints")
print("Press Ctrl+C to exit\n")

step_count = 0
steps_per_waypoint = 1000

try:
    while True:
        # Update target every N steps
        if step_count % steps_per_waypoint == 0:
            waypoint_idx = (waypoint_idx + 1) % len(waypoints)
            target_q = waypoints[waypoint_idx]
            print(f"Moving to waypoint {waypoint_idx + 1}/{len(waypoints)}")
        
        # Apply position control
        p.setJointMotorControlArray(
            bodyUniqueId=robot_id,
            jointIndices=joint_indices,
            controlMode=p.POSITION_CONTROL,
            targetPositions=target_q,
            positionGains=[kp] * len(joint_indices),
            velocityGains=[kd] * len(joint_indices),
            forces=[max_force] * len(joint_indices)
        )
        
        # Step simulation
        p.stepSimulation()
        time.sleep(1/240.0)
        step_count += 1

except KeyboardInterrupt:
    print("\nSimulation stopped")

p.disconnect()
