"""
Realistic UR5 Simulation with High-Quality Rendering
Uses official UR5 3D meshes for realistic visualization
"""
import mujoco
import mujoco.viewer
import numpy as np

def main():
    # Load the realistic model
    model = mujoco.MjModel.from_xml_path('ur5_realistic.xml')
    data = mujoco.MjData(model)
    
    # Enhanced rendering options
    model.vis.global_.offwidth = 1920
    model.vis.global_.offheight = 1080
    model.vis.quality.shadowsize = 4096
    model.vis.quality.offsamples = 8
    
    # Define some interesting poses for the robot
    poses = {
        'home': np.array([0, -1.57, 1.57, -1.57, -1.57, 0]),
        'reach_forward': np.array([0, -1.2, 1.8, -2.2, -1.57, 0]),
        'reach_up': np.array([0, -2.0, 2.2, -1.8, -1.57, 0]),
        'side': np.array([1.57, -1.57, 1.57, -1.57, -1.57, 0]),
    }
    
    current_pose = 'home'
    target_qpos = poses[current_pose].copy()
    
    # PD controller gains
    kp = np.array([2000, 2000, 2000, 500, 500, 500])
    kd = np.array([200, 200, 200, 50, 50, 50])
    
    print("=" * 60)
    print("UR5 Realistic Simulation")
    print("=" * 60)
    print("\nControls:")
    print("  1: Home position")
    print("  2: Reach forward")
    print("  3: Reach up")
    print("  4: Side position")
    print("  Space: Pause/Resume")
    print("  Esc: Quit")
    print("\nMouse:")
    print("  Left drag: Rotate view")
    print("  Right drag: Move view")
    print("  Scroll: Zoom")
    print("=" * 60)
    
    def key_callback(keycode):
        nonlocal current_pose, target_qpos
        
        if keycode == 49:  # '1'
            current_pose = 'home'
            target_qpos = poses[current_pose].copy()
            print(f"Moving to: {current_pose}")
        elif keycode == 50:  # '2'
            current_pose = 'reach_forward'
            target_qpos = poses[current_pose].copy()
            print(f"Moving to: {current_pose}")
        elif keycode == 51:  # '3'
            current_pose = 'reach_up'
            target_qpos = poses[current_pose].copy()
            print(f"Moving to: {current_pose}")
        elif keycode == 52:  # '4'
            current_pose = 'side'
            target_qpos = poses[current_pose].copy()
            print(f"Moving to: {current_pose}")
    
    # Start interactive visualization
    with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
        # Set initial camera position for best view
        viewer.cam.distance = 2.5
        viewer.cam.azimuth = 135
        viewer.cam.elevation = -25
        viewer.cam.lookat = np.array([0.0, 0.0, 0.5])
        
        step = 0
        while viewer.is_running():
            # PD control
            q_error = target_qpos - data.qpos[:6]
            qd_error = -data.qvel[:6]
            data.ctrl[:6] = kp * q_error + kd * qd_error
            
            # Step simulation
            mujoco.mj_step(model, data)
            
            # Update viewer
            viewer.sync()
            
            # Print status every 500 steps
            if step % 500 == 0:
                ee_pos = data.site('ee_site').xpos
                print(f"\rStep: {step:6d} | Pose: {current_pose:15s} | "
                      f"EE Position: [{ee_pos[0]:6.3f}, {ee_pos[1]:6.3f}, {ee_pos[2]:6.3f}]", 
                      end='', flush=True)
            
            step += 1

if __name__ == '__main__':
    main()
