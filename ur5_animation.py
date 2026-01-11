"""
UR5 Robot Animation with Matplotlib 3D Visualization
Works without GUI dependencies - plots robot motion and saves animation
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
import mujoco

class UR5Animator:
    def __init__(self, model_path='ur5_model.xml'):
        """Initialize the UR5 animator."""
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        
        self.n_joints = 6
        self.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ]
        
        # Control parameters
        self.target_positions = np.zeros(6)
        self.kp = np.array([100, 100, 100, 50, 50, 50])
        self.kd = np.array([10, 10, 10, 5, 5, 5])
        
        # Trajectory storage
        self.trajectory = []
        
        print("UR5 Animator initialized!")
        
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
        error = self.target_positions - q
        torque = self.kp * error - self.kd * dq
        return torque
    
    def step(self):
        """Execute one simulation step."""
        control = self.pd_control()
        self.data.ctrl[:] = control
        mujoco.mj_step(self.model, self.data)
        
    def get_link_positions(self):
        """Get positions of all links for visualization."""
        positions = [np.array([0, 0, 0.1])]  # Base position
        
        # Get each link's position using body names
        link_names = ['shoulder_link', 'upper_arm_link', 'forearm_link', 
                     'wrist_1_link', 'wrist_2_link', 'wrist_3_link']
        
        for link_name in link_names:
            try:
                body_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, link_name)
                xpos = self.data.xpos[body_id].copy()
                positions.append(xpos)
            except:
                # Fallback: compute from joint positions
                pass
        
        return np.array(positions)
    
    def simulate_trajectory(self, waypoints, steps_per_waypoint=1000):
        """Simulate robot moving through waypoints and record trajectory."""
        print(f"\nSimulating trajectory with {len(waypoints)} waypoints...")
        self.trajectory = []
        
        # Start from first waypoint
        self.reset(waypoints[0])
        
        for i, waypoint in enumerate(waypoints):
            print(f"  Waypoint {i+1}/{len(waypoints)}: {waypoint}")
            self.set_target_positions(waypoint)
            
            for step in range(steps_per_waypoint):
                self.step()
                
                # Record every 10th frame to keep data manageable
                if step % 10 == 0:
                    positions = self.get_link_positions()
                    self.trajectory.append({
                        'time': self.data.time,
                        'q': self.data.qpos[:self.n_joints].copy(),
                        'positions': positions,
                        'ee_pos': positions[-1]
                    })
        
        print(f"Simulation complete! Recorded {len(self.trajectory)} frames")
        return self.trajectory


def plot_robot_3d(ax, positions, color='blue', alpha=0.7):
    """Plot robot configuration in 3D."""
    # Plot links
    for i in range(len(positions) - 1):
        ax.plot3D([positions[i][0], positions[i+1][0]],
                  [positions[i][1], positions[i+1][1]],
                  [positions[i][2], positions[i+1][2]],
                  color=color, linewidth=3, alpha=alpha)
    
    # Plot joints
    ax.scatter(positions[:, 0], positions[:, 1], positions[:, 2],
               c=color, s=100, alpha=alpha, edgecolors='black', linewidth=1)
    
    # Highlight end-effector
    ax.scatter([positions[-1][0]], [positions[-1][1]], [positions[-1][2]],
               c='red', s=200, marker='*', edgecolors='black', linewidth=2)


def create_animation(trajectory):
    """Create animated visualization of robot motion."""
    print("\nCreating animation...")
    
    fig = plt.figure(figsize=(14, 6))
    
    # 3D robot view
    ax1 = fig.add_subplot(121, projection='3d')
    
    # End-effector trajectory
    ax2 = fig.add_subplot(122, projection='3d')
    
    # Extract end-effector trajectory for plotting
    ee_trajectory = np.array([frame['ee_pos'] for frame in trajectory])
    
    def init():
        ax1.clear()
        ax2.clear()
        
        # Set up 3D view
        for ax in [ax1, ax2]:
            ax.set_xlabel('X (m)')
            ax.set_ylabel('Y (m)')
            ax.set_zlabel('Z (m)')
            ax.set_xlim([-0.8, 0.8])
            ax.set_ylim([-0.8, 0.8])
            ax.set_zlim([0, 1.2])
            ax.view_init(elev=20, azim=45)
        
        ax1.set_title('Robot Configuration')
        ax2.set_title('End-Effector Trajectory')
        
        # Plot full trajectory in second subplot
        ax2.plot3D(ee_trajectory[:, 0], ee_trajectory[:, 1], ee_trajectory[:, 2],
                   'b-', linewidth=1, alpha=0.5, label='Path')
        ax2.legend()
        
        return fig,
    
    def update(frame_idx):
        ax1.clear()
        
        # Set up 3D view
        ax1.set_xlabel('X (m)')
        ax1.set_ylabel('Y (m)')
        ax1.set_zlabel('Z (m)')
        ax1.set_xlim([-0.8, 0.8])
        ax1.set_ylim([-0.8, 0.8])
        ax1.set_zlim([0, 1.2])
        ax1.view_init(elev=20, azim=45)
        ax1.set_title(f'Robot Configuration (t={trajectory[frame_idx]["time"]:.2f}s)')
        
        # Plot ground plane
        xx, yy = np.meshgrid(np.linspace(-0.5, 0.5, 2), np.linspace(-0.5, 0.5, 2))
        ax1.plot_surface(xx, yy, np.zeros_like(xx), alpha=0.2, color='gray')
        
        # Plot robot
        positions = trajectory[frame_idx]['positions']
        plot_robot_3d(ax1, positions)
        
        # Update trajectory view
        if frame_idx > 0:
            # Clear and redraw
            ax2.clear()
            ax2.set_xlabel('X (m)')
            ax2.set_ylabel('Y (m)')
            ax2.set_zlabel('Z (m)')
            ax2.set_xlim([-0.8, 0.8])
            ax2.set_ylim([-0.8, 0.8])
            ax2.set_zlim([0, 1.2])
            ax2.view_init(elev=20, azim=45)
            ax2.set_title('End-Effector Trajectory')
            
            # Plot full trajectory
            ax2.plot3D(ee_trajectory[:, 0], ee_trajectory[:, 1], ee_trajectory[:, 2],
                       'b-', linewidth=1, alpha=0.3)
            
            # Highlight current path
            ax2.plot3D(ee_trajectory[:frame_idx, 0], 
                       ee_trajectory[:frame_idx, 1], 
                       ee_trajectory[:frame_idx, 2],
                       'r-', linewidth=2, label='Traveled')
            
            # Current position
            ax2.scatter([ee_trajectory[frame_idx, 0]], 
                       [ee_trajectory[frame_idx, 1]], 
                       [ee_trajectory[frame_idx, 2]],
                       c='red', s=100, marker='o')
        
        return fig,
    
    # Create animation
    anim = FuncAnimation(fig, update, init_func=init,
                        frames=len(trajectory), interval=50, blit=False)
    
    return anim


def main():
    """Main function to run the animation."""
    print("="*60)
    print("UR5 Robot Animation with Matplotlib")
    print("="*60)
    
    # Create animator
    animator = UR5Animator()
    
    # Define waypoints
    waypoints = [
        np.array([0, -np.pi/4, np.pi/2, -np.pi/4, -np.pi/2, 0]),
        np.array([np.pi/4, -np.pi/3, np.pi/3, -np.pi/6, -np.pi/2, 0]),
        np.array([np.pi/2, -np.pi/4, np.pi/4, 0, -np.pi/2, np.pi/4]),
        np.array([0, -np.pi/6, np.pi/3, -np.pi/6, -np.pi/2, 0]),
        np.array([0, -np.pi/4, np.pi/2, -np.pi/4, -np.pi/2, 0]),
    ]
    
    # Simulate trajectory
    trajectory = animator.simulate_trajectory(waypoints, steps_per_waypoint=500)
    
    # Create animation
    anim = create_animation(trajectory)
    
    # Save animation
    print("\nSaving animation to ur5_animation.gif...")
    try:
        anim.save('ur5_animation.gif', writer='pillow', fps=20)
        print("✓ Animation saved successfully!")
    except Exception as e:
        print(f"Could not save animation: {e}")
    
    # Show plot
    print("\nDisplaying animation window...")
    print("Close the window to continue...")
    plt.tight_layout()
    plt.show()
    
    # Plot static configurations
    plot_static_views(trajectory, waypoints)


def plot_static_views(trajectory, waypoints):
    """Create static plots of key configurations."""
    print("\nCreating static configuration plots...")
    
    fig = plt.figure(figsize=(15, 5))
    
    # Plot configurations at each waypoint
    n_waypoints = len(waypoints)
    frames_per_waypoint = len(trajectory) // n_waypoints
    
    for i in range(min(n_waypoints, 5)):  # Plot up to 5 waypoints
        ax = fig.add_subplot(1, 5, i+1, projection='3d')
        
        frame_idx = i * frames_per_waypoint
        if frame_idx >= len(trajectory):
            frame_idx = len(trajectory) - 1
        
        positions = trajectory[frame_idx]['positions']
        plot_robot_3d(ax, positions, color='blue', alpha=0.8)
        
        # Plot ground
        xx, yy = np.meshgrid(np.linspace(-0.5, 0.5, 2), np.linspace(-0.5, 0.5, 2))
        ax.plot_surface(xx, yy, np.zeros_like(xx), alpha=0.1, color='gray')
        
        ax.set_xlabel('X (m)')
        ax.set_ylabel('Y (m)')
        ax.set_zlabel('Z (m)')
        ax.set_xlim([-0.8, 0.8])
        ax.set_ylim([-0.8, 0.8])
        ax.set_zlim([0, 1.2])
        ax.view_init(elev=20, azim=45)
        ax.set_title(f'Waypoint {i+1}')
    
    plt.tight_layout()
    plt.savefig('ur5_configurations.png', dpi=150, bbox_inches='tight')
    print("✓ Static configurations saved to ur5_configurations.png")
    plt.show()


if __name__ == "__main__":
    main()
