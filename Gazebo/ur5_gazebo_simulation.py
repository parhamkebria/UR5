#!/usr/bin/env python3
"""
UR5 Robot Simulation using Gazebo (ROS-free, Linux only)
=========================================================

This script provides a standalone Gazebo simulation of the UR5 robot without ROS dependencies.
Uses PyGazebo for Python bindings to Gazebo server.

Platform Support:
-----------------
✅ Linux (Ubuntu 20.04+, Debian 11+): Fully supported
❌ macOS: Gazebo 11 deprecated, won't build on modern macOS
❌ Windows: Use WSL2 or alternatives

Alternatives for macOS/Windows:
-------------------------------
- MuJoCo: mjpython ur5_simulation.py (best for macOS)
- Matplotlib: python ur5_animation.py (works everywhere)

Requirements (Linux):
--------------------
- Gazebo Classic 11: sudo apt install gazebo11 libgazebo11-dev
- pygazebo: pip install pygazebo
- protobuf: pip install protobuf
- asyncio (built-in Python 3.7+)

Usage:
------
Linux:
    python ur5_gazebo_simulation.py

macOS (alternative):
    mjpython ur5_simulation.py

Windows (alternatives):
    python ur5_animation.py
    # or use WSL2 with Linux instructions

Features (Linux only):
---------------------
- Realistic physics simulation
- Beautiful 3D visualization with lighting and shadows
- Joint position control via Python
- State monitoring (joint positions, velocities)
- No ROS dependency (standalone)
- Industry-standard simulation platform

Controls:
---------
- The simulation will move the robot through predefined waypoints
- Joint states are printed to console
- Use mouse in Gazebo window: drag to rotate, scroll to zoom
- Close Gazebo window to exit
"""

import asyncio
import sys
import time
import numpy as np
from pygazebo import Manager
from pygazebo.msg import joint_cmd_pb2, model_v_pb2
import subprocess
import os
import signal

class UR5GazeboSimulation:
    def __init__(self, model_name='ur5'):
        """Initialize the UR5 Gazebo simulation."""
        self.model_name = model_name
        self.manager = None
        self.joint_names = [
            'ur5::shoulder_pan_joint',
            'ur5::shoulder_lift_joint', 
            'ur5::elbow_joint',
            'ur5::wrist_1_joint',
            'ur5::wrist_2_joint',
            'ur5::wrist_3_joint'
        ]
        self.current_positions = np.zeros(6)
        self.gazebo_process = None
        
    async def connect(self):
        """Connect to Gazebo server."""
        print("Connecting to Gazebo...")
        self.manager = await Manager.create()
        
        # Subscribe to joint state updates
        self.subscriber = await self.manager.subscribe(
            '/gazebo/default/ur5/joint_state',
            'gazebo.msgs.Model',
            self._joint_state_callback
        )
        
        # Create publisher for joint commands
        self.publisher = await self.manager.advertise(
            '/gazebo/default/ur5/joint_cmd',
            'gazebo.msgs.JointCmd'
        )
        
        print("Connected to Gazebo!")
        await asyncio.sleep(1)  # Wait for connection to stabilize
        
    def _joint_state_callback(self, data):
        """Callback for joint state messages."""
        try:
            model = model_v_pb2.Model()
            model.ParseFromString(data)
            
            for i, joint in enumerate(model.joint):
                if i < 6:
                    self.current_positions[i] = joint.angle[0]
        except Exception as e:
            print(f"Error parsing joint state: {e}")
    
    async def set_joint_positions(self, positions, duration=2.0):
        """
        Move joints to target positions smoothly.
        
        Args:
            positions: numpy array of 6 joint angles (radians)
            duration: time to reach target (seconds)
        """
        if len(positions) != 6:
            raise ValueError("Must provide 6 joint positions")
        
        print(f"\nMoving to: {np.degrees(positions).round(1)}°")
        
        # Send position commands for each joint
        for i, (joint_name, target_pos) in enumerate(zip(self.joint_names, positions)):
            msg = joint_cmd_pb2.JointCmd()
            msg.name = joint_name
            msg.position = target_pos
            msg.velocity = 0.5  # rad/s
            msg.force = 100.0  # Maximum effort
            
            await self.publisher.publish(msg)
        
        # Wait for motion to complete
        await asyncio.sleep(duration)
        
        # Print current state
        print(f"Current positions: {np.degrees(self.current_positions).round(1)}°")
    
    def start_gazebo(self, world_file='ur5_world.sdf', model_file='ur5_robot.urdf'):
        """
        Start Gazebo with the UR5 model.
        
        Args:
            world_file: Path to the world SDF file
            model_file: Path to the robot URDF file
        """
        print("Starting Gazebo...")
        
        # Check if files exist
        if not os.path.exists(world_file):
            print(f"Warning: World file '{world_file}' not found. Using empty world.")
            world_file = None
        
        if not os.path.exists(model_file):
            raise FileNotFoundError(f"Robot model file '{model_file}' not found!")
        
        # Convert URDF to SDF on-the-fly (Gazebo can load URDF directly)
        gazebo_cmd = ['gazebo', '--verbose']
        
        if world_file:
            gazebo_cmd.append(world_file)
        
        # Start Gazebo in background
        try:
            self.gazebo_process = subprocess.Popen(
                gazebo_cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            print("Gazebo started! Waiting for initialization...")
            time.sleep(5)  # Give Gazebo time to start
            
            # Spawn the robot model
            spawn_cmd = [
                'gz', 'model',
                '-f', model_file,
                '-m', 'ur5',
                '-x', '0', '-y', '0', '-z', '0.05'
            ]
            
            result = subprocess.run(
                spawn_cmd,
                capture_output=True,
                text=True
            )
            
            if result.returncode == 0:
                print("UR5 robot spawned successfully!")
            else:
                print(f"Warning: Could not spawn robot: {result.stderr}")
                print("Trying alternative spawn method...")
                
        except FileNotFoundError:
            print("\n❌ Gazebo not found! Please install Gazebo:")
            print("   Ubuntu/Debian: sudo apt install gazebo11 libgazebo11-dev")
            print("   macOS: brew install gazebo")
            sys.exit(1)
        except Exception as e:
            print(f"Error starting Gazebo: {e}")
            sys.exit(1)
    
    def stop_gazebo(self):
        """Stop the Gazebo process."""
        if self.gazebo_process:
            print("\nStopping Gazebo...")
            self.gazebo_process.send_signal(signal.SIGTERM)
            self.gazebo_process.wait(timeout=5)
            print("Gazebo stopped.")
    
    async def run_demo(self):
        """Run a demonstration of the UR5 robot moving through waypoints."""
        print("\n" + "="*60)
        print("UR5 Gazebo Simulation Demo")
        print("="*60)
        
        # Define waypoints (joint configurations in radians)
        waypoints = [
            np.array([0, -np.pi/4, np.pi/2, -np.pi/4, -np.pi/2, 0]),
            np.array([np.pi/4, -np.pi/3, np.pi/3, -np.pi/6, -np.pi/2, 0]),
            np.array([np.pi/2, -np.pi/4, np.pi/4, -np.pi/3, -np.pi/2, np.pi/4]),
            np.array([0, 0, 0, 0, 0, 0])  # Return to home
        ]
        
        try:
            # Connect to Gazebo
            await self.connect()
            
            # Move through waypoints
            for i, waypoint in enumerate(waypoints):
                print(f"\n--- Waypoint {i+1}/{len(waypoints)} ---")
                await self.set_joint_positions(waypoint, duration=3.0)
                await asyncio.sleep(1.0)  # Pause between waypoints
            
            print("\n✓ Demo complete!")
            print("\nPress Ctrl+C to exit...")
            
            # Keep running until interrupted
            while True:
                await asyncio.sleep(1.0)
                
        except KeyboardInterrupt:
            print("\n\nShutting down...")
        except Exception as e:
            print(f"\n❌ Error during demo: {e}")
            print("\nTroubleshooting:")
            print("1. Make sure Gazebo is running")
            print("2. Check that the robot model spawned correctly")
            print("3. Verify pygazebo is installed: pip install pygazebo")
        finally:
            if self.manager:
                await self.manager.close()

def check_dependencies():
    """Check if required dependencies are installed."""
    print("Checking dependencies...")
    
    # Check Gazebo
    try:
        result = subprocess.run(['gazebo', '--version'], 
                              capture_output=True, text=True)
        print(f"✓ Gazebo found: {result.stdout.strip()}")
    except FileNotFoundError:
        print("✗ Gazebo not found")
        print("\nInstallation instructions:")
        print("  Ubuntu/Debian: sudo apt install gazebo11 libgazebo11-dev")
        print("  macOS: brew tap osrf/simulation && brew install gazebo11")
        return False
    
    # Check Python packages
    try:
        import pygazebo
        print(f"✓ pygazebo found")
    except ImportError:
        print("✗ pygazebo not found")
        print("  Install: pip install pygazebo")
        return False
    
    return True

def main():
    """Main entry point."""
    print("""
╔══════════════════════════════════════════════════════════════╗
║           UR5 Robot - Gazebo Simulation (ROS-free)           ║
╚══════════════════════════════════════════════════════════════╝
""")
    
    # Check dependencies
    if not check_dependencies():
        print("\n❌ Missing dependencies. Please install them first.")
        sys.exit(1)
    
    print("\n" + "="*60)
    
    # Create simulation
    sim = UR5GazeboSimulation()
    
    try:
        # Start Gazebo
        sim.start_gazebo()
        
        # Run demonstration
        asyncio.run(sim.run_demo())
        
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    except Exception as e:
        print(f"\n❌ Error: {e}")
    finally:
        # Clean up
        sim.stop_gazebo()
        print("\nGoodbye!")

if __name__ == '__main__':
    main()
