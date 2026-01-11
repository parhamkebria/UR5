#!/usr/bin/env python3
"""
UR5 Robot Simulation Launcher
Choose between different visualization and simulation options
"""

import sys
import subprocess

def print_menu():
    print("="*60)
    print("UR5 Robot Simulation")
    print("="*60)
    print("\nAvailable options:")
    print("  1. Matplotlib Animation (Recommended - Always works)")
    print("     - 3D visualization with animation")
    print("     - Saves GIF and PNG")
    print("     Command: python ur5_animation.py")
    print()
    print("  2. MuJoCo Interactive Viewer (May require display)")
    print("     - Real-time interactive 3D viewer")
    print("     - Rotate/zoom with mouse")
    print("     Command: python ur5_simulation.py")
    print()
    print("  3. No Viewer Mode (Headless)")
    print("     - Runs simulation without graphics")
    print("     - Prints dynamics info")
    print("     Command: python ur5_simulation.py no-viewer")
    print()
    print("  4. PyBullet Simulation (Alternative)")
    print("     - Alternative physics engine")
    print("     Command: python ur5_simulation_pybullet.py")
    print()
    print("="*60)

if __name__ == "__main__":
    print_menu()
    
    if len(sys.argv) > 1:
        choice = sys.argv[1]
    else:
        choice = input("\nEnter choice (1-4) or press Enter for default [1]: ").strip()
        if not choice:
            choice = "1"
    
    if choice == "1":
        print("\n▶ Launching Matplotlib Animation...")
        subprocess.run([sys.executable, "ur5_animation.py"])
    elif choice == "2":
        print("\n▶ Launching MuJoCo Interactive Viewer...")
        subprocess.run([sys.executable, "ur5_simulation.py"])
    elif choice == "3":
        print("\n▶ Running No Viewer Mode...")
        subprocess.run([sys.executable, "ur5_simulation.py", "no-viewer"])
    elif choice == "4":
        print("\n▶ Launching PyBullet Simulation...")
        subprocess.run([sys.executable, "ur5_simulation_pybullet.py"])
    else:
        print(f"\n❌ Invalid choice: {choice}")
        sys.exit(1)
