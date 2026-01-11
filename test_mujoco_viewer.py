#!/usr/bin/env python3
"""
MuJoCo Viewer Diagnostic Tool for macOS
Run this to diagnose why the MuJoCo viewer might not be working
"""

import sys
import os

print("="*60)
print("MuJoCo Viewer Diagnostic Tool")
print("="*60)

# Test 1: Check Python version
print("\n1. Python Version:")
print(f"   {sys.version}")
print(f"   Executable: {sys.executable}")

# Test 2: Check if running in proper environment
print("\n2. Environment Check:")
if os.environ.get('SSH_CONNECTION'):
    print("   ⚠️  WARNING: Running over SSH - viewer won't work")
else:
    print("   ✓ Not running over SSH")

if os.environ.get('DISPLAY'):
    print(f"   DISPLAY: {os.environ['DISPLAY']}")
else:
    print("   DISPLAY: Not set (normal for macOS native)")

# Test 3: Check MuJoCo installation
print("\n3. MuJoCo Installation:")
try:
    import mujoco
    print(f"   ✓ MuJoCo installed: version {mujoco.__version__}")
except ImportError as e:
    print(f"   ✗ MuJoCo not installed: {e}")
    print("   → Install with: pip install mujoco")
    sys.exit(1)

# Test 4: Check viewer module
print("\n4. Viewer Module:")
try:
    import mujoco.viewer
    print("   ✓ mujoco.viewer module found")
except ImportError as e:
    print(f"   ✗ mujoco.viewer not found: {e}")
    sys.exit(1)

# Test 5: Check GLFW
print("\n5. GLFW (Graphics Library):")
try:
    import glfw
    print(f"   ✓ GLFW installed: version {glfw.get_version_string()}")
    
    # Try to initialize GLFW
    if glfw.init():
        print("   ✓ GLFW initialized successfully")
        glfw.terminate()
    else:
        print("   ✗ GLFW failed to initialize")
        print("   → Try: brew install glfw")
except ImportError:
    print("   ⚠️  GLFW not found (MuJoCo has its own GLFW)")
    print("   → Optional: brew install glfw")

# Test 6: Load a simple model
print("\n6. Loading Test Model:")
try:
    xml = """
    <mujoco>
        <worldbody>
            <light diffuse=".5 .5 .5" pos="0 0 3" dir="0 0 -1"/>
            <geom type="plane" size="1 1 0.1" rgba=".9 0 0 1"/>
            <body pos="0 0 1">
                <joint type="free"/>
                <geom type="box" size=".1 .1 .1" rgba="0 .9 0 1"/>
            </body>
        </worldbody>
    </mujoco>
    """
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    print("   ✓ Model loaded successfully")
except Exception as e:
    print(f"   ✗ Failed to load model: {e}")
    sys.exit(1)

# Test 7: Try to launch viewer
print("\n7. Testing Viewer Launch:")
print("   Attempting to open viewer window...")
print("   (This will timeout in 5 seconds if it doesn't work)")

try:
    import signal
    
    def timeout_handler(signum, frame):
        raise TimeoutError("Viewer didn't open in time")
    
    # Set alarm for 5 seconds
    signal.signal(signal.SIGALRM, timeout_handler)
    signal.alarm(5)
    
    try:
        with mujoco.viewer.launch_passive(model, data) as viewer:
            print("   ✓✓✓ SUCCESS! Viewer opened!")
            print("   Close the viewer window to continue...")
            
            # Keep window open until user closes it
            while viewer.is_running():
                mujoco.mj_step(model, data)
                viewer.sync()
        
        print("   ✓ Viewer closed cleanly")
        
    except TimeoutError:
        print("   ✗ Viewer failed to open (timeout)")
        print("\n   Possible fixes:")
        print("   1. Run directly in Terminal.app (not SSH)")
        print("   2. brew install glfw")
        print("   3. pip install --upgrade mujoco")
        print("   4. Try: export MUJOCO_GL=glfw")
        print("   5. Use alternative: python ur5_animation.py")
        
    finally:
        signal.alarm(0)  # Cancel alarm
        
except Exception as e:
    print(f"   ✗ Error: {e}")
    print("\n   → Use alternative: python ur5_animation.py")

print("\n" + "="*60)
print("Diagnostic Complete")
print("="*60)

# Final recommendation
print("\n📝 Recommendation:")
if os.environ.get('SSH_CONNECTION'):
    print("   You're on SSH - use: python ur5_animation.py")
else:
    print("   If viewer didn't open, use: python ur5_animation.py")
    print("   (Matplotlib-based visualization always works)")
