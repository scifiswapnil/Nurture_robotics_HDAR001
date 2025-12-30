
import math
import sys
import time

# Mock rclpy and geometry_msgs for standalone testing
from unittest.mock import MagicMock
sys.modules['rclpy'] = MagicMock()
sys.modules['rclpy.node'] = MagicMock()
sys.modules['geometry_msgs'] = MagicMock()
sys.modules['geometry_msgs.msg'] = MagicMock()

sys.path.append('/home/robotboy/nuture_robotics/src/robot_controller')
from robot_controller.simple_swerve_node import DiffSwerveModule

def test():
    print("--- Test 1: Initialize ---")
    mod = DiffSwerveModule("TestMod", 0.5, 0.5, steer_kp=1.0, max_steer_speed=5.0)
    print(f"Module {mod.name} initialized. Angle={mod.current_angle}")

    print("\n--- Test 2: Drive Forward (No Steer Needed) ---")
    # Robot V = (1, 0, 0). Wheel is at (0.5, 0.5).
    # Wheel Vx = 1 - 0 = 1. Vy = 0.
    # Target Angle = 0. Target Speed = 1.
    vl, vr = mod.update(1.0, 0.0, 0.0, dt=0.02)
    print(f"Update 1: Cur={mod.current_angle:.2f}, Trg={mod.target_angle:.2f}, VL={vl:.2f}, VR={vr:.2f}")
    assert abs(mod.target_angle) < 0.001, "Target angle should be 0"
    assert abs(mod.target_speed - 1.0) < 0.001, "Target speed should be 1"

    print("\n--- Test 3: Strafe Left (90 deg turn) ---")
    # Robot V = (0, 1, 0).
    # Target Angle = pi/2 (1.57).
    # Current Angle = 0.
    # Error = 1.57.
    # Max Step = 5.0 * 0.02 = 0.1 rad.
    # New Angle should be 0 + 0.1 = 0.1.
    vl, vr = mod.update(0.0, 1.0, 0.0, dt=0.02)
    print(f"Update 2: Cur={mod.current_angle:.2f}, Trg={mod.target_angle:.2f}, VL={vl:.2f}, VR={vr:.2f}")
    
    assert mod.current_angle > 0.05, "Should have moved towards target"
    assert mod.current_angle < mod.target_angle, "Should not have reached target yet"

    print("\n--- Test 4: Convergence ---")
    # Run loop until converged
    for i in range(20):
        mod.update(0.0, 1.0, 0.0, dt=0.02)
    
    print(f"Update 20: Cur={mod.current_angle:.2f}, Trg={mod.target_angle:.2f}")
    assert abs(mod.current_angle - mod.target_angle) < 0.01, "Should have converged"

if __name__ == "__main__":
    test()
