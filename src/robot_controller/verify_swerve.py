
import math
import sys
# Add src to path just in case
sys.path.append('/home/robotboy/nuture_robotics/src/robot_controller')

from robot_controller.swerve_math import SwerveKinematics

def test():
    # Initialize
    k = SwerveKinematics(0.5, 0.5, continuous_steering=True)

    # Test 1: Forward (Angle 0) -> Forward (Angle 0)
    # Current = 0. Target = 0. Speed = 1.
    grad, ang, flipped = k.optimize_with_current_state(1.0, 0.0, 0.0)
    print(f"Test 1 (0->0): Speed={grad}, Angle={ang}, Flipped={flipped}")

    # Test 2: Turn 180 (Angle 0 -> Angle pi)
    # Current = 0. Target = pi. Speed = 1.
    grad, ang, flipped = k.optimize_with_current_state(1.0, math.pi, 0.0)
    print(f"Test 2 (0->pi): Speed={grad}, Angle={ang}, Flipped={flipped}")

    # Test 3: Left (Angle pi/2) -> Right (Angle -pi/2)
    # Current = pi/2. Target = -pi/2. Speed = 1.
    grad, ang, flipped = k.optimize_with_current_state(1.0, -math.pi/2, math.pi/2)
    print(f"Test 3 (pi/2->-pi/2): Speed={grad}, Angle={ang}, Flipped={flipped}")

if __name__ == "__main__":
    test()
