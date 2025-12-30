import math

from robot_controller.swerve_math import SwerveKinematics


def test_initialization():
    kinematics = SwerveKinematics(0.5, 0.5)
    assert kinematics.L == 0.5
    assert kinematics.W == 0.5
    assert kinematics.continuous is True


def test_straight_forward():
    # Forward motion (X only)
    k = SwerveKinematics(1.0, 1.0)
    # vx=1, vy=0, omega=0
    results = k.calculate_wheel_states(1.0, 0.0, 0.0)

    for (speed, angle) in results:
        assert math.isclose(speed, 1.0, rel_tol=1e-5)
        # Angle should be 0 (pointing in X)
        assert math.isclose(angle, 0.0, rel_tol=1e-5)


def test_strafing_left():
    # Strafing left (Y only)
    k = SwerveKinematics(1.0, 1.0)
    # vx=0, vy=1, omega=0
    results = k.calculate_wheel_states(0.0, 1.0, 0.0)

    for (speed, angle) in results:
        assert math.isclose(speed, 1.0, rel_tol=1e-5)
        # Angle should be PI/2
        assert math.isclose(angle, math.pi / 2, rel_tol=1e-5)


def test_inplace_rotation():
    # Rotate in place (omega only)
    L = 1.0
    W = 1.0
    k = SwerveKinematics(L, W)
    omega = 1.0
    results = k.calculate_wheel_states(0.0, 0.0, omega)

    # Radius to wheel is sqrt(0.5^2 + 0.5^2) = sqrt(0.5) = 0.707
    r = math.sqrt((L / 2)**2 + (W / 2)**2)
    expected_speed = omega * r

    # FL: (+0.5, +0.5). Tangent is perpendicular to radius.
    # Radius vector angle is 45 deg (pi/4). Tangent (rotation) is +90 -> 135 deg (3pi/4).
    expected_angle_fl = 3 * math.pi / 4

    # FR: (+0.5, -0.5). Angle -45. Tangent +45 (pi/4).
    expected_angle_fr = math.pi / 4

    # BL: (-0.5, +0.5). Angle 135. Tangent 225 (-135) -> -3pi/4.
    expected_angle_bl = -3 * math.pi / 4

    # BR: (-0.5, -0.5). Angle -135. Tangent -45 -> -pi/4.
    expected_angle_br = -math.pi / 4

    expected_angles = [expected_angle_fl, expected_angle_fr, expected_angle_bl, expected_angle_br]

    for i, (speed, angle) in enumerate(results):
        assert math.isclose(speed, expected_speed, rel_tol=1e-5)
        assert math.isclose(angle, expected_angles[i], rel_tol=1e-5)


def test_optimization_flip():
    # Test that optimization flips speed if needed
    k = SwerveKinematics(1.0, 1.0, continuous_steering=True)

    # Current angle 0. Target angle PI. Should stay 0 and flip speed.
    speed, angle = k.optimize_with_current_state(1.0, math.pi, 0.0)
    # Normalized diff is pi. abs(diff) > pi/2 is true. 
    # diff becomes 0.
    # angle = 0 + 0 = 0.
    assert math.isclose(angle, 0.0, abs_tol=1e-5)
    assert math.isclose(speed, -1.0, rel_tol=1e-5)

    # Current angle 0. Target PI/2. Should go to PI/2, speed positive.
    speed, angle = k.optimize_with_current_state(1.0, math.pi / 2, 0.0)
    assert math.isclose(angle, math.pi / 2, abs_tol=1e-5)
    assert math.isclose(speed, 1.0, rel_tol=1e-5)

    # Test winding: Current 720 (4pi). Target 10 deg (0.1745)
    # 4pi is 2 revolutions. Target is slightly above 0.
    # Should rotate to approx 0.1745 (wrapped)
    curr = 4 * math.pi
    target = 0.1745
    speed, angle = k.optimize_with_current_state(1.0, target, curr)
    
    # Expected angle is the target itself because 4pi + 0.1745 wrapped is 0.1745
    expected_angle = 0.1745
    assert math.isclose(angle, expected_angle, abs_tol=1e-5)
    assert math.isclose(speed, 1.0, rel_tol=1e-5)


def test_optimization_limit():
    # If continuous is False, should clamp
    k = SwerveKinematics(1.0, 1.0,
                         continuous_steering=False,
                         min_steering_angle=-1.0,
                         max_steering_angle=1.0)

    # Target 2.0. Should clamp to 1.0
    # Strafe left: angle pi/2 approx 1.57 usually
    # But calculate_wheel_states calls _clamp_angle internally if continuous=False

    # Let's check calculate_wheel_states with clamp
    # strafe left is 1.57 rad. Max is 1.0.
    results = k.calculate_wheel_states(0.0, 1.0, 0.0)
    speed, angle = results[0]
    assert angle == 1.0  # Clamped
