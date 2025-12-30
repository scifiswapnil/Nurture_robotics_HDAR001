import math


class SwerveKinematics:
    def __init__(self,
                 wheel_base,
                 track_width,
                 min_steering_angle=None,
                 max_steering_angle=None,
                 continuous_steering=True):
        """
        Initialize Swerve Kinematics.

        :param wheel_base: Distance between front and back wheels (meters)
        :param track_width: Distance between left and right wheels (meters)
        :param min_steering_angle: Check limits if continuous_steering is False (radians)
        :param max_steering_angle: Check limits if continuous_steering is False (radians)
        :param continuous_steering: If True, allows 360 degree steering and optimization.
        """
        self.L = wheel_base
        self.W = track_width
        self.min_steer = min_steering_angle
        self.max_steer = max_steering_angle
        self.continuous = continuous_steering

        # Wheel positions relative to center: FL, FR, BL, BR
        # FL: (+L/2, +W/2)
        # FR: (+L/2, -W/2)
        # BL: (-L/2, +W/2)
        # BR: (-L/2, -W/2)
        # Note: Standard ROS frame: X forward, Y left.
        # So Front Left is (+x, +y).

    def calculate_wheel_states(self, vx, vy, omega):
        """
        Calculate target steering angles and wheel velocities.

        :param vx: Linear velocity in X (m/s)
        :param vy: Linear velocity in Y (m/s)
        :param omega: Angular velocity (rad/s)
        :return: List of tuples [(speed, angle), ...] for FL, FR, BL, BR
        """
        L = self.L
        W = self.W

        # Based on standard swerve drive kinematics
        # r_FL = (L/2, W/2)
        # v_FL_x = vx - omega * (W/2)
        # v_FL_y = vy + omega * (L/2)

        half_l = L / 2.0
        half_w = W / 2.0

        # FL
        vx_fl = vx - omega * half_w
        vy_fl = vy + omega * half_l

        # FR
        vx_fr = vx + omega * half_w
        vy_fr = vy + omega * half_l

        # BL
        vx_bl = vx - omega * half_w
        vy_bl = vy - omega * half_l

        # BR
        vx_br = vx + omega * half_w
        vy_br = vy - omega * half_l

        wheels = [
            (vx_fl, vy_fl),  # FL
            (vx_fr, vy_fr),  # FR
            (vx_bl, vy_bl),  # BL
            (vx_br, vy_br)   # BR
        ]

        results = []
        for (w_vx, w_vy) in wheels:
            speed = math.sqrt(w_vx**2 + w_vy**2)
            angle = math.atan2(w_vy, w_vx)

            if self.continuous:
                speed, angle = self._optimize_steering(speed, angle)
            else:
                angle = self._clamp_angle(angle)

            results.append((speed, angle))

        return results

    def _optimize_steering(self, speed, angle):
        """
        Optimize steering angle to minimize rotation.

        This is state-dependent in a real controller, but for a simple kinematic calculator
        we usually assume we want the 'standard' angle.
        However, usually 'optimization' requires knowing the CURRENT angle to
        decide whether to flip.

        For this pure kinematic function, we return the calculated angle.
        But if 'continuous' is enabled, we might assume the caller handles the
        wrapping logic or we just return the raw atan2 angle (-pi to pi).
        """
        return speed, angle

    def _clamp_angle(self, angle):
        """Clamp angle to limits if set."""
        if self.min_steer is not None:
            angle = max(self.min_steer, angle)
        if self.max_steer is not None:
            angle = min(self.max_steer, angle)
        return angle

    def optimize_with_current_state(self, target_speed, target_angle, current_angle):
        """
        Optimize based on current angle.

        Finds the closest equivalent angle (can reverse speed).
        This implementation minimizes total rotation (handling winding).
        """
        if not self.continuous:
            return target_speed, self._clamp_angle(target_angle)

        # Calculate the difference between target and current
        diff = target_angle - current_angle

        # Normalize the difference to the range [-pi, pi]
        # This gives the smallest angle to turn to get to target_angle modulus 2pi
        diff = (diff + math.pi) % (2 * math.pi) - math.pi

        flipped = False
        # If the rotation is more than 90 degrees (pi/2), it is closer to
        # rotate to the "opposite" angle and reverse speed using the flipped logic.
        if abs(diff) > math.pi / 2:
            # We want to go to the opposite valid angle
            # The angle difference to the opposite angle is (diff - pi) or (diff + pi)
            # We adjust diff to be the offset to that opposite angle
            if diff > 0:
                diff -= math.pi
            else:
                diff += math.pi
            
            # Reverse speed
            target_speed = -target_speed
            flipped = True

        # The final target angle is the current angle plus the minimized difference
        final_angle = current_angle + diff
        
        return target_speed, final_angle, flipped
