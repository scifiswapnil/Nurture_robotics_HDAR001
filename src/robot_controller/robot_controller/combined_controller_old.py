import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist
import threading
import sys
import select
from math import pi, radians, degrees, sqrt, atan2
import time

class SwerveModule:
    """
    Control class for a single Differential Swerve Module.
    Uses 2 motors mixed to control Azimuth (Heading) and Wheel Velocity.
    """
    def __init__(self, parent_node, theta_id, left_id, right_id, invert_encoder=False):
        self.node = parent_node
        self.theta_id = theta_id
        self.left_id = left_id
        self.right_id = right_id
        self.invert_encoder = invert_encoder
        
        # --- Get Parameters from Parent ---
        self.wheel_base = self.node.get_parameter('wheel_base').value
        
        self.kp_h = self.node.get_parameter('kp_heading').value
        self.ki_h = self.node.get_parameter('ki_heading').value
        self.kd_h = self.node.get_parameter('kd_heading').value
        self.h_int_lim = self.node.get_parameter('heading_integral_limit').value

        self.kp_v = self.node.get_parameter('kp_vel').value
        self.ki_v = self.node.get_parameter('ki_vel').value
        self.kd_v = self.node.get_parameter('kd_vel').value
        
        self.max_rpm = self.node.get_parameter('max_rpm').value
        self.timeout = self.node.get_parameter('timeout_sec').value

        # --- Subscriptions/Publishers ---
        # Note: Using parent node to create pubs/subs
        self.sub_theta = self.node.create_subscription(
            Float64,
            f'/encoder_{self.theta_id}',
            self.encoder_callback,
            10
        )
        
        self.pub_left = self.node.create_publisher(Float64, f'/motor_{self.left_id}/target', 10)
        self.pub_right = self.node.create_publisher(Float64, f'/motor_{self.right_id}/target', 10)

        # --- State Variables ---
        self.current_heading = None 
        self.last_heading_time = 0.0
        
        self.target_heading = None # Absolute target angle
        self.target_vel_rpm = 0.0
        
        # PID State
        self.h_integral = 0.0
        self.h_prev_error = 0.0
        
        self.v_integral = 0.0
        self.v_prev_error = 0.0
        
        self.last_loop_time = time.time()
        self.command_active = False

        # --- Control Loop ---
        # Create a dedicated timer for this robot instance
        self.timer = self.node.create_timer(0.02, self.control_loop) # 50 Hz
        
        self.node.get_logger().info(f"Initialized Swerve Module: Enc={theta_id}, Motors={left_id}/{right_id}")

    def encoder_callback(self, msg):
        # Encoder provides radians, converting to degrees for internal logic
        heading = degrees(msg.data)
        if self.invert_encoder:
            heading = -heading
        self.current_heading = heading
        self.last_heading_time = time.time()

    def normalize_angle(self, angle):
        """Normalize angle to [-180, 180]"""
        while angle > 180:
            angle -= 360
        while angle <= -180:
            angle += 360
        return angle

    def optimize_state(self, current, target, speed):
        """
        Optimize module state to take shortest path.
        If target is > 90 deg away, flip 180 and invert speed.
        """
        diff = self.normalize_angle(target - current)
        
        # If the turn is large (> 90), reverse drive direction to reduce turn
        if abs(diff) > 90.0:
            target = self.normalize_angle(target + 180.0)
            speed = -speed
            
        return target, speed

    def set_command(self, target_deg, speed_rpm):
        """Public method to set target from parent"""
        if self.current_heading is None:
            self.node.get_logger().warn(f"Module {self.theta_id}: Cannot Execute - No heading data.")
            return

        # Optimize Target
        # Input target is absolute, current is absolute
        opt_target, opt_speed = self.optimize_state(self.current_heading, target_deg, speed_rpm)

        # Reset Integrators only on first activation to avoid reset spam
        if not self.command_active:
            self.h_integral = 0.0
            self.h_prev_error = 0.0
            self.v_integral = 0.0
            self.v_prev_error = 0.0
        
        # Calculate Target
        self.target_heading = opt_target
        self.target_vel_rpm = opt_speed
        
        self.command_active = True
        # self.node.get_logger().info(f"Module {self.theta_id}: Command Accepted -> Head {self.target_heading:.1f}, Speed {speed_rpm}")

    def control_loop(self):
        now = time.time()
        dt = now - self.last_loop_time
        self.last_loop_time = now
        
        # 1. Safety Check: Encoder Timeout
        if self.last_heading_time == 0.0 or (now - self.last_heading_time > self.timeout):
            if self.command_active:
                self.node.get_logger().warn(f"Module {self.theta_id}: Encoder timeout! Stopping motors.", throttle_duration_sec=2.0)
            self.stop_motors()
            return
            
        if not self.command_active or self.current_heading is None:
            self.stop_motors()
            return

        # 2. PID - Heading
        # Error = Target - Current (Normalized to shortest path)
        error_h = self.normalize_angle(self.target_heading - self.current_heading)
        
        self.h_integral += error_h * dt
        # Anti-windup
        self.h_integral = max(min(self.h_integral, self.h_int_lim), -self.h_int_lim)
        
        derivative_h = (error_h - self.h_prev_error) / dt if dt > 0 else 0.0
        self.h_prev_error = error_h
        
        omega = (self.kp_h * error_h) + (self.ki_h * self.h_integral) + (self.kd_h * derivative_h)

        # 3. Velocity Control (Open Loop)
        # We trust the motor driver to handle RPM setpoint.
        # PID here was incorrect as it treated target as error without feedback.
        V_out = self.target_vel_rpm

        # 4. Mixing
        adjust = (omega * self.wheel_base / 2.0)
        
        left_rpm = V_out - adjust
        right_rpm = V_out + adjust
        
        # 5. Clamping
        left_rpm = max(min(left_rpm, self.max_rpm), -self.max_rpm)
        right_rpm = max(min(right_rpm, self.max_rpm), -self.max_rpm)
        
        # 6. Publish
        msg_l = Float64()
        msg_l.data = float(left_rpm)
        msg_r = Float64()
        msg_r.data = float(-right_rpm)
        
        self.pub_left.publish(msg_l)
        self.pub_right.publish(msg_r)

    def stop_motors(self):
        msg = Float64()
        msg.data = 0.0
        self.pub_left.publish(msg)
        self.pub_right.publish(msg)


class SwerveDriveNode(Node):
    def __init__(self):
        super().__init__('swerve_drive_node')

        # --- Shared Parameters ---
        self.declare_parameter('wheel_base', 0.2)
        self.declare_parameter('track_width', 0.2) # Assuming square if not set, but explicit is better

                # PID Parameters
        self.declare_parameter('kp_heading', 2.75)
        self.declare_parameter('ki_heading', 1.0)
        self.declare_parameter('kd_heading', 0.0)
        self.declare_parameter('heading_integral_limit', 5.0)
        
        self.declare_parameter('kp_vel', 10.0)
        self.declare_parameter('ki_vel', 1.5)
        self.declare_parameter('kd_vel', 0.0)
        
        # Limits
        self.declare_parameter('max_rpm', 100.0)
        self.declare_parameter('timeout_sec', 0.5)

        # --- Initialize Robots ---
        self.modules = []
        self.modules.append(SwerveModule(self, "one", "one", "two", invert_encoder=False))
        self.modules.append(SwerveModule(self, "four", "seven", "eight", invert_encoder=False))
        self.modules.append(SwerveModule(self, "two", "three", "four", invert_encoder=False))
        self.modules.append(SwerveModule(self, "three", "five", "six", invert_encoder=False))
        
        #     # --- CLI Thread ---
    #     self.cli_thread = threading.Thread(target=self.cli_loop, daemon=True)
    #     self.cli_thread.start()
        
        # --- Subscribers ---
        self.sub_cmd_vel = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        self.get_logger().info("Swerve Drive Started. Listening on /cmd_vel")

    def cmd_vel_callback(self, msg):
        """
        Convert Twist (vx, vy, omega) into wheel speeds and angles.
        Kinematics for 4-wheel swerve.
        Modules:
        0: FL (+x, +y)
        1: FR (+x, -y)
        2: RL (-x, +y)
        3: RR (-x, -y)
        """
        vx = msg.linear.x
        vy = msg.linear.y
        omega = msg.angular.z
        
        L = self.get_parameter('wheel_base').value
        W = self.get_parameter('track_width').value
        
        # Calculate R = sqrt(L^2 + W^2) / 2 for normalizing omega if needed, 
        # but standard equations use component vectors:
        
        # Geometry vectors
        # A = vx - omega * (L/2)
        # B = vx + omega * (L/2)
        # C = vy - omega * (W/2)
        # D = vy + omega * (W/2)
        
        # Check signs depending on RF definition. 
        # Let's use:
        # FL (pos x, pos y) -> Vel = (vx - w*y, vy + w*x) = (vx - w*(W/2), vy + w*(L/2))
        # Wait, standard derivation:
        # STR = vy + omega * L/2
        # FWD = vx - omega * W/2
        # ...
        
        # Using a proven set:
        r = sqrt(L**2 + W**2) / 2.0
        
        # Tangential velocity components due to rotation
        # Vt = omega * r
        # Components at each wheel:
        # FL (+x, +y): (-omega*y, +omega*x)
        # FR (+x, -y): (-omega*y, +omega*x) -> y is negative -> (+, +)
        
        half_l = L / 2.0
        half_w = W / 2.0

        # V_wheel = V_robot + Omega x R_wheel
        
        # FL (x=L/2, y=W/2)
        # Vx_fl = vx - omega * half_w
        # Vy_fl = vy + omega * half_l
        
        # FR (x=L/2, y=-W/2)
        # Vx_fr = vx - omega * (-half_w) = vx + omega * half_w
        # Vy_fr = vy + omega * half_l
        
        # RL (x=-L/2, y=W/2)
        # Vx_rl = vx - omega * half_w
        # Vy_rl = vy + omega * (-half_l) = vy - omega * half_l
        
        # RR (x=-L/2, y=-W/2)
        # Vx_rr = vx - omega * (-half_w) = vx + omega * half_w
        # Vy_rr = vy + omega * (-half_l) = vy - omega * half_l
        
        wheels = [
            (vx - omega * half_w, vy + omega * half_l), # FL (0) 'one'
            (vx + omega * half_w, vy + omega * half_l), # FR (1) 'two'
            (vx - omega * half_w, vy - omega * half_l), # RL (2) 'three'
            (vx + omega * half_w, vy - omega * half_l), # RR (3) 'four'
        ]
        
        # Map to our module list order (Assuming order matches 0,1,2,3 as above)
        # Our list: one(0), two(1), three(2), four(3)
        # NOTE: User initialized:
        # 0: one (Front Left?)
        # 1: two (Front Right?)
        # 2: three (Rear Left?)
        # 3: four (Rear Right?)
        # We assume this mapping.
        
        for i, (wx, wy) in enumerate(wheels):
            speed = sqrt(wx**2 + wy**2)
            angle = degrees(atan2(wy, wx))
            
            # DDSM115 Specification
            # Diameter = 100mm = 0.1m
            # Max RPM = 100
            # RPM = (Velocity / Circumference) * 60
            # Circumference = pi * 0.1
            diameter = 0.1
            circumference = pi * diameter
            target_rpm = (speed / circumference) * 60.0
            # target_rpm = 0            
            self.get_logger().info(f"Speed: {speed:.2f} m/s -> {target_rpm:.2f} RPM")
            
            if i == 0 or i == 2:
                target_rpm = -target_rpm
            
            self.modules[i].set_command(angle, target_rpm)
            print(f"Module {i}: Angle {angle}, RPM {target_rpm}")


    def stop_all(self):
        for mod in self.modules:
            mod.stop_motors()


def main(args=None):
    rclpy.init(args=args)
    node = SwerveDriveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_all()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
