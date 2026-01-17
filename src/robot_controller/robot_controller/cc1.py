import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import threading
import sys
import select
from math import pi, radians, degrees
import time

class DifferentialDrivePID(Node):
    def __init__(self, theta_id, left_id, right_id):
        super().__init__('combined_controller')

        # --- Parameters ---
        self.declare_parameter('wheel_base', 0.2) # Meters? Units matter for consistency
        
        self.declare_parameter('kp_heading', 2.75)
        self.declare_parameter('ki_heading', 1.0)
        self.declare_parameter('kd_heading', 0.0)
        self.declare_parameter('heading_integral_limit', 5.0)
        
        self.declare_parameter('kp_vel', 20.0)
        self.declare_parameter('ki_vel', 1.5)
        self.declare_parameter('kd_vel', 0.0)
        
        # Limits
        self.declare_parameter('max_rpm', 100.0)
        self.declare_parameter('timeout_sec', 0.5) # Safety timeout

        # Get Parameters
        self.theta_id = theta_id
        self.left_id = left_id
        self.right_id = right_id
        self.wheel_base = self.get_parameter('wheel_base').value
        
        self.kp_h = self.get_parameter('kp_heading').value
        self.ki_h = self.get_parameter('ki_heading').value
        self.kd_h = self.get_parameter('kd_heading').value
        self.h_int_lim = self.get_parameter('heading_integral_limit').value

        self.kp_v = self.get_parameter('kp_vel').value
        self.ki_v = self.get_parameter('ki_vel').value
        self.kd_v = self.get_parameter('kd_vel').value
        
        self.max_rpm = self.get_parameter('max_rpm').value
        self.timeout = self.get_parameter('timeout_sec').value

        # --- Subscriptions/Publishers ---
        self.sub_theta = self.create_subscription(
            Float64,
            f'/encoder_{self.theta_id}',
            self.encoder_callback,
            10
        )
        
        self.pub_left = self.create_publisher(Float64, f'/motor_{self.left_id}/target', 10)
        self.pub_right = self.create_publisher(Float64, f'/motor_{self.right_id}/target', 10)

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
        self.timer = self.create_timer(0.05, self.control_loop) # 20 Hz
        
        # --- CLI Thread ---
        self.cli_thread = threading.Thread(target=self.cli_loop, daemon=True)
        self.cli_thread.start()
        
        self.get_logger().info("Combined Controller Started.")
        self.get_logger().info("Enter commands formatted as: <target_angle_deg> <speed_rpm>")
        self.get_logger().info("Example: '30 10' -> Head to 30 deg abs, move at 10 RPM")

    def encoder_callback(self, msg):
        # Encoder provides radians, converting to degrees for internal logic
        self.current_heading = degrees(msg.data)
        self.last_heading_time = time.time()

    def normalize_angle(self, angle):
        """Normalize angle to [-180, 180]"""
        while angle > 180:
            angle -= 360
        while angle <= -180:
            angle += 360
        return angle

    def control_loop(self):
        now = time.time()
        dt = now - self.last_loop_time
        self.last_loop_time = now
        
        # 1. Safety Check: Encoder Timeout
        if self.last_heading_time == 0.0 or (now - self.last_heading_time > self.timeout):
            if self.command_active:
                self.get_logger().warn("Encoder timeout! Stopping motors.", throttle_duration_sec=2.0)
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

        # 3. PID - Velocity
        # Since we don't have velocity feedback mentioned in the prompt besides "user commanded RPM",
        # we treat the error as (TargetRPM - 0) if we want to just "push" it, or more likely,
        # the prompt asks for a PID loop controlling forward speed.
        # Without speed feedback, a PID is degenerate (Open Loop). 
        # We will assume we are controlling 'V' parameter for the mix, treating target_vel_rpm as the setpoint.
        # Ideally we'd have estimated velocity from encoders. 
        # For this implementation, we will apply the PID logic to the Target vs '0' assumption 
        # OR just pass it through if Kp=1, Ki=0, Kd=0. 
        # Let's effectively make it a "Soft Start" or just a direct mapper if Kp=1.
        
        error_v = self.target_vel_rpm # Assuming current forward vel is effectively 'driven' to this
        # Note: A real PID needs feedback. 
        
        self.v_integral += error_v * dt
        derivative_v = (error_v - self.v_prev_error) / dt if dt > 0 else 0.0
        self.v_prev_error = error_v
        
        V_out = (self.kp_v * error_v) + (self.ki_v * self.v_integral) + (self.kd_v * derivative_v)

        # 4. Mixing
        # Standard Differential Drive:
        # left = v - (w * base / 2)
        # right = v + (w * base / 2)
        # Variable Mapping: V_out is forward RPM proxy, omega is rotation correction
        
        # omega here is likely high if error is high. We need to scale it to RPM or similar units.
        # If kp_heading is tuned, omega is directly "adjustment RPM".
        
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

    def cli_loop(self):
        """Thread to handle non-blocking CLI input."""
        while rclpy.ok():
            try:
                # Select allows us to wait for input with a timeout, so we can check rclpy.ok()
                if sys.stdin in select.select([sys.stdin], [], [], 1)[0]:
                    line = sys.stdin.readline()
                    if line:
                        self.process_command(line.strip())
            except Exception as e:
                self.get_logger().error(f"CLI Error: {e}")

    def process_command(self, cmd_str):
        try:
            parts = cmd_str.split()
            if len(parts) == 2:
                target_theta = float(parts[0])
                speed_rpm = float(parts[1])
                
                if self.current_heading is None:
                    self.get_logger().warn("Cannot Execute: No heading data received yet.")
                    return

                # Calculate Target
                self.target_heading = self.normalize_angle(target_theta)
                self.target_vel_rpm = speed_rpm
                
                # Reset Integrators
                self.h_integral = 0.0
                self.h_prev_error = 0.0
                self.v_integral = 0.0
                self.v_prev_error = 0.0
                
                self.command_active = True
                self.get_logger().info(f"Command Accepted: Head {self.target_heading:.1f}, Speed {speed_rpm}")
            else:
                self.get_logger().warn("Invalid Format. Use: <target_deg> <speed_rpm>")
        except ValueError:
            self.get_logger().warn("Invalid Numbers. Use numeric values.")

def main(args=None):
    rclpy.init(args=args)
    # node_one = DifferentialDrivePID("one", "one", "two")
    # node_two = DifferentialDrivePID("two", "three", "four")
    node_three = DifferentialDrivePID("three", "five", "six")
    # node_four = DifferentialDrivePID("four", "seven", "eight")
    
    try:
        # rclpy.spin(node_one)
        # rclpy.spin(node_two)
        rclpy.spin(node_three)
        # rclpy.spin(node_four)
    except KeyboardInterrupt:
        pass
    finally:
        # node_one.stop_motors()
        # node_one.destroy_node()
        # node_two.stop_motors()
        # node_two.destroy_node()
        node_three.stop_motors()
        node_three.destroy_node()
        # node_four.stop_motors()
        # node_four.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
