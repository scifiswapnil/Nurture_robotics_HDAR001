import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import threading
import sys
import select
from math import pi, radians, degrees
import time

class DifferentialDrivePID:
    """
    Helper class for a SINGLE robot's differential drive logic.
    Not a Node itself, but attached to a parent Node.
    """
    def __init__(self, parent_node, theta_id, left_id, right_id):
        self.node = parent_node
        self.theta_id = theta_id
        self.left_id = left_id
        self.right_id = right_id
        
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
        self.timer = self.node.create_timer(0.05, self.control_loop) # 20 Hz
        
        self.node.get_logger().info(f"Initialized Robot Controller: Enc={theta_id}, Motors={left_id}/{right_id}")

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

    def set_command(self, target_deg, speed_rpm):
        """Public method to set target from parent"""
        if self.current_heading is None:
            self.node.get_logger().warn(f"Robot {self.theta_id}: Cannot Execute - No heading data.")
            return

        # Calculate Target
        self.target_heading = self.normalize_angle(target_deg)
        self.target_vel_rpm = speed_rpm
        
        # Reset Integrators
        self.h_integral = 0.0
        self.h_prev_error = 0.0
        self.v_integral = 0.0
        self.v_prev_error = 0.0
        
        self.command_active = True
        self.node.get_logger().info(f"Robot {self.theta_id}: Command Accepted -> Head {self.target_heading:.1f}, Speed {speed_rpm}")

    def control_loop(self):
        now = time.time()
        dt = now - self.last_loop_time
        self.last_loop_time = now
        
        # 1. Safety Check: Encoder Timeout
        if self.last_heading_time == 0.0 or (now - self.last_heading_time > self.timeout):
            if self.command_active:
                self.node.get_logger().warn(f"Robot {self.theta_id}: Encoder timeout! Stopping motors.", throttle_duration_sec=2.0)
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
        error_v = self.target_vel_rpm 
        
        self.v_integral += error_v * dt
        derivative_v = (error_v - self.v_prev_error) / dt if dt > 0 else 0.0
        self.v_prev_error = error_v
        
        V_out = (self.kp_v * error_v) + (self.ki_v * self.v_integral) + (self.kd_v * derivative_v)

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


class SwarmController(Node):
    def __init__(self):
        super().__init__('swarm_controller')

        # --- Shared Parameters ---
        self.declare_parameter('wheel_base', 0.2)
        
        # PID Parameters
        self.declare_parameter('kp_heading', 2.75)
        self.declare_parameter('ki_heading', 1.0)
        self.declare_parameter('kd_heading', 0.0)
        self.declare_parameter('heading_integral_limit', 10.0)
        
        self.declare_parameter('kp_vel', 2.0)
        self.declare_parameter('ki_vel', 0.5)
        self.declare_parameter('kd_vel', 0.0)
        
        # Limits
        self.declare_parameter('max_rpm', 100.0)
        self.declare_parameter('timeout_sec', 0.5)

        # --- Initialize Robots ---
        self.robots = []
        self.robots.append(DifferentialDrivePID(self, "one", "one", "two"))
        self.robots.append(DifferentialDrivePID(self, "two", "three", "four"))
        self.robots.append(DifferentialDrivePID(self, "three", "five", "six"))
        self.robots.append(DifferentialDrivePID(self, "four", "seven", "eight"))
        
    #     # --- CLI Thread ---
    #     self.cli_thread = threading.Thread(target=self.cli_loop, daemon=True)
    #     self.cli_thread.start()
        
    #     self.get_logger().info("Swarm Controller Started with 4 Robots.")
    #     self.get_logger().info("Enter commands formatted as: <target_angle_deg> <speed_rpm>")
    #     self.get_logger().info("Example: '30 10' -> All robots head to 30 deg abs, move at 10 RPM")

    # def cli_loop(self):
    #     """Thread to handle non-blocking CLI input."""
    #     while rclpy.ok():
    #         try:
    #             if sys.stdin in select.select([sys.stdin], [], [], 1)[0]:
    #                 line = sys.stdin.readline()
    #                 if line:
    #                     self.process_command_broadcast(line.strip())
    #         except Exception as e:
    #             self.get_logger().error(f"CLI Error: {e}")

    # def process_command_broadcast(self, cmd_str):
    #     try:
    #         parts = cmd_str.split()
    #         if len(parts) == 2:
    #             target_theta = float(parts[0])
    #             speed_rpm = float(parts[1])
                
    #             self.get_logger().info(f"Broadcasting Command: Target={target_theta}, Speed={speed_rpm}")
                
    #             # Broadcast to all robots
    #             for bot in self.robots:
    #                 bot.set_command(target_theta, speed_rpm)
                    
    #         else:
    #             self.get_logger().warn("Invalid Format. Use: <target_deg> <speed_rpm>")
    #     except ValueError:
    #         self.get_logger().warn("Invalid Numbers. Use numeric values.")

    def stop_all(self):
        for bot in self.robots:
            bot.stop_motors()


def main(args=None):
    rclpy.init(args=args)
    node = SwarmController()
    
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
