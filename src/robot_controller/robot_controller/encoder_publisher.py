import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import serial
import time
import threading
import math

class EncoderPublisher(Node):
    def __init__(self):
        super().__init__('encoder_publisher')
        
        # Parameters
        self.declare_parameter('port', '/dev/ttyUSB1')
        self.declare_parameter('baud', 115200)
        
        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baud').value
        
        # Publishers
        self.pub_enc1 = self.create_publisher(Float64, '/encoder_one', 10)
        self.pub_enc2 = self.create_publisher(Float64, '/encoder_two', 10)
        self.pub_enc3 = self.create_publisher(Float64, '/encoder_three', 10)
        self.pub_enc4 = self.create_publisher(Float64, '/encoder_four', 10)
        
        self.serial_conn = None
        self.running = True
        self.thread = threading.Thread(target=self.read_serial_loop)
        self.thread.start()

    def read_serial_loop(self):
        while self.running and rclpy.ok():
            try:
                if self.serial_conn is None or not self.serial_conn.is_open:
                    try:
                        self.get_logger().info(f'Connecting to {self.port} at {self.baud}...')
                        self.serial_conn = serial.Serial(self.port, self.baud, timeout=1)
                        time.sleep(2) # Wait for reset
                        self.get_logger().info('Connected!')
                    except serial.SerialException as e:
                        self.get_logger().error(f'Failed to connect: {e}')
                        time.sleep(5)
                        continue

                if self.serial_conn.in_waiting > 0:
                    try:
                        line = self.serial_conn.readline().decode('utf-8').strip()
                        parts = line.split(',')
                        if len(parts) == 4:
                            e1, e2, e3, e4 = map(float, parts)
                            
                            # Normalize to [-pi, pi]
                            def normalize(angle):
                                while angle > math.pi:
                                    angle -= 2 * math.pi
                                while angle <= -math.pi:
                                    angle += 2 * math.pi
                                return angle

                            e1 = normalize(e1)
                            e2 = normalize(e2)
                            e3 = normalize(e3)
                            e4 = normalize(e4)
                            
                            msg = Float64()
                            
                            msg.data = e1
                            self.pub_enc1.publish(msg)
                            
                            msg.data = e2
                            self.pub_enc2.publish(msg)
                            
                            msg.data = e3
                            self.pub_enc3.publish(msg)
                            
                            msg.data = e4
                            self.pub_enc4.publish(msg)
                            
                            # logging for debug (optional, maybe verbose)
                            self.get_logger().debug(f'Published: {e1}, {e2}, {e3}, {e4}')
                        else:
                            self.get_logger().warn(f'Invalid data format: {line}')
                    except ValueError:
                        self.get_logger().warn(f'Error parsing: {line}')
                    except Exception as e:
                        self.get_logger().error(f'Error reading: {e}')
                        
            except Exception as e:
                 self.get_logger().error(f'Unexpected error in loop: {e}')
                 time.sleep(1)

    def destroy_node(self):
        self.running = False
        if self.serial_conn:
            self.serial_conn.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = EncoderPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
