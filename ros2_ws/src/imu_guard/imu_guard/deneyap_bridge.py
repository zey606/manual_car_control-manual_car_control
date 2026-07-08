import math
import serial
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import String

G = 9.80665

class DeneyapBridge(Node):
    def __init__(self):
        super().__init__('deneyap_bridge')
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        self.pub = self.create_publisher(Imu, '/imu/deneyap/data', 10)
        self.cmd_pub = self.create_publisher(String, '/robot/motor_cmd', 10)
        self.ser = serial.Serial(port, baud, timeout=1.0)
        self.create_timer(0.01, self.read_loop)
        self.get_logger().info(f'Deneyap IMU bridge basladi: {port}')

    def read_loop(self):
        try:
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
        except Exception as e:
            self.get_logger().warn(f'Seri okuma hatasi: {e}')
            return
        if not line:
            return
        line = line.lstrip('>')
        parts = line.split(',')
        if len(parts) < 8 or parts[0] != 'DATA':
            return
        cmd_char = parts[1]
        try:
            ax, ay, az, gx, gy, gz = [float(x) for x in parts[2:8]]
        except ValueError:
            return

        cmd_msg = String()
        cmd_msg.data = cmd_char
        self.cmd_pub.publish(cmd_msg)

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'deneyap_imu_link'
        msg.linear_acceleration.x = ax * G
        msg.linear_acceleration.y = ay * G
        msg.linear_acceleration.z = az * G
        # NOT: gx,gy,gz derece/s varsayildi -- gerekirse asagidaki cevrimi kaldir
        msg.angular_velocity.x = math.radians(gx)
        msg.angular_velocity.y = math.radians(gy)
        msg.angular_velocity.z = math.radians(gz)
        msg.orientation_covariance[0] = -1.0
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = DeneyapBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
