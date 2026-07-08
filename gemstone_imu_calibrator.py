import math
from collections import deque

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import String

BUFFER_SIZE = 40                 # ~0.4s pencere (Gemstone IMU ~100Hz varsayimiyla)
VAR_THRESHOLD = 0.0005           # rad^2/s^2 - Deneyap'taki 0.05(dps^2) karsiligi, TEST EDEREK AYARLA
EMA_ALPHA = 0.05                 # Deneyap ile ayni agirlik (0.95 eski / 0.05 yeni)

class GemstoneImuCalibrator(Node):
    def __init__(self):
        super().__init__('gemstone_imu_calibrator')

        self.bias = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.hist = {'x': deque(maxlen=BUFFER_SIZE),
                     'y': deque(maxlen=BUFFER_SIZE),
                     'z': deque(maxlen=BUFFER_SIZE)}
        self.last_cmd = None  # ilk komut gelene kadar kalibrasyon yapma

        self.pub = self.create_publisher(Imu, '/imu/data_corrected', 10)
        self.create_subscription(Imu, '/imu/data_raw', self.imu_cb, 10)
        self.create_subscription(String, '/robot/motor_cmd', self.cmd_cb, 10)

        self.get_logger().info('Gemstone IMU calibrator basladi (Deneyap ile senkron).')

    def cmd_cb(self, msg: String):
        if len(msg.data) > 0:
            self.last_cmd = msg.data[0]

    @staticmethod
    def _variance(buf):
        n = len(buf)
        if n == 0:
            return 0.0
        avg = sum(buf) / n
        return sum((v - avg) ** 2 for v in buf) / n

    def imu_cb(self, msg: Imu):
        gx, gy, gz = (msg.angular_velocity.x,
                      msg.angular_velocity.y,
                      msg.angular_velocity.z)
        self.hist['x'].append(gx)
        self.hist['y'].append(gy)
        self.hist['z'].append(gz)

        if (len(self.hist['x']) == BUFFER_SIZE and self.last_cmd == 'S'):
            vx = self._variance(self.hist['x'])
            vy = self._variance(self.hist['y'])
            vz = self._variance(self.hist['z'])
            if vx < VAR_THRESHOLD and vy < VAR_THRESHOLD and vz < VAR_THRESHOLD:
                avgx = sum(self.hist['x']) / BUFFER_SIZE
                avgy = sum(self.hist['y']) / BUFFER_SIZE
                avgz = sum(self.hist['z']) / BUFFER_SIZE
                self.bias['x'] = (1 - EMA_ALPHA) * self.bias['x'] + EMA_ALPHA * avgx
                self.bias['y'] = (1 - EMA_ALPHA) * self.bias['y'] + EMA_ALPHA * avgy
                self.bias['z'] = (1 - EMA_ALPHA) * self.bias['z'] + EMA_ALPHA * avgz

        out = Imu()
        out.header = msg.header
        out.linear_acceleration = msg.linear_acceleration
        out.angular_velocity.x = gx - self.bias['x']
        out.angular_velocity.y = gy - self.bias['y']
        out.angular_velocity.z = gz - self.bias['z']
        out.orientation_covariance[0] = -1.0
        self.pub.publish(out)

def main(args=None):
    rclpy.init(args=args)
    node = GemstoneImuCalibrator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
