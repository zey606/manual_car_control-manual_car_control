from collections import deque

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import String
import message_filters

ACC_THRESHOLD = 1.5
GYRO_THRESHOLD = 0.3
EMA_ALPHA = 0.05

VAR_BUFFER_SIZE = 20        # gercek hareketsizlik icin son N ornek
VAR_THRESHOLD = 0.0008      # rad^2/s^2 - Gemstone gyro varyans esigi, TEST EDEREK AYARLA

class ImuValidator(Node):
    def __init__(self):
        super().__init__('imu_validator')
        self.alert_pub = self.create_publisher(String, '/imu/alert', 10)

        self.offset = {k: 0.0 for k in ['ax', 'ay', 'az', 'gx', 'gy', 'gz']}
        self.offset_initialized = False
        self.last_cmd = None

        self.gyro_hist = {'x': deque(maxlen=VAR_BUFFER_SIZE),
                           'y': deque(maxlen=VAR_BUFFER_SIZE),
                           'z': deque(maxlen=VAR_BUFFER_SIZE)}

        self.create_subscription(String, '/robot/motor_cmd', self.cmd_cb, 10)

        sub_a = message_filters.Subscriber(self, Imu, '/imu/data_corrected')
        sub_b = message_filters.Subscriber(self, Imu, '/imu/deneyap/data')
        ts = message_filters.ApproximateTimeSynchronizer(
            [sub_a, sub_b], queue_size=10, slop=0.1)
        ts.registerCallback(self.compare)

        self.get_logger().info(
            'IMU validator basladi. Offset sadece komut="S" VE gercekten '
            'hareketsizken guncellenecek.')

    def cmd_cb(self, msg: String):
        if len(msg.data) > 0:
            self.last_cmd = msg.data[0]

    @staticmethod
    def _variance(buf):
        n = len(buf)
        if n < VAR_BUFFER_SIZE:
            return float('inf')  # yeterli veri yoksa hareketsiz sayma
        avg = sum(buf) / n
        return sum((v - avg) ** 2 for v in buf) / n

    def compare(self, imu_a: Imu, imu_b: Imu):
        self.gyro_hist['x'].append(imu_a.angular_velocity.x)
        self.gyro_hist['y'].append(imu_a.angular_velocity.y)
        self.gyro_hist['z'].append(imu_a.angular_velocity.z)

        vx = self._variance(self.gyro_hist['x'])
        vy = self._variance(self.gyro_hist['y'])
        vz = self._variance(self.gyro_hist['z'])
        really_stationary = (vx < VAR_THRESHOLD and vy < VAR_THRESHOLD and vz < VAR_THRESHOLD)

        stationary = (self.last_cmd == 'S') and really_stationary

        if stationary:
            raw_diff = {
                'ax': imu_b.linear_acceleration.x - imu_a.linear_acceleration.x,
                'ay': imu_b.linear_acceleration.y - imu_a.linear_acceleration.y,
                'az': imu_b.linear_acceleration.z - imu_a.linear_acceleration.z,
                'gx': imu_b.angular_velocity.x - imu_a.angular_velocity.x,
                'gy': imu_b.angular_velocity.y - imu_a.angular_velocity.y,
                'gz': imu_b.angular_velocity.z - imu_a.angular_velocity.z,
            }
            if not self.offset_initialized:
                self.offset = raw_diff
                self.offset_initialized = True
                self.get_logger().info(
                    f'Ilk offset atandi: ax={self.offset["ax"]:.3f} ay={self.offset["ay"]:.3f} '
                    f'az={self.offset["az"]:.3f} gx={self.offset["gx"]:.3f} '
                    f'gy={self.offset["gy"]:.3f} gz={self.offset["gz"]:.3f}')
            else:
                for k in self.offset:
                    self.offset[k] = (1 - EMA_ALPHA) * self.offset[k] + EMA_ALPHA * raw_diff[k]

        if not self.offset_initialized:
            return

        bx = imu_b.linear_acceleration.x - self.offset['ax']
        by = imu_b.linear_acceleration.y - self.offset['ay']
        bz = imu_b.linear_acceleration.z - self.offset['az']
        bgx = imu_b.angular_velocity.x - self.offset['gx']
        bgy = imu_b.angular_velocity.y - self.offset['gy']
        bgz = imu_b.angular_velocity.z - self.offset['gz']

        da = (abs(imu_a.linear_acceleration.x - bx)
            + abs(imu_a.linear_acceleration.y - by)
            + abs(imu_a.linear_acceleration.z - bz))
        dg = (abs(imu_a.angular_velocity.x - bgx)
            + abs(imu_a.angular_velocity.y - bgy)
            + abs(imu_a.angular_velocity.z - bgz))

        if da > ACC_THRESHOLD or dg > GYRO_THRESHOLD:
            msg = String()
            state = 'DURUYOR' if stationary else 'HAREKET/ELDE'
            msg.data = (f'IMU UYUSMAZLIGI! ({state}) '
                        f'ivme_fark={da:.2f} m/s^2 gyro_fark={dg:.2f} rad/s')
            self.alert_pub.publish(msg)
            self.get_logger().warn(msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = ImuValidator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
