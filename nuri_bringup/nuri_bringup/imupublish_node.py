import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Temperature, Imu
import numpy
import math
import smbus

_EPS = numpy.finfo(float).eps * 4.0
def vector_norm(data, axis=None, out=None):
    data = numpy.array(data, dtype=numpy.float64, copy=True)
    if out is None:
        if data.ndim == 1:
            return math.sqrt(numpy.dot(data, data))
        data *= data
        out = numpy.atleast_1d(numpy.sum(data, axis=axis))
        numpy.sqrt(out, out)
        return out
    else:
        data *= data
        numpy.sum(data, axis=axis, out=out)
        numpy.sqrt(out, out)

def quaternion_about_axis(angle, axis):
    quaternion = numpy.zeros((4, ), dtype=numpy.float64)
    quaternion[:3] = axis[:3]
    qlen = vector_norm(quaternion)
    if qlen > _EPS:
        quaternion *= math.sin(angle/2.0) / qlen
    quaternion[3] = math.cos(angle/2.0)
    return quaternion


class IMUPublisher(Node):
    bus = smbus.SMBus(0)
    ADDR = 0x68
    IMU_FRAME = 'imu_link'
    TEMP_H = 0x41
    TEMP_L = 0x42
    PWR_MGMT_1 = 0x6b
    ACCEL_CONFIG = 0x1C
    ACCEL_XOUT_H = 0x3B
    ACCEL_XOUT_L = 0x3C
    ACCEL_YOUT_H = 0x3D
    ACCEL_YOUT_L = 0x3E
    ACCEL_ZOUT_H = 0x3F
    ACCEL_ZOUT_L = 0x40

    GYRO_CONFIG = 0x1B
    GYRO_XOUT_H = 0x43
    GYRO_XOUT_L = 0x44
    GYRO_YOUT_H = 0x45
    GYRO_YOUT_L = 0x46
    GYRO_ZOUT_H = 0x47
    GYRO_ZOUT_L = 0x48    


    def __init__(self):
        super().__init__('imu_publisher')
        self.bus.write_byte_data(self.ADDR, self.PWR_MGMT_1, 0)
        self.publisher_ = self.create_publisher(Imu, 'imu/data', 10)
        self.publisher_Temp = self.create_publisher(Temperature, 'temperature', 10)
        timer_period = 0.2  # 초 단위

        self.imu_timer = self.create_timer(timer_period, self.publish_imu)
        self.temp_timer = self.create_timer(10, self.publish_temp)

        self.imu_msg = Imu()
        self.imu_msg.header.frame_id = "imu_link"  # Or whatever your frame ID is
        self.get_logger().info('Start imu_publisher...')

    def read_word(self, adr):
        high = self.bus.read_byte_data(self.ADDR, adr)
        low = self.bus.read_byte_data(self.ADDR, adr+1)
        val = (high << 8) + low
        return val

    def read_word_2c(self, adr):
        val = self.read_word(adr)
        if (val >= 0x8000):
            return -((65535 - val) + 1)
        else:
            return val
        
    def publish_temp(self):
        temp_msg = Temperature()
        temp_msg.header.frame_id = self.IMU_FRAME
        temp_msg.temperature = self.read_word_2c(self.TEMP_H)/340.0 + 36.53
        temp_msg.header.stamp = self.get_clock().now().to_msg()
        self.publisher_Temp.publish(temp_msg)
        # self.get_logger().info('Publishing Temperature data')

    def publish_imu(self):

        # Read the acceleration vals
        accel_x = self.read_word_2c(self.ACCEL_XOUT_H) / 16384.0
        accel_y = self.read_word_2c(self.ACCEL_YOUT_H) / 16384.0
        accel_z = self.read_word_2c(self.ACCEL_ZOUT_H) / 16384.0
        
        # Calculate a quaternion representing the orientation
        accel = accel_x, accel_y, accel_z
        ref = numpy.array([0, 0, 1])
        acceln = accel / numpy.linalg.norm(accel)
        axis = numpy.cross(acceln, ref)
        angle = numpy.arccos(numpy.dot(acceln, ref))
        orientation = quaternion_about_axis(angle, axis)

        # Read the gyro vals
        gyro_x = self.read_word_2c(self.GYRO_XOUT_H) / 131.0
        gyro_y = self.read_word_2c(self.GYRO_YOUT_H) / 131.0
        gyro_z = self.read_word_2c(self.GYRO_ZOUT_H) / 131.0

        # Load up the IMU message
        o = self.imu_msg.orientation
        o.x, o.y, o.z, o.w = orientation

        self.imu_msg.linear_acceleration.x = accel_x
        self.imu_msg.linear_acceleration.y = accel_y
        self.imu_msg.linear_acceleration.z = accel_z

        self.imu_msg.angular_velocity.x = gyro_x
        self.imu_msg.angular_velocity.y = gyro_y
        self.imu_msg.angular_velocity.z = gyro_z

        self.imu_msg.header.stamp = self.get_clock().now().to_msg()

        self.publisher_.publish(self.imu_msg)
        # self.get_logger().info('Publishing IMU data')
            
def main(args=None):
    rclpy.init(args=args)
    imu_publisher = IMUPublisher()
    rclpy.spin(imu_publisher)
    imu_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
