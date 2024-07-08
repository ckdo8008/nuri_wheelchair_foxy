import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger

import numpy as np
import math

from nurirobot_msgs.msg import NurirobotPos
from rclpy.qos import qos_profile_sensor_data
from nuri_interface.srv import ResetOdom
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion, Pose
from tf2_ros import TransformBroadcaster


def euler_from_quaternion(quaternion):
    x = quaternion[0]
    y = quaternion[1]
    z = quaternion[2]
    w = quaternion[3]

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def quaternion_from_euler(roll, pitch, yaw):
    """
    Converts euler roll, pitch, yaw to quaternion (w in last place)
    quat = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    q = [0] * 4
    q[0] = cy * cp * sr - sy * sp * cr
    q[1] = sy * cp * sr + cy * sp * cr
    q[2] = sy * cp * cr - cy * sp * sr
    q[3] = cy * cp * cr + sy * sp * sr

    return q


class OdomPose(object):
    x = 0.0
    y = 0.0
    theta = 0.0
    timestamp = 0
    pre_timestamp = 0


class OdomVel(object):
    x = 0.0
    y = 0.0
    w = 0.0


class ComplementaryFilter:
    def __init__(self):
        self.theta = 0.0
        self.pre_theta = 0.0
        self.wheel_ang = 0.0
        self.filter_coef = 2.5
        self.gyro_bias = 0.0
        self.count_for_gyro_bias = 110

    def gyro_calibration(self, gyro):
        self.count_for_gyro_bias -= 1

        if self.count_for_gyro_bias > 100:
            return "Prepare for gyro_calibration"

        self.gyro_bias += gyro
        if self.count_for_gyro_bias == 1:
            self.gyro_bias /= 100
            print("Complete : Gyro calibration")
            return "gyro_calibration OK"

        return "During gyro_calibration"

    def calc_filter(self, gyro, d_time):
        if self.count_for_gyro_bias != 1:
            tmp = self.gyro_calibration(gyro)
            return 0

        gyro -= self.gyro_bias
        self.pre_theta = self.theta
        temp = -1 / self.filter_coef * (-self.wheel_ang + self.pre_theta) + gyro
        self.theta = self.pre_theta + temp * d_time
        return self.theta


def calculate_angle_difference(prev_angle, current_angle, divfeedback):
    diff = current_angle - prev_angle
    if diff > 32766:  # 반대 방향으로 큰 변화가 감지된 경우 (0에서 65533로의 점프)
        diff -= 65533
    elif diff < -32766:  # 반대 방향으로 큰 변화가 감지된 경우 (65533에서 0으로의 점프)
        diff += 65533
    return diff / divfeedback


# class KalmanFilter:
#     def __init__(self):
#         # 1차원 상태를 가정
#         self.state_estimate = np.array([[0]])  # 초기 상태 추정치
#         self.error_covariance = np.array([[1]])  # 초기 오차 공분산
#         self.process_model = np.array([[1]])  # 프로세스 모델
#         self.measurement_model = np.array([[1]])  # 측정 모델
#         self.process_noise_covariance = np.array([[0.1]])  # 프로세스 노이즈 공분산
#         self.measurement_noise_covariance = np.array([[0.1]])  # 측정 노이즈 공분산

#     def predict(self):
#         # 상태 예측
#         self.state_estimate = self.process_model.dot(self.state_estimate)
#         # 오차 공분산 예측
#         self.error_covariance = (
#             self.process_model.dot(self.error_covariance).dot(self.process_model.T)
#             + self.process_noise_covariance
#         )

#     def update(self, measurement):
#         # 칼만 이득 계산
#         kalman_gain = self.error_covariance.dot(self.measurement_model.T).dot(
#             np.linalg.inv(
#                 self.measurement_model.dot(self.error_covariance).dot(
#                     self.measurement_model.T
#                 )
#                 + self.measurement_noise_covariance
#             )
#         )
#         # 상태 업데이트
#         self.state_estimate = self.state_estimate + kalman_gain.dot(
#             measurement - self.measurement_model.dot(self.state_estimate)
#         )
#         # 오차 공분산 업데이트
#         self.error_covariance = (
#             np.eye(1) - kalman_gain.dot(self.measurement_model)
#         ).dot(self.error_covariance)


class NurirobotOdomNode(Node):
    def __init__(self):
        super().__init__("nuri_odom_node")
        self.get_logger().info("nuri_odom_node : start")
        self.wheel_separation = 0.45
        self.wheel_radius = 0.29
        self.divfeedback = 10.0
        self.isOnOdom = True

        self.odom_pose = OdomPose()
        self.odom_pose.timestamp = self.get_clock().now()
        self.odom_pose.pre_timestamp = self.get_clock().now()
        self.odom_vel = OdomVel()

        self.max_lin_vel_x = 2.0
        self.max_ang_vel_z = 2.5
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.imu_z = 0.0
        self.v_left = 0.0
        self.v_right = 0.0
        self.pos_left = 0.0
        self.pos_right = 0.0
        self.lin_vel_x = 0.0
        self.ang_vel_z = 0.0
        self.count = 0

        self._pos = [0.0, 0.0]
        self._rpm = [0.0, 0.0]
        self._gyro = [0.0, 0.0, 0.0]
        self._imu = [0.0, 0.0, 0.0]
        self._prev_pos = [-1.0, -1.0]

        # self._pos_Kalman = [KalmanFilter(), KalmanFilter()]
        # self._pos_Kalman[0].predict()
        # self._pos_Kalman[1].predict()

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_theta = 0.0

        self.remote = False

        self.calc_yaw = ComplementaryFilter()
        self.last_time = self.get_clock().now().seconds_nanoseconds()[0]
        self.srvResetODOM = self.create_service(
            ResetOdom, "reset_odom", self.cbSrv_resetODOM
        )
        self.subPos = self.create_subscription(
            NurirobotPos,
            "nurirobot_driver_node/pos",
            self.cbPos,
            qos_profile_sensor_data,
        )

        self.subImu = self.create_subscription(
            Imu, "imu_link", self.cbImu, qos_profile_sensor_data
        )

        self.pub_Odom = self.create_publisher(Odometry, "odom", qos_profile_sensor_data)
        self.pub_OdomTF = TransformBroadcaster(self)
        self.pub_pose = self.create_publisher(Pose, "pose", qos_profile_sensor_data)
        # self.pub_imu = self.create_publisher(Imu, 'imu_link', qos_profile_sensor_data)
        self.timerProc = self.create_timer(0.01, self.update_robot)

    def cbSrv_resetODOM(self, request, response):
        self.odom_pose.x = request.x
        self.odom_pose.y = request.y
        self.odom_pose.theta = request.theta
        self.get_logger().info(
            "SERVICE: RESET ODOM X:%s, Y:%s, Theta:%s"
            % (request.x, request.y, request.theta)
        )
        return response

    def cbSrv_resetODOM(self, request, response):
        self.odom_pose.x = request.x
        self.odom_pose.y = request.y
        self.odom_pose.theta = request.theta
        self.get_logger().info(
            "SERVICE: RESET ODOM X:%s, Y:%s, Theta:%s"
            % (request.x, request.y, request.theta)
        )
        return response

    def cbImu(self, msg):
        self.imu_z = msg.angular_velocity.z
        orientation_q = msg.orientation
        roll, pitch, yaw = euler_from_quaternion(
            [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        )

        # self.get_logger().info("%f %f %f"%(roll, pitch, yaw))
        self.updatePoseStates(roll, pitch, yaw)

    def updatePoseStates(self, roll, pitch, yaw):
        pose = Pose()
        pose.orientation.x = roll
        pose.orientation.y = pitch
        pose.orientation.z = yaw
        self.pub_pose.publish(pose)

    def cbPos(self, msg):
        if self._prev_pos[msg.id] == -1.0:
            self._prev_pos[msg.id] = msg.pos

        # self._pos_Kalman[msg.id].update(msg.pos)
        # self._pos[msg.id] = self._pos_Kalman[msg.id].state_estimate

        self._pos[msg.id] = msg.pos
        # print(msg.pos)
        return

    def update_robot(self):
        if self._prev_pos[0] == -1.0 or self._prev_pos[1] == -1.0:
            return

        delta_angle_l = calculate_angle_difference(
            self._prev_pos[0], self._pos[0], self.divfeedback
        )
        delta_angle_r = (
            calculate_angle_difference(
                self._prev_pos[1], self._pos[1], self.divfeedback
            )
            * -1
        )

        self._prev_pos[0] = self._pos[0]
        self._prev_pos[1] = self._pos[1]

        # print(self._prev_pos[0], self._pos[0], self._prev_pos[1], self._pos[1] )

        self.compute_odometry(delta_angle_l / 18.9, delta_angle_r / 18.9)

    def compute_odometry(self, delta_left_angle, delta_right_angle):
        # print([delta_left_angle, delta_right_angle] )
        delta_left_angle = math.radians(delta_left_angle)
        delta_right_angle = math.radians(delta_right_angle)

        delta_left_distance = delta_left_angle * self.wheel_radius
        delta_right_distance = delta_right_angle * self.wheel_radius
        delta_distance = (delta_left_distance + delta_right_distance) / 2.0
        delta_theta = (
            delta_right_distance - delta_left_distance
        ) / self.wheel_separation

        self.odom_pose.timestamp = self.get_clock().now()
        dt = (
            self.odom_pose.timestamp - self.odom_pose.pre_timestamp
        ).nanoseconds * 1e-9
        self.odom_pose.pre_timestamp = self.odom_pose.timestamp

        self.calc_yaw.wheel_ang += delta_theta
        self.odom_pose.theta = self.calc_yaw.calc_filter(
            self.imu_z * math.pi / 180.0, dt
        )
        # self.odom_pose.theta += delta_theta

        # print("delta_distance %f, curr Theta: %f, theta: %f"% (delta_distance, delta_theta,self.odom_pose.theta))
        q = quaternion_from_euler(0, 0, self.odom_pose.theta)

        # self.odom_pose.theta += delta_theta
        self.odom_pose.x += delta_distance * math.cos(self.odom_pose.theta)
        self.odom_pose.y += delta_distance * math.sin(self.odom_pose.theta)

        timestamp_now = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp = timestamp_now
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"
        odom.pose.pose.position.x = self.odom_pose.x
        odom.pose.pose.position.y = self.odom_pose.y

        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.twist.twist.linear.x = (
            delta_distance  # Assume a unit time interval for simplicity
        )
        odom.twist.twist.angular.z = (
            delta_theta  # Assume a unit time interval for simplicity
        )
        self.pub_Odom.publish(odom)

        odom_tf = TransformStamped()
        odom_tf.header.frame_id = odom.header.frame_id
        odom_tf.child_frame_id = odom.child_frame_id
        odom_tf.header.stamp = timestamp_now
        odom_tf.transform.translation.x = odom.pose.pose.position.x
        odom_tf.transform.translation.y = odom.pose.pose.position.y
        odom_tf.transform.translation.z = 0.0
        odom_tf.transform.rotation = odom.pose.pose.orientation

        self.pub_OdomTF.sendTransform(odom_tf)

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(
            roll / 2
        ) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(
            roll / 2
        ) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(
            roll / 2
        ) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(
            roll / 2
        ) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)


def main(args=None):
    rclpy.init(args=args)
    node = NurirobotOdomNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
