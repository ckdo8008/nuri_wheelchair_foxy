#!/usr/bin/env python

from tokenize import Double
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.logging import get_logger

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from sensor_msgs.msg import Joy
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import Range
from std_msgs.msg import ByteMultiArray, MultiArrayLayout
from rclpy.qos import QoSProfile

import math

MAX_LIN_VEL = 1.50
MAX_ANG_VEL = 2.00

LIN_VEL_STEP_SIZE = 0.01
ANG_VEL_STEP_SIZE = 0.1

class TeleopJoyNode(Node):

    def __init__(self):
        super().__init__('teleop_joy_node')
        self.declare_parameters(
            namespace='',
            parameters=[
                ('max_fwd_m_s', 1.5),
                ('max_rev_m_s', 0.8),
                ('max_deg_s', 1.2),
            ])
        self.timer_inc = 0
        self.auto_mode = False
        self.headlight_on = False
        self.mode_button_last = 0
        self.colorIdx = 0
        self.colors = [ [255, 0, 0], [255,50, 0], [255,255,0], [0,255,0], 
            [0,0,255], [0,5,255], [100,0,255], [255,255,255] ]
        print(' R1mini Teleop Joystick controller')
        # Get parameter values
        self.max_fwd_vel = self.get_parameter_or('max_fwd_m_s', Parameter('max_fwd_m_s', Parameter.Type.DOUBLE, 1.5)).get_parameter_value().double_value
        self.max_rev_vel = self.get_parameter_or('max_rev_m_s', Parameter('max_rev_m_s', Parameter.Type.DOUBLE, 0.8)).get_parameter_value().double_value
        self.max_ang_vel = self.get_parameter_or('max_deg_s', Parameter('max_deg_s', Parameter.Type.DOUBLE, 2.0)).get_parameter_value().double_value
        print('Param max fwd: %s m/s, max rev: -%s m/s, max ang: %s dev/s'%
            (self.max_fwd_vel,
            self.max_rev_vel,
            self.max_ang_vel)
        )
        self.qos = QoSProfile(depth=10)
        self.pub_twist = self.create_publisher(Twist, 'cmd_vel', self.qos)
        self.pub_raw = self.create_publisher(ByteMultiArray, 'mc_rawdata', self.qos)
        
        self.sub = self.create_subscription(Joy, 'hc/joy', self.cb_joy, 10)
        self.subscan = self.create_subscription(LaserScan, 'scan', self.cb_scan, 10)
        self.subfl = self.create_subscription(Range, 'front_left', self.cbFrontL, 10)
        self.subfr = self.create_subscription(Range, 'front_right', self.cbFrontR, 10)
        self.subleft = self.create_subscription(Range, 'left', self.cbLeft, 10)
        self.subright = self.create_subscription(Range, 'right', self.cbRight, 10)
        self.subback = self.create_subscription(Range, 'back', self.cbBack, 10)
        self.subbottom = self.create_subscription(Range, 'bottom', self.cbBottom, 10)
        
        self.timer = self.create_timer(0.05, self.cb_timer)
        self.twist = Twist()
        self.points = []
        self.chk_forward = False
        self.chk_left_side = False
        self.chk_right_side = False
        self.cos_theta = math.cos(math.pi / 2)  # cos(90°)
        self.sin_theta = math.sin(math.pi / 2)  # sin(90°)
        self.width = 1080
        self.height = 2102
        self.centerX = self.width / 2 - 100
        self.centerY = self.height / 2 - 194
        self.fow = 0
        self.lastfow = 0
        self.rot = 0
        self.forw_prof = 0
        
        self.chk_ul_fl = False
        self.chk_ul_fr = False
        self.chk_ul_l = False
        self.chk_ul_r = False
        self.chk_ul_bt = False
        self.chk_ul_bk = False
        
        self.first = True
        
    def cbFrontL(self, msg):
        if (msg.range < 30):
            self.chk_ul_fl = True
        else:
            self.chk_ul_fl = False
        
    def cbFrontR(self, msg):
        if (msg.range < 30):
            self.chk_ul_fr = True
        else:
            self.chk_ul_fr = False
                
    def cbLeft(self, msg):
        if (msg.range < 30):
            self.chk_ul_l = True
        else:
            self.chk_ul_l = False
        
    def cbRight(self, msg):
        if (msg.range < 30):
            self.chk_ul_r = True
        else:
            self.chk_ul_r = False
        
    def cbBack(self, msg):
        # print(['back', msg.range])   
        if (msg.range < 30):
            self.chk_ul_bk = True
        else:
            self.chk_ul_bk = False        
        
    def cbBottom(self, msg):
        # print(['bottom', msg.range])   
        if (msg.range > 30):
            self.chk_ul_bt = True
        else:
            self.chk_ul_bt = False          

    def sendControlOn(self):
        data0 = [b'\xFF', b'\xFE', b'\x00', b'\x03', b'\xF0', b'\x0C', b'\x00']
        data1 = [b'\xFF', b'\xFE', b'\x01', b'\x03', b'\xEF', b'\x0C', b'\x00']
        msg0 = ByteMultiArray()
        msg0.data = data0
        msg1 = ByteMultiArray()
        msg1.data = data1
        self.pub_raw.publish(msg0)
        self.pub_raw.publish(msg0)
        self.pub_raw.publish(msg1)
        self.pub_raw.publish(msg1)
    
    def laser_scan_to_points(self, laser_scan: LaserScan):
        points = []
        current_angle = laser_scan.angle_min

        for range_value in laser_scan.ranges:
            if range_value is None:
                continue

            x = range_value * math.cos(current_angle)
            y = range_value * math.sin(current_angle)
            points.append(Point(x=x, y=y, z=0.0))  # ROS2에서 Point 메시지 사용
            current_angle += laser_scan.angle_increment

        self.points = points
    
    def check_stop(self):
        chk_forward = False
        chk_left_side = False
        chk_right_side = False
        for point in self.points:
            x = point.x * 350  # 미터를 픽셀로 변환
            y = point.y * 350  # 미터를 픽셀로 변환

            rotated_x = x * self.cos_theta - y * self.sin_theta
            rotated_y = x * self.sin_theta + y * self.cos_theta

            delta_x = rotated_x - 100
            delta_y = rotated_y + 90

            dx = (self.centerX + rotated_x)
            dy = (self.centerY - rotated_y)

            if dx > 0.0 and dy > 0.0:
                if dx <= self.width and dy <= self.height:
                    distance = math.sqrt(delta_x ** 2 + delta_y ** 2)
                    ratio = min(max(distance - 200, 0) / 200, 1)

                    if self.centerX - 20 <= dx <= self.centerX + 220:
                        if self.centerY >= dy >= self.centerY - 200:
                            chk_forward = True
                    else:
                        if ratio < 0.15:
                            if dx < self.centerX + 20:
                                chk_left_side = True
                            elif dx > self.centerX + 180:
                                chk_right_side = True
                                
        self.chk_forward = chk_forward
        self.chk_left_side = chk_left_side
        self.chk_right_side = chk_right_side
    
    def cb_scan(self, msg):
        self.laser_scan_to_points(msg)
        self.check_stop()
        
    def gen_profile(self, v_ref, vout, dt=0.01, amax=0.8):
        da = 0
        dv = 0

        # 속도 프로파일 생성
        if v_ref == vout:
            dv = 0
        else:
            da = (v_ref - vout) / dt
            if abs(da) >= amax:
                da = amax if da > 0 else -amax

        dv = da * dt
        vout += dv
        return vout


    def cb_joy(self, joymsg):
        if self.first:
            self.first = False
            self.sendControlOn()

        self.fow = joymsg.axes[1]
        self.rot = joymsg.axes[0]
        if self.fow >= 0 and self.fow <= 0.05:
            self.fow = 0
        if self.fow < 0 and self.fow >= -0.05:
            self.fow = 0
        
        if self.rot >= 0 and self.rot <= 0.05:
            self.rot = 0
        if self.rot < 0 and self.rot >= -0.05:
            self.rot = 0
    
    def cb_timer(self):
        self.timer_inc+=1
        if self.auto_mode == False:
            if (self.lastfow > 0 and self.fow < 0) or (self.lastfow < 0 and self.fow > 0):
                self.fow = 0
            
            if (self.fow > 0 and (self.chk_ul_fl or self.chk_ul_fr)):
                self.fow = 0
            
            if (self.fow < 0 and (self.chk_ul_bt or self.chk_ul_bk)):
                self.fow = 0

            if (self.rot > 0 and self.chk_ul_l):
                self.rot = 0
                
            if (self.rot < 0 and self.chk_ul_r):
                self.rot = 0
            
            self.lastfow = self.fow
            if self.fow != 0:
                self.forw_prof = self.gen_profile(self.fow, self.forw_prof)
            else:
                self.forw_prof = 0
            
            if self.chk_forward and self.fow > 0:
                self.forw_prof = 0
            
            if (self.chk_left_side and self.rot > 0) or (self.chk_right_side and self.rot < 0):
                self.rot = 0
            
            if self.forw_prof >= 0:
                self.twist.linear.x = self.forw_prof * self.max_fwd_vel
            else:
                self.twist.linear.x = self.forw_prof * self.max_rev_vel
                
            self.twist.angular.z = self.rot * self.max_ang_vel
            self.twist.linear.y = 0.0
            self.twist.linear.z = 0.0
            self.twist.angular.x = 0.0
            self.twist.angular.y = 0.0
            self.pub_twist.publish(self.twist)

def main(args=None):
    rclpy.init(args=args)
    print (args)
    teleop_joy =  TeleopJoyNode()
    rclpy.spin(teleop_joy)
    teleop_joy.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
