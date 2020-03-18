#!/usr/bin/env python
#encoding: utf8
#adt_odom.py
#2018/5/21 Saoto Tsuchiya
#based on motors.py
#Copyright (c) 2016 Ryuichi Ueda <ryuichiueda@gmail.com>
#This software is released under the MIT License.
#http://opensource.org/licenses/mit-license.php

import sys, rospy, math, tf
import time
#from pimouse_ros.msg import MotorFreqs
from geometry_msgs.msg import Twist, Quaternion, TransformStamped, Point
#from std_srvs.srv import Trigger, TriggerResponse
#from pimouse_ros.srv import TimedMotion
from nav_msgs.msg import Odometry

class CalcOdom():
    def __init__(self):
        if not self.set_power(False): sys.exit(1)

        rospy.on_shutdown(self.set_power)
#        self.sub_raw = rospy.Subscriber('motor_raw', MotorFreqs, self.callback_raw_freq)
        self.sub_act_vel = rospy.Subscriber('act_vel', Twist, self.callback_act_vel)
#        self.srv_on = rospy.Service('motor_on', Trigger, self.callback_on)
#        self.srv_off = rospy.Service('motor_off', Trigger, self.callback_off)
#        self.srv_tm = rospy.Service('timed_motion', TimedMotion, self.callback_tm)

        self.pub_odom = rospy.Publisher('odom', Odometry, queue_size=10)
        self.bc_odom = tf.TransformBroadcaster()

        self.x, self.y, self.th = 0.0, 0.0, 0.0
        self.vx, self.vth = 0.0, 0.0

        self.cur_time = rospy.Time.now()
        self.last_time = self.cur_time

    def send_odom(self):
        self.cur_time = rospy.Time.now()

        dt = self.cur_time.to_sec() - self.last_time.to_sec()
        self.x += self.vx * math.cos(self.th) * dt
        self.y += self.vx * math.sin(self.th) * dt
        self.th += self.vth * dt * 0.6

        q = tf.transformations.quaternion_from_euler(0, 0, self.th)
        self.bc_odom.sendTransform((self.x,self.y,0.0), q, self.cur_time,"base_footprint0","odom")

        odom = Odometry()
        odom.header.stamp = self.cur_time
        odom.header.frame_id = "odom"

        odom.pose.pose.position = Point(self.x,self.y,0)
        odom.pose.pose.orientation = Quaternion(*q)

        odom.child_frame_id = "base_footprint"
        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = self.vth

        self.pub_odom.publish(odom)

        self.last_time = self.cur_time

    def set_power(self,onoff=False):
#        en = "/dev/rtmotoren0"
#        try:
#            with open(en,'w') as f:
#                f.write("1\n" if onoff else "0\n")
            self.is_on = onoff
            return True
#        except:
#            rospy.logerr("cannot write to " + en)
#
#        return False

    def callback_act_vel(self,message):
#        if not self.is_on:
#            return
        self.vx = message.linear.x
        self.vth = message.angular.z

    def callback_tm(self,message):
        if not self.is_on:
            rospy.logerr("not enpowered")
            return False

#        dev = "/dev/rtmotor0"
#        try:
#            with open(dev,'w') as f:
#                f.write("%d %d %d\n" %
#                    (message.left_hz,message.right_hz,message.duration_ms))
#        except:
#            rospy.logerr("cannot write to " + dev)
#            return False

        return True

if __name__ == '__main__':
    rospy.init_node('adt_odom')
    odom = CalcOdom()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        odom.send_odom()
        rate.sleep()

