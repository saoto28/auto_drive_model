#!/usr/bin/env python
#encoding: utf8
#pub_trailer_joint.py
#2018/5/31 Saoto Tsuchiya
#based on adt_odom.py

import sys, rospy, math, tf
import time
from geometry_msgs.msg import Twist, Quaternion, TransformStamped, Point
#from nav_msgs.msg import Odometry

class CalcOdom():
    def __init__(self):
        if not self.set_power(False): sys.exit(1)

        rospy.on_shutdown(self.set_power)
        self.sub_act_vel = rospy.Subscriber('act_vel', Twist, self.callback_act_vel)

#        self.pub_odom = rospy.Publisher('odom', Odometry, queue_size=10)
        self.bc_odom = tf.TransformBroadcaster()

	self.th = 0.0

        self.cur_time = rospy.Time.now()
        self.last_time = self.cur_time

    def send_odom(self):
        self.cur_time = rospy.Time.now()

        q = tf.transformations.quaternion_from_euler(0, 0, self.th)
        self.bc_odom.sendTransform((-0.17,0.0, 0.035), q, self.cur_time,"trailer","base_link")

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
        self.th = message.angular.x

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
    rospy.init_node('pub_trailer_joint')
    odom = CalcOdom()

    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        odom.send_odom()
        rate.sleep()
