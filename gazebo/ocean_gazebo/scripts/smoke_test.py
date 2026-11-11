#!/usr/bin/env python
# Checks the headless sim end to end: the same topics as ros_io.m
# (/clock, /odometry/base, /heron/thrusters/{0,1}/input). Drives straight,
# then turns right (thruster 0 = right, 1 = left), and prints the odometry.
import math
import rospy
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from uuv_gazebo_ros_plugins_msgs.msg import FloatStamped

state = {}


def on_odom(m):
    p = m.pose.pose.position
    q = m.pose.pose.orientation
    yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
    t = m.twist.twist
    state['x'] = (p.x, p.y, yaw, t.linear.x, t.linear.y, t.angular.z)


rospy.init_node('smoke_test')
rospy.wait_for_message('/clock', Clock, timeout=60)
rospy.Subscriber('/odometry/base', Odometry, on_odom)
pubs = [rospy.Publisher('/heron/thrusters/%d/input' % i, FloatStamped, queue_size=1)
        for i in (0, 1)]
rospy.wait_for_message('/odometry/base', Odometry, timeout=60)

phases = [(5.0, 0.6, 0.6, 'straight'), (5.0, -0.3, 0.6, 'turn right'), (3.0, 0.0, 0.0, 'stop')]
rate = rospy.Rate(10)
t0 = rospy.get_time()
print('%6s %-10s %8s %8s %8s %7s %7s %7s' % ('t', 'phase', 'x', 'y', 'psi', 'u', 'v', 'r'))
for dur, th0, th1, name in phases:
    tEnd = rospy.get_time() + dur
    while rospy.get_time() < tEnd and not rospy.is_shutdown():
        for pub, val in zip(pubs, (th0, th1)):
            msg = FloatStamped()
            msg.header.stamp = rospy.Time.now()
            msg.data = val
            pub.publish(msg)
        x = state['x']
        print('%6.1f %-10s %8.3f %8.3f %8.3f %7.3f %7.3f %7.3f' % ((rospy.get_time() - t0, name) + x))
        rate.sleep()
print('sim time %.1f s' % (rospy.get_time() - t0))
