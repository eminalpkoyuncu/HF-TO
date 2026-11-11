#!/usr/bin/env python
# /pose_gt (p3d ground truth, twist in the world frame) -> /odometry/base
# with the twist in the body frame (u, v, r), as read by ros_io.m.
import rospy
from nav_msgs.msg import Odometry


def rotate_inv(q, v):
    # R(q)^T v for a unit quaternion q = (x, y, z, w)
    x, y, z, w = -q.x, -q.y, -q.z, q.w
    tx = 2 * (y * v[2] - z * v[1])
    ty = 2 * (z * v[0] - x * v[2])
    tz = 2 * (x * v[1] - y * v[0])
    return (v[0] + w * tx + (y * tz - z * ty),
            v[1] + w * ty + (z * tx - x * tz),
            v[2] + w * tz + (x * ty - y * tx))


def on_gt(m):
    q = m.pose.pose.orientation
    lin = m.twist.twist.linear
    ang = m.twist.twist.angular
    out = Odometry()
    out.header = m.header
    out.child_frame_id = 'base_link'
    out.pose = m.pose
    (out.twist.twist.linear.x, out.twist.twist.linear.y,
     out.twist.twist.linear.z) = rotate_inv(q, (lin.x, lin.y, lin.z))
    (out.twist.twist.angular.x, out.twist.twist.angular.y,
     out.twist.twist.angular.z) = rotate_inv(q, (ang.x, ang.y, ang.z))
    pub.publish(out)


rospy.init_node('base_odometry')
pub = rospy.Publisher('/odometry/base', Odometry, queue_size=1)
rospy.Subscriber('/pose_gt', Odometry, on_gt, queue_size=1, tcp_nodelay=True)
rospy.spin()
