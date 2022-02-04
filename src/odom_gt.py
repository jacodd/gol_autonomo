#!/usr/bin/env python

from math import cos, sin, tan
import rospy
import tf
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from sensor_msgs.msg import JointState, NavSatFix
from nav_msgs.msg import Odometry

class odom_gt():
    def __init__(self):
        self.pub = rospy.Publisher('odom_gt', Odometry, queue_size=10)
        self.sub = rospy.Subscriber('GPS',NavSatFix,self.callback)
        self.odom_broadcaster = tf.TransformBroadcaster()
        self.x = 0
        self.y = 0
        self.z = 0
        
        

    def compute_odom(self):
       
        
        self.current_time = rospy.Time.now()
        # since all odometry is 6DOF we'll need a quaternion created from yaw
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, 0)

        # first, we'll publish the transform over tf
        self.odom_broadcaster.sendTransform(
            (self.x, self.y, 0.),
            odom_quat,
            self.current_time,
            "base_link_gt",
            "odom1"
        )
        self.odom_broadcaster.sendTransform(
                (31.36, 14.98, 0.),
                tf.transformations.quaternion_from_euler(0, 0, 3.14),
                rospy.Time.now(),
                "odom",
                "odom1"
            )

        # next, we'll publish the odometry message over ROS
        odom = Odometry()
        odom.header.stamp = self.current_time
        odom.header.frame_id = "odom1"

        # set the position
        odom.pose.pose = Pose(Point(self.x, self.y, 0.), Quaternion(*odom_quat))

        # set the velocity
        odom.child_frame_id = "base_link_gt"
        odom.twist.twist = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))

        # publish the message
        self.pub.publish(odom)

        

    def callback(self,msg):
        self.x = msg.latitude
        self.y = msg.longitude
        self.z = msg.altitude
        self.compute_odom()


if __name__ == '__main__':
    rospy.init_node("odom_gt")
    print("node initiated")
    odom = odom_gt()
    print("spin")
    rospy.spin()