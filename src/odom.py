#!/usr/bin/env python

from math import cos, sin, tan
import rospy
import tf
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry

class ackerman_odom():
    def __init__(self):
        self.pub = rospy.Publisher('odom', Odometry, queue_size=10)
        self.sub = rospy.Subscriber('/gol/joint_states',JointState,self.callback)
        self.odom_broadcaster = tf.TransformBroadcaster()

        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()

        self.x = 0
        self.y = 0
        self.th = 0

        self.STEERING_FACTOR = 18.0
        self.BASELINE = 1.3
        self.INTERAXIS = 1.765

        self.name = []
        self.position = []
        self.velocity = []
        self.effort = []

    def compute_odom(self):
        v_l = self.velocity[1]
        v_r = self.velocity[0]
        steering = self.position[2]

        self.current_time = rospy.Time.now()
        dt = (self.current_time - self.last_time).to_sec()*0.1
       
        speed = (v_l + v_r)/2
         # ackermam model
        omega = speed*tan(steering)/self.INTERAXIS
        #  diff model
        # omega = (v_r - v_l)/self.BASELINE 

        delta_th = omega * dt
        speed_x = speed * cos(self.th + delta_th/2.0)
        speed_y = speed * sin(self.th + delta_th/2.0)

        delta_x = speed_x * dt 
        delta_y = speed_y * dt
        
        self.x += delta_x
        self.y += delta_y
        self.th += delta_th

        # since all odometry is 6DOF we'll need a quaternion created from yaw
        self.last_time = self.current_time
        odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)

        # first, we'll publish the transform over tf
        self.odom_broadcaster.sendTransform(
            (self.x, self.y, 0.),
            odom_quat,
            self.current_time,
            "base_link",
            "odom"
        )

        # next, we'll publish the odometry message over ROS
        odom = Odometry()
        odom.header.stamp = self.current_time
        odom.header.frame_id = "odom"

        # set the position
        odom.pose.pose = Pose(Point(self.x, self.y, 0.), Quaternion(*odom_quat))

        # set the velocity
        odom.child_frame_id = "base_link"
        odom.twist.twist = Twist(Vector3(speed_x, speed_x, 0), Vector3(0, 0, omega))

        # publish the message
        self.pub.publish(odom)

        

    def callback(self,msg):
        self.name = msg.name
        self.position = msg.position
        self.velocity = msg.velocity
        self.effort = msg.effort
        self.compute_odom()


if __name__ == '__main__':
    rospy.init_node("odom")
    print("node initiated")
    odom = ackerman_odom()
    print("spin")
    rospy.spin()