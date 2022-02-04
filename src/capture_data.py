#!/usr/bin/env python

import cv2
from cv_bridge import CvBridge, CvBridgeError
import message_filters
import math
import numpy as np
import rospy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, Image,NavSatFix
from tf.transformations import euler_from_quaternion

class Ros_data_aquisition(object):

    def __init__(self):
        self.pose_w_cov = None
        self.only_pose = None
        self.position_covariance = None
        self.header = None
        self.twist_data = None
        self.twist_data_covariance = None
        self.orientation = None
        self.orientation = None
        self.angular_velocity = None
        self.linear_acceleration = None
        self.image = None
        self.x = None
        self.y = None
        self.z = None

        # self.sub_odom = rospy.Subscriber('odom_gt', Odometry, self.odom_callback)
        # self.sub_imu = rospy.Subscriber('imu', Imu, self.odom_callback)
        # self.sub_image = rospy.Subscriber('image', Image, self.odom_callback)

        # self.sub_odom = message_filters.Subscriber('odom_gt', Odometry)
        self.sub_navfix = message_filters.Subscriber('GPS',NavSatFix)
        self.sub_imu = message_filters.Subscriber('imu_data', Imu)
        self.sub_image = message_filters.Subscriber('image_ground_cam',Image)

        self.bridge = CvBridge()
        self.label_sequence = 1

        self.annot_file = open("/home/jaco/Desktop/Data/data.csv", "w")
        self.annot_file.write("q1,q2,q3,q4,wx,wy,wz,ax,ay,az,x,y,yaw\n")

        # ts = message_filters.TimeSynchronizer([self.sub_image, self.sub_imu, self.sub_navfix], 10)
        ts = message_filters.ApproximateTimeSynchronizer([self.sub_image, self.sub_imu, self.sub_navfix], 10, 0.2, allow_headerless=False)
        ts.registerCallback(self.callback)

        # rospy.loginfo("Subscribed to /" + str(self.topic_name))

    def callback(self,image, imu , gt):
        # self.odom_callback(gt)
        self.navfix_callback(gt)
        self.imu_callback(imu)
        self.image_callback(image)
        q1 = self.orientation.x
        q2 = self.orientation.y
        q3 = self.orientation.z
        q4 = self.orientation.w
        wx = self.angular_velocity.x
        wy = self.angular_velocity.y
        wz = self.angular_velocity.z
        ax = self.linear_acceleration.x
        ay = self.linear_acceleration.y
        az = self.linear_acceleration.z
        x = self.x
        y = self.y
        _,_,yaw = euler_from_quaternion([q1,q2,q3,q4])

        img_name = "/home/jaco/Desktop/Data/imgs/" + str(self.label_sequence) + ".jpg"
        cv2.imwrite(img_name, self.image)
        self.annot_file.write(str(self.label_sequence) + "," + str(q1) + "," + str(q2) + "," + str(q3) + "," + str(q4) + "," + str(wx) + "," + str(wy) + "," + str(wz) + "," + str(ax) + "," + str(ay) + "," + str(az) + "," + str(x) + "," + str(y) + "," + str(yaw) + "\n")
        rospy.loginfo("Data " + str(self.label_sequence))
        self.label_sequence+=1

    def odom_callback(self, msg):
        self.pose_w_cov = msg.pose
        self.only_pose = msg.pose.pose.position
        self.position_covariance = msg.pose.covariance
        self.twist_data = msg.twist.twist
        self.twist_data_covariance = msg.twist.covariance
        self.orientation = msg.pose.pose.orientation

    def imu_callback(self, msg):
        self.orientation = msg.orientation
        self.angular_velocity = msg.angular_velocity
        self.linear_acceleration = msg.linear_acceleration

    def navfix_callback(self, msg):
        self.x = msg.latitude
        self.y = msg.longitude
        self.z = msg.altitude
        

    def image_callback(self, msg):
        try:
            self.image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)


if __name__ == '__main__':
    rospy.init_node('dataset_capturer', anonymous=True)
    odom_sub = Ros_data_aquisition()
    rospy.loginfo("Capturing data...")
    rospy.spin()
    


