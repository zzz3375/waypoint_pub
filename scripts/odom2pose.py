#!/usr/bin/env python

import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Vector3
import tf.transformations

class OdomToMavros:
    def __init__(self):
        rospy.init_node('odom_to_mavros', anonymous=True)
        
        # Subscribers
        self.odom_sub = rospy.Subscriber('/Odometry', Odometry, self.odom_cb)
        
        # Publishers
        self.vision_pose_pub = rospy.Publisher('/uav0/mavros/vision_pose/pose', PoseStamped, queue_size=10)
        self.vision_speed_pub = rospy.Publisher('/uav0/mavros/vision_speed/speed_vector', Vector3, queue_size=10)
        
        # Variables
        self.start_time = rospy.Time.now()
        self.last_time = rospy.Time.now()
        self.warmup_duration = rospy.Duration(60.0)  # Warm up seconds
        self.warmed_up = False
        
        rospy.loginfo("Odom to MAVROS node initialized")
        
    def odom_cb(self, msg):
        # Current time
        current_time = rospy.Time.now()
        elapsed = current_time - self.start_time

        # Check warmup status
        if not self.warmed_up:
            if elapsed < self.warmup_duration:
                # Still in warmup: red log, no forwarding
                self.warmed_up = False
            else:
                # Warmup just completed
                self.warmed_up = True
                rospy.loginfo("Warmup complete, starting forwarding")

        # Convert Odometry to PoseStamped
        vision_pose = PoseStamped()
        vision_pose.header.stamp = current_time
        vision_pose.header.frame_id = "world"
        vision_pose.pose = msg.pose.pose

        # Convert Odometry twist to speed vector
        speed_vector = Vector3()
        speed_vector.x = msg.twist.twist.linear.x
        speed_vector.y = msg.twist.twist.linear.y
        speed_vector.z = msg.twist.twist.linear.z

        # Calculate yaw from quaternion
        quat = (
            vision_pose.pose.orientation.x,
            vision_pose.pose.orientation.y,
            vision_pose.pose.orientation.z,
            vision_pose.pose.orientation.w
        )
        euler = tf.transformations.euler_from_quaternion(quat)
        yaw = math.degrees(euler[2])  # Convert to degrees

        if self.warmed_up:
            # Publish the messages (only after warmup)
            self.vision_pose_pub.publish(vision_pose)
            self.vision_speed_pub.publish(speed_vector)

            # Green log
            print("\033[92mPosition: x={:.2f}, y={:.2f}, z={:.2f} | Yaw: {:.2f}°\033[0m".format(
                vision_pose.pose.position.x,
                vision_pose.pose.position.y,
                vision_pose.pose.position.z,
                yaw
            ))
        else:
            # Red log during warmup, no forwarding
            print("\033[91mPosition: x={:.2f}, y={:.2f}, z={:.2f} | Yaw: {:.2f}°\033[0m".format(
                vision_pose.pose.position.x,
                vision_pose.pose.position.y,
                vision_pose.pose.position.z,
                yaw
            ))

        self.last_time = current_time

if __name__ == '__main__':
    try:
        converter = OdomToMavros()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
