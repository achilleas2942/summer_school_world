#!/usr/bin/env python3

import rospy
import random
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TwistStamped


class RandomDelayNode:
    def __init__(self):
        self.odom_sub = rospy.Subscriber(
            "/pelican/odometry_sensor1/odometry", Odometry, self.callback_odometry
        )
        self.cmd_vel_sub = rospy.Subscriber(
            "/cmd_vel", TwistStamped, self.callback_cmd_vel
        )
        self.odom_pub = rospy.Publisher("/odometry", Odometry, queue_size=10)
        self.cmd_vel_pub = rospy.Publisher(
            "/pelican/vel_msg", TwistStamped, queue_size=10
        )

    def callback_odometry(self, msg):
        delay = random.uniform(0.0, 0.0)  # Random delay between 0.01 and 0.05 seconds
        self.odom_pub.publish(msg)

    def callback_cmd_vel(self, msg):
        delay = random.uniform(0.0, 0.0)  # Random delay between 0.01 and 0.05 seconds
        self.cmd_vel_pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node("random_delay_node", anonymous=True)
    node = RandomDelayNode()
    rospy.spin()
