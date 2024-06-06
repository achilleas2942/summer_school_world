#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Float32


class DownlinkDelay:
    def __init__(self):
        self.downlink = Float32()
        self.downlink = 0.0
        self.counter = 0
        self.old_header = 0.0

        self.cmd_vel_sub = rospy.Subscriber(
            "/pelican/vel_msg", TwistStamped, self.callback_cmd_vel
        )
        self.down_delay_pub = rospy.Publisher(
            "/downlink_delay", Float32, queue_size=10
        )


    def callback_cmd_vel(self, msg):
        if (
            msg.header.stamp.to_sec() != 0
            and msg.header.stamp.to_sec() != self.old_header
        ):
            recv_time = rospy.Time.now()
            if recv_time.to_sec() != 0:
                pub_time = msg.header.stamp
                delay = (recv_time - pub_time).to_sec()
                if self.counter < 500:  # sliding window of 500 messages
                    self.counter = self.counter + 1
                self.downlink = np.divide(
                    delay + (self.counter - 1) * self.downlink, self.counter
                )
                self.old_header = msg.header.stamp.to_sec()
                self.down_delay_pub.publish(self.downlink)


if __name__ == "__main__":
    rospy.init_node("downlink_delay_node", anonymous=True)
    node = DownlinkDelay()
    rospy.spin()
