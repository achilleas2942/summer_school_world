#!/usr/bin/env python3

import rospy
import threading
import socket
import pickle
from geometry_msgs.msg import TwistStamped


class SocketServer(threading.Thread):

    def __init__(self, uri, port_num, ros_node) -> None:
        threading.Thread.__init__(self)
        self.uri = uri
        self.port_num = port_num
        self.addr_info = socket.getaddrinfo(
            uri, port_num, proto=socket.IPPROTO_UDP)
        self.socket = socket.socket(
            self.addr_info[0][0], self.addr_info[0][1], self.addr_info[0][2])
        self.ros_node = ros_node
        self.should_run = True
        self.socket.settimeout(5000)

    def bind_socket(self):
        self.socket.bind((self.uri, self.port_num))
        try:
            while self.should_run and not rospy.is_shutdown():
                data, addr = self.socket.recvfrom(4096)
                data_variable = pickle.loads(data)
                self.ros_node.create_cmd_msg(data_variable)
        except KeyboardInterrupt:
            print('interrupted!')

    def run(self):
        rospy.loginfo("Socket created uri: {sock_uri}, port num: {port_num}".format(
            sock_uri=self.uri, port_num=self.port_num))
        self.bind_socket()


class Pub_Node():

    def __init__(self, topic_name) -> None:
        self._rate = 100
        self.message = TwistStamped()
        self.checksum = 1
        self.publisher = rospy.Publisher(
            topic_name, TwistStamped, queue_size=1)
        self.thread = threading.Thread(target=self.run_node)

    def run_node(self):
        n_rate = rospy.Rate(self._rate)
        while not rospy.is_shutdown():
            self.publisher.publish(self.message)
            self.checksum += 1
            n_rate.sleep()

    def create_cmd_msg(self, data):
        self.message = data

    def start(self):
        self.thread.start()

    def join(self):
        self.thread.join()


if __name__ == '__main__':
    try:
        rospy.init_node('udp_receive_cmd', anonymous=True)
        topic = '/pelican/vel_msg'
        pub_node = Pub_Node(topic)
        udp_socket = SocketServer(
            "127.0.0.1", 30105, pub_node)
        udp_socket.start()
        pub_node.start()
        rospy.spin()
        udp_socket.should_run = False
        udp_socket.join()
    except rospy.ROSInterruptException:
        print("rospy interrupted")