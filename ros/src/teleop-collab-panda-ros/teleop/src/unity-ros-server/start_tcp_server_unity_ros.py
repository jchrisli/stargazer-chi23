#!/usr/bin/env python

"""
Used to start a TCP server that mediates communication between ROS and Unity
"""

import rospy

from ros_tcp_endpoint import TcpServer, RosPublisher, RosSubscriber, RosService

def main():
    ros_node_name = rospy.get_param("/TCP_NODE_NAME", 'TCPServer')
    tcp_server = TcpServer(ros_node_name)
    rospy.loginfo_once(tcp_server.buffer_size)

    # Start the Server Endpoint
    rospy.init_node(ros_node_name, anonymous=True)
    tcp_server.start()
    rospy.spin()


if __name__ == "__main__":
    main()