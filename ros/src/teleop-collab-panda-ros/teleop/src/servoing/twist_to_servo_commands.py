#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, TwistStamped


class TwistToServo:
    def __init__(self):
        self.node = rospy.init_node("panda_twist_to_servo_node")
        # Change this name according to the topic being published to
        self.SUBSCRIBED_TOPIC = "/joy"
        # Assuming we get a Twist message
        self.subscriber = rospy.Subscriber(self.SUBSCRIBED_TOPIC, Twist, self.send_joy_to_servo, queue_size = 1)
        # We need to convert this to a TwistStamped and send it to the MoveIt Servo server
        self.publisher = rospy.Publisher("servoing/twist_stamped_publisher", TwistStamped, queue_size=1)
        print("Listening for new twist messages")

        rospy.spin()

    def send_joy_to_servo(self, twist_msg):
        print("Received newest twist message; sending to servo server")
        new_twist_stamped = TwistStamped()
        new_twist_stamped.header.stamp = rospy.Time.now()
        new_twist_stamped.header.frame_id = 'panda_link0'
        new_twist_stamped.twist = twist_msg
        print(twist_msg)
        self.publisher.publish(new_twist_stamped)


def main():
    TwistToServo()


if __name__ == '__main__':
    main()
