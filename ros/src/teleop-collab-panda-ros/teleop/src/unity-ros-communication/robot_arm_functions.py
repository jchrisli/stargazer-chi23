#!/usr/bin/env python

import rospy
from teleop_msgs.srv import JointStateToPose, JointStateToPoseRequest, JointStateToPoseResponse, PublishJointState, \
    PublishJointStateRequest, PublishJointStateResponse

# Requires other script to be working at the same time
from geometry_msgs.msg import Pose
from panda_robot import PandaArm
from sensor_msgs.msg import JointState


class RobotArmFunctions:
    def __init__(self):
        self.node = rospy.init_node("return_ee_node")

        self.ee_pose_pub = rospy.Publisher('ee_pose_publisher', Pose, queue_size=10)
        self.rate = rospy.Rate(5)

        # Convert joint state into a pose
        self.joint_state_to_poses_srv = rospy.Service("joint_state_to_poses_service",
                                                      JointStateToPose, self.convert_joint_state_to_pose)
        print("Started service to convert joint state to pose")

        # Service to take joint state request and execute it
        self.move_to_joint_state_srv = rospy.Service("move_to_joint_state_service",
                                                     PublishJointState, self.execute_joint_state_cmd)
        print("Started service to publish joint state for execution")

        self.panda_arm = PandaArm()

        self.publish_ee()

        rospy.spin()

    @staticmethod
    def convert_joint_state_to_pose(self, pose_req):
        joint_angles = pose_req.joint_state.position
        position, rotation = self.panda_arm.forward_kinematics(joint_angles=joint_angles)

        pose_response = JointStateToPoseResponse()
        pose_response.pose = Pose()

        pose_response.pose.position.x = position[0]
        pose_response.pose.position.y = position[1]
        pose_response.pose.position.z = position[2]
        pose_response.pose.orientation.x = rotation.x
        pose_response.pose.orientation.y = rotation.y
        pose_response.pose.orientation.z = rotation.z
        pose_response.pose.orientation.w = rotation.w

        print("Retrieved pose")

        return pose_response

    # Used for executing a joint state command
    @staticmethod
    def execute_joint_state_cmd(self, req):
        # Only getting arm positions
        self.panda_arm.exec_position_cmd(req.joint_state.position)
        print("Executing arm")
        rospy.sleep(0.02)
        res = PublishJointStateResponse()
        res.success = True
        return res

    def publish_ee(self):
        while not rospy.is_shutdown():
            angles = self.panda_arm.angles()
            pos, ori = self.panda_arm.forward_kinematics(joint_angles=angles)
            #pos, ori = self.panda_arm.ee_pose()
            pose_msg = Pose()
            pose_msg.position.x = pos[0]
            pose_msg.position.y = pos[1]
            pose_msg.position.z = pos[2]
            pose_msg.orientation.w = ori.w
            pose_msg.orientation.x = ori.x
            pose_msg.orientation.y = ori.y
            pose_msg.orientation.z = ori.z
            self.ee_pose_pub.publish(pose_msg)
            self.rate.sleep()


def main():
    RobotArmFunctions()


if __name__ == '__main__':
    main()
