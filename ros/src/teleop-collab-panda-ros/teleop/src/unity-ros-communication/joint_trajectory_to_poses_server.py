#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, Pose
from teleop_msgs.srv import JointTrajectoryToPoses, JointTrajectoryToPosesResponse, JointTrajectoryToPosesRequest, \
    ReturnOverlaps, ReturnOverlapsRequest
from teleop_msgs.srv import PoseToMotionPlanRequest, PoseToMotionPlanResponse, PoseToMotionPlan
from trajectory_msgs.msg import JointTrajectoryPoint
from panda_robot import PandaArm
import numpy as np

"""
This script takes a joint trajectory as a request and returns an array of poses that we can use to visualize a trajectory in 3D coordinates in Unity.
Requires the panda_cmds_sub script to be running to retrieve the PandaRobot class object.
"""


class TrajectoryToPoses:
    def __init__(self):
        self.node = rospy.init_node("joint_trajectory_to_poses_server_node")
        self.traj_to_poses_srv = rospy.Service("joint_trajectory_to_poses_service", JointTrajectoryToPoses,
                                               self.convert_traj_to_poses)
        self.poses_to_motion_plan_srv = rospy.Service("pose_to_motion_plan_service", PoseToMotionPlan,
                                                      self.go_to_cartesian_point)
        self.poses_pub = rospy.Publisher('convert_joint_state_to_pose_publisher', PoseStamped, queue_size=1)
        self.panda_arm = PandaArm()
        self.arm_joint_ind = 2
        rospy.spin()

    @staticmethod
    def go_to_cartesian_point(self, req):
        self.panda_arm.move_to_cartesian_pose(
            pos=[req.goal_pose.position.x, req.goal_pose.position.y, req.goal_pose.position.z],
            ori=[req.goal_pose.orientation.w, req.goal_pose.orientation.x,
                 req.goal_pose.orientation.y, req.goal_pose.orientation.z])
        res = PoseToMotionPlanResponse()
        res.success.data = True
        return res

    # Callback function to the request generated from Unity to transform joint traj. to poses
    def convert_traj_to_poses(self, poses_request):
        """Function that converts joint trajectory to poses

        Args:
            poses_request ([type]): JointTrajectoryToPosesRequest (custom)

        Returns:
            [type]: JointTrajectoryToPosesResponse (custom)
        """
        # Get each traj. point converted to cartesian pose (for visualization)

        print("Request to convert joint trajectory to poses received.")
        poses_response = JointTrajectoryToPosesResponse()
        poses_response.poses = []

        for i in range(len(poses_request.joint_trajectory.points)):
            # As the arm joints start from index 2
            joint_angles = poses_request.joint_trajectory.points[i].positions[self.arm_joint_ind:]
            # print(joint_angles)
            position, rotation = self.panda_arm.forward_kinematics(joint_angles=joint_angles)
            new_pose = PoseStamped()  # type: PoseStamped
            new_pose.header.stamp.secs = poses_request.joint_trajectory.points[i].time_from_start.secs
            new_pose.header.stamp.nsecs = poses_request.joint_trajectory.points[i].time_from_start.nsecs
            new_pose.pose.position.x = position[0]
            new_pose.pose.position.y = position[1]
            new_pose.pose.position.z = position[2]
            new_pose.pose.orientation.x = rotation.x
            new_pose.pose.orientation.y = rotation.y
            new_pose.pose.orientation.z = rotation.z
            new_pose.pose.orientation.w = rotation.w
            self.poses_pub.publish(new_pose)
            rospy.sleep(0.001)

        print("Transformed joint trajectory into poses.")

        # Send the trajectory to the segmentation service to get the segments
        # resp = save_and_request_segmentation(request)
        # for i in range(len(resp.overlaps)):
        #   print(resp.overlaps[i].data)
        # response.overlaps = resp.overlaps

        return poses_response

    # Perform segmentation of joint trajectory
    @staticmethod
    def save_and_request_segmentation(self, request):
        """Function that requests segmentation from external service and sends it back to Unity

        Args:
            request ([type]): JointTrajectoryToPosesRequest (custom)

        Returns:
            [type]: ReturnOverlapsResponse (custom)
        """
        # Save the trajectory as a numpy array
        demo_arr = np.zeros((len(request.joint_trajectory.points), len(request.joint_trajectory.points[0].positions) + 1))
        # print(np.shape(demo_arr))

        demo_start_time = 0
        for i in range(len(request.joint_trajectory.points)):
            for j in range(self.arm_joint_ind, len(request.joint_trajectory.points[i].positions) + 1):
                # print(j)
                if j <= len(request.joint_trajectory.points[i].positions) - 1:
                    demo_arr[i, j] = request.joint_trajectory.points[i].positions[j]
                else:
                    original_time = request.joint_trajectory.points[i].time_from_start.secs + \
                                    request.joint_trajectory.points[i].time_from_start.nsecs * pow(10, -9)
                    if i == 0:
                        demo_start_time = original_time
                        actual_time = 0
                    else:
                        actual_time = original_time - demo_start_time

                    demo_arr[i, j] = actual_time
                    # print(demo_arr[i, j])

        # Save the demonstration to file
        #np.savetxt(rospkg.RosPack().get_path('teleop') + '/demos/demo.txt', demo_arr)

        # Call service to request segmentation
        bool_req = ReturnOverlapsRequest()
        bool_req.data.data = True
        seg_service = rospy.ServiceProxy('segment_demonstration_server', ReturnOverlaps)
        response = seg_service(bool_req)
        #for o in response.overlaps:
            #print(o.data)
        return response


def main():
    TrajectoryToPoses()


if __name__ == '__main__':
    main()
