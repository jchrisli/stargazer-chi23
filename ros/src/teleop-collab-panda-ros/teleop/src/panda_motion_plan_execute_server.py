#!/usr/bin/python

import rospy
from franka_moveit import PandaMoveGroupInterface
from teleop_msgs.srv import PosesToJointTraj, PosesToJointTrajRequest, PosesToJointTrajResponse
import tf2_ros
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from tf2_geometry_msgs import do_transform_pose
from teleop_msgs.srv import ExecuteTraj, ExecuteTrajResponse, ExecuteTrajRequest
from moveit_msgs.msg import RobotTrajectory
from franka_interface import ArmInterface
from panda_robot import PandaArm
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
from trac_ik_python.trac_ik import IK
import numpy as np
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class PlanAndExecuteMotion:
    def __init__(self):
        self.node = rospy.init_node("generate_execute_traj_from_poses_server_node")
        self.planner_service = rospy.Service("generate_traj_from_poses_service", PosesToJointTraj,
                                             self.generate_motion_plan)
        self.execute_service = rospy.Service("execute_traj_service", ExecuteTraj, self.execute_motion_plan)
        print("Advertising Panda motion planning and execution service.")

        self.mg = PandaMoveGroupInterface(True)
        self.arm = ArmInterface()
        self.panda_arm = PandaArm()

        self.rotation_offset = 0.785398

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.MIN_PLAN_FRACTION = 0.95
        self.planning_attempts = 0
        self.PLANNING_ATTEMPTS_ALLOWED = 3

        rospy.spin()

    def is_close(self, a, b, tol=1e-3):
        return abs(abs(a) - abs(b)) <= tol

    def oppositeSigns(self, a, b):
        return a * b < 0

    def adjust_value(self, curr_rot, prev_rot):
        if self.is_close(curr_rot, prev_rot) and self.oppositeSigns(curr_rot, prev_rot):
            new_curr_rot = -curr_rot
            #print("old:", curr_rot)
            #print("new:", new_curr_rot)
            return new_curr_rot
        else:
            return curr_rot
        
    def generate_motion_plan_manual(self, req):
        joint_traj = JointTrajectory()
        for i in range(len(req.poses)):
            # Need to also create a joint trajectory
            curr_pose = req.poses[i]
            # Append current pose for vis
            # Create trajectory point
            pos = np.array([curr_pose.position.x, curr_pose.position.y, curr_pose.position.z])
            rot = np.quaternion(curr_pose.orientation.w, curr_pose.orientation.x, curr_pose.orientation.y,
                                curr_pose.orientation.z)

            status, j_des = self.panda_arm.inverse_kinematics(pos, rot)
            if status:
                point = JointTrajectoryPoint()
                point.positions = j_des
                joint_traj.points.append(point)

        res = PosesToJointTrajResponse()
        res.joint_traj = joint_traj
        res.success.data = True
        print("Count: ", len(joint_traj.points))
        return res

    def generate_motion_plan_old(self, req):

        '''prev_rot = [0, 0, 0]
        for p in range(len(req.poses)):
            # Check each axis of angles and fix rotations as needed
            curr_rot = euler_from_quaternion([req.poses[p].orientation.x, req.poses[p].orientation.y,
                                         req.poses[p].orientation.z,
                                         req.poses[p].orientation.w])

            new_curr_eul = []
            if p > 0:
                # Check each axis
                for i in range(len(curr_rot)):
                    adjusted_val = self.adjust_value(curr_rot[i], prev_rot[i])
                    new_curr_eul.append(adjusted_val)

                # Now convert back to quat
                new_quat = quaternion_from_euler(new_curr_eul[0], new_curr_eul[1], new_curr_eul[2])
                req.poses[p].orientation.x = new_quat[0]
                req.poses[p].orientation.y = new_quat[1]
                req.poses[p].orientation.z = new_quat[2]
                req.poses[p].orientation.w = new_quat[3]

                # Set current as previous
                prev_rot = new_curr_eul

                print(curr_rot)
                #print(new_curr_eul)

            else:
                prev_rot = curr_rot'''

    def generate_motion_plan(self, req):
        res = PosesToJointTrajResponse()
        res.success.data = False

        while not res.success.data and self.planning_attempts < self.PLANNING_ATTEMPTS_ALLOWED:
            plan, fraction = self.mg.plan_cartesian_path(req.poses)
            print(plan.joint_trajectory.points[15].time_from_start)

            if fraction <= self.MIN_PLAN_FRACTION:
                res.success.data = False
                self.planning_attempts += 1
                print("Failed attempt ", self.planning_attempts)

            else:
                print("Num points in joint traj: ", len(plan.joint_trajectory.points))
                print("Finished planning joint trajectory")
                res.joint_traj = plan.joint_trajectory
                res.success.data = True
                self.planning_attempts += 1


        # Re-time the trajectory
        '''new_traj = self.mg.arm_group.retime_trajectory(self.mg.arm_group.get_current_state(),
                                                         plan.joint_trajectory,
                                                         algorithm="time_optimal_trajectory_generation")
        res.joint_traj = new_traj'''

        # Reset planning attempts
        self.planning_attempts = 0

        return res

    def execute_motion_plan(self, req):
        plan = RobotTrajectory()
        plan.joint_trajectory = req.joint_traj
        plan.joint_trajectory.joint_names = self.arm.joint_names()
        print("Executing traj")

        # Option 1 - Auto
        self.mg.execute_plan(plan)

        # Option 2 - Slow
        '''joint_names = self.arm.joint_names()
        for i in range(len(plan.joint_trajectory.points)):
            curr_q = self.arm.joint_angles()
            joint_command = dict([(joint, plan.joint_trajectory.points[i].positions[j])
                                  for j, joint in enumerate(joint_names)])
            self.arm.move_to_joint_positions(joint_command, use_moveit=False)'''

        # Option 3 - Manual
        '''for i in range(len(plan.joint_trajectory.points)):
            self.panda_arm.exec_position_cmd(plan.joint_trajectory.points[i].positions)
            rospy.sleep(0.2)'''

        res = ExecuteTrajResponse()
        res.success.data = True
        return res


def main():
    motion_planner = PlanAndExecuteMotion()


if __name__ == '__main__':
    main()