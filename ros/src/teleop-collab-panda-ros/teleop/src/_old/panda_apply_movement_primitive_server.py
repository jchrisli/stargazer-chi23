#!/usr/bin/env python

from posixpath import join
import numpy as np
from ros_dmp.srv import *
from geometry_msgs.msg import PoseStamped, PoseArray
from sensor_msgs.msg import JointState
from panda_robot import PandaArm
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from teleop_msgs.srv import PosesToDMP, PosesToDMPResponse, PosesToDMPRequest
from franka_interface import ArmInterface
from franka_core_msgs.msg import JointCommand, RobotState
import rospy
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from franka_moveit import PandaMoveGroupInterface
from franka_moveit.utils import create_pose_msg

pub = sub = None
mvt = None
panda_arm = None
panda_move_group = None

# All the messages from recording the current robot poses
executed_joint_positions = []
initial_joint_positions = []
plan = None

def get_joint_positions():
    global executed_joint_positions
    executed_joint_positions.append(panda_arm.angles())
    #print("Length of traj: ", len(executed_joint_positions))


def generate_trajectory_new(req):
    print("Request for generated trajectory received from Unity.")
    res = PosesToDMPResponse()

    dmp_request = GenerateMotionRequest()
    dmp_request.dmp_name = "/home/karthikm/teleop_collab_ws/src/ros_dmp/dmp/weights/" + req.primitive_name.data + ".yaml"
    dmp_request.tau = 1.0
    dmp_request.dt = 0.01

    ### Specify start and end pose

    # Create a custom message with the start and end positions
    # dmp_request.initial_pose = PoseStamped()
    initial_pose_stamped = PoseStamped()
    initial_pose_stamped.header.frame_id = "base"
    initial_pose_stamped.pose = req.start_and_end_poses[0]
    # print(req.start_and_end_poses[0])

    # dmp_request.goal_pose = PoseStamped()
    goal_pose_stamped = PoseStamped()
    goal_pose_stamped.header.frame_id = "base"
    goal_pose_stamped.pose = req.start_and_end_poses[1]
    # print(req.start_and_end_poses[1])

    # Set these to the dmp request initial and goal poses
    dmp_request.initial_pose = initial_pose_stamped
    dmp_request.goal_pose = goal_pose_stamped
    print(dmp_request)

    service_client = rospy.ServiceProxy('/generate_motion_service', GenerateMotion)

    try:
        service_client = rospy.ServiceProxy('/generate_motion_service', GenerateMotion)
        dmp_response = service_client(dmp_request)

        if dmp_response.result == "success":
            poses_to_follow = []
            for i in range(len(dmp_response.cart_traj.cartesian_state)):
                res.generated_trajectory_poses.append(dmp_response.cart_traj.cartesian_state[i].pose)

            global panda_move_group, plan
            print("============ Reference frame: ", panda_move_group.arm_group.get_planning_frame())
            plan, fraction = panda_move_group.plan_cartesian_path(res.generated_trajectory_poses)
            print("Fraction followed: ", fraction)
            print("Num points in joint traj: ", len(plan.joint_trajectory.points))
            res.joint_traj = plan.joint_trajectory

            '''for j in range(len(res.joint_traj.points)):
                print(len(res.joint_traj.points[j].positions))
                panda_arm.exec_position_cmd(res.joint_traj.points[j].positions)
                rospy.sleep(0.02)'''

            return res

    except:
        rospy.loginfo("Service call failed.")


# Request generated motion
def generate_trajectory(req):
    print("Request for generated trajectory received from Unity.")
    res = PosesToDMPResponse()

    dmp_request = GenerateMotionRequest()
    dmp_request.dmp_name = "/home/karthikm/catkin_ws/src/ros_dmp/dmp/weights/" + req.primitive_name.data + ".yaml"
    dmp_request.tau = 1.0
    dmp_request.dt = 0.01

    ### Specify start and end pose

    # Create a custom message with the start and end positions
    # dmp_request.initial_pose = PoseStamped()
    initial_pose_stamped = PoseStamped()
    initial_pose_stamped.header.frame_id = "base"
    initial_pose_stamped.pose = req.start_and_end_poses[0]
    # print(req.start_and_end_poses[0])

    # dmp_request.goal_pose = PoseStamped()
    goal_pose_stamped = PoseStamped()
    goal_pose_stamped.header.frame_id = "base"
    goal_pose_stamped.pose = req.start_and_end_poses[1]
    # print(req.start_and_end_poses[1])

    # Set these to the dmp request initial and goal poses
    dmp_request.initial_pose = initial_pose_stamped
    dmp_request.goal_pose = goal_pose_stamped
    #print(dmp_request)

    service_client = rospy.ServiceProxy('/generate_motion_service', GenerateMotion)

    try:
        service_client = rospy.ServiceProxy('/generate_motion_service', GenerateMotion)
        dmp_response = service_client(dmp_request)

        if dmp_response.result == "success":
            
            global executed_joint_positions
            executed_joint_positions = []

            print("Received generated DMP trajectory.")
            print("Num waypoints generated: ", len(dmp_response.cart_traj.cartesian_state))

            # We received the DMP response so now we can do stuff like create poses and also create a joint trajectory to follow
            joint_traj = JointTrajectory()

            # Get initial pos/rot and save it
            global initial_joint_positions
            initial_joint_positions = panda_arm.angles()

            #TODO: Fix this hack of adding states by moving robot - better to use a planner of some sort
            # Add states by moving the robot along (HACK)
            for i in range(len(dmp_response.cart_traj.cartesian_state)):
                # Need to also create a joint trajectory
                # print(dmp_response.cart_traj.cartesian_state[i])
                new_pose_stamped = PoseStamped()
                curr_pose = dmp_response.cart_traj.cartesian_state[i].pose
                # Append current pose for vis
                res.generated_trajectory_poses.append(curr_pose)
                # Create trajectory point
                pos = np.array([curr_pose.position.x, curr_pose.position.y, curr_pose.position.z])
                rot = np.quaternion(curr_pose.orientation.w, curr_pose.orientation.x, curr_pose.orientation.y,
                                    curr_pose.orientation.z)

                status, j_des = panda_arm.inverse_kinematics(pos, rot)

                '''if status:
                    point = JointTrajectoryPoint()
                    point.positions = j_des
                    joint_traj.points.append(point)'''

                if status:
                    panda_arm.exec_position_cmd(j_des)
                    rospy.sleep(0.02)
                    get_joint_positions()

            # Disable saving of positions

            # Save the trajectory into the response
            for p in range(len(executed_joint_positions)):
                point = JointTrajectoryPoint()
                point.positions = executed_joint_positions[p]
                joint_traj.points.append(point)

            res.joint_traj = joint_traj

            print("Finished saving all poses in generated movement. Sending back to Unity.")

            panda_arm.exec_position_cmd(initial_joint_positions)
            rospy.sleep(0.02)

            return res

    except:
        rospy.loginfo("Service call failed.")


def get_arr_from_pose(pose):
    """
    @param pose: pose to convert into an array
    @return: array containing pose and euler rotation
    """
    pose_euler = euler_from_quaternion(
        [pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z], axes='sxyz')

    arr = np.array([pose.position.x, pose.position.y, pose.position.z,
                    pose_euler[0], pose_euler[1], pose_euler[2]])
    return arr


def return_closest_joint_angles(points_to_check):
    for i in range(len(points_to_check), -1, -1):
        if points_to_check[i] != [None]:
            return points_to_check[i]
        else:
            return None


def main():
    rospy.init_node("apply_movement_primitive")
    global sub

    srv = rospy.Service("generate_traj_from_DMP_service",
                        PosesToDMP, generate_trajectory_new)
    print("Advertising poses to DMP service.")

    # Set up arm interface for moveit cartesian path computation
    global mvt, panda_arm, panda_move_group
    r = ArmInterface()
    panda_move_group = PandaMoveGroupInterface()
    #panda_move_group = PandaMoveGroupInterface(True)
    mvt = r.get_movegroup_interface()
    panda_arm = PandaArm()
    rospy.spin()


if __name__ == '__main__':
    main()
