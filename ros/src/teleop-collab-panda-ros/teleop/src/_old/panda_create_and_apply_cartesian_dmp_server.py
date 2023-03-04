#!/usr/bin/env python

import rospy
import numpy as np
from bolero.representation import CartesianDMPBehavior, DMPBehavior
from bolero.datasets import make_minimum_jerk
from teleop_msgs.srv import CreateDMP, CreateDMPResponse
from teleop_msgs.srv import PosesToDMP, PosesToDMPResponse, PosesToDMPRequest
from geometry_msgs.msg import Pose
from franka_moveit import PandaMoveGroupInterface
from franka_interface import ArmInterface
from panda_robot import PandaArm
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from teleop.msg import DMPIndivPoint

dmp_save_directory = "/home/karthikm/catkin_ws/src/teleop-collab-panda-ros/teleop/dmps/"
panda_move_group = None
panda_arm = None

executed_joint_positions = []
f = None
point_pub = None
pose_pub = None

def generate_trajectory_new(req):
    print("Request for generated trajectory received from Unity.")
    res = PosesToDMPResponse()

    dmp = CartesianDMPBehavior(configuration_file)


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

            for j in range(len(res.joint_traj.points)):
                print(len(res.joint_traj.points[j].positions))
                panda_arm.exec_position_cmd(res.joint_traj.points[j].positions)
                rospy.sleep(0.02)

            return res

    except:
        rospy.loginfo("Service call failed.")


def get_joint_positions():
    global executed_joint_positions
    executed_joint_positions.append(panda_arm.angles())
    #print("Length of traj: ", len(executed_joint_positions))


def generate_trajectory_temp(dmp_generation_req):
    end_pose = dmp_generation_req.start_and_end_poses[1]
    global f
    f.move_to_cartesian_pose(pos=[end_pose.position.x, end_pose.position.y, end_pose.position.z], 
    ori=[end_pose.orientation.w, end_pose.orientation.x, end_pose.orientation.y, end_pose.orientation.z])
    print("moved to end pose")
    
    return PosesToDMPResponse()


# Apply primitive that was chosen
def generate_trajectory(dmp_generation_req):
    # Get the name of the right config and save file
    dmp_generation_res = PosesToDMPResponse()

    dmp = CartesianDMPBehavior(configuration_file=dmp_save_directory + dmp_generation_req.primitive_name.data + ".yaml")
    dmp.init(7, 7)
    dmp.load_config(dmp_save_directory + dmp_generation_req.primitive_name.data + "_config.yaml")
    start_pose = dmp_generation_req.start_and_end_poses[0]
    end_pose = dmp_generation_req.start_and_end_poses[1]
    x0 = np.array([start_pose.position.x, start_pose.position.y, start_pose.position.z])
    g =  np.array([end_pose.position.x, end_pose.position.y, end_pose.position.z])
    q0 =  np.array([start_pose.orientation.w, start_pose.orientation.x, start_pose.orientation.y, start_pose.orientation.z])
    qg =  np.array([end_pose.orientation.w, end_pose.orientation.x, end_pose.orientation.y, end_pose.orientation.z])

    dmp.set_meta_parameters(["x0", "g", "q0", "qg"], [x0, g, q0, qg])
    gen_traj = dmp.trajectory()
    print(gen_traj.shape)

    # We received the DMP response so now we can do stuff like create poses and also create a joint trajectory to follow
    joint_traj = JointTrajectory()

    # Get initial pos/rot and save it
    global initial_joint_positions
    initial_joint_positions = panda_arm.angles()

    # Set up the publishers
    global pose_pub, point_pub

    for p in range(gen_traj.shape[0]):
        gen_pose = Pose()
        gen_pose.position.x = gen_traj[p][0]
        gen_pose.position.y = gen_traj[p][1]
        gen_pose.position.z = gen_traj[p][2]
        gen_pose.orientation.w = gen_traj[p][3]
        gen_pose.orientation.x = gen_traj[p][4]
        gen_pose.orientation.y = gen_traj[p][5]
        gen_pose.orientation.z = gen_traj[p][6]
        dmp_generation_res.generated_trajectory_poses.append(gen_pose)
        pose_pub.publish(gen_pose)
        rospy.sleep(0.01)

        '''pos = np.array([gen_pose.position.x, gen_pose.position.y, gen_pose.position.z])
        rot = np.quaternion(gen_pose.orientation.w, gen_pose.orientation.x, gen_pose.orientation.y,
                            gen_pose.orientation.z)

        status, j_des = panda_arm.inverse_kinematics(pos, rot)

        # Execute cmd
        if status:
            panda_arm.exec_position_cmd(j_des)
            rospy.sleep(0.02)
            get_joint_positions()'''
    
    # Save the trajectory into the response
    '''for p in range(len(executed_joint_positions)):
        point = JointTrajectoryPoint()
        point.positions = executed_joint_positions[p]
        joint_traj.points.append(point)
    
    dmp_generation_res.joint_traj = joint_traj'''

    print(len(dmp_generation_res.generated_trajectory_poses))

    print("============ Reference frame: ", panda_move_group.arm_group.get_planning_frame())

    plan, fraction = panda_move_group.plan_cartesian_path(dmp_generation_res.generated_trajectory_poses)
    print("Fraction followed: ", fraction)
    print("Num points in joint traj: ", len(plan.joint_trajectory.points))
    #dmp_generation_res.joint_traj = plan.joint_trajectory
    print("Finished planning joint trajectory")

    global point_pub
    for i in range(len(plan.joint_trajectory.points)):
        point_pub.publish(plan.joint_trajectory.points[i])
        rospy.sleep(0.01)

    return dmp_generation_res

def record_primitive(dmp_creation_req):
    # Create trajectory to follow
    demo_poses = np.zeros((7, len(dmp_creation_req.waypoint_poses) + 1))
    for i in range(len(dmp_creation_req.waypoint_poses)):
        p = dmp_creation_req.waypoint_poses[i].position
        o = dmp_creation_req.waypoint_poses[i].orientation
        demo_poses[:, i] = (p.x, p.y, p.z, o.w, o.x, o.y, o.z)
        #print(demo_poses[:, i])
        #print(demo_poses[:, i])
    # Add last value
    l_p = dmp_creation_req.waypoint_poses[-1].position
    l_o = dmp_creation_req.waypoint_poses[-1].orientation

    demo_poses[:, -1] = (l_p.x, l_p.y, l_p.z, l_o.w, l_o.x, l_o.y, l_o.z)
    print(demo_poses.shape)
    print(len(dmp_creation_req.waypoint_poses))

    x0 = demo_poses[0:3, 0]
    g = demo_poses[0:3, -1]
    
    q0 = demo_poses[3:, 0]
    qg = demo_poses[3:, -1]

    print(x0)
    print(g)
    print(q0)
    print(qg)

    print(demo_poses[:, -1])

    dmp = CartesianDMPBehavior(len(dmp_creation_req.waypoint_poses) * 0.01, 0.01, 50)
    dmp.init(7, 7)
    dmp.set_meta_parameters(["x0", "g", "q0", "qg"], [x0, g, q0, qg])
    #dmp.set_meta_parameters(["x0", "g"], [x0, g])

    dmp.imitate(demo_poses)
    dmp.save(dmp_save_directory + dmp_creation_req.primitive_name.data + '.yaml')
    dmp.save_config(dmp_save_directory + dmp_creation_req.primitive_name.data + '_config.yaml') 
    print("Created DMP")

    '''zeroq = np.array([0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0])
    X_demo = make_minimum_jerk(x0, g, len(dmp_creation_req.waypoint_poses) * 0.01, 0.01)[0]
    print(np.shape(X_demo))
    X_rot = np.tile(zeroq[3:], (X_demo.shape[1], 1)).T
    print(np.shape(X_rot))
    X_demo = np.vstack((X_demo[:, :, 0], X_rot))[:, :, np.newaxis][:, :, 0]
    print(np.shape(X_demo))
    print(X_demo[:, -1])
    print(X_demo[:, -2])
    print(X_demo[:, 0])'''
    #dmp.imitate(X)


def main():
    rospy.init_node("create_cartesian_dmp_server_node")
    rospy.Service("poses_to_new_MP", CreateDMP, record_primitive)
    print("Advertising DMP creation service.")

    
    rospy.Service("generate_traj_from_DMP_service",
                        PosesToDMP, generate_trajectory)
    print("Advertising poses to DMP service.")

    global point_pub, pose_pub
    point_pub = rospy.Publisher('dmp_gen_point_publisher', JointTrajectoryPoint, queue_size=10)
    pose_pub = rospy.Publisher('dmp_gen_pose_publisher', Pose, queue_size=10)

    global panda_move_group, panda_arm, f
    panda_move_group = PandaMoveGroupInterface()
    panda_arm = PandaArm()
    f = ArmInterface()
    print("Setup panda move group interface")
    rospy.spin()


if __name__ == '__main__':
    main()