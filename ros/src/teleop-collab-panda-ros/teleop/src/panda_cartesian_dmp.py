#!/usr/bin/env python3

import rospy
import numpy as np
from movement_primitives.dmp import CartesianDMP
import movement_primitives.io as mp_io
from geometry_msgs.msg import Pose
from trajectory_msgs.msg import JointTrajectoryPoint
from teleop_msgs.srv import CreateDMP, CreateDMPResponse
from teleop_msgs.srv import PosesToDMP, PosesToDMPResponse, PosesToDMPRequest
from teleop_msgs.srv import PosesToJointTraj, PosesToJointTrajRequest, PosesToJointTrajResponse

dmp_save_directory = "/home/karthikm/teleop_collab_ws/src/teleop-collab-panda-ros/teleop/dmps/"

point_pub = None
pose_pub = None
mg = None

# This responds to generation requests but publishes indiv. points since services have
# a limit so service resp. is empty
def generate_primitive(dmp_generation_req):
    dmp_generation_res = PosesToDMPResponse()

    # Load the DMP 
    dmp = mp_io.read_json(dmp_save_directory + dmp_generation_req.primitive_name.data + ".json")

    start_pose = dmp_generation_req.start_and_end_poses[0]
    end_pose = dmp_generation_req.start_and_end_poses[1]
    s = np.array([start_pose.position.x, start_pose.position.y, start_pose.position.z, \
        start_pose.orientation.w, start_pose.orientation.x, start_pose.orientation.y, \
        start_pose.orientation.z])
    g = np.array([end_pose.position.x, end_pose.position.y, end_pose.position.z, \
        end_pose.orientation.w, end_pose.orientation.x, end_pose.orientation.y, \
        end_pose.orientation.z])

    dmp.configure(start_y=s, goal_y=g)
    _, Y = dmp.open_loop()

    # Publish the poses and points now
    global pose_pub, point_pub, mg

    # Publish poses
    for p in range(Y.shape[0]):
        gen_pose = Pose()
        gen_pose.position.x = Y[p][0]
        gen_pose.position.y = Y[p][1]
        gen_pose.position.z = Y[p][2]
        gen_pose.orientation.w = Y[p][3]
        gen_pose.orientation.x = Y[p][4]
        gen_pose.orientation.y = Y[p][5]
        gen_pose.orientation.z = Y[p][6]
        dmp_generation_res.generated_trajectory_poses.append(gen_pose)

        pose_pub.publish(gen_pose)
        rospy.sleep(0.005)

    # Plan cartesian trajectory
    # Call service to get motion plan
    motion_plan_service = rospy.ServiceProxy('generate_traj_from_poses_service', PosesToJointTraj)
    req = PosesToJointTrajRequest()
    req.poses = dmp_generation_res.generated_trajectory_poses
    res = motion_plan_service(req)

    # If it is successful then we can send it back and mark it as a success
    if res.success.data:
        dmp_generation_res.joint_traj = res.joint_traj
        dmp_generation_res.success.data = res.success.data

        global point_pub
        for i in range(len(dmp_generation_res.joint_traj.points)):
            point_pub.publish(dmp_generation_res.joint_traj.points[i])
            rospy.sleep(0.005)

    # To avoid issues of transporting trajectory and points information in a service we clear it and only keep success
    dmp_generation_res = PosesToDMPResponse()
    dmp_generation_res.success.data = True
    return dmp_generation_res

def record_primitive(dmp_creation_req):
    dt = 0.05
    n_steps = len(dmp_creation_req.waypoint_poses) 
    execution_time = n_steps * dt
    dmp = CartesianDMP(execution_time=execution_time, dt=dt,
    n_weights_per_dim=20)

    # Create demo
    T = np.linspace(0, execution_time, n_steps)
    Y = np.zeros((len(dmp_creation_req.waypoint_poses), 7))

    for i in range(len(dmp_creation_req.waypoint_poses)):
        p = dmp_creation_req.waypoint_poses[i].position
        o = dmp_creation_req.waypoint_poses[i].orientation
        Y[i, :] = (p.x, p.y, p.z, o.w, o.x, o.y, o.z)
    
    dmp.imitate(T, Y)
    mp_io.write_json(dmp_save_directory + dmp_creation_req.primitive_name.data + ".json", dmp)

    dmp_creation_res = CreateDMPResponse()

    dmp_creation_res.success.data = True

    return dmp_creation_res


def main():
    rospy.init_node("create_cartesian_dmp_server_node")

    rospy.Service("poses_to_new_MP", CreateDMP, record_primitive)
    print("Advertising DMP creation service.")

    rospy.Service("generate_traj_from_DMP_service",
                        PosesToDMP, generate_primitive)
    print("Advertising poses to DMP service.")

    global point_pub, pose_pub, mg
    point_pub = rospy.Publisher('dmp_gen_point_publisher', JointTrajectoryPoint, queue_size=10)
    pose_pub = rospy.Publisher('dmp_gen_pose_publisher', Pose, queue_size=10)

    rospy.spin()


if __name__ == '__main__':
    main()
