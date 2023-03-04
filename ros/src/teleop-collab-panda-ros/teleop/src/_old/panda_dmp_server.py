#!/usr/bin/env python
import rospy
from dmp.msg import *
from dmp.srv import *
from teleop.srv import PosesToDMP, PosesToDMPResponse, PosesToDMPRequest
from franka_interface import ArmInterface
from panda_robot import PandaArm
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
import numpy as np
import roslib

roslib.load_manifest('dmp')

panda_arm = None

# Learn a DMP from demonstration data
def makeLFDRequest(dims, traj, dt, K_gain,
                   D_gain, num_bases):
    demotraj = DMPTraj()

    for i in range(len(traj)):
        pt = DMPPoint()
        pt.positions = traj[i].positions[2:]
        demotraj.points.append(pt)
        demotraj.times.append(dt*i)

    k_gains = [K_gain]*dims
    d_gains = [D_gain]*dims

    print("Starting LfD...")
    rospy.wait_for_service('learn_dmp_from_demo')
    try:
        lfd = rospy.ServiceProxy('learn_dmp_from_demo', LearnDMPFromDemo)
        resp = lfd(demotraj, k_gains, d_gains, num_bases)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)
    print("LfD done")

    return resp

# Set a DMP as active for planning
def makeSetActiveRequest(dmp_list):
    try:
        sad = rospy.ServiceProxy('set_active_dmp', SetActiveDMP)
        sad(dmp_list)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)

# Generate a plan from a DMP
def makePlanRequest(x_0, x_dot_0, t_0, goal, goal_thresh,
                    seg_length, tau, dt, integrate_iter):
    print("Starting DMP planning...")
    rospy.wait_for_service('get_dmp_plan')
    try:
        gdp = rospy.ServiceProxy('get_dmp_plan', GetDMPPlan)
        resp = gdp(x_0, x_dot_0, t_0, goal, goal_thresh,
                   seg_length, tau, dt, integrate_iter)
    except rospy.ServiceException as e:
        print("Service call failed: %s" % e)
    print("DMP planning done")

    return resp

# Request generated motion
def generate_trajectory(req):
    print("Request for generated trajectory received from Unity.")
    res = PosesToDMPResponse()

    # First create the DMP
    dims = 7
    dt = 1.0
    K = 100
    D = 2.0 * np.sqrt(K)
    num_bases = 4
    traj = req.initial_traj.points
    resp = makeLFDRequest(dims, traj, dt, K, D, num_bases)

    # Set it as the active DMP
    makeSetActiveRequest(resp.dmp_list)

    # Figure out the IK for the start and end poses
    # Create trajectory point
    js_start_and_end = []
    global panda_arm

    '''for i in range(len(req.start_and_end_poses)):
        print(req.start_and_end_poses[i])
        pos = np.array([[req.start_and_end_poses[i].position.x], [req.start_and_end_poses[i].position.y], [req.start_and_end_poses[i].position.z]])
        rot = np.quaternion(req.start_and_end_poses[i].orientation.w, req.start_and_end_poses[i].orientation.x, 
        req.start_and_end_poses[i].orientation.y, req.start_and_end_poses[i].orientation.z)
        #print(pos, rot)

        status, j_des = panda_arm.inverse_kinematics(pos, rot, seed=req.initial_traj.points[0].positions[2:])
        if status:
            js_start_and_end.append(j_des)
            #print("Added joint state")
        else:
            print("ERROR")
    
    print(js_start_and_end)   ''' 

    # Get start pose fw
    fwd_pos, fwd_ori = panda_arm.forward_kinematics(joint_angles = req.initial_traj.points[0].positions[2:])
    #print(fwd_pos, fwd_ori) # returns w, x, y, z
    #print(req.start_and_end_poses[0])
    #js.js_start_and_end.append()
    status, j_des = panda_arm.inverse_kinematics(fwd_pos, fwd_ori) # sending in w, x, y, z
    if status:
        js_start_and_end.append(j_des)

    # Get start pose fw
    fwd_pos, fwd_ori = panda_arm.forward_kinematics(joint_angles = req.initial_traj.points[-1].positions[2:])
    #print(fwd_pos, fwd_ori)
    #print(req.start_and_end_poses[1])
    #js.js_start_and_end.append()
    status, j_des = panda_arm.inverse_kinematics(fwd_pos, fwd_ori)
    if status:
        js_start_and_end.append(j_des)
    
    print(js_start_and_end)

    # Get end pose fw

    # Now, generate a plan
    #x_0 = req.initial_traj.points[0].positions[2:]
    x_0 = js_start_and_end[0]
    x_dot_0 = [0, 0, 0, 0, 0, 0, 0, 0]
    t_0 = 0
    #goal = req.initial_traj.points[-1].positions[2:]
    goal = js_start_and_end[1]
    goal_thresh = [0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2]
    seg_length = -1  # Plan until convergence to goal
    tau = 2 * resp.tau
    dt = 1.0
    integrate_iter = 5  # dt is rather large, so this is > 1
    dmp_plan_resp = makePlanRequest(x_0, x_dot_0, t_0, goal, goal_thresh,
                           seg_length, tau, dt, integrate_iter)

    #print(dmp_plan_resp)

    # Create a trajectory to return
    res.joint_traj = JointTrajectory()
    for i in range(len(dmp_plan_resp.plan.points)):
        traj_point = JointTrajectoryPoint()
        traj_point.positions = dmp_plan_resp.plan.points[i].positions
        traj_point.velocities = dmp_plan_resp.plan.points[i].velocities
        res.joint_traj.points.append(traj_point)

        # Also create waypoints with the plans
        position, rotation = panda_arm.forward_kinematics(joint_angles = dmp_plan_resp.plan.points[i].positions)
        new_pose = Pose()
        new_pose.position.x = position[0]
        new_pose.position.y = position[1]
        new_pose.position.z = position[2]
        new_pose.orientation.x = rotation.x
        new_pose.orientation.y = rotation.y
        new_pose.orientation.z = rotation.z
        new_pose.orientation.w = rotation.w
        
        res.generated_trajectory_poses.append(new_pose)
        #print(new_pose)
    
    return res


    '''dmp_request = GenerateMotionRequest()
    dmp_request.dmp_name = "/home/karthikm/catkin_ws/src/ros_dmp/weights/" + req.primitive_name.data + ".yaml"
    dmp_request.tau = 1.0
    dmp_request.dt = 0.01

    ### Specify start and end pose

    # Create a custom message with the start and end positions
    #dmp_request.initial_pose = PoseStamped()
    initial_pose_stamped = PoseStamped()
    initial_pose_stamped.header.frame_id = "base"
    initial_pose_stamped.pose = req.start_and_end_poses[0]
    #print(req.start_and_end_poses[0])

    #dmp_request.goal_pose = PoseStamped()
    goal_pose_stamped = PoseStamped()
    goal_pose_stamped.header.frame_id = "base"
    goal_pose_stamped.pose = req.start_and_end_poses[1]
    #print(req.start_and_end_poses[1])

    # Set these to the dmp request initial and goal poses
    dmp_request.initial_pose = initial_pose_stamped
    dmp_request.goal_pose = goal_pose_stamped
    print(dmp_request)

    service_client = rospy.ServiceProxy('/generate_motion_service', GenerateMotion)'''
    #dmp_response = service_client(dmp_request)
    #if dmp_response.result == "success":
        #print("YES")



def main():
    rospy.init_node("apply_movement_primitive")
    global sub

    srv = rospy.Service("poses_to_DMP_service",
                        PosesToDMP, generate_trajectory)
    print("Advertising poses to DMP service.")


    # Set up arm interface for moveit cartesian path computation
    global mvt, panda_arm
    r = ArmInterface()
    global mvt
    mvt = r.get_movegroup_interface()
    global panda_arm
    panda_arm = PandaArm()

    rospy.spin()

if __name__ == '__main__':
    main()






