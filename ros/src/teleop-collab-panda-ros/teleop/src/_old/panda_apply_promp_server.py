#!/usr/bin/env python

import rospy
import numpy as np
from numpy.core.records import record
from geometry_msgs.msg import PoseStamped, PoseArray
from panda_robot import PandaArm
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from teleop.srv import PosesToDMP, PosesToDMPResponse, PosesToDMPRequest
from franka_interface import ArmInterface
from teleop.srv import CreateDMP, CreateDMPRequest
from rospy_message_converter import message_converter
from promp.ros import TaskProMP
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose
import random
import json

from urdf_parser_py.urdf import Joint
from scipy.spatial.distance import directed_hausdorff
from analysis.data_analysis import DataAnalysis
from tf.transformations import euler_from_quaternion, quaternion_from_euler


class PrompServer(object):
    def __init__(self):
        self._ik = PandaArm()
        self.srv = rospy.Service("/poses_to_DMP_service", PosesToDMP, self.generate_trajectory)
        self.gen_traj = None
        self.gen_waypoints = []
        self.gen_poses = []

    def generate_trajectory(self, mp_generation_req):
        # Create TaskProMP
        mp = TaskProMP('panda')
        mp.ik_solver.set_ik(self._ik)
        print("Created task ProMP")

        # Load the appropriate ProMP
        mp_filename = "/home/karthikm/catkin_ws/src/teleop-collab-panda-ros/teleop/src/saved_promps/" + \
                      mp_generation_req.primitive_name.data + ".json"
        with open(mp_filename) as json_file:
            promp_data = dict(json.load(json_file))
            # print(type(promp_data))
            promp_data_msg = message_converter.convert_dictionary_to_ros_message('teleop/CreateDMP', promp_data,
                                                                                 kind='request')
            # print(promp_data_msg)

        # Add the demos back into the MP using the correct message types: Path and JointTrajectory
        promp_path = Path()
        for i in range(len(promp_data_msg.waypoint_poses)):
            pose_stamped = PoseStamped()
            pose_stamped.header.frame_id = 'base'
            pose_stamped.pose = promp_data_msg.waypoint_poses[i]
            promp_path.poses.append(pose_stamped)

        # Slice trajectory to include only the arm joints
        promp_traj = JointTrajectory()
        promp_traj.joint_names = promp_data_msg.robot_traj.joint_trajectory.joint_names[2:]
        promp_traj.points = promp_data_msg.robot_traj.joint_trajectory.points
        for i in range(len(promp_traj.points)):
            promp_traj.points[i].positions = promp_traj.points[i].positions[2:]
            promp_traj.points[i].velocities = promp_traj.points[i].velocities[2:]
            promp_traj.points[i].accelerations = promp_traj.points[i].accelerations[2:]
            promp_traj.points[i].effort = promp_traj.points[i].effort[2:]

        mp.add_demonstration(promp_path, promp_traj)

        for j in range(4):
            new_demo_path = self.add_noise_to_demo(promp_path)
            mp.add_demonstration(new_demo_path, promp_traj)

        # Create the trajectory as per requirement
        start_pose = mp_generation_req.start_and_end_poses[0]
        start_point = [[start_pose.position.x, start_pose.position.y, start_pose.position.z],
                       [start_pose.orientation.x, start_pose.orientation.y, start_pose.orientation.z,
                        start_pose.orientation.w]]

        # Mid-point
        '''midpoint_pose = mp_generation_req.start_and_end_poses[1]
        midpoint_pose = [[midpoint_pose.position.x, midpoint_pose.position.y, midpoint_pose.position.z],
                         [midpoint_pose.orientation.x, midpoint_pose.orientation.y, midpoint_pose.orientation.z,
                          midpoint_pose.orientation.w]]'''

        # Get end:
        end_pose = mp_generation_req.start_and_end_poses[-1]
        end_point = [[end_pose.position.x, end_pose.position.y, end_pose.position.z],
                     [end_pose.orientation.x, end_pose.orientation.y, end_pose.orientation.z, end_pose.orientation.w]]

        print("NUM VIA POINTS: ", mp.num_viapoints)

        mp.clear_viapoints()
        #mp.add_viapoint(0.5, midpoint_pose)
        mp.set_start(start_point)
        mp.set_goal(end_point)
        # mp.add_viapoint(0., start_point, 1)
        # mp.add_viapoint(1., end_point, 1)

        print("OLD start: ", promp_data_msg.waypoint_poses[0])
        print("OLD end: ", promp_data_msg.waypoint_poses[-1])

        print("NEW start: ", start_point)
        print("NEW end: ", end_point)

        print("NUM VIA POINTS: ", mp.num_viapoints)

        # Create response
        #mean_duration = float(mp.mean_duration)
        #mean_duration = float(mp.mean_duration)
        #new_duration = (float(mp.mean_duration) * 10)
        #print(mean_duration, new_duration)
        self.gen_traj, self.gen_waypoints, failures = mp.generate_trajectory()
        # mp.plot()
        # print("Generated trajectory: ", gen_traj)

        print("Num waypoints: ", len(self.gen_waypoints))
        print("Num traj: ", len(self.gen_traj.points))
        print("Failures: ", failures)

        # print(gen_poses)
        for i in range(len(self.gen_waypoints)):
            new_pose = Pose()
            new_pose.position.x = self.gen_waypoints[i][0]
            new_pose.position.y = self.gen_waypoints[i][1]
            new_pose.position.z = self.gen_waypoints[i][2]

            new_pose.orientation.x = self.gen_waypoints[i][3]
            new_pose.orientation.y = self.gen_waypoints[i][4]
            new_pose.orientation.z = self.gen_waypoints[i][5]
            new_pose.orientation.w = self.gen_waypoints[i][6]
            self.gen_poses.append(new_pose)
            # print(new_pose)

        # Return the response including trajectory and poses
        res = PosesToDMPResponse()
        res.joint_traj = self.gen_traj
        res.generated_trajectory_poses = self.gen_poses

        '''for i in range(len(self.gen_traj.points)):
            mp.ik_solver._ik.exec_position_cmd(self.gen_traj.points[i].positions)
            rospy.sleep(0.025)
        self._reset_generated()'''
        
        # Calculate metrics
        print(len(self.gen_poses))
        #self.get_mse(mp_generation_req.start_and_end_poses, self.gen_poses)
        #self.get_hausdorff_dist(mp_generation_req.start_and_end_poses, self.gen_poses)

        # Reset the generated stuff
        self._reset_generated()

        # Return response
        return res

    def add_noise_to_demo(self, path, noise=0.05):
        for i in range(len(path.poses)):
            path.poses[i].pose.position.x += random.uniform(-noise, noise)
            path.poses[i].pose.position.y += random.uniform(-noise, noise)
            path.poses[i].pose.position.z += random.uniform(-noise, noise)

            path.poses[i].pose.orientation.x += random.uniform(-noise, noise)
            path.poses[i].pose.orientation.y += random.uniform(-noise, noise)
            path.poses[i].pose.orientation.z += random.uniform(-noise, noise)
            path.poses[i].pose.orientation.w += random.uniform(-noise, noise)

        return path

    # Reset generated trajectory plus poses
    def _reset_generated(self):
        self.gen_traj = None
        self.gen_poses = []
        self.gen_waypoints = []
    
    def get_hausdorff_dist(self, recorded_poses, generated_poses):
        """
        Get distance between two pose arrays
        @param recorded_poses:
        @param generated_poses:
        """
        # Get recorded poses array
        demo_pose_array = []
        for i in range(len(recorded_poses)):
            demo_pose = self.get_arr_from_pose(recorded_poses[i])
            demo_pose = demo_pose[0:3]
            demo_pose_array.append(demo_pose)
        demo_pose_array = np.asarray(demo_pose_array)
        print("DEMO POSE ARRAY SHAPE: ", demo_pose_array.shape)

        # Get generated poses array
        gen_pose_array = []
        for j in range(len(generated_poses)):
            gen_pose = self.get_arr_from_pose(generated_poses[j])
            gen_pose = gen_pose[0:3]
            gen_pose_array.append(gen_pose)
        gen_pose_array = np.asarray(gen_pose_array)
        print("GEN POSE ARRAY SHAPE: ", gen_pose_array.shape)

        haus_dist = directed_hausdorff(demo_pose_array, gen_pose_array)
        print("HAUSDORFF DIST: ", haus_dist)


    def get_mse(self, req_poses, gen_poses):
        """
        Gets MSE between two poses
        @param req_poses: pose array containing several poses (2 or more)
        @param gen_cartesian_state: custom message containing poses generated from DMP
        """
        print("Getting DMP metrics..")
        start_pose_req = self.get_arr_from_pose(req_poses[0])
        start_pose_gen = self.get_arr_from_pose(gen_poses[0])
        start_mse = DataAnalysis.mean_squared_error(start_pose_req, start_pose_gen)
        print("Start mse: ", start_mse)

        end_pose_req = self.get_arr_from_pose(req_poses[-1])
        end_pose_gen = self.get_arr_from_pose(gen_poses[-1])
        end_mse = DataAnalysis.mean_squared_error(end_pose_req, end_pose_gen)
        print("End mse: ", end_mse)


    def get_arr_from_pose(self, pose):
        """
        @param pose: pose to convert into an array
        @return: array containing pose and euler rotation
        """
        pose_euler = euler_from_quaternion(
            [pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z], axes='sxyz')

        arr = np.array([pose.position.x, pose.position.y, pose.position.z,
                        pose_euler[0], pose_euler[1], pose_euler[2]])
        return arr


def main():
    rospy.init_node("apply_movement_primitive")
    promp_server = PrompServer()
    rospy.spin()


if __name__ == '__main__':
    main()
