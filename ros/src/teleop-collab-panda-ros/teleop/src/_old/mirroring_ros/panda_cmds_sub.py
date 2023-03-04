#!/usr/bin/env python

import rospy
import numpy as np
import time
import json
from teleop.msg import MovePandaCommand
from teleop.srv import RecordStateActionPairs, RecordStateActionPairsResponse, CreateDMP
from panda_robot import PandaArm
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from sensor_msgs.msg import JointState
import random

'''
Used to send commands from Unity but only the controller commands
to be sent to the Gazebo robot and subscribed to joint states.
'''
class PandaRobot():
    def __init__(self):
        # Establish the correct axis mappings
        self.linear_x_map = 0
        self.linear_y_map = 1
        self.linear_z_map = 4

        self.angular_x_map = 0
        self.angular_y_map = 1
        self.angular_z_map = 4

        # Button mappings
        self.switch_to_angular = 5
        self.gripper_open = 1
        self.gripper_close = 0
        self.gripper_moving = False
        self.msg_count = 0

        # Scales
        self.scale_linear = 0.01
        self.scale_angular = 0.05

        # Joystick thresholds
        self.input_thresh = 0.01

        # Movement values
        self.gripper_increment = 0.1999 / 50
        self.gripper_width = 0.0001
        self.MIN_WIDTH = 0.0001
        self.MAX_WIDTH = 0.07

        # Create instance of robot
        self.robot = PandaArm()
        self.has_gripper = self.robot.get_gripper() is not None

        self.linear_plus_angular = {'angular_x': 0, 'angular_y': 0, 'angular_z': 0,
                                    'linear_x': 0, 'linear_y': 0, 'linear_z': 0}

        # Demonstration-related characteristics
        self.is_recording = False
        self.state_action_pairs = []
        self.state_action_dictionary = {"State": [], "Action": []}
        self.demo_data = []
        self.time_elapsed = 0
        
        # Set up service to start/stop recording of state-action pairs for behavioral cloning
        self.current_joint_state = None
        #recording_service = rospy.Service(
            #"state_action_pair_recording_service", RecordStateActionPairs, self.respond_to_recording_request)
        
        # Set up service for KMP recordings
        #self.create_srv = rospy.Service("poses_to_new_KMP", CreateDMP, self.record_kmp)


    # Callback to subscriber for current joint state of robot
    def save_current_joint_state(self, state):
        self.current_joint_state = state
        # print(self.current_joint_state)

    # Execute movement request
    def execute_command_callback(self, msg):
        exec_command = False

        if msg.angular_control.data == False:
            if not (abs(msg.x_axis.data) < self.input_thresh and abs(msg.y_axis.data) < self.input_thresh and
                    abs(msg.z_axis.data) < self.input_thresh):

                exec_command = True
                #print("Planning command")
                pos, rot = self.robot.ee_pose()
                rot_euler = euler_from_quaternion(
                    [rot.w, rot.x, rot.y, rot.z], axes='sxyz')

                # Record state-action pair
                #state = [pos[0], pos[1], pos[2], rot_euler[0], rot_euler[1], rot_euler[2]]
                state = JointState()
                state.name = self.current_joint_state.name[2:]
                state.position = self.current_joint_state.position[2:]
                state.velocity = self.current_joint_state.velocity[2:]
                state.effort = self.current_joint_state.effort[2:]
                
                action_unscaled = np.array([msg.x_axis.data, msg.y_axis.data, msg.z_axis.data])

                action = np.array([msg.x_axis.data * self.scale_linear, msg.y_axis.data *
                                   self.scale_linear, msg.z_axis.data * self.scale_linear])

                if self.is_recording:
                    self.record_state_action_pair(
                        state, action_unscaled, type_of_action="pos")

                # Actually update the robot's desired position
                pos += action

                # STORE POS, VAL
                vel_ee, vel_omg = self.robot.ee_velocity()
                if self.is_recording:
                    self.time_elapsed += 0.05
                    self.demo_data.append([self.time_elapsed, pos[0], pos[1], pos[2], rot_euler[0], rot_euler[1], rot_euler[2], vel_ee[0], vel_ee[1], vel_ee[2], vel_omg[0], vel_omg[1], vel_omg[2]])
                #print(data)
        
        else:
            if not (abs(msg.x_axis.data) < self.input_thresh and abs(msg.y_axis.data) < self.input_thresh and
                    abs(msg.z_axis.data) < self.input_thresh):

                exec_command = True
                pos, rot = self.robot.ee_pose()
                rot_euler = euler_from_quaternion(
                    [rot.w, rot.x, rot.y, rot.z], axes='sxyz')

                # Record state-action pair
                #state = [pos[0], pos[1], pos[2], rot_euler[0], rot_euler[1], rot_euler[2]]
                state = JointState()
                state.name = self.current_joint_state.name[2:]
                state.position = self.current_joint_state.position[2:]
                state.velocity = self.current_joint_state.velocity[2:]
                state.effort = self.current_joint_state.effort[2:]

                action_unscaled = np.array([msg.y_axis.data, msg.x_axis.data, msg.z_axis.data])

                action = np.array([msg.y_axis.data * self.scale_angular, msg.x_axis.data *
                                   self.scale_angular, msg.z_axis.data * self.scale_angular])

                if self.is_recording:
                    self.record_state_action_pair(
                        state, action_unscaled, type_of_action="ori")

                # Actually update the robot's desired position
                rot_euler += action
                new_rot = quaternion_from_euler(
                    rot_euler[0], rot_euler[1], rot_euler[2], axes='sxyz')
                rot = np.quaternion(
                    new_rot[0], new_rot[1], new_rot[2], new_rot[3])
                
                # STORE POS, VAL
                vel_ee, vel_omg = self.robot.ee_velocity()

                if self.is_recording:
                    self.time_elapsed += 0.05
                    self.demo_data.append([self.time_elapsed, pos[0], pos[1], pos[2], rot_euler[0], rot_euler[1], rot_euler[2], vel_ee[0], vel_ee[1], vel_ee[2], vel_omg[0], vel_omg[1], vel_omg[2]])

        # Plan and execute new joint angles
        if exec_command:
            status, j_des = self.robot.inverse_kinematics(pos, rot)
            if status:
                print("Applying joy command")
                # print(pos)unit
                self.robot.exec_position_cmd(j_des)

    # Function to record state-action pair at each timestep
    def record_state_action_pair(self, state, partial_action_vector, type_of_action):
        action_vector = []

        if type_of_action == "pos":
            action_vector = [partial_action_vector[0],
                             partial_action_vector[1], partial_action_vector[2], 0, 0, 0]

        elif type_of_action == "ori":
            action_vector = [0, 0, 0, partial_action_vector[0],
                             partial_action_vector[1], partial_action_vector[2]]

        # Put all info into array
        new_state = []
        for i in range(len(state.name)):
            new_state.append(state.position[i])
            new_state.append(state.velocity[i])
            new_state.append(state.effort[i])

        self.state_action_dictionary['State'].append(new_state)
        self.state_action_dictionary['Action'].append(action_vector)

        #current_state_action_pair = [state, action_vector]
        #print("Current s-a pair: ", current_state_action_pair)
        # self.state_action_pairs.append(current_state_action_pair)
        #print("Length of recording: ", len(self.state_action_pairs))

    # Requests to start or stop recording of state-action pairs
    def respond_to_recording_request(self, req):
        # FOR BEHAVIORAL CLONING RECORDINGS
        print("Received request.")
        if req.start_or_stop_recording.data == "start":
            self.is_recording = True
            print("Started recording")
        elif req.start_or_stop_recording.data == "stop":
            is_recording = False
            print("Stopped recording")

            if len(self.state_action_dictionary['State']) > 0:
                self.save_state_action_pair_to_file()
                #self.state_action_pairs = []
                print("Num. states: ", len(
                    self.state_action_dictionary['State']))
                print("Num. actions: ", len(
                    self.state_action_dictionary['Action']))

                self.state_action_dictionary['State'] = []
                self.state_action_dictionary['Action'] = []

        res = RecordStateActionPairsResponse()
        res.success.data = self.is_recording
        return res
    
    # Record KMP-style demos to file
    def record_kmp(self, req, num_traj=11):
        if len(self.demo_data) > 0:
            #demo_data = np.asarray(self.demo_data)
            all_demos = np.array([])
            self.demo_data = np.asarray(self.demo_data).T

            for i in range(num_traj):
                if i == 0:
                    all_demos = np.hstack((all_demos, self.demo_data)) if all_demos.size else self.demo_data
                    print(np.shape(all_demos))
                    print(all_demos[0])
                    print(all_demos[1])
                    #all_demos.append(self.demo_data)
                else:
                    new_data = self.add_noise(self.demo_data)
                    all_demos = np.hstack((all_demos, new_data)) if all_demos.size else new_data
                    #all_demos.append(new_data)

            #demo_data = np.asarray(all_demos)
            print(np.shape(all_demos))
            demo_array_to_list = all_demos.tolist()
            with open("/home/karthikm/catkin_ws/src/teleop-collab-ros/teleop/src/saved_kmps/" + \
                    req.primitive_name.data + ".json", 'w') as f:
                json.dump(demo_array_to_list, f)
                print("Saved KMP demos to drive.")

            self.demo_data = []
            self.time_elapsed = 0

    def add_noise(self, data, noise=0.01):
        for i in range(len(data)):
            if i > 0:
                #print(data[i])
                for j in range(len(data[i])):
                    #print(data[i][j])
                    data[i][j] += random.uniform(-noise, noise)
        return data

    def save_state_action_pair_to_file(self):
        timestr = time.strftime("%Y%m%d-%H%M%S")
        filename = "demonstration_" + timestr
        # Save filename
        filename = '/home/karthikm/Desktop/' + filename + '.json'
        print(filename)
        with open(filename, 'w') as f:
            json.dump(self.state_action_dictionary, f)
            print("Saved file")


def main():
    rospy.init_node("panda_move_from_joy")
    print("Subscribed to panda commands topic.")

    def clean_shutdown():
        print("\nExiting example.")

    rospy.on_shutdown(clean_shutdown)

    # Create instance of Panda
    panda_robot = PandaRobot()
    rospy.Subscriber("/publish_panda_cmds", MovePandaCommand,
                     panda_robot.execute_command_callback, queue_size=1)

    rospy.Subscriber('/panda_simulator/custom_franka_state_controller/joint_states',
                     JointState, panda_robot.save_current_joint_state)
                     

    rospy.spin()


if __name__ == '__main__':
    main()
