/*******************************************************************************
 *      Title     : cpp_interface_example.cpp
 *      Project   : moveit_servo
 *      Created   : 11/20/2019
 *      Author    : Andy Zelenak
 *
 * BSD 3-Clause License
 *
 * Copyright (c) 2019, Los Alamos National Security, LLC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of the copyright holder nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *******************************************************************************/

#include <std_msgs/Int8.h>
#include <geometry_msgs/TwistStamped.h>

#include <moveit_servo/servo.h>
#include <moveit_servo/status_codes.h>
#include <moveit_servo/pose_tracking.h>
#include <moveit_servo/make_shared_from_pool.h>
#include <thread>
#include <future>
using namespace std::chrono_literals;

static const std::string LOGNAME = "servo_server";

// Class for monitoring status of moveit_servo
class StatusMonitor
{
public:
  StatusMonitor(ros::NodeHandle& nh, const std::string& topic)
  {
    sub_ = nh.subscribe(topic, 1, &StatusMonitor::statusCB, this);
  }

private:
  void statusCB(const std_msgs::Int8ConstPtr& msg)
  {
    auto latest_status = static_cast<moveit_servo::StatusCode>(msg->data);
    if (latest_status != status_)
    {
      status_ = latest_status;
      const auto& status_str = moveit_servo::SERVO_STATUS_CODE_MAP.at(status_);
      ROS_INFO_STREAM_NAMED(LOGNAME, "Servo status: " << status_str);
    }
  }
  moveit_servo::StatusCode status_ = moveit_servo::StatusCode::INVALID;
  ros::Subscriber sub_;
};


class PandaCameraPoseTracking
{
    public:
        PandaCameraPoseTracking(): lin_tol(0.01, 0.01, 0.01),
                                   rot_tol(0.1),
                                   moveRobotThread(nullptr),
                                   moveRobotDone(true)
        {
            // Give node handle the correct name to retrieve parameters
            nh = LOGNAME;

            // Load the planning scene monitor
            planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor;
            planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>("robot_description");
            if (!planning_scene_monitor->getPlanningScene())
            {
                ROS_ERROR_STREAM_NAMED(LOGNAME, "Error in setting up the PlanningSceneMonitor.");
                exit(EXIT_FAILURE);
            }

            planning_scene_monitor->startSceneMonitor();
            planning_scene_monitor->startWorldGeometryMonitor(
                planning_scene_monitor::PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC,
                planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC,
                false /* skip octomap monitor */);
            planning_scene_monitor->startStateMonitor();

            // Run the servo node C++ interface in a new timer to ensure a constant outgoing message rate.
            //moveit_servo::Servo servo(nh, planning_scene_monitor);
            //servo.start();

            // Initialize tracker
            tracker = std::make_shared<moveit_servo::PoseTracking>(nh, planning_scene_monitor);

            // Subscribe to servo status (and log it when it changes)
            //StatusMonitor status_monitor(nh, servo.getParameters().status_topic);
            StatusMonitor status_monitor(nh, "servo_status");

            //joint_servo_pub = nh.advertise<control_msgs::JointJog>(servo.getParameters().joint_command_in_topic, 1);
            // twist_stamped_pub = nh.advertise<geometry_msgs::TwistStamped>("/servo_server/delta_twist_cmds", 1);
            // ros::NodeHandle nh_two;
            // twist_stamped_sub = nh_two.subscribe("servoing/twist_stamped_publisher",
            //                                      1, &sub_and_pub::send_twist_to_servo, this);
        
            /// Subscribe to the same topic, just to start a new moveToPose thread when needed
            ros::NodeHandle nh_two;
            pose_stamped_sub = nh_two.subscribe("target_pose",
                                                 1, &PandaCameraPoseTracking::start_moving_robot, this);
            ns_pose_stamped_pub = nh_two.advertise<geometry_msgs::PoseStamped>("/servo_server/target_pose", 1);

            ros::AsyncSpinner spinner(8);
            spinner.start();
            ros::waitForShutdown();
        }

        ~PandaCameraPoseTracking() {
            /// Wait for the robot control thread to finish
            tracker->stopMotion();
            if(moveRobotThread) {
              moveRobotThread->join();
              delete moveRobotThread;
              ROS_INFO_STREAM_NAMED(LOGNAME, "Stopping move robot thread.");

            }
        }

        // Publish servo command to servo's default input twist topic
        void start_moving_robot(const geometry_msgs::PoseStamped::ConstPtr& msg)
        {
            //ROS_INFO_STREAM(msg->twist);
            // auto rot_tol_cp = rot_tol;
            // auto lin_tol_cp = lin_tol;
            // auto tracker_cp = tracker;
            //ROS_INFO_STREAM_NAMED(LOGNAME, "Message age: "<< (ros::Time::now() - msg->header.stamp).toSec());
            ns_pose_stamped_pub.publish(msg);

            if(!moveRobotThread) {

              moveRobotThread = new std::thread([this] {
                  this->moveRobotDone = false;
                  auto ret = this->tracker->moveToPose(this->lin_tol, this->rot_tol, 0.1);
                  this->moveRobotDone = true;
              });
            } else if (moveRobotDone)
            {
              // So the previous move is done, start new function call
              moveRobotThread->join();
              delete moveRobotThread;
              moveRobotThread = new std::thread([this] {
                  this->moveRobotDone = false;
                  auto ret = this->tracker->moveToPose(this->lin_tol, this->rot_tol, 0.1);
                  this->moveRobotDone = true;
              });
            }
        }


    
    private:
        ros::NodeHandle nh;
        ros::Publisher ns_pose_stamped_pub;
        ros::Subscriber pose_stamped_sub;
        //ros::Subscriber twist_stamped_sub;
        std::shared_ptr<moveit_servo:: PoseTracking> tracker;
        Eigen::Vector3d lin_tol;
        double rot_tol;
        std::thread* moveRobotThread;
        std::atomic<int> moveRobotDone;
        //std::promise<moveit_servo::PoseTrackingStatusCode> moveRobotRet;
};

/**
 * Instantiate the C++ servo node interface.
 * Allows others to send joy or joint commands.
 */
int main(int argc, char** argv)
{
    ros::init(argc, argv, LOGNAME);

    PandaCameraPoseTracking panda_tracking;

    //ros::spin();
    return 0;
}
