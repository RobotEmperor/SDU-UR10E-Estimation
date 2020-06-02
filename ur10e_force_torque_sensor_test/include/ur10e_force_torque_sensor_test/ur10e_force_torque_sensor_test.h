/*
 * ur10e_force_torque_sensor_test.h
 *
 *  Created on: Dec 29, 2019
 *      Author: yik
 */

#ifndef UR10E_FORCE_TORQUE_SENSOR_TEST_H_
#define UR10E_FORCE_TORQUE_SENSOR_TEST_H_

#include <Eigen/Dense>

// file load
#include <fstream>
#include <string>
#include <iostream>
#include <sstream>
#include <stdio.h>

//ur_rtde library
#include <ur_rtde/rtde_receive_interface.h>
#include <ur_rtde/rtde_control_interface.h>

//rw_robwork
#include <rw/invkin/ClosedFormIKSolverUR.hpp>
#include <rw/invkin/JacobianIKSolver.hpp>
#include <rw/kinematics.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rw/math.hpp>
#include <rw/models/SerialDevice.hpp>
#include <rw/models/WorkCell.hpp>
#include <rw/pathplanning/PlannerConstraint.hpp>
#include <rw/pathplanning/QSampler.hpp>
#include <rw/proximity/CollisionDetector.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

//xenomai rt system
#include <unistd.h>
#include <signal.h>
#include <cstdlib>
#include <sys/mman.h>
#include <sys/types.h>
#include <alchemy/task.h>
#include <alchemy/timer.h>

//ros message system
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"

#include "sdu_sensor/ft_filter.h"
#include "sdu_sensor/tool_estimation.h"
#include "sdu_math/end_point_to_rad_cal.h"
#include "ur10e_force_torque_sensor_test/task_motion.h"

#define CLOCK_RES 1e-9 //Clock resolution is 1 us by default 1e-9
#define LOOP_PERIOD 2e6 //Expressed in ticks // 2ms control time
//RTIME period = 1000000000;
RT_TASK loop_task;

using namespace std;
using namespace ur_rtde;
using namespace rw::math;
using namespace rw::models;
using rw::invkin::ClosedFormIKSolverUR;
using rw::kinematics::State;
using rw::loaders::WorkCellLoader;

void initialize();

bool gazebo_check;

//std::shared_ptr<FTfilter> ft_filter;
std::shared_ptr<ToolEstimation> tool_estimation;
std::shared_ptr<Kinematics> ur10e_kinematics;
std::shared_ptr<CalRad> ur10e_traj;
std::shared_ptr<TaskMotion> ur10e_task;


std::shared_ptr<RTDEReceiveInterface> rtde_receive;
std::shared_ptr<RTDEControlInterface> rtde_control;

std::vector<double> joint_positions;
std::vector<double> force_data;
std::vector<double> tool_linear_acc_data;
std::vector<double> tcp_pose_data;
std::vector<double> tcp_target_pose_data;
std::vector<double> tcp_speed_data;

Eigen::MatrixXd raw_force_torque_data;
Eigen::MatrixXd raw_tool_acc_data;
Eigen::MatrixXd tool_acc_data;
Eigen::MatrixXd contacted_force_data;
Eigen::MatrixXd tcp_pose;
Eigen::MatrixXd tcp_target_pose;
Eigen::MatrixXd tcp_speed;
Eigen::MatrixXd desired_pose_matrix;
Eigen::MatrixXd gravity;

double control_time;
double sampling_time;
double time_count;
double motion_time;

//real time data save
string data_line;
string getActualQ;
string getActualTCPPose;
string getTargetTCPPose;
string getActualTCPForce;
string getActualToolAccelerometer;
string getFilteredForce;
string getContactedForceTorque;
string getActualToolSpeed;
string getActualToolAcc;

//ros
ros::Publisher filtered_force_torque_data_pub;
ros::Publisher raw_force_torque_data_pub;

ros::Publisher joint_cur_value_pub;
ros::Publisher ee_cur_value_pub;

ros::Publisher gazebo_shoulder_pan_position_pub;
ros::Publisher gazebo_shoulder_lift_position_pub;
ros::Publisher gazebo_elbow_position_pub;
ros::Publisher gazebo_wrist_1_position_pub;
ros::Publisher gazebo_wrist_2_position_pub;
ros::Publisher gazebo_wrist_3_position_pub;

std_msgs::Float64MultiArray raw_force_torque_data_msg;
std_msgs::Float64MultiArray filtered_force_torque_data_msg;
std_msgs::Float64MultiArray joint_cur_value_msg;
std_msgs::Float64MultiArray ee_cur_value_msg;

std_msgs::Float64 gazebo_shoulder_pan_position_msg;
std_msgs::Float64 gazebo_shoulder_lift_position_msg;
std_msgs::Float64 gazebo_elbow_position_msg;
std_msgs::Float64 gazebo_wrist_1_position_msg;
std_msgs::Float64 gazebo_wrist_2_position_msg;
std_msgs::Float64 gazebo_wrist_3_position_msg;

//void RawForceTorqueDataMsgCallBack(const std_msgs::Float64MultiArray::ConstPtr& msg);
//void ZeroCommandMsgCallBack(const std_msgs::Bool::ConstPtr& msg);

void CommandDataMsgCallBack (const std_msgs::Float64MultiArray::ConstPtr& msg);
void TaskCommandDataMsgCallBack (const std_msgs::String::ConstPtr& msg);

//for ros test
std::vector<double> joint_vector;
std::vector<double> desired_pose_vector;
bool zero_command;
std::string task_command;

//tf
rw::math::Transform3D<> tf_desired;
rw::math::Transform3D<> tf_current;

Eigen::MatrixXd tf_current_matrix;


//q solution
std::vector<rw::math::Q> solutions;

#endif /* UR10E_FORCE_TORQUE_SENSOR_TEST_H_ */
