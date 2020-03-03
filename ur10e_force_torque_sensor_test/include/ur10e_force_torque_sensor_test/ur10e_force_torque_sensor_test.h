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
#include "std_msgs/Bool.h"

#include "sdu_sensor/ft_filter.h"
#include "sdu_sensor/tool_estimation.h"

#define CLOCK_RES 1e-9 //Clock resolution is 1 ns by default
#define LOOP_PERIOD 2e6 //Expressed in ticks // 2ms control time
//RTIME period = 1000000000;
RT_TASK loop_task;

using namespace std;
using namespace ur_rtde;

void initialize();

std::shared_ptr<FTfilter> ft_filter;
std::shared_ptr<ToolEstimation> tool_estimation;
std::shared_ptr<Kinematics> ur10e_kinematics;


Eigen::MatrixXd raw_force_torque_data;
Eigen::MatrixXd raw_tool_acc_data;
Eigen::MatrixXd tool_acc_data;
Eigen::MatrixXd contacted_force_data;
Eigen::MatrixXd tcp_pose;
Eigen::MatrixXd tcp_speed;

double control_time;
double sampling_time;
double time_count;

//real time data save
string data_line;
string getActualQ;
string getActualTCPPose;
string getActualTCPForce;
string getActualToolAccelerometer;
string getFilteredForce;
string getContactedForceTorque;
string getActualToolSpeed;
string getActualToolAcc;

//ros
ros::Publisher filtered_force_torque_data_pub;
std_msgs::Float64MultiArray filtered_force_torque_data_msg;

//void RawForceTorqueDataMsgCallBack(const std_msgs::Float64MultiArray::ConstPtr& msg);
//void ZeroCommandMsgCallBack(const std_msgs::Bool::ConstPtr& msg);

//for ros test
std::vector<double> joint_vector;
bool zero_command;

#endif /* UR10E_FORCE_TORQUE_SENSOR_TEST_H_ */
