/*
 * task_motion.h
 *
 *  Created on: May 15, 2020
 *      Author: yik
 */

#ifndef TASK_MOTION_H_
#define TASK_MOTION_H_

#include <Eigen/Dense>
// file load
#include <fstream>
#include <string>
#include <iostream>
#include <sstream>
#include <stdio.h>

//yaml
#include <yaml-cpp/yaml.h>

//sdu_math
#include "sdu_math/kinematics.h"
#include "sdu_math/end_point_to_rad_cal.h"
#include <rw/math.hpp>

//log
#include "ur10e_force_torque_sensor_test/log.h"

using namespace rw::math;

class TaskMotion
{

public:
  TaskMotion();
  ~TaskMotion();
  void initialize(double control_time_);
  void robot_initialize(); // joint space
  void trans_tcp_to_base_motion(std::string load_path_);
  void load_task_motion(std::string path_, std::string motion_);

  void run_task_motion();
  void generate_trajectory();

  bool task_status();

  double calculate_velocity(double first_point,double second_point, double interval_time);
  double calculate_next_velocity(double first_vel, double second_vel);
  void calculate_init_final_velocity(int point_number);


  void clear_task_motion();

  void set_point(double x, double y, double z, double roll, double pitch, double yaw, double time);
  void set_initial_pose(double x, double y, double z, double roll, double pitch, double yaw);
  void set_initial_pose_eaa(double x, double y, double z, double axis_x, double axis_y, double axis_z);
  void set_current_pose_eaa(double x, double y, double z, double axis_x, double axis_y, double axis_z);

  void load_data_initialize();
  void load_data_tcp_motion();

  void change_motion(std::string motion_);

  std::vector<double> get_current_pose();
  std::vector<double> get_desired_force_torque();

private:

  int number_of_point;
  int all_point;
  int init_all_point;
  int init_belt_task_all_point;
  int tcp_all_point;

  int current_point;
  bool check_change;
  bool task_done;
  bool base_frame_;

  //
  std::map<int, std::vector<double>> motion_start_time_vector;
  std::map<int, std::vector<double>> motion_task_pose_vector;
  std::map<int, std::vector<double>> motion_task_init_vel_vector;
  std::map<int, std::vector<double>> motion_task_final_vel_vector;

  //
  std::map<int, std::vector<double>> init_motion_start_time_vector;
  std::map<int, std::vector<double>> init_motion_task_pose_vector;
  std::map<int, std::vector<double>> init_motion_task_init_vel_vector;
  std::map<int, std::vector<double>> init_motion_task_final_vel_vector;

  //init belt
  std::map<int, std::vector<double>> init_belt_motion_start_time_vector;
  std::map<int, std::vector<double>> init_belt_motion_task_pose_vector;
  std::map<int, std::vector<double>> init_belt_motion_task_init_vel_vector;
  std::map<int, std::vector<double>> init_belt_motion_task_final_vel_vector;


  //tcp
  std::map<int, std::vector<double>> tcp_motion_start_time_vector;
  std::map<int, std::vector<double>> tcp_motion_task_pose_vector;

  std::map<int, std::vector<double>> tcp_motion_task_init_vel_vector;
  std::map<int, std::vector<double>> tcp_motion_task_final_vel_vector;

  //tcp force
  std::map<int, std::vector<double>> tcp_motion_desired_force_vector;


  std::vector<double> current_pose_vector;
  std::vector<double> current_force_torque_vector;

  std::shared_ptr<CalRad> robot_traj;
  Eigen::MatrixXd desired_pose_matrix;

  Transform3D<> tf_tcp_desired_pose_;
  Transform3D<> tf_initial_pose_;
  Transform3D<> tf_current_pose_;
  Transform3D<> tf_desired_pose_;

  Wrench6D<> tf_force_desired_;
  Wrench6D<> tf_tcp_desired_force_;
};
#endif /* TASK_MOTION_H_ */
