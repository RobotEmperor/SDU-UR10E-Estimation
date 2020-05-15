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

//log
#include "ur10e_force_torque_sensor_test/log.h"

using namespace std;

class TaskMotion
{

public:
  TaskMotion();
  ~TaskMotion();
  void initialize(double control_time_);
  void robot_initialize(); // joint space
  void load_task_motion(std::string path_);
  void run_task_motion();
  void clear_task_motion();

  std::vector<double> get_current_pose();



private:

  int number_of_point;

  std::map<int, std::vector<double>> motion_start_time_vector;
  std::map<int, std::vector<double>> motion_task_vector;

  std::vector<double> current_pose_vector;
  Eigen::MatrixXd desired_pose_matrix;
  std::shared_ptr<CalRad> ur10e_traj;


};





////////////////////////////////////////////////////////////////////////////







#endif /* TASK_MOTION_H_ */
