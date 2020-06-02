/*
 * force_controller.h
 *
 *  Created on: May 27, 2020
 *      Author: yik
 */
#ifndef FORCE_CONTROLLER_H_
#define FORCE_CONTROLLER_H_

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

class ForceController
{

public:
  ForceController();
  ~ForceController();
  void initialize(double control_time_);
  void contact_force_controller(Eigen::MatrixXd contact_force_data);

private:
  Eigen::MatrixXd compensated_pose_matrix;
  std::shared_ptr<CalRad> gain_traj;
};



#endif /* FORCE_CONTROLLER_H_ */
