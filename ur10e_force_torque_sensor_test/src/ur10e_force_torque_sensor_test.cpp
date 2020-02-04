/*
 * ur10e_force_torque_sensor_test.cpp
 *
 *  Created on: Dec 29, 2019
 *      Author: yik
 */

#include "ur10e_force_torque_sensor/ur10e_force_torque_sensor.h"
#include "ur10e_force_torque_sensor_test/ur10e_force_torque_sensor_test.h"

#include <ur_rtde/rtde_receive_interface.h>
#include <unistd.h>
#include <signal.h>
#include <cstdlib>
#include <thread>
#include <time.h>

//timer_t firstTimerID;

using namespace ur_rtde;
volatile sig_atomic_t stop;

void inthand(int signum) {
  stop = 1;
}

void RawForceTorqueDataMsgCallBack(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  if(msg->data.size() < 6)
  {
    raw_force_torque_data(0,0) = 0;
    raw_force_torque_data(1,0) = 0;
    raw_force_torque_data(2,0) = 0;
    raw_force_torque_data(3,0) = 0;
    raw_force_torque_data(4,0) = 0;
    raw_force_torque_data(5,0) = 0;
    return; // some data is missed
  }
  else
  {
    raw_force_torque_data(0,0) = msg->data[0];
    raw_force_torque_data(1,0) = msg->data[1];
    raw_force_torque_data(2,0) = msg->data[2];
    raw_force_torque_data(3,0) = msg->data[3];
    raw_force_torque_data(4,0) = msg->data[4];
    raw_force_torque_data(5,0) = msg->data[5];
  }
}

int main (int argc, char **argv)
{
  raw_force_torque_data.resize(6,1);
  raw_force_torque_data.fill(0);


  printf("Force Torque Sensor Test Node Start \n");
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  filtered_force_torque_data_pub = n.advertise<std_msgs::Float64MultiArray>("chatter", 10);

  // ros subsrcibe
  ros::Subscriber raw_force_torque_data_sub;

  raw_force_torque_data_sub = n.subscribe("/sdu/ur10e/raw_force_torque_data", 10, RawForceTorqueDataMsgCallBack);


  control_time = 0.002;
  sampling_time = 2.0;

  ft_sensor = new Ur10eFTsensor;
  tool_estimation = new PoseEstimation;
  ur10e_kinematics = new Kinematics;


  std::string init_data_path;
  init_data_path = "../config/init_data.yaml";


  ft_sensor->parse_init_data(init_data_path);
  ft_sensor->initialize();

  //tool_estimation ->initialize();

  // offset variables init
  offset_check = true;
  time_count =0.0;

    RTDEReceiveInterface rtde_receive("192.168.1.129");
    std::vector<double> joint_positions        = rtde_receive.getActualQ();
    std::vector<double> force_data             = rtde_receive.getActualTCPForce();
    std::vector<double> tool_linear_acc_data   = rtde_receive.getActualToolAccelerometer();
    std::vector<double> tool_pose_data         = rtde_receive.getActualTCPPose();
    std::vector<double> torque_data         = rtde_receive.getTargetMoment();



    tool_linear_acc_data  = rtde_receive.getActualToolAccelerometer();
    tool_pose_data = rtde_receive.getActualTCPPose();
    force_data     = rtde_receive.getActualTCPForce();

  for(int num = 0;num < 6; num ++)
  {
     //ft_sensor->ft_offset_data(num,0) = force_data[num];
     ft_sensor->ft_offset_data(num,0) = raw_force_torque_data(num,0);
  }

  cout <<  ft_sensor->ft_offset_data << "\n\n";



  // makeTimer(&firstTimerID, 2, 2); //2ms
  signal(SIGINT, inthand);

  while (!stop)
  {
    usleep(2000); // 2ms
        force_data     = rtde_receive.getActualTCPForce();
        torque_data         = rtde_receive.getTargetMoment();
        joint_positions        = rtde_receive.getActualQ();
        ur10e_kinematics->joint_positions = joint_positions;

        for(int num = 0;num < 6; num ++)
        {
          raw_force_torque_data(num,0) = force_data[num];
        }

//        //offset initialize
//        if(time_count < sampling_time)
//        {
//          offset_check = true;
//          ft_sensor->offset_init(raw_force_torque_data, offset_check);
//          //tool_estimation->offset_init(tool_estimation -> tool_linear_acc_data, offset_check);
//        }
//        if(time_count > sampling_time && offset_check == true)
//        {
//          offset_check = false;
//          ft_sensor->offset_init(raw_force_torque_data, offset_check);
//          //tool_estimation->offset_init(tool_estimation -> tool_linear_acc_data, offset_check);
//          //printf("loop\n");
//        }
//        force_data     = rtde_receive.getActualTCPForce();
//
//        for(int num = 0;num < 6; num ++)
//        {
//          raw_force_torque_data(num,0) = force_data[num];
//        }


    ft_sensor->signal_processing(raw_force_torque_data);
    //
    ft_sensor->collision_detection_processing(ft_sensor->ft_filtered_data);

    ur10e_kinematics->calculate_jacobian(ur10e_kinematics->joint_positions);

    cout << (ur10e_kinematics->jacobian_force_matrix.transpose()).determinant() << "\n\n";
    ur10e_kinematics->calculate_end_effector_force(torque_data);
    //
    //

    filtered_force_torque_data_msg.data.push_back(raw_force_torque_data(0,0));
    filtered_force_torque_data_msg.data.push_back(raw_force_torque_data(1,0));
    filtered_force_torque_data_msg.data.push_back(raw_force_torque_data(2,0));
    filtered_force_torque_data_msg.data.push_back(raw_force_torque_data(3,0));
    filtered_force_torque_data_msg.data.push_back(raw_force_torque_data(4,0));
    filtered_force_torque_data_msg.data.push_back(raw_force_torque_data(5,0));


    filtered_force_torque_data_msg.data.push_back(ft_sensor->ft_filtered_data(0,0));
    filtered_force_torque_data_msg.data.push_back(ft_sensor->ft_filtered_data(1,0));
    filtered_force_torque_data_msg.data.push_back(ft_sensor->ft_filtered_data(2,0));
    filtered_force_torque_data_msg.data.push_back(ft_sensor->ft_filtered_data(3,0));
    filtered_force_torque_data_msg.data.push_back(ft_sensor->ft_filtered_data(4,0));
    filtered_force_torque_data_msg.data.push_back(ft_sensor->ft_filtered_data(5,0));

    filtered_force_torque_data_msg.data.push_back(ur10e_kinematics->force_matrix(0,0));
    filtered_force_torque_data_msg.data.push_back(ur10e_kinematics->force_matrix(1,0));
    filtered_force_torque_data_msg.data.push_back(ur10e_kinematics->force_matrix(2,0));
    filtered_force_torque_data_msg.data.push_back(torque_data[0]);
    filtered_force_torque_data_msg.data.push_back(torque_data[1]);
    filtered_force_torque_data_msg.data.push_back(torque_data[2]);

    filtered_force_torque_data_pub.publish(filtered_force_torque_data_msg);
    filtered_force_torque_data_msg.data.clear();





    ros::spinOnce();
  }


  usleep(10000);
  cout << "complete" << "\n\n";

  delete ft_sensor;
  delete tool_estimation;
  delete ur10e_kinematics;

  // timer_delete(&firstTimerID);
  printf("exiting safely\n");
  system("pause");

  return 0;



  //  ur10e_kinematics->temp_data(0,0) = tool_linear_acc_data[0];
  //  ur10e_kinematics->temp_data(1,0) = tool_linear_acc_data[1];
  //  ur10e_kinematics->temp_data(2,0) = tool_linear_acc_data[2];
  //  ur10e_kinematics->calculate_forward_kinematics(6,ur10e_kinematics->joint_positions);
  //  ur10e_kinematics->temp_data = ur10e_kinematics->transformation_result*ur10e_kinematics->temp_data;
  //
  //  tool_estimation -> tool_linear_acc_data(0,0) = ur10e_kinematics->temp_data(0,0);
  //  tool_estimation -> tool_linear_acc_data(1,0) = ur10e_kinematics->temp_data(1,0);
  //  tool_estimation -> tool_linear_acc_data(2,0) = ur10e_kinematics->temp_data(2,0);

  //    // offset initialize
  //    if(time_count < sampling_time)
  //    {
  //      offset_check = true;
  //      ft_sensor->offset_init(raw_force_torque_data, offset_check);
  //      tool_estimation->offset_init(tool_estimation -> tool_linear_acc_data, offset_check);
  //    }
  //    if(time_count > sampling_time && offset_check == true)
  //    {
  //      offset_check = false;
  //      ft_sensor->offset_init(raw_force_torque_data, offset_check);
  //      tool_estimation->offset_init(tool_estimation -> tool_linear_acc_data, offset_check);
  //
  //      cout << tool_estimation->offset_data << "\n\n";
  //      cout << ur10e_kinematics->transformation_result << "\n\n";
  //      cout << tool_pose_data[0] << "\n\n";
  //      cout << tool_pose_data[1] << "\n\n";
  //      cout << tool_pose_data[2] << "\n\n";

}
