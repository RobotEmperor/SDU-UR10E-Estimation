/*
 * ur10e_force_torque_sensor_test.cpp
 *
 *  Created on: Dec 29, 2019
 *      Author: yik
 */
#include "ur10e_force_torque_sensor_test/ur10e_force_torque_sensor_test.h"
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>

using namespace ur_rtde;
volatile sig_atomic_t stop;
double time_count = 0.0;

ofstream out("test.csv");

string data_line;




void timer_handler (int signum)
{
//  static int count = 0;
//  printf("timer expired %d timers\n", ++count);
  time_count += 0.002;
  printf("time_count = %f \n",time_count);

  data_line = to_string(time_count);

  out<<data_line<<endl;
}


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
    return; // some data are missed
  }
  else
  {
    raw_force_torque_data(0,0) = msg->data[0];
    raw_force_torque_data(1,0) = msg->data[1];
    raw_force_torque_data(2,0) = msg->data[2];
    raw_force_torque_data(3,0) = msg->data[3];
    raw_force_torque_data(4,0) = msg->data[4];
    raw_force_torque_data(5,0) = msg->data[5];

    joint_vector[0] = msg->data[18];
    joint_vector[1] = msg->data[19];
    joint_vector[2] = msg->data[20];
    joint_vector[3] = msg->data[21];
    joint_vector[4] = msg->data[22];
    joint_vector[5] = msg->data[23];

    raw_tool_acc_data(0,0) = msg->data[15];
    raw_tool_acc_data(1,0) = msg->data[16];
    raw_tool_acc_data(2,0) = msg->data[17];

    tcp_pose(0,0) = msg->data[24];
    tcp_pose(1,0) = msg->data[25];
    tcp_pose(2,0) = msg->data[26];
    tcp_pose(3,0) = msg->data[27];
    tcp_pose(4,0) = msg->data[28];
    tcp_pose(5,0) = msg->data[29];
  }
}

void ZeroCommandMsgCallBack(const std_msgs::Bool::ConstPtr& msg)
{
  zero_command = true;
}

int main (int argc, char **argv)
{
  printf("Force Torque Sensor Test Node Start \n");
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  filtered_force_torque_data_pub = n.advertise<std_msgs::Float64MultiArray>("/sdu/chatter", 10);

  // ros subsrcibe
  ros::Subscriber raw_force_torque_data_sub;
  ros::Subscriber zero_command_sub;

  //raw_force_torque_data_sub = n.subscribe("/sdu/ur10e/raw_force_torque_data", 10, RawForceTorqueDataMsgCallBack);
  raw_force_torque_data_sub = n.subscribe("/chatter", 10, RawForceTorqueDataMsgCallBack);
  zero_command_sub          = n.subscribe("/sdu/zero", 10, ZeroCommandMsgCallBack);

  control_time = 0.002;
  raw_force_torque_data.resize(6,1);
  raw_force_torque_data.fill(0);

  raw_tool_acc_data.resize(4,1);
  raw_tool_acc_data.fill(0);
  raw_tool_acc_data(3,0)=1;

  tool_acc_data.resize(4,1);
  tool_acc_data.fill(0);
  tool_acc_data(3,0) = 1;

  tcp_pose.resize(6,1);
  tcp_pose.fill(0);

  std::string init_data_path;
  init_data_path = "../config/init_data.yaml";

  ft_filter = std::make_shared<FTfilter>();
  tool_estimation = std::make_shared<ToolEstimation>();
  ur10e_kinematics = std::make_shared<Kinematics>();


  ft_filter->initialize(init_data_path);
  tool_estimation->initialize();
  tool_estimation->set_parameters(control_time, 4.118);
  zero_command = false;

  joint_vector.resize(6);

  //  RTDEReceiveInterface rtde_receive("192.168.1.129");
  //  std::vector<double> joint_positions        = rtde_receive.getActualQ();
  //  std::vector<double> force_data             = rtde_receive.getActualTCPForce();
  //  std::vector<double> tool_linear_acc_data   = rtde_receive.getActualToolAccelerometer();
  //  std::vector<double> tcp_pose_data         = rtde_receive.getActualTCPPose();
  //  std::vector<double> tcp_speed_data         = rtde_receive.getActualTCPSpeed();
  //
  //
  //  tool_linear_acc_data  = rtde_receive.getActualToolAccelerometer();
  //  tcp_pose_data = rtde_receive.getActualTCPPose();
  //  force_data     = rtde_receive.getActualTCPForce();


//  usleep(3000000); //
//
//  // offset initialize function
//  for(int num = 1;num < 1001; num ++)
//  {
//    ros::spinOnce();
//    //force_data     = rtde_receive.getActualTCPForce();
//    //tool_linear_acc_data  = rtde_receive.getActualToolAccelerometer();
//    for(int var = 0; var < 6; var ++)
//    {
//      //raw_force_torque_data(var,0) = force_data[var];
//    }
//    //raw_tool_acc_data(0, 0) = tool_linear_acc_data[0];
//    //raw_tool_acc_data(1, 0) = tool_linear_acc_data[1];
//    //raw_tool_acc_data(2, 0) = tool_linear_acc_data[2];
//
//    ft_filter -> offset_init(raw_force_torque_data, 1000);
//    tool_estimation -> offset_init(raw_tool_acc_data, 1000);
//    usleep(2000); // 2ms
//  }
  //cout <<  tool_estimation->offset_data << "\n\n";

  cout <<"\n--------------------------\n";

  cout <<  ft_filter->get_offset_data() << "\n\n";

  cout <<"\n--------------------------\n";

  cout <<  tool_estimation->get_offset_data() << "\n\n";


  signal(SIGINT, inthand);

  struct sigaction sa;
  struct itimerval timer;

  /* Install timer_handler as the signal handler for SIGVTALRM. */
  memset (&sa, 0, sizeof (sa));
  sa.sa_handler = &timer_handler;
  sigaction (SIGVTALRM, &sa, NULL);

  /* Configure the timer to expire after 250 msec... */
  timer.it_value.tv_sec = 0;
  timer.it_value.tv_usec = 2000;

  /* ... and every 250 msec after that. */
  timer.it_interval.tv_sec = 0;
  timer.it_interval.tv_usec = 2000;

  /* Start a virtual timer. It counts down whenever this process is executing. */
  setitimer (ITIMER_VIRTUAL, &timer, NULL);

//  string data_line;
//  string getActualQ;
//  string getActualTCPPose;
//  string getActualTCPForce;
//  string getActualToolAccelerometer;
//  string getFilteredForce;
//  string getContactedForceTorque;
//  getActualTCPPose = ",actual_tcp_pose_x,actual_tcp_pose_y,actual_tcp_pose_z,actual_tcp_pose_r,actual_tcp_pose_p,actual_tcp_pose_y";
//  getActualTCPForce = ",actual_tcp_force_x,actual_tcp_force_y,actual_tcp_force_z,actual_tcp_force_r,actual_tcp_force_p,actual_tcp_force_y";
//  getActualToolAccelerometer = ",actual_tcp_acc_x,actual_tcp_acc_y,actual_tcp_acc_z";
//  getFilteredForce = ",filtered_force_x,filtered_force_y,filtered_force_z,filtered_torque_r,filtered_torque_p,filtered_torque_y";
//  getContactedForceTorque = ",contacted_force_x,contacted_force_y,contacted_force_z,contacted_torque_r,contacted_torque_p,contacted_torque_y";
//
//  ofstream out("test.csv");
//
//  data_line = "time"+getActualTCPPose+getActualTCPForce+getActualToolAccelerometer+getFilteredForce+getContactedForceTorque;
//
//  out<<data_line<<endl;
//
//  getActualTCPPose = "";
//  getActualTCPForce = "";
//  getFilteredForce = "";
//  getContactedForceTorque = "";
//  getActualToolAccelerometer = "";

  while (!stop)
  {

    //usleep(100); // 2ms



    //tool_linear_acc_data  = rtde_receive.getActualToolAccelerometer();
    //force_data     = rtde_receive.getActualTCPForce();
    //joint_positions        = rtde_receive.getActualQ();
    //tcp_pose_data = rtde_receive.getActualTCPPose();

    if(zero_command == true)
    {
      // offset initialize function
      for(int num = 1;num < 1001; num ++)
      {
        //ros::spinOnce();
        //force_data     = rtde_receive.getActualTCPForce();
        for(int var = 0; var < 6; var ++)
        {
          //raw_force_torque_data(var,0) = force_data[var];
        }
        ft_filter -> offset_init(raw_force_torque_data, 1000);

        usleep(2000); // 2ms
      }
      cout <<"\n--------------------------\n";

      cout <<  ft_filter ->get_offset_data() << "\n\n";

      zero_command = false;
    }

    for(int var = 0; var < 6; var ++)
    {
      //raw_force_torque_data(var,0) = force_data[var];
    }
    //raw_tool_acc_data(0, 0) = tool_linear_acc_data[0];
    //raw_tool_acc_data(1, 0) = tool_linear_acc_data[1];
    //raw_tool_acc_data(2, 0) = tool_linear_acc_data[2];


    //    for(int num = 0;num < 6; num ++)
    //    {
    //      raw_force_torque_data(num,0) = force_data[num];
    //    }
//    tool_acc_data = raw_tool_acc_data - tool_estimation->get_offset_data();
//    ur10e_kinematics->calculate_forward_kinematics(joint_vector);
//    //ur10e_kinematics->calculate_forward_kinematics(joint_positions);
//    tool_acc_data = ur10e_kinematics->get_tf_base_to_tool(tool_acc_data);
//    tool_estimation->set_acc_input_data(tool_acc_data);
//    tool_estimation->set_pose_input_data(tcp_pose);
//    tool_estimation->get_angular_acc();
//
//
//
//    //tool_estimation->get_angular_acc();
//
//    //contacted_force_data = tool_estimation->get_contacted_force(raw_force_torque_data - ft_filter->get_offset_data(), tool_acc_data);
//
//    ft_filter->filter_processing(raw_force_torque_data);
//
//    tool_estimation->get_one_axis_inertia_tensor(ft_filter->get_filtered_data(),"x");
//    tool_estimation->get_one_axis_inertia_tensor(ft_filter->get_filtered_data(),"y");
//    tool_estimation->get_one_axis_inertia_tensor(ft_filter->get_filtered_data(),"z");
//
//    contacted_force_data = tool_estimation->get_contacted_force(ft_filter->get_filtered_data(), tool_acc_data);

    //cout << ur10e_kinematics->get_axis_to_euler_angle(tcp_pose_data[3], tcp_pose_data[4], tcp_pose_data[5]) << "\n\n";
    //cout << "--------------------------------" << "\n\n";

    //cout << tool_estimation ->get_one_axis_inertia_tensor(ft_filter->get_filtered_data(), "x");


    //cout << tool_estimation->inertia_of_tool_ << "\n\n";
    //
    //    filtered_force_torque_data_msg.data.push_back(raw_force_torque_data(0,0));
    //    filtered_force_torque_data_msg.data.push_back(raw_force_torque_data(1,0));
    //    filtered_force_torque_data_msg.data.push_back(raw_force_torque_data(2,0));
    //    filtered_force_torque_data_msg.data.push_back(raw_force_torque_data(3,0));
    //    filtered_force_torque_data_msg.data.push_back(raw_force_torque_data(4,0));
    //    filtered_force_torque_data_msg.data.push_back(raw_force_torque_data(5,0));
    //
    //
    //    filtered_force_torque_data_msg.data.push_back(ft_filter->get_filtered_data()(0,0));
    //    filtered_force_torque_data_msg.data.push_back(ft_filter->get_filtered_data()(1,0));
    //    filtered_force_torque_data_msg.data.push_back(ft_filter->get_filtered_data()(2,0));
    //    filtered_force_torque_data_msg.data.push_back(ft_filter->get_filtered_data()(3,0));
    //    filtered_force_torque_data_msg.data.push_back(ft_filter->get_filtered_data()(4,0));
    //    filtered_force_torque_data_msg.data.push_back(ft_filter->get_filtered_data()(5,0));
    //
    //    filtered_force_torque_data_msg.data.push_back(contacted_force_data(0,0));
    //    filtered_force_torque_data_msg.data.push_back(contacted_force_data(1,0));
    //    filtered_force_torque_data_msg.data.push_back(contacted_force_data(2,0));
    //
    //    filtered_force_torque_data_msg.data.push_back(contacted_force_data(3,0));
    //    filtered_force_torque_data_msg.data.push_back(contacted_force_data(4,0));
    //    filtered_force_torque_data_msg.data.push_back(contacted_force_data(5,0));


    //    filtered_force_torque_data_msg.data.push_back(contacted_force_data(0,0));
    //    filtered_force_torque_data_msg.data.push_back(contacted_force_data(1,0));
    //    filtered_force_torque_data_msg.data.push_back(contacted_force_data(2,0));

    //filtered_force_torque_data_msg.data.push_back(tool_linear_acc_data[0]);
    //filtered_force_torque_data_msg.data.push_back(tool_linear_acc_data[1]);
    //filtered_force_torque_data_msg.data.push_back(tool_linear_acc_data[2]);

    //filtered_force_torque_data_msg.data.push_back(joint_positions[0]);
    //filtered_force_torque_data_msg.data.push_back(joint_positions[1]);
    //filtered_force_torque_data_msg.data.push_back(joint_positions[2]);
    //filtered_force_torque_data_msg.data.push_back(joint_positions[3]);
    //filtered_force_torque_data_msg.data.push_back(joint_positions[4]);
    //filtered_force_torque_data_msg.data.push_back(joint_positions[5]);
    //
    //filtered_force_torque_data_msg.data.push_back(tcp_pose_data[0]);
    //filtered_force_torque_data_msg.data.push_back(tcp_pose_data[1]);
    //filtered_force_torque_data_msg.data.push_back(tcp_pose_data[2]);
    //filtered_force_torque_data_msg.data.push_back(tcp_pose_data[3]);
    //filtered_force_torque_data_msg.data.push_back(tcp_pose_data[4]);
    //filtered_force_torque_data_msg.data.push_back(tcp_pose_data[5]);

    // filtered_force_torque_data_msg.data.push_back(ur10e_kinematics->get_axis_to_euler_angle(tcp_pose_data[3], tcp_pose_data[4], tcp_pose_data[5])(0,0));
    // filtered_force_torque_data_msg.data.push_back(ur10e_kinematics->get_axis_to_euler_angle(tcp_pose_data[3], tcp_pose_data[4], tcp_pose_data[5])(1,0));
    // filtered_force_torque_data_msg.data.push_back(ur10e_kinematics->get_axis_to_euler_angle(tcp_pose_data[3], tcp_pose_data[4], tcp_pose_data[5])(2,0));

    // position will be added
    // transformation check


    // filtered_force_torque_data_pub.publish(filtered_force_torque_data_msg);
    // filtered_force_torque_data_msg.data.clear();

    //    for(int num = 0; num<6; num++)
    //    {
    //      getActualTCPPose += ","+to_string(tcp_pose(num,0));
    //      getActualTCPForce += ","+to_string(raw_force_torque_data(num,0));
    //      getFilteredForce += ","+to_string(ft_filter->get_filtered_data()(num,0));
    //      getContactedForceTorque += ","+to_string(contacted_force_data(num,0));
    //    }
    //    for(int num = 0; num<3; num++)
    //    {
    //      getActualToolAccelerometer += ","+to_string(tool_acc_data(num,0));
    //    }




    //ros::spinOnce();
  }



  usleep(10000);
  out.close();
  cout << "complete and save" << "\n\n";
  printf("exiting safely\n");
  system("pause");

  return 0;

}
