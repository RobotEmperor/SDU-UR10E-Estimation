/*
 * ur10e_force_torque_sensor_test.cpp
 *
 *  Created on: Dec 29, 2019
 *      Author: yik
 */
#include "ur10e_force_torque_sensor_test/ur10e_force_torque_sensor_test.h"

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

    joint_vector[0] = msg->data[12];
    joint_vector[1] = msg->data[13];
    joint_vector[2] = msg->data[14];
    joint_vector[3] = msg->data[15];
    joint_vector[4] = msg->data[16];
    joint_vector[5] = msg->data[17];

    raw_tool_acc_data(0,0) = msg->data[18];
    raw_tool_acc_data(1,0) = msg->data[19];
    raw_tool_acc_data(2,0) = msg->data[20];
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
  filtered_force_torque_data_pub = n.advertise<std_msgs::Float64MultiArray>("/chatter", 10);

  // ros subsrcibe
  ros::Subscriber raw_force_torque_data_sub;
  ros::Subscriber zero_command_sub;

  //raw_force_torque_data_sub = n.subscribe("/sdu/ur10e/raw_force_torque_data", 10, RawForceTorqueDataMsgCallBack);
  raw_force_torque_data_sub = n.subscribe("/sdu/chatter", 10, RawForceTorqueDataMsgCallBack);
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

  RTDEReceiveInterface rtde_receive("192.168.1.129");
  std::vector<double> joint_positions        = rtde_receive.getActualQ();
  std::vector<double> force_data             = rtde_receive.getActualTCPForce();
  std::vector<double> tool_linear_acc_data   = rtde_receive.getActualToolAccelerometer();
  std::vector<double> tcp_pose_data         = rtde_receive.getActualTCPPose();
  std::vector<double> tcp_speed_data         = rtde_receive.getActualTCPSpeed();
  std::vector<double> torque_data         = rtde_receive.getTargetMoment();

  tool_linear_acc_data  = rtde_receive.getActualToolAccelerometer();
  tcp_pose_data = rtde_receive.getActualTCPPose();
  force_data     = rtde_receive.getActualTCPForce();


  usleep(3000000); //

  // offset initialize function
  for(int num = 1;num < 1001; num ++)
  {
    ros::spinOnce();
    force_data     = rtde_receive.getActualTCPForce();
    tool_linear_acc_data  = rtde_receive.getActualToolAccelerometer();
    for(int var = 0; var < 6; var ++)
    {
      raw_force_torque_data(var,0) = force_data[var];
    }
    raw_tool_acc_data(0, 0) = tool_linear_acc_data[0];
    raw_tool_acc_data(1, 0) = tool_linear_acc_data[1];
    raw_tool_acc_data(2, 0) = tool_linear_acc_data[2];

    ft_filter -> offset_init(raw_force_torque_data, 1000);
    tool_estimation -> offset_init(raw_tool_acc_data, 1000);
    usleep(2000); // 2ms
  }
  //cout <<  tool_estimation->offset_data << "\n\n";

  cout <<"\n--------------------------\n";

  cout <<  ft_filter->get_offset_data() << "\n\n";

  cout <<"\n--------------------------\n";

  cout <<  tool_estimation->get_offset_data() << "\n\n";


  signal(SIGINT, inthand);

  while (!stop)
  {
    usleep(2000); // 2ms
        tool_linear_acc_data  = rtde_receive.getActualToolAccelerometer();
        force_data     = rtde_receive.getActualTCPForce();
        joint_positions        = rtde_receive.getActualQ();

    if(zero_command == true)
    {
      // offset initialize function
      for(int num = 1;num < 1001; num ++)
      {
        //ros::spinOnce();
        //force_data     = rtde_receive.getActualTCPForce();
        for(int var = 0; var < 6; var ++)
        {
          raw_force_torque_data(var,0) = force_data[var];
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
      raw_force_torque_data(var,0) = force_data[var];
    }
    raw_tool_acc_data(0, 0) = tool_linear_acc_data[0];
    raw_tool_acc_data(1, 0) = tool_linear_acc_data[1];
    raw_tool_acc_data(2, 0) = tool_linear_acc_data[2];


    //    for(int num = 0;num < 6; num ++)
    //    {
    //      raw_force_torque_data(num,0) = force_data[num];
    //    }
    tool_acc_data = raw_tool_acc_data - tool_estimation->get_offset_data();
    //ur10e_kinematics->calculate_forward_kinematics(joint_vector);
    ur10e_kinematics->calculate_forward_kinematics(joint_positions);
    tool_acc_data = ur10e_kinematics->get_tf_base_to_tool(tool_acc_data);

    contacted_force_data = tool_estimation->get_contacted_force(raw_force_torque_data - ft_filter->get_offset_data(), tool_acc_data);

    ft_filter->filter_processing(raw_force_torque_data);

    filtered_force_torque_data_msg.data.push_back(raw_force_torque_data(0,0));
    filtered_force_torque_data_msg.data.push_back(raw_force_torque_data(1,0));
    filtered_force_torque_data_msg.data.push_back(raw_force_torque_data(2,0));
    filtered_force_torque_data_msg.data.push_back(raw_force_torque_data(3,0));
    filtered_force_torque_data_msg.data.push_back(raw_force_torque_data(4,0));
    filtered_force_torque_data_msg.data.push_back(raw_force_torque_data(5,0));


    filtered_force_torque_data_msg.data.push_back(ft_filter->get_filtered_data()(0,0));
    filtered_force_torque_data_msg.data.push_back(ft_filter->get_filtered_data()(1,0));
    filtered_force_torque_data_msg.data.push_back(ft_filter->get_filtered_data()(2,0));
    filtered_force_torque_data_msg.data.push_back(ft_filter->get_filtered_data()(3,0));
    filtered_force_torque_data_msg.data.push_back(ft_filter->get_filtered_data()(4,0));
    filtered_force_torque_data_msg.data.push_back(ft_filter->get_filtered_data()(5,0));


    filtered_force_torque_data_msg.data.push_back(contacted_force_data(0,0));
    filtered_force_torque_data_msg.data.push_back(contacted_force_data(1,0));
    filtered_force_torque_data_msg.data.push_back(contacted_force_data(2,0));

    filtered_force_torque_data_msg.data.push_back(tool_linear_acc_data[0]);
    filtered_force_torque_data_msg.data.push_back(tool_linear_acc_data[1]);
    filtered_force_torque_data_msg.data.push_back(tool_linear_acc_data[2]);

    filtered_force_torque_data_msg.data.push_back(joint_positions[0]);
    filtered_force_torque_data_msg.data.push_back(joint_positions[1]);
    filtered_force_torque_data_msg.data.push_back(joint_positions[2]);
    filtered_force_torque_data_msg.data.push_back(joint_positions[3]);
    filtered_force_torque_data_msg.data.push_back(joint_positions[4]);
    filtered_force_torque_data_msg.data.push_back(joint_positions[5]);

    // position will be added
    // transformation check


    filtered_force_torque_data_pub.publish(filtered_force_torque_data_msg);
    filtered_force_torque_data_msg.data.clear();

    ros::spinOnce();
  }

  usleep(10000);
  cout << "complete" << "\n\n";

  printf("exiting safely\n");
  system("pause");

  return 0;

}
