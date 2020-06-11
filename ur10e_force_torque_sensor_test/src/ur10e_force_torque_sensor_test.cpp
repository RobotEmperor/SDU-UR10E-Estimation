/*
 * ur10e_force_torque_sensor_test.cpp
 *
 *  Created on: Dec 29, 2019
 *      Author: yik
 */
#include "ur10e_force_torque_sensor_test/ur10e_force_torque_sensor_test.h"
#include "ur10e_force_torque_sensor_test/log.h"
#define WC_FILE "/home/yik/sdu_ws/SDU-UR10E-Estimation/wc/UR10e_2018/UR10e.xml"

const WorkCell::Ptr wc = WorkCellLoader::Factory::load(WC_FILE);
const SerialDevice::Ptr device = wc->findDevice<SerialDevice>("UR10e");

void loop_task_proc(void *arg)
{
  double previous_t = 0.0;
  int sum_count = 0;
  double sum_delayed_time = 0.0;
  double aver_delayed_time = 0.0;

  if (wc.isNull())
    RW_THROW("WorkCell could not be loaded.");
  if (device.isNull())
    RW_THROW("UR10e device could not be found.");

  State state = wc->getDefaultState();
  ClosedFormIKSolverUR solver(device, state);
  solver.setCheckJointLimits(true);

  RT_TASK *curtask;
  RT_TASK_INFO curtaskinfo;
  RTIME tstart;

  curtask = rt_task_self();
  rt_task_inquire(curtask, &curtaskinfo);

  printf("Starting task %s with period of %f ms ....\n", curtaskinfo.name, control_time*1000);
  //Make the task periodic with a specified loop period
  rt_task_set_periodic(NULL, TM_NOW, LOOP_PERIOD);
  tstart = rt_timer_read();

  if(!gazebo_check)
  {
    std::cout << COLOR_GREEN_BOLD << "Real robot start " << COLOR_RESET << std::endl;
    force_data            = rtde_receive->getActualTCPForce();
    tool_linear_acc_data  = rtde_receive->getActualToolAccelerometer();
    tcp_pose_data         = rtde_receive->getActualTCPPose();

    tool_acc_data(0,0) = -tool_linear_acc_data[0];
    tool_acc_data(1,0) = -tool_linear_acc_data[1];
    tool_acc_data(2,0) = -tool_linear_acc_data[2];

    tf_current = Transform3D<> (Vector3D<>(tcp_pose_data[0], tcp_pose_data[1], tcp_pose_data[2]), EAA<>(tcp_pose_data[3], tcp_pose_data[4], tcp_pose_data[5]).toRotation3D());

    for(int num_row = 0; num_row < 4; num_row ++)
    {
      for(int num_col = 0; num_col < 4; num_col ++)
      {
        tf_current_matrix(num_row,num_col) = tf_current(num_row,num_col);
      }
    }

    tool_estimation->set_orientation_data(tf_current_matrix);
    tool_estimation->set_gravity_input_data(tool_acc_data.block(0,0,3,1));

    cout << tool_acc_data.block(0,0,3,1) << endl;
  }
  else
  {
    std::cout << COLOR_GREEN_BOLD << "Gazebo robot start " << COLOR_RESET << std::endl;
  }
  usleep(3000000);

  while(1){
    //    if((rt_timer_read() - tstart)/1000000.0  - previous_t > (control_time*1000 + 0.15))
    //    {
    //      printf("delayed Loop time: %.5f ms\n",((rt_timer_read() - tstart)/1000000.0  - previous_t) - control_time*1000);
    //    }
    sum_delayed_time += ((rt_timer_read() - tstart)/1000000.0  - previous_t) - control_time*1000;
    if(sum_count%1500 == 0 && sum_count != 0) // every 3 seconds,
    {
      aver_delayed_time = sum_delayed_time/sum_count;
      printf("Average delayed time in 3s: %.5f ms\n",aver_delayed_time);
      sum_delayed_time = 0;
      sum_count = 0;
    }
    previous_t = (rt_timer_read() - tstart)/1000000.0;
    time_count += 0.002;
    sum_count += 1;

    //motion controller algorithm
    ur10e_task->set_current_pose_eaa(compensated_pose_vector[0], compensated_pose_vector[1], compensated_pose_vector[2],compensated_pose_vector[3], compensated_pose_vector[4], compensated_pose_vector[5]);

    ur10e_task->run_task_motion();

    ur10e_task->generate_trajectory();

    //motion reference
    desired_pose_vector = ur10e_task->get_current_pose();
    desired_force_torque_vector = ur10e_task->get_desired_force_torque();

    if(!gazebo_check)
    {
      force_data            = rtde_receive->getActualTCPForce();
      tool_linear_acc_data  = rtde_receive->getActualToolAccelerometer();
      tcp_pose_data         = rtde_receive->getActualTCPPose();
      //joint_positions       = rtde_receive->getActualQ();

      // data types will be changed
      for(int var = 0; var < 6; var ++)
      {
        raw_force_torque_data(var,0) = force_data[var];
        tcp_pose(var,0) = tcp_pose_data[var];
      }
      tool_acc_data(0,0) = -tool_linear_acc_data[0];
      tool_acc_data(1,0) = -tool_linear_acc_data[1];
      tool_acc_data(2,0) = -tool_linear_acc_data[2];

      tf_current = Transform3D<> (Vector3D<>(tcp_pose_data[0], tcp_pose_data[1], tcp_pose_data[2]), EAA<>(tcp_pose_data[3], tcp_pose_data[4], tcp_pose_data[5]).toRotation3D());
      for(int num_row = 0; num_row < 4; num_row ++)
      {
        for(int num_col = 0; num_col < 4; num_col ++)
        {
          tf_current_matrix(num_row,num_col) = tf_current(num_row,num_col);
        }
      }

      // force torque sensor filtering
      tool_estimation->set_orientation_data(tf_current_matrix);
      //tool_acc_data = tf_current_matrix*tool_acc_data;
      contacted_force_data = tool_estimation->get_estimated_force(raw_force_torque_data, tool_acc_data.block(0,0,3,1));

      ////controller for force compensation

      force_x_compensator->set_pid_gain(f_kp,f_ki,f_kd);
      //force_y_compensator->set_pid_gain(f_kp,f_ki,f_kd);
      //force_z_compensator->set_pid_gain(f_kp,f_ki,f_kd);


      force_x_compensator->PID_calculate(ur10e_task->get_desired_force_torque()[0],contacted_force_data(0,0));
      //force_y_compensator->PID_calculate(1,contacted_force_data(1,0));
      //force_z_compensator->PID_calculate(1,contacted_force_data(2,0));

      //ros communication
      test_data_msg.data.push_back(force_x_compensator->get_final_output());
      test_data_msg.data.push_back(compensated_pose_vector[0]);

      raw_force_torque_data_msg.data.push_back(raw_force_torque_data(0,0));
      raw_force_torque_data_msg.data.push_back(raw_force_torque_data(1,0));
      raw_force_torque_data_msg.data.push_back(raw_force_torque_data(2,0));
      raw_force_torque_data_msg.data.push_back(raw_force_torque_data(3,0));
      raw_force_torque_data_msg.data.push_back(raw_force_torque_data(4,0));
      raw_force_torque_data_msg.data.push_back(raw_force_torque_data(5,0));

      filtered_force_torque_data_msg.data.push_back(contacted_force_data(0,0));
      filtered_force_torque_data_msg.data.push_back(contacted_force_data(1,0));
      filtered_force_torque_data_msg.data.push_back(contacted_force_data(2,0));
      filtered_force_torque_data_msg.data.push_back(contacted_force_data(3,0));
      filtered_force_torque_data_msg.data.push_back(contacted_force_data(4,0));
      filtered_force_torque_data_msg.data.push_back(contacted_force_data(5,0));

      test_data_pub.publish(test_data_msg);
      raw_force_torque_data_pub.publish(raw_force_torque_data_msg);
      filtered_force_torque_data_pub.publish(filtered_force_torque_data_msg);
      filtered_force_torque_data_msg.data.clear();
      raw_force_torque_data_msg.data.clear();
      test_data_msg.data.clear();

      //data recording
      for(int num = 0; num<6; num++)
      {
        getActualTCPForce += " "+to_string(raw_force_torque_data(num,0));
        getFilteredForce += " "+to_string(contacted_force_data(num,0));
        getContactedForceTorque += " "+to_string(contacted_force_data(num,0));
        getActualQ +=" "+to_string(0);
      }
      for(int num = 0; num<3; num++)
      {
        getActualToolAccelerometer += " "+to_string(tool_acc_data(num,0));
        getTargetTCPPose += " "+to_string(0);
        getActualTCPPose += " "+to_string(tcp_pose(num,0));
        getActualToolSpeed +=" "+to_string(0);
        getActualToolAcc +=" "+to_string(0);

      }
      for(int num = 0; num<3; num++)
      {
        getActualTCPPose += " "+to_string(tcp_pose(num+3,0));
        getTargetTCPPose += " "+to_string(0);
      }
    }

    compensated_pose_vector[0] = desired_pose_vector[0]; // + force_x_compensator->get_final_output();
    compensated_pose_vector[1] = desired_pose_vector[1]; //+ force_x_compensator->get_final_output();
    compensated_pose_vector[2] = desired_pose_vector[2]; //+ force_x_compensator->get_final_output();

    compensated_pose_vector[3] = desired_pose_vector[3]; //+ force_x_compensator->get_final_output();
    compensated_pose_vector[4] = desired_pose_vector[4]; //+ force_x_compensator->get_final_output();
    compensated_pose_vector[5] = desired_pose_vector[5]; //+ force_x_compensator->get_final_output();

    tf_desired = Transform3D<> (Vector3D<>(compensated_pose_vector[0], compensated_pose_vector[1], compensated_pose_vector[2]),
        EAA<>(compensated_pose_vector[3], compensated_pose_vector[4], compensated_pose_vector[5]).toRotation3D());


    std::vector<Q> solutions = solver.solve(tf_desired, state);


    //from covid - robot
    for (unsigned int num = 0; num < 6; num ++)
    {
      // LOG_DEBUG("[%s]", "Wrapping for joint");
      const double diffOrig = fabs(current_Q[num] - solutions[1][num]);
      // LOG_DEBUG("[diffOrig: %e , %e]", (solutions[i][j])/rw::math::Pi*180.0, diffOrig );

      const double diffAdd = fabs(current_Q[num] - (solutions[1][num] + 2 * rw::math::Pi));
      // LOG_DEBUG("[diffAdd: %e , %e]", (solutions[i][j]+2*rw::math::Pi)/rw::math::Pi*180.0, diffAdd );

      const double diffSub = fabs(current_Q[num] - (solutions[1][num] - 2 * rw::math::Pi));
      // LOG_DEBUG("[diffSub: %e , %e]", (solutions[i][j]-2*rw::math::Pi)/rw::math::Pi*180.0, diffSub );

      if (diffAdd < diffOrig && diffAdd < diffSub)
      {
        solutions[1][num] += 2 * rw::math::Pi;
      }
      else if (diffSub < diffOrig && diffSub < diffAdd)
      {
        solutions[1][num] -= 2 * rw::math::Pi;
      }
    }

    for(int num = 0; num <6 ; num ++)
    {
      if(fabs((solutions[1].toStdVector()[num] - current_Q[num])/control_time) > 45*DEGREE2RADIAN)
      {
        cout << "::" << num << "::" << fabs((solutions[1].toStdVector()[num] - current_Q[num])/control_time) << endl;
        std::cout << COLOR_RED_BOLD << "Robot speed is so FAST" << COLOR_RESET << std::endl;
        joint_vel_limits = true;
      }
    }

    if(!joint_vel_limits)
    {
      if(!gazebo_check)
      {
        rtde_control->servoJ(solutions[1].toStdVector(),0,0,0.002,0.04,100);

        //gazebo_shoulder_pan_position_msg.data = solutions[1].toStdVector()[0];
        //gazebo_shoulder_lift_position_msg.data = solutions[1].toStdVector()[1];
        //gazebo_elbow_position_msg.data = solutions[1].toStdVector()[2];
        //gazebo_wrist_1_position_msg.data = solutions[1].toStdVector()[3];
        //gazebo_wrist_2_position_msg.data = solutions[1].toStdVector()[4];
        //gazebo_wrist_3_position_msg.data = solutions[1].toStdVector()[5];


        //gazebo_shoulder_pan_position_pub.publish(gazebo_shoulder_pan_position_msg);
        //gazebo_shoulder_lift_position_pub.publish(gazebo_shoulder_lift_position_msg);
        //gazebo_elbow_position_pub.publish(gazebo_elbow_position_msg);
        //gazebo_wrist_1_position_pub.publish(gazebo_wrist_1_position_msg);
        //gazebo_wrist_2_position_pub.publish(gazebo_wrist_2_position_msg);
        //gazebo_wrist_3_position_pub.publish(gazebo_wrist_3_position_msg);

      }
      else
      {

        gazebo_shoulder_pan_position_msg.data = solutions[1].toStdVector()[0];
        gazebo_shoulder_lift_position_msg.data = solutions[1].toStdVector()[1];
        gazebo_elbow_position_msg.data = solutions[1].toStdVector()[2];
        gazebo_wrist_1_position_msg.data = solutions[1].toStdVector()[3];
        gazebo_wrist_2_position_msg.data = solutions[1].toStdVector()[4];
        gazebo_wrist_3_position_msg.data = solutions[1].toStdVector()[5];


        gazebo_shoulder_pan_position_pub.publish(gazebo_shoulder_pan_position_msg);
        gazebo_shoulder_lift_position_pub.publish(gazebo_shoulder_lift_position_msg);
        gazebo_elbow_position_pub.publish(gazebo_elbow_position_msg);
        gazebo_wrist_1_position_pub.publish(gazebo_wrist_1_position_msg);
        gazebo_wrist_2_position_pub.publish(gazebo_wrist_2_position_msg);
        gazebo_wrist_3_position_pub.publish(gazebo_wrist_3_position_msg);

      }
      for(int num = 0; num <6 ; num ++)
      {
        current_Q[num] = solutions[1].toStdVector()[num];
      }
    }


    data_line = to_string(time_count)+getTargetTCPPose+getActualTCPPose+getActualTCPForce+getFilteredForce+getContactedForceTorque+
        getActualToolAccelerometer+getActualToolSpeed+getActualToolAcc+getActualQ+"\n";

    out.write(data_line.c_str(),data_line.size());

    getTargetTCPPose = "";
    getActualTCPPose = "";
    getActualTCPForce = "";
    getFilteredForce = "";
    getContactedForceTorque = "";
    getActualToolAccelerometer = "";
    getActualToolSpeed = "";
    getActualToolAcc = "";
    getActualQ = "";

    ros::spinOnce();

    rt_task_wait_period(NULL);
  }
}
void CommandDataMsgCallBack (const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  for(int num = 0; num < 6 ; num++)
  {
    desired_pose_matrix(num,1) = msg->data[num];
  }
  if(msg->data[6] <= 0)
    return;

  for(int num = 0; num < 6 ; num++)
  {
    motion_time =  msg->data[6];
  }

  task_command = "";
  ur10e_task->set_point(msg->data[0], msg->data[1], msg->data[2], msg->data[3], msg->data[4], msg->data[5], msg->data[6]);
}
void TaskCommandDataMsgCallBack (const std_msgs::String::ConstPtr& msg)
{
  static std::string path_ = "";
  task_command = msg->data;

  ur10e_task->clear_task_motion();

  path_ = "/home/yik/sdu_ws/SDU-UR10E-Estimation/config/" + task_command + ".yaml";

  if(!task_command.compare("initialize") || !task_command.compare("initialize_belt_task"))
  {
    ur10e_task->load_task_motion(path_);
  }
  else
  {
    ur10e_task->set_initial_pose_eaa(desired_pose_vector[0], desired_pose_vector[1], desired_pose_vector[2], desired_pose_vector[3], desired_pose_vector[4], desired_pose_vector[5]); // set to be robot initial values
    ur10e_task->trans_tcp_to_base_motion(path_);
  }
}
void PidGainCommandMsgCallBack (const std_msgs::Float64MultiArray::ConstPtr& msg)
{
  f_kp = msg->data[0];
  f_ki = msg->data[1];
  f_kd = msg->data[2];

  YAML::Emitter y_out;
  std::string path_ = "../config/force_pid_gain.yaml";

  y_out << YAML::BeginMap;
  y_out << YAML::Key << "p_gain";
  y_out << YAML::Value << f_kp;
  y_out << YAML::Key << "i_gain";
  y_out << YAML::Value << f_ki ;
  y_out << YAML::Key << "d_gain";
  y_out << YAML::Value << f_kd;
  y_out << YAML::EndMap;
  std::ofstream fout(path_.c_str());
  fout << y_out.c_str(); // dump it back into the file
}
void initialize()
{
  control_time = 0.002;
  tool_estimation = std::make_shared<ToolEstimation>();

  ur10e_traj = std::make_shared<CalRad>();
  ur10e_task = std::make_shared<TaskMotion>();
  force_x_compensator = std::make_shared<PID_function>(control_time, 0.02, -0.02, 0, 0, 0, 0.0001, -0.0001);
  force_y_compensator = std::make_shared<PID_function>(control_time, 0.02, -0.02, 0, 0, 0, 0.0001, -0.0001);
  force_z_compensator = std::make_shared<PID_function>(control_time, 0.02, -0.02, 0, 0, 0, 0.0001, -0.0001);

  //kinematics
  tf_current_matrix.resize(4,4);
  tf_current_matrix.fill(0);

  // fifth order traj
  ur10e_traj->set_control_time(control_time);

  //motion task
  ur10e_task->initialize(control_time);

  //robot A
  //[-0.45581,0.42951,0.27748,1.037,2.504,-2.504]
  ur10e_task->set_initial_pose(-0.6548931267372944, 0.21557220865110965, 0.2596808650645809, -0.7290212721930756, -1.7599986104134355, 1.7600122635046096); // set to be robot initial values
  ur10e_task->set_initial_pose_eaa(-0.6548931267372944, 0.21557220865110965, 0.2596808650645809, -0.7290212721930756, -1.7599986104134355, 1.7600122635046096); // set to be robot initial values

  //robot B
  //ur10e_task->set_initial_pose(-0.5, 0.184324, 0.5875, -180*DEGREE2RADIAN, 0, 0); // set to be robot initial values

  motion_time = 5;

  tcp_pose.resize(6,1);
  tcp_pose.fill(0);

  desired_pose_matrix.resize(6,8);
  desired_pose_matrix.fill(0);

  task_command = "";

  //robot A
  //  desired_pose_matrix(0,1) = -0.5;
  //  desired_pose_matrix(1,1) = 0.184324;
  //  desired_pose_matrix(2,1) = 0.5875;
  //  desired_pose_matrix(3,1) = -180*DEGREE2RADIAN;
  //  desired_pose_matrix(4,1) = 0;
  //  desired_pose_matrix(5,1) = 0;

  //ft sensor
  raw_force_torque_data.resize(6,1);
  raw_force_torque_data.fill(0);

  tool_acc_data.resize(4,1);
  tool_acc_data.fill(0);
  tool_acc_data(3,0)=1;

  contacted_force_data.resize(6,1);
  contacted_force_data.fill(0);

  // initial and compensated pose load
  desired_pose_vector.resize(6);
  desired_force_torque_vector.resize(6);
  compensated_pose_vector.resize(6);

  for(int num = 0; num < 6; num ++)
  {
    compensated_pose_vector[num] = 0;
    desired_force_torque_vector[num] = 0;
  }

  // real robot

  //robot A
  //  desired_pose_vector[0] = -0.41361;
  //  desired_pose_vector[1] = 0.03022;
  //  desired_pose_vector[2] = 0.45822;
  //  desired_pose_vector[3] = 2.222;
  //  desired_pose_vector[4] = 2.220;
  //  desired_pose_vector[5] = 0.001;

  desired_pose_vector[0] = -0.6548931267372944;
  desired_pose_vector[1] = 0.21557220865110965;
  desired_pose_vector[2] = 0.2596808650645809;
  desired_pose_vector[3] = -0.7290212721930756;
  desired_pose_vector[4] = -1.7599986104134355;
  desired_pose_vector[5] = 1.7600122635046096;


  //robot B
  //  desired_pose_vector[0] = -0.5;
  //  desired_pose_vector[1] = 0.184324;
  //  desired_pose_vector[2] = 0.5875;
  //  desired_pose_vector[3] = -180*DEGREE2RADIAN;
  //  desired_pose_vector[4] = 0;
  //  desired_pose_vector[5] = 0;

  // force controller
  f_kp = 0;
  f_ki = 0;
  f_kd = 0;

  tool_estimation->initialize();
  tool_estimation ->set_parameters(control_time, 1.75);

  //gazebo init
  //robot A
  gazebo_shoulder_pan_position_msg.data = 2.65787;
  gazebo_shoulder_lift_position_msg.data = -1.85522;
  gazebo_elbow_position_msg.data = -2.03556;
  gazebo_wrist_1_position_msg.data = -2.3924;
  gazebo_wrist_2_position_msg.data = -2.83993;
  gazebo_wrist_3_position_msg.data =  -3.14157;

  current_Q[0] = 2.65787;
  current_Q[1] = -1.85522;
  current_Q[2] = -2.03556;
  current_Q[3] = -2.3924;
  current_Q[4] = -2.83993;
  current_Q[5] =  -3.14157;


  joint_vel_limits = false;


  //initial position

  //  0 Q[6]{2.47941, 2.5424, 1.98852, 1.75312, -3.01825, -3.141}
  //  1 Q[6]{2.47941, -1.85904, -1.98852, -2.43477, -3.01825, -3.141}
  //  2 Q[6]{2.47941, 2.13072, 2.06185, -1.05013, 3.01825, 0.000591903}
  //  3 Q[6]{2.47941, -2.20645, -2.06185, 1.12756, 3.01825, 0.000591903}
  //  4 Q[6]{-1.21905, -1.28237, 1.98827, -0.705646, -0.433526, 3.14111}
  //  5 Q[6]{-1.21905, 0.599148, -1.98827, 1.38937, -0.433526, 3.14111}
  //  6 Q[6]{-1.21905, -0.935248, 2.06211, 2.01498, 0.433526, -0.000478964}
  //  7 Q[6]{-1.21905, 1.01099, -2.06211, -2.09022, 0.433526, -0.000478964}


  //
  //  //robot B
  //  gazebo_shoulder_pan_position_msg.data = 3.1213190230795913;
  //  gazebo_shoulder_lift_position_msg.data = -1.2650450021126538;
  //  gazebo_elbow_position_msg.data = -1.9836958306426968;
  //  gazebo_wrist_1_position_msg.data = -1.4640438428275298;
  //  gazebo_wrist_2_position_msg.data = 1.5707963268419363;
  //  gazebo_wrist_3_position_msg.data = 1.5505226962847685;

  //data variables define
  getTargetTCPPose = " target_tcp_pose_x target_tcp_pose_y target_tcp_pose_z target_tcp_pose_r target_tcp_pose_p target_tcp_pose_y";
  getActualTCPPose = " actual_tcp_pose_x actual_tcp_pose_y actual_tcp_pose_z actual_tcp_pose_r actual_tcp_pose_p actual_tcp_pose_y";
  getActualTCPForce = " actual_tcp_force_x actual_tcp_force_y actual_tcp_force_z actual_tcp_force_r actual_tcp_force_p actual_tcp_force_y";
  getFilteredForce = " filtered_force_x filtered_force_y filtered_force_z filtered_torque_r filtered_torque_p filtered_torque_y";
  getContactedForceTorque = " contacted_force_x contacted_force_y contacted_force_z contacted_torque_r contacted_torque_p contacted_torque_y";
  getActualToolAccelerometer = " actual_tcp_acc_x actual_tcp_acc_y actual_tcp_acc_z";
  getActualToolSpeed = " actual_tcp_speed_r actual_tcp_speed_p actual_tcp_speed_y";
  getActualToolAcc = " actual_tcp_acc_r actual_tcp_acc_p actual_tcp_acc_y";
  getActualQ = " actual_q_x actual_q_y actual_q_z";

  data_line = "time"+getTargetTCPPose+getActualTCPPose+getActualTCPForce+getFilteredForce+getContactedForceTorque+
      getActualToolAccelerometer+getActualToolSpeed+getActualToolAcc+getActualQ+"\n";

  //out << data_line << std::endl;
  out.write(data_line.c_str(),data_line.size());

  getTargetTCPPose = "";
  getActualTCPPose = "";
  getActualTCPForce = "";
  getFilteredForce = "";
  getContactedForceTorque = "";
  getActualToolAccelerometer = "";
  getActualToolSpeed = "";
  getActualToolAcc = "";
  getActualQ = "";
}


int main (int argc, char **argv)
{
  gazebo_check = true;
  std::string silmulation_on_off;
  std::cout << COLOR_YELLOW_BOLD << "Simulation On [ yes / no ]" << COLOR_RESET << std::endl;
  cin >> silmulation_on_off;

  if(!silmulation_on_off.compare("yes") || !silmulation_on_off.compare("y"))
    std::cout << COLOR_GREEN_BOLD << "Setting up Simulation " << COLOR_RESET << std::endl;
  else
  {
    std::cout << COLOR_GREEN_BOLD << "REAL Robot, Be careful to run:" << COLOR_RESET << std::endl;
    std::cout << COLOR_GREEN_BOLD << "Are you sure ? [yes / no]" << COLOR_RESET << std::endl;
    cin >> silmulation_on_off;
    if(!silmulation_on_off.compare("no") || !silmulation_on_off.compare("n"))
    {
      std::cout << COLOR_RED_BOLD << "Terminate program" << COLOR_RESET << std::endl;
      return 0;
    }
    gazebo_check = false;
  }

  std::cout << COLOR_GREEN_BOLD << "Program Start:" << COLOR_RESET << std::endl;
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;

  //ros publisher
  joint_cur_value_pub = n.advertise<std_msgs::Float64MultiArray>("/sdu/ur10e/joint_cur_value", 10);
  ee_cur_value_pub = n.advertise<std_msgs::Float64MultiArray>("/sdu/ur10e/ee_cur_value", 10);
  raw_force_torque_data_pub = n.advertise<std_msgs::Float64MultiArray>("/sdu/ur10e/raw_force_torque_data", 10);
  filtered_force_torque_data_pub = n.advertise<std_msgs::Float64MultiArray>("/sdu/ur10e/filtered_force_torque_data", 10);

  test_data_pub = n.advertise<std_msgs::Float64MultiArray>("/sdu/ur10e/test_data", 10);

  // ros subsrcibe
  ros::Subscriber zero_command_sub;
  ros::Subscriber command_sub;
  ros::Subscriber task_command_sub;
  ros::Subscriber pid_gain_command_sub;

  command_sub = n.subscribe("/sdu/ur10e/ee_command", 10, CommandDataMsgCallBack);
  task_command_sub = n.subscribe("/sdu/ur10e/task_command", 10, TaskCommandDataMsgCallBack);
  pid_gain_command_sub = n.subscribe("/sdu/ur10e/pid_gain_command", 10, PidGainCommandMsgCallBack);

  initialize();
  usleep(3000000);

    gazebo_shoulder_pan_position_pub = n.advertise<std_msgs::Float64>("/ur10e_robot/shoulder_pan_position/command", 10);
    gazebo_shoulder_lift_position_pub = n.advertise<std_msgs::Float64>("/ur10e_robot/shoulder_lift_position/command", 10);
    gazebo_elbow_position_pub = n.advertise<std_msgs::Float64>("/ur10e_robot/elbow_position/command", 10);
    gazebo_wrist_1_position_pub = n.advertise<std_msgs::Float64>("/ur10e_robot/wrist_1_position/command", 10);
    gazebo_wrist_2_position_pub = n.advertise<std_msgs::Float64>("/ur10e_robot/wrist_2_position/command", 10);
    gazebo_wrist_3_position_pub = n.advertise<std_msgs::Float64>("/ur10e_robot/wrist_3_position/command", 10);

    gazebo_shoulder_pan_position_pub.publish(gazebo_shoulder_pan_position_msg);
    gazebo_shoulder_lift_position_pub.publish(gazebo_shoulder_lift_position_msg);
    gazebo_elbow_position_pub.publish(gazebo_elbow_position_msg);
    gazebo_wrist_1_position_pub.publish(gazebo_wrist_1_position_msg);
    gazebo_wrist_2_position_pub.publish(gazebo_wrist_2_position_msg);
    gazebo_wrist_3_position_pub.publish(gazebo_wrist_3_position_msg);

  if(gazebo_check)
  {
    gazebo_shoulder_pan_position_pub = n.advertise<std_msgs::Float64>("/ur10e_robot/shoulder_pan_position/command", 10);
    gazebo_shoulder_lift_position_pub = n.advertise<std_msgs::Float64>("/ur10e_robot/shoulder_lift_position/command", 10);
    gazebo_elbow_position_pub = n.advertise<std_msgs::Float64>("/ur10e_robot/elbow_position/command", 10);
    gazebo_wrist_1_position_pub = n.advertise<std_msgs::Float64>("/ur10e_robot/wrist_1_position/command", 10);
    gazebo_wrist_2_position_pub = n.advertise<std_msgs::Float64>("/ur10e_robot/wrist_2_position/command", 10);
    gazebo_wrist_3_position_pub = n.advertise<std_msgs::Float64>("/ur10e_robot/wrist_3_position/command", 10);

    gazebo_shoulder_pan_position_pub.publish(gazebo_shoulder_pan_position_msg);
    gazebo_shoulder_lift_position_pub.publish(gazebo_shoulder_lift_position_msg);
    gazebo_elbow_position_pub.publish(gazebo_elbow_position_msg);
    gazebo_wrist_1_position_pub.publish(gazebo_wrist_1_position_msg);
    gazebo_wrist_2_position_pub.publish(gazebo_wrist_2_position_msg);
    gazebo_wrist_3_position_pub.publish(gazebo_wrist_3_position_msg);
  }
  else
  {
    const std::string robot_ip = "192.168.1.130"; // robot B

    rtde_receive = std::make_shared<RTDEReceiveInterface>(robot_ip);
    rtde_control = std::make_shared<RTDEControlInterface>(robot_ip);

    rtde_control->zeroFtSensor();

    std::cout << COLOR_YELLOW_BOLD << "Robot connected to your program" << COLOR_RESET << std::endl;
    std::cout << COLOR_RED_BOLD << "Robot will move 2 seconds later" << COLOR_RESET << std::endl;
    usleep(2000000);
    rtde_control->moveJ(current_Q,0.1,0.1); // move to initial pose
    std::cout << COLOR_RED_BOLD << "Send" << COLOR_RESET << std::endl;

    usleep(2000000);
  }

  ros::spinOnce();

  usleep(3000000);

  std::cout << COLOR_GREEN << "All of things were initialized!" << COLOR_RESET << std::endl;

  //real time task
  char str[35];
  mlockall(MCL_CURRENT | MCL_FUTURE); //Lock the memory to avoid memory swapping for this program

  printf("Starting cyclic task...\n");
  sprintf(str, "Belt Task Start");
  rt_task_create(&loop_task, str, 0, 80, 0);//Create the real time task
  rt_task_start(&loop_task, &loop_task_proc, 0);//Since task starts in suspended mode, start task

  std::cout << COLOR_GREEN << "Real time task loop was created!" << COLOR_RESET << std::endl;

  pause();
  rt_task_delete(&loop_task);

  usleep(3000000);
  if(!gazebo_check)
  {
    rtde_control->servoStop();
  }
  out.close();
  cout << "complete and save" << "\n\n";
  printf("exiting safely\n");
  return 0;
}
