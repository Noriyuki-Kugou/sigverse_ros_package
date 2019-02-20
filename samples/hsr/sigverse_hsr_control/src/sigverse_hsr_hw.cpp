#include <sigverse_hsr_control/sigverse_hsr_hw.h>

SigverseHSR::SigverseHSR()
{
  is_cmd_value_initialized = false;
  pub_arm_trajectory_      = nh_.advertise<trajectory_msgs::JointTrajectory>("/hsrb/arm_trajectory_controller/command_sigverse", 10);
  pub_head_trajectory_     = nh_.advertise<trajectory_msgs::JointTrajectory>("/hsrb/head_trajectory_controller/command_sigverse", 10);
  pub_gripper_trajectory_  = nh_.advertise<trajectory_msgs::JointTrajectory>("/hsrb/gripper_controller/command_sigverse", 10);
  sub_joint_states_        = nh_.subscribe("/hsrb/joint_states_sigverse", 100, &SigverseHSR::jointStateCallback, this);

  // register the joint state interface
  hardware_interface::JointStateHandle state_handle_arm_lift              ("arm_lift_joint",               &pos_[0], &vel_[0], &eff_[0]);
  hardware_interface::JointStateHandle state_handle_arm_flex              ("arm_flex_joint",               &pos_[1], &vel_[1], &eff_[1]);
  hardware_interface::JointStateHandle state_handle_arm_roll              ("arm_roll_joint",               &pos_[2], &vel_[2], &eff_[2]);
  hardware_interface::JointStateHandle state_handle_wrist_flex            ("wrist_flex_joint",             &pos_[3], &vel_[3], &eff_[3]);
  hardware_interface::JointStateHandle state_handle_wrist_roll            ("wrist_roll_joint",             &pos_[4], &vel_[4], &eff_[4]);
  hardware_interface::JointStateHandle state_handle_head_pan              ("head_pan_joint",               &pos_[5], &vel_[5], &eff_[5]);
  hardware_interface::JointStateHandle state_handle_head_tilt             ("head_tilt_joint",              &pos_[6], &vel_[6], &eff_[6]);
  hardware_interface::JointStateHandle state_handle_hand_motor            ("hand_motor_joint",             &pos_[7], &vel_[7], &eff_[7]);
  hardware_interface::JointStateHandle state_handle_hand_l_spring_proximal("hand_l_spring_proximal_joint", &pos_[8], &vel_[8], &eff_[8]);
  hardware_interface::JointStateHandle state_handle_hand_r_spring_proximal("hand_r_spring_proximal_joint", &pos_[9], &vel_[9], &eff_[9]);
  jnt_state_interface_.registerHandle(state_handle_arm_lift);
  jnt_state_interface_.registerHandle(state_handle_arm_flex);
  jnt_state_interface_.registerHandle(state_handle_arm_roll);
  jnt_state_interface_.registerHandle(state_handle_wrist_flex);
  jnt_state_interface_.registerHandle(state_handle_wrist_roll);
  jnt_state_interface_.registerHandle(state_handle_head_pan);
  jnt_state_interface_.registerHandle(state_handle_head_tilt);
  jnt_state_interface_.registerHandle(state_handle_hand_motor);
  jnt_state_interface_.registerHandle(state_handle_hand_l_spring_proximal);
  jnt_state_interface_.registerHandle(state_handle_hand_r_spring_proximal);
  registerInterface(&jnt_state_interface_);

  // register the joint position interface
  hardware_interface::JointHandle pos_handle_arm_lift  (jnt_state_interface_.getHandle("arm_lift_joint"),   &cmd_[0]);
  hardware_interface::JointHandle pos_handle_arm_flex  (jnt_state_interface_.getHandle("arm_flex_joint"),   &cmd_[1]);
  hardware_interface::JointHandle pos_handle_arm_roll  (jnt_state_interface_.getHandle("arm_roll_joint"),   &cmd_[2]);
  hardware_interface::JointHandle pos_handle_wrist_flex(jnt_state_interface_.getHandle("wrist_flex_joint"), &cmd_[3]);
  hardware_interface::JointHandle pos_handle_wrist_roll(jnt_state_interface_.getHandle("wrist_roll_joint"), &cmd_[4]);
  hardware_interface::JointHandle pos_handle_head_pan  (jnt_state_interface_.getHandle("head_pan_joint"),   &cmd_[5]);
  hardware_interface::JointHandle pos_handle_head_tilt (jnt_state_interface_.getHandle("head_tilt_joint"),  &cmd_[6]);
  hardware_interface::JointHandle pos_handle_hand_motor(jnt_state_interface_.getHandle("hand_motor_joint"), &cmd_[7]);
  jnt_pos_interface_.registerHandle(pos_handle_arm_lift);
  jnt_pos_interface_.registerHandle(pos_handle_arm_flex);
  jnt_pos_interface_.registerHandle(pos_handle_arm_roll);
  jnt_pos_interface_.registerHandle(pos_handle_wrist_flex);
  jnt_pos_interface_.registerHandle(pos_handle_wrist_roll);
  jnt_pos_interface_.registerHandle(pos_handle_head_pan);
  jnt_pos_interface_.registerHandle(pos_handle_head_tilt);
  jnt_pos_interface_.registerHandle(pos_handle_hand_motor);
  registerInterface(&jnt_pos_interface_);

  // register the joint limits interface
  JointLimits joint_limits;
  SoftJointLimits soft_joint_limits;
  getJointLimits("arm_lift_joint",   nh_, joint_limits);
  PositionJointSoftLimitsHandle joint_limit_arm_lift    (pos_handle_arm_lift, joint_limits, soft_joint_limits);
  getJointLimits("arm_flex_joint",   nh_, joint_limits);
  PositionJointSoftLimitsHandle joint_limit_arm_flex    (pos_handle_arm_flex, joint_limits, soft_joint_limits);
  getJointLimits("arm_roll_joint",   nh_, joint_limits);
  PositionJointSoftLimitsHandle joint_limit_arm_roll   (pos_handle_arm_roll,  joint_limits, soft_joint_limits);
  getJointLimits("wrist_flex_joint", nh_, joint_limits);
  PositionJointSoftLimitsHandle joint_limit_wrist_flex(pos_handle_wrist_flex, joint_limits, soft_joint_limits);
  getJointLimits("wrist_roll_joint", nh_, joint_limits);
  PositionJointSoftLimitsHandle joint_limit_wrist_roll(pos_handle_wrist_roll, joint_limits, soft_joint_limits);
  getJointLimits("head_pan_joint",   nh_, joint_limits);
  PositionJointSoftLimitsHandle joint_limit_head_pan    (pos_handle_head_pan, joint_limits, soft_joint_limits);
  getJointLimits("head_tilt_joint",  nh_, joint_limits);
  PositionJointSoftLimitsHandle joint_limit_head_tilt  (pos_handle_head_tilt, joint_limits, soft_joint_limits);
  getJointLimits("hand_motor_joint", nh_, joint_limits);
  PositionJointSoftLimitsHandle joint_limit_hand_motor(pos_handle_hand_motor, joint_limits, soft_joint_limits);
  position_joint_soft_limits_interface_.registerHandle(joint_limit_arm_lift);
  position_joint_soft_limits_interface_.registerHandle(joint_limit_arm_flex);
  position_joint_soft_limits_interface_.registerHandle(joint_limit_arm_roll);
  position_joint_soft_limits_interface_.registerHandle(joint_limit_wrist_flex);
  position_joint_soft_limits_interface_.registerHandle(joint_limit_wrist_roll);
  position_joint_soft_limits_interface_.registerHandle(joint_limit_head_pan);
  position_joint_soft_limits_interface_.registerHandle(joint_limit_head_tilt);
  position_joint_soft_limits_interface_.registerHandle(joint_limit_hand_motor);
  registerInterface(&position_joint_soft_limits_interface_);
}

void SigverseHSR::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  for(int i = 0; i < msg->name.size(); i++)
  {
    if(msg->name[i]      == "arm_lift_joint"){               pos_[0] = msg->position[i]; }
    else if(msg->name[i] == "arm_flex_joint"){               pos_[1] = msg->position[i]; }
    else if(msg->name[i] == "arm_roll_joint"){               pos_[2] = msg->position[i]; }
    else if(msg->name[i] == "wrist_flex_joint"){             pos_[3] = msg->position[i]; }
    else if(msg->name[i] == "wrist_roll_joint"){             pos_[4] = msg->position[i]; }
    else if(msg->name[i] == "head_pan_joint"){               pos_[5] = msg->position[i]; }
    else if(msg->name[i] == "head_tilt_joint"){              pos_[6] = msg->position[i]; }
    else if(msg->name[i] == "hand_motor_joint"){             pos_[7] = msg->position[i]; }
    else if(msg->name[i] == "hand_l_spring_proximal_joint"){ pos_[8] = msg->position[i]; }
    else if(msg->name[i] == "hand_r_spring_proximal_joint"){ pos_[9] = msg->position[i]; }
  }

  if(is_cmd_value_initialized == false){
    for(int i = 0; i < msg->name.size(); i++)
    {
      if(msg->name[i]      == "arm_lift_joint"){               cmd_[0] = msg->position[i]; }
      else if(msg->name[i] == "arm_flex_joint"){               cmd_[1] = msg->position[i]; }
      else if(msg->name[i] == "arm_roll_joint"){               cmd_[2] = msg->position[i]; }
      else if(msg->name[i] == "wrist_flex_joint"){             cmd_[3] = msg->position[i]; }
      else if(msg->name[i] == "wrist_roll_joint"){             cmd_[4] = msg->position[i]; }
      else if(msg->name[i] == "head_pan_joint"){               cmd_[5] = msg->position[i]; }
      else if(msg->name[i] == "head_tilt_joint"){              cmd_[6] = msg->position[i]; }
      else if(msg->name[i] == "hand_motor_joint"){             cmd_[7] = msg->position[i]; }
      else if(msg->name[i] == "hand_l_spring_proximal_joint"){ cmd_[8] = msg->position[i]; }
      else if(msg->name[i] == "hand_r_spring_proximal_joint"){ cmd_[9] = msg->position[i]; }
    }
    is_cmd_value_initialized = true;
  }
}

void SigverseHSR::read(ros::Time time, ros::Duration period){}

void SigverseHSR::write(ros::Time time, ros::Duration period)
{
  if(is_cmd_value_initialized == false){ return; }
  publishTargetArmPosition();
  publishTargetHeadPosition();
  publishTargetHandPosition();
  position_joint_soft_limits_interface_.enforceLimits(period);
}

void SigverseHSR::publishTargetArmPosition()
{
  trajectory_msgs::JointTrajectory traj;

  traj.joint_names.push_back("arm_lift_joint");
  traj.joint_names.push_back("arm_flex_joint");
  traj.joint_names.push_back("arm_roll_joint");
  traj.joint_names.push_back("wrist_flex_joint");
  traj.joint_names.push_back("wrist_roll_joint");

  traj.points.resize(1);
  traj.points[0].positions.resize(5);
  traj.points[0].positions[0] = cmd_[0];
  traj.points[0].positions[1] = cmd_[1];
  traj.points[0].positions[2] = cmd_[2];
  traj.points[0].positions[3] = cmd_[3];
  traj.points[0].positions[4] = cmd_[4];

  traj.points[0].time_from_start = ros::Duration(0.0);

  pub_arm_trajectory_.publish(traj);
}

void SigverseHSR::publishTargetHeadPosition()
{
  trajectory_msgs::JointTrajectory traj;

  traj.joint_names.push_back("head_pan_joint");
  traj.joint_names.push_back("head_tilt_joint");

  traj.points.resize(1);
  traj.points[0].positions.resize(2);
  traj.points[0].positions[0] = cmd_[5];
  traj.points[0].positions[1] = cmd_[6];

  traj.points[0].time_from_start = ros::Duration(0.0);

  pub_head_trajectory_.publish(traj);
}

void SigverseHSR::publishTargetHandPosition()
{
  trajectory_msgs::JointTrajectory traj;

  traj.joint_names.push_back("hand_motor_joint");
  traj.points.resize(1);
  traj.points[0].positions.resize(1);
  traj.points[0].positions[0] = cmd_[7];
  traj.points[0].time_from_start = ros::Duration(0.0);

  pub_gripper_trajectory_.publish(traj);
}
