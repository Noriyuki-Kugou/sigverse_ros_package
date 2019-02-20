#include <sigverse_pr2_control/sigverse_pr2_hw.h>
#include <iostream> // for debug

SigversePR2::SigversePR2()
{
  is_cmd_value_initialized  = false;
  pub_left_arm_trajectory_  = nh_.advertise<trajectory_msgs::JointTrajectory>("/l_arm_controller/command", 10);
  pub_right_arm_trajectory_ = nh_.advertise<trajectory_msgs::JointTrajectory>("/r_arm_controller/command", 10);
  pub_head_trajectory_      = nh_.advertise<trajectory_msgs::JointTrajectory>("/head_traj_controller/command", 10);
  pub_torso_trajectory_     = nh_.advertise<trajectory_msgs::JointTrajectory>("/torso_controller/command", 10);
  sub_joint_states_         = nh_.subscribe("/joint_states_sigverse", 100, &SigversePR2::jointStateCallback, this);

  // register the joint state interface
  hardware_interface::JointStateHandle state_handle_fl_caster_rotation    ("fl_caster_rotation_joint",     &pos_[0],  &vel_[0],  &eff_[0]);
  hardware_interface::JointStateHandle state_handle_fl_caster_l_wheel     ("fl_caster_l_wheel_joint",      &pos_[1],  &vel_[1],  &eff_[1]);
  hardware_interface::JointStateHandle state_handle_fl_caster_r_wheel     ("fl_caster_r_wheel_joint",      &pos_[2],  &vel_[2],  &eff_[2]);
  hardware_interface::JointStateHandle state_handle_fr_caster_rotation    ("fr_caster_rotation_joint",     &pos_[3],  &vel_[3],  &eff_[3]);
  hardware_interface::JointStateHandle state_handle_fr_caster_l_wheel     ("fr_caster_l_wheel_joint",      &pos_[4],  &vel_[4],  &eff_[4]);
  hardware_interface::JointStateHandle state_handle_fr_caster_r_wheel     ("fr_caster_r_wheel_joint",      &pos_[5],  &vel_[5],  &eff_[5]);
  hardware_interface::JointStateHandle state_handle_bl_caster_rotation    ("bl_caster_rotation_joint",     &pos_[6],  &vel_[6],  &eff_[6]);
  hardware_interface::JointStateHandle state_handle_bl_caster_l_wheel     ("bl_caster_l_wheel_joint",      &pos_[7],  &vel_[7],  &eff_[7]);
  hardware_interface::JointStateHandle state_handle_bl_caster_r_wheel     ("bl_caster_r_wheel_joint",      &pos_[8],  &vel_[8],  &eff_[8]);
  hardware_interface::JointStateHandle state_handle_br_caster_rotation    ("br_caster_rotation_joint",     &pos_[9],  &vel_[9],  &eff_[9]);
  hardware_interface::JointStateHandle state_handle_br_caster_l_wheel     ("br_caster_l_wheel_joint",      &pos_[10], &vel_[10], &eff_[10]);
  hardware_interface::JointStateHandle state_handle_br_caster_r_wheel     ("br_caster_r_wheel_joint",      &pos_[11], &vel_[11], &eff_[11]);
  hardware_interface::JointStateHandle state_handle_torso_lift            ("torso_lift_joint",             &pos_[12], &vel_[12], &eff_[12]);
  hardware_interface::JointStateHandle state_handle_torso_lift_motor_screw("torso_lift_motor_screw_joint", &pos_[13], &vel_[13], &eff_[13]);
  hardware_interface::JointStateHandle state_handle_head_pan              ("head_pan_joint",               &pos_[14], &vel_[14], &eff_[14]);
  hardware_interface::JointStateHandle state_handle_head_tilt             ("head_tilt_joint",              &pos_[15], &vel_[15], &eff_[15]);
  hardware_interface::JointStateHandle state_handle_laser_tilt_mount      ("laser_tilt_mount_joint",       &pos_[16], &vel_[16], &eff_[16]);
  hardware_interface::JointStateHandle state_handle_r_upper_arm_roll      ("r_upper_arm_roll_joint",       &pos_[17], &vel_[17], &eff_[17]);
  hardware_interface::JointStateHandle state_handle_r_shoulder_pan        ("r_shoulder_pan_joint",         &pos_[18], &vel_[18], &eff_[18]);
  hardware_interface::JointStateHandle state_handle_r_shoulder_lift       ("r_shoulder_lift_joint",        &pos_[19], &vel_[19], &eff_[19]);
  hardware_interface::JointStateHandle state_handle_r_forearm_roll        ("r_forearm_roll_joint",         &pos_[20], &vel_[20], &eff_[20]);
  hardware_interface::JointStateHandle state_handle_r_elbow_flex          ("r_elbow_flex_joint",           &pos_[21], &vel_[21], &eff_[21]);
  hardware_interface::JointStateHandle state_handle_r_wrist_flex          ("r_wrist_flex_joint",           &pos_[22], &vel_[22], &eff_[22]);
  hardware_interface::JointStateHandle state_handle_r_wrist_roll          ("r_wrist_roll_joint",           &pos_[23], &vel_[23], &eff_[23]);
  hardware_interface::JointStateHandle state_handle_r_gripper             ("r_gripper_joint",              &pos_[24], &vel_[24], &eff_[24]);
  hardware_interface::JointStateHandle state_handle_r_gripper_l_finger    ("r_gripper_l_finger_joint",     &pos_[25], &vel_[25], &eff_[25]);
  hardware_interface::JointStateHandle state_handle_r_gripper_r_finger    ("r_gripper_r_finger_joint",     &pos_[26], &vel_[26], &eff_[26]);
  hardware_interface::JointStateHandle state_handle_r_gripper_r_finger_tip("r_gripper_r_finger_tip_joint", &pos_[27], &vel_[27], &eff_[27]);
  hardware_interface::JointStateHandle state_handle_r_gripper_l_finger_tip("r_gripper_l_finger_tip_joint", &pos_[28], &vel_[28], &eff_[28]);
  hardware_interface::JointStateHandle state_handle_r_gripper_motor_screw ("r_gripper_motor_screw_joint",  &pos_[29], &vel_[29], &eff_[29]);
  hardware_interface::JointStateHandle state_handle_r_gripper_motor_slider("r_gripper_motor_slider_joint", &pos_[30], &vel_[30], &eff_[30]);
  hardware_interface::JointStateHandle state_handle_l_upper_arm_roll      ("l_upper_arm_roll_joint",       &pos_[31], &vel_[31], &eff_[31]);
  hardware_interface::JointStateHandle state_handle_l_shoulder_pan        ("l_shoulder_pan_joint",         &pos_[32], &vel_[32], &eff_[32]);
  hardware_interface::JointStateHandle state_handle_l_shoulder_lift       ("l_shoulder_lift_joint",        &pos_[33], &vel_[33], &eff_[33]);
  hardware_interface::JointStateHandle state_handle_l_forearm_roll        ("l_forearm_roll_joint",         &pos_[34], &vel_[34], &eff_[34]);
  hardware_interface::JointStateHandle state_handle_l_elbow_flex          ("l_elbow_flex_joint",           &pos_[35], &vel_[35], &eff_[35]);
  hardware_interface::JointStateHandle state_handle_l_wrist_flex          ("l_wrist_flex_joint",           &pos_[36], &vel_[36], &eff_[36]);
  hardware_interface::JointStateHandle state_handle_l_wrist_roll          ("l_wrist_roll_joint",           &pos_[37], &vel_[37], &eff_[37]);
  hardware_interface::JointStateHandle state_handle_l_gripper             ("l_gripper_joint",              &pos_[38], &vel_[38], &eff_[38]);
  hardware_interface::JointStateHandle state_handle_l_gripper_l_finger    ("l_gripper_l_finger_joint",     &pos_[39], &vel_[39], &eff_[39]);
  hardware_interface::JointStateHandle state_handle_l_gripper_r_finger    ("l_gripper_r_finger_joint",     &pos_[40], &vel_[40], &eff_[40]);
  hardware_interface::JointStateHandle state_handle_l_gripper_r_finger_tip("l_gripper_r_finger_tip_joint", &pos_[41], &vel_[41], &eff_[41]);
  hardware_interface::JointStateHandle state_handle_l_gripper_l_finger_tip("l_gripper_l_finger_tip_joint", &pos_[42], &vel_[42], &eff_[42]);
  hardware_interface::JointStateHandle state_handle_l_gripper_motor_screw ("l_gripper_motor_screw_joint",  &pos_[43], &vel_[43], &eff_[43]);
  hardware_interface::JointStateHandle state_handle_l_gripper_motor_slider("l_gripper_motor_slider_joint", &pos_[44], &vel_[44], &eff_[44]);
  jnt_state_interface_.registerHandle(state_handle_fl_caster_rotation);
  jnt_state_interface_.registerHandle(state_handle_fl_caster_l_wheel);
  jnt_state_interface_.registerHandle(state_handle_fl_caster_r_wheel);
  jnt_state_interface_.registerHandle(state_handle_fr_caster_rotation);
  jnt_state_interface_.registerHandle(state_handle_fr_caster_l_wheel);
  jnt_state_interface_.registerHandle(state_handle_fr_caster_r_wheel);
  jnt_state_interface_.registerHandle(state_handle_bl_caster_rotation);
  jnt_state_interface_.registerHandle(state_handle_bl_caster_l_wheel);
  jnt_state_interface_.registerHandle(state_handle_bl_caster_r_wheel);
  jnt_state_interface_.registerHandle(state_handle_br_caster_rotation);
  jnt_state_interface_.registerHandle(state_handle_br_caster_l_wheel);
  jnt_state_interface_.registerHandle(state_handle_br_caster_r_wheel);
  jnt_state_interface_.registerHandle(state_handle_torso_lift);
  jnt_state_interface_.registerHandle(state_handle_torso_lift_motor_screw);
  jnt_state_interface_.registerHandle(state_handle_head_pan);
  jnt_state_interface_.registerHandle(state_handle_head_tilt);
  jnt_state_interface_.registerHandle(state_handle_laser_tilt_mount);
  jnt_state_interface_.registerHandle(state_handle_r_upper_arm_roll);
  jnt_state_interface_.registerHandle(state_handle_r_shoulder_pan);
  jnt_state_interface_.registerHandle(state_handle_r_shoulder_lift);
  jnt_state_interface_.registerHandle(state_handle_r_forearm_roll);
  jnt_state_interface_.registerHandle(state_handle_r_elbow_flex);
  jnt_state_interface_.registerHandle(state_handle_r_wrist_flex);
  jnt_state_interface_.registerHandle(state_handle_r_wrist_roll);
  jnt_state_interface_.registerHandle(state_handle_r_gripper);
  jnt_state_interface_.registerHandle(state_handle_r_gripper_l_finger);
  jnt_state_interface_.registerHandle(state_handle_r_gripper_r_finger);
  jnt_state_interface_.registerHandle(state_handle_r_gripper_r_finger_tip);
  jnt_state_interface_.registerHandle(state_handle_r_gripper_l_finger_tip);
  jnt_state_interface_.registerHandle(state_handle_r_gripper_motor_screw);
  jnt_state_interface_.registerHandle(state_handle_r_gripper_motor_slider);
  jnt_state_interface_.registerHandle(state_handle_l_upper_arm_roll);
  jnt_state_interface_.registerHandle(state_handle_l_shoulder_pan);
  jnt_state_interface_.registerHandle(state_handle_l_shoulder_lift);
  jnt_state_interface_.registerHandle(state_handle_l_forearm_roll);
  jnt_state_interface_.registerHandle(state_handle_l_elbow_flex);
  jnt_state_interface_.registerHandle(state_handle_l_wrist_flex);
  jnt_state_interface_.registerHandle(state_handle_l_wrist_roll);
  jnt_state_interface_.registerHandle(state_handle_l_gripper);
  jnt_state_interface_.registerHandle(state_handle_l_gripper_l_finger);
  jnt_state_interface_.registerHandle(state_handle_l_gripper_r_finger);
  jnt_state_interface_.registerHandle(state_handle_l_gripper_r_finger_tip);
  jnt_state_interface_.registerHandle(state_handle_l_gripper_l_finger_tip);
  jnt_state_interface_.registerHandle(state_handle_l_gripper_motor_screw);
  jnt_state_interface_.registerHandle(state_handle_l_gripper_motor_slider);
  registerInterface(&jnt_state_interface_);

  // register the joint position interface
  hardware_interface::JointHandle pos_handle_l_upper_arm_roll(jnt_state_interface_.getHandle("l_upper_arm_roll_joint"), &cmd_[31]);
  hardware_interface::JointHandle pos_handle_l_shoulder_pan  (jnt_state_interface_.getHandle("l_shoulder_pan_joint"),   &cmd_[32]);
  hardware_interface::JointHandle pos_handle_l_shoulder_lift (jnt_state_interface_.getHandle("l_shoulder_lift_joint"),  &cmd_[33]);
  hardware_interface::JointHandle pos_handle_l_elbow_flex    (jnt_state_interface_.getHandle("l_elbow_flex_joint"),     &cmd_[34]);
  hardware_interface::JointHandle pos_handle_l_forearm_roll  (jnt_state_interface_.getHandle("l_forearm_roll_joint"),   &cmd_[35]);
  hardware_interface::JointHandle pos_handle_l_wrist_flex    (jnt_state_interface_.getHandle("l_wrist_flex_joint"),     &cmd_[36]);
  hardware_interface::JointHandle pos_handle_l_wrist_roll    (jnt_state_interface_.getHandle("l_wrist_roll_joint"),     &cmd_[37]);
  hardware_interface::JointHandle pos_handle_r_upper_arm_roll(jnt_state_interface_.getHandle("r_upper_arm_roll_joint"), &cmd_[17]);
  hardware_interface::JointHandle pos_handle_r_shoulder_pan  (jnt_state_interface_.getHandle("r_shoulder_pan_joint"),   &cmd_[18]);
  hardware_interface::JointHandle pos_handle_r_shoulder_lift (jnt_state_interface_.getHandle("r_shoulder_lift_joint"),  &cmd_[19]);
  hardware_interface::JointHandle pos_handle_r_elbow_flex    (jnt_state_interface_.getHandle("r_elbow_flex_joint"),     &cmd_[20]);
  hardware_interface::JointHandle pos_handle_r_forearm_roll  (jnt_state_interface_.getHandle("r_forearm_roll_joint"),   &cmd_[21]);
  hardware_interface::JointHandle pos_handle_r_wrist_flex    (jnt_state_interface_.getHandle("r_wrist_flex_joint"),     &cmd_[22]);
  hardware_interface::JointHandle pos_handle_r_wrist_roll    (jnt_state_interface_.getHandle("r_wrist_roll_joint"),     &cmd_[23]);
  hardware_interface::JointHandle pos_handle_head_pan        (jnt_state_interface_.getHandle("head_pan_joint"),         &cmd_[14]);
  hardware_interface::JointHandle pos_handle_head_tilt       (jnt_state_interface_.getHandle("head_tilt_joint"),        &cmd_[15]);
  hardware_interface::JointHandle pos_handle_torso_lift      (jnt_state_interface_.getHandle("torso_lift_joint"),       &cmd_[12]);

  jnt_pos_interface_.registerHandle(pos_handle_l_upper_arm_roll);
  jnt_pos_interface_.registerHandle(pos_handle_l_shoulder_pan);
  jnt_pos_interface_.registerHandle(pos_handle_l_shoulder_lift);
  jnt_pos_interface_.registerHandle(pos_handle_l_elbow_flex);
  jnt_pos_interface_.registerHandle(pos_handle_l_forearm_roll);
  jnt_pos_interface_.registerHandle(pos_handle_l_wrist_flex);
  jnt_pos_interface_.registerHandle(pos_handle_l_wrist_roll);
  jnt_pos_interface_.registerHandle(pos_handle_r_upper_arm_roll);
  jnt_pos_interface_.registerHandle(pos_handle_r_shoulder_pan);
  jnt_pos_interface_.registerHandle(pos_handle_r_shoulder_lift);
  jnt_pos_interface_.registerHandle(pos_handle_r_elbow_flex);
  jnt_pos_interface_.registerHandle(pos_handle_r_forearm_roll);
  jnt_pos_interface_.registerHandle(pos_handle_r_wrist_flex);
  jnt_pos_interface_.registerHandle(pos_handle_r_wrist_roll);
  jnt_pos_interface_.registerHandle(pos_handle_head_pan);
  jnt_pos_interface_.registerHandle(pos_handle_head_tilt);
  jnt_pos_interface_.registerHandle(pos_handle_torso_lift);
  registerInterface(&jnt_pos_interface_);

  // register the joint limits interface
  JointLimits joint_limits;
  SoftJointLimits soft_joint_limits;
  getJointLimits("l_shoulder_pan_joint",   nh_, joint_limits);
  PositionJointSoftLimitsHandle joint_limit_l_shoulder_pan  (pos_handle_l_shoulder_pan,   joint_limits, soft_joint_limits);
  getJointLimits("l_shoulder_lift_joint",  nh_, joint_limits);
  PositionJointSoftLimitsHandle joint_limit_l_shoulder_lift (pos_handle_l_shoulder_lift,  joint_limits, soft_joint_limits);
  getJointLimits("l_upper_arm_roll_joint", nh_, joint_limits);
  PositionJointSoftLimitsHandle joint_limit_l_upper_arm_roll(pos_handle_l_upper_arm_roll, joint_limits, soft_joint_limits);
  getJointLimits("l_elbow_flex_joint",     nh_, joint_limits);
  PositionJointSoftLimitsHandle joint_limit_l_elbow_flex    (pos_handle_l_elbow_flex,     joint_limits, soft_joint_limits);
  getJointLimits("l_forearm_roll_joint",   nh_, joint_limits);
  PositionJointSoftLimitsHandle joint_limit_l_forearm_roll  (pos_handle_l_forearm_roll,   joint_limits, soft_joint_limits);
  getJointLimits("l_wrist_flex_joint",     nh_, joint_limits);
  PositionJointSoftLimitsHandle joint_limit_l_wrist_flex    (pos_handle_l_wrist_flex,     joint_limits, soft_joint_limits);
  getJointLimits("l_wrist_roll_joint",     nh_, joint_limits);
  PositionJointSoftLimitsHandle joint_limit_l_wrist_roll    (pos_handle_l_wrist_roll,     joint_limits, soft_joint_limits);
  getJointLimits("r_shoulder_pan_joint",   nh_, joint_limits);
  PositionJointSoftLimitsHandle joint_limit_r_shoulder_pan  (pos_handle_r_shoulder_pan,   joint_limits, soft_joint_limits);
  getJointLimits("r_shoulder_lift_joint",  nh_, joint_limits);
  PositionJointSoftLimitsHandle joint_limit_r_shoulder_lift (pos_handle_r_shoulder_lift,  joint_limits, soft_joint_limits);
  getJointLimits("r_upper_arm_roll_joint", nh_, joint_limits);
  PositionJointSoftLimitsHandle joint_limit_r_upper_arm_roll(pos_handle_r_upper_arm_roll, joint_limits, soft_joint_limits);
  getJointLimits("r_elbow_flex_joint",     nh_, joint_limits);
  PositionJointSoftLimitsHandle joint_limit_r_elbow_flex    (pos_handle_r_elbow_flex,     joint_limits, soft_joint_limits);
  getJointLimits("r_forearm_roll_joint",   nh_, joint_limits);
  PositionJointSoftLimitsHandle joint_limit_r_forearm_roll  (pos_handle_r_forearm_roll,   joint_limits, soft_joint_limits);
  getJointLimits("r_wrist_flex_joint",     nh_, joint_limits);
  PositionJointSoftLimitsHandle joint_limit_r_wrist_flex    (pos_handle_r_wrist_flex,     joint_limits, soft_joint_limits);
  getJointLimits("r_wrist_roll_joint",     nh_, joint_limits);
  PositionJointSoftLimitsHandle joint_limit_r_wrist_roll    (pos_handle_r_wrist_roll,     joint_limits, soft_joint_limits);
  getJointLimits("head_pan_joint",         nh_, joint_limits);
  PositionJointSoftLimitsHandle joint_limit_head_pan        (pos_handle_head_pan,         joint_limits, soft_joint_limits);
  getJointLimits("head_tilt_joint",        nh_, joint_limits);
  PositionJointSoftLimitsHandle joint_limit_head_tilt       (pos_handle_head_tilt,        joint_limits, soft_joint_limits);
  getJointLimits("torso_lift_joint",       nh_, joint_limits);
  PositionJointSoftLimitsHandle joint_limit_torso_lift      (pos_handle_torso_lift,       joint_limits, soft_joint_limits);
  position_joint_soft_limits_interface_.registerHandle(joint_limit_l_shoulder_pan);
  position_joint_soft_limits_interface_.registerHandle(joint_limit_l_shoulder_lift);
  position_joint_soft_limits_interface_.registerHandle(joint_limit_l_upper_arm_roll);
  position_joint_soft_limits_interface_.registerHandle(joint_limit_l_elbow_flex);
  position_joint_soft_limits_interface_.registerHandle(joint_limit_l_forearm_roll);
  position_joint_soft_limits_interface_.registerHandle(joint_limit_l_wrist_flex);
  position_joint_soft_limits_interface_.registerHandle(joint_limit_l_wrist_roll);
  position_joint_soft_limits_interface_.registerHandle(joint_limit_r_shoulder_pan);
  position_joint_soft_limits_interface_.registerHandle(joint_limit_r_shoulder_lift);
  position_joint_soft_limits_interface_.registerHandle(joint_limit_r_upper_arm_roll);
  position_joint_soft_limits_interface_.registerHandle(joint_limit_r_elbow_flex);
  position_joint_soft_limits_interface_.registerHandle(joint_limit_r_forearm_roll);
  position_joint_soft_limits_interface_.registerHandle(joint_limit_r_wrist_flex);
  position_joint_soft_limits_interface_.registerHandle(joint_limit_r_wrist_roll);
  position_joint_soft_limits_interface_.registerHandle(joint_limit_head_pan);
  position_joint_soft_limits_interface_.registerHandle(joint_limit_head_tilt);
  position_joint_soft_limits_interface_.registerHandle(joint_limit_torso_lift);
  registerInterface(&position_joint_soft_limits_interface_);
}

void SigversePR2::jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  for(int i = 0; i < msg->name.size(); i++)
  {
    if(msg->name[i]      == "fl_caster_rotation_joint"){     pos_[0]  = msg->position[i]; }
    else if(msg->name[i] == "fl_caster_l_wheel_joint"){      pos_[1]  = msg->position[i]; }
    else if(msg->name[i] == "fl_caster_r_wheel_joint"){      pos_[2]  = msg->position[i]; }
    else if(msg->name[i] == "fr_caster_rotation_joint"){     pos_[3]  = msg->position[i]; }
    else if(msg->name[i] == "fr_caster_l_wheel_joint"){      pos_[4]  = msg->position[i]; }
    else if(msg->name[i] == "fr_caster_r_wheel_joint"){      pos_[5]  = msg->position[i]; }
    else if(msg->name[i] == "bl_caster_rotation_joint"){     pos_[6]  = msg->position[i]; }
    else if(msg->name[i] == "bl_caster_l_wheel_joint"){      pos_[7]  = msg->position[i]; }
    else if(msg->name[i] == "bl_caster_r_wheel_joint"){      pos_[8]  = msg->position[i]; }
    else if(msg->name[i] == "br_caster_rotation_joint"){     pos_[9]  = msg->position[i]; }
    else if(msg->name[i] == "br_caster_l_wheel_joint"){      pos_[10] = msg->position[i]; }
    else if(msg->name[i] == "br_caster_r_wheel_joint"){      pos_[11] = msg->position[i]; }
    else if(msg->name[i] == "torso_lift_joint"){             pos_[12] = msg->position[i]; }
    else if(msg->name[i] == "torso_lift_motor_screw_joint"){ pos_[13] = msg->position[i]; }
    else if(msg->name[i] == "head_pan_joint"){               pos_[14] = msg->position[i]; }
    else if(msg->name[i] == "head_tilt_joint"){              pos_[15] = msg->position[i]; }
    else if(msg->name[i] == "laser_tilt_mount_joint"){       pos_[16] = msg->position[i]; }
    else if(msg->name[i] == "r_upper_arm_roll_joint"){       pos_[17] = msg->position[i]; }
    else if(msg->name[i] == "r_shoulder_pan_joint"){         pos_[18] = msg->position[i]; }
    else if(msg->name[i] == "r_shoulder_lift_joint"){        pos_[19] = msg->position[i]; }
    else if(msg->name[i] == "r_forearm_roll_joint"){         pos_[20] = msg->position[i]; }
    else if(msg->name[i] == "r_elbow_flex_joint"){           pos_[21] = msg->position[i]; }
    else if(msg->name[i] == "r_wrist_flex_joint"){           pos_[22] = msg->position[i]; }
    else if(msg->name[i] == "r_wrist_roll_joint"){           pos_[23] = msg->position[i]; }
    else if(msg->name[i] == "r_gripper_joint"){              pos_[24] = msg->position[i]; }
    else if(msg->name[i] == "r_gripper_l_finger_joint"){     pos_[25] = msg->position[i]; }
    else if(msg->name[i] == "r_gripper_r_finger_joint"){     pos_[26] = msg->position[i]; }
    else if(msg->name[i] == "r_gripper_r_finger_tip_joint"){ pos_[27] = msg->position[i]; }
    else if(msg->name[i] == "r_gripper_l_finger_tip_joint"){ pos_[28] = msg->position[i]; }
    else if(msg->name[i] == "r_gripper_motor_screw_joint"){  pos_[29] = msg->position[i]; }
    else if(msg->name[i] == "r_gripper_motor_slider_joint"){ pos_[30] = msg->position[i]; }
    else if(msg->name[i] == "l_upper_arm_roll_joint"){       pos_[31] = msg->position[i]; }
    else if(msg->name[i] == "l_shoulder_pan_joint"){         pos_[32] = msg->position[i]; }
    else if(msg->name[i] == "l_shoulder_lift_joint"){        pos_[33] = msg->position[i]; }
    else if(msg->name[i] == "l_forearm_roll_joint"){         pos_[34] = msg->position[i]; }
    else if(msg->name[i] == "l_elbow_flex_joint"){           pos_[35] = msg->position[i]; }
    else if(msg->name[i] == "l_wrist_flex_joint"){           pos_[36] = msg->position[i]; }
    else if(msg->name[i] == "l_wrist_roll_joint"){           pos_[37] = msg->position[i]; }
    else if(msg->name[i] == "l_gripper_joint"){              pos_[38] = msg->position[i]; }
    else if(msg->name[i] == "l_gripper_l_finger_joint"){     pos_[39] = msg->position[i]; }
    else if(msg->name[i] == "l_gripper_r_finger_joint"){     pos_[40] = msg->position[i]; }
    else if(msg->name[i] == "l_gripper_r_finger_tip_joint"){ pos_[41] = msg->position[i]; }
    else if(msg->name[i] == "l_gripper_l_finger_tip_joint"){ pos_[42] = msg->position[i]; }
    else if(msg->name[i] == "l_gripper_motor_screw_joint"){  pos_[43] = msg->position[i]; }
    else if(msg->name[i] == "l_gripper_motor_slider_joint"){ pos_[44] = msg->position[i]; }
  }

  if(is_cmd_value_initialized == false){
    for(int i = 0; i < msg->name.size(); i++)
    {
      if(msg->name[i]      == "fl_caster_rotation_joint"){     cmd_[0]  = msg->position[i]; }
      else if(msg->name[i] == "fl_caster_l_wheel_joint"){      cmd_[1]  = msg->position[i]; }
      else if(msg->name[i] == "fl_caster_r_wheel_joint"){      cmd_[2]  = msg->position[i]; }
      else if(msg->name[i] == "fr_caster_rotation_joint"){     cmd_[3]  = msg->position[i]; }
      else if(msg->name[i] == "fr_caster_l_wheel_joint"){      cmd_[4]  = msg->position[i]; }
      else if(msg->name[i] == "fr_caster_r_wheel_joint"){      cmd_[5]  = msg->position[i]; }
      else if(msg->name[i] == "bl_caster_rotation_joint"){     cmd_[6]  = msg->position[i]; }
      else if(msg->name[i] == "bl_caster_l_wheel_joint"){      cmd_[7]  = msg->position[i]; }
      else if(msg->name[i] == "bl_caster_r_wheel_joint"){      cmd_[8]  = msg->position[i]; }
      else if(msg->name[i] == "br_caster_rotation_joint"){     cmd_[9]  = msg->position[i]; }
      else if(msg->name[i] == "br_caster_l_wheel_joint"){      cmd_[10] = msg->position[i]; }
      else if(msg->name[i] == "br_caster_r_wheel_joint"){      cmd_[11] = msg->position[i]; }
      else if(msg->name[i] == "torso_lift_joint"){             cmd_[12] = msg->position[i]; }
      else if(msg->name[i] == "torso_lift_motor_screw_joint"){ cmd_[13] = msg->position[i]; }
      else if(msg->name[i] == "head_pan_joint"){               cmd_[14] = msg->position[i]; }
      else if(msg->name[i] == "head_tilt_joint"){              cmd_[15] = msg->position[i]; }
      else if(msg->name[i] == "laser_tilt_mount_joint"){       cmd_[16] = msg->position[i]; }
      else if(msg->name[i] == "r_upper_arm_roll_joint"){       cmd_[17] = msg->position[i]; }
      else if(msg->name[i] == "r_shoulder_pan_joint"){         cmd_[18] = msg->position[i]; }
      else if(msg->name[i] == "r_shoulder_lift_joint"){        cmd_[19] = msg->position[i]; }
      else if(msg->name[i] == "r_forearm_roll_joint"){         cmd_[20] = msg->position[i]; }
      else if(msg->name[i] == "r_elbow_flex_joint"){           cmd_[21] = msg->position[i]; }
      else if(msg->name[i] == "r_wrist_flex_joint"){           cmd_[22] = msg->position[i]; }
      else if(msg->name[i] == "r_wrist_roll_joint"){           cmd_[23] = msg->position[i]; }
      else if(msg->name[i] == "r_gripper_joint"){              cmd_[24] = msg->position[i]; }
      else if(msg->name[i] == "r_gripper_l_finger_joint"){     cmd_[25] = msg->position[i]; }
      else if(msg->name[i] == "r_gripper_r_finger_joint"){     cmd_[26] = msg->position[i]; }
      else if(msg->name[i] == "r_gripper_r_finger_tip_joint"){ cmd_[27] = msg->position[i]; }
      else if(msg->name[i] == "r_gripper_l_finger_tip_joint"){ cmd_[28] = msg->position[i]; }
      else if(msg->name[i] == "r_gripper_motor_screw_joint"){  cmd_[29] = msg->position[i]; }
      else if(msg->name[i] == "r_gripper_motor_slider_joint"){ cmd_[30] = msg->position[i]; }
      else if(msg->name[i] == "l_upper_arm_roll_joint"){       cmd_[31] = msg->position[i]; }
      else if(msg->name[i] == "l_shoulder_pan_joint"){         cmd_[32] = msg->position[i]; }
      else if(msg->name[i] == "l_shoulder_lift_joint"){        cmd_[33] = msg->position[i]; }
      else if(msg->name[i] == "l_forearm_roll_joint"){         cmd_[34] = msg->position[i]; }
      else if(msg->name[i] == "l_elbow_flex_joint"){           cmd_[35] = msg->position[i]; }
      else if(msg->name[i] == "l_wrist_flex_joint"){           cmd_[36] = msg->position[i]; }
      else if(msg->name[i] == "l_wrist_roll_joint"){           cmd_[37] = msg->position[i]; }
      else if(msg->name[i] == "l_gripper_joint"){              cmd_[38] = msg->position[i]; }
      else if(msg->name[i] == "l_gripper_l_finger_joint"){     cmd_[39] = msg->position[i]; }
      else if(msg->name[i] == "l_gripper_r_finger_joint"){     cmd_[40] = msg->position[i]; }
      else if(msg->name[i] == "l_gripper_r_finger_tip_joint"){ cmd_[41] = msg->position[i]; }
      else if(msg->name[i] == "l_gripper_l_finger_tip_joint"){ cmd_[42] = msg->position[i]; }
      else if(msg->name[i] == "l_gripper_motor_screw_joint"){  cmd_[43] = msg->position[i]; }
      else if(msg->name[i] == "l_gripper_motor_slider_joint"){ cmd_[44] = msg->position[i]; }
    }
    is_cmd_value_initialized = true;
  }
}

void SigversePR2::read(ros::Time time, ros::Duration period){}

void SigversePR2::write(ros::Time time, ros::Duration period)
{
  if(is_cmd_value_initialized == false){ return; }
  publishTargetLeftArmPosition();
  publishTargetRightArmPosition();
  publishTargetHeadPosition();
  publishTargetTorsoPosition();
  position_joint_soft_limits_interface_.enforceLimits(period);
}

void SigversePR2::publishTargetLeftArmPosition()
{
  trajectory_msgs::JointTrajectory traj;

  traj.joint_names.push_back("l_upper_arm_roll_joint");
  traj.joint_names.push_back("l_shoulder_pan_joint");
  traj.joint_names.push_back("l_shoulder_lift_joint");
  traj.joint_names.push_back("l_elbow_flex_joint");
  traj.joint_names.push_back("l_forearm_roll_joint");
  traj.joint_names.push_back("l_wrist_flex_joint");
  traj.joint_names.push_back("l_wrist_roll_joint");

  traj.points.resize(1);
  traj.points[0].positions.resize(7);
  traj.points[0].positions[0] = cmd_[31];
  traj.points[0].positions[1] = cmd_[32];
  traj.points[0].positions[2] = cmd_[33];
  traj.points[0].positions[3] = cmd_[34];
  traj.points[0].positions[4] = cmd_[35];
  traj.points[0].positions[3] = cmd_[36];
  traj.points[0].positions[4] = cmd_[37];

  traj.points[0].time_from_start = ros::Duration(0.0);

  pub_left_arm_trajectory_.publish(traj);
}

void SigversePR2::publishTargetRightArmPosition()
{
    trajectory_msgs::JointTrajectory traj;

    traj.joint_names.push_back("r_upper_arm_roll_joint");
    traj.joint_names.push_back("r_shoulder_pan_joint");
    traj.joint_names.push_back("r_shoulder_lift_joint");
    traj.joint_names.push_back("r_elbow_flex_joint");
    traj.joint_names.push_back("r_forearm_roll_joint");
    traj.joint_names.push_back("r_wrist_flex_joint");
    traj.joint_names.push_back("r_wrist_roll_joint");

    traj.points.resize(1);
    traj.points[0].positions.resize(7);
    traj.points[0].positions[0] = cmd_[17];
    traj.points[0].positions[1] = cmd_[18];
    traj.points[0].positions[2] = cmd_[19];
    traj.points[0].positions[3] = cmd_[20];
    traj.points[0].positions[4] = cmd_[21];
    traj.points[0].positions[3] = cmd_[22];
    traj.points[0].positions[4] = cmd_[23];

    traj.points[0].time_from_start = ros::Duration(0.0);

    pub_right_arm_trajectory_.publish(traj);
}

void SigversePR2::publishTargetHeadPosition()
{
  trajectory_msgs::JointTrajectory traj;
  traj.joint_names.push_back("head_pan_joint");
  traj.joint_names.push_back("head_tilt_joint");

  traj.points.resize(1);
  traj.points[0].positions.resize(2);
  traj.points[0].positions[0] = cmd_[14];
  traj.points[0].positions[1] = cmd_[15];

  traj.points[0].time_from_start = ros::Duration(0.0);

  pub_head_trajectory_.publish(traj);
}

void SigversePR2::publishTargetTorsoPosition()
{
    trajectory_msgs::JointTrajectory traj;
    traj.joint_names.push_back("torso_lift_joint");

    traj.points.resize(1);
    traj.points[0].positions.resize(1);
    traj.points[0].positions[0] = cmd_[12];

    traj.points[0].time_from_start = ros::Duration(0.0);

    pub_torso_trajectory_.publish(traj);
}
