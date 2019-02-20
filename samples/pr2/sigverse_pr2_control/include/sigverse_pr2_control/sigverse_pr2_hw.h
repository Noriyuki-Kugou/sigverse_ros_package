#include <ros/ros.h>
#include <ros/package.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <hardware_interface/robot_hw.h>
#include <angles/angles.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <pr2_controllers_msgs/Pr2GripperCommand.h>
#include <map>
#include <string>
#include <vector>

using namespace hardware_interface;
using joint_limits_interface::JointLimits;
using joint_limits_interface::SoftJointLimits;
using joint_limits_interface::PositionJointSoftLimitsHandle;
using joint_limits_interface::PositionJointSoftLimitsInterface;

class SigversePR2 : public hardware_interface::RobotHW
{
public:
  SigversePR2();

  ros::NodeHandle nh_;
  ros::Publisher  pub_left_arm_trajectory_;
  ros::Publisher  pub_right_arm_trajectory_;
  ros::Publisher  pub_head_trajectory_;
  ros::Publisher  pub_torso_trajectory_;
  ros::Subscriber sub_joint_states_;

  ros::Time getTime() const { return ros::Time::now(); }
  ros::Duration getPeriod() const { return ros::Duration(0.01); }

  void read(ros::Time, ros::Duration);
  void write(ros::Time, ros::Duration);
  void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg);
  void publishTargetLeftArmPosition();
  void publishTargetRightArmPosition();
  void publishTargetHeadPosition();
  void publishTargetTorsoPosition();

  bool is_cmd_value_initialized;

protected:
  JointStateInterface              jnt_state_interface_;
  PositionJointInterface           jnt_pos_interface_;
  PositionJointSoftLimitsInterface position_joint_soft_limits_interface_;
  double cmd_[17];
  double pos_[17];
  double vel_[17];
  double eff_[17];
};
