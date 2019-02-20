#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <sigverse_pr2_control/sigverse_pr2_hw.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "sigverse_pr2_control");
  ros::NodeHandle nh;

  SigversePR2 sigverse_pr2;
  controller_manager::ControllerManager cm(&sigverse_pr2, nh);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  while(ros::ok())
  {
    ros::Time now = sigverse_pr2.getTime();
    ros::Duration dt = sigverse_pr2.getPeriod();

    sigverse_pr2.read(now, dt);
    cm.update(now, dt);

    sigverse_pr2.write(now, dt);
    dt.sleep();
  }
  spinner.stop();

  return 0;
}
