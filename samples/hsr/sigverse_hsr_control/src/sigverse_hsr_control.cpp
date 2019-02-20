#include <ros/ros.h>
#include <controller_manager/controller_manager.h>
#include <sigverse_hsr_control/sigverse_hsr_hw.h>

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "sigverse_hsr_control");
  ros::NodeHandle nh;

  SigverseHSR sigverse_hsr;
  controller_manager::ControllerManager cm(&sigverse_hsr, nh);

  ros::AsyncSpinner spinner(1);
  spinner.start();

  while(ros::ok())
  {
    ros::Time now = sigverse_hsr.getTime();
    ros::Duration dt = sigverse_hsr.getPeriod();

    sigverse_hsr.read(now, dt);
    cm.update(now, dt);

    sigverse_hsr.write(now, dt);
    dt.sleep();
  }
  spinner.stop();

  return 0;
}
