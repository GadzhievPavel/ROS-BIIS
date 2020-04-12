#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
static sensor_msgs::LaserScan scan;
bool flag = false;
void callbackScan(const sensor_msgs::LaserScan &data){
  scan = data;
  flag = true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "LR4_process_scan");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/scan",1000,callbackScan);

  ros::Rate r(10);
  while (ros::ok()) {
    if(flag){
      for (;;) {

      }
      flag = false;
    }
  }

}
