#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <vector>
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
      std::vector <float> crit;
      for (int i = 1;i< scan.ranges.size();i++) {
        float preRange, range, postRange;
        try {
          range = scan.ranges.at(i);
          preRange = scan.ranges.at(i-1);
          postRange = scan.ranges.at(i+1);

          crit.push_back(abs(abs(range - preRange)+abs(postRange - range) - 2*abs(postRange - preRange)));
        } catch (std::out_of_range) {
          ROS_INFO("WARN %d out of range",i);
        }
      }
      flag = false;
    }
  }

}
