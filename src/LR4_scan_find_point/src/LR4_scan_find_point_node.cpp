#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <vector>
static sensor_msgs::LaserScan scan;
bool flag = false;
float threashold = 0.1;

int delta =10;
struct UniquePoint{
  int index;
  std::vector<float> hist;
};

std::vector<float> calcHist(sensor_msgs::LaserScan scan, int start, int finish,int bins){
  float oneBins = scan.range_max/bins;
  std::vector<float> hist;
  for (int var = 0; var < bins; ++var) {
    hist.push_back(0);
  }
  for (int i = start; i<=finish;i++){
    int index = scan.ranges.at(i)/oneBins;
    ROS_INFO("++ to hist.at(%d)",index);
    if(index<0){
      index=0;
    }else if (index>=hist.size()) {
      index=hist.size()-1;
    }
    hist.at(index)++;
  }
  for (int i = 0;i<hist.size();i++) {
    hist.at(i)=hist.at(i)/hist.size();
  }
  return hist;
}

void callbackScan(const sensor_msgs::LaserScan &data){
  scan = data;
  flag = true;
  ROS_INFO("updatePacket size scan %d",scan.ranges.size());
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "LR4_scan_find_point_node");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/scan",1000,callbackScan);

  ros::Rate r(10);

  while (ros::ok()) {
    if(flag){
      std::vector<UniquePoint> pointsStructur;
      std::vector <double> crit;
      for(int i = 1; i < scan.ranges.size()-1;i++){
        float preRange, range, postRange;
        range = scan.ranges.at(i);
        preRange = scan.ranges.at(i-1);
        postRange = scan.ranges.at(i+1);
        double c = abs(abs(range - preRange)+abs(postRange - range) - 2*abs(postRange - preRange));
        //ROS_INFO("crit %f",c);
        crit.push_back(c);
      }

      //ROS_INFO("Size scan %d",scan.ranges.size());
      //ROS_INFO("Size crit %d",crit.size());

      for (int i = 0;i<crit.size();i++) {
        UniquePoint uniquePoint;
        if(crit.at(i)> threashold){
          uniquePoint.index=i+1;
          ROS_INFO("SCAN_INFO index %d range %f crit %f max %f",uniquePoint.index,scan.ranges.at(uniquePoint.index), crit.at(i),scan.range_max );
          int deltaR=delta;
          int deltaL=delta;
          if (uniquePoint.index-deltaL<0){
            int err = uniquePoint.index-deltaL;
            deltaR=+((-1)*err);
            deltaL=0;
          }if(uniquePoint.index+deltaR>=scan.ranges.size()){
            int err = scan.ranges.size()-uniquePoint.index+deltaR;
            deltaL=+err;
            deltaR=scan.ranges.size()-1-uniquePoint.index;

          }
          ROS_INFO("input index %d   deltaR %d    deltaL %d", uniquePoint.index,deltaR,deltaL);
          uniquePoint.hist=calcHist(scan,uniquePoint.index-deltaL,uniquePoint.index+deltaR,50);
          pointsStructur.push_back(uniquePoint);
        }
      }
      ROS_INFO("uniques Points &d", pointsStructur.size());
      flag=false;
    }
    ros::spinOnce();
    r.sleep();
  }
}
