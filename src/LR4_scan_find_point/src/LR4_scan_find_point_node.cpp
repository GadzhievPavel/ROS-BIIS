#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <vector>
#include <visualization_msgs/MarkerArray.h>

static sensor_msgs::LaserScan scan;
static bool flag = false;
static double threashold = 0.1;

static int delta =10;
struct UniquePoint{
  int index;
  std::vector<double> hist;
  float x,y;
};
static std::vector<UniquePoint> pointsStructur;

std::vector<double> calcHist(sensor_msgs::LaserScan scan, int start, int finish,int bins){
  float oneBins = scan.range_max/bins;
  std::vector<double> hist;
  for (int var = 0; var < bins; ++var) {
    hist.push_back(0);
  }
  for (int i = start; i<=finish;i++){
    int index = scan.ranges.at(i)/oneBins;
    //ROS_INFO("++ to hist.at(%d)",index);
    if(index<0){
      index=0;
    }else if (index>=hist.size()) {
      index=hist.size()-1;
    }
    hist.at(index)++;
  }
  for (int i = 0;i<hist.size();i++) {
    hist.at(i)=hist.at(i)/hist.size();
    //ROS_INFO("hist at %d == %f",i,hist.at(i));
  }
  return hist;
}

double calculateCorr(std::vector<double> H1, std::vector<double> H2){
  double middleH1=0;
  double middleH2=0;
  
  for (int i=0;i<H1.size();i++) {
    middleH1+=H1.at(i);
  }
  middleH1=middleH1/H1.size();
  //ROS_INFO("middleH1 %f",middleH1);
  for (int i=0;i<H2.size();i++) {
    middleH2+=H2.at(i);
  }
  middleH2=middleH2/H2.size();
  //ROS_INFO("middleH2 %f",middleH2);
  double sum =0;
  double sumQuad=0;
  for (int i=0;i<H1.size() && i<H2.size();i++) {
    sum+=(H1.at(i)-middleH1)*(H2.at(i)-middleH2);
    sumQuad+=pow(H1.at(i)-middleH1,2)*pow(H2.at(i)-middleH2,2);
    //ROS_INFO("sum: %f  sumQuad %f",sum,sumQuad);
  }
  return sum/sqrt(sumQuad);
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
  ros::Publisher pub = nh.advertise<visualization_msgs::MarkerArray>("/markers",1000);
  visualization_msgs::MarkerArray arrays;
  ros::Rate r(10);

  while (ros::ok()) {
    if(flag){
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
          //ROS_INFO("input index %d   deltaR %d    deltaL %d", uniquePoint.index,deltaR,deltaL);
          uniquePoint.hist=calcHist(scan,uniquePoint.index-deltaL,uniquePoint.index+deltaR,50);
          uniquePoint.x=scan.ranges.at(uniquePoint.index)*sin(uniquePoint.index*scan.angle_increment+scan.angle_min);
          uniquePoint.y=scan.ranges.at(uniquePoint.index)*cos(uniquePoint.index*scan.angle_increment+scan.angle_min);
          pointsStructur.push_back(uniquePoint);
          ROS_INFO("index point %d, x:%f y:%f",uniquePoint.index,uniquePoint.x,uniquePoint.y);
        }
      }
      for (int i = 0;i<pointsStructur.size();i++) {
        for (int j = i+1;j<pointsStructur.size();j++) {
          double corr = calculateCorr(pointsStructur.at(i).hist,pointsStructur.at(j).hist);
          ROS_INFO("correlation %f", corr);
          if(corr>threashold){
            pointsStructur.at(i).x=(pointsStructur.at(i).x+pointsStructur.at(j).x)/2;
            pointsStructur.at(i).y=(pointsStructur.at(i).y+pointsStructur.at(j).y)/2;
            pointsStructur.erase(pointsStructur.begin()+j);
            j--;
          }
        }
      }
      arrays.markers.clear();
      for (int i =0;i<pointsStructur.size();i++) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "odom";
        marker.header.stamp = ros::Time::now();
        marker.ns = "my_namespace";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x=pointsStructur.at(i).x;
        marker.pose.position.y=pointsStructur.at(i).y;
        marker.pose.position.z=0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 1;
        marker.scale.y = 0.1;
        marker.scale.z = 0.1;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        arrays.markers.push_back(marker);
      }
      ROS_INFO("size marker %d",arrays.markers.size());
      ROS_INFO("uniques Points %d", pointsStructur.size());
      flag=false;
    }
    pub.publish(arrays);
    ros::spinOnce();
    r.sleep();
  }
}
