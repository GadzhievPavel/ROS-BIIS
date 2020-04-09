#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>
static sensor_msgs::PointCloud cloud;
sensor_msgs::PointCloud tempCloud;
nav_msgs::OccupancyGrid grid;
bool new_data;
void callback(const sensor_msgs::PointCloud &data){
  cloud = data;
  new_data=true;
}

int main(int argc, char **argv){

  ros::init(argc, argv, "LR3_node");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("point_cloud",1000,callback);
  ros::Publisher pub = nh.advertise<nav_msgs::OccupancyGrid>("map",1000);
  grid.info.width =600;
  grid.info.height =600;
  grid.info.resolution = 0.25;
  grid.header.frame_id ="map";
  grid.info.origin.position.x = -1 * grid.info.resolution * grid.info.width/2;
  grid.info.origin.position.y = -1 * grid.info.resolution * grid.info.height/2;
  grid.data.resize(grid.info.width * grid.info.height);

  ros::Rate r(10.0);
  while(ros::ok()){
    if(new_data){
      tempCloud=cloud;
      tempCloud.points.clear();
      int height=600, width=600;
      int data[width][height];
      for (int i = 0;i<width;i++) {
        for (int j = 0;j<height;j++) {
          data[i][j]=50;
        }
      }

      ROS_INFO("Cloud size=%d",cloud.points.size());
      for (int i = 0;i< cloud.points.size();i++) {
          if( cloud.points.at(i).z>0.25){
            int w = (cloud.points.at(i).x/grid.info.resolution)+300;
            int h = (cloud.points.at(i).y/grid.info.resolution)+300;
           //ROS_INFO("W:%f",cloud.points.at(i).x/grid.info.resolution);
           //ROS_INFO("H:%f",cloud.points.at(i).y/grid.info.resolution);
           data[w][h]=100;
        }
      }

      ROS_INFO("SIZE grid %d",grid.data.size());
      ROS_INFO("SIZE data %d",height*width);
      for (int i=0;i<width;i++) {
        for (int j = 0;j<height;j++) {
          int index = j+i*grid.info.width;
          grid.data.at(index)=data[i][j];
        }
      }
      new_data=false;
      pub.publish(grid);
    }
    ros::spinOnce();
    r.sleep();
  }
}
