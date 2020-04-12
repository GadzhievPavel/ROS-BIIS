#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>

static sensor_msgs::PointCloud cloud;
sensor_msgs::PointCloud tempCloud;
nav_msgs::OccupancyGrid grid;
bool new_data;
static int height=600;
static int width=600;
int data [600][600];

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
  grid.header.frame_id ="base_link";
  grid.info.origin.position.x = -1 * grid.info.resolution * grid.info.width/2;
  grid.info.origin.position.y = -1 * grid.info.resolution * grid.info.height/2;
  grid.data.resize(grid.info.width * grid.info.height);

  ros::Rate r(10.0);
  while(ros::ok()){
    if(new_data){
      tempCloud=cloud;
      tempCloud.points.clear();

      for (int i = 0;i<width;i++) {
        for (int j = 0;j<height;j++) {
          data[i][j]=50;
        }
      }

      //ROS_INFO("Cloud size=%d",cloud.points.size());
      for (int i = 0;i< cloud.points.size();i++) {
        //ROS_INFO("Z: %f",cloud.points.at(i).z);
        if(cloud.points.at(i).z>0.25){
          int w = (cloud.points.at(i).x/grid.info.resolution)+300;
          int h = (cloud.points.at(i).y/grid.info.resolution)+300;
          //ROS_INFO("W:%f",cloud.points.at(i).x/grid.info.resolution);
          data[w][h]=100;
        }
      }

      double teta = asin(grid.info.resolution/height);
      ROS_INFO("angle tete:%f",teta);
      for (double angle = 0;angle<2*3.1415;angle=angle+(8*teta)) {
        ROS_INFO("cur angle:%f",angle);
          for (double R=1.0; R<300;R++ ){
                  int x = (R*sin(angle))+300;
                  int y = (R*cos(angle))+300;
                  ROS_INFO("cur_radius %f x:%d y:%d",R,x,y);
                  if(x>600 || y>600 || x<0 || y<0 || data[x][y]==100){
                    ROS_INFO("break;");
                    break;
                  }
                  data[x][y]=0;
                }
      }

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
