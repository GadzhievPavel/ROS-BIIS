#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
static sensor_msgs::PointCloud cloud;
static bool flag = false;
static bool flagPCL = false;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);

void callbackPoints(const sensor_msgs::PointCloud &data){
  cloud = data;
  flag = true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "LR5_ICP_node");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/point_cloud_tf",1000,callbackPoints);

  ros::Rate r(10);

  while (ros::ok()) {

    if(flag){
      cloud_in->width = 16;
      cloud_in->height = 32;
      cloud_in->is_dense =false;
      cloud_in->resize(cloud_in->width * cloud_in->height);
      ROS_INFO("RESIZE %d",cloud_in->points.size());
      for (size_t i = 0; i < cloud.points.size(); ++i){
        cloud_in->points.at(i).x = cloud.points.at(i).x;
        cloud_in->points.at(i).y = cloud.points.at(i).y;
        cloud_in->points.at(i).z = cloud.points.at(i).z;
        ROS_INFO("ITERATION %d", i);
      }
      if(flagPCL){
        ROS_INFO("PCL start");
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(cloud_in);
        icp.setInputTarget(cloud_out);
        icp.setMaximumIterations(50);
        pcl::PointCloud<pcl::PointXYZ> Final;
        icp.align(Final);
      }
      *cloud_out = *cloud_in;
      flagPCL = true;
      flag = false;
    }
    ros::spinOnce();
    r.sleep();
  }
}
