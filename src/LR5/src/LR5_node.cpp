#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud.h>

static sensor_msgs::LaserScan scan;
static nav_msgs::Odometry odom;
static ros::Time current_time;
static bool flag = false;

void callbackLaser(const sensor_msgs::LaserScan &data){
  scan = data;
  ROS_INFO("Range %f, size %d",scan.range_max,scan.ranges.size());
  flag = true;
  current_time = ros::Time::now();
}

void callbackOdom(const nav_msgs::Odometry &data){
  odom = data;
}

void transformation(){

}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "LR5_node");
  ros::NodeHandle nh;
  ros::Subscriber subScan = nh.subscribe("/scan",1000,callbackLaser);
  ros::Subscriber subOdom = nh.subscribe("/odom",1000,callbackOdom);
  tf::StampedTransform transform;
  tf::TransformBroadcaster br;
 // ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud>("/point_cloud_tf",1000);
  ros::Rate r(10.0);
  while(ros::ok()){
    if (flag){

      tf::Quaternion q(odom.pose.pose.orientation.x,
                       odom.pose.pose.orientation.y,
                       odom.pose.pose.orientation.z,
                       odom.pose.pose.orientation.w);
      double yaw, pitch, roll;
      tf::Matrix3x3 mat(q);
      mat.getRPY(roll,pitch,yaw);
      transform.setOrigin(tf::Vector3(0.5,0.1,0.0));
      q = q.normalize();
      transform.setRotation(q);
      transform.frame_id_ ="/base_link";
      transform.child_frame_id_="/odom";

    }
    br.sendTransform(transform);
    //pub.publish(pointCloud);
    ros::spinOnce();
    r.sleep();
  }
}
