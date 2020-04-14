#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>
static nav_msgs::Odometry odom,postOdom;
static bool odomFlag=false;
static ros::Time current_time;
static geometry_msgs::TransformStamped odom2base_link;
static geometry_msgs::TransformStamped baselink2laser;

void odomCallback(const nav_msgs::Odometry &data){
  odom=data;
  odomFlag=true;
  current_time = ros::Time::now();
}
static sensor_msgs::LaserScan scan;
void scanCallback(const sensor_msgs::LaserScan &data){
  ROS_INFO("scan");
  scan=data;
}

void transformOdom2Baselink(){
  odom2base_link.header.stamp=current_time;
  odom2base_link.header.frame_id="odom";
  odom2base_link.child_frame_id="base_link";
  odom2base_link.transform.translation.x=odom.pose.pose.position.x;
  odom2base_link.transform.translation.y=odom.pose.pose.position.y;
  odom2base_link.transform.translation.z=0;
  geometry_msgs::Quaternion q;
  q.w = odom.pose.pose.orientation.w;
  q.x = odom.pose.pose.orientation.x;
  q.y = odom.pose.pose.orientation.y;
  q.z = odom.pose.pose.orientation.z;
  odom2base_link.transform.rotation = q;
}

void transformBaselink2Laser(){
  baselink2laser.header.stamp=current_time;
  baselink2laser.header.frame_id="base_link";
  baselink2laser.child_frame_id="laser";
  baselink2laser.transform.translation.x=0;
  baselink2laser.transform.translation.y=0;
  baselink2laser.transform.translation.z=0;
  geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(0);
  q.y=-1*q.y;
  q.x=-1*q.x;
  baselink2laser.transform.rotation = q;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "LR4_node");
  ros::NodeHandle nh;
  ros::Subscriber subScan = nh.subscribe("/scan",1000,scanCallback);
  ros::Subscriber subOdom = nh.subscribe("/odom",1000,odomCallback);
  ros::Publisher pub = nh.advertise<nav_msgs::Odometry>("/postOdom",1000);
  tf::TransformBroadcaster odom2baseLinkBroadcast;
  tf::TransformBroadcaster baselink2laserBroadcast;

  ros::Rate r(10.0);
  while (ros::ok()) {
    if(odomFlag ==true){
      transformOdom2Baselink();
      transformBaselink2Laser();
      odom2baseLinkBroadcast.sendTransform(odom2base_link);
      baselink2laserBroadcast.sendTransform(baselink2laser);
      postOdom=odom;
      postOdom.header.frame_id="laser";
      postOdom.header.stamp = current_time;
      pub.publish(postOdom);
    }
    ros::spinOnce();
    r.sleep();
  }
}
