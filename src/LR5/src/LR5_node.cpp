#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud.h>

static sensor_msgs::LaserScan scan;
static nav_msgs::Odometry odom;
static ros::Time current_time;
static geometry_msgs::TransformStamped odom2base_link;
static geometry_msgs::TransformStamped baselink2laser;
static sensor_msgs::PointCloud pointCloud;
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

void transformOdom2Baselink(){
  odom2base_link.header.stamp = current_time;
  odom2base_link.header.frame_id = "laser";
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

void  transformBaselink2Laser(){
  baselink2laser.header.stamp=current_time;
  baselink2laser.header.frame_id="base_link";
  baselink2laser.child_frame_id="odom";
  baselink2laser.transform.translation.x=0;
  baselink2laser.transform.translation.y=0;
  baselink2laser.transform.translation.z=0;
  geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw(0);
  baselink2laser.transform.rotation = q;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "LR5_node");
  ros::NodeHandle nh;
  ros::Subscriber subScan = nh.subscribe("/scan",1000,callbackLaser);
  ros::Subscriber subOdom = nh.subscribe("/odom",1000,callbackOdom);
  tf::TransformBroadcaster odom2baseLinkBroadcast;
  tf::TransformBroadcaster baselink2laserBroadcast;
  ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud>("/point_cloud_tf",1000);

  ros::Rate r(10.0);
  pointCloud.header.frame_id="laser";

  while(ros::ok()){
    if (flag){
      pointCloud.points.clear();
      pointCloud.channels.clear();
      transformOdom2Baselink();
      transformBaselink2Laser();
      odom2baseLinkBroadcast.sendTransform(odom2base_link);
      baselink2laserBroadcast.sendTransform(baselink2laser);

      sensor_msgs::ChannelFloat32 channel;

      for (int i=0;i < scan.ranges.size();i++) {
        geometry_msgs::Point32 point;
        point.x=scan.ranges.at(i)*sin(i*scan.angle_increment+scan.angle_min);
        point.y=scan.ranges.at(i)*cos(i*scan.angle_increment+scan.angle_min);
        point.z=0;
        pointCloud.header.stamp=current_time;
        pointCloud.points.push_back(point);
        channel.name = "channel 1";
        channel.values.push_back(100);
      }
      pointCloud.channels.push_back(channel);
    }
    pub.publish(pointCloud);
    ros::spinOnce();
    r.sleep();
  }
}
