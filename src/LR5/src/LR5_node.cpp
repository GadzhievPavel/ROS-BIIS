#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud.h>

static sensor_msgs::LaserScan scan;
static nav_msgs::Odometry odom;
static ros::Time current_time;

static sensor_msgs::PointCloud pointCloud;
static bool flag = false;
static bool flagNAN = true;

void callbackLaser(const sensor_msgs::LaserScan &data){
  scan = data;
  ROS_INFO("Range %f, size %d",scan.range_max,scan.ranges.size());
  flag = true;
  current_time = ros::Time::now();
}

void callbackOdom(const nav_msgs::Odometry &data){
  odom = data;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "LR5_node");
  ros::NodeHandle nh;
  ros::Subscriber subScan = nh.subscribe("/scan",1000,callbackLaser);
  ros::Subscriber subOdom = nh.subscribe("/odom",1000,callbackOdom);
  tf::StampedTransform transform;
  tf::TransformBroadcaster br;
  ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud>("/point_cloud_tf",1000);
  tf::Quaternion q;
  ros::Rate r(10.0);
  pointCloud.header.frame_id="odom";
  while(ros::ok()){
    if (flag){
      pointCloud.points.clear();
      pointCloud.channels.clear();

      double yaw, pitch, roll;
      tf::Matrix3x3 mat(q);
      mat.getEulerYPR(yaw, pitch, roll);
      transform.setOrigin(tf::Vector3(0.5,0.1,0.0));
      q.setW(odom.pose.pose.orientation.w);
      q.setX(odom.pose.pose.orientation.x);
      q.setY(odom.pose.pose.orientation.y);
      q.setZ(odom.pose.pose.orientation.z);
      q = q.normalize();
      transform.setRotation(q);
      transform.frame_id_ ="/base_link";
      transform.child_frame_id_="/odom";

      sensor_msgs::ChannelFloat32 channel;
      for (int i=0;i < scan.ranges.size();i++) {
        geometry_msgs::Point32 point;
        point.x = scan.ranges.at(i)*sin(i*scan.angle_increment+scan.angle_min);
        point.y = scan.ranges.at(i)*cos(i*scan.angle_increment+scan.angle_min);
        point.z = 0;

        point.x += odom.pose.pose.position.x;
        point.y += odom.pose.pose.position.y;
        point.z += odom.pose.pose.position.z;

        point.x += cos(yaw)*point.x - sin(yaw)*point.y;
        point.y += sin(yaw)*point.x + cos(yaw)*point.y;

        ROS_INFO("POINT x%f y%f z%f",point.x,point.y,point.z);
        pointCloud.header.stamp=current_time;
        pointCloud.points.push_back(point);
        channel.name = "channel 1";
        channel.values.push_back(100);
      }
      pointCloud.channels.push_back(channel);
    }
    br.sendTransform(transform);
    pub.publish(pointCloud);
    ros::spinOnce();
    r.sleep();
  }
}
