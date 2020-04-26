#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <tf/transform_broadcaster.h>
static sensor_msgs::PointCloud cloud;
static sensor_msgs::PointCloud globalCloud;
static sensor_msgs::LaserScan scan;
static bool flag = false;
static bool flagPCL = false;
static pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
static pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);

void callbackPoints(const sensor_msgs::LaserScan &data){
  scan = data;
  flag = true;
  cloud.points.clear();
  cloud.channels.clear();
  sensor_msgs::ChannelFloat32 channel;
  channel.name = "ch1";
  for(int i =0; i < scan.ranges.size(); i++){
    geometry_msgs::Point32 point;
    point.x = scan.ranges.at(i)*sin(i*scan.angle_increment+scan.angle_min);
    point.y = scan.ranges.at(i)*cos(i*scan.angle_increment+scan.angle_min);
    if(!(point.x == point.x) || !(point.y == point.y) || (point.x == INFINITY) || (point.x == -INFINITY)
       || (point.y == INFINITY) || (point.y == -INFINITY)){
      point.x=0;
      point.y=0;
    }
    cloud.points.push_back(point);
    channel.values.push_back(100);

  }
  cloud.header.frame_id="odom";
  cloud.header.stamp = scan.header.stamp;
  cloud.channels.push_back(channel);
  ROS_INFO("SIZE POINT %d", cloud.points.size());
}

bool findInCloud(geometry_msgs::Point p, sensor_msgs::PointCloud cloud){
  bool flag = true;
  for(int i = 0; i < cloud.points.size(); i++){
    if((p.x == cloud.points.at(i).x) || (p.y == cloud.points.at(i).y)){
      flag = false;
    }
  }
  return flag;
}
int main(int argc, char **argv)
{
  ros::init(argc, argv, "LR5_ICP_node");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/scan",1000,callbackPoints);
  ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud>("pcl_cloud",1000);
  tf::StampedTransform transform;
  tf::TransformBroadcaster br;
  ros::Rate r(10);


  globalCloud.header.frame_id = "pcl";
  while (ros::ok()) {

    if(flag){

      globalCloud.header.stamp=cloud.header.stamp;
      cloud_in->width = cloud.points.size();
      cloud_in->height = 1;
      cloud_in->is_dense =false;
      cloud_in->resize(cloud_in->width * cloud_in->height);
      //ROS_INFO("RESIZE %d",cloud_in->points.size());
      for (size_t i = 0; i < cloud.points.size(); ++i){
        cloud_in->points.at(i).x = cloud.points.at(i).x;
        cloud_in->points.at(i).y = cloud.points.at(i).y;
        cloud_in->points.at(i).z = cloud.points.at(i).z;
        //ROS_INFO("ITERATION %d", i);
      }
      ROS_INFO("PCL start");
      if(!cloud_out->empty()){
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(cloud_in);
        icp.setInputTarget(cloud_out);
        icp.setMaximumIterations(50);
        icp.setTransformationEpsilon(1e-9);
        icp.setMaxCorrespondenceDistance (0.2);
        pcl::PointCloud<pcl::PointXYZ> Final;
        icp.align(Final);
        std::cout << "has converged:" << icp.hasConverged() << " score: "
                  << icp.getFitnessScore() << std::endl;
        //std::cout << icp.getFinalTransformation() << std::endl;
        Eigen::Matrix4f mat = icp.getFinalTransformation();
        transformPointCloud(*cloud_in, *cloud_out, mat);
        std::cout << mat << std::endl;
        std::cout <<"X "<< mat(0,3) << " Y "<<mat(1,3)<<" Z "<<mat(2,3)<<std::endl;
        sensor_msgs::ChannelFloat32 channel;
        channel.name = "channel 1";

        for (int i = 0; i<cloud_out->points.size();i++){
          geometry_msgs::Point p;
          p.x=cloud_out->points.at(i).x;// + mat(0,3);
          p.y=cloud_out->points.at(i).y;// + mat(1,3);
          p.z=cloud_out->points.at(i).z;// + mat(2,3);
          //p.x = mat(0,0)* p.x + p.y * mat(0,1);
          //p.y = mat(1,0)* p.x + p.y * mat(1,1);
          if(findInCloud(p,globalCloud)){
            channel.values.push_back(100);
            globalCloud.channels.push_back(channel);
            globalCloud.points.push_back(cloud.points.at(i));
          }
        }

        for(int i =0; i<globalCloud.points.size();i++){
          globalCloud.points.at(i).x += mat(0,3);
          globalCloud.points.at(i).y += mat(1,3);
          globalCloud.points.at(i).z += mat(2,3);
          globalCloud.points.at(i).x = mat(0,0)* globalCloud.points.at(i).x + globalCloud.points.at(i).y * mat(0,1);
          globalCloud.points.at(i).y = mat(1,0)* globalCloud.points.at(i).x + globalCloud.points.at(i).y * mat(1,1);
        }

        tf::Quaternion q(mat(0,3),mat(1,3),mat(2,3),mat(3,3));
        double yaw, pitch, roll;
        tf::Matrix3x3 m(q);
        m.getRPY(roll,pitch,yaw);
        q = q.normalize();
        transform.setRotation(q);
        transform.frame_id_ ="/odom";
        transform.child_frame_id_="/pcl";

      }

      *cloud_out = *cloud_in;
      flagPCL = true;
      flag = false;
      br.sendTransform(transform);
      pub.publish(globalCloud);
    }
    ros::spinOnce();
    r.sleep();
  }
}
