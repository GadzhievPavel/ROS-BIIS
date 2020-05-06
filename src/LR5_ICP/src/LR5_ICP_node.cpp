#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <geometry_msgs/Point.h>
#include <pcl_ros/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

static sensor_msgs::PointCloud cloud;
static sensor_msgs::PointCloud globalCloud;
static sensor_msgs::LaserScan scan;
static bool flag = false;
static bool flagPCL = false;
static pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
static pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
static pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_global(new pcl::PointCloud<pcl::PointXYZ>);


void callbackPoints(const sensor_msgs::LaserScan &data){
    scan = data;
    flag = true;
    cloud.points.clear();
    cloud.channels.clear();
    sensor_msgs::ChannelFloat32 channel;
    channel.name = "ch1";
    for(int i =0; i < scan.ranges.size(); i++){
        geometry_msgs::Point32 point;
        point.x = scan.ranges.at(i)*cos(i*scan.angle_increment+scan.angle_min);
        point.y = scan.ranges.at(i)*sin(i*scan.angle_increment+scan.angle_min);
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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "LR5_ICP_node");
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/scan",10,callbackPoints);
    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud>("pcl_cloud",1000);
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("cloud_odom",10);
    tf::StampedTransform transform;
    tf::TransformBroadcaster br;
    ros::Rate r(10);

    Eigen::Matrix4f global_transform = Eigen::Matrix4f::Identity();
    sensor_msgs::ChannelFloat32 channel;
    channel.name = "channel 1";
    globalCloud.channels.push_back(channel);
    globalCloud.header.frame_id = "/odom";
    while (ros::ok()) {

        if(flag){

            globalCloud.header.stamp=cloud.header.stamp;
            cloud_in->is_dense =true;
            cloud_in->clear();
            for (size_t i = 0; i < cloud.points.size(); ++i){
                geometry_msgs::Point32 p = cloud.points.at(i);
                float dist = p.x * p.x + p.y * p.y;
                if(dist > 0.2) {
                    pcl::PointXYZ p_pcl;
                    p_pcl.x = p.x;
                    p_pcl.y = p.y;
                    p_pcl.z = p.z;
                    cloud_in->points.push_back(p_pcl);
                }
            }
            ROS_INFO("PCL start");
            if(!cloud_out->empty()){
                ROS_INFO("Cloud sizes: in %d out %d", cloud_in->points.size(), cloud_out->points.size());
                pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
                icp.setInputSource(cloud_in);
                icp.setInputTarget(cloud_out);
                icp.setMaximumIterations(2000);
                icp.setTransformationEpsilon(1e-9);
                icp.setMaxCorrespondenceDistance (0.35);

                pcl::PointCloud<pcl::PointXYZ> Final;
                icp.align(Final);
                std::cout << "has converged:" << icp.hasConverged() << " score: "
                          << icp.getFitnessScore() << std::endl;
                //std::cout << icp.getFinalTransformation() << std::endl;
                Eigen::Matrix4f mat = icp.getFinalTransformation();
                //Одометрия из сопоставления
                global_transform *= mat;
                transformPointCloud(*cloud_in, *cloud_out, global_transform);
                *cloud_global += *cloud_out;

                std::cout<<"local transform "<<std::endl;
                std::cout<<mat<<std::endl;
                nav_msgs::Odometry odom;
                odom.header.stamp = ros::Time::now();
                odom.header.frame_id = "/odom";
                //Параметры положения
                odom.pose.pose.position.x = global_transform(0,3);
                odom.pose.pose.position.y = global_transform(1,3);
                odom.pose.pose.position.z = 0.0;
                Eigen::Matrix3f rot_mat = global_transform.topLeftCorner(3,3);
                Eigen::Quaternionf e_q(rot_mat);
                odom.pose.pose.orientation.x = e_q.x();
                odom.pose.pose.orientation.y = e_q.y();
                odom.pose.pose.orientation.z = e_q.z();
                odom.pose.pose.orientation.w = e_q.w();
                odom.child_frame_id = "/pcl";

                //Публикуем сообщение с одометрией
                odom_pub.publish(odom);
                //-----------------------------
                globalCloud.channels.at(0).values.clear();
                globalCloud.points.clear();
                for (int i = 0; i<cloud_global->points.size();i++){
                    geometry_msgs::Point32 p;
                    p.x=cloud_global->points.at(i).x;
                    p.y=cloud_global->points.at(i).y;
                    p.z=cloud_global->points.at(i).z;
                    globalCloud.channels.at(0).values.push_back(100);
                    globalCloud.points.push_back(p);
                }
                tf::Quaternion q(mat(0,3),mat(1,3),mat(2,3),mat(3,3));
                double yaw, pitch, roll;
                tf::Matrix3x3 m(q);
                m.getRPY(roll,pitch,yaw);
                q = q.normalize();
                transform.setRotation(q);
                transform.frame_id_ ="/odom";
                transform.child_frame_id_="/pcl";

                //Multiscan
                transformPointCloud(*cloud_global, *cloud_out, global_transform.inverse());
            }
            else {
                *cloud_out = *cloud_in;
            }
            flagPCL = true;
            flag = false;
            br.sendTransform(transform);
            pub.publish(globalCloud);
        }
        ros::spinOnce();
        r.sleep();
    }
}
