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
#include <nav_msgs/Path.h>
#include <Eigen/Dense>

static sensor_msgs::PointCloud cloud;
static sensor_msgs::PointCloud globalCloud;
static sensor_msgs::LaserScan scan;
static bool flag = false;
static bool flagPCL = false;
static pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
static pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_out(new pcl::PointCloud<pcl::PointXYZ>);
static pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_global(new pcl::PointCloud<pcl::PointXYZ>);


static nav_msgs::Odometry odomS;
void callbackOdom(const nav_msgs::Odometry &data){
    odomS=data;
}
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
    ros::Subscriber odomSub = nh.subscribe("/odom",1000,callbackOdom);

    ros::Publisher pub = nh.advertise<sensor_msgs::PointCloud>("pcl_cloud",1000);
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("cloud_odom",10);
    ros::Publisher pubPathICP = nh.advertise<nav_msgs::Path>("pathICP", 100);
    ros::Publisher pubPathOdom = nh.advertise<nav_msgs::Path>("pathOdom", 100);
    ros::Publisher pubPathKalm = nh.advertise<nav_msgs::Path>("pathKalm", 100);

    tf::StampedTransform transform;
    tf::TransformBroadcaster br;
    ros::Rate r(10);

    Eigen::Matrix4f global_transform = Eigen::Matrix4f::Identity();
    sensor_msgs::ChannelFloat32 channel;
    channel.name = "channel 1";
    globalCloud.channels.push_back(channel);
    globalCloud.header.frame_id = "/odom";

    nav_msgs::Path pathICP;
    nav_msgs::Path pathOdom;
    nav_msgs::Path pathKalm;


    double odomPrev_x = 0;
    double odomPrev_y = 0;
    double odomPrev_yaw = 0;

    Eigen::Vector4f vecI4;
    Eigen::Vector3f vecI3;
    Eigen::Vector3f vecP;
    Eigen::Vector3f vecR;
    Eigen::Vector3f vecQ;

    vecI4 << 1, 1, 1, 1;
    vecI3 << 1, 1, 1;
    vecP << 0.5, 0.5, 0.5;
    vecR << 0.1, 0.1, 0.1;
    vecQ << 0.001, 0.001, 0.001;

    Eigen::Vector3f kalmX;
    Eigen::Matrix3f kalmP = vecP.asDiagonal();
    Eigen::Matrix3f kalmG;

    const Eigen::Matrix3f kalmA = vecI3.asDiagonal();
    const Eigen::Matrix3f kalmB = vecI3.asDiagonal();
    const Eigen::Matrix3f kalmI = vecI3.asDiagonal();
    const Eigen::Matrix3f kalmR = vecR.asDiagonal();
    const Eigen::Matrix3f kalmQ = vecQ.asDiagonal();

    while (ros::ok()) {

        if(flag){

          tf::Quaternion quat;
          quat.setX(odomS.pose.pose.orientation.x);
          quat.setY(odomS.pose.pose.orientation.y);
          quat.setZ(odomS.pose.pose.orientation.z);
          quat.setW(odomS.pose.pose.orientation.w);
          tf::Matrix3x3 m(quat);
          double roll, pitch, yaw;
          m.getRPY(roll, pitch, yaw);

          Eigen::Matrix4f odomMat;

          odomMat(0, 0) = m[0][0];
          odomMat(0, 1) = m[0][1];
          odomMat(0, 2) = m[0][2];
          odomMat(0, 3) = odomS.pose.pose.position.x;

          odomMat(1, 0) = m[1][0];
          odomMat(1, 1) = m[1][1];
          odomMat(1, 2) = m[1][2];
          odomMat(1, 3) = odomS.pose.pose.position.y;

          odomMat(2, 0) = m[2][0];
          odomMat(2, 1) = m[2][1];
          odomMat(2, 2) = m[2][2];
          odomMat(2, 3) = odomS.pose.pose.position.z;

          odomMat(3, 0) = m[0][0];
          odomMat(3, 1) = m[0][0];
          odomMat(3, 2) = m[0][0];
          odomMat(3, 3) = 1;

          Eigen::Matrix4f odomMatProc = vecI4.asDiagonal();


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
                Eigen::Matrix4f mat = icp.getFinalTransformation();

                //Одометрия из сопоставления
                global_transform *= mat;
                transformPointCloud(*cloud_in, *cloud_out, global_transform);
                *cloud_global += *cloud_out;
                odomMatProc = global_transform * odomMat;
                //фильтрация Калмана

                Eigen::Vector3f kalmU;
                kalmU(0) = odomS.pose.pose.position.x - odomPrev_x;
                kalmU(1) = odomS.pose.pose.position.y - odomPrev_y;
                kalmU(2) = yaw - odomPrev_yaw;

                odomPrev_x = odomS.pose.pose.position.x;
                odomPrev_y = odomS.pose.pose.position.y;
                odomPrev_yaw = yaw;

                kalmX = kalmA * kalmX + kalmB * kalmU;
                kalmP = kalmA * kalmP * kalmA.transpose() + kalmQ;

                Eigen::Matrix3f forInverse;
                forInverse = kalmP + kalmR;
                kalmG = kalmP * forInverse.inverse();
                Eigen::Vector3f kalmZ;

                kalmZ(0) = global_transform(0, 3);
                kalmZ(1) = global_transform(1, 3);
                kalmZ(2) = yaw;

                kalmX = kalmX + kalmG * (kalmZ - kalmX);
                std::cout<<"kalmX"<<kalmX;
                kalmP = (kalmI  - kalmG) * kalmP;

                geometry_msgs::PoseStamped poseProc;

                poseProc.pose.position.x = odomMatProc(0, 3);
                poseProc.pose.position.y = odomMatProc(1, 3);
                poseProc.pose.position.z = odomMatProc(2, 3);
                pathICP.poses.push_back(poseProc);

                poseProc.pose.position.x = odomS.pose.pose.position.x;
                poseProc.pose.position.y = odomS.pose.pose.position.y;
                poseProc.pose.position.z = odomS.pose.pose.position.z;
                pathOdom.poses.push_back(poseProc);

                poseProc.pose.position.x = kalmX(0);
                poseProc.pose.position.y = kalmX(1);
                poseProc.pose.position.z = 0;
                pathKalm.poses.push_back(poseProc);

                std::cout<<"local transform "<<std::endl;
                std::cout<<mat<<std::endl;
                nav_msgs::Odometry odom;
                std::cout<<"X отфильтрованная"<<kalmX(0);
                std::cout<<"Y отфильтрованная"<<kalmX(1);
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
                odomPrev_x = odomS.pose.pose.position.x;
                odomPrev_y = odomS.pose.pose.position.y;
                odomPrev_yaw = yaw;
            }
            flagPCL = true;
            flag = false;
            br.sendTransform(transform);
            pub.publish(globalCloud);
            pathICP.header.stamp=ros::Time::now();
            pathKalm.header.stamp=ros::Time::now();
            pathOdom.header.stamp=ros::Time::now();
            pathICP.header.frame_id="/odom";
            pathKalm.header.frame_id="/odom";
            pathOdom.header.frame_id="/odom";
            pubPathICP.publish(pathICP);
            pubPathKalm.publish(pathKalm);
            pubPathOdom.publish(pathOdom);
        }
        ros::spinOnce();
        r.sleep();
    }
}
