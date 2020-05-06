#include <ros/ros.h>
#include <QXmlStreamReader>
#include <QFile>
#include <QString>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <geographic_msgs/GeoPoint.h>
#include <geodesy/utm.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "LR7_node");
  ros::NodeHandle nh;
  sensor_msgs::NavSatFix coordinates;
  nav_msgs::Path path;
  double x0 = 0;
  double y0 = 0;
  double x,y;
  geometry_msgs::PoseStamped poseStamped;
  path.poses.clear();
  ros::Publisher pub = nh.advertise<nav_msgs::Path>("/path", 100);
  QString pathToFile = "/home/pavel/Загрузки/0427071932-56234.gpx";
  QString extension;
  extension = pathToFile.right(3);
  if(extension=="gpx"){
    QFile file(pathToFile);
    if(!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
      ROS_INFO("File open error: %s", file.errorString().data());
    }
    QXmlStreamReader inputStream(&file);
    while (!inputStream.atEnd() && !inputStream.hasError())
    {
      inputStream.readNext();
      if (inputStream.isStartElement()) {
        QString name = inputStream.name().toString();
        if (name == "trkpt"){
          coordinates.latitude = inputStream.attributes().value("lon").toFloat();
          coordinates.longitude =  inputStream.attributes().value("lat").toFloat();

          ROS_INFO("%f %f", coordinates.latitude, coordinates.longitude);

          geographic_msgs::GeoPoint geo_pt;
          geo_pt.latitude = coordinates.latitude;
          geo_pt.longitude = coordinates.longitude;
          geodesy::UTMPoint utm_pt(geo_pt);

          x = utm_pt.easting;
          y = utm_pt.northing;

          if(x0 == 0){
            x0 = x;
            y0 = y;
          }

          poseStamped.pose.position.x = utm_pt.easting-x0;
          poseStamped.pose.position.y = utm_pt.northing-y0;

          path.poses.push_back(poseStamped);

          ROS_INFO("Path poses %f %f", poseStamped.pose.position.x, poseStamped.pose.position.y);


        }
      }
    }
    ros::Rate r(10);
    while(nh.ok())
    {
      path.header.stamp = ros::Time::now();
      path.header.frame_id = "gps";
      pub.publish(path);
      r.sleep();
    }
  }
}
