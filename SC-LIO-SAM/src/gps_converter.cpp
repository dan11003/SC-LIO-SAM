#include "utility.h"

#include "pcl/io/ply_io.h"
#include <fstream>
#include "sensor_msgs/NavSatFix.h"
#include "wgs84_converter.hpp"



using std::endl;


class GpsConverter
{

public:

    ros::Publisher pubGpsOdom;
    ros::Subscriber subGPS;
    ros::NodeHandle nh_;
    nav_msgs::Odometry odomMsg_;


public:
    void gpsHandler(const sensor_msgs::NavSatFixConstPtr& gpsMsg)
    {
      static const std::array<double, 2> refrence_projection = {gpsMsg->longitude, gpsMsg->latitude};
      std::array<double, 2> current_polar = {gpsMsg->longitude, gpsMsg->latitude};
      std::array<double, 2> current_cart = wgs84::toCartesian(refrence_projection,current_polar);
      Eigen::Vector3d pGps((double)current_cart[0],  (double)current_cart[1], 0.0);
      const Eigen::AngleAxisd rollAngle(0, Eigen::Vector3d::UnitX());
      const Eigen::AngleAxisd pitchAngle(0, Eigen::Vector3d::UnitY());
      const Eigen::AngleAxisd yawAngle(-90*M_PI/180.0, Eigen::Vector3d::UnitZ());

      const Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
      const Eigen::Matrix3d rotationMatrix = q.matrix();
      const Eigen::Vector3d pGpsCorrected = rotationMatrix*pGps;

      //static const std::array<double, 2> first_cart = current_cart;
      //current_cart[0] = current_cart[0] - first_cart[0];
      //current_cart[1] = current_cart[1] - first_cart[1];
      //cout <<"current: " << pGpsCorrected[0] << " " <<current_cart[1] << endl;
      //cout <<"current: " << first_cart[0] << " " <<first_cart[0] << endl;
      odomMsg_.pose.pose.position.x = pGpsCorrected(0); odomMsg_.pose.pose.position.y = -pGpsCorrected(1); odomMsg_.pose.pose.position.z = 0;
      odomMsg_.pose.covariance[0] = gpsMsg->position_covariance[0];
      odomMsg_.pose.covariance[7] = gpsMsg->position_covariance[4];
      odomMsg_.pose.covariance[14] = gpsMsg->position_covariance[8];
      odomMsg_.pose.covariance[21] = 1000;
      odomMsg_.pose.covariance[28] = 1000;
      odomMsg_.pose.covariance[35] = 1000;
      odomMsg_.header.stamp = gpsMsg->header.stamp;

      pubGpsOdom.publish(odomMsg_);

    }

    GpsConverter(ros::NodeHandle& nh) : nh_(nh)
    {
      odomMsg_.header.frame_id = "odom";
      odomMsg_.child_frame_id = "gps_cart";
      odomMsg_.pose.pose.orientation.w = 1; odomMsg_.pose.pose.orientation.x = 0; odomMsg_.pose.pose.orientation.y = 0; odomMsg_.pose.pose.orientation.z = 1;
      pubGpsOdom = nh_.advertise<nav_msgs::Odometry>("/fix/latched/odom", 1000);

      subGPS = nh_.subscribe("/fix/latched", 1000, &GpsConverter::gpsHandler, this);

    }


};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "gps_converter");
    ros::NodeHandle nh("~");
    GpsConverter converter(nh);

    ros::spin();

    return 0;
}
