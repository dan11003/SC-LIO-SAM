#pragma once

#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <opencv2/opencv.hpp>


#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h> 
#include <pcl_conversions/pcl_conversions.h>

#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
 #include <gtsam/nonlinear/Values.h>

#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>

#include <gtsam/nonlinear/ISAM2.h>


#include <limits>
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>
#include <sstream>
#include "boost/format.hpp"
#include <boost/filesystem.hpp>


using namespace gtsam;
using std::endl;

using symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)
using symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::G; // GPS pose

using namespace std;

typedef std::numeric_limits< double > dbl;

typedef pcl::PointXYZI PointType;

enum class SensorType { MULRAN, VELODYNE, OUSTER };

void SaveMerged(const std::vector<pcl::PointCloud<PointType>::Ptr> clouds, const std::vector<Eigen::Affine3d> poses, const std::string& directory, double downsample_size){
  boost::filesystem::create_directories(directory);
  pcl::PointCloud<pcl::PointXYZI>::Ptr merged_transformed(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::PointCloud<pcl::PointXYZI> merged_downsamapled;
  std::cout << "Save merged point cloud to:\n" << directory << std::endl <<  std::endl;

  for(int i = 0; i < poses.size() ; i++) {
      pcl::PointCloud<pcl::PointXYZI> tmp_transformed;
      pcl::transformPointCloud(*clouds[i], tmp_transformed, poses[i]);
      *merged_transformed += tmp_transformed;
  }
  cout << "Downsample point cloud resolution " << downsample_size << endl;


  pcl::VoxelGrid<pcl::PointXYZI> sor;
  sor.setInputCloud (merged_transformed);
  sor.setLeafSize (downsample_size, downsample_size, downsample_size);
  sor.filter (merged_downsamapled);

  pcl::io::savePCDFileBinary(directory + std::string("floam_merged.pcd"), *merged_transformed);
  if(!merged_downsamapled.empty()){
    const std::string path_downsampled = directory + std::string("floam_merged_downsampled_leaf_") + std::to_string(downsample_size) + ".pcd";
    pcl::io::savePCDFileBinary(path_downsampled, merged_downsamapled);
  }else{
    std::cout << "No downsampled point cloud saved - increase \"output_downsample_size\"" << std::endl;
  }
}

void SaveOdom(
    const std::string& dump_directory,
    const std::vector<Eigen::Affine3d>& poses,
    const std::vector<double>& keyframe_stamps,
    const std::vector<pcl::PointCloud<PointType>::Ptr>& clouds){

  boost::filesystem::create_directories(dump_directory);

  std::cout << "Save odom to:\n" << dump_directory << std::endl << std::endl;
  //std::cout << "Save clouds: " << clouds.size() << std::endl;
  for(int i = 0; i < clouds.size(); i++) {

    ros::Time t(keyframe_stamps[i]);
    std::string filename = dump_directory + str( boost::format("/%lf_%lf") % t.sec % t.nsec);

    pcl::io::savePCDFileBinary( filename + ".pcd", *clouds[i]);

    std::ofstream data_ofs(filename + ".odom");
    Eigen::Matrix<double,4,4> mat = poses[i].matrix();
    data_ofs << mat(0,0) << " " << mat(0,1) << " " << mat(0,2) << " " << mat(0,3) << std::endl;
    data_ofs << mat(1,0) << " " << mat(1,1) << " " << mat(1,2) << " " << mat(1,3) << std::endl;
    data_ofs << mat(2,0) << " " << mat(2,1) << " " << mat(2,2) << " " << mat(2,3) << std::endl;
    data_ofs << mat(3,0) << " " << mat(3,1) << " " << mat(3,2) << " " << mat(3,3) << std::endl;
    data_ofs.close();
  }
}


void SaveData(const std::string& directory,
              std::vector<pcl::PointCloud<PointType>::Ptr> cornerCloudKeyFrames,
              std::vector<pcl::PointCloud<PointType>::Ptr> edgeCloudKeyFrames,
              gtsam::Values& isamCurrentEstimate,
              const std::vector<double>& stamps){


std::vector<Eigen::Affine3d> poses;
std::vector<pcl::PointCloud<PointType>::Ptr> clouds;
  for(const auto& key_value: isamCurrentEstimate) {

    auto p = dynamic_cast<const GenericValue<Pose3>*>(&key_value.value);
    if (!p) continue;
    const Pose3& pose = p->value();
    const Eigen::Affine3d m(pose.matrix());
    poses.push_back(std::move(m));
  }

  for(int i = 0 ; i <cornerCloudKeyFrames.size() ; i++){
    pcl::PointCloud<PointType>::Ptr cld_tmp = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());
    *cld_tmp += *cornerCloudKeyFrames[i];
    *cld_tmp += *edgeCloudKeyFrames[i];
    clouds.push_back(cld_tmp);
  }
  cout <<"Save poses: " << poses.size() << ", stamps: " << stamps.size() << ", clouds: " << clouds.size() << endl;

  SaveOdom(directory+"odom/", poses, stamps, clouds);
  SaveMerged(clouds, poses, directory, 0.3);
}



/* Returns created subfolder with timestamp */
std::string CreateFolder(const std::string& basePath){

  auto t = std::time(nullptr);
  auto tm = *std::localtime(&t);

  //const std::string timeStr(std::put_time(&tm, "%Y-%m-%d_%H-%M"));

  std::time_t now = std::time(NULL);
  std::tm * ptm = std::localtime(&now);
  char buffer[32];
  // Format: Mo, 15.06.2009 20:20:00
  std::strftime(buffer, 32, "%a_%Y.%m.%d_%H:%M:%S", ptm);


  const std::string dir = basePath + "/SC_" + std::string(buffer) + std::string("/");
  std::cout << dir << std::endl;
  if (boost::filesystem::create_directories(dir)){
      std::cout << "Created new directory" << "\n";
  }
  return dir;
}


class ParamServer
{
public:

    ros::NodeHandle nh;

    std::string robot_id;

    //Topics
    string pointCloudTopic;
    string imuTopic;
    string odomTopic;
    string gpsTopic;

    //Frames
    string lidarFrame;
    string baselinkFrame;
    string odometryFrame;
    string mapFrame;

    // GPS Settings
    bool useImuHeadingInitialization;
    bool useGpsElevation;
    float gpsCovThreshold;
    float poseCovThreshold;

    // Save pcd
    bool savePCD;
    string savePCDDirectory;

    // Velodyne Sensor Configuration: Velodyne
    SensorType sensor;
    int N_SCAN;
    int Horizon_SCAN;
    int downsampleRate;
    float lidarMinRange;
    float lidarMaxRange;

    // IMU
    float imuAccNoise;
    float imuGyrNoise;
    float imuAccBiasN;
    float imuGyrBiasN;
    float imuGravity;
    float imuRPYWeight;
    vector<double> extRotV;
    vector<double> extRPYV;
    vector<double> extTransV;
    Eigen::Matrix3d extRot;
    Eigen::Matrix3d extRPY;
    Eigen::Vector3d extTrans;
    Eigen::Quaterniond extQRPY;

    // LOAM
    float edgeThreshold;
    float surfThreshold;
    int edgeFeatureMinValidNum;
    int surfFeatureMinValidNum;

    // voxel filter paprams
    float odometrySurfLeafSize;
    float mappingCornerLeafSize;
    float mappingSurfLeafSize ;

    float z_tollerance; 
    float rotation_tollerance;

    // CPU Params
    int numberOfCores;
    double mappingProcessInterval;

    // Surrounding map
    float surroundingkeyframeAddingDistThreshold; 
    float surroundingkeyframeAddingAngleThreshold; 
    float surroundingKeyframeDensity;
    float surroundingKeyframeSearchRadius;
    
    // Loop closure
    bool  loopClosureEnableFlagSC, loopClosureEnableFlagRS;
    float loopClosureFrequency;
    int   surroundingKeyframeSize;
    float historyKeyframeSearchRadius;
    float historyKeyframeSearchTimeDiff;
    int   historyKeyframeSearchNum;
    float historyKeyframeFitnessScore;

    // global map visualization radius
    float globalMapVisualizationSearchRadius;
    float globalMapVisualizationPoseDensity;
    float globalMapVisualizationLeafSize;

    ParamServer()
    {
        nh.param<std::string>("/robot_id", robot_id, "roboat");

        nh.param<std::string>("lio_sam/pointCloudTopic", pointCloudTopic, "points_raw");
        nh.param<std::string>("lio_sam/imuTopic", imuTopic, "imu_correct");
        nh.param<std::string>("lio_sam/odomTopic", odomTopic, "odometry/imu");
        nh.param<std::string>("lio_sam/gpsTopic", gpsTopic, "odometry/gps");

        nh.param<std::string>("lio_sam/lidarFrame", lidarFrame, "base_link");
        nh.param<std::string>("lio_sam/baselinkFrame", baselinkFrame, "base_link");
        nh.param<std::string>("lio_sam/odometryFrame", odometryFrame, "odom");
        nh.param<std::string>("lio_sam/mapFrame", mapFrame, "map");

        nh.param<bool>("lio_sam/useImuHeadingInitialization", useImuHeadingInitialization, false);
        nh.param<bool>("lio_sam/useGpsElevation", useGpsElevation, false);
        nh.param<float>("lio_sam/gpsCovThreshold", gpsCovThreshold, 2.0);
        nh.param<float>("lio_sam/poseCovThreshold", poseCovThreshold, 25.0);

        nh.param<bool>("lio_sam/savePCD", savePCD, false);
        nh.param<std::string>("lio_sam/savePCDDirectory", savePCDDirectory, "/Downloads/LOAM/");
        savePCDDirectory = CreateFolder(savePCDDirectory);

        std::string sensorStr;
        nh.param<std::string>("lio_sam/sensor", sensorStr, "");
        if (sensorStr == "velodyne")
        {
            sensor = SensorType::VELODYNE;
        }
        else if (sensorStr == "ouster")
        {
            sensor = SensorType::OUSTER;
        }
        else if (sensorStr == "mulran")
        {
            sensor = SensorType::MULRAN;
        }
        else
        {
            ROS_ERROR_STREAM(
                "Invalid sensor type (must be either 'velodyne' or 'ouster' or 'mulran'): " << sensorStr);
            ros::shutdown();
        }

        nh.param<int>("lio_sam/N_SCAN", N_SCAN, 16);
        nh.param<int>("lio_sam/Horizon_SCAN", Horizon_SCAN, 1800);
        nh.param<int>("lio_sam/downsampleRate", downsampleRate, 1);
        nh.param<float>("lio_sam/lidarMinRange", lidarMinRange, 1.0);
        nh.param<float>("lio_sam/lidarMaxRange", lidarMaxRange, 1000.0);

        nh.param<float>("lio_sam/imuAccNoise", imuAccNoise, 0.01);
        nh.param<float>("lio_sam/imuGyrNoise", imuGyrNoise, 0.001);
        nh.param<float>("lio_sam/imuAccBiasN", imuAccBiasN, 0.0002);
        nh.param<float>("lio_sam/imuGyrBiasN", imuGyrBiasN, 0.00003);
        nh.param<float>("lio_sam/imuGravity", imuGravity, 9.80511);
        nh.param<float>("lio_sam/imuRPYWeight", imuRPYWeight, 0.01);
        nh.param<vector<double>>("lio_sam/extrinsicRot", extRotV, vector<double>());
        nh.param<vector<double>>("lio_sam/extrinsicRPY", extRPYV, vector<double>());
        nh.param<vector<double>>("lio_sam/extrinsicTrans", extTransV, vector<double>());
        extRot = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRotV.data(), 3, 3);
        extRPY = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRPYV.data(), 3, 3);
        extTrans = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extTransV.data(), 3, 1);
        extQRPY = Eigen::Quaterniond(extRPY);

        nh.param<float>("lio_sam/edgeThreshold", edgeThreshold, 0.1);
        nh.param<float>("lio_sam/surfThreshold", surfThreshold, 0.1);
        nh.param<int>("lio_sam/edgeFeatureMinValidNum", edgeFeatureMinValidNum, 10);
        nh.param<int>("lio_sam/surfFeatureMinValidNum", surfFeatureMinValidNum, 100);

        nh.param<float>("lio_sam/odometrySurfLeafSize", odometrySurfLeafSize, 0.2);
        nh.param<float>("lio_sam/mappingCornerLeafSize", mappingCornerLeafSize, 0.2);
        nh.param<float>("lio_sam/mappingSurfLeafSize", mappingSurfLeafSize, 0.2);

        nh.param<float>("lio_sam/z_tollerance", z_tollerance, FLT_MAX);
        nh.param<float>("lio_sam/rotation_tollerance", rotation_tollerance, FLT_MAX);

        nh.param<int>("lio_sam/numberOfCores", numberOfCores, 2);
        nh.param<double>("lio_sam/mappingProcessInterval", mappingProcessInterval, 0.15);

        nh.param<float>("lio_sam/surroundingkeyframeAddingDistThreshold", surroundingkeyframeAddingDistThreshold, 1.0);
        nh.param<float>("lio_sam/surroundingkeyframeAddingAngleThreshold", surroundingkeyframeAddingAngleThreshold, 0.2);
        nh.param<float>("lio_sam/surroundingKeyframeDensity", surroundingKeyframeDensity, 1.0);
        nh.param<float>("lio_sam/surroundingKeyframeSearchRadius", surroundingKeyframeSearchRadius, 50.0);

        nh.param<bool>("lio_sam/loopClosureEnableFlagSC", loopClosureEnableFlagSC, true);
        nh.param<bool>("lio_sam/loopClosureEnableFlagRS", loopClosureEnableFlagRS, true);
        cout << "loopClosureEnableFlagSC" << loopClosureEnableFlagSC << endl;
        cout << "loopClosureEnableFlagRS" << loopClosureEnableFlagRS << endl;
        nh.param<float>("lio_sam/loopClosureFrequency", loopClosureFrequency, 1.0);
        nh.param<int>("lio_sam/surroundingKeyframeSize", surroundingKeyframeSize, 50);
        nh.param<float>("lio_sam/historyKeyframeSearchRadius", historyKeyframeSearchRadius, 10.0);
        nh.param<float>("lio_sam/historyKeyframeSearchTimeDiff", historyKeyframeSearchTimeDiff, 30.0);
        nh.param<int>("lio_sam/historyKeyframeSearchNum", historyKeyframeSearchNum, 25);
        nh.param<float>("lio_sam/historyKeyframeFitnessScore", historyKeyframeFitnessScore, 0.3);

        nh.param<float>("lio_sam/globalMapVisualizationSearchRadius", globalMapVisualizationSearchRadius, 1e3);
        nh.param<float>("lio_sam/globalMapVisualizationPoseDensity", globalMapVisualizationPoseDensity, 10.0);
        nh.param<float>("lio_sam/globalMapVisualizationLeafSize", globalMapVisualizationLeafSize, 1.0);

        usleep(100);
    }

    sensor_msgs::Imu imuConverter(const sensor_msgs::Imu& imu_in)
    {
        sensor_msgs::Imu imu_out = imu_in;
        // rotate acceleration
        Eigen::Vector3d acc(imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);
        acc = extRot * acc;
        imu_out.linear_acceleration.x = acc.x();
        imu_out.linear_acceleration.y = acc.y();
        imu_out.linear_acceleration.z = acc.z();
        // rotate gyroscope
        Eigen::Vector3d gyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);
        gyr = extRot * gyr;
        imu_out.angular_velocity.x = gyr.x();
        imu_out.angular_velocity.y = gyr.y();
        imu_out.angular_velocity.z = gyr.z();
        // rotate roll pitch yaw
        Eigen::Quaterniond q_from(imu_in.orientation.w, imu_in.orientation.x, imu_in.orientation.y, imu_in.orientation.z);
        Eigen::Quaterniond q_final = q_from * extQRPY;
        imu_out.orientation.x = q_final.x();
        imu_out.orientation.y = q_final.y();
        imu_out.orientation.z = q_final.z();
        imu_out.orientation.w = q_final.w();

        if (sqrt(q_final.x()*q_final.x() + q_final.y()*q_final.y() + q_final.z()*q_final.z() + q_final.w()*q_final.w()) < 0.1)
        {
            ROS_ERROR("Invalid quaternion, please use a 9-axis IMU!");
            ros::shutdown();
        }

        return imu_out;
    }
};


sensor_msgs::PointCloud2 publishCloud(ros::Publisher *thisPub, pcl::PointCloud<PointType>::Ptr thisCloud, ros::Time thisStamp, std::string thisFrame)
{
    sensor_msgs::PointCloud2 tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;
    if (thisPub->getNumSubscribers() != 0)
        thisPub->publish(tempCloud);
    return tempCloud;
}

template<typename T>
double ROS_TIME(T msg)
{
    return msg->header.stamp.toSec();
}


template<typename T>
void imuAngular2rosAngular(sensor_msgs::Imu *thisImuMsg, T *angular_x, T *angular_y, T *angular_z)
{
    *angular_x = thisImuMsg->angular_velocity.x;
    *angular_y = thisImuMsg->angular_velocity.y;
    *angular_z = thisImuMsg->angular_velocity.z;
}


template<typename T>
void imuAccel2rosAccel(sensor_msgs::Imu *thisImuMsg, T *acc_x, T *acc_y, T *acc_z)
{
    *acc_x = thisImuMsg->linear_acceleration.x;
    *acc_y = thisImuMsg->linear_acceleration.y;
    *acc_z = thisImuMsg->linear_acceleration.z;
}


template<typename T>
void imuRPY2rosRPY(sensor_msgs::Imu *thisImuMsg, T *rosRoll, T *rosPitch, T *rosYaw)
{
    double imuRoll, imuPitch, imuYaw;
    tf::Quaternion orientation;
    tf::quaternionMsgToTF(thisImuMsg->orientation, orientation);
    tf::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);

    *rosRoll = imuRoll;
    *rosPitch = imuPitch;
    *rosYaw = imuYaw;
}


float pointDistance(PointType p)
{
    return sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
}


float pointDistance(PointType p1, PointType p2)
{
    return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z));
}

void saveSCD(std::string fileName, Eigen::MatrixXd matrix, std::string delimiter = " ")
{
    // delimiter: ", " or " " etc.

    int precision = 3; // or Eigen::FullPrecision, but SCD does not require such accruate precisions so 3 is enough.
    const static Eigen::IOFormat the_format(precision, Eigen::DontAlignCols, delimiter, "\n");
 
    std::ofstream file(fileName);
    if (file.is_open())
    {
        file << matrix.format(the_format);
        file.close();
    }
}

std::string padZeros(int val, int num_digits = 6) {
  std::ostringstream out;
  out << std::internal << std::setfill('0') << std::setw(num_digits) << val;
  return out.str();
}
