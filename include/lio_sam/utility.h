#pragma once
#include "lio_sam/generics.h"
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

// #include <opencv2/opencv.hpp>

using namespace gtsam;

using symbol_shorthand::B; // Bias  (ax,ay,az,gx,gy,gz)
using symbol_shorthand::G; // GPS pose
using symbol_shorthand::V; // Vel   (xdot,ydot,zdot)
using symbol_shorthand::X; // Pose3 (x,y,z,r,p,y)

using namespace std;

typedef std::numeric_limits<double> dbl;

enum class SensorType
{
    MULRAN,
    VELODYNE,
    OUSTER
};

class ParamServer
{
public:
    ros::NodeHandle nh;

    std::string robot_id;

    // Topics
    string pointCloudTopic;
    string imuTopic;
    string odomTopic;
    string gpsTopic;

    // Frames
    string lidarFrame;
    string baselinkFrame;
    // string odometryFrame;
    string mapFrame;

    // GPS Settings
    bool useImuHeadingInitialization;
    bool useGpsElevation;
    float gpsCovThreshold;
    float poseCovThreshold;

    // Save pcd
    bool export_pcd = false;
    string savePCDDirectory;
    bool save_Posegraph = false;
    bool save_BALM = false;
    bool save_BALM2 = false;

    bool save_odom = false;
    bool saveRefinementGraph = false;

    bool use_gps = false;
    float gps_noise_scaling_factor = 1.0;
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
    float mappingSurfLeafSize;

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
    bool loopClosureEnableFlagSC, loopClosureEnableFlagRS;
    float loopClosureFrequency;
    int surroundingKeyframeSize;
    float historyKeyframeSearchRadius;
    float historyKeyframeSearchTimeDiff;
    int historyKeyframeSearchNum;
    float historyKeyframeFitnessScore;

    // global map visualization radius
    float globalMapVisualizationSearchRadius;
    float globalMapVisualizationPoseDensity;
    float globalMapVisualizationLeafSize;

    float datum_sweref_x = 0, datum_sweref_y = 0, datum_sweref_z = 0;
    bool use_datum = true;

    ParamServer()
    {
        nh.param<std::string>("/robot_id", robot_id, "roboat");

        nh.param<std::string>("lio_sam/pointCloudTopic", pointCloudTopic, "points_raw");
        nh.param<std::string>("lio_sam/imuTopic", imuTopic, "imu_correct");
        nh.param<std::string>("lio_sam/odomTopic", odomTopic, "odometry/imu");
        nh.param<std::string>("lio_sam/gpsTopic", gpsTopic, "odometry/gps");

        nh.param<std::string>("lio_sam/lidarFrame", lidarFrame, "base_link");
        nh.param<std::string>("lio_sam/baselinkFrame", baselinkFrame, "base_link_scliosam");
        // nh.param<std::string>("lio_sam/odometryFrame", odometryFrame, "odom");
        nh.param<std::string>("lio_sam/mapFrame", mapFrame, "map");

        nh.param<bool>("lio_sam/useImuHeadingInitialization", useImuHeadingInitialization, false);
        nh.param<bool>("lio_sam/useGpsElevation", useGpsElevation, false);
        nh.param<float>("lio_sam/gpsCovThreshold", gpsCovThreshold, 1.0);
        nh.param<float>("lio_sam/poseCovThreshold", poseCovThreshold, 25.0);

        nh.param<bool>("/slam_save_balm", save_BALM, false);
        nh.param<bool>("/slam_save_balm2", save_BALM2, false);
        nh.param<bool>("/slam_save_posegraph", save_Posegraph, false);
        nh.param<bool>("/slam_save_odom", save_odom, false);
        nh.param<bool>("/export_slam_pcd", export_pcd, false);
        nh.param<bool>("/saveRefinementGraph", saveRefinementGraph, false);

        cout << "SLAM - save_BALM: " << save_BALM << "SLAM - save_BALM2: " << save_BALM2 << ", save_Posegraph: " << save_Posegraph << ", save_odom: " << save_odom << ", export_slam_pcd: " << export_pcd << endl;

        nh.param<std::string>("/directory_output", savePCDDirectory, "/Downloads/MISSING_directory_output/");
        savePCDDirectory = IO::CreateFolder(savePCDDirectory, "", "SLAM");

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
        nh.param<bool>("lio_sam/use_gps", use_gps, false);
        nh.param<float>("lio_sam/noise_scaling_factor", gps_noise_scaling_factor, 1.0);

        /*nh.param<float>("gnss/datum_sweref_x", datum_sweref_x, 0.0);
        nh.param<float>("gnss/datum_sweref_y", datum_sweref_y, 0.0);
        nh.param<float>("gnss/datum_sweref_z", datum_sweref_z, 0.0);
        nh.param<bool>("gnss/use_datum", use_datum, false);*/
        /*if (use_datum)
            ROS_INFO_STREAM("datum_sweref_x: " << datum_sweref_x << ", datum_sweref_y: " << datum_sweref_y << ", datum_sweref_z: " << datum_sweref_z);
        else
            ROS_INFO("Datum not set");*/

        usleep(100);
    }
};

template <typename T>
double ROS_TIME(T msg)
{
    return msg->header.stamp.toSec();
}

template <typename T>
void imuAngular2rosAngular(sensor_msgs::Imu *thisImuMsg, T *angular_x, T *angular_y, T *angular_z)
{
    *angular_x = thisImuMsg->angular_velocity.x;
    *angular_y = thisImuMsg->angular_velocity.y;
    *angular_z = thisImuMsg->angular_velocity.z;
}

template <typename T>
void imuAccel2rosAccel(sensor_msgs::Imu *thisImuMsg, T *acc_x, T *acc_y, T *acc_z)
{
    *acc_x = thisImuMsg->linear_acceleration.x;
    *acc_y = thisImuMsg->linear_acceleration.y;
    *acc_z = thisImuMsg->linear_acceleration.z;
}

template <typename T>
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

inline float pointDistance(PointType p)
{
    return sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
}

inline float pointDistance(PointType p1, PointType p2)
{
    return sqrt((p1.x - p2.x) * (p1.x - p2.x) + (p1.y - p2.y) * (p1.y - p2.y) + (p1.z - p2.z) * (p1.z - p2.z));
}

void SaveData(const std::string &directory,
              std::vector<pcl::PointCloud<PointType>::Ptr> clouds,
              gtsam::Values &isamCurrentEstimate,
              const std::vector<double> &stamps,
              const Eigen::Vector3d& datum_offset,
              bool save_balm,
              bool save_odom,
              bool save_posegraph,
              bool save_balm2);
