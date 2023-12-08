//#ifndef GENERICS_H
//#define GENERICS_H
#pragma once
#define PCL_NO_PRECOMPILE
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/search/impl/search.hpp>

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


#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

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
#include "boost/format.hpp"
#include <boost/filesystem.hpp>

#include <boost/serialization/base_object.hpp>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include "boost/serialization/serialization.hpp"
#include <boost/serialization/export.hpp>
#include <boost/serialization/version.hpp>
#include "boost/serialization/shared_ptr.hpp"
#include "boost/serialization/vector.hpp"
//#include "ndt_generic/eigen_utils.h"
#include "boost/archive/binary_iarchive.hpp"
#include "boost/archive/binary_oarchive.hpp"
#include "boost/serialization/map.hpp"
#include "eigen3/Eigen/Dense"
#include <eigen3/Eigen/Geometry>
#include <eigen3/Eigen/Core>



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
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>


using std::endl; using std::cout; using std::cerr;


//
namespace vel_point{
struct PointXYZIRTC
{
    PCL_ADD_POINT4D;                    // quad-word XYZ
    float         intensity;            ///< laser intensity reading
    std::uint16_t ring;                 ///< laser ring number
    float         time;                 ///< laser time reading
    float         curvature;            ///< laser gemetry curvature
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW     // ensure proper alignment
}
EIGEN_ALIGN16;
}  // namespace velodyne_pcl

POINT_CLOUD_REGISTER_POINT_STRUCT(vel_point::PointXYZIRTC,
                                  (float, x, x)
                                  (float, y, y)
                                  (float, z, z)
                                  (float, intensity, intensity)
                                  (std::uint16_t, ring, ring)
                                  (float, time, time)
                                  (float, curvature, curvature))

using PointType = vel_point::PointXYZIRTC;
typedef pcl::PointCloud<PointType> VelCurve;



/*
    * A point cloud type that has 6D pose info ([x,y,z,roll,pitch,yaw] intensity is time stamp)
    */
struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY;                  // preferred way of adding a XYZ+padding
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW   // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT (PointXYZIRPYT,
                                   (float, x, x) (float, y, y)
                                   (float, z, z) (float, intensity, intensity)
                                   (float, roll, roll) (float, pitch, pitch) (float, yaw, yaw)
                                   (double, time, time))

typedef PointXYZIRPYT  PointTypePose;
typedef pcl::PointCloud<pcl::PointXYZINormal> NormalCloud;
typedef pcl::PointCloud<pcl::PointXYZI> IntensityCloud;

namespace IO{

void createTransformedPointCloud(const pcl::PointCloud<PointType>::Ptr input, const Eigen::Vector3d& transform, pcl::PointCloud<PointType>::Ptr output);

void SaveMerged(const std::vector<pcl::PointCloud<PointType>::Ptr> clouds, const std::vector<Eigen::Affine3d> poses, const Eigen::Vector3d& datum_offset, const std::string& directory, double downsample_size);

void SaveOdom(
        const std::string& dump_directory,
        const std::vector<Eigen::Affine3d>& poses,
        const std::vector<double>& keyframe_stamps,
        const std::vector<pcl::PointCloud<PointType>::Ptr>& clouds);


void SavePosesHomogeneousBALM(
        const std::vector<pcl::PointCloud<PointType>::Ptr>& clouds,
        const std::vector<Eigen::Affine3d>& poses,
        const std::string& directory,
        double downsample_size);

void SaveBALM2(
        const std::string& dump_directory,
        const std::vector<Eigen::Affine3d>& poses,
        const std::vector<double>& keyframe_stamps,
        const std::vector<pcl::PointCloud<PointType>::Ptr>& clouds);

/* Returns created subfolder with timestamp */
std::string CreateFolder(const std::string& basePath, const std::string& prefix, const std::string& callerName);

void saveSCD(std::string fileName, Eigen::MatrixXd matrix, std::string delimiter = " ");

std::string padZeros(int val, int num_digits = 6);
}

sensor_msgs::Imu imuConverter(const sensor_msgs::Imu& imu_in, const Eigen::Quaterniond& extQRPY);

inline pcl::PointXYZ ToXYZ(const PointType& pnt){
    pcl::PointXYZ p;
    p.x = pnt.x;
    p.y = pnt.y;
    p.z = pnt.z;
    return p;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr ToCloudXYZ(pcl::PointCloud<PointType>::Ptr cloud);

Eigen::Isometry3d vectorToAffine3d(double x, double y, double z, double ex, double ey, double ez);

Eigen::Isometry3d EigenCombine(const Eigen::Quaterniond& q, const Eigen::Vector3d& transl);

//sensor_msgs::PointCloud2 publishCloud(ros::Publisher *thisPub, pcl::PointCloud<PointType>::Ptr thisCloud, ros::Time thisStamp, std::string thisFrame);

pcl::PointCloud<pcl::PointXYZI>::Ptr VelToIntensityCopy(const pcl::PointCloud<vel_point::PointXYZIRTC>::Ptr VelCloud);

typedef struct
{
    float l3; // largest eigen value;
    float planarity;
    int nSamples;
    //Eigen::Matrix3d cov;
    Eigen::Vector3d normal;
    //Eigen::Vector3d mean;
    Eigen::Vector3d centerPoint;
    float entropy;
    float intensity;
    float time;
    float curvature;

    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
      //ar & queue_new_nodes_;
      ar & l3;
      ar & planarity;
      ar & nSamples;
      ar & normal;
      ar & centerPoint;
      ar & entropy;
      ar & intensity;
      ar & time;
      ar & curvature;
    }

}SurfelPointInfo;

class SurfElCloud
{
public:
  SurfElCloud() {}

  NormalCloud::Ptr GetPointCloud(int intensity = 0)const;

  std::vector<double> GetPointCloudTime()const;

  auto begin(){return cloud.begin();}

  auto end(){return cloud.end();}

  SurfElCloud Transform(const Eigen::Isometry3d& transform)const;

  std::vector<SurfelPointInfo> cloud;

protected:


  friend class boost::serialization::access;
  template<class Archive>
  void serialize(Archive & ar, const unsigned int version)
  {
    //ar & queue_new_nodes_;
    ar & cloud;
  }

};

void SortTime(pcl::PointCloud<vel_point::PointXYZIRTC>::Ptr cloud);

struct lessThanKey
{
  inline bool operator() (const PointType& p1, const PointType& p2)const
  {
    return (p1.time < p2.time);
  }
};

//Runtime: 72 ms, faster than 99.56% of C++ online submissions for Find K Closest Elements.
//Memory Usage: 31 MB, less than 16.67% of C++ online submissions for Find K Closest Elements.
/*class NNSearchArray {
public:
    std::vector<int> findClosestElements(std::vector<double>& arr, int k, float max, float query);
};*/



/*
class SurfelExtraction
{

public:
  SurfelExtraction(pcl::PointCloud<PointType>::Ptr& surf_in, int n_scan_lines, float scan_period);

  void Extract(SurfElCloud& surfelCloud);

private:

  void LineNNSearch( const int ring, const double query, int &row, Eigen::MatrixXd& neighbours);

  bool GetNeighbours(const vel_point::PointXYZIRTC& pnt, Eigen::MatrixXd& neighbours);

  bool EstimateNormal(const vel_point::PointXYZIRTC& pnt, SurfelPointInfo& surfEl);

  void Initialize();

  pcl::PointCloud<PointType>::Ptr surf_in_;
  std::vector<pcl::PointCloud<PointType>::Ptr> ringClouds_; //sorted in time, and segmented per ring
  std::vector<std::vector<double> > times_;

  pcl::PointXYZINormal defaultNormal;
  int n_scan_lines_;
  float scan_period_;
  int nr_neighbours = 2;

};*/


struct Node{
    Eigen::Isometry3d T;
    Eigen::Quaterniond imu;
    std::vector<pcl::PointCloud<PointType>::Ptr> segmented_scans;

    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
      //ar & queue_new_nodes_;
      ar & T;
      ar & imu;
      ar & segmented_scans;
    }
};



class PoseGraph{
public:
    PoseGraph(){}
    typedef std::map<std::pair<int,int>,Eigen::Isometry3d> Constraint;

   void AddNode(const Node& node,  int id){ nodes[id] = node;}

   void AddConstraint(const Eigen::Isometry3d& c, int from, int to){ constraints[std::make_pair(from,to)] = c;}

    std::map<std::pair<int,int>,Eigen::Isometry3d> constraints;

    std::map<int,Node> nodes;

    std::map<int,SurfElCloud> surfels_;

    std::string ToString(){return "vertices: " + std::to_string(nodes.size()) + ", edges: " + std::to_string(constraints.size()); }
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
      //ar & queue_new_nodes_;
      ar & constraints;
      ar & nodes;
      ar & surfels_;
    }

    static bool LoadGraph(const std::string& path, boost::shared_ptr<PoseGraph> &graph);

    static void SaveGraph(const std::string& path, boost::shared_ptr<PoseGraph> &graph);

};



namespace boost {
namespace serialization {


template<typename Archive>
void serialize(Archive& ar,  pcl::PointCloud<PointType>& points, const unsigned version) {
  ar & points.header.stamp;
  ar & points.header.seq;
  ar & points.header.frame_id;
  ar & points.height;
  ar & points.width;
  points.resize(points.height*points.width);
  for(int i=0; i<points.size();i++){
    ar & points[i];
  }
}

template<typename Archive>
void serialize(Archive& ar, PointType &point, const unsigned version) {
  ar & point.data;
  ar & point.curvature;
  ar & point.time;
  ar & point.intensity;
  ar & point.ring;
}

 template<typename Archive>
void serialize(Archive& ar, Eigen::Isometry3d &o, const unsigned version) {
  for (int i = 0; i < 16; i++) {
    ar & o.data()[i];
  }
}

template<typename Archive>
void serialize(Archive& ar, Eigen::Vector3d &o, const unsigned version) {
 for (int i = 0; i < 3; i++) {
   ar & o.data()[i];
 }
}

template<typename Archive>
void serialize(Archive& ar, Eigen::Quaterniond &o, const unsigned version) {
 for (int i = 0; i < 4; i++) {
   ar & o.coeffs().data()[i];
 }
}


} // namespace serialization
} // namespace boost

//#endif // GENERICS_H
