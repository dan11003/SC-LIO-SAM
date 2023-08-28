#include "lio_sam/utility.h"



void SaveData(const std::string& directory,
              std::vector<pcl::PointCloud<PointType>::Ptr> cornerCloudKeyFrames,
              std::vector<pcl::PointCloud<PointType>::Ptr> lesscornerCloudKeyFrames,
              std::vector<pcl::PointCloud<PointType>::Ptr> flatCloudKeyFrames,
              gtsam::Values& isamCurrentEstimate,
              const std::vector<double>& stamps,
              bool save_balm,
              bool save_odom,
              bool save_posegraph,
              bool save_balm2){


std::vector<Eigen::Affine3d> poses;
std::vector<pcl::PointCloud<PointType>::Ptr> clouds;
  for(int i = 0 ; i<isamCurrentEstimate.size(); i++) {
    gtsam::Pose3 pose = isamCurrentEstimate.at<gtsam::Pose3>(i);
    const Eigen::Affine3d m(pose.matrix());
    poses.push_back(std::move(m));
  }

  for(int i = 0 ; i <cornerCloudKeyFrames.size() ; i++){
    pcl::PointCloud<PointType>::Ptr cld_tmp = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());
    *cld_tmp += *cornerCloudKeyFrames[i];
    *cld_tmp += *flatCloudKeyFrames[i];
    *cld_tmp += *lesscornerCloudKeyFrames[i];

    clouds.push_back(cld_tmp);
  }
  cout <<"\"SLAM\" - Save poses: " << poses.size() << ", stamps: " << stamps.size() << ", clouds: " << clouds.size() << endl;

  if(save_odom)
    IO::SaveOdom(directory+"odom/", poses, stamps, clouds);
  if(save_balm)
    IO::SavePosesHomogeneousBALM(clouds, poses, directory+"balm/", 0.3);
  if(save_balm2)
    IO::SaveBALM2(directory + "BALM2/", poses, stamps, clouds);
  if(save_posegraph){
    std::cerr << "Saving of posegraph not implemented yet for sc liosam" << std::endl;
  }
  IO::SaveMerged(clouds, poses, directory, 0.3);

}
