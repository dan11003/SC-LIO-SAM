#include "lio_sam/utility.h"



void SaveData(const std::string& directory,
              std::vector<pcl::PointCloud<PointType>::Ptr> clouds,
              gtsam::Values& isamCurrentEstimate,
              const std::vector<double>& stamps,
              const Eigen::Vector3d& datum_offset,
              bool save_balm,
              bool save_odom,
              bool save_posegraph,
              bool save_balm2){


std::vector<Eigen::Affine3d> poses;
  for(int i = 0 ; i<isamCurrentEstimate.size(); i++) {
    gtsam::Pose3 pose = isamCurrentEstimate.at<gtsam::Pose3>(i);
    Eigen::Affine3d m(pose.matrix());
    poses.push_back(std::move(m));
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
  IO::SaveMerged(clouds, poses, datum_offset, directory, 0.3);

}
