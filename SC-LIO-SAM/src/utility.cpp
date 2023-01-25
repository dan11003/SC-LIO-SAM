#include "utility.h"

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
  for(int i = 0 ; i<isamCurrentEstimate.size(); i++) {
    gtsam::Pose3 pose = isamCurrentEstimate.at<gtsam::Pose3>(i);

    /*auto p = dynamic_cast<const GenericValue<Pose3>*>(isamCurrentEstimate.at<Pose3>(i).);
    if (!p) continue;
    const Pose3& pose = p->value();*/
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

