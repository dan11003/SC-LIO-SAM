#include "lio_sam/utility.h"

void SaveData(const std::string &directory,
              std::vector<pcl::PointCloud<PointType>::Ptr> clouds,
              gtsam::Values &isamCurrentEstimate,
              const std::vector<double> &stamps,
              const Eigen::Vector3d &datum_offset,
              bool save_balm,
              bool save_odom,
              bool save_posegraph,
              bool save_balm2)
{

  std::vector<Eigen::Affine3d> poses;
  for (int i = 0; i < isamCurrentEstimate.size(); i++)
  {
    gtsam::Pose3 pose = isamCurrentEstimate.at<gtsam::Pose3>(i);
    Eigen::Affine3d m(pose.matrix());
    poses.push_back(std::move(m));
  }

  cout << "\"SLAM\" - Save poses: " << poses.size() << ", stamps: " << stamps.size() << ", clouds: " << clouds.size() << endl;

  if (save_odom)
    IO::SaveOdom(directory + "odom/", poses, stamps, clouds);
  if (save_balm)
    IO::SavePosesHomogeneousBALM(clouds, poses, directory + "balm/", 0.3);
  if (save_balm2)
    IO::SaveBALM2(directory + "BALM2/", poses, stamps, clouds);
  if (save_posegraph)
  {
    std::cerr << "Saving of posegraph not implemented yet for sc liosam" << std::endl;
  }
  IO::SaveMerged(clouds, poses, datum_offset, directory, 0.3);
}


GpsReadLog::GpsReadLog(const std::string& path){
  //open filestream
  cout << "Load gcp log from: " << path << endl;
  file.open(path);
  if(!file.is_open()){
    std::cerr << "Could not open file " << path << std::endl;
  }
}
std::vector<std::string> GpsReadLog::split(const std::string &s, char delimiter) {
    std::vector<std::string> tokens;
    std::istringstream tokenStream(s);
    std::string token;
    while (std::getline(tokenStream, token, delimiter)) {
        tokens.push_back(token);
    }
    return tokens;
}

void GpsReadLog::Read(std::vector<nav_msgs::Odometry>& gps_log){
  cout << "Reading gcp" << endl;
  std::string line;
  std::getline(file, line);
    
    while (std::getline(file, line)) {
        // Split the line into fields
        std::vector<std::string> fields = split(line, ',');

        // Assuming the CSV format has three fields: Identifier	Latitude	Longitude	AdjustedAltitude	Time
        if (fields.size() >= 5) {
          nav_msgs::Odometry gps;
          const std::string identifier = fields[0];
          gps.child_frame_id = identifier;
          gps.pose.pose.position.y = std::stod(fields[1]);
          gps.pose.pose.position.x = std::stod(fields[2]);
          gps.pose.pose.position.z = std::stod(fields[3]);
          const double stamp = std::stod(fields[4]);
          gps.header.stamp = ros::Time(stamp);
          gps.pose.covariance[0] = 0.01;//
          gps.pose.covariance[7] = 0.01;
          gps.pose.covariance[14] = 0.01;
          gps.twist.twist.linear.x = 0.0; // please insert something reasonable here
          gps.twist.twist.linear.y = 0.0;
          gps.twist.twist.linear.z = 0.0;

          gps_log.push_back(gps);
        }
    }
    file.close();
}