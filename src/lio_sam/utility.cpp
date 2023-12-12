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

GpsLog::GpsLog(const std::string &directory, const std::string &filename) : directory_(directory), filename_(filename)
{
  if (!directory_.empty())
  {
    if (directory_.back() != '/')
    {
      directory_ += '/';
    }
    if (!boost::filesystem::exists(directory_))
    {
      boost::filesystem::create_directories(directory_);
    }
  }
  if (!filename_.empty())
  {
    gps_log_file_.open(directory_ + filename_);
    if (gps_log_file_.is_open())
    {
      gps_log_file_ << "Identifier, Latitude, Longitude, Adjusted Altitude, Time" << std::endl;
    }
    else
    {
      std::cerr << "Failed to open GPS log file: " << directory_ + filename_ << std::endl;
    }
  }
}
void GpsLog::write(std::string &identifier, double x, double y, double z, double time, double noise_x)
{
  if (gps_log_file_.is_open())
  {
    //write with fixed precision, non scientific notation
    gps_log_file_ << std::fixed << std::setprecision(10) << identifier << ", " << x << ", " << y << ", " << z << ", " << time << ", " << noise_x << std::endl;
  }
}
void GpsLog::Save()
{
  if (gps_log_file_.is_open())
  {
    cout << "GPS log Saved to " << directory_ + filename_ << endl;
    gps_log_file_.close();
  }
}

GpsLog::~GpsLog()
{
  Save();
}
