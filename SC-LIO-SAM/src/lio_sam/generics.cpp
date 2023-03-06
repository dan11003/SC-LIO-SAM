#include "lio_sam/generics.h"


namespace IO{
void SaveMerged(const std::vector<pcl::PointCloud<PointType>::Ptr> clouds, const std::vector<Eigen::Affine3d> poses, const std::string& directory, double downsample_size){
    boost::filesystem::create_directories(directory);
    pcl::PointCloud<PointType>::Ptr merged_transformed(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType> merged_downsamapled;
    std::cout << "\"SLAM\" - Save merged point cloud to: " << directory << std::endl <<  std::endl;

    for(int i = 0; i < poses.size() ; i++) {
        pcl::PointCloud<PointType> tmp_transformed;
        pcl::transformPointCloud(*clouds[i], tmp_transformed, poses[i]);
        *merged_transformed += tmp_transformed;
    }
    cout << "\"SLAM\" - Downsample point cloud resolution " << downsample_size << endl;


    pcl::VoxelGrid<PointType> sor;
    sor.setInputCloud (merged_transformed);
    sor.setLeafSize (downsample_size, downsample_size, downsample_size);
    sor.filter (merged_downsamapled);

    pcl::io::savePCDFileBinary(directory + std::string("sc-lio-sam_.pcd"), *merged_transformed);
    if(!merged_downsamapled.empty()){
        const std::string path_downsampled = directory + std::string("sc-lio-sam_merged_downsampled_leaf_") + std::to_string(downsample_size) + ".pcd";
        pcl::io::savePCDFileBinary(path_downsampled, merged_downsamapled);
    }else{
        std::cout << "\"SLAM\" - No downsampled point cloud saved - increase \"output_downsample_size\"" << std::endl;
    }
}

void SaveOdom(
        const std::string& dump_directory,
        const std::vector<Eigen::Affine3d>& poses,
        const std::vector<double>& keyframe_stamps,
        const std::vector<pcl::PointCloud<PointType>::Ptr>& clouds){

    boost::filesystem::create_directories(dump_directory);

    std::cout << "\"SLAM\" - Save odom to: " << dump_directory << std::endl << std::endl;
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


void SavePosesHomogeneousBALM(const std::vector<pcl::PointCloud<PointType>::Ptr>& clouds, const std::vector<Eigen::Affine3d>& poses, const std::string& directory, double downsample_size){
    boost::filesystem::create_directories(directory);
    const std::string filename = directory + "alidarPose.csv";
    std::fstream stream(filename.c_str(), std::fstream::out);
    std::cout << "\"SLAM\" - Save BALM to: " << directory << std::endl << std::endl;
    /*std::cout << "Saving clouds size: " <<clouds.size() << std::endl;;
    std::cout << "Saving poses size: " SavePosesHomogeneousBALM<<poses.size() << std::endl;*/
    pcl::PointCloud<PointType>::Ptr merged_transformed(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType> merged_downsamapled;


    for(int i = 0; i < poses.size() ; i++) {
        ros::Time tRos;
        pcl_conversions::fromPCL(clouds[i]->header.stamp, tRos);
        const double time = tRos.toSec();
        const Eigen::MatrixXd m = poses[i].matrix();

        stream <<std::fixed <<m(0,0) <<  "," << m(0,1) <<  "," << m(0,2) <<  "," << m(0,3) <<  "," << endl <<
                 m(1,0) <<  "," << m(1,1) <<  "," << m(1,2) <<  "," << m(1,3) <<  "," << endl <<
                 m(2,0) <<  "," << m(2,1) <<  "," << m(2,2) <<  "," << m(2,3) <<  "," << endl <<
                 m(3,0) <<  "," << m(3,1) <<  "," << m(3,2) <<  "," << time <<  "," << endl;
        const std::string pcdPath = directory + std::string("full") + std::to_string(i) + ".pcd";
        pcl::io::savePCDFileBinary(pcdPath, *clouds[i]);
    }
}

std::string CreateFolder(const std::string& basePath, const std::string& prefix, const std::string& callerName){

    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);

    //const std::string timeStr(std::put_time(&tm, "%Y-%m-%d_%H-%M"));

    std::time_t now = std::time(NULL);
    std::tm * ptm = std::localtime(&now);
    char buffer[32];
    // Format: Mo, 15.06.2009 20:20:00
    std::strftime(buffer, 32, "%a_%Y.%m.%d_%H:%M:%S", ptm);


    const std::string dir = basePath + "/" + prefix +"_" + std::string(buffer) + std::string("/");

    if (boost::filesystem::create_directories(dir)){
        std::cout << "\""<<callerName<<"\" - Created new output directory: "  << dir << std::endl;
    }
    return dir;
}

void saveSCD(std::string fileName, Eigen::MatrixXd matrix, std::string delimiter){

    int precision = 3; // or Eigen::FullPrecision, but SCD does not require such accruate precisions so 3 is enough.
    const static Eigen::IOFormat the_format(precision, Eigen::DontAlignCols, delimiter, "\n");
    std::ofstream file(fileName);
    if (file.is_open())
    {
        file << matrix.format(the_format);
        file.close();
    }
}

} // namespace IO


sensor_msgs::Imu imuConverter(const sensor_msgs::Imu& imu_in, const Eigen::Quaterniond& extQRPY){
    sensor_msgs::Imu imu_out = imu_in;
    Eigen::Matrix3d extRot(extQRPY.toRotationMatrix());
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


pcl::PointCloud<pcl::PointXYZ>::Ptr ToCloudXYZ(pcl::PointCloud<PointType>::Ptr cloud){
    pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>());
    output->header = cloud->header;
    output->resize(cloud->size());
    for(int i=0 ; i < cloud->size() ; i++){
        output->points[i] = ToXYZ(cloud->points[i]);
    }
    return output;
}

Eigen::Isometry3d vectorToAffine3d(double x, double y, double z, double ex, double ey, double ez) {
  return Eigen::Translation<double, 3>(x, y, z) *
      Eigen::AngleAxis<double>(ex, Eigen::Vector3d::UnitX()) *
      Eigen::AngleAxis<double>(ey, Eigen::Vector3d::UnitY()) *
      Eigen::AngleAxis<double>(ez, Eigen::Vector3d::UnitZ());
}


bool PoseGraph::LoadGraph(const std::string& path, boost::shared_ptr<PoseGraph> &graph){
  try {
    std::ifstream ifs(path);
    boost::archive::binary_iarchive ia(ifs);
    ia >> graph;
    ifs.close();
    cout<<"Graph ("<<graph->ToString()<<")succesfully loaded from: "<<path<<endl;
    return true;
  }catch (std::exception &e) {
    std::cerr<<"Graph could not be loaded from: "<<path<<std::endl;
    return false;
  }
}

void PoseGraph::SaveGraph(const std::string& path, boost::shared_ptr<PoseGraph> &graph){
  if(graph==NULL){
    std::cout<<"Nothing to save"<<endl;
    return;
  }
  try {
    std::cout<<"Save graph to: "<<path<<endl;
    std::ofstream ofs(path);
    boost::archive::binary_oarchive oa(ofs);
    oa << graph;
    ofs.close();
  }catch (std::exception &e) {
  }
}


std::vector<int> NNSearchArray::findClosestElements(std::vector<double>& arr, int k, float max, float query) {

  auto itr_begin = arr.begin();
  auto itr_end = arr.end();

  std::vector<int> indicies;
  auto it = std::lower_bound(itr_begin, itr_end, query);

  if(it == itr_begin){
    for(auto itr = itr_begin ; itr != itr_begin+k ; itr++){
      indicies.push_back(std::distance(itr_begin,itr));
    }
    return indicies; // return vector<int>(itr_begin(), itr_begin()+k);
  }else if(it == itr_end){
    for(auto itr = itr_end - k ; itr != itr_end ; itr++){
      indicies.push_back(std::distance(itr_begin,itr));
    }
    return indicies; //return vector<int>(itr_end() - k, itr_end());

  }

  const int n = arr.size();
  int idx = it - itr_begin;
  int left, right;
  left = idx - k/2;
  right = left + k;

  // cout << left << ", " << right << " -> ";

  if(left < 0){
    left = 0;
    right = left +k;
  }

  if(right > n){
    right = n;
    left = right - k;
  }

  while(left > 0 && query - arr[left-1] <= arr[right-1] - query){
    right--;
    left--;
  }

  while(right < n && arr[(right-1)+1] - query  < query - arr[left]){
    right++;
    left++;
  }

  for(auto itr = itr_begin + left  ; itr != itr_begin + right ; itr++){
    indicies.push_back(std::distance(itr_begin,itr));
  }
  //std::cout << std::endl << std::endl;
  return indicies;
}


NormalCloud::Ptr SurfElCloud::GetPointCloud()const{
  NormalCloud::Ptr output(new NormalCloud());
  for(auto && surfEl : cloud){
    pcl::PointXYZINormal pnt;
    pnt.x = surfEl.centerPoint(0); pnt.y = surfEl.centerPoint(1); pnt.z = surfEl.centerPoint(2);
    pnt.normal_x = surfEl.normal(0); pnt.normal_y = surfEl.normal(1);  pnt.normal_z = surfEl.normal(2);
    pnt.intensity = surfEl.intensity;
    output->push_back(std::move(pnt));
  }
  return output;
}

SurfElCloud SurfElCloud::Transform(const Eigen::Isometry3d& transform)const{
  SurfElCloud SurfElTransformed = *this;
  for(auto itr = SurfElTransformed.begin(); itr != SurfElTransformed.end(); itr++){
    itr->centerPoint = transform*itr->centerPoint;
    itr->normal = transform.linear()*itr->normal;
  }
  return SurfElTransformed;
}
SurfelExtraction::SurfelExtraction(pcl::PointCloud<PointType>::Ptr& surf_in, int n_scan_lines, float scan_period) : n_scan_lines_(n_scan_lines), scan_period_(scan_period){
  surf_in_ = surf_in;
  Initialize();
}

void SortTime(pcl::PointCloud<vel_point::PointXYZIRTC>::Ptr cloud){

  std::sort(cloud->begin(),cloud->end(), lessThanKey());

}


void SurfelExtraction::Initialize(){

  ringClouds_.resize(n_scan_lines_);
  times_.resize(n_scan_lines_);
  for(int i = 0 ; i < ringClouds_.size() ; i++){ // initiaize clouds
    ringClouds_[i] = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());
  }
  for(auto&& pnt : surf_in_->points){ // Fill up
    ringClouds_[pnt.ring]->push_back(pnt);
  }
  for(auto && cloud : ringClouds_){ // Sort
    SortTime(cloud);
  }
  for(auto&& cloud : ringClouds_){ // And create the same structure for doubles
    for(auto && pnt : cloud->points){
      times_[pnt.ring].push_back(pnt.time); // Please preallocate next time daniel!!!!
    }
  }

}
void SurfelExtraction::Extract(SurfElCloud& surfelCloud){

 for(auto && pnt : surf_in_->points){
    SurfelPointInfo pntSurfEl;
    if(EstimateNormal(pnt, pntSurfEl)){
      surfelCloud.cloud.push_back(std::move(pntSurfEl));
    }
  }
}
void SurfelExtraction::LineNNSearch( const int ring, const double query, int &row, Eigen::MatrixXd& neighbours){

  NNSearchArray NNSearch;
  const int scans_par_line = 1800;
  float hor_time_res = 6*scan_period_/scans_par_line; // max 6 points from center
  //cout << "max tdiff - not implemented yet: " << scans_par_line << std::endl;
  std::vector<int> indicies = NNSearch.findClosestElements(times_[ring], 5, hor_time_res, query);
  //cout << "nearby: " << indicies.size() << endl;
  if(indicies.size() > 0){
    for(int first_row = row; row< first_row + indicies.size() ; row++){ // i is row in matrix
      //cout << i - first_row << endl;
      const int idx = indicies[row - first_row]; // zero index
      //cout << "idx" << idx << endl;
      const Eigen::Vector3d  pntNear(ringClouds_[ring]->points[idx].x, ringClouds_[ring]->points[idx].y, ringClouds_[ring]->points[idx].z);
      neighbours.block<1,3>(row,0) = pntNear;
      //cout << "neigbour " << neighbours.block<1,3>(i,0).transpose() << endl;
    }
  }
}
bool SurfelExtraction::GetNeighbours(const vel_point::PointXYZIRTC& pnt, Eigen::MatrixXd& neighbours){
  const int ring = pnt.ring;
  const double time = pnt.time;
  std::vector<int> search;
  // not last ring
  neighbours = Eigen::MatrixXd(15,3);
  int first = 0;

  ///cout << "ring: "<< ring << endl;
  //cout << "dim: " << neighbours.rows() << " x " << neighbours.cols() << endl;
  LineNNSearch(ring, time,first, neighbours);
  //cout << "dim: " << neighbours.rows() << " x " << neighbours.cols() << endl;
  if(ring < n_scan_lines_ - 1){
    LineNNSearch(ring+1, time, first, neighbours);
  }
  //cout << "dim: " << neighbours.rows() << " x " << neighbours.cols() << endl;
  // not first ring
  if(ring > 0 ){
    LineNNSearch(ring-1, time, first, neighbours);
  }
  neighbours.conservativeResize(first,3);
  //cout << "dim: " << neighbours.rows() << " x " << neighbours.cols() << endl;

  return true;


}
bool SurfelExtraction::EstimateNormal(const vel_point::PointXYZIRTC& pnt, SurfelPointInfo& surfel){

  //cout << "EstimateNormal" << endl;
  Eigen::MatrixXd X; //neighbours
  const bool statusOK = GetNeighbours(pnt, X); // 3 x Nsamples
  if(!statusOK){
    return false;
  }
  /*pcl::PointCloud<pcl::PointXYZ> cloud, cloud_pnt;

  for(int i = 0 ; i <X.rows() ; i++){
    pcl::PointXYZ p(X(i,0), X(i,1), X(i,2));
    cloud.push_back(p);
  }
  cout << X << endl;
  cout <<"rows: " <<  X.rows() << endl;
  pcl::PointXYZ pnt_xyz(pnt.x, pnt.y, pnt.z);
  cloud_pnt.push_back(pnt_xyz);
  PublishCloud("surf", *surf_in_, "base_link", ros::Time::now() );
  PublishCloud("center", cloud_pnt, "base_link", ros::Time::now() );
  PublishCloud("neighbours", cloud, "base_link", ros::Time::now() );
  */

  //PublishCloud(const std::string& topic, Cloud& cloud, const std::string& frame_id, const ros::Time& t);
  const int Nsamples = X.rows();
  Eigen::Matrix<double,1,3> mean(0,0,0);  // 3 x 1

  for(Eigen::Index i=0 ; i<Nsamples ; i++)
    mean += X.block<1,3>(i,0); // compute sum
  mean/=Nsamples;

  for(Eigen::Index i=0 ; i<Nsamples ; i++) // subtract mean
    X.block<1,3>(i,0) = X.block<1,3>(i,0) - mean;

  const Eigen::Matrix3d cov = 1.0/(Nsamples - 1.0)*X.transpose()*X;
  Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(cov);


  const float l1 = std::sqrt(es.eigenvalues()[0]);  // l1 < l2 < l3
  const float l2 = std::sqrt(es.eigenvalues()[1]);
  const float l3 = std::sqrt(es.eigenvalues()[2]);
  const float planarity = 1 - (l1 + l2)/ (l1 + l2 + l3); // this should be it when l1 -> 0  & l2/l3 is high  planarity -> 1 if l3 >> l1+l2

  Eigen::Vector3d normal = es.eigenvectors().col(0);

  Eigen::Matrix <double, 3, 1> vp (pnt.x, pnt.y, pnt.z);
  if(vp.dot(normal)> 0) // when looking at point from origin, the normal should not look back at you
    normal *=-1;

  surfel.centerPoint = Eigen::Vector3d(pnt.x, pnt.y, pnt.z);
  //surfel.mean = mean;
  surfel.l3 = l3;
  //surfel.cov = cov;
  surfel.nSamples = Nsamples;
  surfel.planarity = planarity; // this should be it when l1 -> 0  & l2/l3 is high  planarity -> 1 if l3 >> l1+l2
  surfel.normal = normal;
  surfel.entropy = 0.5*log(1 + 2*M_PI*M_E*cov.determinant());
  surfel.intensity = pnt.intensity;
  surfel.time = pnt.time;
  surfel.curvature = pnt.curvature;
  //surfel.intensity = planarity;
  return true;
}

Eigen::Isometry3d EigenCombine(const Eigen::Quaterniond& q, const Eigen::Vector3d& transl){
  Eigen::Matrix4d prediction = Eigen::Matrix4d::Zero();
  Eigen::Matrix4d pred_mat;
  pred_mat.setIdentity();
  pred_mat.block<3,3>(0,0) = q.toRotationMatrix();
  pred_mat.block<3,1>(0,3) = transl;
  return Eigen::Isometry3d(pred_mat);
}




