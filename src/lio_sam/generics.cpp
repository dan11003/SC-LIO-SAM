#include "lio_sam/generics.h"

namespace IO
{

    void createTransformedPointCloud(const pcl::PointCloud<PointType>::Ptr input, const Eigen::Vector3d &transform, pcl::PointCloud<PointType>::Ptr output)
    {
        if (input == nullptr)
        {
            std::throw_with_nested(std::runtime_error("Incorrect usage - Input cloud is null"));
        }
        else if (output == nullptr)
        {
            output = pcl::PointCloud<PointType>::Ptr(pcl::PointCloud<PointType>().makeShared());
            output->header = input->header;
        }
        output->resize(input->size());
        for (int i = 0; i < input->size(); i++)
        {
            output->points[i].x = input->points[i].x + transform(0);
            output->points[i].y = input->points[i].y + transform(1);
            output->points[i].z = input->points[i].z + transform(2);
            output->points[i].intensity = input->points[i].intensity;
            output->points[i].ring = input->points[i].ring;
            output->points[i].time = input->points[i].time;
            output->points[i].curvature = input->points[i].curvature;
        }
    }
    void SaveMerged(const std::vector<pcl::PointCloud<PointType>::Ptr> clouds, const std::vector<Eigen::Affine3d> poses, const Eigen::Vector3d &datum_offset, const std::string &directory, double downsample_size)
    {
        boost::filesystem::create_directories(directory);
        pcl::PointCloud<PointType>::Ptr merged(pcl::PointCloud<PointType>().makeShared());
        pcl::PointCloud<PointType>::Ptr merged_transformed(pcl::PointCloud<PointType>().makeShared());
        pcl::PointCloud<PointType>::Ptr merged_downsamapled(pcl::PointCloud<PointType>().makeShared());
        pcl::PointCloud<PointType>::Ptr merged_downsamapled_transformed(pcl::PointCloud<PointType>().makeShared());

        std::cout << "\"SLAM\" - Saving point cloud to: " << directory << std::endl;

        for (int i = 0; i < poses.size(); i++) // build local cloud
        {
            pcl::PointCloud<PointType> tmp_transformed;
            pcl::transformPointCloud(*clouds[i], tmp_transformed, poses[i]);
            *merged += tmp_transformed;
        }
        pcl::io::savePCDFileBinary(directory + std::string("sc-lio-sam.pcd"), *merged);

        /*if (datum_offset.norm() > 0.01)
        {
            cout << "Save with datum offset: " << datum_offset.transpose() << endl;
            createTransformedPointCloud(merged, datum_offset, merged_transformed);
            // 10 points from the merged_transformed
            pcl::io::savePCDFileBinary(directory + std::string("sc-lio-sam_sweref.pcd"), *merged_transformed);
            //pcl::io::savePCDFileASCII( directory + std::string("ascii_sc-lio-sam_sweref.pcd"), *merged_transformed);
        }*/

        /*cout << "\"SLAM\" - Downsample point cloud resolution " << downsample_size << endl;
        pcl::VoxelGrid<PointType> sor;
        sor.setInputCloud(merged);
        sor.setLeafSize(downsample_size, downsample_size, downsample_size);
        sor.filter(*merged_downsamapled);

        if (!merged_downsamapled->empty())
        {
            const std::string path_ds = directory + std::string("sc-lio-sam_merged_downsampled_leaf_") + std::to_string(downsample_size) + "_local.pcd";
            pcl::io::savePCDFileBinary(path_ds, *merged_downsamapled);
            if (datum_offset.norm() > 0.01)
            {
                createTransformedPointCloud(merged_downsamapled, datum_offset, merged_downsamapled_transformed);
                const std::string path_ds_transformed = directory + std::string("sc-lio-sam_merged_downsampled_leaf_") + std::to_string(downsample_size) + "_sweref.pcd";
                pcl::io::savePCDFileBinary(path_ds_transformed, *merged_downsamapled_transformed);
                //pcl::io::savePCDFileASCII("ascii_" + path_ds_transformed, *merged_downsamapled_transformed);
            }
        }
        else
            std::cout << "\"SLAM\" - No downsampled point cloud saved - increase \"output_downsample_size\"" << std::endl;
        */
    }

    void SaveOdom(
        const std::string &dump_directory,
        const std::vector<Eigen::Affine3d> &poses,
        const std::vector<double> &keyframe_stamps,
        const std::vector<pcl::PointCloud<PointType>::Ptr> &clouds)
    {

        boost::filesystem::create_directories(dump_directory);

        std::cout << "\"SLAM\" - Save odom to: " << dump_directory << std::endl
                  << std::endl;
        // std::cout << "Save clouds: " << clouds.size() << std::endl;
        for (int i = 0; i < clouds.size(); i++)
        {

            ros::Time t(keyframe_stamps[i]);
            std::string filename = dump_directory + str(boost::format("/%lf_%lf") % t.sec % t.nsec);

            pcl::io::savePCDFileBinary(filename + ".pcd", *clouds[i]);

            std::ofstream data_ofs(filename + ".odom");
            Eigen::Matrix<double, 4, 4> mat = poses[i].matrix();
            data_ofs << mat(0, 0) << " " << mat(0, 1) << " " << mat(0, 2) << " " << mat(0, 3) << std::endl;
            data_ofs << mat(1, 0) << " " << mat(1, 1) << " " << mat(1, 2) << " " << mat(1, 3) << std::endl;
            data_ofs << mat(2, 0) << " " << mat(2, 1) << " " << mat(2, 2) << " " << mat(2, 3) << std::endl;
            data_ofs << mat(3, 0) << " " << mat(3, 1) << " " << mat(3, 2) << " " << mat(3, 3) << std::endl;
            data_ofs.close();
        }
    }
    void SaveBALM2(
        const std::string &dump_directory,
        const std::vector<Eigen::Affine3d> &poses,
        const std::vector<double> &keyframe_stamps,
        const std::vector<pcl::PointCloud<PointType>::Ptr> &clouds)
    {

        boost::filesystem::create_directories(dump_directory);
        boost::filesystem::create_directories(dump_directory + "/pcd");

        std::cout << "\"SLAM\" - Save odom to: " << dump_directory << std::endl
                  << std::endl;
        // std::cout << "Save clouds: " << clouds.size() << std::endl;
        const std::string poseFilename = dump_directory + "pose.json";
        std::ofstream data_ofs(poseFilename);
        for (int i = 0; i < clouds.size(); i++)
        {
            std::stringstream ss;

            ss << std::setw(5) << std::setfill('0') << i;

            pcl::io::savePCDFileBinary(dump_directory + "/pcd/" + ss.str() + ".pcd", *clouds[i]);
            const Eigen::Vector3d trans = poses[i].translation();
            Eigen::Quaterniond q(poses[i].linear());
            data_ofs << std::fixed << std::setprecision(9) << trans(0) << " " << trans(1) << " " << trans(2) << " " << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << std::endl;
        }
        data_ofs.close();
    }
    
    void SaveImages(const std::string &directory,
        const std::vector<Eigen::Affine3d> &poses,
        const std::vector<double> &keyframe_stamps,
        std::map<int,sensor_msgs::CompressedImage> &images)
    {
        boost::filesystem::create_directories(directory);
        const std::string cam_filename = directory + "cameraPoses.csv";
        std::fstream cam_stream(cam_filename.c_str(), std::fstream::out);

        std::cout << "\"SLAM\" - Save Images and Poses to: " << directory << std::endl;
        //File	Time	Long	Lat	Alt	course	pitch	roll

        cam_stream << "File\tTime\tLong\tLat\tAlt\tcourse\tpitch\troll" << std::endl;
        //iterate through all images
        int img_nr = 0;
        for(const auto &[idx,comprImage] : images){
            img_nr++;
            //get pose
            if(idx >= poses.size()){
                std::throw_with_nested(std::runtime_error("Incorrect usage - idx out of bounds"));
                return;
            }
            const Eigen::Affine3d pose = poses[idx];
            const Eigen::Vector3d euler = pose.linear().eulerAngles(2,1,0)*180.0/M_PI;
            const double time = keyframe_stamps[idx];
          
            //save image
            const std::string img_name = std::to_string(img_nr) + ".jpg";
            const std::string imgPath = directory + img_name;
            std::ofstream imgFile(imgPath, std::ios::out | std::ios::binary);
            imgFile.write((char*)&comprImage.data[0], comprImage.data.size());
            imgFile.close();
            //save pose with 5 decimals precision, non scientific
            cam_stream << std::fixed << std::setprecision(5) << std::quoted(img_name) << "\t" << time << "\t" << pose.translation().x() << "\t" << pose.translation().y() << "\t" << pose.translation().z() << "\t" << euler(0) << "\t" << euler(1) << "\t" << euler(2) << std::endl;
        }
        cam_stream.close();

        const std::string pose_filename = directory + "allPoses.csv";
        std::fstream all_stream(pose_filename.c_str(), std::fstream::out);
        for(int i = 0; i < poses.size(); i++){
            const Eigen::Affine3d pose = poses[i];
            const Eigen::Vector3d euler = pose.linear().eulerAngles(2,1,0)*180.0/M_PI;;
            const double time = keyframe_stamps[i];
            all_stream <<  std::fixed << std::setprecision(5) << pose.translation().x() << "" << pose.translation().y() << "," << pose.translation().z() << "," << euler(0) << "," << euler(1) << "," << euler(2) << std::endl;
        }
        all_stream.close();
        

    
    }

    void SavePosesHomogeneousBALM(const std::vector<pcl::PointCloud<PointType>::Ptr> &clouds, const std::vector<Eigen::Affine3d> &poses, const std::string &directory, double downsample_size)
    {
        boost::filesystem::create_directories(directory);
        const std::string filename = directory + "alidarPose.csv";
        std::fstream stream(filename.c_str(), std::fstream::out);
        std::cout << "\"SLAM\" - Save BALM to: " << directory << std::endl
                  << std::endl;
        /*std::cout << "Saving clouds size: " <<clouds.size() << std::endl;;
        std::cout << "Saving poses size: " SavePosesHomogeneousBALM<<poses.size() << std::endl;*/
        pcl::PointCloud<PointType>::Ptr merged_transformed(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType> merged_downsamapled;

        for (int i = 0; i < poses.size(); i++)
        {
            ros::Time tRos;
            pcl_conversions::fromPCL(clouds[i]->header.stamp, tRos);
            const double time = tRos.toSec();
            const Eigen::MatrixXd m = poses[i].matrix();

            stream << std::fixed << m(0, 0) << "," << m(0, 1) << "," << m(0, 2) << "," << m(0, 3) << "," << endl
                   << m(1, 0) << "," << m(1, 1) << "," << m(1, 2) << "," << m(1, 3) << "," << endl
                   << m(2, 0) << "," << m(2, 1) << "," << m(2, 2) << "," << m(2, 3) << "," << endl
                   << m(3, 0) << "," << m(3, 1) << "," << m(3, 2) << "," << time << "," << endl;
            const std::string pcdPath = directory + std::string("full") + std::to_string(i) + ".pcd";
            pcl::io::savePCDFileBinary(pcdPath, *clouds[i]);
        }
    }

    std::string CreateFolder(const std::string &basePath, const std::string &prefix, const std::string &callerName)
    {

        auto t = std::time(nullptr);
        auto tm = *std::localtime(&t);

        // const std::string timeStr(std::put_time(&tm, "%Y-%m-%d_%H-%M"));

        std::time_t now = std::time(NULL);
        std::tm *ptm = std::localtime(&now);
        char buffer[32];
        // Format: Mo, 15.06.2009 20:20:00
        std::strftime(buffer, 32, "%a_%Y.%m.%d_%H:%M:%S", ptm);

        const std::string dir = basePath + "/" + prefix + "_" + std::string(buffer) + std::string("/");

        if (boost::filesystem::create_directories(dir))
        {
            std::cout << "\"" << callerName << "\" - Created new output directory: " << dir << std::endl;
        }
        return dir;
    }

    void saveSCD(std::string fileName, Eigen::MatrixXd matrix, std::string delimiter)
    {

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

sensor_msgs::Imu imuConverter(const sensor_msgs::Imu &imu_in, const Eigen::Quaterniond &extQRPY)
{
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

    if (sqrt(q_final.x() * q_final.x() + q_final.y() * q_final.y() + q_final.z() * q_final.z() + q_final.w() * q_final.w()) < 0.1)
    {
        ROS_ERROR("Invalid quaternion, please use a 9-axis IMU!");
        ros::shutdown();
    }

    return imu_out;
}

pcl::PointCloud<pcl::PointXYZ>::Ptr ToCloudXYZ(pcl::PointCloud<PointType>::Ptr cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>());
    output->header = cloud->header;
    output->resize(cloud->size());
    for (int i = 0; i < cloud->size(); i++)
    {
        output->points[i] = ToXYZ(cloud->points[i]);
    }
    return output;
}

Eigen::Isometry3d vectorToAffine3d(double x, double y, double z, double ex, double ey, double ez)
{
    return Eigen::Translation<double, 3>(x, y, z) *
           Eigen::AngleAxis<double>(ex, Eigen::Vector3d::UnitX()) *
           Eigen::AngleAxis<double>(ey, Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxis<double>(ez, Eigen::Vector3d::UnitZ());
}

bool PoseGraph::LoadGraph(const std::string &path, boost::shared_ptr<PoseGraph> &graph)
{
    try
    {
        std::ifstream ifs(path);
        boost::archive::binary_iarchive ia(ifs);
        ia >> graph;
        ifs.close();
        cout << "Graph (" << graph->ToString() << ")succesfully loaded from: " << path << endl;
        return true;
    }
    catch (std::exception &e)
    {
        std::cerr << "Graph could not be loaded from: " << path << std::endl;
        return false;
    }
}

void PoseGraph::SaveGraph(const std::string &path, boost::shared_ptr<PoseGraph> &graph)
{
    if (graph == NULL)
    {
        std::cout << "Nothing to save" << endl;
        return;
    }
    try
    {
        std::cout << "Save graph to: " << path << endl;
        std::ofstream ofs(path);
        boost::archive::binary_oarchive oa(ofs);
        oa << graph;
        ofs.close();
    }
    catch (std::exception &e)
    {
    }
}

NormalCloud::Ptr SurfElCloud::GetPointCloud(int intensity) const
{
    NormalCloud::Ptr output(new NormalCloud());
    for (auto &&surfEl : cloud)
    {
        pcl::PointXYZINormal pnt;
        pnt.curvature = surfEl.time;
        pnt.x = surfEl.centerPoint(0);
        pnt.y = surfEl.centerPoint(1);
        pnt.z = surfEl.centerPoint(2);
        pnt.normal_x = surfEl.normal(0);
        pnt.normal_y = surfEl.normal(1);
        pnt.normal_z = surfEl.normal(2);
        if (intensity == 0)
            pnt.intensity = surfEl.intensity;
        else if (intensity == 1)
            pnt.intensity = surfEl.curvature;
        else if (intensity == 2)
            pnt.intensity = surfEl.nSamples;
        else if (intensity == 3)
            pnt.intensity = surfEl.entropy;
        else if (intensity == 4)
            pnt.intensity = surfEl.planarity;

        output->push_back(std::move(pnt));
    }
    return output;
}
std::vector<double> SurfElCloud::GetPointCloudTime() const
{
    std::vector<double> stamps(cloud.size());
    for (int i = 0; i < cloud.size(); i++)
    {
        stamps[i] = cloud[i].time;
    }
    return stamps;
}

SurfElCloud SurfElCloud::Transform(const Eigen::Isometry3d &transform) const
{
    SurfElCloud SurfElTransformed = *this;
    for (auto itr = SurfElTransformed.begin(); itr != SurfElTransformed.end(); itr++)
    {
        itr->centerPoint = transform * itr->centerPoint;
        itr->normal = transform.linear() * itr->normal;
    }
    return SurfElTransformed;
}

Eigen::Isometry3d EigenCombine(const Eigen::Quaterniond &q, const Eigen::Vector3d &transl)
{
    Eigen::Matrix4d prediction = Eigen::Matrix4d::Zero();
    Eigen::Matrix4d pred_mat;
    pred_mat.setIdentity();
    pred_mat.block<3, 3>(0, 0) = q.toRotationMatrix();
    pred_mat.block<3, 1>(0, 3) = transl;
    return Eigen::Isometry3d(pred_mat);
}
