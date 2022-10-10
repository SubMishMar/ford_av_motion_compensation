#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

#include <rosbag/bag.h>

#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/PointCloud2.h>

#include <string>
#include <iostream>
#include <experimental/filesystem>
#include <set>
#include <utility>
#include <fstream>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <typeinfo>
#include "pcl_ros/point_cloud.h"
#include "pcl_conversions/pcl_conversions.h"

#include "Eigen/Dense"
#include "map"

using namespace std;
namespace fs = std::experimental::filesystem;

class MotionCompensator {
private:
    std::string pcd_folder_name_;
    std::string pose_file_name_;
    std::map<long double, std::pair<Eigen::Quaterniond, Eigen::Vector3d>> pose_timestamped_;
    ros::NodeHandle nh_;
    ros::Publisher unCompensatedCloudPublisher_;
    ros::Publisher compensatedCloudPublisher_;
    Eigen::Matrix4d lidarBodyExtrinsicCalibration = Eigen::Matrix4d::Identity();
public:
    MotionCompensator(ros::NodeHandle& nh, string& pcd_folder_name, string& pose_file_name){
        nh_ = nh;
        pcd_folder_name_ = std::move(pcd_folder_name);
        pose_file_name_ = std::move(pose_file_name);
        unCompensatedCloudPublisher_ = nh_.advertise<sensor_msgs::PointCloud2>("/uncompensated_cloud", 1);
        compensatedCloudPublisher_ = nh_.advertise<sensor_msgs::PointCloud2>("/compensated_cloud", 1);

        /// TODO: Write a yaml parser for loading the extrinsic calib b/w pose file and lidar data
        Eigen::Quaterniond quaternion_excal = Eigen::Quaterniond (0.003382, -0.705243, -0.708949, 0.003492);
        Eigen::Matrix3d rotation_excal = Eigen::Matrix3d(quaternion_excal);
        Eigen::Vector3d translation_excal = Eigen::Vector3d(1.11228, 0.369431, -1.41456);
        lidarBodyExtrinsicCalibration.block(0, 0, 3, 3) = rotation_excal;
        lidarBodyExtrinsicCalibration.block(0, 3, 3, 1) = translation_excal;

        ///
        readPoseFile();
        readAndCompensatePCDScans();
    }

    template<typename Map> typename Map::iterator greatest_less(Map & m, typename Map::key_type const& k) {
        auto it = m.lower_bound(k);
        if(it != m.begin()) {
            return --it;
        }
        return m.end();
    }

    Eigen::Matrix4d getScan_T_Point(const std::pair<Eigen::Quaterniond, Eigen::Vector3d>& w_pose_scan,
                                    const std::pair<Eigen::Quaterniond, Eigen::Vector3d>& w_pose_point){
        Eigen::Quaterniond w_quat_scan = w_pose_scan.first;
        Eigen::Vector3d w_position_scan = w_pose_scan.second;

        Eigen::Quaterniond w_quat_point = w_pose_point.first;
        Eigen::Vector3d w_position_point = w_pose_point.second;

        Eigen::Quaterniond scan_quat_point = w_quat_scan.inverse()*w_quat_point;
        Eigen::Vector3d scan_position_point = w_quat_scan.inverse()*(w_position_point - w_position_scan);

        Eigen::Matrix3d scan_R_point = Eigen::Matrix3d(scan_quat_point);

        Eigen::Matrix4d scan_T_point = Eigen::Matrix4d::Identity();
        scan_T_point.block(0, 0, 3, 3) = scan_R_point;
        scan_T_point.block(0, 3, 3,1 ) = scan_position_point;

        return scan_T_point;
    }

    std::pair<Eigen::Quaterniond, Eigen::Vector3d> getInterpolatedPose(std::pair<Eigen::Quaterniond, Eigen::Vector3d> pose_start,
                                                                       long double timestamp_start,
                                                                       std::pair<Eigen::Quaterniond, Eigen::Vector3d> pose_end,
                                                                       long double timestamp_end,
                                                                       long double timestamp_query){
        if(timestamp_end <= timestamp_start){
            /// TODO: return a warning
        }
        if(timestamp_query <= timestamp_start || timestamp_query >= timestamp_end){
            /// TODO: return a warning
        }
        long double alpha = (timestamp_query - timestamp_start) / (timestamp_end - timestamp_start);
        Eigen::Vector3d translation_interpolated = (1 - alpha)*pose_start.second + alpha*pose_end.second;
        Eigen::Quaterniond  quaternion_interpolated = pose_start.first.slerp(alpha, pose_end.first).normalized();
        return {quaternion_interpolated, translation_interpolated};
    }

    std::pair<Eigen::Quaterniond, Eigen::Vector3d> getInterpolatedPose(long double key){
        auto it_lb = greatest_less(pose_timestamped_, key);
        auto it_ub = pose_timestamped_.lower_bound(key); /// greater than or equal to key
        std::pair<Eigen::Quaterniond, Eigen::Vector3d> pose_start = it_lb->second;
        long double timestamp_start = it_lb->first;
        std::pair<Eigen::Quaterniond, Eigen::Vector3d> pose_end = it_ub->second;
        long double timestamp_end = it_ub->first;
        long double timestamp_query = key;
        std::pair<Eigen::Quaterniond, Eigen::Vector3d> interpolated_pose_scan;
        if(pose_timestamped_.find(key) == pose_timestamped_.end()) {
            interpolated_pose_scan = getInterpolatedPose(pose_start, timestamp_start, pose_end, timestamp_end, timestamp_query);
        } else {
            interpolated_pose_scan = pose_timestamped_[key];
        }
        return interpolated_pose_scan;
    }

    void readPoseFile(){
        std::ifstream csvFileReader;
        csvFileReader.open(pose_file_name_.c_str());
        if (!csvFileReader.is_open()) {
            std::cout << "Wrong Path!" << std::endl;
        }
        std::string line;
        while (getline(csvFileReader, line)) {
            if (line.empty()) // skip empty lines:
            {
                std::cout << "empty line!" << std::endl;
                continue;
            }
            std::istringstream iss(line);
            std::string lineStream;
            std::string::size_type sz;

            std::vector <long double> row;

            while (getline(iss, lineStream, ',')) {
                try {
                    row.push_back(stold(lineStream,&sz)); // convert to double
                }
                catch (const std::invalid_argument&) {
//                    std::cerr << "No conversion could be performed" << std::endl;
                }
                catch (const std::out_of_range&) {
//                    std::cerr << "Could not convert string to long double, value falls out of range" << std::endl;
                }
            }
            if(row.size()!=11){
                continue;
            }
            long double timestamp = row[0];
            long double seq = row[1];
            long double sec = row[2];
            long double nsec = row[3];
            Eigen::Vector3d position = Eigen::Vector3d(row[4], row[5], row[6]);
            long double qx = row[7];
            long double qy = row[8];
            long double qz = row[9];
            long double qw = row[10];
            Eigen::Quaterniond orientation = Eigen::Quaterniond(qw, qx, qy, qz).normalized();
            pose_timestamped_[timestamp] = {orientation, position};
        }
    }

    pcl::PointCloud<pcl::PointXYZINormal>::Ptr compensateScan(const pcl::PointCloud<pcl::PointXYZINormal>::Ptr& cloud, long double key){
        std::pair<Eigen::Quaterniond, Eigen::Vector3d> interpolated_pose_scan = getInterpolatedPose(key);
        pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_out (new pcl::PointCloud<pcl::PointXYZINormal>);
        for (const auto& point: *cloud) {
            double point_x = point.x;
            double point_y = point.y;
            double point_z = point.z;
            double intensity = point.normal_x;
            double ring = point.normal_y;
            long double t = point.normal_z*1000000000.; // time
            long double t_from_scan_ts = key + t;
            std::pair<Eigen::Quaterniond, Eigen::Vector3d> interpolated_pose_point = getInterpolatedPose(t_from_scan_ts);
            Eigen::Matrix4d scan_T_point = lidarBodyExtrinsicCalibration.inverse()*getScan_T_Point(interpolated_pose_scan, interpolated_pose_point)*lidarBodyExtrinsicCalibration;
            Eigen::Vector4d X_compensated = scan_T_point*Eigen::Vector4d(point_x, point_y, point_z, 1);

            pcl::PointXYZINormal ptXYZINormal;
            ptXYZINormal.x = X_compensated.x();
            ptXYZINormal.y = X_compensated.y();
            ptXYZINormal.z = X_compensated.z();
            ptXYZINormal.normal_x = intensity;
            ptXYZINormal.normal_y = ring;
            ptXYZINormal.normal_z = point.normal_z;
            cloud_out->push_back(ptXYZINormal);
        }
        cloud_out->header = cloud->header;
        cloud_out->height = cloud->height;
        cloud_out->width = cloud->width;
        cloud_out->is_dense = cloud->is_dense;
        return cloud_out;
    }

    ros::Time getRosTime(long double timestamp){
        long timestamp_long = long(timestamp);
        long timestamp_seconds = timestamp_long/1000000000;
        long timestamp_nanoseconds = timestamp_long%1000000000;
        ros::Time timestamp_ros;
        timestamp_ros.sec = uint32_t(timestamp_seconds);
        timestamp_ros.nsec = uint32_t(timestamp_nanoseconds);
        return timestamp_ros;
    }

    void publishCloud(pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud, ros::Publisher& cloud_pub, long double timestamp){
        sensor_msgs::PointCloud2 ros_cloud;

        pcl::toROSMsg(*cloud, ros_cloud);
        ros_cloud.header.frame_id = "lidar";
        ros_cloud.header.stamp = getRosTime(timestamp);
        cloud_pub.publish(ros_cloud);
    }

    void readAndCompensatePCDScans(){
        set<fs::path> sorted_by_name;
        for (const auto & entry : fs::directory_iterator(pcd_folder_name_))
            sorted_by_name.insert(entry.path());

        //--- print the files sorted by filename
        for (auto &filename : sorted_by_name) {
            const std::string& pcd_filename = filename;
            std::string base_filename = pcd_filename.substr(pcd_filename.find_last_of("/\\") + 1);
            std::string::size_type const p(base_filename.find_last_of('.'));
            std::string file_without_extension = base_filename.substr(0, p);

            pcl::PointCloud<pcl::PointXYZINormal>::Ptr unCompensatedScan (new pcl::PointCloud<pcl::PointXYZINormal>);
            if (pcl::io::loadPCDFile<pcl::PointXYZINormal> (filename, *unCompensatedScan) == -1) {
                PCL_ERROR ("Couldn't read file pcd file\n");
                return;
            }
            long double key = std::stold(file_without_extension);
            publishCloud(unCompensatedScan, unCompensatedCloudPublisher_, key);
            pcl::PointCloud<pcl::PointXYZINormal>::Ptr compensatedScan = compensateScan(unCompensatedScan, key);
            publishCloud(compensatedScan, compensatedCloudPublisher_, key);
            sleep(1);
        }
    }
};

int main(int argc, char **argv){
    ros::init(argc, argv, "ford_av_motion_compensation");
    ros::NodeHandle nh("~");

    std::string pcd_folder_name = "/home/usl/Downloads/Sample_pcds_Subodh_2017-10-26-V2-Log1_200_frame_pcds/lidar_blue_scan";
    std::string pose_file_name = "/home/usl/Downloads/from-kunjan/pose_ground_truth.csv";

    MotionCompensator motionCompensator(nh, pcd_folder_name, pose_file_name);

    return 0;
}