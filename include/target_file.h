/*! \class      TargetFile
 *  \brief      Methods to add and delete target drill locations for drilling and defastening
 *  \date       December 6, 2022
 *  \author     Michelle Crivella
 *  \copyright  BOEING PROPRIETARY, CONFIDENTAL, AND/OR TRADE SECRET, COPYRIGHT 2021/ THE BOEING COMPANY,  UNPUBLISHED WORK.  ALL RIGHTS RESERVED.
*/

#ifndef TargetFile_H
#define TargetFile_H

#include <string>
#include <set>
#include <vector>
#include <geometry_msgs/Pose.h>
#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <ros/ros.h>

namespace FDS_Interfaces
{
    enum TargetType
    {
        COUNTERSINK,
        DEFASTEN,
        LOCALIZATION,
    };

    enum TargetStatus
    {
        NOT_COMPLETED,
        SKIP,
        COMPLETED,
    };

    static const std::string DETECTED_TARGET_FILE_NAME = "detected_targets.yaml";

    class A_Target
    {
        public:
            A_Target(std::string id, int num, FDS_Interfaces::TargetStatus status, KDL::Frame target_frame, FDS_Interfaces::TargetType type, bool manually_added, std::string image_file, std::pair<int, int> pix_coordinates, double diameter, std::vector<double> joint_states, geometry_msgs::Pose fds_tcp_frame_to_base)
            {
                task_id_ = id;
                task_num_ = num;
                target_frame_ = target_frame;
                type_ = type;
                manually_added_ = manually_added;
                image_file_ = image_file;
                pixel_coordinates_ = pix_coordinates;
                diameter_ = diameter;
                joint_states_ = joint_states;
                fds_tcp_frame_to_base_ = fds_tcp_frame_to_base;
                status_ = status;
            }
            A_Target()
            {

            }

            ~A_Target()   {  }

            std::string task_id_;
            int task_num_;
            FDS_Interfaces::TargetStatus status_;
            std::string image_file_;
            bool manually_added_;
            KDL::Frame target_frame_;
            FDS_Interfaces::TargetType type_;
            std::pair <int, int> pixel_coordinates_;
            double diameter_;
            std::vector<double> joint_states_;
            geometry_msgs::Pose fds_tcp_frame_to_base_;
    };

    class TargetData
    {
        public:
            TargetData(bool realMode, ros::NodeHandle& pnh);
            ~TargetData();

            std::vector<A_Target> target_array_;

            std::vector<float> HOLE_TARGET_COLOR_RGB{255.0, 0.0, 255.0};               // purple
            std::vector<float> DEFASTEN_TARGET_COLOR_RGB{255.0, 255.0, 0.0};           // yellow
            std::vector<float> LOCALIZATION_TARGET_COLOR_RGB{0.0, 255.0, 255.0};       // green
            std::vector<float> COMPLETED_TARGET_COLOR_RGB{255.0, 0.0, 0.0};            // red

            double Z_OFFSET_FOR_VISUALIZATION = 0.0127;             // .5 inches

            bool readTargetDataFromFile(std::string fname);
            bool writeTargetDataToFile(std::string fname);
            bool deleteTargetfile(std::string fname);
            bool updateTargetStatus(std::string id, FDS_Interfaces::TargetStatus status);
            void addToTargetList(std::string id, int num, FDS_Interfaces::TargetStatus status, bool manually_added, KDL::Frame target, FDS_Interfaces::TargetType cycle_type, std::string image, std::pair<int, int> xy_pixels, double target_diameter, std::vector<double> robot_states, geometry_msgs::Pose tcp_frame_to_base);
            void append(std::vector<A_Target> v);
            void clear();
            bool removeFromTargetList(std::string id);
            void get_targets_as_pointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& points);
            void pointcloud_to_targets_file(pcl::PointCloud<pcl::PointXYZ>::Ptr points);
            void visualizeTargetsAsMarkers(std::string frame);
            void visualizeTargetsAsFrames(std::string frame);
            void removeAllMarkers(std::string frame);
            std::vector<std::string> getTargetNames();
            std::string get_target_waypoint_str(std::string target_name);

        private:
            bool realMode_;
            ros::NodeHandle pnh_;

            ros::Publisher completed_pub_;
            ros::Publisher fastener_pub_;
            ros::Publisher hole_pub_;
            ros::Publisher localization_pub_;
            ros::Publisher task_name_pub_;
            ros::Publisher target_poses_pub_;
    };
}

#endif // TargetFile_H
