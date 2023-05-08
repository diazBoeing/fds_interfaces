/*! \file       TargetFile.cpp
 *  @copydoc    TargetFile.h
*/
#include <algorithm>
#include <iostream>
#include <fstream>
#include <iomanip>
#include <set>

#include <yaml-cpp/yaml.h>
#include "ik_solution_file.h"
#include "target_file.h"

namespace FDS_Interfaces
{
    TargetData::TargetData(bool realMode, ros::NodeHandle& pnh)
        : realMode_(realMode)
        , pnh_(pnh)
        , completed_pub_(pnh_.advertise<visualization_msgs::MarkerArray>("completed_targets", 1, true))
        , fastener_pub_(pnh_.advertise<visualization_msgs::MarkerArray>("fastener_targets", 1, true))
        , hole_pub_(pnh_.advertise<visualization_msgs::MarkerArray>("hole_targets", 1, true))
        , localization_pub_(pnh_.advertise<visualization_msgs::MarkerArray>("localization_targets", 1, true))
        , task_name_pub_(pnh_.advertise<visualization_msgs::MarkerArray>("target_names", 1, true))
        , target_poses_pub_(pnh_.advertise<geometry_msgs::PoseArray>("targets_poses", 1, true))
    {

    }

    TargetData::~TargetData()
    {

    }

    bool TargetData::readTargetDataFromFile(std::string fname)
    {
        bool success = true;

        target_array_.clear();
        try
        {
            YAML::Node node = YAML::LoadFile(fname);
            //std::cout << "Found: " << node.size() << " FDS Task(s)" << std::endl;
            for (std::size_t i = 0; i < node.size(); i++)
            {
                std::string task_id = node[i]["task_id"].as<std::string>();
                int num = node[i]["task_num"].as<int>();
                int type = node[i]["type"].as<int>();
                int status = node[i]["status"].as<int>();
                bool manually_added = node[i]["manually_added"].as<bool>();
                std::vector<double> position = node[i]["task_position"].as<std::vector<double>>();
                std::vector<double> orientation = node[i]["task_orientation"].as<std::vector<double>>();
                KDL::Frame target_frame = FDS_Interfaces::IKSolutionFile::getKDLFrameFromVectors(position, orientation);
                std::string target_image = node[i]["image_file"].as<std::string>();
                std::vector<int> pix_array = node[i]["pixel_target"].as<std::vector<int>>();
                std::pair<int, int> pix_coords = std::make_pair(pix_array[0], pix_array[1]);
                double diameter = node[i]["diameter"].as<double>();
                std::vector<double> robot_joint_states = node[i]["robot_pose"].as<std::vector<double>>();
                std::vector<double> tcp_frame_position = node[i]["tcp_to_base_frame_position"].as<std::vector<double>>();
                std::vector<double> tcp_frame_orientation = node[i]["tcp_to_base_frame_rotation"].as<std::vector<double>>();

                geometry_msgs::Pose tcp_frame;
                tcp_frame.position.x = tcp_frame_position[0];
                tcp_frame.position.y = tcp_frame_position[1];
                tcp_frame.position.z = tcp_frame_position[2];
                tcp_frame.orientation.x = tcp_frame_orientation[0];
                tcp_frame.orientation.y = tcp_frame_orientation[1];
                tcp_frame.orientation.z = tcp_frame_orientation[2];
                tcp_frame.orientation.w = tcp_frame_orientation[3];

                addToTargetList(task_id, num, FDS_Interfaces::TargetStatus(status), manually_added,
                                target_frame, FDS_Interfaces::TargetType(type),
                                target_image, pix_coords, diameter,
                                robot_joint_states, tcp_frame);
            }
            std::cout << "FDS_Interfaces:ReadTargetFile:: number of targets = " << target_array_.size() << std::endl;
        }
        catch (std::exception& exc)
        {
            success = false;
            std::string emsg = "TargetFile: Unable to read in data from " + fname + ":" + exc.what();
            std::cout << emsg << std::endl;
        }

        return success;
    }

    bool TargetData::writeTargetDataToFile(std::string fname)
    {
        bool success = true;

        try
        {
            YAML::Emitter out;

            out << YAML::BeginSeq;
            for (unsigned int i=0; i<target_array_.size(); i++)
            {
                A_Target tar = target_array_.at(i);
                KDL::Frame target = tar.target_frame_;

                out << YAML::BeginMap;
                out << YAML::Key << "task_id" << YAML::Value << YAML::DoubleQuoted << tar.task_id_;
                out << YAML::Key << "task_num" << YAML::Value << tar.task_num_;
                out << YAML::Key << "type" << YAML::Value << tar.type_;
                out << YAML::Key << "manually_added" << YAML::Value << YAML::TrueFalseBool << tar.manually_added_;
                out << YAML::Key << "status" << YAML::Value << tar.status_;
                //std::cout << "Write target: " << tar.task_id_ << ", num = " << tar.task_num_ << std::endl;
                out << YAML::Key << "task_position" << YAML::Value << YAML::Flow << FDS_Interfaces::IKSolutionFile::getPositionVectorFromKDLFrame(target);
                out << YAML::Key << "task_orientation" << YAML::Value << YAML::Flow << FDS_Interfaces::IKSolutionFile::getOrientationVectorFromKDLFrame(target);
                out << YAML::Key << "image_file" << YAML::Value << YAML::DoubleQuoted << tar.image_file_;
                out << YAML::Key << "pixel_target" << YAML::Flow << std::vector<int> {tar.pixel_coordinates_.first, tar.pixel_coordinates_.second};
                out << YAML::Key << "diameter" << YAML::Value << tar.diameter_;
                out << YAML::Key << "robot_pose" << YAML::Value << YAML::Flow << tar.joint_states_;

                std::vector<double> tcp_position;
                tcp_position.push_back(tar.fds_tcp_frame_to_base_.position.x);
                tcp_position.push_back(tar.fds_tcp_frame_to_base_.position.y);
                tcp_position.push_back(tar.fds_tcp_frame_to_base_.position.z);
                std::vector<double> tcp_rotation;
                tcp_rotation.push_back(tar.fds_tcp_frame_to_base_.orientation.x);
                tcp_rotation.push_back(tar.fds_tcp_frame_to_base_.orientation.y);
                tcp_rotation.push_back(tar.fds_tcp_frame_to_base_.orientation.z);
                tcp_rotation.push_back(tar.fds_tcp_frame_to_base_.orientation.w);

                out << YAML::Key << "tcp_to_base_frame_position" << YAML::Value << YAML::Flow << tcp_position;
                out << YAML::Key << "tcp_to_base_frame_rotation" << YAML::Value << YAML::Flow << tcp_rotation;

                out << YAML::EndMap;
            }
            out << YAML::EndSeq;

            std::ofstream fout(fname.c_str());
            fout << out.c_str();
            fout.close();
        }
        catch (std::exception& e1)
        {
            success = false;
            std::string emsg = "Failed to save file:\n\texception:  " + std::string(e1.what());
            std::cout << emsg << std::endl;
        }
        return success;
    }

    bool TargetData::deleteTargetfile(std::string fname)
    {
        bool success = true;
        if (access(fname.c_str(), F_OK ) != -1)
        {
            int result = std::remove(fname.c_str());
            if (result < 0)
                success = false;
        }
        target_array_.clear();

        return success;
    }

    bool TargetData::updateTargetStatus(std::string id, FDS_Interfaces::TargetStatus status)
    {
        bool success = false;

        std::vector<A_Target>::iterator it;
        for (it = target_array_.begin(); it != target_array_.end(); it++)
            if (it->task_id_.compare(id)==0)
            {
                it->status_= status;
                success = true;
            }

        return success;
    }

    void TargetData::addToTargetList(std::string id, int num, FDS_Interfaces::TargetStatus status, bool manually_added, KDL::Frame target, FDS_Interfaces::TargetType cycle_type, std::string image, std::pair<int, int> xy_pixels, double target_diameter, std::vector<double> robot_states, geometry_msgs::Pose tcp_frame_to_base)
    {
        A_Target tar(id, num, status, target, FDS_Interfaces::TargetType(cycle_type), manually_added, image, xy_pixels, target_diameter, robot_states, tcp_frame_to_base);
        target_array_.push_back(tar);
    }

    bool TargetData::removeFromTargetList(std::string id)
    {
        std::vector<A_Target>::iterator it;
        for (it = target_array_.begin(); it != target_array_.end();)
        {
            if (it->task_id_.compare(id)==0)
               it = target_array_.erase(it);
            else
               it++;
        }
    }

    void TargetData::append(std::vector<A_Target> v)
    {
        for (unsigned int i=0; i < v.size(); i++)
            target_array_.push_back(v.at(i));
    }

    void TargetData::clear()
    {
        target_array_.clear();
    }

    void TargetData::get_targets_as_pointcloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& points)
    {
        for (unsigned int i=0; i<target_array_.size(); i++)
        {
            pcl::PointXYZ point;
            point.x = target_array_.at(i).target_frame_.p(0);
            point.y = target_array_.at(i).target_frame_.p(1);
            point.z = target_array_.at(i).target_frame_.p(2);
            points->push_back(point);
        }
    }

    void TargetData::pointcloud_to_targets_file(pcl::PointCloud<pcl::PointXYZ>::Ptr points)
    {
        geometry_msgs::Pose p;
        p.position.x = 0.0;
        p.position.y = 0.0;
        p.position.z = 0.0;
        p.orientation.x = 0.0;
        p.orientation.y = 0.0;
        p.orientation.z = 0.0;
        p.orientation.w = 1.0;

        for (unsigned int i=0; i<points->size(); i++)
        {
            pcl::PointXYZ pt = points->at(i);
            FDS_Interfaces::A_Target target_item;

            std::stringstream target_id;
            target_id << "FS_" << std::setw(4) << std::setfill('0') << (i+1);

            target_item.task_id_ = target_id.str();
            target_item.type_ = FDS_Interfaces::DEFASTEN;
            target_item.task_num_ = i+1;
            target_item.status_ = FDS_Interfaces::NOT_COMPLETED;
            target_item.manually_added_ = false;
            target_item.image_file_ = "";
            target_item.pixel_coordinates_.first = 0;
            target_item.pixel_coordinates_.second = 0;

            KDL::Frame fr;
            fr = KDL::Frame::Identity();
            fr.p(0) = pt.x;
            fr.p(1) = pt.y;
            fr.p(2) = pt.z;
            target_item.target_frame_ = fr;

            target_item.diameter_ = .25;
            target_item.joint_states_ = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
            target_item.fds_tcp_frame_to_base_ = p;

            target_array_.push_back(target_item);
        }

        writeTargetDataToFile(FDS_Interfaces::DETECTED_TARGET_FILE_NAME);
    }

    void TargetData::visualizeTargetsAsMarkers(std::string frame)
    {
        removeAllMarkers(frame);

        visualization_msgs::MarkerArray hole_marker_array;
        visualization_msgs::MarkerArray fastener_marker_array;
        visualization_msgs::MarkerArray localization_marker_array;
        visualization_msgs::MarkerArray completed_marker_array;
        visualization_msgs::MarkerArray task_names_array;

        for (unsigned int i=0; i<target_array_.size(); i++)
        {
            visualization_msgs::Marker marker;
            marker.header.frame_id = frame;
            marker.header.stamp = ros::Time();
            marker.ns = target_array_.at(i).task_id_;
            marker.id = target_array_.at(i).task_num_;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = target_array_.at(i).target_frame_.p(0);
            marker.pose.position.y = target_array_.at(i).target_frame_.p(1);
            marker.pose.position.z = target_array_.at(i).target_frame_.p(2) + Z_OFFSET_FOR_VISUALIZATION;
            double x, y, z, w;
            target_array_.at(i).target_frame_.M.GetQuaternion(x, y, z, w);
            marker.pose.orientation.x = x;
            marker.pose.orientation.y = y;
            marker.pose.orientation.z = z;
            marker.pose.orientation.w = w;
            marker.scale.x = 0.02;
            marker.scale.y = 0.02;
            marker.scale.z = 0.02;
            marker.color.a = 0.5;

            if (target_array_.at(i).status_ == FDS_Interfaces::TargetStatus::COMPLETED)
            {
                marker.color.r = COMPLETED_TARGET_COLOR_RGB[0];
                marker.color.g = COMPLETED_TARGET_COLOR_RGB[1];
                marker.color.b = COMPLETED_TARGET_COLOR_RGB[2];
                completed_marker_array.markers.push_back(marker);
            }
            else if (target_array_.at(i).type_ == FDS_Interfaces::TargetType::COUNTERSINK)
            {
                marker.color.r = HOLE_TARGET_COLOR_RGB[0];
                marker.color.g = HOLE_TARGET_COLOR_RGB[1];
                marker.color.b = HOLE_TARGET_COLOR_RGB[2];
                hole_marker_array.markers.push_back(marker);
            }
            else if (target_array_.at(i).type_ == FDS_Interfaces::TargetType::DEFASTEN)
            {
                marker.color.r = DEFASTEN_TARGET_COLOR_RGB[0];
                marker.color.g = DEFASTEN_TARGET_COLOR_RGB[1];
                marker.color.b = DEFASTEN_TARGET_COLOR_RGB[2];
                fastener_marker_array.markers.push_back(marker);
            }
            else if (target_array_.at(i).type_ == FDS_Interfaces::TargetType::LOCALIZATION)
            {
                marker.color.r = LOCALIZATION_TARGET_COLOR_RGB[0];
                marker.color.g = LOCALIZATION_TARGET_COLOR_RGB[1];
                marker.color.b = LOCALIZATION_TARGET_COLOR_RGB[2];
                localization_marker_array.markers.push_back(marker);
            }

            visualization_msgs::Marker task_name;
            task_name.header.frame_id = frame;
            task_name.header.stamp = ros::Time::now();
            task_name.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
            task_name.action = visualization_msgs::Marker::ADD;
            task_name.text = std::to_string(target_array_.at(i).task_num_);
            task_name.ns = target_array_.at(i).task_id_ + "_name";
            task_name.id = target_array_.at(i).task_num_;
            task_name.color.a = 1.0;
            task_name.color.r = 1.0;                // white
            task_name.color.g = 1.0;
            task_name.color.b = 1.0;
            task_name.scale.z = .02;
            task_name.pose.orientation.w = 1.0;
            task_name.pose.position.x = target_array_.at(i).target_frame_.p(0);
            task_name.pose.position.y = target_array_.at(i).target_frame_.p(1);
            task_name.pose.position.z = target_array_.at(i).target_frame_.p(2) + Z_OFFSET_FOR_VISUALIZATION;

            task_names_array.markers.push_back(task_name);
        }

        completed_pub_.publish(completed_marker_array);
        fastener_pub_.publish(fastener_marker_array);
        hole_pub_.publish(hole_marker_array);
        localization_pub_.publish(localization_marker_array);
        task_name_pub_.publish(task_names_array);
    }

    void TargetData::removeAllMarkers(std::string frame)
    {
        visualization_msgs::MarkerArray ma;
        visualization_msgs::Marker markerD;
        markerD.header.frame_id = frame;
        markerD.action = visualization_msgs::Marker::DELETEALL;
        ma.markers.push_back(markerD);

        completed_pub_.publish(ma);
        fastener_pub_.publish(ma);
        hole_pub_.publish(ma);
        localization_pub_.publish(ma);
        task_name_pub_.publish(ma);
    }

    void TargetData::visualizeTargetsAsFrames(std::string frame)
    {
        if (target_array_.size()> 0)
        {
            geometry_msgs::PoseArray poses;
            for (unsigned int i=0; i<target_array_.size(); i++)
            {
                KDL::Frame target_frame = target_array_.at(i).target_frame_;
                geometry_msgs::Pose pose_msg;
                tf::PoseKDLToMsg(target_frame, pose_msg);
                poses.poses.push_back(pose_msg);
            }

            if (!frame.empty())
            {
              poses.header.frame_id = frame;
              poses.header.stamp = ros::Time::now();

              target_poses_pub_.publish(poses);
            }
        }
    }

    std::vector<std::string> TargetData::getTargetNames()
    {
        std::vector<std::string> return_list;
        for (unsigned int i=0; i<target_array_.size(); i++)
            if (target_array_.at(i).type_ != FDS_Interfaces::LOCALIZATION)
                return_list.push_back(target_array_.at(i).task_id_);
        return return_list;
    }

    std::string TargetData::get_target_waypoint_str(std::string target_name)
    {
        for (unsigned int i=0; i<target_array_.size(); i++)
        {
            if (target_name.compare(target_array_.at(i).task_id_) == 0)
            {
                KDL::Frame wp = target_array_.at(i).target_frame_;
                std::stringstream s;
                s << "[";
                s << std::setw(5) << wp.p(0) << ", " << wp.p(1) << ", " << wp.p(2) << ", ";
                double x, y, z, w;
                wp.M.GetQuaternion(x, y, z, w);
                s << std::setw(5) << x << ", " << y << ", " << z << ", " << w;
                s << "]";
                return s.str();
            }
        }
    }

}
