/*! \file       waypoint_data_file.cpp
 *  @copydoc    waypoint_data_file.h
*/

#include <iostream>
#include <fstream>
#include <iomanip>
#include <utility>

#include <Eigen/Geometry>
#include <eigen_conversions/eigen_kdl.h>
#include <geometry_msgs/Pose.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_kdl/tf2_kdl.h>

#include "waypoint_data_file.h"

namespace FDS_Interfaces
{
    WaypointDataFile::WaypointDataFile()
    {

    }

    WaypointDataFile::WaypointDataFile(ros::NodeHandle& pn)
        : pnh_(pn)
        , waypoint_pub_(pnh_.advertise<geometry_msgs::PoseArray>("Waypoints", 1, true))
    {

    }

    WaypointDataFile::~WaypointDataFile()  {   }

    bool WaypointDataFile::write_WaypointDataFile(std::string filename, a5_sanding_process_planning::ToolPath tool_path)
    {
        bool success = true;

        //std::cout << "WaypointDataFile::write_WaypointDataFile: writing to " << filename_ << std::endl;

        try
        {

            if (a5_sanding_process_planning::serialize(filename, tool_path))
            {
              ROS_ERROR_STREAM("WaypointDataFile: Unable to serialize path from file: " << filename);
              return false;
            }

            filename_ = filename;
        }
        catch (std::exception& e1)
        {
            success = false;
            std::string emsg = "WaypointDataFile: Failed to save file:\n\texception:  " + std::string(e1.what());
            std::cout << emsg << std::endl;
        }

        return success;
    }

    bool WaypointDataFile::read_WaypointDataFile(std::string filename)
    {
        bool success = true;

        try
        {
            if (!a5_sanding_process_planning::deserialize(filename, tool_path_))
            {
              ROS_ERROR_STREAM("WaypointDataFile: Unable to deserialize path from file: " << filename);
              return false;
            }

            filename_ = filename;
        }
        catch (std::exception& e1)
        {
            success = false;
            std::string emsg = "WaypointDataFile: Failed to save file:\n\texception:  " + std::string(e1.what());
            std::cout << emsg << std::endl;
        }

        return success;
    }

    void WaypointDataFile::getWaypointData(a5_sanding_process_planning::ToolPath& tool_path)
    {
        tool_path = tool_path_;
    }

    void WaypointDataFile::getWaypointDataFlippedZs(a5_sanding_process_planning::ToolPath& tool_path, std::string axis)
    {
        tool_path = tool_path_;

        Eigen::Vector3d vec(0.0, 0.0, 0.0);
        if (axis.compare("x") == 0)
            vec[0] = 1.0;
        else        // axis = "y"
            vec[1] = 1.0;

        Eigen::AngleAxisd flip_z (M_PI, vec);

        if (!tool_path.segments.empty())
        {
          for (std::size_t i = 0; i < tool_path.segments.size(); ++i)
          {
              auto segment = tool_path.segments.at(i);
              for (std::size_t j = 0; j < segment.size(); ++j)
              {
                  Eigen::Isometry3d pt = segment.at(j);
                  std::cout << "Before flip " << get_eigen_str(pt);

                  pt.rotate(flip_z);
                  tool_path.segments.at(i).at(j) = pt;

                  std::cout << ", after " << get_eigen_str(tool_path.segments.at(i).at(j)) << std::endl;
              }
          }
        }
    }

    void WaypointDataFile::getWaypointData2(std::vector<KDL::Frame>& poses)
    {
        if (!tool_path_.segments.empty())
        {
          for (std::size_t i = 0; i < tool_path_.segments.size(); ++i)
          {
              const auto segment = tool_path_.segments.at(i);
              for (std::size_t j = 0; j < segment.size(); ++j)
              {
                  const auto pt = segment.at(j);
                  geometry_msgs::Pose p;
                  p = tf2::toMsg(pt);
                  KDL::Frame pose;
                  tf2::fromMsg(p, pose);
                  poses.push_back(pose);
              }
          }
        }
    }

    std::string WaypointDataFile::get_eigen_str(Eigen::Isometry3d pt)
    {
        geometry_msgs::Pose p;
        p = tf2::toMsg(pt);

        std::stringstream s;
        s << "\t(";
        s << std::setprecision(4) << p.position.x << ", " << p.position.y << ", " << p.position.z;
        s << ") \t(";
        s << std::setprecision(3) << p.orientation.x << ", " << p.orientation.y << ", " << p.orientation.z << ", " << p.orientation.w;
        s << ")\t";
        return s.str();
    }


    void WaypointDataFile::printWaypointDataForMatlab(a5_sanding_process_planning::ToolPath tool_path)
    {
        if (tool_path.segments.empty())
           tool_path = tool_path_;

        if (!tool_path.segments.empty())
        {
          std::cout << "waypoints = [";
          for (std::size_t i = 0; i < tool_path.segments.size(); ++i)
          {
              const auto segment = tool_path.segments.at(i);
              for (std::size_t j = 0; j < segment.size(); ++j)
              {
                  const auto pt = segment.at(j);
                  geometry_msgs::Pose p;
                  p = tf2::toMsg(pt);
                  std::cout << std::setprecision(4) << p.position.x << ", " << p.position.y << ", " << p.position.z << "; ";

                  //          << "\t{" << p.orientation.x << ", " << p.orientation.y << ", " << p.orientation.z << ", " << p.orientation.w << "},"
                  //          << std::endl;
              }
          }
          std::cout << "];" << std::endl;
        }
    }

    void WaypointDataFile::printTRTaskForMatlab(a5_sanding_process_planning::ToolPath tool_path)
    {
        if (tool_path.segments.empty())
           tool_path = tool_path_;

        if (!tool_path.segments.empty())
        {
            for (std::size_t i = 0; i < tool_path.segments.size(); ++i)
            {
                const auto segment = tool_path.segments.at(i);
                for (std::size_t j = 0; j < segment.size(); ++j)
                {
                    const Eigen::Isometry3d  pt = segment.at(j);
                    KDL::Frame pose;
                    tf::transformEigenToKDL(pt, pose);

                    std::cout << "base_quats(:,:," << (j+1) << ") = [";
                    std::cout << std::setprecision(6) << pose(0,0) << " " << pose(0,1) << " " << pose(0,2) << " " << pose(0,3) << "; ";
                    std::cout << std::setprecision(6) << pose(1,0) << " " << pose(1,1) << " " << pose(1,2) << " " << pose(1,3) << "; ";
                    std::cout << std::setprecision(6) << pose(2,0) << " " << pose(2,1) << " " << pose(2,2) << " " << pose(2,3) << "; ";
                    std::cout << std::setprecision(6) << pose(3,0) << " " << pose(3,1) << " " << pose(3,2) << " " << pose(3,3) << "; ";
                    std::cout << "]; " << std::endl;
                }
            }
        }
    }

    void WaypointDataFile::visualizeWaypoints(std::string frame)
    {
        if (!tool_path_.segments.empty())
        {
            geometry_msgs::PoseArray poses;

            for (const auto& segment : tool_path_.segments)
            {
              for (const auto& pose : segment)
              {
                geometry_msgs::Pose pose_msg;
                pose_msg = tf2::toMsg(pose);
                poses.poses.push_back(pose_msg);
              }
            }

            if (!frame.empty())
            {
              poses.header.frame_id = frame;
              poses.header.stamp = ros::Time::now();
            }

            waypoint_pub_.publish(poses);
        }
    }

}
