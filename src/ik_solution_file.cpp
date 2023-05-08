/*! \file       ik_solution_file.cpp
 *  @copydoc    ik_solution_file.h
*/

#include <iostream>
#include <fstream>
#include <iomanip>
#include <set>

#include <yaml-cpp/yaml.h>
#include "ik_solution_file.h"

namespace FDS_Interfaces
{
    IKSolutionFile::IKSolutionFile(std::string folder)
    {
        folder_ = folder;
        filename_ = folder+"ik_solutions.yaml";
    }

    IKSolutionFile::~IKSolutionFile()
    {

    }

    bool IKSolutionFile::write_IKSolutionFile(std::vector<FDS_Interfaces::IK_Solution> ik_solutions, int max_results)
    {
        bool success = true;

        try
        {
            YAML::Emitter out;

            out << YAML::BeginSeq;
            for (unsigned int i=0; i<ik_solutions.size(); i++)
            {
                FDS_Interfaces::IK_Solution solution = ik_solutions.at(i);
                KDL::Frame pose = solution.task_pose;

                //std::cout << "Solutions for " << solution.task_id << std::endl;

                out << YAML::BeginMap;
                out << YAML::Key << "task_id" << YAML::Value << YAML::DoubleQuoted << solution.task_id;
                out << YAML::Key << "panel_id" << YAML::Value << YAML::DoubleQuoted << solution.panel_id;
                out << YAML::Key << "task_position" << YAML::Value << YAML::Flow << FDS_Interfaces::IKSolutionFile::getPositionVectorFromKDLFrame(pose);
                out << YAML::Key << "task_orientation" << YAML::Value << YAML::Flow << FDS_Interfaces::IKSolutionFile::getOrientationVectorFromKDLFrame(pose);
                out << YAML::Key << "reachability_result" << YAML::Value << YAML::DoubleQuoted << solution.message;

                out << YAML::Key << "reachability_results";

                out << YAML::BeginSeq;
                for (unsigned int j=0; j<solution.reachability_results.size(); j++)
                {
                    FDS_Interfaces::Base_Position_Solution bps = solution.reachability_results.at(j);

                    //std::cout << "   Base Position " << bps.base_position_id << std::endl;

                    out << YAML::BeginMap;
                    out << YAML::Key << "base_position_id" << YAML::Value << YAML::DoubleQuoted << bps.base_position_id;
                    out << YAML::Key << "base_position" << YAML::Value << YAML::Flow << FDS_Interfaces::IKSolutionFile::getPositionVectorFromKDLFrame(bps.base_position_frame);
                    out << YAML::Key << "base_orientation" << YAML::Value << YAML::Flow << FDS_Interfaces::IKSolutionFile::getOrientationVectorFromKDLFrame(bps.base_position_frame);
                    out << YAML::Key << "task_new_position" << YAML::Value << YAML::Flow << FDS_Interfaces::IKSolutionFile::getPositionVectorFromKDLFrame(bps.task_new_pose);
                    out << YAML::Key << "task_new_orientation" << YAML::Value << YAML::Flow << FDS_Interfaces::IKSolutionFile::getOrientationVectorFromKDLFrame(bps.task_new_pose);
                    out << YAML::Key << "result_from_this_base_position" << YAML::Value << YAML::DoubleQuoted << bps.result_from_this_bp;

                    FDS_Interfaces::IK_Solution_Set solution_set = bps.ordered_solutions;
                    FDS_Interfaces::IK_Solution_Set::iterator it = solution_set.begin();

                    out << YAML::Key << "ik_solutions";
                    out << YAML::BeginSeq;
                    int loop_cnt = 0;
                    while (it != solution_set.end())
                    {
                        FDS_Interfaces::IK_Solution_And_Value jnts_and_value = *it;
                        KDL::JntArray joint_values = jnts_and_value.solution;
                        std::vector<double> jnt_values = FDS_Interfaces::IKSolutionFile::KDLJntArrayToStdVec(joint_values);
                        double value = jnts_and_value.value;

                        out << YAML::BeginMap;
                        out << YAML::Key << "value" << YAML::Value << value;
                        out << YAML::Key << "joint_values" << YAML::Value << YAML::Flow << jnt_values;
                        out << YAML::EndMap;

                        loop_cnt++;
                        it++;

                        if ((max_results > 0) && (loop_cnt > max_results))
                           break;
                    }

                    out << YAML::EndSeq;
                    out << YAML::EndMap;
                }
                out << YAML::EndSeq;
                out << YAML::EndMap;
            }
            out << YAML::EndSeq;

            std::ofstream fout(filename_.c_str());
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


    bool IKSolutionFile::read_IKSolutionFile()
    {
        bool success = true;

        try
        {
            YAML::Node node = YAML::LoadFile(filename_);
            //std::cout << "RobotPoseManager::load_poses_file: number of poses: " << node.size() << std::endl;
            for (std::size_t i = 0; i < node.size(); i++)
            {

                std::string task_id = node[i]["task_id"].as<std::string>();
                //std::cout << "Solutions for " << task_id << std::endl;
                std::string panel_id = node[i]["panel_id"].as<std::string>();
                std::vector<double> position = node[i]["task_position"].as<std::vector<double>>();
                std::vector<double> orientation = node[i]["task_orientation"].as<std::vector<double>>();
                KDL::Frame frame = getKDLFrameFromVectors(position, orientation);
                std::string msg = node[i]["reachability_result"].as<std::string>();

                IK_Solution ik_solution(task_id, panel_id, frame);
                ik_solution.message = msg;

                YAML::Node base_position_results = node[i]["reachability_results"];
                if (base_position_results != NULL)
                {
                    for (std::size_t j = 0; j < base_position_results.size(); j++)
                    {
                        std::string bps_id = base_position_results[j]["base_position_id"].as<std::string>();
                        //std::cout << "   Base Position " << bps_id << std::endl;
                        std::vector<double> bp_position = base_position_results[j]["base_position"].as<std::vector<double>>();
                        std::vector<double> bp_orientation = base_position_results[j]["base_orientation"].as<std::vector<double>>();
                        KDL::Frame bp_frame = getKDLFrameFromVectors(bp_position, bp_orientation);
                        std::vector<double> new_position = base_position_results[j]["task_new_position"].as<std::vector<double>>();
                        std::vector<double> new_orientation = base_position_results[j]["task_new_orientation"].as<std::vector<double>>();
                        KDL::Frame task_new_frame = getKDLFrameFromVectors(new_position, new_orientation);
                        std::string bp_msg = base_position_results[j]["result_from_this_base_position"].as<std::string>();

                        FDS_Interfaces::IK_Solution_Set solutions_value_set;
                        YAML::Node joint_value_pairs = base_position_results[j]["ik_solutions"];
                        if (joint_value_pairs != NULL)
                        {
                            for (std::size_t k = 0; k < joint_value_pairs.size(); k++)
                            {
                                double value = joint_value_pairs[k]["value"].as<double>();
                                std::vector<double> joint_values = joint_value_pairs[k]["joint_values"].as<std::vector<double>>();
                                KDL::JntArray jnt_array = getKDLJntArrayFromVector(joint_values);
                                solutions_value_set.insert(FDS_Interfaces::IK_Solution_And_Value(value, jnt_array));
                            }
                        }

                        FDS_Interfaces::Base_Position_Solution bps(bps_id, bp_frame, task_new_frame, bp_msg, solutions_value_set);
                        ik_solution.add_base_position_solution(bps);
                    }
                }
                ik_solutions_.push_back(ik_solution);
            }                
        }
        catch (std::exception& exc)
        {
            success = false;
            std::string emsg = "IKSolutionFile::read_IKSolutionFile: Unable to read in data from " + filename_ + ":" + exc.what();
            std::cout << emsg << std::endl;
        }

        return success;

    }

    std::vector<std::string> IKSolutionFile::getPanelNames()
    {
        std::vector<std::string> result;
        std::set<std::string> hashTable;
        for (unsigned int i=0; i<ik_solutions_.size(); i++)
        {
            std::string panel_id = ik_solutions_.at(i).panel_id;

            if (!panel_id.empty())
            {
                std::set<std::string>::iterator it = hashTable.find(panel_id);
                if (it == hashTable.end())
                {
                    hashTable.insert(panel_id);
                    result.push_back(panel_id);
                }
            }
        }
        return result;
    }

    void IKSolutionFile::getTaskSolutionsByPanelId(std::string panel_id, std::vector<FDS_Interfaces::IK_Solution>& panel_points)
    {
        if (panel_id.empty())
            panel_points = ik_solutions_;
        else
            for (unsigned int i=0; i<ik_solutions_.size(); i++)
            {
                FDS_Interfaces::IK_Solution solution = ik_solutions_.at(i);
                if (panel_id.compare(solution.panel_id) == 0)
                    panel_points.push_back(solution);
            }
    }

    std::vector<std::string> IKSolutionFile::getTasksByPanel(std::string panel_id, std::string status)
    {
        std::vector<FDS_Interfaces::IK_Solution> solutions;
        getTaskSolutionsByPanelId(panel_id, solutions);

        std::vector<std::string> task_names;
        for (unsigned int j=0; j<solutions.size(); j++)
        {
            FDS_Interfaces::IK_Solution solution = solutions.at(j);
            if (solution.message.compare(status) == 0)
                task_names.push_back(solution.task_id);
        }
        return task_names;
    }

    std::vector<std::string> IKSolutionFile::getTasksByPanelAndBP(std::string panel_id, std::string bp_id, std::string status)
    {
        std::vector<std::string> task_names;

        std::vector<FDS_Interfaces::IK_Solution> solutions;
        getTaskSolutionsByPanelId(panel_id, solutions);

        for (unsigned int j=0; j<solutions.size(); j++)
        {
            FDS_Interfaces::IK_Solution solution = solutions.at(j);

            for (unsigned int k=0; k<solution.reachability_results.size(); k++)
            {
                FDS_Interfaces::Base_Position_Solution bps = solution.reachability_results.at(k);
                if (bp_id.compare(bps.base_position_id) == 0)
                {
                    if (status.compare(bps.result_from_this_bp) == 0)
                        task_names.push_back(solutions.at(j).task_id);
                }
            }
        }

        return task_names;
    }

    std::vector<std::vector<double>> IKSolutionFile::getBestIKSolutions(std::string panel_id)
    {
        std::vector<std::vector<double>> bestIKSolutions;

        for (unsigned int i=0; i<ik_solutions_.size(); i++)
        {
            FDS_Interfaces::IK_Solution solution = ik_solutions_.at(i);
            if (solution.message.compare(std::string(FDS_Interfaces::IKSolutionFile::REACHABLE)) == 0)
            {
                if ((panel_id.empty()) || (panel_id.compare(solution.panel_id) == 0))
                {
                    KDL::JntArray jnts = solution.reachability_results.at(0).getBestIKSolution();
                    std::vector<double> jntVec = KDLJntArrayToStdVec(jnts);
                    bestIKSolutions.push_back(jntVec);
                }
            }
        }

        return bestIKSolutions;
    }

    std::vector<std::vector<double>> IKSolutionFile::getClosestIKSolutions(std::string bp_id)
    {
        std::vector<std::vector<double>> closestIKSolutions;

        std::string task_id = ik_solutions_.at(0).task_id;
        std::vector<double> current_joint_values = getBestIKSolutionByTask(task_id, bp_id);
        closestIKSolutions.push_back(current_joint_values);

        for (unsigned int i=1; i<ik_solutions_.size(); i++)
        {
            FDS_Interfaces::IK_Solution solution = ik_solutions_.at(i);
            std::vector<double> next_joint_values = getClosestIKSolutionByTask(solution.task_id, bp_id, current_joint_values);
            closestIKSolutions.push_back(next_joint_values);
        }

        return closestIKSolutions;
    }

    std::vector<double> IKSolutionFile::getBestIKSolutionByTask(std::string task_id, std::string bp_id)
    {
        std::vector<double> result;

        for (unsigned int i=0; i<ik_solutions_.size(); i++)
        {
            FDS_Interfaces::IK_Solution solution = ik_solutions_.at(i);
            if (task_id.compare(solution.task_id) == 0)
            {
                if (solution.message.compare(std::string(FDS_Interfaces::IKSolutionFile::REACHABLE)) == 0)
                    for (unsigned int j=0; j<solution.reachability_results.size(); j++)
                    {
                        FDS_Interfaces::Base_Position_Solution bp_solution = solution.reachability_results.at(j);
                        if ((bp_id.empty()) || (bp_id.compare(bp_solution.base_position_id) == 0))
                        {
                            KDL::JntArray jnt_values = bp_solution.getBestIKSolution();
                            result = KDLJntArrayToStdVec(jnt_values);
                            break;
                        }
                    }
            }
        }
        return result;
    }

    std::vector<double> IKSolutionFile::getClosestIKSolutionByTask(std::string task_id, std::string bp_id, std::vector<double> current_joint_positions)
    {
        std::vector<double> closest_solution;
        double best_value = 99999.9;
        for (unsigned int i=0; i<ik_solutions_.size(); i++)
        {
            FDS_Interfaces::IK_Solution solution = ik_solutions_.at(i);
            if (task_id.compare(solution.task_id) == 0)
            {
                if (solution.message.compare(std::string(FDS_Interfaces::IKSolutionFile::REACHABLE)) == 0)
                    for (unsigned int j=0; j<solution.reachability_results.size(); j++)
                    {
                        FDS_Interfaces::Base_Position_Solution bp_solution = solution.reachability_results.at(j);
                        if ((bp_id.empty()) || (bp_id.compare(bp_solution.base_position_id) == 0))
                        {
                            std::set<FDS_Interfaces::IK_Solution_And_Value>::iterator it;
                            for (it = bp_solution.ordered_solutions.begin(); it != bp_solution.ordered_solutions.end(); ++it)
                            {
                                KDL::JntArray joint_values = it->solution;
                                std::vector<double> jv_vector = IKSolutionFile::KDLJntArrayToStdVec(joint_values);
                                double distance = calculateDistanceBetweenJointPositions(current_joint_positions, jv_vector);
                                if (distance < best_value)
                                {
                                    //std::cout << "\tUpdating closest pose for task " << task_id << std::endl;
                                    best_value = distance;
                                    closest_solution = jv_vector;
                                }
                            }
                            break;
                        }
                    }
            }
        }

        return closest_solution;
    }

    double IKSolutionFile::calculateDistanceBetweenJointPositions(std::vector<double> jv1, std::vector<double> jv2)
    {
        double distance = 0.0;
        if (jv1.size() == jv2.size())
        {
            for (unsigned int i=0; i<jv1.size(); i++)
                distance = distance + abs(jv1.at(i) - jv2.at(i));
        }
            // TODO, handle error if vectors are not the same size

        return distance;
    }

    visualization_msgs::MarkerArray IKSolutionFile::getMarkerArrayByPanelId(std::string panel_id, std::string frame)
    {
        visualization_msgs::MarkerArray point_marker_array;

        for (unsigned int i=0; i<ik_solutions_.size(); i++)
        {
            FDS_Interfaces::IK_Solution solution = ik_solutions_.at(i);
            if (panel_id.compare(solution.panel_id) == 0)
            {
                visualization_msgs::Marker marker;
                marker.header.frame_id = frame;
                marker.header.stamp = ros::Time();
                marker.ns = solution.task_id;
                marker.id = 0;
                marker.type = visualization_msgs::Marker::SPHERE;
                marker.action = visualization_msgs::Marker::ADD;
                marker.pose.position.x = solution.task_pose.p(0);
                marker.pose.position.y = solution.task_pose.p(1);
                marker.pose.position.z = solution.task_pose.p(2);
                double x, y, z, w;
                solution.task_pose.M.GetQuaternion(x, y, z, w);
                marker.pose.orientation.x = x;
                marker.pose.orientation.y = y;
                marker.pose.orientation.z = z;
                marker.pose.orientation.w = w;
                marker.scale.x = 0.05;
                marker.scale.y = 0.05;
                marker.scale.z = 0.05;
                marker.color.a = 1.0;
                if (solution.message.compare(std::string(FDS_Interfaces::IKSolutionFile::REACHABLE)) == 0)
                    marker.color.g = 1.0f;
                else if (solution.message.compare(std::string(FDS_Interfaces::IKSolutionFile::NO_COLLISION_FREE_SOLUTIONS)) == 0)
                    marker.color.b = 1.0f;
                else if (solution.message.compare(std::string(FDS_Interfaces::IKSolutionFile::NOT_REACHABLE)) == 0)
                    marker.color.r = 1.0;
                point_marker_array.markers.push_back(marker);
            }
        }

        return point_marker_array;
    }

    geometry_msgs::PoseArray IKSolutionFile::getPoseArrayByPanelId(std::string panel_id, const std::string &frame)
    {
      geometry_msgs::PoseArray poses;

      for (unsigned int i=0; i<ik_solutions_.size(); i++)
      {
          FDS_Interfaces::IK_Solution solution = ik_solutions_.at(i);
          if (panel_id.compare(solution.panel_id) == 0)
          {
              FDS_Interfaces::IK_Solution solution = ik_solutions_.at(i);

              KDL::Frame pose = solution.task_pose;

              geometry_msgs::Pose pose_msg;
              tf::poseKDLToMsg(pose, pose_msg);
              poses.poses.push_back(pose_msg);
          }
      }

      if (!frame.empty())
      {
        poses.header.frame_id = frame;
        poses.header.stamp = ros::Time::now();
      }

      return poses;
    }


    visualization_msgs::MarkerArray IKSolutionFile::getMarkerArrayByBasePositionId(std::string bp_id, std::string frame)
    {
        visualization_msgs::MarkerArray point_marker_array;

        for (unsigned int i=0; i<ik_solutions_.size(); i++)
        {
            FDS_Interfaces::IK_Solution solution = ik_solutions_.at(i);

            for (unsigned int j=0; j<solution.reachability_results.size(); j++)
            {
                FDS_Interfaces::Base_Position_Solution bps = solution.reachability_results.at(j);
                if (bp_id.compare(bps.base_position_id) == 0)
                {
                    KDL::Frame pose = bps.task_new_pose;

                    visualization_msgs::Marker marker;
                    marker.header.frame_id = frame;
                    marker.header.stamp = ros::Time();
                    marker.ns = solution.task_id;
                    marker.id = 0;
                    marker.type = visualization_msgs::Marker::SPHERE;
                    marker.action = visualization_msgs::Marker::ADD;
                    marker.pose.position.x = pose.p(0);
                    marker.pose.position.y = pose.p(1);
                    marker.pose.position.z = pose.p(2);
                    double x, y, z, w;
                    pose.M.GetQuaternion(x, y, z, w);
                    marker.pose.orientation.x = x;
                    marker.pose.orientation.y = y;
                    marker.pose.orientation.z = z;
                    marker.pose.orientation.w = w;
                    marker.scale.x = 0.01;
                    marker.scale.y = 0.01;
                    marker.scale.z = 0.01;
                    marker.color.a = 1.0;
                    if (bps.result_from_this_bp.compare(std::string(FDS_Interfaces::IKSolutionFile::REACHABLE)) == 0)
                        marker.color.g = 1.0f;
                    else if (bps.result_from_this_bp.compare(std::string(FDS_Interfaces::IKSolutionFile::NO_COLLISION_FREE_SOLUTIONS)) == 0)
                        marker.color.b = 1.0f;
                    else if (bps.result_from_this_bp.compare(std::string(FDS_Interfaces::IKSolutionFile::NOT_REACHABLE)) == 0)
                        marker.color.r = 1.0;
                    point_marker_array.markers.push_back(marker);
                }
            }
        }

        return point_marker_array;
    }

    geometry_msgs::PoseArray IKSolutionFile::getPoseArrayByBasePosition(std::string bp_id, const std::string &frame)
    {
      geometry_msgs::PoseArray poses;

      for (unsigned int i=0; i<ik_solutions_.size(); i++)
      {
          FDS_Interfaces::IK_Solution solution = ik_solutions_.at(i);

          for (unsigned int j=0; j<solution.reachability_results.size(); j++)
          {
              FDS_Interfaces::Base_Position_Solution bps = solution.reachability_results.at(j);
              if (bp_id.compare(bps.base_position_id) == 0)
              {
                  KDL::Frame pose = bps.task_new_pose;

                  geometry_msgs::Pose pose_msg;
                  tf::poseKDLToMsg(pose, pose_msg);
                  poses.poses.push_back(pose_msg);
              }
          }
      }

      if (!frame.empty())
      {
        poses.header.frame_id = frame;
        poses.header.stamp = ros::Time::now();
      }

      return poses;
    }

    void IKSolutionFile::printIKSolutionResultsForMatlab(std::vector<FDS_Interfaces::IK_Solution>& ik_solutions)
    {
        std::vector<KDL::Frame> reachable;
        std::vector<KDL::Frame> no_collision_free;
        std::vector<KDL::Frame> unreachable;
        std::vector<KDL::Frame> unknown;

        for (unsigned int i=0; i<ik_solutions.size(); i++)
        {
            FDS_Interfaces::IK_Solution solution = ik_solutions.at(i);
            if (solution.message.compare(std::string(FDS_Interfaces::IKSolutionFile::REACHABLE)) == 0)
                reachable.push_back(solution.task_pose);
            else if (solution.message.compare(std::string(FDS_Interfaces::IKSolutionFile::NO_COLLISION_FREE_SOLUTIONS)) == 0)
                no_collision_free.push_back(solution.task_pose);
            else if (solution.message.compare(std::string(FDS_Interfaces::IKSolutionFile::NOT_REACHABLE)) == 0)
                unreachable.push_back(solution.task_pose);
            else
                unknown.push_back(solution.task_pose);
        }

        FDS_Interfaces::IKSolutionFile::printPosesForMatlab("reachable", reachable);
        FDS_Interfaces::IKSolutionFile::printPosesForMatlab("unreachable", unreachable);
        FDS_Interfaces::IKSolutionFile::printPosesForMatlab("nocollisionfree", no_collision_free);
        FDS_Interfaces::IKSolutionFile::printPosesForMatlab("unknown", unknown);
    }

    KDL::Frame IKSolutionFile::getKDLFrameFromVectors(std::vector<double> position, std::vector<double> rpy)
    {
        KDL::Vector p(position.at(0), position.at(1), position.at(2));
        KDL::Rotation r;
        r = r.RPY(rpy.at(0), rpy.at(1), rpy.at(2));

        KDL::Frame pose;
        pose.p = p;
        pose.M = r;
        return pose;
    }

    KDL::JntArray IKSolutionFile::getKDLJntArrayFromVector(std::vector<double> jnt_values)
    {
        KDL::JntArray j;

        j.resize(jnt_values.size());
        for (unsigned int i=0; i<jnt_values.size(); i++)
            j(i) = jnt_values.at(i);

        return j;
    }

    std::vector<double> IKSolutionFile::getPositionVectorFromKDLFrame(KDL::Frame pose)
    {
        std::vector<double> xyz_values = {pose.p(0), pose.p(1), pose.p(2)};
        return xyz_values;
    }

    std::vector<double> IKSolutionFile::getOrientationVectorFromKDLFrame(KDL::Frame pose)
    {
        double roll, pitch, yaw;
        pose.M.GetRPY(roll, pitch, yaw);
        std::vector<double> rpy_values = {roll, pitch, yaw};
        return rpy_values;
    }

    std::vector<double> IKSolutionFile::KDLJntArrayToStdVec(KDL::JntArray& jnts)
    {
        std::vector<double> vec;
        for (size_t i=0; i < jnts.data.size(); i++)
            vec.push_back(jnts.data[i]);
        return vec;
    }

    std::string IKSolutionFile::get_pose_str(KDL::Frame pose)
    {
        double roll, pitch, yaw;
        pose.M.GetRPY(roll, pitch, yaw);

        std::stringstream s;
        s << "\t(";
        s << std::setprecision(4) << pose.p(0) << ", " << pose.p(1) << ", " << pose.p(2);
        s << ") \t(";
        s << std::setprecision(3) << roll << ", " << pitch << ", " << yaw;
        s << ")\t";
        return s.str();
    }

    std::string IKSolutionFile::get_joint_str(KDL::JntArray jnts)
    {
        std::stringstream s;
        s << " [";
        for (std::size_t i = 0; i < jnts.data.size()-1; i++)
            s << std::setprecision(2) << jnts.data[i] << ", ";
        s << jnts.data[jnts.data.size()-1] << "] ";
        return s.str();
    }

    std::string IKSolutionFile::get_joint_str_from_vector(std::vector<double> jnts)
    {
        std::stringstream s;
        s << " [";
        for (std::size_t i = 0; i < jnts.size()-1; i++)
            s << std::setprecision(2) << jnts.at(i) << ", ";
        s << jnts.at(jnts.size()-1) << "] ";
        return s.str();
    }

    void IKSolutionFile::printPosesForMatlab(std::string varName, std::vector<KDL::Frame> poses)
    {
        KDL::Frame pose;
        std::cout << varName << " = [";
        for (unsigned int i = 0; i < poses.size(); i++)
        {
            pose = poses.at(i);
            std::cout << std::setprecision(6) << pose.p(0) << ", " << pose.p(1) << ", " << pose.p(2) << "; ";
        }
        std::cout << "];" << std::endl;
    }

}
