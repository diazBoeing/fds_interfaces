/*! \file       base_position_file.cpp
 *  @copydoc    base_position_file.h
*/

#include <iostream>
#include <fstream>
#include <iomanip>

#include <yaml-cpp/yaml.h>
#include "ik_solution_file.h"
#include "base_position_file.h"

namespace FDS_Interfaces
{
    BasePositionFile::BasePositionFile(std::string folder)
    {
        folder_ = folder;
        filename_ = folder+"base_positions.yaml";
    }

    BasePositionFile::~BasePositionFile()  {   }

    bool BasePositionFile::write_BasePositionFile(std::vector<FDS_Interfaces::PanelBasePositions> all_positions)
    {
        bool success = true;
        all_aircraft_base_positions_ = all_positions;

        //std::cout << "BasePositionFile::write_BasePositionFile: writing to " << filename_ << std::endl;

        try
        {
            YAML::Emitter out;
            out << YAML::BeginSeq;
            for (size_t i = 0; i < all_positions.size(); ++i)
            {
                FDS_Interfaces::PanelBasePositions panel_base_position = all_positions.at(i);

                out << YAML::BeginMap;
                out << YAML::Key << "Panels" << YAML::Value << YAML::Null;
                out << YAML::Key << "panel_id" << YAML::Value << YAML::DoubleQuoted << panel_base_position.panel_id_;

                out << YAML::Key << "base_positions";
                out << YAML::BeginSeq;
                for (unsigned int j=0; j<panel_base_position.base_positions_.size(); j++)
                {
                    FDS_Interfaces::BasePosition bp = panel_base_position.base_positions_.at(j);
                    std::string bp_id = bp.first;
                    KDL::Frame pose = bp.second;

                    std::vector<double> xyz_values = {pose.p(0), pose.p(1), pose.p(2)};
                    double roll, pitch, yaw;
                    pose.M.GetRPY(roll, pitch, yaw);
                    std::vector<double> rpy_values = {roll, pitch, yaw};

                    out << YAML::BeginMap;
                    out << YAML::Key << "base_position_id" << YAML::Value << YAML::DoubleQuoted << bp_id;
                    out << YAML::Key << "position" << YAML::Value << YAML::Flow << xyz_values;
                    out << YAML::Key << "orientation" << YAML::Value << YAML::Flow << rpy_values;
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

    bool BasePositionFile::read_BasePositionFile()
    {
        bool success = true;

        try
        {
            YAML::Node node = YAML::LoadFile(filename_);
            //std::cout << "BasePositionFile::read_BasePositionFile: number of panels: " << node.size() << std::endl;
            for (std::size_t i = 0; i < node.size(); i++)
            {
                std::string panel_id = node[i]["panel_id"].as<std::string>();

                FDS_Interfaces::PanelBasePositions panel_base_positions;
                panel_base_positions.panel_id_ = panel_id;

                YAML::Node bps = node[i]["base_positions"];
                if (bps != NULL)
                {
                    for (std::size_t j = 0; j < bps.size(); j++)
                    {
                        std::string bp_id = bps[j]["base_position_id"].as<std::string>();
                        std::vector<double> position = bps[j]["position"].as<std::vector<double>>();
                        std::vector<double> orientation = bps[j]["orientation"].as<std::vector<double>>();
                        KDL::Frame frame = FDS_Interfaces::IKSolutionFile::getKDLFrameFromVectors(position, orientation);
                        FDS_Interfaces::BasePosition bp(bp_id, frame);
                        panel_base_positions.base_positions_.push_back(bp);
                    }
                }

                all_aircraft_base_positions_.push_back(panel_base_positions);
            }
        }
        catch (std::exception& exc)
        {
            success = false;
            std::string emsg = "IKSolutionFile::read_IKSolutionFile: Unable to read in data from " + filename_ + ":" + exc.what();
            std::cout << emsg << std::endl;

            std::cout << "Defaulting to the origin of the robot coordinate system for base position" << std::endl;
            std::string bp_id = "RobotBase";
            std::vector<double> position{0.0, 0.0, 0.0};
            std::vector<double> orientation{0.0, 0.0, 0.0};
            KDL::Frame frame = FDS_Interfaces::IKSolutionFile::getKDLFrameFromVectors(position, orientation);
            FDS_Interfaces::BasePosition bp(bp_id, frame);
            FDS_Interfaces::PanelBasePositions panel_base_positions;
            panel_base_positions.base_positions_.push_back(bp);
            all_aircraft_base_positions_.push_back(panel_base_positions);
        }

        return success;
    }

    FDS_Interfaces::PanelBasePositions BasePositionFile::getBasePositionsByPanelName(std::string panel_id)
    {
        FDS_Interfaces::PanelBasePositions panel_base_position;

        for (size_t i=0; i < all_aircraft_base_positions_.size(); ++i)
        {
            panel_base_position = all_aircraft_base_positions_.at(i);
            if (panel_id.compare(panel_base_position.panel_id_) == 0)
                break;
        }

        return panel_base_position;
    }

    void BasePositionFile::printBasePositionsForMatlab(std::string panel_id)
    {
        std::cout << "base_positions = [";
        for (size_t i=0; i < all_aircraft_base_positions_.size(); ++i)
        {
            FDS_Interfaces::PanelBasePositions panel_base_position = all_aircraft_base_positions_.at(i);
            if ((panel_id.empty()) || (panel_id.compare(panel_base_position.panel_id_) == 0))
            {
                for (unsigned int j=0; j<panel_base_position.base_positions_.size(); j++)
                {
                    FDS_Interfaces::BasePosition bp = panel_base_position.base_positions_.at(j);
                    KDL::Frame pose = bp.second;
                    std::cout << std::setprecision(6) << pose.p(0) << ", " << pose.p(1) << ", " << pose.p(2) << "; ";
                }
            }
        }
        std::cout << "];" << std::endl;
    }

    void BasePositionFile::printBaseQuaternionsForMatlab(std::string panel_id)
    {
        for (size_t i=0; i < all_aircraft_base_positions_.size(); ++i)
        {
            FDS_Interfaces::PanelBasePositions panel_base_position = all_aircraft_base_positions_.at(i);
            if ((panel_id.empty()) || (panel_id.compare(panel_base_position.panel_id_) == 0))
            {
                for (unsigned int j=0; j<panel_base_position.base_positions_.size(); j++)
                {
                    std::cout << "base_quats(:,:," << (j+1) << ") = [";
                    FDS_Interfaces::BasePosition bp = panel_base_position.base_positions_.at(j);
                    KDL::Frame pose = bp.second;
                    std::cout << std::setprecision(6) << pose(0,0) << " " << pose(0,1) << " " << pose(0,2) << " " << pose(0,3) << "; ";
                    std::cout << std::setprecision(6) << pose(1,0) << " " << pose(1,1) << " " << pose(1,2) << " " << pose(1,3) << "; ";
                    std::cout << std::setprecision(6) << pose(2,0) << " " << pose(2,1) << " " << pose(2,2) << " " << pose(2,3) << "; ";
                    std::cout << std::setprecision(6) << pose(3,0) << " " << pose(3,1) << " " << pose(3,2) << " " << pose(3,3) << "; ";
                    std::cout << "]; " << std::endl;
                }
            }
        }
    }

}
