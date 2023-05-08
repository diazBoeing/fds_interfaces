/*! \file       OrientationFile.cpp
 *  @copydoc    OrientationFile.h
*/

#include <iostream>
#include <fstream>
#include <iomanip>

#include <yaml-cpp/yaml.h>
#include "orientation_file.h"


namespace FDS_Interfaces
{
    OrientationFile::OrientationFile(std::string data_directory)
        : data_directory_(data_directory)
    {
        current_orientation_file_name_ = data_directory_ + CURRENT_ORIENTATION_FILE_NAME;
    }

    OrientationFile::~OrientationFile() { }

    bool OrientationFile::read_scan_plan_file(std::string filename)
    {
        bool success = true;
        try
        {
            YAML::Node node = YAML::LoadFile(filename);
            orientation_data_.orientation = node["orientation"].as<int>();
            orientation_data_.first_joint_positions = node["first_joint_positions"].as<std::vector<double>>();
            orientation_data_.motion_plan_filename = node["motion_plan_filename"].as<std::string>();
        }
        catch (std::exception& exc)
        {
            success = false;
            std::cout  << "Unable to parse scan plan file: " + filename << ": " << exc.what() << std::endl;
        }

        return success;
    }

    bool OrientationFile::save_current_orientation(int orientation, std::string mesh_filename)
    {
        bool success = true;
        try
        {
            YAML::Node root;
            root["orientation"] = std::to_string(orientation);
            root["mesh_filename"] = mesh_filename;

            std::ofstream fout(current_orientation_file_name_.c_str());
            fout << root;
            fout.close();
        }
        catch (std::exception& exc)
        {
            success = false;
            std::cout  << "Unable to write orientation file: " + current_orientation_file_name_ << ": " << exc.what();
        }
        return success;
    }

    Current_Orientation OrientationFile::get_current_orientation()
    {
        Current_Orientation co;
        co.orientation = -1;

        try
        {
            YAML::Node node = YAML::LoadFile(current_orientation_file_name_);
            co.orientation = node["orientation"].as<int>();
            co.mesh_filename = node["mesh_filename"].as<std::string>();
        }
        catch (std::exception& exc)
        {
            std::cout  << "Unable to parse orientation file: " + current_orientation_file_name_ << ": " << exc.what();
        }

        return co;
    }
}
