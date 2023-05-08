/*! \file       test_waypoint_file.cpp
 *  \brief      test reading and printing results of a Waypoint File
 *  \date       Dec 1, 2021
 *  \author     Michelle Crivella
 *  \copyright  BOEING PROPRIETARY, CONFIDENTAL, AND/OR TRADE SECRET, COPYRIGHT 2021/ THE BOEING COMPANY,  UNPUBLISHED WORK.  ALL RIGHTS RESERVED.
*/

#include <iostream>

#include "orientation_file.h"

int main(int argc, char** argv)
{
    std::string data_folder;
    if (const char* home_folder = std::getenv("HOME"))
    {
        std::string tmp(home_folder);
        data_folder = tmp + "/fds_data/";
    }
    else
    {
        std::cout << "Defaulting to hardcoded data folder" << std::endl;
        data_folder = "/home/ros-industrial/fds_data/";
    }

    FDS_Interfaces::OrientationFile of(data_folder);

    std::string scan_plan_filename = data_folder + FDS_Interfaces::ORIENTATION_FOLDER + "scan_0_down.yaml";
    bool success = of.read_scan_plan_file(scan_plan_filename);

    if (success)
    {
        std::cout << "orientation = " << of.orientation_data_.orientation << std::endl;
        std::cout << "motion plan file = " << of.orientation_data_.motion_plan_filename << std::endl;
    }
    else
        std::cout << "Error reading scan plan file " << scan_plan_filename << std::endl;

    FDS_Interfaces::Current_Orientation co = of.get_current_orientation();
    if (co.orientation > -1)
    {
        std::cout << "orientation = " << co.orientation << std::endl;
        std::cout << "mesh file = " << co.mesh_filename << std::endl;
    }
    else
        std::cout << "Error reading current orientation file " << std::endl;

    return 0;
}
