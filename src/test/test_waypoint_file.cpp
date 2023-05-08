/*! \file       test_waypoint_file.cpp
 *  \brief      test reading and printing results of a Waypoint File
 *  \date       Dec 1, 2021
 *  \author     Michelle Crivella
 *  \copyright  BOEING PROPRIETARY, CONFIDENTAL, AND/OR TRADE SECRET, COPYRIGHT 2021/ THE BOEING COMPANY,  UNPUBLISHED WORK.  ALL RIGHTS RESERVED.
*/

#include <iostream>
#include <kdl/frames.hpp>
#include "a5_sanding_process_planning/tool_path_planner.h"
#include <waypoint_data_file.h>
#include "aircraft_data_file.h"

int main(int argc, char** argv)
{
    std::string filename = "/home/ros-industrial/fds_data/scan_0/waypoints.yaml";
    FDS_Interfaces::WaypointDataFile waypoint_file;
    waypoint_file.read_WaypointDataFile(filename);
    //waypoint_file.printWaypointDataForMatlab();


    a5_sanding_process_planning::ToolPath tool_path;
    waypoint_file.printTRTaskForMatlab(tool_path);

    waypoint_file.getWaypointDataFlippedZs(tool_path, "x");
    waypoint_file.printTRTaskForMatlab(tool_path);

    /*
    FDS_Interfaces::AircraftDataFile ac_datafile;
    std::vector<KDL::Frame> poses;
    waypoint_file.getWaypointData2(poses);
    ac_datafile.setAircraftDataFromPoses(poses);
    ac_datafile.printTRTaskForMatlab("");
    */

    return 0;
}
