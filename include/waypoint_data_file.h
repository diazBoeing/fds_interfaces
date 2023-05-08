/*! \class      WaypointDataFile
 *  \brief      Methods to read/write a waypoint data file
 *  \date       Nov 30, 2021
 *  \author     Michelle Crivella
 *  \copyright  BOEING PROPRIETARY, CONFIDENTAL, AND/OR TRADE SECRET, COPYRIGHT 2021/ THE BOEING COMPANY,  UNPUBLISHED WORK.  ALL RIGHTS RESERVED.
*/

#ifndef WAYPOINT_DATA_FILE_H
#define WAYPOINT_DATA_FILE_H

#include <string>
#include <vector>

#include <kdl/frames.hpp>

#include "a5_sanding_process_planning/serialization.h"
#include "a5_sanding_process_planning/tool_path_planner.h"
#include "a5_sanding_process_planning/utils.h"

#include <ros/ros.h>

namespace FDS_Interfaces
{
    class WaypointDataFile
    {
        public:
            WaypointDataFile();
            WaypointDataFile(ros::NodeHandle& pn);
            ~WaypointDataFile();

            bool write_WaypointDataFile(std::string filename, a5_sanding_process_planning::ToolPath tool_path);
            bool read_WaypointDataFile(std::string filename);

            void getWaypointData(a5_sanding_process_planning::ToolPath& tool_path);
            void getWaypointDataFlippedZs(a5_sanding_process_planning::ToolPath& tool_path, std::string axis);
            void getWaypointData2(std::vector<KDL::Frame>& poses);
            void printWaypointDataForMatlab(a5_sanding_process_planning::ToolPath tool_path);
            void printTRTaskForMatlab(a5_sanding_process_planning::ToolPath tool_path);

            void visualizeWaypoints(std::string frame);

            static std::string get_eigen_str(Eigen::Isometry3d pt);
	    
       private:
            std::string filename_;
            a5_sanding_process_planning::ToolPath tool_path_;

            ros::NodeHandle pnh_;
            ros::ServiceServer server_;
            ros::Publisher waypoint_pub_;
    };
}

#endif // WAYPOINT_DATA_FILE_H
