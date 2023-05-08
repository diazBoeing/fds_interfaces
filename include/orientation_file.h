/*! \class      OrientationFile
 *  \brief      The Orientation File (or the scan plan) contains all the information necessary to scan a workpiece in order to create a surface mesh.
 *  \date       January 20, 2023
 *  \author     Michelle Crivella
 *  \copyright  BOEING PROPRIETARY, CONFIDENTAL, AND/OR TRADE SECRET, COPYRIGHT 2021/ THE BOEING COMPANY,  UNPUBLISHED WORK.  ALL RIGHTS RESERVED.
*/

#ifndef ORIENTATION_FILE_H
#define ORIENTATION_FILE_H

#include <string>
#include <set>
#include <vector>

namespace FDS_Interfaces
{
    static const std::string CURRENT_ORIENTATION_FILE_NAME = "current_orientation.yaml";
    static const std::string ORIENTATION_FOLDER = "scan_plans/";
    static const std::string MESH_FOLDER = "mesh_files/";
    static const std::string AIRCRAFT_DATA_FOLDER = "aircraft/";

    class An_Orientation
    {
        public:
            An_Orientation()  {  }
            ~An_Orientation()   {  }

            int orientation;
            std::vector<double> first_joint_positions;
            std::string motion_plan_filename;
    };

    class Current_Orientation
    {
        public:
            Current_Orientation()  {  }
            ~Current_Orientation()   {  }

            int orientation;
            std::string mesh_filename;
    };

    class OrientationFile
    {
        public:
            OrientationFile(std::string data_directory);
            ~OrientationFile();

            An_Orientation orientation_data_;

            bool read_scan_plan_file(std::string filename);
            Current_Orientation get_current_orientation();
            bool save_current_orientation(int orientation, std::string mesh_filename);

        private:
            std::string data_directory_;
            std::string current_orientation_file_name_;
    };
}

#endif // ORIENTATION_FILE_H
