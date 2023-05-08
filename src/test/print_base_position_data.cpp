#include <ros/ros.h>

#include "aircraft_data_file.h"
#include "base_position_file.h"

int main(int argc, char** argv)
{
    bool success = true;
    ros::init(argc, argv, "print_base_position_file");
    ros::NodeHandle nh;

    std::string data_folder;
    if (const char* home_folder = std::getenv("HOME"))
    {
        std::string tmp(home_folder);
        data_folder = tmp + "/fds_data/aircraft/";
    }
    else
        data_folder = "/home/ros-industrial/fds_data/aircraft/";

    std::string panel_id = "";
    if (argc > 1)
        panel_id = argv[1];

    FDS_Interfaces::AircraftDataFile ac_datafile_(false, nh);
    std::string filename = data_folder + "AircraftData_TestWing.xml";
    success = ac_datafile_.readFile_testWing(filename);

    FDS_Interfaces::BasePositionFile bpf(data_folder);
    bpf.read_BasePositionFile();

    if (panel_id.empty())
    {
        std::vector<std::string> panel_names = ac_datafile_.getPanelNames();
        for (unsigned int i=0; (success) && (i<panel_names.size()); i++)
        {
            std::string panel_id = panel_names.at(i);
            FDS_Interfaces::PanelBasePositions base_positions = bpf.getBasePositionsByPanelName(panel_id);
            std::cout << "Panel " << panel_id << ", base positions = " << base_positions.base_positions_.size() << std::endl;
        }
        std::cout << "Total number of Panels =  " << panel_names.size() << std::endl;
    }
    else
    {
        ac_datafile_.printAllAircraftPointsForMatlab("pts");
        ac_datafile_.printPanelDataForMatlab(panel_id);

        bpf.printBasePositionsForMatlab(panel_id);
        bpf.printBaseQuaternionsForMatlab(panel_id);
    }

    return success;
}
