/*! \file       test_aircraft_data_file.cpp
 *  \brief      test reading and printing results of aircraft data File
 *  \date       Oct 14, 2021
 *  \author     Michelle Crivella
 *  \copyright  BOEING PROPRIETARY, CONFIDENTAL, AND/OR TRADE SECRET, COPYRIGHT 2021/ THE BOEING COMPANY,  UNPUBLISHED WORK.  ALL RIGHTS RESERVED.
*/

#include <iostream>

#include <ros/ros.h>

#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>

#include "PointCloudOperations.h"
#include "target_file.h"
#include "aircraft_data_file.h"

enum Search_Type {BY_TASK_ID, BY_PANEL_ID, BY_DISTANCE};

std::string remove_extension(const std::string& filename)
{
    size_t lastdot = filename.find_last_of(".");
    if (lastdot == std::string::npos) return filename;
    return filename.substr(0, lastdot);
}

void save_data_as_pointcloud(FDS_Interfaces::AircraftDataFile ac_datafile, std::string filename)
{
    std::string fname = remove_extension(filename);

    pcl::PointCloud<pcl::PointXYZ>::Ptr points(new pcl::PointCloud<pcl::PointXYZ>);
    ac_datafile.getTaskDataPointCloud(points);
    std::cout << "Total number of Points =  " << points->size() << std::endl;
    std::string pcd_filename = fname + ".pcd";
    pcl::io::savePCDFileASCII(pcd_filename, *points);
}

void extract_and_transform_data(FDS_Interfaces::AircraftDataFile ac_datafile, Search_Type search_type, std::string search_string, double max_distance)
{
    std::string filename;
    pcl::PointCloud<pcl::PointXYZ>::Ptr points(new pcl::PointCloud<pcl::PointXYZ>);

    if (!search_string.empty())
    {
        if (search_type == BY_PANEL_ID)
        {
            std::cout << "Extracting data by panel id: " << search_string << std::endl;
            ac_datafile.getTaskDataPointCloudForPanel(search_string, points);
        }
        else if (search_type == BY_TASK_ID)
        {
            std::cout << "Extracting data by task id: " << search_string << std::endl;
            for (unsigned int i=0; i<ac_datafile.aircraft_data_.size(); i++)
            {
                FDS_Interfaces::Panel panel = ac_datafile.aircraft_data_.at(i);
                for (unsigned int j=0; j < panel.tasks_.size(); j++)
                {
                    FDS_Interfaces::Task_Data task = panel.tasks_.at(j);
                    if (task.task_id_.find(search_string) != std::string::npos)
                    {
                        pcl::PointXYZ point;
                        point.x = task.pose_.p(0);
                        point.y = task.pose_.p(1);
                        point.z = task.pose_.p(2);
                        points->push_back(point);
                    }
                }
            }
        }
        else  // BY_DISTANCE
        {
            std::cout << "Extracting data by distance to task: " << search_string << std::endl;
            FDS_Interfaces::Task_Data task;
            bool found = ac_datafile.getTaskByID(search_string, task);
            if (found)
            {
                std::cout << "Found task " << task.task_id_ << std::endl;
                pcl::PointXYZ point;
                point.x = task.pose_.p(0);
                point.y = task.pose_.p(1);
                point.z = task.pose_.p(2);

                pcl::PointCloud<pcl::PointXYZ>::Ptr aircraft_points(new pcl::PointCloud<pcl::PointXYZ>);
                ac_datafile.getTaskDataPointCloud(aircraft_points);

                MC_Utils::PointCloudOperations pc_ops;
                pc_ops.getPointsWithinDistance(aircraft_points, point, max_distance, points);
            }
        }
    }

    if (points->size() > 0)
    {
        std::cout << "Number of xtracted points = " << points->size() << std::endl;

                                                                        // Save off extracted points to a cloud
        filename = search_string + ".pcd";
        pcl::io::savePCDFileASCII(filename, *points);

        //MC_Utils::PointCloudOperations pc_ops;
        //MC_Utils::PointCloudOperations::PointCloudStats stats;
        //pc_ops.getPointCloudStats(points, stats);
        //pc_ops.printPointCloudStats("Extracted points stats:", stats);

                                                                        // Transform points
        pcl::PointCloud<pcl::PointXYZ>::Ptr rotated_points(new pcl::PointCloud<pcl::PointXYZ>);
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        //transform.rotate(Eigen::AngleAxisf(M_PI*1.3, Eigen::Vector3f::UnitZ()));
        transform.rotate(Eigen::AngleAxisf(M_PI/2.0, Eigen::Vector3f::UnitZ()));
        float trans_x = 0.4327;
        float trans_y = 0.29811;
        float trans_z = 0.0;
        transform.translation() << trans_x, trans_y, trans_z;
        pcl::transformPointCloud (*points, *rotated_points, transform);

                                                                        // Save off rotated points to a cloud
        filename = search_string + "_rotated.pcd";
        pcl::io::savePCDFileASCII(filename, *rotated_points);
    }
    else
        std::cout << "No points found for search string " << search_string << std::endl;
}

void test_generate_data(FDS_Interfaces::AircraftDataFile  aircraftData_)
{
    double spacing = .5;
    bool flip_normal = false;
    std::string chain_start_ = "fds_base_frame_link";

                                        // below robot
    {
        std::cout << "Points below Robot: " << std::endl;
        double x = 1.0;
        double y = 1.0;
        double z = 0.0127;
        flip_normal = false;
        pcl::PointXYZ top_left(-1.0*x, y, z);
        pcl::PointXYZ top_right(x, y, z);
        pcl::PointXYZ bottom_left(-1.0*x, -1.0*y, z);
        pcl::PointXYZ bottom_right(x, -1.0*y, z);
        aircraftData_.generateTestData(top_left, top_right, bottom_left, bottom_right, flip_normal, spacing, chain_start_);
        aircraftData_.printTRTaskForMatlab("");
    }

                                        // over robot
    {
        std::cout << "Points above Robot: " << std::endl;
        double x = 1.0;
        double y = 1.0;
        double z = 1.5;
        flip_normal = false;
        pcl::PointXYZ top_left(-1.0*x, y, z);
        pcl::PointXYZ top_right(x, y, z);
        pcl::PointXYZ bottom_left(-1.0*x, -1.0*y, z);
        pcl::PointXYZ bottom_right(x, -1.0*y, z);
        aircraftData_.generateTestData(top_left, top_right, bottom_left, bottom_right, flip_normal, spacing, chain_start_);
        aircraftData_.printTRTaskForMatlab("");
    }

                                        // front of robot
    {
        std::cout << "Points in front Robot: " << std::endl;
        double max_reach_y = 0.7112;
        flip_normal = false;
        pcl::PointXYZ top_left(-1.0, max_reach_y, 1.0);
        pcl::PointXYZ top_right(1.0, max_reach_y, 1.0);
        pcl::PointXYZ bottom_left(-1.0, max_reach_y, -1.0);
        pcl::PointXYZ bottom_right(1.0, max_reach_y, -1.0);
        aircraftData_.generateTestData(top_left, top_right, bottom_left, bottom_right, flip_normal, spacing, chain_start_);
        aircraftData_.printTRTaskForMatlab("");
    }

                                    // behind robot
    {
        std::cout << "Points behind Robot: " << std::endl;
        double max_reach_y = -0.7112;
        flip_normal = true;
        pcl::PointXYZ top_left(-1.0, max_reach_y, 1.0);
        pcl::PointXYZ top_right(1.0, max_reach_y, 1.0);
        pcl::PointXYZ bottom_left(-1.0, max_reach_y, -1.0);
        pcl::PointXYZ bottom_right(1.0, max_reach_y, -1.0);
        aircraftData_.generateTestData(top_left, top_right, bottom_left, bottom_right, flip_normal, spacing, chain_start_);
        aircraftData_.printTRTaskForMatlab("");
    }

                                    // below robot at 45 angle
    {
        std::cout << "Points at 45 degrees to Robot: " << std::endl;
        double max_reach_y = 1.0;
        double min_reach_y = 0.3048;
        flip_normal = false;
        pcl::PointXYZ top_left(-1.0, max_reach_y, 1.0);
        pcl::PointXYZ top_right(1.0, max_reach_y, 1.0);
        pcl::PointXYZ bottom_left(-1.0, min_reach_y, 0.0);
        pcl::PointXYZ bottom_right(1.0, min_reach_y, 0.0);
        aircraftData_.generateTestData(top_left, top_right, bottom_left, bottom_right, flip_normal, spacing, chain_start_);
        aircraftData_.printTRTaskForMatlab("");
    }

                                // over robot at 135 angle
    {
        std::cout << "Points at 135 degrees to Robot: " << std::endl;
        double max_reach_y = 0.864;
        double min_reach_y = -0.6096;
        flip_normal = false;
        pcl::PointXYZ top_left(-1.2, min_reach_y, 1.0);
        pcl::PointXYZ top_right(1.0, min_reach_y, 1.0);
        pcl::PointXYZ bottom_left(-1.2, max_reach_y, 0.0);
        pcl::PointXYZ bottom_right(1.0, max_reach_y, 0.0);
        aircraftData_.generateTestData(top_left, top_right, bottom_left, bottom_right, flip_normal, spacing, chain_start_);
        aircraftData_.printTRTaskForMatlab("");
    }

    /*
    double max_reach = 1.3;
    double height_above_aircraft = .0254;   // 1"
    aircraftData_.generateTestData((-1.0*max_reach), max_reach, (-1.0*max_reach), max_reach, height_above_aircraft, spacing, M_PI, 0.0, 0.0, chain_start_);
    */
}

void test_read_file(FDS_Interfaces::AircraftDataFile& ac_datafile, std::string filename)
{
    bool success = ac_datafile.readFile(filename);
    if (success)
    {
        std::vector<FDS_Interfaces::Panel> panels = ac_datafile.getAircraftData("");
        for (unsigned int i=0; i<panels.size(); i++)
        {
            FDS_Interfaces::Panel panel = panels.at(i);
            std::cout << "Panel " << panel.panel_id_ << ", number of tasks = " << panel.tasks_.size() << std::endl;
        }
        std::cout << "Total number of Panels =  " << panels.size() << std::endl;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_aircraft_file");
    ros::NodeHandle nh;
    FDS_Interfaces::AircraftDataFile ac_datafile(false, nh);

    //std::string filename = "/home/ros-industrial/fds_data/aircraft/AircraftData_F15.xml";
    //std::string filename = "/home/ros-industrial/fds_data/aircraft/AircraftData_TestWing.xml";
    std::string filename = "/home/ros-industrial/fds_data/aircraft/FDS_Coupon_Cart_MED_COUPON.csv";

    //std::cout << "argc = " << argc << std::endl;
    std::string search_string = "";
    double max_distance = 0.0;
    if (argc > 1)
        search_string = argv[1];
    if (argc > 2)
        max_distance = atof(argv[2]);

    test_read_file(ac_datafile, filename);
    save_data_as_pointcloud(ac_datafile, filename);
    //extract_and_transform_data(ac_datafile, BY_TASK_ID, search_string, max_distance);
    //extract_and_transform_data(ac_datafile, BY_DISTANCE, search_string, max_distance);

    return 0;
}
