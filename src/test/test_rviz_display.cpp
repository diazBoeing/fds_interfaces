/*! \file       test_rviz_display.cpp
 *  \brief      publish drillPoints (MarkerArrays) for RVIZ display
 *  \date       Oct 6, 2021
 *  \author     Michelle Crivella
 *  \copyright  BOEING PROPRIETARY, CONFIDENTAL, AND/OR TRADE SECRET, COPYRIGHT 2021/ THE BOEING COMPANY,  UNPUBLISHED WORK.  ALL RIGHTS RESERVED.
*/

/*
 * Never create NodeHandles, ros::Publisher or ros::Subscribers in the body of callbacks or in short-lived functions.
 * It will not work.
 * Also never call publish(..) immediately after constructing a ros::Publisher.
 * There is no time for Subscribers to actually subscribe, so no one receives your message.
 */

#include <iostream>

#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include "aircraft_data_file.h"
#include "ik_solution_file.h"

class AircraftDataServer
{
    public:

      AircraftDataServer(ros::NodeHandle& nh, std::string fr, FDS_Interfaces::IKSolutionFile ik_solution_file,
                         std::vector<ros::Publisher> publishers, std::vector<ros::Publisher> point_publishers)
        : nh_(nh)
        , frame_(fr)
        , ik_solution_file_(ik_solution_file)
        , publishers_(publishers)
        , point_publishers_(point_publishers)
      {
        server_ = nh_.advertiseService("read_aircraft_data", &AircraftDataServer::readAircraftData_Callback, this);
        ROS_INFO_STREAM("Started Aircaft Data server. Call the " << server_.getService() << "' service to publish drill point data");
      }

    private:

      bool readAircraftData_Callback(std_srvs::TriggerRequest&, std_srvs::TriggerResponse& res)
      {
          res.success = true;
          res.message = "Success!";

          generatePoseArrays();
          generateMarkerArrays();

          return true;
      }

      void generateMarkerArrays()
      {

          std::vector<std::string> panel_names = ik_solution_file_.getPanelNames();
          for (unsigned int i=0; i<panel_names.size(); i++)
          {
              std::string topic_name = "PanelPoints_" + panel_names.at(i);
              std::cout << "test_rviz_display:: advertising and publising to topic " << topic_name << std::endl;

              visualization_msgs::MarkerArray point_marker_array = ik_solution_file_.getMarkerArrayByPanelId(panel_names.at(i), frame_);
              //visualization_msgs::MarkerArray point_marker_array = ik_solution_file_.getMarkerArrayByBasePositionId("bp_0082", frame_);
              ros::Publisher panelpoints_data_pub = point_publishers_.at(i);
              panelpoints_data_pub.publish(point_marker_array);
          }
      }

      void generatePoseArrays()
      {

          std::vector<std::string> panel_names = ik_solution_file_.getPanelNames();
          for (unsigned int i=0; i<panel_names.size(); i++)
          {
              std::string topic_name = "Panel_" + panel_names.at(i);
              std::cout << "test_rviz_display:: advertising and publising to topic " << topic_name << std::endl;

              geometry_msgs::PoseArray panel_data = ik_solution_file_.getPoseArrayByPanelId(panel_names.at(i), frame_);
              //geometry_msgs::PoseArray panel_data = ik_solution_file_.getPoseArrayByBasePosition("bp_0082", frame_);
              ros::Publisher panel_data_pub = publishers_.at(i);
              panel_data_pub.publish(panel_data);
          }
      }

      ros::NodeHandle nh_;
      std::string filename_;
      std::string frame_;
      ros::ServiceServer server_;
      std::vector<ros::Publisher> publishers_;
      std::vector<ros::Publisher> point_publishers_;
      FDS_Interfaces::IKSolutionFile ik_solution_file_;
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "aircraft_data_test_node");

    bool success = true;
    std::string ik_folder;
    std::string frame;
    ros::NodeHandle pnh{ "~" }, nh;

    try
    {
                                                                    // Get the parameters
        pnh.param("ik_solutions_folder", ik_folder, std::string("/home/ros-industrial/fds_data/reachTest/"));
        pnh.param("frame", frame, std::string("aircraft"));

                                                                    // Advertise publishers
        FDS_Interfaces::IKSolutionFile ik_solution_file(ik_folder);

        success = ik_solution_file.read_IKSolutionFile();

        if (success)
        {
            std::vector<std::string> panel_names = ik_solution_file.getPanelNames();
            std::vector<ros::Publisher> publishers;
            std::vector<ros::Publisher> point_publishers;
            for (unsigned int i=0; i<panel_names.size(); i++)
            {
                std::string topic_name = "Panel_" + panel_names.at(i);
                ros::Publisher panel_data_pub = nh.advertise<geometry_msgs::PoseArray>(topic_name, 1, true);
                publishers.push_back((panel_data_pub));

                std::string topic_name2 = "PanelPoints_" + panel_names.at(i);
                ros::Publisher panelpoints_data_pub = nh.advertise<visualization_msgs::MarkerArray>(topic_name2, 1, true);
                point_publishers.push_back((panelpoints_data_pub));
            }

            AircraftDataServer aircraftDataServer(nh, frame, ik_solution_file, publishers, point_publishers);
            ros::spin();
        }
        else
            std::cout << "test_rviz_display:: could not ik solutions file from folder " << ik_folder << std::endl;
    }
    catch (const std::exception& ex)
    {
        ROS_ERROR_STREAM(ex.what());
        return -1;
    }

    return 0;
}
