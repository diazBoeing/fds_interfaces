/*! \class      AircraftDataFile
 *  \brief      Methods to read the aircraft data file
 *  \date       Sep 27, 2021
 *  \author     Michelle Crivella
 *  \copyright  BOEING PROPRIETARY, CONFIDENTAL, AND/OR TRADE SECRET, COPYRIGHT 2021/ THE BOEING COMPANY,  UNPUBLISHED WORK.  ALL RIGHTS RESERVED.
*/

#ifndef AIRCRAFT_DATA_FILE_H
#define AIRCRAFT_DATA_FILE_H

#include <string>
#include <vector>

#include "rapidxml/rapidxml.hpp"
#include <kdl/frames.hpp>
#include <kdl_conversions/kdl_msg.h>        // get poseArrayFromPane
#include <geometry_msgs/Pose.h>             // get poseArrayFromPane
#include <geometry_msgs/PoseArray.h>        // get poseArrayFromPane
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <visualization_msgs/MarkerArray.h>

namespace FDS_Interfaces
{
    class Manufacturing_Data
    {
        public:
            Manufacturing_Data()  {   }

            Manufacturing_Data(std::string f, std::string ma, double mat, bool c)
            {
                fastenerID_ = f;
                materialType_ = ma;
                materialThickness_ = mat;
                countersink_ = c;
            }

            ~Manufacturing_Data() {  }

            std::string fastenerID_;
            std::string materialType_;
            double materialThickness_;
            bool countersink_;
    };

    class Task_Data
    {
        public:
            Task_Data(std::string id, KDL::Frame pose, FDS_Interfaces::Manufacturing_Data md)
            {
                task_id_ = id;
                pose_ = pose;
                manufacturing_data_ = md;
            }
            ~Task_Data()   {  }

            std::string task_id_;
            KDL::Frame pose_;
            FDS_Interfaces::Manufacturing_Data manufacturing_data_;
    };

    class Panel
    {
        public:
            Panel(std::string id, std::vector<FDS_Interfaces::Task_Data> td)
            {
                panel_id_ = id;
                tasks_ = td;
            }
            ~Panel()  {  }

            std::string panel_id_;
            std::vector<FDS_Interfaces::Task_Data> tasks_;
    };

    class AircraftDataFile
    {
        public:
            AircraftDataFile();
            ~AircraftDataFile();

            /*!
             * \brief readFile - reads the given aircraft data file
             * \param filename - used, the filename
             * \return true if successful, false otherwise
             */
            bool readFile_F15(std::string filename);
            bool readFile_testWing(std::string filename);

            /*!
             * \brief readFilePanelIdFromHoleID - reads the aircraft data file but extracts the first segment of the hole identifier
             *  and uses that as the panel id (ie, does not use the panel id in the file). Only used if the first segment of the hole
             *  identifier can be used to group holes for a specific panel.  NOTE: Not Used!
             * \param filename - used, the filename
             * \return true if successful, false otherwise
             */
            bool readFilePanelIdFromHoleID(std::string filename);

            void generateTestData(double min_x, double max_x, double min_y, double max_y, double z, double trans_increment,
                                  double rot_x, double rot_y, double rot_z, std::string reference_frame);

            std::vector<Panel> getAircraftData(std::string panel_id);
            std::vector<KDL::Frame> getKDLFrames();
            std::vector<std::string> getPanelNames();
            std::vector<Task_Data> getTaskDataForPanel(std::string panel_id);
            std::vector<KDL::Frame> getTaskDataKDLForPanel(std::string panel_id);
            void getTaskDataPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr points);
            void getTaskDataPointCloudForPanel(std::string panel_id, pcl::PointCloud<pcl::PointXYZ>::Ptr points);
            void getRVIZDisplay(std::vector<visualization_msgs::MarkerArray>& marker, std::string frame);
            geometry_msgs::PoseArray getPoseArrayForPanel(std::string panel_id, const std::string &frame);
            visualization_msgs::MarkerArray getMarkerArrayByPanelId(std::string panel_id, std::string frame);

            void printAllAircraftPointsForMatlab(std::string label);
            void printPanelDataForMatlab(std::string panel_id);

        private:
            std::string filename_;
            std::vector<Panel> aircraft_data_;
            std::string reference_frame_;

            const char* readAttribute(const rapidxml::xml_node<> * node, const char* attrName);
    };
}

#endif // AIRCRAFT_DATA_FILE_H
