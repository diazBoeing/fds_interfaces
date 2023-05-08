/*! \file       aircraft_data_file.cpp
 *  @copydoc    aircraft_data_file.h
*/

#include <iostream>
#include <fstream>
#include <iomanip>
#include <stdexcept>
#include <cstdlib>

#include <boost/algorithm/string.hpp>
#include <tf_conversions/tf_eigen.h>
#include <tf2_eigen/tf2_eigen.h>

#include "PointCloudOperations.h"
#include "ik_solution_file.h"           // get_pose_str
#include "aircraft_data_file.h"

namespace FDS_Interfaces
{
    AircraftDataFile::AircraftDataFile(bool realMode, ros::NodeHandle& pnh)
        : realMode_(realMode)
        , pnh_(pnh)
        , points_pub_(pnh_.advertise<visualization_msgs::MarkerArray>("aircraft_points", 1, true))
    {

    }

    AircraftDataFile::AircraftDataFile()
        : realMode_(false)
    {

    }

    AircraftDataFile::~AircraftDataFile()  {  }

    bool AircraftDataFile::readFile(std::string filename)
    {
         bool success = true;
         aircraft_data_.clear();
         //std::cout << "AircraftDataFile::readFile: " << filename << std::endl;

         std::size_t found = filename.find(".csv");
         if (found != std::string::npos)
         {
             success = readFile_csv(filename);
         }
         else
         {
             found = filename.find(".xml");
             if (found != std::string::npos)
             {
                 rapidxml::xml_document<> doc;
                 rapidxml::xml_node<> * root_node;

                                                 // Read the xml file into a vector
                 std::ifstream theFile(filename.c_str(), std::ifstream::in);
                 std::vector<char> buffer((std::istreambuf_iterator<char>(theFile)), std::istreambuf_iterator<char>());
                 buffer.push_back('\0');
                                                                                 // Parse the buffer using the xml file parsing library into doc
                 doc.parse<0>(&buffer[0]);
                 root_node = doc.first_node("Aircraft_Data");
                 int version = std::atoi(readAttribute(root_node, "version"));
                 if (version == 1)
                     success = readFile_F15(filename);
                 else
                     success = readFile_testWing(filename);
             }
             else
                 std::cout << "AircraftDataFile::readFile unknown format (not csv or xml): " << filename << std::endl;
         }

         return success;
    }

    bool AircraftDataFile::readFile_csv(std::string filename)
    {
        bool success = true;
        std::cout << "AircraftDataFile::readFile_csv: " << filename << std::endl;

        try
        {
            CsvParser *csvparser = CsvParser_new(filename.c_str(), ",", 0);
            CsvRow *row;

            row = CsvParser_getRow(csvparser);				// skip the header row

            std::vector<FDS_Interfaces::Task_Data> tasks;
            while ((row = CsvParser_getRow(csvparser)))
            {
                const char **rowFields = CsvParser_getFields(row);
                if (CsvParser_getNumFields(row) >= 7)
                {
                    std::string task_id = rowFields[1];
                    double x = atof(rowFields[2]);
                    double y = atof(rowFields[3]);
                    double z = atof(rowFields[4]);
                    double i = atof(rowFields[5]);
                    double j = atof(rowFields[6]);
                    double k = atof(rowFields[7]);

                    KDL::Vector end_location(x, y, z);
                    KDL::Rotation end_rotation;
                    end_rotation = end_rotation.RPY(i, j, k);

                    KDL::Frame pose;
                    pose.p = end_location;
                    pose.M = end_rotation;

                    FDS_Interfaces::Manufacturing_Data md("fastenerid", "aluminum", .1, false);

                    FDS_Interfaces::Task_Data td(task_id, pose, md);
                    tasks.push_back(td);
                }
            }
            FDS_Interfaces::Panel panel("Coupon", tasks);
            aircraft_data_.push_back(panel);
        }
        catch (std::exception& exc)
        {
            success = false;
            std::string emsg = "AircraftDataFile::readFile_csv: Unable to read in data from " + filename_ + ":" + exc.what();
            std::cout << emsg << std::endl;
        }


        return success;
    }

    bool AircraftDataFile::readFile_F15(std::string filename)
    {
        bool success = true;

        std::cout << "AircraftDataFile::readFile_F15: " << filename << std::endl;

        std::string panel_id;
        std::string task_id;
        try
        {
            rapidxml::xml_document<> doc;
            rapidxml::xml_node<> * root_node;
            rapidxml::xml_node<> * panel_node;
            rapidxml::xml_node<> * hole_node;

                                            // Read the xml file into a vector
            std::ifstream theFile(filename.c_str(), std::ifstream::in);
            std::vector<char> buffer((std::istreambuf_iterator<char>(theFile)), std::istreambuf_iterator<char>());
            buffer.push_back('\0');
                                                                            // Parse the buffer using the xml file parsing library into doc
            doc.parse<0>(&buffer[0]);
                                                                            // Find our root node

            root_node = doc.first_node("Aircraft_Data");
                                                                            //check for expected file format
            if (root_node == NULL) throw std::runtime_error("Expected node is NULL, check Aircraft Data file format");

            panel_node = root_node->first_node("panel");
                                                                                // Iterate over panels
            while (panel_node != NULL)
            {               
                panel_id = readAttribute(panel_node, "ID");
                boost::replace_all(panel_id, " ", "_");       // replace all spaces with underlines
                //std::cout << "AircraftDataFile::readFile: panel = " << panel_id << std::endl;

                hole_node = panel_node->first_node("hole");

                std::vector<FDS_Interfaces::Task_Data> tasks;
                while (hole_node != NULL)
                {

                    task_id = readAttribute(hole_node, "ID");

                    const rapidxml::xml_node<> * cad_info = hole_node->first_node("cadData");
                    const rapidxml::xml_node<> * coordinates = cad_info->first_node("coordinates");
                    double x = std::atof(readAttribute(coordinates, "x"));
                    double y = std::atof(readAttribute(coordinates, "y"));
                    double z = std::atof(readAttribute(coordinates, "z"));

                    const rapidxml::xml_node<> * oriention = cad_info->first_node("vector");
                    double i = std::atof(readAttribute(oriention, "i"));
                    double j = std::atof(readAttribute(oriention, "j"));
                    double k = std::atof(readAttribute(oriention, "k"));

                    KDL::Vector end_location(x, y, z);
                    KDL::Rotation end_rotation;
                    end_rotation = end_rotation.RPY(i, j, k);

                    KDL::Frame pose;
                    pose.p = end_location;
                    pose.M = end_rotation;

                    //std::cout << "AircraftDataFile::readFile: added " << panel_id << ", task " << task_id << " " << ReachTaskList::get_pose_str(pose) << std::endl;


                    // TODO, finish this
                    FDS_Interfaces::Manufacturing_Data md("fastenerid", "aluminum", .1, false);

                    FDS_Interfaces::Task_Data td(task_id, pose, md);
                    tasks.push_back(td);

                    hole_node = hole_node->next_sibling();
                }
                FDS_Interfaces::Panel panel(panel_id, tasks);
                aircraft_data_.push_back(panel);

                panel_node = panel_node->next_sibling();
            }

        }
        catch (std::exception& exc)
        {
            success = false;
            std::string emsg = "AircraftDataFile::readFile_F15: Unable to read in data from " + filename_ + ":" + exc.what();
            std::cout << emsg << std::endl;
        }

        return success;
    }

    bool AircraftDataFile::readFile_testWing(std::string filename)
    {
        bool success = true;
        double z_threshold = 0.7;

        std::cout << "AircraftDataFile::readFile_testWing = " << filename << std::endl;

        std::string panel_id;
        std::string task_id;
        try
        {
            rapidxml::xml_document<> doc;
            rapidxml::xml_node<> * root_node;
            rapidxml::xml_node<> * panel_node;
            rapidxml::xml_node<> * panel_zone_node;
            rapidxml::xml_node<> * hole_node;

                                            // Read the xml file into a vector
            std::ifstream theFile(filename.c_str(), std::ifstream::in);
            std::vector<char> buffer((std::istreambuf_iterator<char>(theFile)), std::istreambuf_iterator<char>());
            buffer.push_back('\0');
                                                                            // Parse the buffer using the xml file parsing library into doc
            doc.parse<0>(&buffer[0]);
                                                                            // Find our root node

            root_node = doc.first_node("Aircraft_Data");
                                                                            //check for expected file format
            if (root_node == NULL) throw std::runtime_error("Expected node is NULL, check Aircraft Data file format");

            panel_node = root_node->first_node("panel");
                                                                                // Iterate over panels
            while (panel_node != NULL)
            {
                std::string panel_name = readAttribute(panel_node, "name");
                //std::cout << "AircraftDataFile::readFile: panel = " << panel_id << std::endl;

                panel_zone_node = panel_node->first_node("panel_zone");
                while (panel_zone_node != NULL)
                {
                    std::string panel_zone_name = readAttribute(panel_zone_node, "panel_zone");
                    panel_id = panel_name+"_"+panel_zone_name;
                    boost::replace_all(panel_id, " ", "_");       // replace all spaces with underlines

                    hole_node = panel_zone_node->first_node("hole");

                    std::vector<FDS_Interfaces::Task_Data> tasks;
                    while (hole_node != NULL)
                    {

                        task_id = readAttribute(hole_node, "ID");

                        //const rapidxml::xml_node<> * cad_info = hole_node->first_node("cadData");
                        const rapidxml::xml_node<> * coordinates = hole_node->first_node("coordinates");
                        double x = std::atof(readAttribute(coordinates, "x"));
                        double y = std::atof(readAttribute(coordinates, "y"));
                        double z = std::atof(readAttribute(coordinates, "z"));

                        const rapidxml::xml_node<> * oriention = hole_node->first_node("vector");
                        double i = std::atof(readAttribute(oriention, "i"));
                        double j = std::atof(readAttribute(oriention, "j"));
                        double k = std::atof(readAttribute(oriention, "k"));

                        KDL::Vector end_location(x, y, z);
                        KDL::Rotation end_rotation;
                        end_rotation = end_rotation.RPY(i, j, k);

                        KDL::Frame pose;
                        pose.p = end_location;
                        pose.M = end_rotation;

                        //std::cout << "AircraftDataFile::readFile: added " << panel_id << ", task " << task_id << " " << ReachTaskList::get_pose_str(pose) << std::endl;


                        // TODO, finish this
                        FDS_Interfaces::Manufacturing_Data md("fastenerid", "aluminum", .1, false);

                        FDS_Interfaces::Task_Data td(task_id, pose, md);
                        if (z > z_threshold)
                            std::cout << "readFile_testWing: discarding task " << panel_id << "/" << task_id <<", z is greater than " <<
                                         z_threshold << FDS_Interfaces::IKSolutionFile::get_pose_str(pose) << std::endl;
                        else
                            tasks.push_back(td);

                        hole_node = hole_node->next_sibling();
                    }
                    FDS_Interfaces::Panel panel(panel_id, tasks);
                    aircraft_data_.push_back(panel);

                    panel_zone_node = panel_zone_node->next_sibling();
                }
                panel_node = panel_node->next_sibling();
            }

        }
        catch (std::exception& exc)
        {
            success = false;
            std::string emsg = "AircraftDataFile::readFile_Testwing: Unable to read in data from " + filename_ + ":" + exc.what();
            std::cout << emsg << std::endl;
        }

        return success;
    }

    void AircraftDataFile::setAircraftDataFromPoses(std::vector<KDL::Frame> poses)
    {
        aircraft_data_.clear();
        FDS_Interfaces::Manufacturing_Data md("fastenerid", "aluminum", .1, false);

        std::vector<FDS_Interfaces::Task_Data> tasks;
        for (unsigned int i=0; i<poses.size(); i++)
        {
            std::stringstream name;
            name << "task_" << std::setw(3) << std::setfill('0') << (i+1);

            KDL::Frame pose = poses.at(i);
            FDS_Interfaces::Task_Data td(name.str(), pose, md);
            tasks.push_back(td);
        }

        std::string panel_id = "Plane";
        FDS_Interfaces::Panel panel(panel_id, tasks);
        aircraft_data_.push_back(panel);
    }

    bool AircraftDataFile::readFilePanelIdFromHoleID(std::string filename)
    {
        bool success = true;

        std::cout << "AircraftDataFile::readFile: filename = " << filename << std::endl;

        std::string panel_id;
        std::string task_id;
        try
        {
            rapidxml::xml_document<> doc;
            rapidxml::xml_node<> * root_node;
            rapidxml::xml_node<> * panel_node;
            rapidxml::xml_node<> * hole_node;

                                            // Read the xml file into a vector
            std::ifstream theFile(filename.c_str(), std::ifstream::in);
            std::vector<char> buffer((std::istreambuf_iterator<char>(theFile)), std::istreambuf_iterator<char>());
            buffer.push_back('\0');
                                                                            // Parse the buffer using the xml file parsing library into doc
            doc.parse<0>(&buffer[0]);
                                                                            // Find our root node

            root_node = doc.first_node("Aircraft_Data");
                                                                            //check for expected file format
            if (root_node == NULL) throw std::runtime_error("Expected node is NULL, check Aircraft Data file format");

            panel_node = root_node->first_node("panel");
                                                                                // Iterate over panels
            while (panel_node != NULL)
            {
                //panel_id = readAttribute(panel_node, "ID");
                //std::cout << "AircraftDataFile::readFile: panel = " << panel_id << std::endl;

                hole_node = panel_node->first_node("hole");
                if (hole_node != NULL)
                {
                    task_id = readAttribute(hole_node, "ID");
                    std::size_t pos = task_id.find("-");
                    panel_id = task_id.substr(0, pos);
                    std::string save_panel_id = panel_id;

                    std::vector<FDS_Interfaces::Task_Data> tasks;
                    while (hole_node != NULL)
                    {
                        const rapidxml::xml_node<> * cad_info = hole_node->first_node("cadData");
                        const rapidxml::xml_node<> * coordinates = cad_info->first_node("coordinates");
                        double x = std::atof(readAttribute(coordinates, "x"));
                        double y = std::atof(readAttribute(coordinates, "y"));
                        double z = std::atof(readAttribute(coordinates, "z"));

                        const rapidxml::xml_node<> * oriention = cad_info->first_node("vector");
                        double i = std::atof(readAttribute(oriention, "i"));
                        double j = std::atof(readAttribute(oriention, "j"));
                        double k = std::atof(readAttribute(oriention, "k"));

                        KDL::Vector end_location(x, y, z);
                        KDL::Rotation end_rotation;
                        end_rotation = end_rotation.RPY(i, j, k);

                        KDL::Frame pose;
                        pose.p = end_location;
                        pose.M = end_rotation;

                        //std::cout << "AircraftDataFile::readFile: added " << panel_id << ", task " << task_id << " " << ReachTaskList::get_pose_str(pose) << std::endl;


                        // TODO, finish this
                        FDS_Interfaces::Manufacturing_Data md("fastenerid", "aluminum", .1, false);

                        FDS_Interfaces::Task_Data td(task_id, pose, md);
                        tasks.push_back(td);

                        hole_node = hole_node->next_sibling();

                        if (hole_node != NULL)
                        {
                            task_id = readAttribute(hole_node, "ID");
                            std::size_t pos = task_id.find("-");
                            panel_id = task_id.substr(0, pos);
                            if (panel_id.compare(save_panel_id) !=0)
                            {
                                FDS_Interfaces::Panel panel(save_panel_id, tasks);
                                std::cout << "AircraftDataFile::readFile: added panel = " << save_panel_id << ", number of holes = " << tasks.size() << std::endl;
                                aircraft_data_.push_back(panel);
                                save_panel_id = panel_id;
                            }
                        }
                        else
                        {
                            FDS_Interfaces::Panel panel(save_panel_id, tasks);
                            aircraft_data_.push_back(panel);
                            std::cout << "AircraftDataFile::readFile: added panel = " << save_panel_id << ", number of holes = " << tasks.size() << std::endl;
                        }
                    }

                    panel_node = panel_node->next_sibling();
                }
            }

        }
        catch (std::exception& exc)
        {
            success = false;
            std::string emsg = "AircraftDataFile::readFile: Unable to read in data from " + filename_ + ":" + exc.what();
            std::cout << emsg << std::endl;
        }

        return success;
    }

    void AircraftDataFile::generateTestData(double min_x, double max_x, double min_y, double max_y, double z, double trans_increment,
                          double rot_x, double rot_y, double rot_z, std::string reference_frame)
    {
        reference_frame_ = reference_frame;

        FDS_Interfaces::Manufacturing_Data md("fastenerid", "aluminum", .1, false);

        std::vector<FDS_Interfaces::Task_Data> tasks_quad1;
        std::vector<FDS_Interfaces::Task_Data> tasks_quad2;
        std::vector<FDS_Interfaces::Task_Data> tasks_quad3;
        std::vector<FDS_Interfaces::Task_Data> tasks_quad4;

        double x_val = min_x;
        double y_val = min_y;

        int index = 1;
        while (x_val < max_x)
        {
            while (y_val < max_y)
            {
                std::stringstream name;
                name << "task_" << std::setw(3) << std::setfill('0') << index;

                KDL::Vector end_location(x_val, y_val, z);
                KDL::Rotation end_rotation;
                end_rotation = end_rotation.RPY(rot_x, rot_y, rot_z);

                KDL::Frame pose;
                pose.p = end_location;
                pose.M = end_rotation;

                FDS_Interfaces::Task_Data td(name.str(), pose, md);
                if ((x_val < 0) && (y_val < 0))
                    tasks_quad1.push_back(td);
                else if ((x_val < 0) && (y_val > 0))
                    tasks_quad2.push_back(td);
                else if ((x_val > 0) && (y_val > 0))
                    tasks_quad3.push_back(td);
                else
                    tasks_quad4.push_back(td);

                y_val = y_val + trans_increment;
                index++;
            }
            x_val = x_val + trans_increment;
            y_val = min_y;
        }

        FDS_Interfaces::Panel panel1("Quad1", tasks_quad1);
        FDS_Interfaces::Panel panel2("Quad2", tasks_quad2);
        FDS_Interfaces::Panel panel3("Quad3", tasks_quad3);
        FDS_Interfaces::Panel panel4("Quad4", tasks_quad4);
        aircraft_data_.push_back(panel1);
        aircraft_data_.push_back(panel2);
        aircraft_data_.push_back(panel3);
        aircraft_data_.push_back(panel4);
    }

    void AircraftDataFile::generateTestData(pcl::PointXYZ top_left, pcl::PointXYZ top_right, pcl::PointXYZ bottom_left, pcl::PointXYZ bottom_right, bool flip_normal, double spacing, std::string reference_frame)
    {
        aircraft_data_.clear();
        MC_Utils::PointCloudOperations pc_ops;
        reference_frame_ = reference_frame;

        std::vector<float> robot_center = {0.0, 0.0, 0.5};
        Eigen::Vector3f view_pt = Eigen::Map<Eigen::Vector3f, Eigen::Unaligned>(robot_center.data(), robot_center.size());
        Eigen::Vector3f rpy;
        pc_ops.calculateOrientationFromViewPt(top_left, top_right, bottom_left, bottom_right, view_pt, rpy);
        if (flip_normal)
            rpy[0] = -1.0 * rpy[0];

        KDL::Rotation end_rotation;
        end_rotation = end_rotation.RPY(rpy[0], rpy[1], rpy[2]);

        FDS_Interfaces::Manufacturing_Data md("fastenerid", "aluminum", .1, false);
        std::vector<FDS_Interfaces::Task_Data> tasks;

        float length, width;
        pc_ops.getDistancePointToPoint(top_left, top_right, length);
        pc_ops.getDistancePointToPoint(top_left, bottom_left, width);

        double span_length;
        double span_width;

        pcl::PointXYZ start_pt = top_left;
        pcl::PointXYZ end_pt = top_right;

        span_width = 0.0;
        int index = 1;
        while (span_width < width+spacing)
        {
            span_length = 0.0;
            while (span_length < length+spacing)
            {
                pcl::PointXYZ new_pt;
                pc_ops.getPointOnLine(start_pt, end_pt, span_length, new_pt);

                std::stringstream name;
                name << "task_" << std::setw(3) << std::setfill('0') << index;
                KDL::Vector end_location(new_pt.x, new_pt.y, new_pt.z);
                KDL::Frame pose;
                pose.p = end_location;
                pose.M = end_rotation;

                FDS_Interfaces::Task_Data td(name.str(), pose, md);
                tasks.push_back(td);
                std::cout << "AircraftDataFile::generateTestData: added task " << name.str() << " " << FDS_Interfaces::IKSolutionFile::get_pose_str(pose) << std::endl;

                index++;

                span_length = span_length + spacing;
            }
            span_width = span_width + spacing;
            pc_ops.getPointOnLine(top_left, bottom_left, span_width, start_pt);
            pc_ops.getPointOnLine(top_right, bottom_right, span_width, end_pt);
        }
        FDS_Interfaces::Panel plane("Plane", tasks);
        aircraft_data_.push_back(plane);
    }


    const char* AircraftDataFile::readAttribute(const rapidxml::xml_node<> * node, const char* attrName)
    {
        if (node==NULL)
            throw std::runtime_error("Expected node is NULL, check Aircraft Data file format");

        rapidxml::xml_attribute<> * attr = node->first_attribute(attrName);
        if (attr == NULL)
        {
            const char* nodeName = node->name();
            std::string msg = "Attribute \"" + std::string(attrName) + "\" not found in " + std::string(nodeName) + " node";
            throw std::runtime_error(msg);
        }

        return attr->value();
    }

    std::vector<Panel> AircraftDataFile::getAircraftData(std::string panel_id)
    {
        std::vector<Panel> returnVector;

        if (panel_id.empty())
            returnVector = aircraft_data_;
        else
        {

            for (unsigned int i=0; i<aircraft_data_.size(); i++)
            {
                FDS_Interfaces::Panel panel = aircraft_data_.at(i);
                if (panel_id.compare(panel.panel_id_) == 0)
                {
                    returnVector.push_back(panel);
                    break;
                }
            }
        }
        return returnVector;
    }

    std::vector<KDL::Frame> AircraftDataFile::getKDLFrames()
    {
        std::vector<KDL::Frame> poses;

        for (unsigned int i=0; i<aircraft_data_.size(); i++)
        {
            FDS_Interfaces::Panel panel = aircraft_data_.at(i);
            for (unsigned int j=0; j < panel.tasks_.size(); j++)
            {
                FDS_Interfaces::Task_Data task = panel.tasks_.at(j);
                poses.push_back(task.pose_);
            }
        }

        return poses;
    }

    std::vector<std::string> AircraftDataFile::getPanelNames()
    {
        std::vector<std::string> panel_names;
        for (unsigned int i=0; i<aircraft_data_.size(); i++)
            panel_names.push_back(aircraft_data_.at(i).panel_id_);

        return panel_names;
    }

    std::vector<Task_Data> AircraftDataFile::getTaskDataForPanel(std::string panel_id)
    {
        std::vector<Task_Data> tasks;
        for (unsigned int i=0; i<aircraft_data_.size(); i++)
        {
            FDS_Interfaces::Panel panel = aircraft_data_.at(i);
            if (panel_id.compare(panel.panel_id_) == 0)
            {
                tasks = panel.tasks_;
                //std::cout << "AircraftDataFile::getTaskDataForPanel: found tasks for panel " << panel_id << std::endl;
                break;
            }
        }
        return tasks;
    }

    bool AircraftDataFile::getTaskByID(std::string task_id, Task_Data& task)
    {
        //std::cout << "AircraftDataFile::getTaskByID: looking for: " << task_id << std::endl;
        bool found = false;
        for (unsigned int i=0; ((i<aircraft_data_.size()) && (!found)); i++)
        {
            FDS_Interfaces::Panel panel = aircraft_data_.at(i);
            for (unsigned int j=0; ((j < panel.tasks_.size()) && (!found)); j++)
            {
                task = panel.tasks_.at(j);
                //std::cout << "AircraftDataFile::getTaskByID: task_id = " << task.task_id_ << std::endl;
                if (task_id.compare(task.task_id_) == 0)
                    found = true;
            }
        }

        //std::cout << "AircraftDataFile::getTaskByID: exiting, found = " << found << std::endl;
        return found;
    }

    std::vector<KDL::Frame> AircraftDataFile::getTaskDataKDLForPanel(std::string panel_id)
    {
        std::vector<KDL::Frame> poses;
        std::vector<Task_Data> tasks = getTaskDataForPanel(panel_id);
        for (unsigned int i=0; i<tasks.size(); i++)
            poses.push_back(tasks.at(i).pose_);

        return poses;
    }

    void AircraftDataFile::getTaskDataPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr& points)
    {
        for (unsigned int i=0; i<aircraft_data_.size(); i++)
        {
            FDS_Interfaces::Panel panel = aircraft_data_.at(i);
            for (unsigned int j=0; j < panel.tasks_.size(); j++)
            {
                FDS_Interfaces::Task_Data task = panel.tasks_.at(j);
                pcl::PointXYZ point;
                point.x = task.pose_.p(0);
                point.y = task.pose_.p(1);
                point.z = task.pose_.p(2);
                points->push_back(point);
            }
        }
    }

    void AircraftDataFile::getTaskDataPointCloudForPanel(std::string panel_id, pcl::PointCloud<pcl::PointXYZ>::Ptr& points)
    {
        std::vector<Task_Data> tasks = getTaskDataForPanel(panel_id);
        for (unsigned int i=0; i<tasks.size(); i++)
        {
            Task_Data task = tasks.at(i);
            pcl::PointXYZ point;
            point.x = task.pose_.p(0);
            point.y = task.pose_.p(1);
            point.z = task.pose_.p(2);
            points->push_back(point);
        }
    }

    void AircraftDataFile::transform_aircraft_data_and_publish(geometry_msgs::TransformStamped transform, std::string frame)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cad_points(new pcl::PointCloud<pcl::PointXYZ>);
        getTaskDataPointCloud(cad_points);
        //std::cout << "AircraftDataFile::transform_aircraft_data_and_publish: number of cad points " << cad_points->size() << std::endl;

        Eigen::Affine3d affine_transform =  tf2::transformToEigen(transform);

        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_points(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud (*cad_points, *transformed_points, affine_transform.inverse());

        removeAllMarkers(frame);

        visualization_msgs::MarkerArray aircraft_data;
        int pt_index  = 0;
        for (unsigned int i=0; i < aircraft_data_.size(); i++)
        {
            FDS_Interfaces::Panel panel = aircraft_data_.at(i);
            for (unsigned int j=0; j<panel.tasks_.size(); j++)
            {

                Task_Data task = panel.tasks_.at(j);
                visualization_msgs::Marker drillPoint;

                pcl::PointXYZ pt = transformed_points->at(pt_index);

                drillPoint.header.frame_id = frame;
                drillPoint.header.stamp = ros::Time();
                drillPoint.ns = task.task_id_;
                drillPoint.type = visualization_msgs::Marker::SPHERE;
                drillPoint.action = visualization_msgs::Marker::ADD;

                drillPoint.pose.position.x = pt.x;
                drillPoint.pose.position.y = pt.y;
                drillPoint.pose.position.z = pt.z;

                drillPoint.pose.orientation.x = 0.0;
                drillPoint.pose.orientation.y = 0.0;
                drillPoint.pose.orientation.z = 0.0;
                drillPoint.pose.orientation.w = 1.0;

                drillPoint.scale.x = .02;
                drillPoint.scale.y = .02;
                drillPoint.scale.z = .02;
                drillPoint.color.a = 0.5;
                drillPoint.color.r = POINT_COLOR_RGB[0];
                drillPoint.color.g = POINT_COLOR_RGB[1];
                drillPoint.color.b = POINT_COLOR_RGB[2];

                aircraft_data.markers.push_back(drillPoint);
                pt_index++;
            }
        }
        points_pub_.publish(aircraft_data);

    }

    void AircraftDataFile::printAllAircraftPointsForMatlab(std::string label)
    {
        std::cout << label << " = [";
        for (unsigned int i = 0; i < aircraft_data_.size(); i++)
        {
            FDS_Interfaces::Panel panel = aircraft_data_.at(i);
            for (unsigned int j=0; j < panel.tasks_.size(); j++)
            {
                FDS_Interfaces::Task_Data task = panel.tasks_.at(j);
                std::cout << std::setprecision(6) << task.pose_.p(0) << ", " << task.pose_.p(1) << ", " << task.pose_.p(2) << "; ";
            }
        }
        std::cout << "];" << std::endl;
    }

    void AircraftDataFile::printTRTaskForMatlab(std::string panel_id)
    {
        for (unsigned int i = 0; i < aircraft_data_.size(); i++)
        {
            FDS_Interfaces::Panel panel = aircraft_data_.at(i);
            if ((panel_id.empty()) || (panel_id.compare(panel.panel_id_) == 0))
            {
                for (unsigned int j=0; j < panel.tasks_.size(); j++)
                {
                    FDS_Interfaces::Task_Data task = panel.tasks_.at(j);
                    KDL::Frame pose = task.pose_;
                    std::cout << "base_quats(:,:," << (j+1) << ") = [";
                    std::cout << std::setprecision(6) << pose(0,0) << " " << pose(0,1) << " " << pose(0,2) << " " << pose(0,3) << "; ";
                    std::cout << std::setprecision(6) << pose(1,0) << " " << pose(1,1) << " " << pose(1,2) << " " << pose(1,3) << "; ";
                    std::cout << std::setprecision(6) << pose(2,0) << " " << pose(2,1) << " " << pose(2,2) << " " << pose(2,3) << "; ";
                    std::cout << std::setprecision(6) << pose(3,0) << " " << pose(3,1) << " " << pose(3,2) << " " << pose(3,3) << "; ";
                    std::cout << "]; " << std::endl;
                }
            }
        }
    }

    void AircraftDataFile::printPanelDataForMatlab(std::string panel_id)
    {
        std::cout << "t = \'" << panel_id << "\';" << std::endl;
        std::cout << "panel_pts = [";
        std::vector<Task_Data> tasks = getTaskDataForPanel(panel_id);
        for (unsigned int i=0; i<tasks.size(); i++)
        {
            Task_Data task = tasks.at(i);
            std::cout << std::setprecision(6) << task.pose_.p(0) << ", " << task.pose_.p(1) << ", " << task.pose_.p(2) << "; ";
        }
        std::cout << "];" << std::endl;
    }

    /*
     * drillPoints are of type Marker (red - not reachable, green - reachable, black - completed)
     *      Green points also have the axis (a geometry_msgs::PoseArray)
     *
     * drillPoints are pushed into MarkerArray Panel
     *
     * Panel is pushed to MarkerArray AircraftData
     *
     */

    void AircraftDataFile::visualizeAircraftPoints(std::string frame)
    {
        removeAllMarkers(frame);

        visualization_msgs::MarkerArray aircraft_data;
        getRVIZDisplay(aircraft_data, frame);

        points_pub_.publish(aircraft_data);
    }

    void AircraftDataFile::removeAllMarkers(std::string frame)
    {
        visualization_msgs::MarkerArray ma;
        visualization_msgs::Marker markerD;
        markerD.header.frame_id = frame;
        markerD.action = visualization_msgs::Marker::DELETEALL;
        ma.markers.push_back(markerD);

        points_pub_.publish(ma);
    }

    void AircraftDataFile::getRVIZDisplay(visualization_msgs::MarkerArray& aircraft_data, std::string frame)
    {

        for (unsigned int i = 0; i < aircraft_data_.size(); i++)
        {
            FDS_Interfaces::Panel panel = aircraft_data_.at(i);
            for (unsigned int j=0; j<panel.tasks_.size(); j++)
            {
                Task_Data task = panel.tasks_.at(j);
                visualization_msgs::Marker drillPoint;

                drillPoint.header.frame_id = frame;
                drillPoint.header.stamp = ros::Time();
                drillPoint.ns = task.task_id_;
                drillPoint.type = visualization_msgs::Marker::SPHERE;
                drillPoint.action = visualization_msgs::Marker::ADD;

                drillPoint.pose.position.x = task.pose_.p(0);
                drillPoint.pose.position.y = task.pose_.p(1);
                drillPoint.pose.position.z = task.pose_.p(2) + .0127;
                double x, y, z, w;
                task.pose_.M.GetQuaternion(x, y, z, w);
                drillPoint.pose.orientation.x = x;
                drillPoint.pose.orientation.y = y;
                drillPoint.pose.orientation.z = z;
                drillPoint.pose.orientation.w = w;

                drillPoint.scale.x = .02;
                drillPoint.scale.y = .02;
                drillPoint.scale.z = .02;
                drillPoint.color.a = 1.0;
                drillPoint.color.r = POINT_COLOR_RGB[0];
                drillPoint.color.g = POINT_COLOR_RGB[1];
                drillPoint.color.b = POINT_COLOR_RGB[2];

                aircraft_data.markers.push_back(drillPoint);
            }
        }
    }

    visualization_msgs::MarkerArray AircraftDataFile::getMarkerArrayByPanelId(std::string panel_id, std::string frame)
    {
        visualization_msgs::MarkerArray point_marker_array;

        for (unsigned int i = 0; i < aircraft_data_.size(); i++)
        {
            FDS_Interfaces::Panel panel = aircraft_data_.at(i);
            if (panel_id.compare(panel.panel_id_) == 0)
            {
                for (unsigned int j=0; j<panel.tasks_.size(); j++)
                {
                    Task_Data task = panel.tasks_.at(j);

                    visualization_msgs::Marker marker;
                    marker.header.frame_id = frame;
                    marker.header.stamp = ros::Time();
                    marker.ns = task.task_id_;
                    marker.id = 0;
                    marker.type = visualization_msgs::Marker::SPHERE;
                    marker.action = visualization_msgs::Marker::ADD;
                    marker.pose.position.x = task.pose_.p(0);
                    marker.pose.position.y = task.pose_.p(1);
                    marker.pose.position.z = task.pose_.p(2);
                    double x, y, z, w;
                    task.pose_.M.GetQuaternion(x, y, z, w);
                    marker.pose.orientation.x = x;
                    marker.pose.orientation.y = y;
                    marker.pose.orientation.z = z;
                    marker.pose.orientation.w = w;
                    marker.scale.x = 0.01;
                    marker.scale.y = 0.01;
                    marker.scale.z = 0.01;
                    marker.color.a = 1.0;
                    marker.color.g = 1.0f;
                    point_marker_array.markers.push_back(marker);
                }
            }
        }

        return point_marker_array;
    }

    geometry_msgs::PoseArray AircraftDataFile::getPoseArrayForPanel(std::string panel_id, const std::string &frame)
    {
      geometry_msgs::PoseArray poses;

      for (unsigned int i = 0; i < aircraft_data_.size(); i++)
      {
          FDS_Interfaces::Panel panel = aircraft_data_.at(i);
          if (panel_id.compare(panel.panel_id_) == 0)
          {
              for (unsigned int j=0; j<panel.tasks_.size(); j++)
              {
                  Task_Data task = panel.tasks_.at(j);
                  geometry_msgs::Pose pose_msg;
                  tf::poseKDLToMsg(task.pose_, pose_msg);
                  poses.poses.push_back(pose_msg);
              }
          }
      }

      if (!frame.empty())
      {
        poses.header.frame_id = frame;
        poses.header.stamp = ros::Time::now();
      }

      return poses;
    }


}
