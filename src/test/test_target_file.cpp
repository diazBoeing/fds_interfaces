
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <target_file.h>
#include <waypoint_data_file.h>
#include <ik_solution_file.h>

void test1()
{
    ros::NodeHandle nh;
    FDS_Interfaces::TargetData targetData(false, nh);

    std::string task_id = "test_id";
    std::vector<double> position = {10.5, -5.0, 0.5};
    std::vector<double> orientation = {10.58, -21.0, 23.3, 0.89};
    int type = 0;
    KDL::Frame target_frame = FDS_Interfaces::IKSolutionFile::getKDLFrameFromVectors(position, orientation);
    std::string target_image = "/home/ros-industrial/fds_data/D12-09_T09_59_35/D12-09_T10_00_00.png";
    std::pair<int, int> pix_coords = std::make_pair(10, 30);
    double diameter = 0.75;
    std::vector<double> robot_joint_states = {0.30306, -0.065036, 0.237616, -3.87894, -1.87348, -0.3030};

    geometry_msgs::Pose tcp_frame;
    tcp_frame.position.x = 0.383;
    tcp_frame.position.y = 0.383;
    tcp_frame.position.z = 1.017;
    tcp_frame.orientation.x = 0.609;
    tcp_frame.orientation.y = -0.383;
    tcp_frame.orientation.z = -0.536;
    tcp_frame.orientation.w = -0.441;
    targetData.addToTargetList(task_id, 1, FDS_Interfaces::TargetStatus::NOT_COMPLETED, false, target_frame, FDS_Interfaces::TargetType(type), target_image, pix_coords, diameter, robot_joint_states, tcp_frame);

    std::string output_path = "/home/ros-industrial/fds_data/test_target_output.yaml";
    targetData.writeTargetDataToFile(output_path);
    std::cout << "Wrote targets file to: " << output_path << std::endl;

    targetData.readTargetDataFromFile(output_path);

    std::vector<std::string> task_names = targetData.getTargetNames();
    std::cout << "Reading targets from: " << output_path  << std::endl;
    std::cout << task_names.size() << " target(s) found:" << std::endl;

    for (int i = 0; i < task_names.size(); i++)
        std::cout << " - " << task_names[i]<< std::endl;

}

void write_data_from_pointcloud(std::string fname)
{
    std::cout << "Reading in pcd file: " << fname << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>(fname, *cloud);

    ros::NodeHandle nh;
    FDS_Interfaces::TargetData td(false, nh);
    td.pointcloud_to_targets_file(cloud);
}

std::string remove_extension(const std::string& filename)
{
    size_t lastdot = filename.find_last_of(".");
    if (lastdot == std::string::npos) return filename;
    return filename.substr(0, lastdot);
}

void write_data_to_pointcloud(std::string filename)
{
    ros::NodeHandle nh;
    FDS_Interfaces::TargetData td(false, nh);

    td.readTargetDataFromFile(filename);

    pcl::PointCloud<pcl::PointXYZ>::Ptr points(new pcl::PointCloud<pcl::PointXYZ>);;
    td.get_targets_as_pointcloud(points);

    std::string pcd_filename = remove_extension(filename) + ".pcd";
    std::cout << "Total number of Points =  " << points->size() << ".  Saving to " << pcd_filename << std::endl;
    pcl::io::savePCDFileASCII(pcd_filename, *points);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_target_file");

    //test1();
    if (argc > 0)
    {
        std::string filename = argv[1];
        //write_data_from_pointcloud(filename);
        write_data_to_pointcloud(filename);
    }

    return 0;
}
