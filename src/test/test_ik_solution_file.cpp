/*! \file       test_ik_solution_file.cpp
 *  \brief      test reading and printing results of IK Solution File
 *  \date       Sep 30, 2021
 *  \author     Michelle Crivella
 *  \copyright  BOEING PROPRIETARY, CONFIDENTAL, AND/OR TRADE SECRET, COPYRIGHT 2021/ THE BOEING COMPANY,  UNPUBLISHED WORK.  ALL RIGHTS RESERVED.
*/

#include <iostream>
#include <ik_solution_file.h>

void test_writeIKSolutions(FDS_Interfaces::IKSolutionFile iks)
{
    std::string panel_id = "PANEL1";

    std::string task1_id = "TASK1";
    std::vector<double> task1_position{15.0, -3.0, .05};
    std::vector<double> task1_orientation{1.0, -2.0, 0.0};
    KDL::Frame task1_frame = FDS_Interfaces::IKSolutionFile::getKDLFrameFromVectors(task1_position, task1_orientation);

    std::string task2_id = "TASK2";
    std::vector<double> task2_position{15.0, -3.0, .05};
    std::vector<double> task2_orientation{1.0, -2.0, 0.0};
    KDL::Frame task2_frame = FDS_Interfaces::IKSolutionFile::getKDLFrameFromVectors(task2_position, task2_orientation);

    std::vector<FDS_Interfaces::IK_Solution> ik_solutions;
    std::vector<double> joint_values{1.0, 2.0, 1.5, 2.0, 0.5, 0.6};
    KDL::JntArray jnt_array = FDS_Interfaces::IKSolutionFile::getKDLJntArrayFromVector(joint_values);
    FDS_Interfaces::IK_Solution_And_Value ik1(8.5, jnt_array);
    FDS_Interfaces::IK_Solution_And_Value ik2(7.5, jnt_array);
    FDS_Interfaces::IK_Solution_And_Value ik3(9.5, jnt_array);

    FDS_Interfaces::IK_Solution_Set set1;
    set1.insert(ik1);
    set1.insert(ik2);
    set1.insert(ik3);

    FDS_Interfaces::IK_Solution_Set set2;
    set2.insert(ik1);
    set2.insert(ik2);

    FDS_Interfaces::IK_Solution_Set notReachable;

    std::string bp1_id = "BP1";
    std::vector<double> bp1_position{15.0, -3.0, .05};
    std::vector<double> bp1_orientation{1.0, -2.0, 0.0};
    KDL::Frame bp1_frame = FDS_Interfaces::IKSolutionFile::getKDLFrameFromVectors(bp1_position, bp1_orientation);

    std::string bp2_id = "BP2";
    std::vector<double> bp2_position{13.0, -2.8, .05};
    std::vector<double> bp2_orientation{1.0, -2.0, 0.0};
    KDL::Frame bp2_frame = FDS_Interfaces::IKSolutionFile::getKDLFrameFromVectors(bp2_position, bp2_orientation);

    FDS_Interfaces::Base_Position_Solution bps1(bp1_id, bp1_frame, task1_frame, FDS_Interfaces::IKSolutionFile::REACHABLE, set1);
    FDS_Interfaces::Base_Position_Solution bps2(bp2_id, bp2_frame, task1_frame, FDS_Interfaces::IKSolutionFile::NOT_REACHABLE, notReachable);

    FDS_Interfaces::IK_Solution reach_result1(task1_id, panel_id, task1_frame);
    reach_result1.add_base_position_solution(bps1);
    reach_result1.add_base_position_solution(bps2);
    reach_result1.message= FDS_Interfaces::IKSolutionFile::REACHABLE;

    ik_solutions.push_back(reach_result1);

    FDS_Interfaces::Base_Position_Solution bps3(bp1_id, bp1_frame, task2_frame, FDS_Interfaces::IKSolutionFile::REACHABLE, set1);
    FDS_Interfaces::Base_Position_Solution bps4(bp2_id, bp2_frame, task2_frame, FDS_Interfaces::IKSolutionFile::REACHABLE, set2);
    FDS_Interfaces::Base_Position_Solution bps5("BP3", bp2_frame, task2_frame, FDS_Interfaces::IKSolutionFile::NO_COLLISION_FREE_SOLUTIONS, notReachable);

    FDS_Interfaces::IK_Solution reach_result2(task2_id, panel_id, task2_frame);
    reach_result2.add_base_position_solution(bps3);
    reach_result2.add_base_position_solution(bps4);
    reach_result2.add_base_position_solution(bps5);
    reach_result1.message = FDS_Interfaces::IKSolutionFile::REACHABLE;

    ik_solutions.push_back(reach_result2);

    iks.write_IKSolutionFile(ik_solutions, 10);
}

int main(int argc, char** argv)
{
    FDS_Interfaces::IKSolutionFile ik_solution_file("/home/ros-industrial/fds_data/scan_0/");

    //std::cout << "***Write Ik solution file... " << std::endl;
    //test_writeIKSolutions(ik_solution_file);

    std::cout << "***Read in Ik solution file... " << std::endl;
    ik_solution_file.read_IKSolutionFile();

    std::vector<double> current_joint_values = ik_solution_file.getBestIKSolutionByTask("task_001", "");
    std::cout << "Current joint positions: " << FDS_Interfaces::IKSolutionFile::get_joint_str_from_vector(current_joint_values) << std::endl;
    current_joint_values = ik_solution_file.getClosestIKSolutionByTask("task_002", "", current_joint_values);
    std::cout << "   Next: " << FDS_Interfaces::IKSolutionFile::get_joint_str_from_vector(current_joint_values) << std::endl;
    current_joint_values = ik_solution_file.getClosestIKSolutionByTask("task_003", "", current_joint_values);
    std::cout << "   Next: " << FDS_Interfaces::IKSolutionFile::get_joint_str_from_vector(current_joint_values) << std::endl;
    current_joint_values = ik_solution_file.getClosestIKSolutionByTask("task_004", "", current_joint_values);
    std::cout << "   Next: " << FDS_Interfaces::IKSolutionFile::get_joint_str_from_vector(current_joint_values) << std::endl;

    return 0;
}
