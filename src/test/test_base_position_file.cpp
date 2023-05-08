/*! \file       test_base_position_file.cpp
 *  \brief      test reading and printing results of a Base Position File
 *  \date       Oct 14, 2021
 *  \author     Michelle Crivella
 *  \copyright  BOEING PROPRIETARY, CONFIDENTAL, AND/OR TRADE SECRET, COPYRIGHT 2021/ THE BOEING COMPANY,  UNPUBLISHED WORK.  ALL RIGHTS RESERVED.
*/

#include <iostream>
#include <base_position_file.h>

int main(int argc, char** argv)
{
    FDS_Interfaces::BasePositionFile bpf("/home/ros-industrial/fds_data/aircraft/");
    bpf.read_BasePositionFile();
    //bpf.printBasePositionsForMatlab("Outboard_Torque_Box_and_Panel_100_B");
    bpf.printBasePositionsForMatlab("");

    return 0;
}
