/*! \class      no cpp, just a header file.  FDSInterfaces::SM_Commands
 *  \brief      Commands that can be sent to the Flexible Drill System state machine
 *  \date       Aug 11, 2021
 *  \author     Michelle Crivella
 *  \copyright  BOEING PROPRIETARY, CONFIDENTAL, AND/OR TRADE SECRET, COPYRIGHT 2021/ THE BOEING COMPANY,  UNPUBLISHED WORK.  ALL RIGHTS RESERVED.
*/

#ifndef  FDSSM_COMMANDS_INCLUDE
#define FDSSM_COMMANDS_INCLUDE

namespace FDS_Interfaces
{
    const static std::string FOLLOW_TRAJ_ACTION_SERVICE = "/joint_trajectory_action";
    //const static std::string FOLLOW_TRAJ_ACTION_SERVICE = "/follow_joint_trajectory";
    const static std::string A5_GENERIC_FOLLOW_TRAJ_ACTION_SERVICE = "execute_generic_process";
    const static std::string A5_SCAN_FOLLOW_TRAJ_ACTION_SERVICE = "detailed_scan_execution";
    const static std::string TOOL_ENABLE_SERVICE = "tool_enable";
    const static std::string TOOL_RUN_SERVICE = "tool_run";

    enum SM_Commands
    {
        SET_DATA_FOLDER,
        SET_EE_TYPE,

        ENABLE_ROBOT,
        DISABLE_ROBOT,
        PAUSE_ROBOT,

        MOVE_TO_POSE,
        SPIN_ROBOT,

        TOGGLE_LIGHT,
        TOGGLE_IFM,
        CHECK_EE_STATUS,

        MOVE_TOOL_Z,
        MOVE_CAMERA_TO_WAYPOINT,
        MOVE_TOOL_TO_WAYPOINT,

        GENERATE_IFM_MOTION_PLAN_FROM_WAYPOINTS,
        GENERATE_CAMERA_MOTION_PLAN_FROM_WAYPOINTS,
        GENERATE_TOOL_MOTION_PLAN_FROM_WAYPOINTS,

        GENERATE_MOTION_PLAN_FROM_POSES,
        GENERATE_CAMERA_WAYPOINT_PLAN_FROM_MESH,

        VISUALIZE_MOTION_PLAN,

        EXECUTE_MOTION_PLAN_DEMO,
        EXECUTE_MOTION_PLAN,

        DRILL_ONLY,
        DETECT_PART,

        FINE_TUNE_POSITION,
        EXECUTE_ALL_MACHINING_TASKS,

        COLLECT_CAMERA_IMAGE,
        COLLECT_3D_DATA,
        GRAB_POINTCLOUD,
        AGGREGATE_SCANS,
        CREATE_MESH,
        VISUALIZE_MESH,
        SET_CAMERA_EXPOSURE,
        DETECT_CYCLE_TARGET,

        COUPLE_EE,
        RELEASE_EE,
        RUN_DRILL,
        RUN_DEFAST,
        STOP_SYSTEM,
    };

}

#endif /* FDSSM_COMMANDS_INCLUDE */

