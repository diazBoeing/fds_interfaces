/*! \class      IKSolutionFile
 *  \brief      Methods to read and save Inverse Kinematic (IK) solutions for a set of point locations/orientations (ie, frames)
 *  \date       Sep 23, 2021
 *  \author     Michelle Crivella
 *  \copyright  BOEING PROPRIETARY, CONFIDENTAL, AND/OR TRADE SECRET, COPYRIGHT 2021/ THE BOEING COMPANY,  UNPUBLISHED WORK.  ALL RIGHTS RESERVED.
*/

#ifndef IK_SOLUTION_FILE_H
#define IK_SOLUTION_FILE_H

#include <string>
#include <set>
#include <vector>

#include <kdl/frames.hpp>
#include <kdl/jntarray.hpp>

#include <kdl_conversions/kdl_msg.h>        // get poseArray
#include <geometry_msgs/Pose.h>             // get poseArray
#include <geometry_msgs/PoseArray.h>        // get poseArray

#include <visualization_msgs/MarkerArray.h>

namespace FDS_Interfaces
{
    class IK_Solution_And_Value
    {
        public:
            IK_Solution_And_Value(double v, KDL::JntArray jv)
            {
                value = v;
                solution = jv;
            }
            ~IK_Solution_And_Value() {  }

            KDL::JntArray solution;
            double value;

            bool operator<(const IK_Solution_And_Value& t) const
            {
                return (this->value < t.value);
            }
    };            // IK Solution and its value
    typedef std::set<FDS_Interfaces::IK_Solution_And_Value> IK_Solution_Set;                        // ordered set of solutions

    class Base_Position_Solution
    {
        public:
            Base_Position_Solution(std::string n, KDL::Frame bp_frame, KDL::Frame task_frame, std::string m, IK_Solution_Set s)
            {
                base_position_id = n;
                base_position_frame = bp_frame;
                task_new_pose = task_frame;
                result_from_this_bp = m;
                ordered_solutions = s;
            }

            Base_Position_Solution()  {  }
            ~Base_Position_Solution()  {  }

            KDL::JntArray getBestIKSolution()
            {
                return ordered_solutions.begin()->solution;
            }

            std::string base_position_id;
            KDL::Frame base_position_frame;
            KDL::Frame task_new_pose;
            std::string result_from_this_bp;
            IK_Solution_Set ordered_solutions;
    };

    class IK_Solution
    {
        public:
            IK_Solution() {  }
            IK_Solution(std::string n, std::string pn, KDL::Frame p)
            {
                task_id = n;
                panel_id = pn;
                task_pose = p;
            }
            ~IK_Solution() {  }

            void add_base_position_solution(FDS_Interfaces::Base_Position_Solution bps)
            {
                reachability_results.push_back(bps);
            }

            std::string task_id;
            std::string panel_id;
            KDL::Frame task_pose;
            std::string message;
            std::vector<FDS_Interfaces::Base_Position_Solution> reachability_results;
    };

    class IKSolutionFile
    {
        public:
            IKSolutionFile(std::string folder);
            ~IKSolutionFile();

            static constexpr char* NOT_REACHABLE = "NOT REACHABLE";
            static constexpr char* NO_COLLISION_FREE_SOLUTIONS = "NO COLLISION FREE SOLUTIONS";
            static constexpr char* REACHABLE = "REACHABLE";

            bool write_IKSolutionFile(std::vector<FDS_Interfaces::IK_Solution> ik_solutions, int max_results);
            bool write_IKSolutionFile2(std::vector<FDS_Interfaces::IK_Solution> ik_solutions, int max_results);
            bool read_IKSolutionFile();

            std::vector<std::string> getPanelNames();
            void getTaskSolutionsByPanelId(std::string panel_id, std::vector<FDS_Interfaces::IK_Solution>& panel_points);
            bool getBestBasePositionByPanel(std::string panel_id, Base_Position_Solution& bps);
            std::vector<std::string> getTasksByPanel(std::string panel_id, std::string status);
            std::vector<std::string> getTasksByPanelAndBP(std::string panel_id, std::string bp_id, std::string status);
            std::vector<std::vector<double>> getBestIKSolutions(std::string panel_id);
            std::vector<std::vector<double>> getClosestIKSolutions(std::string bp_id);
            std::vector<double> getBestIKSolutionByTask(std::string task_id, std::string bp_id);
            std::vector<double> getClosestIKSolutionByTask(std::string task_id, std::string bp_id, std::vector<double> current_joint_positions);
            double calculateDistanceBetweenJointPositions(std::vector<double> jv1, std::vector<double> jv2);

            //std::vector<FDS_Interfaces::IK_Solution> getIKSolutions(std::string );
            //bool getIKSolutionByTaskPanelID(std::string panel_id, std::string task_id, FDS_Interfaces::IK_Solution& result);
            visualization_msgs::MarkerArray getMarkerArrayByPanelId(std::string panel_id, std::string frame);
            geometry_msgs::PoseArray getPoseArrayByPanelId(std::string panel_id, const std::string &frame);
            visualization_msgs::MarkerArray getMarkerArrayByBasePositionId(std::string bp_id, std::string frame);
            geometry_msgs::PoseArray getPoseArrayByBasePosition(std::string bp_id, const std::string &frame);

            static void printIKSolutionResultsForMatlab(std::vector<FDS_Interfaces::IK_Solution>& ik_solutions);
            static std::vector<double> KDLJntArrayToStdVec(KDL::JntArray& jnts);
            static void printPosesForMatlab(std::string varName, std::vector<KDL::Frame> poses);

            static std::string get_joint_str(KDL::JntArray jnts);
            static std::string get_joint_str_from_vector(std::vector<double> jnts);
            static std::string get_pose_str(KDL::Frame pose);
            static KDL::Frame getKDLFrameFromVectors(std::vector<double> position, std::vector<double> rpy);
            static KDL::JntArray getKDLJntArrayFromVector(std::vector<double> jnt_values);
            static std::vector<double> getPositionVectorFromKDLFrame(KDL::Frame pose);
            static std::vector<double> getOrientationVectorFromKDLFrame(KDL::Frame pose);

        private:
            std::string folder_;
            std::string filename_;
            std::vector<IK_Solution> ik_solutions_;


    };
}

#endif // IK_SOLUTION_FILE_H
