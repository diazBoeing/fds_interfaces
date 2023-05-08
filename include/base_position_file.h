/*! \class      BasePositionFile
 *  \brief      Methods to read the base positions for each panel of the aircraft data 
 *  \date       Oct 14, 2021
 *  \author     Michelle Crivella
 *  \copyright  BOEING PROPRIETARY, CONFIDENTAL, AND/OR TRADE SECRET, COPYRIGHT 2021/ THE BOEING COMPANY,  UNPUBLISHED WORK.  ALL RIGHTS RESERVED.
*/

#ifndef BASEPOSITION_DATA_FILE_H
#define BASEPOSITION_DATA_FILE_H

#include <string>
#include <vector>

#include <kdl/frames.hpp>
#include <visualization_msgs/MarkerArray.h>

namespace FDS_Interfaces
{
    typedef std::pair<std::string, KDL::Frame> BasePosition;
    class PanelBasePositions
    {
        public:
            PanelBasePositions()  {   }
            PanelBasePositions(std::string p)
            {
                panel_id_ = p;
            }

            ~PanelBasePositions() {  }

            std::string panel_id_;
            std::vector<BasePosition> base_positions_;
    };

    class BasePositionFile
    {
        public:
            BasePositionFile(std::string folder);
            ~BasePositionFile();

            bool write_BasePositionFile(std::vector<FDS_Interfaces::PanelBasePositions> all_positions);
            bool read_BasePositionFile();

            FDS_Interfaces::PanelBasePositions getBasePositionsByPanelName(std::string panel_id);
            void printBasePositionsForMatlab(std::string panel_id);
            void printBaseQuaternionsForMatlab(std::string panel_id);
	    
       private:
            std::string folder_;
            std::string filename_;
            std::vector<FDS_Interfaces::PanelBasePositions> all_aircraft_base_positions_;
    };
}

#endif // BASEPOSITION_DATA_FILE_H
