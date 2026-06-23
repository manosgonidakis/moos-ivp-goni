/************************************************************/
/*    NAME: Gonidakis                                       */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: GenRescue.h                                     */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef GenRescue_HEADER
#define GenRescue_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include <map>
#include <set>
#include <string>
#include <vector>
#include "XYPoint.h"

class GenRescue : public AppCastingMOOSApp
{
 public:
   GenRescue();
   ~GenRescue();

 protected: // Standard MOOSApp functions to overload  
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload 
   bool buildReport();

 protected:
   void RegisterVariables();

 private: // Δικές μας μεταβλητές για την πλοήγηση
   
   double m_nav_x;
   double m_nav_y;
   double m_nav_heading;
   double m_nav_speed;
   double m_enemy_x;
   double m_enemy_y;

   // Λίστα (Map) για την αποθήκευση των κολυμβητών: (ID -> XYPoint)
   bool m_first_run_done; 

   std::map<std::string, XYPoint> m_swimmers;
   std::set<std::string> m_rescued_swimmers;
   std::vector<XYPoint> m_obstacles;
   std::string m_obstacle_source;
   std::vector<XYPoint> m_region;
   bool m_path_update_needed;

 private: // State variables
};

#endif 
