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
   double m_enemy_heading;
   double m_enemy_speed;

   // Λίστα (Map) για την αποθήκευση των κολυμβητών: (ID -> XYPoint)
   bool m_first_run_done;
   bool m_returned;
   bool m_deadlock_escape;        // A/B feature flag (από config)
   bool m_use_exact_tsp;          // exact TSP (default OFF — μετρήθηκε πιο αργό)
   bool m_flyby_rescue;           // flyby (default OFF — A/B testing)
   bool m_cluster_split;          // cluster-based planning (default ON — anti-whipsaw, NO concede)
   double m_cluster_link_dist;    // single-linkage distance (m) για ομαδοποίηση swimmers

   // Deadlock-Escape state
   double m_stuck_ref_x;
   double m_stuck_ref_y;
   double m_stuck_ref_time;
   bool   m_stuck_init;
   double m_escape_until;
   std::map<std::string, double> m_buoy_blacklist;     // id -> expiry time (temp)
   std::map<std::string, int>    m_swimmer_escape_count; // id -> πόσες φορές προκάλεσε escape
   std::set<std::string>         m_abandoned;          // μόνιμα εγκαταλειμμένοι (repeated deadlock)

   std::map<std::string, XYPoint> m_swimmers;
   std::set<std::string> m_rescued_swimmers;
   std::map<std::string, double> m_swimmer_approach_times;
   std::vector<XYPoint> m_obstacles;
   std::string m_obstacle_source;
   std::vector<XYPoint> m_region;
   bool m_path_update_needed;

 private: // State variables
};

#endif 
