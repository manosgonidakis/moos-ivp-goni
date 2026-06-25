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
   double m_buoy_block;           // swimmers <buoy_block από buoy → ΔΕΝ τους στοχεύουμε (loop avoidance)
   double m_edge_block;           // swimmers <edge_block από το όριο → ΔΕΝ τους στοχεύουμε (επικίνδυνο)

   // Inside-approach: swimmers κοντά στο όριο τους πλησιάζουμε ΑΠΟ ΜΕΣΑ (παρεμβολή
   // approach-waypoint προς το κέντρο) → λιγότερο momentum προς τα έξω, λιγότερα freezes.
   bool   m_inside_approach;
   double m_approach_edge;        // swimmer <approach_edge από όριο → χρησιμοποίησε inside approach
   double m_approach_dist;        // πόσο μέσα μπαίνει το approach-waypoint

   // Enemy-pressure (legal shepherding): πλησιάζουμε εχθρό κοντά στο όριο ώστε η
   // ΔΙΚΗ ΤΟΥ avoidance να τον σπρώξει εκτός. Καμία σύγκρουση (κρατάμε press_gap).
   std::map<std::string, XYPoint> m_contacts;  // όνομα→θέση όλων των άλλων οχημάτων
   std::string m_teammate;        // όνομα scout μας (εξαιρείται από εχθρούς)
   bool   m_enemy_press;          // flag
   double m_press_edge;           // enemy <press_edge από όριο → υποψήφιος
   double m_press_range;          // μόνο αν είμαστε <press_range από τον enemy (opportunistic)
   double m_press_gap;            // πόσο κοντά πλησιάζουμε (>collision range)

   double m_press_dur;            // διάρκεια μιας σπρωξιάς (s)
   double m_press_cd;             // cooldown πριν την επόμενη σπρωξιά (s)
   double m_press_until;          // active press μέχρι αυτόν τον χρόνο
   double m_press_cd_until;       // δεν ξεκινά νέα press πριν αυτόν τον χρόνο

   // Self-defense: αν εχθρός μας στριμώχνει στο όριο, τραβιόμαστε προς το κέντρο
   // (αποφυγή OpRegion halt/πάγωμα στην άκρη).
   bool   m_self_defend;          // flag
   double m_defend_edge;          // είμαστε <defend_edge από όριο → ευάλωτοι
   double m_defend_range;         // εχθρός <defend_range από εμάς → απειλή
   double m_defend_push;          // πόσο προς το κέντρο τραβιόμαστε
   double m_defend_dur;           // διάρκεια μιας αμυντικής κίνησης (s)
   double m_defend_cd;            // cooldown πριν την επόμενη (s)
   double m_defend_until;
   double m_defend_cd_until;

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
