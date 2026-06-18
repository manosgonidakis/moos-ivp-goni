/************************************************************/
/*    NAME: Gonidakis                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: GenPath.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/

#ifndef GenPath_HEADER
#define GenPath_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"

class GenPath : public AppCastingMOOSApp
{
 public:
   GenPath();
   ~GenPath();

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
  std::vector<std::string> m_point_queue; // Αποθήκη για τα VISIT_POINT που καταφθάνουν
  bool                     m_got_last_point; // Έγινε true όταν έρθει το "lastpoint"
  
  double                   m_nav_x; // Η τρέχουσα θέση X του σκάφους
  double                   m_nav_y; // Η τρέχουσα θέση Y του σκάφους

 private: // State variables
};

#endif 
