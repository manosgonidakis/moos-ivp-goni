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
#include <string>
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
   
   double m_nav_x; // Η τρέχουσα θέση X του σκάφους
   double m_nav_y; // Η τρέχουσα θέση Y του σκάφους

   // Λίστα (Map) για την αποθήκευση των κολυμβητών: (ID -> XYPoint)
   std::map<std::string, XYPoint> m_swimmers;

 private: // State variables
};

#endif 