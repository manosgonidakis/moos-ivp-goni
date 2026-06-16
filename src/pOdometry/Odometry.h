/************************************************************/
/* NAME: Emmanouil                                       */
/* ORGN: MIT, Cambridge MA                               */
/* FILE: Odometry.h                                      */
/************************************************************/

#ifndef Odometry_HEADER
#define Odometry_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"

class Odometry : public AppCastingMOOSApp
{
 public:
   Odometry();
   ~Odometry();

 protected: // Standard MOOSApp functions to overload  
   bool OnNewMail(MOOSMSG_LIST &NewMail);
   bool Iterate();
   bool OnConnectToServer();
   bool OnStartUp();

 protected: // Standard AppCastingMOOSApp function to overload 
   bool buildReport();

 protected:
   void registerVariables();

 private: // State variables
   bool   m_first_reading; // <-- ΔΙΟΡΘΩΘΗΚΕ: Έφυγε το 'x' από εδώ!
   double m_current_x;
   double m_current_y;
   double m_previous_x;
   double m_previous_y;
   double m_total_distance;
   double m_last_nav_time; // Για το Staleness Check
   double m_depth_thresh;          // Read Limit
   double m_odometry_dist_at_depth; //distance from lower limit
   double m_nav_depth;
};

#endif