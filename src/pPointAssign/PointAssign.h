/************************************************************/
/*    NAME: manosgonidakis                                              */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: PointAssign.h                                          */
/*    DATE: December 29th, 1963                             */
/************************************************************/
#ifndef PointAssign_HEADER
#define PointAssign_HEADER

#include "MOOS/libMOOS/Thirdparty/AppCasting/AppCastingMOOSApp.h"
#include <vector>
#include <string>

class PointAssign : public AppCastingMOOSApp
{
public:
  PointAssign();
  ~PointAssign();

protected:
  bool OnNewMail(MOOSMSG_LIST &NewMail);
  bool Iterate();
  bool OnConnectToServer();
  bool OnStartUp();
  bool buildReport();
  void RegisterVariables(); // 📌 ΑΥΤΗ Η ΓΡΑΜΜΗ ΕΛΕΙΠΕ ΚΑΙ ΠΡΟΚΑΛΟΥΣΕ ΤΟ ERROR 5!

private:
  std::vector<std::string> m_vnames;           
  bool                     m_assign_by_region; 
  unsigned int             m_point_counter;    

  void postViewPoint(double x, double y, std::string label, std::string color);
  std::vector<std::string> m_all_received_points; // Αποθήκη για τα σημεία
  std::vector<std::string> m_ready_vehicles;       // Ποια σκάφη δήλωσαν έτοιμα
};

#endif