/************************************************************/
/* NAME: Emmanouil                                          */
/* ORGN: MIT, Cambridge MA                                  */
/* FILE: Odometry.cpp                                       */
/************************************************************/

#include <iterator>
#include <cmath>
#include "MBUtils.h"
#include "ACTable.h"
#include "Odometry.h"

using namespace std;

//---------------------------------------------------------
// Constructor()
Odometry::Odometry()
{
  m_first_reading          = true;
  m_current_x              = 0.0;
  m_current_y              = 0.0;
  m_previous_x             = 0.0;
  m_previous_y             = 0.0;
  m_total_distance         = 0.0;
  m_last_nav_time          = 0.0;
  m_depth_thresh           = 0.0; // <-- ΔΙΟΡΘΩΘΗΚΕ: Μπήκε το ';'
  m_odometry_dist_at_depth = 0.0;
  m_nav_depth              = 0.0;
}

//---------------------------------------------------------
// Destructor
Odometry::~Odometry()
{
}

//---------------------------------------------------------
// Procedure: OnNewMail()
bool Odometry::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key    = msg.GetKey();

    if (key == "NAV_X") {
      m_current_x = msg.GetDouble();
      m_last_nav_time = msg.GetTime(); 
    }
    else if(key == "NAV_Y") {
      m_current_y = msg.GetDouble();
      m_last_nav_time = msg.GetTime();
    }
    // 📌 ΠΡΟΣΘΕΣΕ ΑΥΤΟ: Διαβάζουμε το βάθος από το UUV
    else if(key == "NAV_DEPTH") {
      m_nav_depth = msg.GetDouble();
    }
    else if(key != "APPCAST_REQ") {
      reportRunWarning("Unhandled Mail: " + key);
    }
  }
   return(true);
}

//---------------------------------------------------------
// Procedure: OnConnectToServer()
bool Odometry::OnConnectToServer()
{
   registerVariables();
   return(true);
}

//---------------------------------------------------------
// Procedure: Iterate()
bool Odometry::Iterate()
{
  AppCastingMOOSApp::Iterate();

  // Staleness
  double current_moos_time = MOOSTime();
  if (m_last_nav_time > 0 && (current_moos_time - m_last_nav_time >= 10.0)) {
    reportRunWarning("NAV data is stale! No update for over 10 seconds.");
  } else {
    retractRunWarning("NAV data is stale! No update for over 10 seconds.");
  }

  // Υπολογισμός Ευκλείδη
  if (m_first_reading) {
    m_previous_x = m_current_x;
    m_previous_y = m_current_y;
    m_first_reading = false;
  }
  else {
    double dx = m_current_x - m_previous_x;
    double dy = m_current_y - m_previous_y;
    double step_distance = hypot(dx, dy);

    m_total_distance += step_distance;

    // 📌 ΔΙΟΡΘΩΘΗΚΕ: Χρήση του step_distance αντί του distance_increment
    if (m_nav_depth > m_depth_thresh) {
        m_odometry_dist_at_depth += step_distance;
    }

    // Δημοσίευση της νέας τιμής στη MOOSDB
    Notify("ODOMETRY_DIST_AT_DEPTH", m_odometry_dist_at_depth);
  }

  m_previous_x = m_current_x;
  m_previous_y = m_current_y;

  Notify("ODOMETRY_DIST", m_total_distance);

  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
bool Odometry::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if(!m_MissionReader.GetConfiguration(GetAppName(), sParams))
    reportConfigWarning("No config block found for " + GetAppName());

  STRING_LIST::iterator p;
  for(p=sParams.begin(); p!=sParams.end(); p++) {
    string orig  = *p;
    string line  = *p;
    string param = tolower(biteStringX(line, '='));
    string value = stripBlankEnds(line);

    bool handled = false;
    if(param == "foo") {
      handled = true;
    }
    else if(param == "bar") {
      handled = true;
    }
    // 📌 ΠΡΟΣΘΕΣΕ ΑΥΤΟ: Διαβάζει σωστά το depth_thresh από το .moos αρχείο
    else if(param == "depth_thresh") {
      m_depth_thresh = atof(value.c_str());
      handled = true;
    }

    if(!handled)
      reportUnhandledConfigWarning(orig);
  }
  
  registerVariables();  
  return(true);
}

//---------------------------------------------------------
// Procedure: registerVariables()
void Odometry::registerVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("NAV_X", 0);
  Register("NAV_Y", 0);
  Register("NAV_DEPTH", 0); // <-- ΠΡΟΣΘΕΣΕ ΑΥΤΟ!
}

//------------------------------------------------------------
// Procedure: buildReport()
bool Odometry::buildReport() 
{
  m_msgs << "============================================" << endl;
  m_msgs << "pOdometry Mission Status                    " << endl;
  m_msgs << "============================================" << endl;
  m_msgs << "Distance at Depth Threshold: " << m_odometry_dist_at_depth << " m" << endl;
  m_msgs << "Depth Threshold Set: " << m_depth_thresh << " m" << endl;
  m_msgs << "============================================" << endl;
  
  ACTable actab(3);
  actab << "Current X | Current Y | Total Distance";
  actab.addHeaderLines();
  actab << m_current_x << m_current_y << m_total_distance;
  m_msgs << actab.getFormattedString();

  return(true);
}