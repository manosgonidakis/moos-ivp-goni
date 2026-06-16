/************************************************************/
/* NAME: Emmanouil                                       */
/* ORGN: MIT, Cambridge MA                               */
/* FILE: Odometry.cpp                                    */
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
  m_first_reading  = true;
  m_current_x      = 0.0;
  m_current_y      = 0.0;
  m_previous_x     = 0.0;
  m_previous_y     = 0.0;
  m_total_distance = 0.0;
  m_last_nav_time  = 0.0;
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
      cout << "nav_x: " << m_current_x << endl;
    }
    else if(key == "NAV_Y") {
      m_current_y = msg.GetDouble();
      m_last_nav_time = msg.GetTime();
      cout << "nav_y: " << m_current_y << endl; // <-- ΔΙΟΡΘΩΘΗΚΕ: Μπήκε το ';'
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

  // Έλεγχος Staleness (Άσκηση 5.2)
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
  }

  m_previous_x = m_current_x;
  m_previous_y = m_current_y;

  Notify("ODOMETRY_DIST", m_total_distance);
  cout << "odometry_dist calculated: " << m_total_distance << endl; //

  AppCastingMOOSApp::PostReport();
  return(true);
}

//---------------------------------------------------------
// Procedure: OnStartUp()
// <-- ΔΙΟΡΘΩΘΗΚΕ: Καθαρίστηκε το περίεργο σχόλιο της γραμμής 119!
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
    string value = line;

    bool handled = false;
    if(param == "foo") {
      handled = true;
    }
    else if(param == "bar") {
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
}

//------------------------------------------------------------
// Procedure: buildReport()
bool Odometry::buildReport() 
{
  m_msgs << "============================================" << endl;
  m_msgs << "pOdometry Mission Status                    " << endl;
  m_msgs << "============================================" << endl;
  
  ACTable actab(3);
  actab << "Current X | Current Y | Total Distance";
  actab.addHeaderLines();
  actab << m_current_x << m_current_y << m_total_distance;
  m_msgs << actab.getFormattedString();

  return(true);
}