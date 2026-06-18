#include <iterator>
#include <cmath>
#include "MBUtils.h"
#include "XYSegList.h" 
#include "GenPath.h"

using namespace std;

GenPath::GenPath()
{
  m_got_last_point = false;
  m_nav_x = 0;
  m_nav_y = 0;
}

GenPath::~GenPath()
{
}

bool GenPath::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key = msg.GetKey();

    if(key == "NAV_X") {
      m_nav_x = msg.GetDouble();
    }
    else if(key == "NAV_Y") {
      m_nav_y = msg.GetDouble();
    }
    else if(key == "VISIT_POINT") {
      string sval = msg.GetString();

      if(sval == "firstpoint") {
        m_point_queue.clear();
        m_got_last_point = false;
      }
      else if(sval == "lastpoint") {
        m_got_last_point = true;
      }
      else {
        m_point_queue.push_back(sval);
      }
    }
  }
  return(true);
}

bool GenPath::Iterate()
{
  AppCastingMOOSApp::Iterate();

  if(m_got_last_point && !m_point_queue.empty()) {
    
    XYSegList my_path; 
    double current_x = m_nav_x;
    double current_y = m_nav_y;

    vector<string> remaining_points = m_point_queue;

    while(!remaining_points.empty()) {
      int best_index = -1;
      double min_distance = -1;
      double best_x = 0, best_y = 0;

      // 📌 ΕΔΩ ΕΙΝΑΙ Ο FOR LOOP ΠΟΥ ΛΕΓΑΜΕ:
      for(size_t i = 0; i < remaining_points.size(); i++) {
        string sval = remaining_points[i];
        double px = 0, py = 0;
        bool x_ok = false, y_ok = false;
        
        // --- Parsing του X ---
        size_t x_pos = sval.find("x=");
        if(x_pos != string::npos) {
          size_t comma_pos = sval.find(",", x_pos);
          string s_x = (comma_pos != string::npos) ? sval.substr(x_pos + 2, comma_pos - (x_pos + 2)) : sval.substr(x_pos + 2);
          if(!s_x.empty()) {
            try {
              px = stod(s_x); // 📌 Θωράκιση
              x_ok = true;
            } catch (...) { x_ok = false; } // Αν αποτύχει, απλά αγνόησέ το
          }
        }

        // --- Parsing του Y ---
        size_t y_pos = sval.find("y=");
        if(y_pos != string::npos) {
          size_t comma_pos = sval.find(",", y_pos);
          string s_y = (comma_pos != string::npos) ? sval.substr(y_pos + 2, comma_pos - (y_pos + 2)) : sval.substr(y_pos + 2);
          if(!s_y.empty()) {
            try {
              py = stod(s_y); // 📌 Θωράκιση
              y_ok = true;
            } catch (...) { y_ok = false; } // Αν αποτύχει, απλά αγνόησέ το
          }
        }

        if(x_ok && y_ok) {
          double dist = hypot(px - current_x, py - current_y);
          if(min_distance < 0 || dist < min_distance) {
            min_distance = dist;
            best_index = i;
            best_x = px;
            best_y = py;
          }
        }
      } 

      if(best_index != -1) {
        my_path.add_vertex(best_x, best_y);
        current_x = best_x;
        current_y = best_y;
        remaining_points.erase(remaining_points.begin() + best_index);
      } else {
        break; 
      }
    } // Εδώ κλείνει το while

    // 📌 ΠΡΟΣΘΕΣΕ ΑΥΤΟΝ ΤΟΝ ΕΛΕΓΧΟ ΕΔΩ:
    if(my_path.size() > 0) {
      // Μορφοποίηση της διαδρομής για το Helm
      string update_str = "points=" + my_path.get_spec(); 
      
      // Ενημέρωση των μεταβλητών στη MOOSDB
      Notify("GENPATH_POINTS", update_str);
      Notify("TRAVERSE_POINTS", "true");
    }

    // Καθαρίζουμε πάντα στο τέλος για να είμαστε έτοιμοι για τον επόμενο γύρο
    m_point_queue.clear();
    m_got_last_point = false;
  }

  return(true);
}
bool GenPath::OnConnectToServer()
{
  RegisterVariables();
  return(true);
}

bool GenPath::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();
  RegisterVariables();  
  return(true);
}

void GenPath::RegisterVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("NAV_X", 0);
  Register("NAV_Y", 0);
  Register("VISIT_POINT", 0); 
}

bool GenPath::buildReport() 
{
  m_msgs << "============================================" << endl;
  m_msgs << "GenPath Status:                             " << endl;
  m_msgs << "============================================" << endl;
  m_msgs << "Points in queue: " << m_point_queue.size() << endl;
  m_msgs << "Got last point:  " << (m_got_last_point ? "Yes" : "No") << endl;
  return(true);
}