#include <iterator>
#include <algorithm>
#include "MBUtils.h"       
#include "XYPoint.h"
#include "PointAssign.h"

using namespace std;

PointAssign::PointAssign()
{
  m_assign_by_region = false;
  m_point_counter = 0;
}

PointAssign::~PointAssign()
{
}

bool PointAssign::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  if(m_vnames.empty()) return(true);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key = msg.GetKey();

    if(key == "VISIT_POINT") {
      string sval = msg.GetString(); 

      // 1️⃣ ΠΕΡΙΠΤΩΣΗ: Πρώτο ή Τελευταίο σημείο
      if(sval == "firstpoint" || sval == "lastpoint") {
        for(size_t i = 0; i < m_vnames.size(); i++) {
          Notify("VISIT_POINT_" + toupper(m_vnames[i]), sval);
        }
      }
      // 2️⃣ ΠΕΡΙΠΤΩΣΗ: Κανονικό Σημείο προς μοιρασιά
      else {
        string target_vehicle = "";
        double x = 0;
        bool x_found = false;

        // --- Native C++ Parsing για το X ---
        size_t x_pos = sval.find("x=");
        if(x_pos != string::npos) {
          size_t comma_pos = sval.find(",", x_pos);
          string s_x = (comma_pos != string::npos) ? 
                       sval.substr(x_pos + 2, comma_pos - (x_pos + 2)) : 
                       sval.substr(x_pos + 2);
          if(!s_x.empty()) {
            x = stod(s_x);
            x_found = true;
          }
        }

        // Σενάριο Α: Μοιρασιά ανά περιοχή (East-West)
        if(m_vnames.size() >= 2 && m_assign_by_region && x_found) {
          if(m_vnames.size() >= 2 && m_assign_by_region && x_found) {
            if(x < 87.5) {
              target_vehicle = m_vnames; // <--- ΣΩΣΤΟ

              } else {
              target_vehicle = m_vnames[1]; // <--- ΣΩΣΤΟ
            }
          }
        }
        // Σενάριο Β: Εναλλάξ μοιρασιά (Alternating)
        else {
          int index = m_point_counter % m_vnames.size();
          target_vehicle = m_vnames[index];
          m_point_counter++;
        }

        // Αποστολή και Οπτικοποίηση
        if(!target_vehicle.empty()) {
          Notify("VISIT_POINT_" + toupper(target_vehicle), sval);

          // --- Native C++ Parsing για τα Y και ID (για το plot) ---
          string s_id = "", s_y = "";
          
          size_t id_pos = sval.find("id=");
          if(id_pos != string::npos) {
            size_t comma_pos = sval.find(",", id_pos);
            s_id = (comma_pos != string::npos) ? 
                   sval.substr(id_pos + 3, comma_pos - (id_pos + 3)) : 
                   sval.substr(id_pos + 3);
          }

          size_t y_pos = sval.find("y=");
          if(y_pos != string::npos) {
            size_t comma_pos = sval.find(",", y_pos);
            s_y = (comma_pos != string::npos) ? 
                  sval.substr(y_pos + 2, comma_pos - (y_pos + 2)) : 
                  sval.substr(y_pos + 2);
          }

          if(x_found && !s_y.empty() && !s_id.empty()) {
            double y = stod(s_y);
            
            string vname_low = tolower(target_vehicle);
            string color = "yellow"; 
            if(vname_low == "gilda")      color = "red";
            else if(vname_low == "henry") color = "magenta"; 

            postViewPoint(x, y, s_id, color);
          }
        }
      } 
    } 
  } 
  
  return(true);
}

bool PointAssign::Iterate()
{
  AppCastingMOOSApp::Iterate();
  return(true);
}

bool PointAssign::OnConnectToServer()
{
   RegisterVariables();
   
   //Στέλνουμε σήμα στο uTimerScript ότι είμαστε έτοιμοι!
   Notify("POINTS_REQ", "false"); // Στο uTimerScript, το false ξεπαγώνει τη pause_var
   return(true);
}

bool PointAssign::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();

  STRING_LIST sParams;
  m_MissionReader.GetConfiguration(GetAppName(), sParams);

  STRING_LIST::iterator p;
  for(p=sParams.begin(); p!=sParams.end(); p++) {
    string orig  = *p;
    string line  = *p;
    string param = tolower(biteString(line, '='));
    string value = line; 

    if(param == "vname") {
      if(!value.empty()) {
        m_vnames.push_back(tolower(value)); 
      }
    }
    else if(param == "assign_by_region") {
      m_assign_by_region = (tolower(value) == "true");
    }
  }

  m_point_counter = 0;
  RegisterVariables();  
  return(true);
}

void PointAssign::RegisterVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  Register("VISIT_POINT", 0);
  Register("VEHICLE_READY", 0); // 📌 Θα το στέλνει το σκάφος όταν ανοίγει!
}

void PointAssign::postViewPoint(double x, double y, string label, string color)
{
  XYPoint point(x, y);
  point.set_label(label);
  point.set_color("vertex", color);  
  point.set_param("vertex_size", "4");

  string spec = point.get_spec();    
  Notify("VIEW_POINT", spec);
}

bool PointAssign::buildReport() 
{
  m_msgs << "============================================" << endl;
  m_msgs << "PointAssign Status:                         " << endl;
  m_msgs << "============================================" << endl;
  m_msgs << "Vehicles tracked: " << m_vnames.size() << endl;
  m_msgs << "Assign by Region: " << (m_assign_by_region ? "Yes" : "No") << endl;
  m_msgs << "Points distributed: " << m_point_counter << endl;
  return(true);
}