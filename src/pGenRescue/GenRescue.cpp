#include <iterator>
#include <cmath>
#include "MBUtils.h"
#include "XYSegList.h" 
#include "GenRescue.h"

using namespace std;

// Constructor
GenRescue::GenRescue()
{
  m_nav_x = 0;
  m_nav_y = 0;
  m_enemy_x = 0; // Αρχικοποίηση θέσης αντιπάλου
  m_enemy_y = 0;
  m_path_update_needed = false;
  m_first_run_done = false;
}

// Destructor
GenRescue::~GenRescue()
{
}

bool GenRescue::OnConnectToServer()
{
  RegisterVariables();
  return(true);
}

bool GenRescue::OnNewMail(MOOSMSG_LIST &NewMail)
{
  AppCastingMOOSApp::OnNewMail(NewMail);

  MOOSMSG_LIST::iterator p;
  for(p=NewMail.begin(); p!=NewMail.end(); p++) {
    CMOOSMsg &msg = *p;
    string key = msg.GetKey();

    // 1. Ενημέρωση της δικής μας θέσης
    if(key == "NAV_X") {
      m_nav_x = msg.GetDouble();
    }
    else if(key == "NAV_Y") {
      m_nav_y = msg.GetDouble();
    }
    
    // 📌 ΝΕΟ: Ενημέρωση της θέσης του αντιπάλου (είτε παίζεις με το 'ben' είπε με το 'abe')
    else if(key == "NAV_X_BEN" || key == "NAV_X_ABE") {
      if(m_host_community != msg.GetCommunity()) { // Σιγουρευόμαστε ότι είναι ο άλλος
        m_enemy_x = msg.GetDouble();
      }
    }
    else if(key == "NAV_Y_BEN" || key == "NAV_Y_ABE") {
      if(m_host_community != msg.GetCommunity()) {
        m_enemy_y = msg.GetDouble();
      }
    }
    
    // 2. Λήψη νέου κολυμβητή από τον Διαιτητή (uFldRescueMgr) [cite: 156, 184]
    else if(key == "SWIMMER_ALERT" && msg.IsString()) {
      string sval = msg.GetString(); 

      double x = 0;
      double y = 0;
      string id = "";

      vector<string> svector = parseString(sval, ',');
      for(unsigned int i=0; i<svector.size(); i++) {
        string param = biteStringX(svector[i], '=');
        string value = svector[i];
        
        param = stripBlankEnds(param);
        value = stripBlankEnds(value);
        
        if(param == "x") x = atof(value.c_str());
        else if(param == "y") y = atof(value.c_str());
        else if(param == "id" || param == "name") id = value; 
      }

      bool is_active = (m_swimmers.find(id) != m_swimmers.end());
      bool is_rescued = (m_rescued_swimmers.find(id) != m_rescued_swimmers.end());

      if (!is_active && !is_rescued) {
          XYPoint new_swimmer(x, y);
          new_swimmer.set_label(id);
          m_swimmers[id] = new_swimmer;
          m_path_update_needed = true; 
      }
    }
    
    // 3. Ειδοποίηση ότι ένας κολυμβητής διασώθηκε [cite: 349]
    else if(key == "FOUND_SWIMMER" && msg.IsString()) {
      string sval = msg.GetString(); 
      string id = "";
      string finder = "";

      vector<string> svector = parseString(sval, ',');
      for(unsigned int i=0; i<svector.size(); i++) {
        string param = biteStringX(svector[i], '=');
        string value = svector[i];
        
        param = stripBlankEnds(param);
        value = stripBlankEnds(value);
        
        if(param == "id" || param == "name") id = value;
        else if(param == "finder") finder = value;
      }

      if (m_swimmers.find(id) != m_swimmers.end()) {
          m_swimmers.erase(id);
          m_path_update_needed = true;
      }
      
      m_rescued_swimmers.insert(id);
    }
  }
  return(true);
}

bool GenRescue::Iterate()
{
  AppCastingMOOSApp::Iterate();

  if(m_swimmers.empty()) {
    AppCastingMOOSApp::PostReport();
    return(true);
  }

  // --- 1. ΕΛΕΓΧΟΣ ΚΑΙ ΑΠΟΣΤΟΛΗ RESCUE_REQUEST --- [cite: 185]
  for(auto const& swimmer_pair : m_swimmers) {
    std::string id = swimmer_pair.first;
    XYPoint point = swimmer_pair.second;
    
    double dist = hypot(point.x() - m_nav_x, point.y() - m_nav_y);

    if(dist <= 5.0) { // [cite: 186]
      string request = "vname=" + m_host_community + ",name=" + id;
      Notify("RESCUE_REQUEST", request); // [cite: 185]
    }
  }

  // --- 2. ΕΝΗΜΕΡΩΣΗ ΔΙΑΔΡΟΜΗΣ (Με Διορθώσεις Claude Code) ---
  // Αντικαταστήσαμε το static με το μέλος m_first_run_done
  if (m_path_update_needed || (!m_first_run_done && !m_swimmers.empty())) {
      
      XYSegList my_path; 
      double current_x = m_nav_x;
      double current_y = m_nav_y;

      map<string, XYPoint> remaining_swimmers = m_swimmers;

      while(!remaining_swimmers.empty()) {
        string best_id = "";
        double min_distance = -1;
        double best_x = 0, best_y = 0;

        for(auto const& swimmer_pair : remaining_swimmers) {
          std::string id = swimmer_pair.first;
          XYPoint point = swimmer_pair.second;
          double px = point.x();
          double py = point.y();
          
          double dist = hypot(px - current_x, py - current_y);
          double dist_to_enemy = hypot(px - m_enemy_x, py - m_enemy_y);

          bool enemy_known = (m_enemy_x != 0 || m_enemy_y != 0);
          if (enemy_known && dist_to_enemy < dist && remaining_swimmers.size() > 1) {
              continue;
          }

          if(min_distance < 0 || dist < min_distance) {
            min_distance = dist; 
            best_id = id;
            best_x = px;
            best_y = py;
          }
        } 

        if(best_id != "") {
          my_path.add_vertex(best_x, best_y);
          current_x = best_x; 
          current_y = best_y;
          remaining_swimmers.erase(best_id); 
        } else {
          // 📌 ΔΙΟΡΘΩΘΗΚΕ: Αν όλοι είναι πιο κοντά στον εχθρό, 
          // βρίσκουμε τον γεωμετρικά κοντινότερο σε εμάς και όχι τον αλφαβητικά πρώτο!
          string nearest_id = "";
          double nearest_dist = -1;
          double nx = 0, ny = 0;

          for(auto const& sp : remaining_swimmers) {
            double d = hypot(sp.second.x() - current_x, sp.second.y() - current_y);
            if(nearest_dist < 0 || d < nearest_dist) {
              nearest_dist = d;
              nearest_id = sp.first;
              nx = sp.second.x();
              ny = sp.second.y();
            }
          }

          if(nearest_id != "") {
            my_path.add_vertex(nx, ny);
            current_x = nx;
            current_y = ny;
            remaining_swimmers.erase(nearest_id);
          } else {
            break;
          }
        }
      }

      // Ενημέρωση του Helm
      if(my_path.size() > 0) {
        string update_str = "points=" + my_path.get_spec(); 
        Notify("RESCUE_UPDATES", update_str); 
        m_first_run_done = true; // 📌 Χρήση της m_ μεταβλητής
      }
      
      m_path_update_needed = false; 
  }

  AppCastingMOOSApp::PostReport();
  return(true);
}

bool GenRescue::OnStartUp()
{
  AppCastingMOOSApp::OnStartUp();
  RegisterVariables();  
  return(true);
}

void GenRescue::RegisterVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  
  Register("NAV_X", 0);
  Register("NAV_Y", 0);
  Register("SWIMMER_ALERT", 0); // [cite: 184, 333]
  Register("FOUND_SWIMMER", 0); // [cite: 349]

  // 📌 Γινόμαστε συνδρομητές στις live θέσεις των δύο πιθανών αντιπάλων
  Register("NAV_X_BEN", 0);
  Register("NAV_Y_BEN", 0);
  Register("NAV_X_ABE", 0);
  Register("NAV_Y_ABE", 0);
}

bool GenRescue::buildReport() 
{
  m_msgs << "============================================" << endl;
  m_msgs << "pGenRescue " << m_host_community << " Status:" << endl;
  m_msgs << "============================================" << endl;
  
  bool nav_ok = (m_nav_x != 0 || m_nav_y != 0);
  m_msgs << "NAV_X/Y Received:        " << (nav_ok ? "true" : "false") << endl;
  m_msgs << "Enemy X/Y Position:      [" << m_enemy_x << ", " << m_enemy_y << "]" << endl;
  m_msgs << endl;
  m_msgs << "Rescue Status              " << endl;
  m_msgs << "------------------------ " << endl;
  m_msgs << "Active Swimmers in list: " << m_swimmers.size() << endl;
  m_msgs << "Rescued Swimmers:        " << m_rescued_swimmers.size() << endl;
  m_msgs << "Path Update Needed:      " << (m_path_update_needed ? "true" : "false") << endl;

  return(true);
}
