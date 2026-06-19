#include <iterator>
#include <cmath>
#include "MBUtils.h"
#include "XYSegList.h" 
#include "GenRescue.h" // Αλλαγή σε GenRescue

using namespace std;

// Constructor
GenRescue::GenRescue()
{
  // Αφαιρέσαμε το m_got_last_point γιατί δεν υπάρχουν πια firstpoint/lastpoint
  m_nav_x = 0;
  m_nav_y = 0;
  m_path_update_needed = false;
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

    // 1. Ενημέρωση της τρέχουσας θέσης του σκάφους
    if(key == "NAV_X") {
      m_nav_x = msg.GetDouble();
    }
    else if(key == "NAV_Y") {
      m_nav_y = msg.GetDouble();
    }
    
    // 2. Λήψη νέου κολυμβητή από τον Διαιτητή (uFldRescueMgr)
    else if(key == "SWIMMER_ALERT" && msg.IsString()) {
      string sval = msg.GetString(); 

      double x = 0;
      double y = 0;
      string id = "";

      // Διαχωρισμός του string με βάση το κόμμα
      vector<string> svector = parseString(sval, ',');
      for(unsigned int i=0; i<svector.size(); i++) {
        string param = biteStringX(svector[i], '=');
        string value = svector[i];
        
        // ΝΕΟ ΚΟΜΜΑΤΙ: Καθαρισμός από κενά (spaces) δεξιά και αριστερά
        param = stripBlankEnds(param);
        value = stripBlankEnds(value);
        
        if(param == "x") x = atof(value.c_str());
        else if(param == "y") y = atof(value.c_str());
        
        // Δέχεται και 'id' και 'name'
        else if(param == "id" || param == "name") id = value; 
      }

      bool is_active = (m_swimmers.find(id) != m_swimmers.end());
      bool is_rescued = (m_rescued_swimmers.find(id) != m_rescued_swimmers.end());

      if (!is_active && !is_rescued) {
          XYPoint new_swimmer(x, y);
          new_swimmer.set_label(id);
          m_swimmers[id] = new_swimmer;
          m_path_update_needed = true; //
      }
    }
    
    // 3. Ειδοποίηση ότι ένας κολυμβητής βρέθηκε/διασώθηκε
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
        
        // 📌 ΔΙΟΡΘΩΘΗΚΕ: Εδώ ξεκινάει με σκέτο "if" και το επόμενο είναι "else if"
        if(param == "id" || param == "name") {
          id = value;
        }
        else if(param == "finder") {
          finder = value;
        }
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

  // --- 1. ΕΛΕΓΧΟΣ ΚΑΙ ΑΠΟΣΤΟΛΗ RESCUE_REQUEST (Τρέχει πάντα) ---
  for(auto const& swimmer_pair : m_swimmers) {
    std::string id = swimmer_pair.first;
    XYPoint point = swimmer_pair.second;
    
    double dist = hypot(point.x() - m_nav_x, point.y() - m_nav_y);

    if(dist <= 5.0) {
      string request = "vname=" + m_host_community + ",name=" + id;
      Notify("RESCUE_REQUEST", request);
    }
  }

  // --- 2. ΕΝΗΜΕΡΩΣΗ ΔΙΑΔΡΟΜΗΣ (Τρέχει ΜΟΝΟ όταν υπάρχει αλλαγή) ---
  static bool first_run_done = false;
  if (m_path_update_needed || (!first_run_done && !m_swimmers.empty())) {
      
      XYSegList my_path; 
      double current_x = m_nav_x;
      double current_y = m_nav_y;

      map<string, XYPoint> remaining_swimmers = m_swimmers;

      // Greedy Αλγόριθμος
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
          break; 
        }
      }

      // Ενημέρωση του Helm
      if(my_path.size() > 0) {
        string update_str = "points=" + my_path.get_spec(); 
        Notify("RESCUE_UPDATES", update_str);
        first_run_done = true; // Κλειδώνουμε ότι στείλαμε την πρώτη διαδρομή!
      }
      
      // Σημαντικό: Κατεβάζουμε το flag για να μην ξανατρέξει στο επόμενο Iterate 
      // μέχρι να έρθει νέο μήνυμα αλλαγής!
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
  
  // Το σκάφος πρέπει να ξέρει τη θέση του
  Register("NAV_X", 0);
  Register("NAV_Y", 0);
  
  // Το σκάφος πρέπει να ακούει τον Διαιτητή (uFldRescueMgr)
  Register("SWIMMER_ALERT", 0); 
  Register("FOUND_SWIMMER", 0); 
}
bool GenRescue::buildReport() 
{
  m_msgs << "============================================" << endl;
  m_msgs << "pGenRescue " << m_host_community << " Status:" << endl;
  m_msgs << "============================================" << endl;
  
  // Έλεγχος αν έχουμε λάβει στίγμα
  bool nav_ok = (m_nav_x != 0 || m_nav_y != 0);
  m_msgs << "NAV_X/Y Received:        " << (nav_ok ? "true" : "false") << endl;
  
  m_msgs << endl;
  m_msgs << "Rescue Status              " << endl;
  m_msgs << "------------------------ " << endl;
  
  // Δείχνουμε πόσοι κολυμβητές υπάρχουν αυτή τη στιγμή
  m_msgs << "Active Swimmers in list: " << m_swimmers.size() << endl;

  // --- ΝΕΕΣ ΠΡΟΣΘΗΚΕΣ ΓΙΑ ΚΑΛΥΤΕΡΟ APPCASTING ---
  // Δείχνουμε πόσους έχουμε ήδη σώσει
  m_msgs << "Rescued Swimmers:        " << m_rescued_swimmers.size() << endl;
  
  // Δείχνουμε αν εκκρεμεί ανανέωση της διαδρομής (θα το βλέπεις να αναβοσβήνει "true" στιγμιαία)
  m_msgs << "Path Update Needed:      " << (m_path_update_needed ? "true" : "false") << endl;
  // ----------------------------------------------

  return(true);
}
