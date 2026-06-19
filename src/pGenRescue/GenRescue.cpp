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
}

// Destructor
GenRescue::~GenRescue()
{
}

bool GenRescue::OnNewMail(MOOSMSG_LIST &NewMail)
{
  // Απαραίτητο για να λειτουργεί σωστά το AppCasting
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
      string sval = msg.GetString(); // Μορφή: "x=23, y=54, id=04"

      double x = 0;
      double y = 0;
      string id = "";

      // Διαχωρισμός του string με βάση το κόμμα
      vector<string> svector = parseString(sval, ',');
      for(unsigned int i=0; i<svector.size(); i++) {
        string param = biteStringX(svector[i], '=');
        string value = svector[i];
        
        if(param == "x") x = atof(value.c_str());
        else if(param == "y") y = atof(value.c_str());
        else if(param == "id") id = value;
      }

      // Ελέγχουμε αν υπάρχει ήδη αυτό το id. Αν όχι, το προσθέτουμε στο map.
      if (m_swimmers.find(id) == m_swimmers.end()) {
          XYPoint new_swimmer(x, y);
          new_swimmer.set_label(id);
          m_swimmers[id] = new_swimmer;
      }
    }
    
    // 3. Ειδοποίηση ότι ένας κολυμβητής βρέθηκε/διασώθηκε
    else if(key == "FOUND_SWIMMER" && msg.IsString()) {
      string sval = msg.GetString(); // Μορφή: "id=01, finder=abe"
      string id = "";
      string finder = "";

      vector<string> svector = parseString(sval, ',');
      for(unsigned int i=0; i<svector.size(); i++) {
        string param = biteStringX(svector[i], '=');
        string value = svector[i];
        
        if(param == "id") id = value;
        else if(param == "finder") finder = value;
      }

      // Αν ο κολυμβητής υπάρχει στη λίστα μας, τον διαγράφουμε για να μην πάμε άδικα
      if (m_swimmers.find(id) != m_swimmers.end()) {
          m_swimmers.erase(id);
      }
    }
  }
  return(true);
}

bool GenRescue::Iterate()
{
  AppCastingMOOSApp::Iterate();

  // Αν δεν έχουμε κολυμβητές στη λίστα, δεν κάνουμε τίποτα ακόμα
  if(m_swimmers.empty()) {
    AppCastingMOOSApp::PostReport();
    return(true);
  }

  // --- ΝΕΟ ΚΟΜΜΑΤΙ: ΕΛΕΓΧΟΣ ΚΑΙ ΑΠΟΣΤΟΛΗ RESCUE_REQUEST ---
  
  for(auto const& swimmer_pair : m_swimmers) {
    std::string id = swimmer_pair.first;
    XYPoint point = swimmer_pair.second;
    // Υπολογισμός απόστασης του σκάφους από τον κολυμβητή
    double dist = hypot(point.x() - m_nav_x, point.y() - m_nav_y);

    // Αν πλησιάσαμε στα 5 μέτρα ή πιο κοντά (rescue_rng_max)
    if(dist <= 5.0) {
      // Μορφοποίηση μηνύματος, π.χ. "vname=abe,name=04"
      string request = "vname=" + m_host_community + ",name=" + id;
      Notify("RESCUE_REQUEST", request);
    }
  }
  // --------------------------------------------------------


  XYSegList my_path; 
  double current_x = m_nav_x;
  double current_y = m_nav_y;

  // Φτιάχνουμε ένα προσωρινό αντίγραφο των κολυμβητών για να υπολογίσουμε τη διαδρομή
  // (δεν πειράζουμε το κανονικό m_swimmers για να μην τους "ξεχάσουμε"!)
  map<string, XYPoint> remaining_swimmers = m_swimmers;

  // Ο Greedy Αλγόριθμός σου, προσαρμοσμένος για το map:
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
      remaining_swimmers.erase(best_id); // Αφαιρούμε από το ΠΡΟΣΩΡΙΝΟ map
    } else {
      break; 
    }
  } // Τέλος του while

  // Ενημέρωση του Helm
  if(my_path.size() > 0) {
    // Μορφοποίηση της διαδρομής
    string update_str = "points=" + my_path.get_spec(); 
    
    // Στέλνουμε τη νέα διαδρομή στη συμπεριφορά (Τσέκαρε αν λέγεται RESCUE_UPDATES στο meta_vehicle.bhv σου)
    Notify("RESCUE_UPDATES", update_str);
  }

  AppCastingMOOSApp::PostReport();
  return(true);
}

bool GenRescue::OnConnectToServer()
{
  RegisterVariables();
  // Το VEHICLE_READY δεν χρειάζεται πια για τον Διαιτητή, 
  // αλλά δεν κάνει κακό να το αφήσεις αν το χρησιμοποιείς κάπου αλλού.
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
  
  // Έλεγχος αν έχουμε λάβει στίγμα (αφού αρχικοποιούνται στο 0)
  bool nav_ok = (m_nav_x != 0 || m_nav_y != 0);
  m_msgs << "NAV_X/Y Received:        " << (nav_ok ? "true" : "false") << endl;
  
  m_msgs << endl;
  m_msgs << "Rescue Status              " << endl;
  m_msgs << "------------------------ " << endl;
  
  // Δείχνουμε πόσοι κολυμβητές υπάρχουν αυτή τη στιγμή στη λίστα/map μας
  // Εφόσον τους διαγράφουμε μόλις διασωθούν, αυτός ο αριθμός θα μειώνεται!
  m_msgs << "Active Swimmers in list: " << m_swimmers.size() << endl;

  return(true);
}
