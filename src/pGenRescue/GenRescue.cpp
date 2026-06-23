#include <iterator>
#include <cmath>
#include <algorithm>
#include "MBUtils.h"
#include "XYSegList.h"
#include "NodeRecord.h"
#include "NodeRecordUtils.h"
#include "GenRescue.h"

using namespace std;

// 2-opt improvement για ΑΝΟΙΧΤΗ διαδρομή που ξεκινά στο (x0,y0).
// Αναδιατάσσει τα 'pts' in-place ώστε να μειωθεί το συνολικό μήκος,
// αφαιρώντας τα crossings που αφήνει ο greedy nearest-neighbor.
static void twoOptPath(double x0, double y0, std::vector<XYPoint>& pts)
{
  size_t n = pts.size();
  if(n < 3) return;

  bool improved = true;
  int guard = 0;
  while(improved && guard < 1000) {
    improved = false;
    guard++;
    for(size_t i = 0; i < n - 1; i++) {
      // Σημείο πριν το τμήμα: i==0 → άγκυρα (x0,y0), αλλιώς pts[i-1]
      double ax = (i == 0) ? x0 : pts[i-1].x();
      double ay = (i == 0) ? y0 : pts[i-1].y();
      for(size_t j = i + 1; j < n; j++) {
        double cx_i = pts[i].x(), cy_i = pts[i].y();
        double cx_j = pts[j].x(), cy_j = pts[j].y();

        // Μόνο οι δύο ακμές-σύνορα αλλάζουν με την αντιστροφή του i..j
        double d_before = hypot(cx_i - ax, cy_i - ay);
        double d_after  = hypot(cx_j - ax, cy_j - ay);

        if(j + 1 < n) {
          double nx = pts[j+1].x(), ny = pts[j+1].y();
          d_before += hypot(nx - cx_j, ny - cy_j);
          d_after  += hypot(nx - cx_i, ny - cy_i);
        }

        if(d_after + 1e-9 < d_before) {
          std::reverse(pts.begin() + i, pts.begin() + j + 1);
          improved = true;
        }
      }
    }
  }
}

// Boundary-aware clamp: τραβά το waypoint προς τα μέσα ώστε να απέχει >= margin
// από ΚΑΘΕ edge του region. Έτσι η βάρκα δεν στοχεύει ποτέ μέσα στη halt ζώνη
// του OpRegion. Το pull περιορίζεται σε MAX_PULL ώστε να παραμένει εντός
// rescue_rng (5m) από τον πραγματικό swimmer. Robust σε CW/CCW (ελέγχει centroid).
static XYPoint clampInside(const std::vector<XYPoint>& region, XYPoint p, double margin)
{
  size_t n = region.size();
  if(n < 3) return p;

  double cxg = 0, cyg = 0;
  for(size_t i = 0; i < n; i++) { cxg += region[i].x(); cyg += region[i].y(); }
  cxg /= n; cyg /= n;

  double px = p.x(), py = p.y();
  for(int pass = 0; pass < 4; pass++) {
    for(size_t i = 0; i < n; i++) {
      double ax = region[i].x(),       ay = region[i].y();
      double bx = region[(i+1)%n].x(), by = region[(i+1)%n].y();
      double dx = bx - ax, dy = by - ay;
      double len = hypot(dx, dy);
      if(len < 1e-9) continue;
      double nx = -dy/len, ny = dx/len;
      // Σιγουρέψου ότι η κάθετος δείχνει προς τα μέσα (προς το centroid)
      if((cxg-ax)*nx + (cyg-ay)*ny < 0) { nx = -nx; ny = -ny; }
      double s = (px-ax)*nx + (py-ay)*ny;   // απόσταση «μέσα» από το edge
      if(s < margin) { px += nx*(margin-s); py += ny*(margin-s); }
    }
  }

  // Μην μετακινείς το waypoint πάνω από rescue_rng από τον swimmer
  double mvx = px - p.x(), mvy = py - p.y(), mv = hypot(mvx, mvy);
  const double MAX_PULL = 5.0;
  if(mv > MAX_PULL) { px = p.x() + mvx/mv*MAX_PULL; py = p.y() + mvy/mv*MAX_PULL; }
  return XYPoint(px, py);
}

// Constructor
GenRescue::GenRescue()
{
  m_nav_x = 0;
  m_nav_y = 0;
  m_nav_heading = 0;
  m_nav_speed = 0;
  m_enemy_x = 0;
  m_enemy_y = 0;
  m_enemy_heading = 0;
  m_enemy_speed = 0;
  m_path_update_needed = false;
  m_first_run_done = false;
  m_returned = false;
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
    else if(key == "NAV_HEADING") {
      m_nav_heading = msg.GetDouble();
    }
    else if(key == "NAV_SPEED") {
      m_nav_speed = msg.GetDouble();
    }
    
    // Θέση + heading + speed αντιπάλου μέσω NODE_REPORT (Lab 12 §5.2 — standard channel)
    else if(key == "NODE_REPORT" && msg.IsString()) {
      NodeRecord rec = string2NodeRecord(msg.GetString());
      if(rec.getName() != m_host_community) {   // αγνόησε το δικό μας report
        m_enemy_x       = rec.getX();
        m_enemy_y       = rec.getY();
        m_enemy_heading = rec.getHeading();
        m_enemy_speed   = rec.getSpeed();
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
          // Αν είχαμε γυρίσει σπίτι και έρθει νέος swimmer, ξανα-ενεργοποιήσου
          if(m_returned) { Notify("RETURN", "false"); m_returned = false; }
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
      m_swimmer_approach_times.erase(id);   // καθάρισε τον give-up timer

      m_rescued_swimmers.insert(id);
    }

    // 4. Live όριο επιχείρησης — ΑΝΤΙΚΑΘΙΣΤΑ το hardcoded region.
    //    Format: "pts={x,y:x,y:...},edge_color=...". Παίρνουμε μόνο το pts={...}.
    else if(key == "RESCUE_REGION" && msg.IsString()) {
      string sval = msg.GetString();

      string pts_str = "";
      size_t p1 = sval.find("pts={");
      if(p1 != string::npos) {
        size_t p2 = sval.find("}", p1);
        if(p2 != string::npos)
          pts_str = sval.substr(p1 + 5, p2 - (p1 + 5));
      }

      if(pts_str != "") {
        vector<string> verts = parseString(pts_str, ':');
        if(verts.size() >= 3) {        // έγκυρο πολύγωνο πριν αντικαταστήσουμε
          m_region.clear();
          for(unsigned int i = 0; i < verts.size(); i++) {
            string vx = biteStringX(verts[i], ',');
            string vy = verts[i];
            m_region.push_back(XYPoint(atof(vx.c_str()), atof(vy.c_str())));
          }
        }
      }
    }
  }
  return(true);
}

bool GenRescue::Iterate()
{
  AppCastingMOOSApp::Iterate();

  if(m_swimmers.empty()) {
    // Assignment 1 (Lab 12): όταν αδειάσει η λίστα αφού έχουμε ήδη δουλέψει
    // (π.χ. ο τελευταίος swimmer διασώθηκε/αφαιρέθηκε), γύρνα σπίτι.
    if(m_first_run_done && !m_returned) {
      Notify("RETURN", "true");
      m_returned = true;
    }
    AppCastingMOOSApp::PostReport();
    return(true);
  }

  // --- 1. RESCUE_REQUEST + Give-Up Timer (safety-net για μη-σώσιμους swimmers) ---
  std::vector<std::string> giveup_ids;
  for(auto const& swimmer_pair : m_swimmers) {
    std::string id = swimmer_pair.first;
    XYPoint point = swimmer_pair.second;

    double dist = hypot(point.x() - m_nav_x, point.y() - m_nav_y);

    if(dist <= 6.5) {
      string request = "vname=" + m_host_community + ",name=" + id;
      Notify("RESCUE_REQUEST", request); // [cite: 185]
    }

    // Give-Up Timer: αν είμαστε <7m για >12s χωρίς FOUND_SWIMMER → μη-σώσιμος
    // (π.χ. unreg swimmer που θέλει scouting). Τον εγκαταλείπουμε για να μην κολλάμε.
    if(dist < 7.0) {
      if(m_swimmer_approach_times.find(id) == m_swimmer_approach_times.end())
        m_swimmer_approach_times[id] = m_curr_time;            // πρώτη προσέγγιση
      else if((m_curr_time - m_swimmer_approach_times[id]) > 12.0)
        giveup_ids.push_back(id);
    }
    else {
      m_swimmer_approach_times.erase(id);  // απομακρυνθήκαμε → reset (απόφυγε false give-up)
    }
  }

  // Εφαρμογή give-up εκτός του loop (ασφαλές erase από το m_swimmers)
  for(unsigned int i = 0; i < giveup_ids.size(); i++) {
    string gid = giveup_ids[i];
    m_swimmers.erase(gid);
    m_swimmer_approach_times.erase(gid);
    m_path_update_needed = true;
    reportRunWarning("Giving up on un-rescuable swimmer: " + gid);
  }

  // --- 2. ΕΝΗΜΕΡΩΣΗ ΔΙΑΔΡΟΜΗΣ (Με Διορθώσεις Claude Code) ---
  // Αντικαταστήσαμε το static με το μέλος m_first_run_done
  if (m_path_update_needed || (!m_first_run_done && !m_swimmers.empty())) {
      
      vector<XYPoint> ordered_pts;
      double current_x = m_nav_x;
      double current_y = m_nav_y;

      map<string, XYPoint> remaining_swimmers = m_swimmers;

      bool enemy_known = (m_enemy_x != 0 || m_enemy_y != 0);
      double cluster_radius = 40.0;
      double current_heading = m_nav_heading;

      while(!remaining_swimmers.empty()) {
        string best_id = "";
        double best_score = -1;
        double best_x = 0, best_y = 0;

        // Proximity Override: αν υπάρχει swimmer <=12m, παρέκαμψε το scoring
        double urgent_dist = -1;
        double r_urgent = 12.0;
        for(auto const& sp : remaining_swimmers) {
          double px = sp.second.x();
          double py = sp.second.y();
          double d  = hypot(px - current_x, py - current_y);
          if(d > r_urgent) continue;
          double d_enemy = hypot(px - m_enemy_x, py - m_enemy_y);
          if(enemy_known && d_enemy < d && remaining_swimmers.size() > 1) continue;
          if(urgent_dist < 0 || d < urgent_dist) {
            urgent_dist = d;
            best_id = sp.first;
            best_x  = px;
            best_y  = py;
          }
        }
        if(best_id != "") {
          double next_bearing = atan2(best_x - current_x, best_y - current_y) * 180.0 / M_PI;
          if(next_bearing < 0) next_bearing += 360.0;
          current_heading = next_bearing;
          ordered_pts.push_back(XYPoint(best_x, best_y));
          current_x = best_x;
          current_y = best_y;
          remaining_swimmers.erase(best_id);
          continue;
        }

        for(auto const& swimmer_pair : remaining_swimmers) {
          std::string id = swimmer_pair.first;
          XYPoint point = swimmer_pair.second;
          double px = point.x();
          double py = point.y();

          double dist = hypot(px - current_x, py - current_y);
          double dist_to_enemy = hypot(px - m_enemy_x, py - m_enemy_y);

          // Aggressive Contest (Λεβιές #1): equal-speed κούρσα → η γεωμετρία μοιράζει
          // το πεδίο μόνη της. Παραχωρούμε ΜΟΝΟ σαφώς χαμένες υποθέσεις: ο εχθρός είναι
          // σχεδόν πάνω στον swimmer (<10m) ΚΑΙ αρκετά πιο κοντά από εμάς (<0.66·dist).
          // Ήπιο tie-breaker (+60), ΟΧΙ veto — διεκδικούμε το δίκαιο μερίδιό μας.
          double ttt_penalty = 0.0;
          if (enemy_known && remaining_swimmers.size() > 1) {
              // (a) Proximity concede: ο αντίπαλος σχεδόν πάνω στον swimmer
              if (dist_to_enemy < 10.0 && dist_to_enemy < dist * 0.66)
                  ttt_penalty = 60.0;

              // (b) Heading concede (Lab 12 §5.2 strategy 2): ο αντίπαλος ΚΙΝΕΙΤΑΙ
              // και κατευθύνεται προς τον swimmer (relative bearing ~0). Το m_enemy_speed>0.3
              // αποτρέπει concede σε κολλημένο/νεκρό αντίπαλο.
              if (m_enemy_speed > 0.3 && dist_to_enemy < 40.0 && dist_to_enemy < dist) {
                  double brg = atan2(px - m_enemy_x, py - m_enemy_y) * 180.0/M_PI;
                  if(brg < 0) brg += 360.0;
                  double dh = fabs(brg - m_enemy_heading);
                  if(dh > 180.0) dh = 360.0 - dh;
                  if(dh < 25.0)                 // μέσα στον «κώνο» πορείας του αντιπάλου
                      ttt_penalty += 80.0;
              }
          }

          // Count neighbors within cluster_radius
          int neighbors = 0;
          for(auto const& other : remaining_swimmers) {
            if(other.first == id) continue;
            double d = hypot(other.second.x() - px, other.second.y() - py);
            if(d <= cluster_radius) neighbors++;
          }

          // Time-to-Target (TTT) με Γραμμική Ορμο-Ποινή
          // time_turn κλιμακώνεται με ταχύτητα × κανονικοποιημένη γωνία
          // ώστε υψηλή ταχύτητα + μεγάλη στροφή να τιμωρείται σωστά (BlueBoat sideslip)
          double bearing = atan2(px - current_x, py - current_y) * 180.0 / M_PI;
          if(bearing < 0) bearing += 360.0;
          double heading_error = fabs(current_heading - bearing);
          if(heading_error > 180.0) heading_error = 360.0 - heading_error;
          double speed         = (m_nav_speed < 0.2) ? 1.0 : m_nav_speed;
          double time_turn     = (heading_error / 30.0) * (1.0 + speed * (heading_error / 180.0));
          double time_straight = dist / speed;
          double ttt           = time_turn + time_straight + ttt_penalty;

          // Virtual Obstacle Filter: penalize paths that pass within 12m of any buoy
          for(unsigned int oi = 0; oi < m_obstacles.size(); oi++) {
            double ax = current_x, ay = current_y;
            double abx = px - ax,  aby = py - ay;
            double ab2 = abx*abx + aby*aby;
            double t = 0.0;
            if(ab2 > 1e-9) {
              t = ((m_obstacles[oi].x()-ax)*abx + (m_obstacles[oi].y()-ay)*aby) / ab2;
              if(t < 0.0) t = 0.0;
              if(t > 1.0) t = 1.0;
            }
            double cx = ax + t*abx, cy = ay + t*aby;
            if(hypot(m_obstacles[oi].x()-cx, m_obstacles[oi].y()-cy) < 12.0)
              ttt += 500.0;
          }

          double score = ((neighbors + 1) * (neighbors + 1)) / (ttt + 1e-6);

          if(score > best_score) {
            best_score = score;
            best_id = id;
            best_x = px;
            best_y = py;
          }
        }

        if(best_id != "") {
          double next_bearing = atan2(best_x - current_x, best_y - current_y) * 180.0 / M_PI;
          if(next_bearing < 0) next_bearing += 360.0;
          current_heading = next_bearing;
          ordered_pts.push_back(XYPoint(best_x, best_y));
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
            ordered_pts.push_back(XYPoint(nx, ny));
            current_x = nx;
            current_y = ny;
            remaining_swimmers.erase(nearest_id);
          } else {
            break;
          }
        }
      }

      // Λεβιές #2: 2-opt pass πάνω στη greedy σειρά — αφαιρεί crossings/backtrack,
      // ξεκινώντας από την τρέχουσα θέση της βάρκας ως άγκυρα.
      twoOptPath(m_nav_x, m_nav_y, ordered_pts);

      // Boundary-aware: τράβα κάθε waypoint >=5m μέσα από το όριο ώστε η βάρκα
      // να μην στοχεύει ποτέ στη halt ζώνη του OpRegion (halt_dist=4.5m).
      XYSegList my_path;
      for(unsigned int k = 0; k < ordered_pts.size(); k++) {
        XYPoint safe = clampInside(m_region, ordered_pts[k], 5.0);
        my_path.add_vertex(safe.x(), safe.y());
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

  STRING_LIST sParams;
  m_MissionReader.EnableVerbatimQuoting(false);
  if(m_MissionReader.GetConfiguration(GetAppName(), sParams)) {
    STRING_LIST::iterator p;
    for(p = sParams.begin(); p != sParams.end(); p++) {
      string line  = *p;
      string param = tolower(biteStringX(line, '='));
      string value = line;
      if(param == "obstacle") {
        double ox = 0, oy = 0;
        vector<string> parts = parseString(value, ',');
        for(unsigned int i = 0; i < parts.size(); i++) {
          string kv = parts[i];
          string k  = biteStringX(kv, '=');
          k = stripBlankEnds(k);
          kv = stripBlankEnds(kv);
          if(k == "x") ox = atof(kv.c_str());
          else if(k == "y") oy = atof(kv.c_str());
        }
        m_obstacles.push_back(XYPoint(ox, oy));
      }
    }
  }

  // Αν φορτώθηκε έστω ένα obstacle από το config, η πηγή είναι το config
  if(!m_obstacles.empty())
    m_obstacle_source = "config";

  // Fallback: known buoy centers if none parsed from config
  if(m_obstacles.empty()) {
    m_obstacles.push_back(XYPoint(-35.0,  -6.0));
    m_obstacles.push_back(XYPoint(-62.5, -17.9));
    m_obstacles.push_back(XYPoint(-95.0, -28.0));
    m_obstacle_source = "FALLBACK (hardcoded!)";
  }

  // Το region ΔΕΝ είναι hardcoded πια — έρχεται live από το RESCUE_REGION
  // (βλ. OnNewMail). Μέχρι να ληφθεί, m_region άδειο → clampInside επιστρέφει
  // raw waypoints (safety gate), ώστε να ΜΗΝ ξανασυμβεί mis-projection στο νερό.

  RegisterVariables();
  return(true);
}

void GenRescue::RegisterVariables()
{
  AppCastingMOOSApp::RegisterVariables();
  
  Register("NAV_X", 0);
  Register("NAV_Y", 0);
  Register("NAV_HEADING", 0);
  Register("NAV_SPEED", 0);
  Register("SWIMMER_ALERT", 0); // [cite: 184, 333]
  Register("FOUND_SWIMMER", 0); // [cite: 349]
  Register("RESCUE_REGION", 0); // live όριο για boundary-aware clamp

  // 📌 Θέση/heading/speed αντιπάλου μέσω NODE_REPORT (Lab 12 §5.2)
  Register("NODE_REPORT", 0);
}

bool GenRescue::buildReport() 
{
  m_msgs << "============================================" << endl;
  m_msgs << "pGenRescue " << m_host_community << " Status:" << endl;
  m_msgs << "============================================" << endl;
  
  bool nav_ok = (m_nav_x != 0 || m_nav_y != 0);
  m_msgs << "NAV_X/Y Received:        " << (nav_ok ? "true" : "false") << endl;
  m_msgs << "Obstacles Loaded:        " << m_obstacles.size()
         << "  [source: " << m_obstacle_source << "]" << endl;
  m_msgs << "Region Verts (live):     " << m_region.size()
         << (m_region.empty() ? "  [clamp OFF - raw waypoints]" : "  [clamp ON]") << endl;
  m_msgs << "Enemy X/Y Position:      [" << m_enemy_x << ", " << m_enemy_y << "]" << endl;
  m_msgs << endl;
  m_msgs << "Rescue Status              " << endl;
  m_msgs << "------------------------ " << endl;
  m_msgs << "Active Swimmers in list: " << m_swimmers.size() << endl;
  m_msgs << "Rescued Swimmers:        " << m_rescued_swimmers.size() << endl;
  m_msgs << "Path Update Needed:      " << (m_path_update_needed ? "true" : "false") << endl;

  return(true);
}