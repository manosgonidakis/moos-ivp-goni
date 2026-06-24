/*****************************************************************/
/*    NAME: M.Benjamin                                           */
/*    ORGN: Dept of Mechanical Eng / CSAIL, MIT Cambridge MA     */
/*    FILE: BHV_Scout.cpp                                        */
/*    DATE: April 30th 2022                                      */
/*****************************************************************/

#include <cstdlib>
#include <math.h>
#include "BHV_Scout.h"
#include "MBUtils.h"
#include "AngleUtils.h"
#include "BuildUtils.h"
#include "GeomUtils.h"
#include "ZAIC_PEAK.h"
#include "OF_Coupler.h"
#include "XYFormatUtilsPoly.h"

using namespace std;

//-----------------------------------------------------------
// Constructor()

BHV_Scout::BHV_Scout(IvPDomain gdomain) : 
  IvPBehavior(gdomain)
{
  IvPBehavior::setParam("name", "scout");
 
  // Default values for behavior state variables
  m_osx  = 0;
  m_osy  = 0;
  m_curr_time = 0;
  m_ptx = 0;
  m_pty = 0;

  // All distances are in meters, all speed in meters per second
  // Default values for configuration parameters
  m_desired_speed  = 1;
  m_capture_radius = 10;
  m_lane_width     = 15;   // απόσταση γραμμών σάρωσης (≈ 2× sensor radius)
  m_avoid_radius   = 30;   // hint του Lab: μη σκαουτάρεις <30m από rescue
  m_known_radius   = 0;    // OFF by default: 30m×πολλούς swimmers απέκλειε ΟΛΟ το grid
                           // (ταλάντωση/μη-κάλυψη). Το rescue-avoidance αρκεί δυναμικά.
                           // Βάλε >0 στο config μόνο αν θες ελαφρύ known-swimmer skip.
  m_inset          = 10;   // τράβα τα endpoints 10m μέσα → no overshoot εκτός ορίου
  m_obstacle_radius = 16;  // μη στοχεύεις waypoint <16m από buoy (αποφυγή stuck)

  m_pt_set = false;

  // Lawnmower state
  m_lawn_idx   = 0;
  m_lawn_built = false;

  // Stuck-detection
  m_stuck_x    = 0;
  m_stuck_y    = 0;
  m_stuck_time = 0;
  m_stuck_init = false;

  // Rescue tracking
  m_rescue_x = 0;
  m_rescue_y = 0;
  m_rescue_known = false;
  m_rescue_utc   = 0;

  addInfoVars("NAV_X, NAV_Y");
  addInfoVars("RESCUE_REGION");
  // Τα παρακάτω μπορεί να ΛΕΙΠΟΥΝ στην αρχή (πριν εμφανιστούν) → no_warning
  // ώστε το getBufferStringVal να μη γεμίζει BHV_WARNING στο helm.
  addInfoVars("SCOUTED_SWIMMER", "no_warning");
  addInfoVars("NODE_REPORT", "no_warning");   // θέση teammate (rescue) για avoidance
  addInfoVars("SWIMMER_ALERT", "no_warning"); // registered swimmers (ο rescue θα τους πάρει)
}

//---------------------------------------------------------------
// Procedure: setParam() - handle behavior configuration parameters

bool BHV_Scout::setParam(string param, string val) 
{
  // Convert the parameter to lower case for more general matching
  param = tolower(param);
  
  bool handled = true;
  if(param == "capture_radius")
    handled = setPosDoubleOnString(m_capture_radius, val);
  else if(param == "desired_speed")
    handled = setPosDoubleOnString(m_desired_speed, val);
  else if(param == "tmate")
    handled = setNonWhiteVarOnString(m_tmate, val);
  else if(param == "lane_width")
    handled = setPosDoubleOnString(m_lane_width, val);
  else if(param == "avoid_radius")
    handled = setPosDoubleOnString(m_avoid_radius, val);
  else if(param == "known_radius")
    handled = setPosDoubleOnString(m_known_radius, val);
  else if(param == "inset")
    handled = setPosDoubleOnString(m_inset, val);
  else if(param == "obstacle_radius")
    handled = setPosDoubleOnString(m_obstacle_radius, val);
  else if(param == "obstacle") {
    // Format: "x=..,y=.." (όπως στο pGenRescue). Συσσώρευσε buoy centers.
    double ox = 0, oy = 0; bool hx = false, hy = false;
    vector<string> parts = parseString(val, ',');
    for(unsigned int i = 0; i < parts.size(); i++) {
      string k = tolower(stripBlankEnds(biteStringX(parts[i], '=')));
      string v = stripBlankEnds(parts[i]);
      if(k == "x") { ox = atof(v.c_str()); hx = true; }
      else if(k == "y") { oy = atof(v.c_str()); hy = true; }
    }
    if(hx && hy) m_obstacles.push_back(XYPoint(ox, oy));
    else handled = false;
  }
  else
    handled = false;

  srand(time(NULL));

  return(handled);
}

//-----------------------------------------------------------
// Procedure: onEveryState()

void BHV_Scout::onEveryState(string str) 
{
  if(!getBufferVarUpdated("SCOUTED_SWIMMER"))
    return;

  string report = getBufferStringVal("SCOUTED_SWIMMER");
  if(report == "")
    return;

  if(m_tmate == "") {
    postWMessage("Mandatory Teammate name is null");
    return;
  }
  postOffboardMessage(m_tmate, "SWIMMER_ALERT", report);
}

//-----------------------------------------------------------
// Procedure: onIdleState()

void BHV_Scout::onIdleState() 
{
  m_curr_time = getBufferCurrTime();
}

//-----------------------------------------------------------
// Procedure: onRunState()

IvPFunction *BHV_Scout::onRunState() 
{
  // Part 1: Get vehicle position from InfoBuffer and post a 
  // warning if problem is encountered
  bool ok1, ok2;
  m_osx = getBufferDoubleVal("NAV_X", ok1);
  m_osy = getBufferDoubleVal("NAV_Y", ok2);
  m_curr_time = getBufferCurrTime();
  if(!ok1 || !ok2) {
    postWMessage("No ownship X/Y info in info_buffer.");
    return(0);
  }

  // Ενημέρωσε θέση rescue teammate + λίστα γνωστών swimmers (για avoidance)
  updateRescuePos();
  updateKnownSwimmers();

  // Δυναμικό avoidance: αν ο τρέχων στόχος έπεσε κοντά στον rescue, σε γνωστό
  // swimmer (θα τον πάρει ο rescue), ή σε buoy → πέτα τον. Διάλεξε επόμενο.
  if(m_pt_set && (rescueNear(m_ptx, m_pty) || knownSwimmerNear(m_ptx, m_pty)
                  || obstacleNear(m_ptx, m_pty))) {
    m_pt_set = false;
    postViewPoint(false);
  }

  // STUCK-DETECTION (anti-deadlock): αν για >12s δεν προχωράμε >5m ενώ έχουμε
  // στόχο (π.χ. κολλήσαμε πίσω από buoy), παράτα τον στόχο → επόμενο waypoint.
  if(m_pt_set) {
    if(!m_stuck_init) {
      m_stuck_x = m_osx; m_stuck_y = m_osy;
      m_stuck_time = m_curr_time; m_stuck_init = true;
    }
    else if((m_curr_time - m_stuck_time) >= 12.0) {
      double net = hypot(m_osx - m_stuck_x, m_osy - m_stuck_y);
      m_stuck_x = m_osx; m_stuck_y = m_osy; m_stuck_time = m_curr_time;
      if(net < 5.0) {
        m_pt_set = false;          // stuck → άλλαξε waypoint
        postViewPoint(false);
        postEventMessage("Scout stuck (net=" + doubleToStringX(net,1) + ") → skip waypoint");
      }
    }
  }
  else
    m_stuck_init = false;

  // Part 2: Determine if the vehicle has reached the destination
  // point and if so, declare completion.
  updateScoutPoint();
  double dist = hypot((m_ptx-m_osx), (m_pty-m_osy));
  //postEventMessage("Dist=" + doubleToStringX(dist,1));
  if(dist <= m_capture_radius) {
    m_pt_set = false;
    postViewPoint(false);
    return(0);
  }

  // Part 3: Post the waypoint as a string for consumption by 
  // a viewer application.
  postViewPoint(true);

  // Part 4: Build the IvP function 
  IvPFunction *ipf = buildFunction();
  if(ipf == 0) 
    postWMessage("Problem Creating the IvP Function");
  
  return(ipf);
}

//-----------------------------------------------------------
// Procedure: updateScoutPoint()

void BHV_Scout::updateScoutPoint()
{
  if(m_pt_set)
    return;

  // (1) Σιγουρέψου ότι έχουμε region + χτισμένο lawnmower grid
  if(!m_lawn_built) {
    string region_str = getBufferStringVal("RESCUE_REGION");
    if(region_str == "") {
      postWMessage("Unknown RESCUE_REGION");
      return;
    }
    postRetractWMessage("Unknown RESCUE_REGION");

    XYPolygon region = string2Poly(region_str);
    if(!region.is_convex()) {
      postWMessage("Badly formed RESCUE_REGION");
      return;
    }
    m_rescue_region = region;
    generateLawnmower();
    m_lawn_built = !m_lawn_pts.empty();
  }

  // (2) Fallback: αν για κάποιο λόγο δεν φτιάχτηκε grid, γύρνα σε random point
  if(m_lawn_pts.empty()) {
    double ptx = 0, pty = 0;
    if(randPointInPoly(m_rescue_region, ptx, pty)) {
      m_ptx = ptx; m_pty = pty; m_pt_set = true; m_stuck_init = false;
      postEventMessage("Fallback rand pt: " + doubleToStringX(ptx) + "," + doubleToStringX(pty));
    }
    return;
  }

  // (3) Διάλεξε το επόμενο waypoint του grid, παρακάμπτοντας όσα είναι κοντά
  //     είτε στο rescue είτε σε γνωστό swimmer (registered/scouted) — εκεί ο
  //     rescue θα πάει ούτως ή άλλως. Ο scout κυνηγά μόνο τις ΤΥΦΛΕΣ ζώνες.
  for(size_t tries = 0; tries < m_lawn_pts.size(); tries++) {
    XYPoint cand = m_lawn_pts[m_lawn_idx];
    m_lawn_idx = (m_lawn_idx + 1) % m_lawn_pts.size();
    if(rescueNear(cand.x(), cand.y()) || knownSwimmerNear(cand.x(), cand.y())
       || obstacleNear(cand.x(), cand.y()))
      continue;
    m_ptx = cand.x();
    m_pty = cand.y();
    m_pt_set = true;
    m_stuck_init = false;   // νέος στόχος → reset παράθυρο stuck-detection
    postEventMessage("Lawn pt " + uintToString(m_lawn_idx) + ": "
                     + doubleToStringX(m_ptx,1) + "," + doubleToStringX(m_pty,1));
    return;
  }

  // (4) Όλα κλειδωμένα/κοντά → πάρε το επόμενο ούτως ή άλλως (μη κολλήσεις)
  XYPoint cand = m_lawn_pts[m_lawn_idx];
  m_lawn_idx = (m_lawn_idx + 1) % m_lawn_pts.size();
  m_ptx = cand.x();
  m_pty = cand.y();
  m_pt_set = true;
  m_stuck_init = false;
}

//-----------------------------------------------------------
// Procedure: generateLawnmower()
//   Φτιάχνει boustrophedon (zig-zag) waypoints που καλύπτουν ΟΛΟ το region
//   σε παράλληλες γραμμές απόστασης m_lane_width. Οι γραμμές τραβιούνται κατά
//   τη ΜΕΓΑΛΥΤΕΡΗ διάσταση του bounding box (λιγότερες στροφές). Κρατά μόνο τα
//   άκρα κάθε γραμμής που πέφτουν ΜΕΣΑ στο πολύγωνο.

void BHV_Scout::generateLawnmower()
{
  m_lawn_pts.clear();
  m_lawn_idx = 0;

  // Fallback obstacles (σταθερά buoys) αν δεν δόθηκαν στο config — ίδια με pGenRescue
  if(m_obstacles.empty()) {
    m_obstacles.push_back(XYPoint(-35.0,  -6.0));
    m_obstacles.push_back(XYPoint(-62.5, -17.9));
    m_obstacles.push_back(XYPoint(-95.0, -28.0));
  }

  double xmin = m_rescue_region.get_min_x();
  double xmax = m_rescue_region.get_max_x();
  double ymin = m_rescue_region.get_min_y();
  double ymax = m_rescue_region.get_max_y();

  double lane = (m_lane_width >= 1.0) ? m_lane_width : 15.0;
  double step = 2.0;          // ανάλυση εύρεσης in-poly εύρους κατά μήκος της γραμμής
  double ins  = m_inset;      // πόσο μέσα από το όριο τραβάμε endpoints/lanes
  bool horiz = (xmax - xmin) >= (ymax - ymin);  // σάρωση κατά τη μεγάλη διάσταση
  int k = 0;

  if(horiz) {
    // Οριζόντιες γραμμές: σταθερό y (inset από top/bottom), σαρώνουμε x
    for(double y = ymin + ins; y <= ymax - ins; y += lane, k++) {
      double xa = 0, xb = 0; bool found = false;
      for(double x = xmin; x <= xmax; x += step) {
        if(m_rescue_region.contains(x, y)) {
          if(!found) { xa = x; found = true; }
          xb = x;
        }
      }
      if(!found) continue;
      // INSET κατά μήκος της γραμμής → no overshoot εκτός ορίου στις στροφές
      double mid = (xa + xb) / 2.0;
      xa += ins; xb -= ins;
      if(xa > xb) { xa = mid; xb = mid; }   // πολύ κοντή γραμμή → ένα σημείο
      if(k % 2 == 0) {
        m_lawn_pts.push_back(XYPoint(xa, y));
        m_lawn_pts.push_back(XYPoint(xb, y));
      } else {
        m_lawn_pts.push_back(XYPoint(xb, y));
        m_lawn_pts.push_back(XYPoint(xa, y));
      }
    }
  }
  else {
    // Κάθετες γραμμές: σταθερό x (inset από left/right), σαρώνουμε y
    for(double x = xmin + ins; x <= xmax - ins; x += lane, k++) {
      double ya = 0, yb = 0; bool found = false;
      for(double y = ymin; y <= ymax; y += step) {
        if(m_rescue_region.contains(x, y)) {
          if(!found) { ya = y; found = true; }
          yb = y;
        }
      }
      if(!found) continue;
      double mid = (ya + yb) / 2.0;
      ya += ins; yb -= ins;
      if(ya > yb) { ya = mid; yb = mid; }
      if(k % 2 == 0) {
        m_lawn_pts.push_back(XYPoint(x, ya));
        m_lawn_pts.push_back(XYPoint(x, yb));
      } else {
        m_lawn_pts.push_back(XYPoint(x, yb));
        m_lawn_pts.push_back(XYPoint(x, ya));
      }
    }
  }

  postEventMessage("Lawnmower built: " + uintToString(m_lawn_pts.size())
                   + " waypoints, inset=" + doubleToStringX(ins,0) + "m, obs="
                   + uintToString(m_obstacles.size()));
}

//-----------------------------------------------------------
// Procedure: updateRescuePos()
//   Διαβάζει το τελευταίο NODE_REPORT και, αν αφορά τον rescue teammate
//   (κατ' όνομα m_tmate, ή οποιοδήποτε άλλο όχημα αν δεν έχει δοθεί tmate),
//   αποθηκεύει τη θέση του + timestamp για το avoidance.

void BHV_Scout::updateRescuePos()
{
  bool ok;
  string report = getBufferStringVal("NODE_REPORT", ok);
  if(!ok || report == "")
    return;

  string name; double rx = 0, ry = 0; bool hasx = false, hasy = false;
  vector<string> svec = parseString(report, ',');
  for(unsigned int i = 0; i < svec.size(); i++) {
    string key = tolower(stripBlankEnds(biteStringX(svec[i], '=')));
    string val = stripBlankEnds(svec[i]);
    if(key == "name")      name = tolower(val);
    else if(key == "x")  { rx = atof(val.c_str()); hasx = true; }
    else if(key == "y")  { ry = atof(val.c_str()); hasy = true; }
  }
  if(!hasx || !hasy)
    return;

  string tm = tolower(m_tmate);
  bool match = (tm != "") ? (name == tm) : (name != tolower(m_us_name));
  if(match) {
    m_rescue_x = rx;
    m_rescue_y = ry;
    m_rescue_known = true;
    m_rescue_utc = m_curr_time;
  }
}

//-----------------------------------------------------------
// Procedure: rescueNear()
//   true αν το (x,y) είναι <m_avoid_radius από την (πρόσφατη) θέση του rescue.

bool BHV_Scout::rescueNear(double x, double y) const
{
  if(!m_rescue_known)
    return(false);
  if((m_curr_time - m_rescue_utc) > 20.0)   // stale θέση → μην εμπιστεύεσαι
    return(false);
  return(hypot(x - m_rescue_x, y - m_rescue_y) < m_avoid_radius);
}

//-----------------------------------------------------------
// Procedure: updateKnownSwimmers()
//   Μαζεύει (dedup κατά id) τους swimmers που είναι ΗΔΗ γνωστοί στην ομάδα μας:
//   registered (SWIMMER_ALERT) + όσους έχει ήδη εντοπίσει ο scout (SCOUTED_SWIMMER).
//   Ο rescue θα τους επισκεφτεί όλους → ο scout τους αποφεύγει.

void BHV_Scout::updateKnownSwimmers()
{
  const char* vars[2] = {"SWIMMER_ALERT", "SCOUTED_SWIMMER"};
  for(int v = 0; v < 2; v++) {
    bool ok;
    string s = getBufferStringVal(vars[v], ok);
    if(!ok || s == "")
      continue;

    string id = ""; double x = 0, y = 0; bool hasx = false, hasy = false;
    vector<string> kv = parseString(s, ',');
    for(unsigned int i = 0; i < kv.size(); i++) {
      string k = tolower(stripBlankEnds(biteStringX(kv[i], '=')));
      string val = stripBlankEnds(kv[i]);
      if(k == "id" || k == "name") id = val;
      else if(k == "x") { x = atof(val.c_str()); hasx = true; }
      else if(k == "y") { y = atof(val.c_str()); hasy = true; }
    }
    if(!hasx || !hasy)
      continue;
    if(id == "")
      id = doubleToStringX(x,1) + "," + doubleToStringX(y,1);
    if(m_known_ids.count(id))
      continue;
    m_known_ids.insert(id);
    m_known_swimmers.push_back(XYPoint(x, y));
  }
}

//-----------------------------------------------------------
// Procedure: knownSwimmerNear()
//   true αν το (x,y) είναι <m_known_radius από οποιονδήποτε γνωστό swimmer.

bool BHV_Scout::knownSwimmerNear(double x, double y) const
{
  if(m_known_radius <= 0)   // disabled → πλήρης κάλυψη lawnmower
    return(false);
  for(size_t i = 0; i < m_known_swimmers.size(); i++)
    if(hypot(x - m_known_swimmers[i].x(), y - m_known_swimmers[i].y()) < m_known_radius)
      return(true);
  return(false);
}

//-----------------------------------------------------------
// Procedure: obstacleNear()
//   true αν το (x,y) είναι <m_obstacle_radius από buoy. Αποτρέπει το να
//   στοχεύει ο scout waypoint μέσα στη ζώνη αποφυγής buoy (που προκαλεί stuck).

bool BHV_Scout::obstacleNear(double x, double y) const
{
  for(size_t i = 0; i < m_obstacles.size(); i++)
    if(hypot(x - m_obstacles[i].x(), y - m_obstacles[i].y()) < m_obstacle_radius)
      return(true);
  return(false);
}

//-----------------------------------------------------------
// Procedure: postViewPoint()

void BHV_Scout::postViewPoint(bool viewable) 
{

  XYPoint pt(m_ptx, m_pty);
  pt.set_vertex_size(5);
  pt.set_vertex_color("orange");
  pt.set_label(m_us_name + "'s next waypoint");
  
  string point_spec;
  if(viewable)
    point_spec = pt.get_spec("active=true");
  else
    point_spec = pt.get_spec("active=false");
  postMessage("VIEW_POINT", point_spec);
}


//-----------------------------------------------------------
// Procedure: buildFunction()

IvPFunction *BHV_Scout::buildFunction() 
{
  if(!m_pt_set)
    return(0);
  
  ZAIC_PEAK spd_zaic(m_domain, "speed");
  spd_zaic.setSummit(m_desired_speed);
  spd_zaic.setPeakWidth(0.5);
  spd_zaic.setBaseWidth(1.0);
  spd_zaic.setSummitDelta(0.8);  
  if(spd_zaic.stateOK() == false) {
    string warnings = "Speed ZAIC problems " + spd_zaic.getWarnings();
    postWMessage(warnings);
    return(0);
  }
  
  double rel_ang_to_wpt = relAng(m_osx, m_osy, m_ptx, m_pty);
  ZAIC_PEAK crs_zaic(m_domain, "course");
  crs_zaic.setSummit(rel_ang_to_wpt);
  crs_zaic.setPeakWidth(0);
  crs_zaic.setBaseWidth(180.0);
  crs_zaic.setSummitDelta(0);  
  crs_zaic.setValueWrap(true);
  if(crs_zaic.stateOK() == false) {
    string warnings = "Course ZAIC problems " + crs_zaic.getWarnings();
    postWMessage(warnings);
    return(0);
  }

  IvPFunction *spd_ipf = spd_zaic.extractIvPFunction();
  IvPFunction *crs_ipf = crs_zaic.extractIvPFunction();

  OF_Coupler coupler;
  IvPFunction *ivp_function = coupler.couple(crs_ipf, spd_ipf, 50, 50);

  return(ivp_function);
}
