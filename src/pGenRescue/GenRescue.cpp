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

// Buoy-clearance: σπρώχνει το waypoint ΑΚΤΙΝΙΚΑ έξω από το κοντινότερο buoy ώστε
// ο ΣΤΟΧΟΣ να μη βρίσκεται ποτέ μέσα στη ζώνη Allstop του BHV_AvoidObstacleV24
// (που κηρύσσει "obstacle unavoidable" στα ~<8m από κέντρο buoy και φρενάρει).
// Το push περιορίζεται σε push_max (<=4m) ώστε ο swimmer να μένει εντός rescue_rng:
// η βάρκα φτάνει στο ασφαλές σημείο, ο swimmer είναι <=4m → διασώζεται κανονικά.
static XYPoint clearOfBuoys(const std::vector<XYPoint>& obstacles, XYPoint p,
                            double danger, double push_max)
{
  double px = p.x(), py = p.y();
  int bi = -1; double bd = 1e9;
  for(size_t i = 0; i < obstacles.size(); i++) {
    double d = hypot(px - obstacles[i].x(), py - obstacles[i].y());
    if(d < bd) { bd = d; bi = (int)i; }
  }
  if(bi < 0 || bd >= danger) return p;        // μακριά απ' όλα τα buoys → ok

  double dx = px - obstacles[bi].x(), dy = py - obstacles[bi].y(), d = bd;
  if(d < 1e-6) { dx = 1; dy = 0; d = 1; }      // ακριβώς πάνω στο buoy → διάλεξε άξονα
  double push = danger - d;
  if(push > push_max) push = push_max;
  return XYPoint(px + dx/d*push, py + dy/d*push);
}

// Edge cost: Ευκλείδεια απόσταση + buoy penalty (αν το τμήμα a→b περνά <12m από buoy,
// +500 — ίδια λογική με το Virtual Obstacle Filter). Κρατά τη βέλτιστη διαδρομή μακριά
// από τα buoys ενώ παραμένει pairwise (συμβατό με Held-Karp).
static double tspEdge(double ax, double ay, double bx, double by,
                      const std::vector<XYPoint>& obstacles)
{
  double base = hypot(bx-ax, by-ay);
  double abx = bx-ax, aby = by-ay, ab2 = abx*abx + aby*aby;
  for(size_t o = 0; o < obstacles.size(); o++) {
    double t = 0.0;
    if(ab2 > 1e-9) {
      t = ((obstacles[o].x()-ax)*abx + (obstacles[o].y()-ay)*aby) / ab2;
      if(t < 0) t = 0; if(t > 1) t = 1;
    }
    double cx = ax + t*abx, cy = ay + t*aby;
    if(hypot(obstacles[o].x()-cx, obstacles[o].y()-cy) < 12.0) base += 500.0;
  }
  return base;
}

// Exact open-path TSP (Held-Karp DP) από σταθερή αρχή (sx,sy), επισκεπτόμενο ΟΛΑ τα pts.
// Επιστρέφει τη ΒΕΛΤΙΣΤΗ σειρά (ελάχιστο συνολικό κόστος, ανοιχτή διαδρομή). O(2^n·n^2).
// Καλείται μόνο για n<=16 (ο caller ελέγχει· αλλιώς greedy+2-opt fallback).
static std::vector<XYPoint> exactTSP(double sx, double sy,
                                     const std::vector<XYPoint>& pts,
                                     const std::vector<XYPoint>& obstacles)
{
  int n = (int) pts.size();
  if(n <= 1) return pts;

  std::vector<double> cs(n), cost((size_t)n*n, 0.0);
  for(int j = 0; j < n; j++) {
    cs[j] = tspEdge(sx, sy, pts[j].x(), pts[j].y(), obstacles);
    for(int i = 0; i < n; i++)
      if(i != j)
        cost[(size_t)i*n + j] = tspEdge(pts[i].x(), pts[i].y(), pts[j].x(), pts[j].y(), obstacles);
  }

  int FULL = 1 << n;
  const double INF = 1e18;
  std::vector<double> dp((size_t)FULL*n, INF);
  std::vector<int>    par((size_t)FULL*n, -1);
  for(int j = 0; j < n; j++) dp[(size_t)(1<<j)*n + j] = cs[j];

  for(int mask = 1; mask < FULL; mask++) {
    for(int j = 0; j < n; j++) {
      if(!(mask & (1<<j))) continue;
      double cur = dp[(size_t)mask*n + j];
      if(cur >= INF) continue;
      for(int k = 0; k < n; k++) {
        if(mask & (1<<k)) continue;
        int nmask = mask | (1<<k);
        double nd = cur + cost[(size_t)j*n + k];
        size_t idx = (size_t)nmask*n + k;
        if(nd < dp[idx]) { dp[idx] = nd; par[idx] = j; }
      }
    }
  }

  double best = INF; int bj = -1;
  for(int j = 0; j < n; j++) {
    double v = dp[(size_t)(FULL-1)*n + j];
    if(v < best) { best = v; bj = j; }
  }

  std::vector<int> order;
  int mask = FULL-1, j = bj;
  while(j != -1) { order.push_back(j); int pj = par[(size_t)mask*n + j]; mask ^= (1<<j); j = pj; }
  std::reverse(order.begin(), order.end());

  std::vector<XYPoint> result;
  for(size_t i = 0; i < order.size(); i++) result.push_back(pts[order[i]]);
  return result;
}

// Spatial Clustering (single-linkage / connected components): ομαδοποιεί τους
// swimmers σε γεωγραφικές γειτονιές («οικόπεδα»). Δύο swimmers ανήκουν στο ίδιο
// cluster αν απέχουν <= link_dist (μεταβατικά). ΔΕΝ χαρίζει τίποτα — απλώς
// επιτρέπει τοπικό solve (ταχύτητα, anti-halt) και anti-whipsaw ordering ζωνών.
static std::vector< std::vector<XYPoint> >
clusterPoints(const std::vector<XYPoint>& pts, double link_dist)
{
  size_t n = pts.size();
  std::vector<int> comp(n, -1);
  int nc = 0;
  for(size_t i = 0; i < n; i++) {
    if(comp[i] != -1) continue;
    comp[i] = nc;
    std::vector<size_t> stack;
    stack.push_back(i);
    while(!stack.empty()) {
      size_t u = stack.back(); stack.pop_back();
      for(size_t v = 0; v < n; v++) {
        if(comp[v] != -1) continue;
        if(hypot(pts[u].x()-pts[v].x(), pts[u].y()-pts[v].y()) <= link_dist) {
          comp[v] = nc;
          stack.push_back(v);
        }
      }
    }
    nc++;
  }
  std::vector< std::vector<XYPoint> > clusters(nc);
  for(size_t i = 0; i < n; i++)
    clusters[comp[i]].push_back(pts[i]);
  return clusters;
}

// Greedy open-path tour από σταθερή αρχή (sx,sy) με heading sh, πάνω σε ΣΥΓΚΕΚΡΙΜΕΝΟ
// σύνολο σημείων. Ίδια λογική με το pure-aggressive scoring: (α) urgent override για
// swimmer <12m, (β) score = (neighbors+1)^2 / TTT με turn-penalty + buoy-penalty.
// Καμία concede — επισκέπτεται ΟΛΑ τα σημεία του set. Χρησιμοποιείται είτε για όλο
// το πεδίο είτε τοπικά ανά cluster.
static std::vector<XYPoint>
greedyTour(double sx, double sy, double sh,
           const std::vector<XYPoint>& pts,
           const std::vector<XYPoint>& obstacles,
           double nav_speed, double cluster_radius)
{
  std::vector<XYPoint> ordered;
  std::vector<bool> used(pts.size(), false);
  double cx = sx, cy = sy, ch = sh;
  size_t remaining = pts.size();

  while(remaining > 0) {
    int best = -1;

    // (α) Urgent override: swimmer <12m → άρπαξέ τον (ο κοντινότερος)
    int urg = -1; double urgd = -1;
    for(size_t i = 0; i < pts.size(); i++) {
      if(used[i]) continue;
      double d = hypot(pts[i].x()-cx, pts[i].y()-cy);
      if(d > 12.0) continue;
      if(urgd < 0 || d < urgd) { urgd = d; urg = (int)i; }
    }

    if(urg >= 0)
      best = urg;
    else {
      // (β) Score = (neighbors+1)^2 / TTT
      double best_score = -1;
      for(size_t i = 0; i < pts.size(); i++) {
        if(used[i]) continue;
        double px = pts[i].x(), py = pts[i].y();
        double dist = hypot(px-cx, py-cy);

        int neighbors = 0;
        for(size_t j = 0; j < pts.size(); j++) {
          if(j == i || used[j]) continue;
          if(hypot(pts[j].x()-px, pts[j].y()-py) <= cluster_radius) neighbors++;
        }

        double bearing = atan2(px-cx, py-cy) * 180.0 / M_PI;
        if(bearing < 0) bearing += 360.0;
        double he = fabs(ch - bearing);
        if(he > 180.0) he = 360.0 - he;
        double speed = (nav_speed < 0.2) ? 1.0 : nav_speed;
        double ttt = (he/30.0)*(1.0 + speed*(he/180.0)) + dist/speed;

        for(size_t oi = 0; oi < obstacles.size(); oi++) {
          double abx = px-cx, aby = py-cy, ab2 = abx*abx + aby*aby, t = 0.0;
          if(ab2 > 1e-9) {
            t = ((obstacles[oi].x()-cx)*abx + (obstacles[oi].y()-cy)*aby) / ab2;
            if(t < 0) t = 0; if(t > 1) t = 1;
          }
          double ox = cx + t*abx, oy = cy + t*aby;
          if(hypot(obstacles[oi].x()-ox, obstacles[oi].y()-oy) < 12.0) ttt += 500.0;
        }

        double score = ((neighbors+1)*(neighbors+1)) / (ttt + 1e-6);
        if(score > best_score) { best_score = score; best = (int)i; }
      }
    }

    if(best < 0) break;

    double nb = atan2(pts[best].x()-cx, pts[best].y()-cy) * 180.0 / M_PI;
    if(nb < 0) nb += 360.0;
    ch = nb;
    ordered.push_back(pts[best]);
    cx = pts[best].x(); cy = pts[best].y();
    used[best] = true;
    remaining--;
  }
  return ordered;
}

// Flyby Rescue: αφαιρεί ενδιάμεσα waypoints που απέχουν < thr από το ευθύγραμμο
// τμήμα των ΚΡΑΤΗΜΕΝΩΝ γειτόνων τους — η βάρκα τα σώζει "εν κινήσει" (εντός του
// rescue_rng 5m) χωρίς να σταματήσει/στρίψει. Λιγότερα waypoints → λιγότερο
// φρενάρισμα → ταχύτερη συλλογή. Δεν επιτρέπει διαδοχικά skips (κάθε skipped
// swimmer εγγυάται ότι βρίσκεται ανάμεσα σε ΔΥΟ κρατημένα σημεία).
static void flybyReduce(double sx, double sy, std::vector<XYPoint>& pts, double thr)
{
  if(pts.size() < 2) return;
  std::vector<XYPoint> kept;
  double px = sx, py = sy;     // τελευταίο ΚΡΑΤΗΜΕΝΟ σημείο
  bool justSkipped = false;
  for(size_t i = 0; i < pts.size(); i++) {
    if(i == pts.size()-1) { kept.push_back(pts[i]); break; }  // πάντα κράτα τον τελευταίο

    double ax=px, ay=py, bx=pts[i+1].x(), by=pts[i+1].y();
    double abx=bx-ax, aby=by-ay, ab2=abx*abx+aby*aby, t=0.0;
    if(ab2>1e-9){ t=((pts[i].x()-ax)*abx+(pts[i].y()-ay)*aby)/ab2; if(t<0)t=0; if(t>1)t=1; }
    double cx=ax+t*abx, cy=ay+t*aby;
    double d = hypot(pts[i].x()-cx, pts[i].y()-cy);

    if(!justSkipped && d < thr && t > 0.05 && t < 0.95) {
      justSkipped = true;       // skip — σώζεται εν κινήσει· ΜΗΝ ενημερώσεις prev_kept
      continue;
    }
    kept.push_back(pts[i]);
    px = pts[i].x(); py = pts[i].y();
    justSkipped = false;
  }
  pts = kept;
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
  m_deadlock_escape = true;        // default ON
  m_use_exact_tsp = false;         // default OFF — greedy+2-opt είναι ταχύτερο (turn-aware)
  m_flyby_rescue = false;          // default OFF (A/B)
  m_cluster_split = true;          // default ON — clustering χωρίς concede (anti-whipsaw)
  m_cluster_link_dist = 40.0;      // swimmers <=40m → ίδιο «οικόπεδο»
  m_stuck_ref_x = 0;
  m_stuck_ref_y = 0;
  m_stuck_ref_time = 0;
  m_stuck_init = false;
  m_escape_until = 0;
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
      bool is_abandoned = (m_abandoned.find(id) != m_abandoned.end());

      if (!is_active && !is_rescued && !is_abandoned) {
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
    if(m_first_run_done && !m_returned) {
      // ΤΕΛΟΣ: όλοι διασώθηκαν → γύρνα σπίτι (Assignment 1, Lab 12)
      Notify("RETURN", "true");
      m_returned = true;
    }
    else if(!m_first_run_done) {
      // ΑΡΧΗ / Fail-Safe: δεν ήρθαν ακόμα swimmers (early-deploy). ΜΗΝ αφήσεις
      // τη βάρκα να τρέξει στο default (0,0) — κράτα τη στη θέση της μέχρι να
      // φορτώσει το πεδίο (απέτρεψε το HaltPoly thrashing που είδαμε στα logs).
      // ΣΙΩΠΗΛΑ — το buildReport δείχνει "Active Swimmers: 0" (όχι run-warning).
      Notify("RESCUE_UPDATES", "points=" + doubleToStringX(m_nav_x,2)
                                    + "," + doubleToStringX(m_nav_y,2));
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

  // --- 1b. DEADLOCK-ESCAPE ---
  // Αν είμαστε σε escape mode, κράτα το escape waypoint μέχρι να λήξει ο timer.
  if(m_curr_time < m_escape_until) {
    AppCastingMOOSApp::PostReport();
    return(true);
  }
  // Ανίχνευση με ΠΑΡΑΘΥΡΟ: net displacement σε 10s. Πιάνει και ΤΑΛΑΝΤΩΣΗ-επιτόπου
  // (η κολλημένη βάρκα κουνιέται αλλά δεν προχωρά). Κανονικά σε 10s διανύει 12-16m.
  if(m_deadlock_escape && !m_swimmers.empty() && m_first_run_done) {
    if(!m_stuck_init) {
      m_stuck_ref_x = m_nav_x; m_stuck_ref_y = m_nav_y;
      m_stuck_ref_time = m_curr_time; m_stuck_init = true;
    }
    else if((m_curr_time - m_stuck_ref_time) >= 10.0) {
      double net = hypot(m_nav_x - m_stuck_ref_x, m_nav_y - m_stuck_ref_y);
      // reset checkpoint (παράθυρο)
      m_stuck_ref_x = m_nav_x; m_stuck_ref_y = m_nav_y; m_stuck_ref_time = m_curr_time;

      // Κοντινότερο buoy από την ΤΩΡΙΝΗ θέση
      double best_bd = 1e9; int bi = -1;
      for(unsigned int oi = 0; oi < m_obstacles.size(); oi++) {
        double d = hypot(m_obstacles[oi].x()-m_nav_x, m_obstacles[oi].y()-m_nav_y);
        if(d < best_bd) { best_bd = d; bi = oi; }
      }
      if(net < 6.0 && bi >= 0 && best_bd < 15.0) {
        // DEADLOCK. (α) escape waypoint 20m μακριά από το buoy
        double ax = m_nav_x - m_obstacles[bi].x();
        double ay = m_nav_y - m_obstacles[bi].y();
        double al = hypot(ax, ay);
        if(al < 1e-6) { ax = 1; ay = 0; al = 1; }
        XYPoint esc = clampInside(m_region,
                        XYPoint(m_nav_x + (ax/al)*20.0, m_nav_y + (ay/al)*20.0), 5.0);
        Notify("RESCUE_UPDATES", "points=" + doubleToStringX(esc.x(),2)
                                      + "," + doubleToStringX(esc.y(),2));
        // (β) Βρες τον behind-buoy swimmer (κοντινότερο στο buoy)
        string cul = ""; double cbd = 1e9;
        for(auto const& sp : m_swimmers) {
          double d = hypot(sp.second.x()-m_obstacles[bi].x(), sp.second.y()-m_obstacles[bi].y());
          if(d < cbd) { cbd = d; cul = sp.first; }
        }
        // Anti-oscillation: 1η φορά → temp blacklist (25s, δεύτερη ευκαιρία από άλλη γωνία).
        // 2η φορά ίδιος swimmer → ΜΟΝΙΜΗ εγκατάλειψη (σταμάτα να σπαταλάς χρόνο σε ταλάντωση).
        if(cul != "") {
          m_swimmer_escape_count[cul]++;
          if(m_swimmer_escape_count[cul] >= 2) {
            m_swimmers.erase(cul);
            m_abandoned.insert(cul);
            m_buoy_blacklist.erase(cul);
            m_swimmer_approach_times.erase(cul);
            reportRunWarning("DEADLOCK-ESCAPE: permanently abandoning " + cul + " (repeated deadlock)");
          }
          else {
            m_buoy_blacklist[cul] = m_curr_time + 25.0;
            reportRunWarning("DEADLOCK-ESCAPE: routing wide around buoy");
          }
        }
        Notify("PGR_DEADLOCK_ESCAPE", "buoy=" + doubleToStringX(m_obstacles[bi].x(),1)
               + "," + doubleToStringX(m_obstacles[bi].y(),1) + ",net=" + doubleToStringX(net,1));
        m_escape_until = m_curr_time + 8.0;
        m_stuck_init = false;
        m_path_update_needed = true;
        AppCastingMOOSApp::PostReport();
        return(true);
      }
    }
  }

  // --- 2. ΕΝΗΜΕΡΩΣΗ ΔΙΑΔΡΟΜΗΣ (Με Διορθώσεις Claude Code) ---
  // Αντικαταστήσαμε το static με το μέλος m_first_run_done
  if (m_path_update_needed || (!m_first_run_done && !m_swimmers.empty())) {
      
      vector<XYPoint> ordered_pts;

      map<string, XYPoint> remaining_swimmers = m_swimmers;

      // Deadlock-Escape: εξαίρεσε τους προσωρινά blacklisted (behind-buoy) swimmers
      // ώστε ο planner να μην ξαναστείλει τη βάρκα στο ίδιο deadlock. Fallback: αν
      // αδειάσει το set, κράτα όλους (μη μείνεις χωρίς στόχο).
      {
        map<string, XYPoint> avail;
        for(auto const& sp : remaining_swimmers) {
          auto bit = m_buoy_blacklist.find(sp.first);
          if(bit != m_buoy_blacklist.end() && m_curr_time < bit->second) continue;
          avail[sp.first] = sp.second;
        }
        if(!avail.empty()) remaining_swimmers = avail;
      }

      double cluster_radius = 40.0;

      // === MAX AGGRESSION: ΚΑΝΕΝΑΣ swimmer δεν χαρίζεται ===
      // ΔΕΝ αφαιρούμε ποτέ swimmer από το planning set λόγω αντιπάλου. Όλοι
      // μένουν στο tour και ο planner τους διεκδικεί. Αν ο εχθρός προλάβει
      // κάποιον, έρχεται FOUND_SWIMMER → φεύγει από το m_swimmers → replan
      // (δεν σπαταλάμε χρόνο σε ήδη-χαμένο). Σε equal-speed κούρσα η γεωμετρία
      // μοιράζει το πεδίο μόνη της και κερδίζουμε ΚΑΘΕ λάθος του αντιπάλου.

      // Build vector από τους διαθέσιμους swimmers
      std::vector<XYPoint> swset;
      for(auto const& sp : remaining_swimmers) swset.push_back(sp.second);

      // === EXACT TSP (Held-Karp): βέλτιστη διαδρομή για N<=16 (flag, default OFF) ===
      bool use_exact = (m_use_exact_tsp && swset.size() <= 16);

      if(use_exact) {
        ordered_pts = exactTSP(m_nav_x, m_nav_y, swset, m_obstacles);
      }
      else if(m_cluster_split && swset.size() >= 2) {
        // === CLUSTER-BASED PLANNING (ΧΩΡΙΣ concede) ===
        // Σπάμε το πεδίο σε «οικόπεδα» (clusters) και λύνουμε ΤΟΠΙΚΟ tour ανά οικόπεδο,
        // επισκεπτόμενοι τα οικόπεδα nearest-first. Όφελος: (1) anti-whipsaw — τελειώνεις
        // μια γειτονιά πριν κάνεις transit στην άλλη, χωρίς ο global solver να σε στέλνει
        // «σφήνα» πέρα-δώθε, (2) ταχύτητα/anti-halt — μικρά τοπικά TSP. ΔΕΝ χαρίζουμε
        // κανένα cluster: τα επισκεπτόμαστε ΟΛΑ — μένει 100% aggressive.
        std::vector< std::vector<XYPoint> > clusters =
          clusterPoints(swset, m_cluster_link_dist);

        double cx = m_nav_x, cy = m_nav_y, ch = m_nav_heading;
        std::vector<bool> cdone(clusters.size(), false);

        for(size_t step = 0; step < clusters.size(); step++) {
          // Διάλεξε το ΚΟΝΤΙΝΟΤΕΡΟ μη-επισκεμμένο cluster (min απόσταση μέλους)
          int bestc = -1; double bestd = -1;
          for(size_t c = 0; c < clusters.size(); c++) {
            if(cdone[c]) continue;
            double cd = -1;
            for(size_t k = 0; k < clusters[c].size(); k++) {
              double d = hypot(clusters[c][k].x()-cx, clusters[c][k].y()-cy);
              if(cd < 0 || d < cd) cd = d;
            }
            if(bestd < 0 || cd < bestd) { bestd = cd; bestc = (int)c; }
          }
          if(bestc < 0) break;
          cdone[bestc] = true;

          // Τοπικό tour μέσα στο cluster, ξεκινώντας από την τρέχουσα θέση
          std::vector<XYPoint> local =
            greedyTour(cx, cy, ch, clusters[bestc], m_obstacles, m_nav_speed, cluster_radius);
          twoOptPath(cx, cy, local);   // τοπικό 2-opt από το σημείο εισόδου

          // Άλυσος: πρόσθεσε το τοπικό tour, ενημέρωσε θέση/heading στο τέλος του
          for(size_t k = 0; k < local.size(); k++) ordered_pts.push_back(local[k]);
          if(!local.empty()) {
            double prevx = (local.size() >= 2) ? local[local.size()-2].x() : cx;
            double prevy = (local.size() >= 2) ? local[local.size()-2].y() : cy;
            XYPoint last = local.back();
            double nb = atan2(last.x()-prevx, last.y()-prevy) * 180.0 / M_PI;
            if(nb < 0) nb += 360.0;
            ch = nb; cx = last.x(); cy = last.y();
          }
        }
      }
      else {
        // Ένα μόνο «οικόπεδο» (clustering off ή <2 swimmers) → global greedy+2-opt
        ordered_pts = greedyTour(m_nav_x, m_nav_y, m_nav_heading,
                                 swset, m_obstacles, m_nav_speed, cluster_radius);
        twoOptPath(m_nav_x, m_nav_y, ordered_pts);
      }

      // Flyby: αφαίρεσε σχεδόν-collinear ενδιάμεσα waypoints (σώζονται εν κινήσει
      // εντός 5m). thr=3.0m → ~2m margin κάτω από το rescue_rng για path deviation.
      if(m_flyby_rescue)
        flybyReduce(m_nav_x, m_nav_y, ordered_pts, 3.0);

      // Boundary-aware: τράβα κάθε waypoint >=5m μέσα από το όριο ώστε η βάρκα
      // να μην στοχεύει ποτέ στη halt ζώνη του OpRegion (halt_dist=4.5m).
      XYSegList my_path;
      for(unsigned int k = 0; k < ordered_pts.size(); k++) {
        // 1) Σπρώξε τον στόχο έξω από ζώνη buoy (αποφυγή AvoidObstacle Allstop)
        XYPoint cleared = clearOfBuoys(m_obstacles, ordered_pts[k], 12.0, 4.0);
        // 2) Μετά κράτα τον εντός ορίου (boundary-aware)
        XYPoint safe = clampInside(m_region, cleared, 5.0);
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
      else if(param == "deadlock_escape") {
        string v = tolower(stripBlankEnds(value));
        m_deadlock_escape = (v == "true");
      }
      else if(param == "use_exact_tsp") {
        string v = tolower(stripBlankEnds(value));
        m_use_exact_tsp = (v == "true");
      }
      else if(param == "flyby_rescue") {
        string v = tolower(stripBlankEnds(value));
        m_flyby_rescue = (v == "true");
      }
      else if(param == "cluster_split") {
        string v = tolower(stripBlankEnds(value));
        m_cluster_split = (v == "true");
      }
      else if(param == "cluster_link_dist") {
        double d = atof(stripBlankEnds(value).c_str());
        if(d > 0) m_cluster_link_dist = d;
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
  m_msgs << "Aggressive (no concede): ALWAYS ON  [hardcoded — no swimmer conceded]" << endl;
  m_msgs << "Cluster-Split Planning:  " << (m_cluster_split ? "ON" : "OFF")
         << "  [link_dist=" << m_cluster_link_dist << "m, NO concede]" << endl;
  m_msgs << "Deadlock-Escape:         " << (m_deadlock_escape ? "ON" : "OFF")
         << "  [blacklist:" << m_buoy_blacklist.size()
         << " abandoned:" << m_abandoned.size() << "]" << endl;
  m_msgs << "Flyby Rescue:            " << (m_flyby_rescue ? "ON" : "OFF") << endl;
  m_msgs << "Enemy X/Y Position:      [" << m_enemy_x << ", " << m_enemy_y << "]" << endl;
  m_msgs << endl;
  m_msgs << "Rescue Status              " << endl;
  m_msgs << "------------------------ " << endl;
  m_msgs << "Active Swimmers in list: " << m_swimmers.size() << endl;
  m_msgs << "Rescued Swimmers:        " << m_rescued_swimmers.size() << endl;
  m_msgs << "Path Update Needed:      " << (m_path_update_needed ? "true" : "false") << endl;

  return(true);
}