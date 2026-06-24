/*****************************************************************/
/*    NAME: M.Benjamin,                                          */
/*    ORGN: Dept of Mechanical Eng / CSAIL, MIT Cambridge MA     */
/*    FILE: BHV_Scout.h                                          */
/*    DATE: April 30th 2022                                      */
/*                                                               */
/* This program is free software; you can redistribute it and/or */
/* modify it under the terms of the GNU General Public License   */
/* as published by the Free Software Foundation; either version  */
/* 2 of the License, or (at your option) any later version.      */
/*                                                               */
/* This program is distributed in the hope that it will be       */
/* useful, but WITHOUT ANY WARRANTY; without even the implied    */
/* warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR       */
/* PURPOSE. See the GNU General Public License for more details. */
/*                                                               */
/* You should have received a copy of the GNU General Public     */
/* License along with this program; if not, write to the Free    */
/* Software Foundation, Inc., 59 Temple Place - Suite 330,       */
/* Boston, MA 02111-1307, USA.                                   */
/*****************************************************************/
 
#ifndef BHV_SCOUT_HEADER
#define BHV_SCOUT_HEADER

#include <string>
#include <vector>
#include <set>
#include "IvPBehavior.h"
#include "XYPoint.h"
#include "XYPolygon.h"

class BHV_Scout : public IvPBehavior {
public:
  BHV_Scout(IvPDomain);
  ~BHV_Scout() {};

  bool         setParam(std::string, std::string);
  void         onIdleState();
  IvPFunction* onRunState();
  void         onEveryState(std::string);

protected:
  IvPFunction* buildFunction();
  void         updateScoutPoint();
  void         postViewPoint(bool viewable=true);
  void         generateLawnmower();    // φτιάχνει τα boustrophedon waypoints
  void         updateRescuePos();       // διαβάζει τη θέση του rescue teammate (NODE_REPORT)
  bool         rescueNear(double x, double y) const;  // <avoid_radius από rescue;
  void         updateKnownSwimmers();   // μαζεύει registered + scouted swimmers
  bool         knownSwimmerNear(double x, double y) const; // <known_radius από γνωστό swimmer;
  bool         obstacleNear(double x, double y) const;     // <obstacle_radius από buoy;

protected: // State variables
  double   m_osx;
  double   m_osy;
  double   m_curr_time;

  double   m_ptx;
  double   m_pty;
  bool     m_pt_set;

  XYPolygon m_rescue_region;

  // Lawnmower coverage state
  std::vector<XYPoint> m_lawn_pts;      // ordered boustrophedon endpoints
  unsigned int         m_lawn_idx;      // επόμενο waypoint στη λίστα
  bool                 m_lawn_built;    // φτιάχτηκε ήδη το grid;

  // Rescue-teammate tracking (για avoidance)
  double   m_rescue_x;
  double   m_rescue_y;
  bool     m_rescue_known;
  double   m_rescue_utc;                // πότε ελήφθη τελευταία θέση (staleness)

  // Known swimmers (registered + scouted) — ο rescue θα τους πάρει, ο scout τους αποφεύγει
  std::vector<XYPoint>  m_known_swimmers;
  std::set<std::string> m_known_ids;    // dedup

  // Obstacles (buoys) — ο scout αποφεύγει να στοχεύει waypoints κοντά τους
  std::vector<XYPoint>  m_obstacles;

  // Stuck-detection (anti-deadlock πίσω από buoy)
  double   m_stuck_x;
  double   m_stuck_y;
  double   m_stuck_time;
  bool     m_stuck_init;

protected: // Config variables
  double m_capture_radius;
  double m_desired_speed;
  double m_lane_width;                  // απόσταση μεταξύ παράλληλων γραμμών σάρωσης
  double m_avoid_radius;                // ακτίνα αποφυγής γύρω από rescue (default 30m)
  double m_known_radius;                // ακτίνα αποφυγής γύρω από known swimmer (default 30m)
  double m_inset;                       // πόσο μέσα από το όριο τραβάμε τα endpoints (default 10m)
  double m_obstacle_radius;             // ακτίνα αποφυγής waypoint γύρω από buoy (default 16m)

  std::string m_tmate;
};

#define IVP_EXPORT_FUNCTION
extern "C" {
  IVP_EXPORT_FUNCTION IvPBehavior * createBehavior(std::string name, IvPDomain domain) 
  {return new BHV_Scout(domain);}
}
#endif
