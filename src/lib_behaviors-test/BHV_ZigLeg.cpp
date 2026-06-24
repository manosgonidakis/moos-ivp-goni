/************************************************************/
/*    NAME: Manos Gonidakis                                 */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: BHV_ZigLeg.cpp                                   */
/*    DATE: 2026-06-24                                       */
/************************************************************/

#include <cstdlib>
#include "BHV_ZigLeg.h"
#include "MBUtils.h"
#include "AngleUtils.h"
#include "BuildUtils.h"
#include "ZAIC_PEAK.h"
#include "XYRangePulse.h"

using namespace std;

//-----------------------------------------------------------
// Procedure: Constructor

BHV_ZigLeg::BHV_ZigLeg(IvPDomain gdomain) : IvPBehavior(gdomain)
{
  IvPBehavior::setParam("name", "zigleg");
  m_domain = subDomain(m_domain, "course");

  // Default τιμές παραμέτρων διαμόρφωσης
  m_zig_angle      = 45;
  m_zig_duration   = 10;
  m_pulse_range    = 20;
  m_pulse_duration = 4;

  // Default τιμές μεταβλητών κατάστασης
  m_osx            = 0;
  m_osy            = 0;
  m_osh            = 0;
  m_curr_time      = 0;
  m_wpt_index      = 0;
  m_index_set      = false;
  m_mark_time      = 0;
  m_pulse_pending  = false;
  m_zig_active     = false;
  m_zig_start_time = 0;
  m_marked_heading = 0;

  // WPT_INDEX μπορεί να λείπει στην αρχή (πριν φτάσουμε σε waypoint) → no_warning
  addInfoVars("NAV_X, NAV_Y, NAV_HEADING");
  addInfoVars("WPT_INDEX", "no_warning");
}

//---------------------------------------------------------------
// Procedure: setParam

bool BHV_ZigLeg::setParam(string param, string val)
{
  param = tolower(param);
  double double_val = atof(val.c_str());

  if((param == "zig_angle") && isNumber(val)) {
    m_zig_angle = double_val;
    return(true);
  }
  else if((param == "zig_duration") && (double_val >= 0) && isNumber(val)) {
    m_zig_duration = double_val;
    return(true);
  }
  else if((param == "pulse_range") && (double_val >= 0) && isNumber(val)) {
    m_pulse_range = double_val;
    return(true);
  }
  else if((param == "pulse_duration") && (double_val >= 0) && isNumber(val)) {
    m_pulse_duration = double_val;
    return(true);
  }
  return(false);
}

//-----------------------------------------------------------
// Procedure: updateInfoIn

void BHV_ZigLeg::updateInfoIn()
{
  bool ok1, ok2, ok3;
  m_osx = getBufferDoubleVal("NAV_X", ok1);
  m_osy = getBufferDoubleVal("NAV_Y", ok2);
  m_osh = getBufferDoubleVal("NAV_HEADING", ok3);
  if(!ok1 || !ok2 || !ok3)
    postWMessage("No ownship NAV_X/NAV_Y/NAV_HEADING in info_buffer.");

  m_curr_time = getBufferCurrTime();
}

//-----------------------------------------------------------
// Procedure: onRunState

IvPFunction *BHV_ZigLeg::onRunState()
{
  updateInfoIn();

  bool ok_index;
  int new_index = (int)(getBufferDoubleVal("WPT_INDEX", ok_index));

  if(ok_index) {
    if(!m_index_set) {
      m_index_set = true;
      m_wpt_index = new_index;
    }
    else if(new_index != m_wpt_index) {
      // Φτάσαμε σε waypoint → προγραμμάτισε zigleg σε 5 δευτερόλεπτα
      m_wpt_index     = new_index;
      m_mark_time     = m_curr_time;
      m_pulse_pending = true;
    }
  }

  // 5s μετά την αλλαγή δείκτη: ξεκίνα το zigleg (παλμός + έναρξη IvP function)
  if(m_pulse_pending && ((m_curr_time - m_mark_time) >= 5.0)) {
    postRangePulse();
    m_pulse_pending  = false;
    m_zig_active     = true;
    m_zig_start_time = m_curr_time;
    m_marked_heading = m_osh;     // κλείδωσε το heading έναρξης (αλλιώς κάνει κύκλο)
  }

  // Όσο είμαστε στο παράθυρο zig_duration → παρήγαγε IvP function στο heading
  if(m_zig_active) {
    if((m_curr_time - m_zig_start_time) <= m_zig_duration)
      return(buildZigFunction());
    else
      m_zig_active = false;
  }

  return(0);
}

//-----------------------------------------------------------
// Procedure: buildZigFunction - ZAIC_PEAK πάνω στο course, με κορυφή
//            στο (heading_έναρξης + zig_angle)

IvPFunction *BHV_ZigLeg::buildZigFunction()
{
  double zig_heading = angle360(m_marked_heading + m_zig_angle);

  ZAIC_PEAK crs_zaic(m_domain, "course");
  crs_zaic.setSummit(zig_heading);
  crs_zaic.setPeakWidth(0);
  crs_zaic.setBaseWidth(180.0);
  crs_zaic.setSummitDelta(0);
  crs_zaic.setValueWrap(true);

  if(crs_zaic.stateOK() == false) {
    postWMessage("Course ZAIC problems " + crs_zaic.getWarnings());
    return(0);
  }

  IvPFunction *ipf = crs_zaic.extractIvPFunction();
  if(ipf)
    ipf->setPWT(m_priority_wt);

  return(ipf);
}

//-----------------------------------------------------------
// Procedure: postRangePulse

void BHV_ZigLeg::postRangePulse()
{
  XYRangePulse pulse;
  pulse.set_x(m_osx);
  pulse.set_y(m_osy);
  pulse.set_label("bhv_zigleg");
  pulse.set_rad(m_pulse_range);
  pulse.set_time(m_curr_time);
  pulse.set_color("edge", "green");
  pulse.set_color("fill", "green");
  pulse.set_duration(m_pulse_duration);

  postMessage("VIEW_RANGE_PULSE", pulse.get_spec());
}
