/************************************************************/
/*    NAME: Manos Gonidakis                                 */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: BHV_Pulse.cpp                                    */
/*    DATE: 2026-06-24                                       */
/************************************************************/

#include <cstdlib>
#include "BHV_Pulse.h"
#include "MBUtils.h"
#include "XYRangePulse.h"

using namespace std;

//-----------------------------------------------------------
// Procedure: Constructor

BHV_Pulse::BHV_Pulse(IvPDomain gdomain) : IvPBehavior(gdomain)
{
  IvPBehavior::setParam("name", "pulse");

  // Default τιμές παραμέτρων διαμόρφωσης
  m_pulse_range    = 20;
  m_pulse_duration = 4;

  // Default τιμές μεταβλητών κατάστασης
  m_osx            = 0;
  m_osy            = 0;
  m_curr_time      = 0;
  m_wpt_index      = 0;
  m_index_set      = false;
  m_mark_time      = 0;
  m_pulse_pending  = false;

  // Πληροφορίες που χρειαζόμαστε από το info buffer του helm.
  // WPT_INDEX δημοσιεύεται από το (αδελφό) waypoint behavior — μπορεί να ΜΗΝ
  // υπάρχει στην αρχή (πριν φτάσουμε σε waypoint), οπότε "no_warning".
  addInfoVars("NAV_X, NAV_Y");
  addInfoVars("WPT_INDEX", "no_warning");
}

//---------------------------------------------------------------
// Procedure: setParam - χειρισμός παραμέτρων από το .bhv

bool BHV_Pulse::setParam(string param, string val)
{
  param = tolower(param);
  double double_val = atof(val.c_str());

  if((param == "pulse_range") && (double_val >= 0) && isNumber(val)) {
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
// Procedure: updateInfoIn - φέρε θέση/χρόνο/δείκτη από το info buffer

void BHV_Pulse::updateInfoIn()
{
  bool ok1, ok2;
  m_osx = getBufferDoubleVal("NAV_X", ok1);
  m_osy = getBufferDoubleVal("NAV_Y", ok2);
  if(!ok1 || !ok2)
    postWMessage("No ownship NAV_X/NAV_Y in info_buffer.");

  m_curr_time = getBufferCurrTime();
}

//-----------------------------------------------------------
// Procedure: onRunState

IvPFunction *BHV_Pulse::onRunState()
{
  updateInfoIn();

  // Διάβασε τον τρέχοντα δείκτη waypoint (από το waypoint behavior)
  bool ok_index;
  int new_index = (int)(getBufferDoubleVal("WPT_INDEX", ok_index));

  if(ok_index) {
    if(!m_index_set) {
      // Πρώτη φορά: κράτα baseline, ΜΗΝ παράγεις παλμό στην εκκίνηση
      m_index_set = true;
      m_wpt_index = new_index;
    }
    else if(new_index != m_wpt_index) {
      // Ο δείκτης άλλαξε → φτάσαμε σε waypoint. Μάρκαρε τον χρόνο,
      // ο παλμός θα παραχθεί 5 δευτερόλεπτα αργότερα.
      m_wpt_index     = new_index;
      m_mark_time     = m_curr_time;
      m_pulse_pending = true;
    }
  }

  // 5 δευτερόλεπτα μετά την αλλαγή δείκτη → παρήγαγε τον παλμό μία φορά
  if(m_pulse_pending && ((m_curr_time - m_mark_time) >= 5.0)) {
    postRangePulse();
    m_pulse_pending = false;
  }

  // Το Pulse behavior δεν επηρεάζει την τροχιά → καμία IvP function
  return(0);
}

//-----------------------------------------------------------
// Procedure: postRangePulse - δημοσίευσε VIEW_RANGE_PULSE στη θέση μας

void BHV_Pulse::postRangePulse()
{
  XYRangePulse pulse;
  pulse.set_x(m_osx);
  pulse.set_y(m_osy);
  pulse.set_label("bhv_pulse");
  pulse.set_rad(m_pulse_range);
  pulse.set_time(m_curr_time);
  pulse.set_color("edge", "yellow");
  pulse.set_color("fill", "yellow");
  pulse.set_duration(m_pulse_duration);

  string spec = pulse.get_spec();
  postMessage("VIEW_RANGE_PULSE", spec);
}
