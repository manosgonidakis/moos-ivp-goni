/************************************************************/
/*    NAME: Manos Gonidakis                                 */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: BHV_Pulse.h                                      */
/*    DATE: 2026-06-24                                       */
/************************************************************/

#ifndef Pulse_HEADER
#define Pulse_HEADER

#include <string>
#include "IvPBehavior.h"

class BHV_Pulse : public IvPBehavior {
public:
  BHV_Pulse(IvPDomain);
  ~BHV_Pulse() {};

  bool         setParam(std::string, std::string);
  IvPFunction* onRunState();

protected: // Local Utility functions
  void         updateInfoIn();
  void         postRangePulse();

protected: // Configuration parameters
  double   m_pulse_range;       // radius (m) στο οποίο φτάνει ο παλμός
  double   m_pulse_duration;    // διάρκεια (s) εμφάνισης του παλμού

protected: // State variables
  double   m_osx;               // ownship x (NAV_X)
  double   m_osy;               // ownship y (NAV_Y)
  double   m_curr_time;         // τρέχων χρόνος (info buffer)

  int      m_wpt_index;         // τελευταίος γνωστός δείκτης waypoint
  bool     m_index_set;         // έχουμε δει έγκυρο WPT_INDEX τουλάχιστον μία φορά;

  double   m_mark_time;         // χρονική στιγμή που άλλαξε ο δείκτης
  bool     m_pulse_pending;     // εκκρεμεί παλμός 5s μετά την αλλαγή;
};

#ifdef WIN32
   #define IVP_EXPORT_FUNCTION __declspec(dllexport)
#else
   #define IVP_EXPORT_FUNCTION
#endif

extern "C" {
  IVP_EXPORT_FUNCTION IvPBehavior * createBehavior(std::string name, IvPDomain domain)
  {return new BHV_Pulse(domain);}
}
#endif
