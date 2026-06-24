/************************************************************/
/*    NAME: Manos Gonidakis                                 */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: BHV_ZigLeg.h                                     */
/*    DATE: 2026-06-24                                       */
/************************************************************/

#ifndef ZigLeg_HEADER
#define ZigLeg_HEADER

#include <string>
#include "IvPBehavior.h"

class BHV_ZigLeg : public IvPBehavior {
public:
  BHV_ZigLeg(IvPDomain);
  ~BHV_ZigLeg() {};

  bool         setParam(std::string, std::string);
  IvPFunction* onRunState();

protected: // Local Utility functions
  void         updateInfoIn();
  void         postRangePulse();
  IvPFunction* buildZigFunction();

protected: // Configuration parameters
  double   m_zig_angle;         // offset (deg) από το heading έναρξης
  double   m_zig_duration;      // διάρκεια (s) παραγωγής της IvP function
  double   m_pulse_range;       // οπτικός παλμός (όπως στο Pulse)
  double   m_pulse_duration;

protected: // State variables
  double   m_osx;
  double   m_osy;
  double   m_osh;               // ownship heading (NAV_HEADING)
  double   m_curr_time;

  int      m_wpt_index;
  bool     m_index_set;

  double   m_mark_time;         // χρονική στιγμή αλλαγής δείκτη
  bool     m_pulse_pending;     // εκκρεμεί η έναρξη του zigleg (5s μετά);

  bool     m_zig_active;        // παράγουμε αυτή τη στιγμή IvP function;
  double   m_zig_start_time;    // χρονική στιγμή έναρξης του zigleg
  double   m_marked_heading;    // heading τη στιγμή έναρξης (σταθερό για το leg)
};

#ifdef WIN32
   #define IVP_EXPORT_FUNCTION __declspec(dllexport)
#else
   #define IVP_EXPORT_FUNCTION
#endif

extern "C" {
  IVP_EXPORT_FUNCTION IvPBehavior * createBehavior(std::string name, IvPDomain domain)
  {return new BHV_ZigLeg(domain);}
}
#endif
