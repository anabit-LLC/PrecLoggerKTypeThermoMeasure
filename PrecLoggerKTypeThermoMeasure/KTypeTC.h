#ifndef K_TYPE_TC_H
#define K_TYPE_TC_H
/* --------------------------------------------------------------------------
 *  KTypeTC  –  NIST-ITS-90 Type-K thermocouple helper for Arduino / C++
 * --------------------------------------------------------------------------
 *  Usage:
 *      #include "KTypeTC.h"
 *      double mv  = 1.532;   // measured thermocouple voltage in mV
 *      double tCJ = 25.0;    // reference-junction temperature in °C
 *      double tHot = KTypeTC::temperature(mv, tCJ);
 * --------------------------------------------------------------------------*/

#include <Arduino.h>   // pulls in <math.h> & isnan() for all cores

class KTypeTC
{
public:
    /** Return the hot-junction temperature (°C).
     *  @param tc_mV       Measured thermocouple voltage (millivolts)
     *  @param ref_temp_C  Reference-junction temperature (°C)
     *  @return Calculated temperature or NAN on range/argument error
     */
    static double temperature(double tc_mV, double ref_temp_C);

private:
    /* --- helpers (implementation in .cpp) --- */
    static double  _poly(const double *c, size_t n, double x);
    static double  _tempToEmf(double t);   // °C ➜ mV
    static double  _emfToTemp(double mv);  // mV ➜ °C
};

/* ---------------------------------------------------------------------- */
#endif  /* K_TYPE_TC_H */