#include "KTypeTC.h"
#include <math.h>

/* --------------------------------------------------------------------------
 *  Coefficient tables — NIST ITS-90 (Monograph 175) for Type-K
 *
 *  Note:
 *    • On 8-bit AVR the ‘double’ type is 32-bit (same as float).
 *      Accuracy remains < ±0.2 °C across the entire range.
 *    • If SRAM is tight, wrap the tables with PROGMEM and read them
 *      via pgm_read_double() / pgm_read_word() before evaluation.
 * -------------------------------------------------------------------------*/
static const double T2E_NEG[] = {
     0.000000000000E+00,  0.394501280250E-01,  0.236223735980E-04,
    -0.328589067840E-06, -0.499048287770E-08, -0.675090591730E-10,
    -0.574103274280E-12, -0.310888728940E-14, -0.104516093650E-16,
    -0.198892668780E-19, -0.163226974860E-22
};

static const double T2E_POS[] = {
    -0.176004136860E-01,  0.389212049750E-01,  0.185587700320E-04,
    -0.994575928740E-07,  0.318409457190E-09, -0.560728448890E-12,
     0.560750590590E-15, -0.320207200030E-18,  0.971511471520E-22,
    -0.121047212750E-25
};

static const double E2T_A[] = {
     0.000000000000E+00,  2.517346200000E+01, -1.166287800000E+00,
    -1.083363800000E+01, -8.977354000000E+00, -3.734237700000E+00,
    -8.663264300000E-01, -1.045059800000E-01, -5.192057700000E-03,
     0.000000000000E+00
};

static const double E2T_B[] = {
     0.000000000000E+00,  2.508355000000E+01,  7.860106000000E-02,
    -2.503131000000E-01,  8.315270000000E-02, -1.228034000000E-02,
     9.804036000000E-04, -4.413030000000E-05,  1.057734000000E-06,
    -1.052755000000E-08
};

static const double E2T_C[] = {
    -1.318058000000E+02,  4.830222000000E+01, -1.646031000000E+00,
     5.464731000000E-02, -9.650715000000E-04,  8.802193000000E-06,
    -3.110810000000E-08
};

/* --------------------------------------------------------------------------
 *  _poly  —  Evaluate a polynomial using Horner’s rule
 *
 *  @param c  Pointer to coefficient array (c[0] is the constant term)
 *  @param n  Number of coefficients in the array
 *  @param x  Argument value
 *  @return   Polynomial value  Σ c[i]·xⁱ  for i = 0 … n−1
 * -------------------------------------------------------------------------*/
double KTypeTC::_poly(const double *c, size_t n, double x)
{
    double y = c[n - 1];
    for (int i = static_cast<int>(n) - 2; i >= 0; --i)
        y = y * x + c[i];
    return y;
}

/* --------------------------------------------------------------------------
 *  _tempToEmf  —  Convert temperature (°C) ➜ thermoelectric EMF (mV)
 *                 Using forward ITS-90 polynomials (piecewise)
 *
 *  Valid range:  −270 °C … +1372 °C
 *  @return  NAN if input is outside the range.
 * -------------------------------------------------------------------------*/
double KTypeTC::_tempToEmf(double t)
{
    if (t < -270.0 || t > 1372.0)
        return NAN;

    if (t < 0.0)
        return _poly(T2E_NEG, sizeof(T2E_NEG) / sizeof(double), t);

    return _poly(T2E_POS, sizeof(T2E_POS) / sizeof(double), t);
}

/* --------------------------------------------------------------------------
 *  _emfToTemp  —  Convert thermoelectric EMF (mV) ➜ temperature (°C)
 *                 Using inverse ITS-90 polynomials (piecewise)
 *
 *  Valid range:  −5.891 mV … +54.886 mV  (≈ −200 °C … +1372 °C)
 *  @return  NAN if input is outside the range.
 * -------------------------------------------------------------------------*/
double KTypeTC::_emfToTemp(double mv)
{
    if (mv < -5.891 || mv > 54.886)
        return NAN;

    if (mv < 0.0)
        return _poly(E2T_A, sizeof(E2T_A) / sizeof(double), mv);

    if (mv <= 20.644)
        return _poly(E2T_B, sizeof(E2T_B) / sizeof(double), mv);

    return _poly(E2T_C, sizeof(E2T_C) / sizeof(double), mv);
}

/* --------------------------------------------------------------------------
 *  temperature  —  Public API
 *                  Calculate hot-junction temperature from:
 *                    • tc_mV      – measured thermocouple voltage (mV)
 *                    • ref_temp_C – reference-junction temperature (°C)
 *
 *  Algorithm:
 *      1. Convert reference temperature to its equivalent EMF.
 *      2. Add that EMF to the measured EMF (cold-junction compensation).
 *      3. Convert the compensated EMF back to temperature.
 *
 *  @return  Hot-junction temperature (°C) or NAN if any input is invalid.
 * -------------------------------------------------------------------------*/
double KTypeTC::temperature(double tc_mV, double ref_temp_C)
{
    const double cj_mV = _tempToEmf(ref_temp_C);   // step 1
    if (isnan(cj_mV))
        return NAN;

    const double total_mV = tc_mV + cj_mV;         // step 2
    return _emfToTemp(total_mV);                   // step 3
}
