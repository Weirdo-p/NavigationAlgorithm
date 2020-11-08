#include "../include/troposphere.h"
#include <iostream>
#include <math.h>
#include "../include/coordinate.h"
#include "../include/common.h"

using namespace std;

double Hopefield(const double E, const double H, double t0,
                 double p0, double RH0)
{
    double E1 = Rad2Deg(E);
    double hw = 11000, H0 = 0.0;
    double T0 = t0 + 273.16;
    double RH = RH0 * exp(-0.0006396 * (H - H0));
    double p = p0 * pow(1 - 0.0000226 * (H - H0), 5.225);
    double T = T0 - 0.0065 * (H - H0);
    double e = RH * exp(-37.2465 + 0.213166 * T - 0.000256908 * T * T);
    double hd = 40136 + 148.72 * (T0 - 273.16);
    double Kd = 155.2e-7 * p / T * (hd - H);
    double Kw = 155.2e-7 * 4810.0 / (T * T) * e * (hw - H);
    double left = Kd / sin(Deg2Rad(sqrt(E1 * E1 + 6.25)));
    double right = Kw / sin(Deg2Rad(sqrt(E1 * E1 + 2.25)));
    return left + right;
}
