#ifndef _TROPOSPHERE_H_
#define _TROPOSPHERE_H_

/*******************************************
 * Hopefield model
 * @param   E   elevating angle
 * @param   H   altitude
 * @param   t0  temperature in centigrade
 * @param   p0  pressure
 * @param   RH0 relative humidity
*******************************************/
double Hopefield(const double E, const double H, double t0 = 15,
                 double p0 = 1013.25, double RH0 = 0.5);
#endif