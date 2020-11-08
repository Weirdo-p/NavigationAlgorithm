/******************************************************
 * to declare functions related to coordinate transform
 * @author XUZHUO
******************************************************/
#ifndef _COORDINATE_H_
#define _COORDINATE_H_

#include "common.h"

// WGS84基椭球参数
extern const double a_w;
extern const double b_w;

// CGCS2000 椭球参数
extern const double a_c;
extern const double b_c;

/************************************
 * function: to transform deg to rad
 * @param deg  double  degree
 * @return  rad  double  rad
************************************/
double Deg2Rad(const double deg);

/************************************
 * function: to transform rad to deg
 * @param  rad  double rad
 * @return  deg  double degree
************************************/
double Rad2Deg(const double rad);

/************************************
 * function: 计算卯酉圈曲率半径
 * @param   blh        [in]  BLH
 * @param   ellipsoid  [in] ELLIPSOID
 * @param   N          [out] N
 * @return flag  status code
 *         true  success
 *         false deadly error
************************************/
bool GetN(const BLH blh, const ELLIPSOID ellipsoid, double &N);

/**********************************
 * function: transform BLH to XYZ
 * @param blh [in]   BLH
 * @param ellipsoid  ELLIPSOID
 * @param xzy [out]  XYZ
 * @return flag  status code
 *         true  success
 *         false deadly error
**********************************/
bool BLH2XYZ(BLH blh, ELLIPSOID ellipsoid, XYZ &xyz);

/**********************************
 * function: transform XYZ to BLH
 * @param xyz [in]   XYZ
 * @param ellipsoid  ELLIPSOID
 * @param blh [out]  BLH
 * @return flag  status code
 *         true  success
 *         false deadly error
**********************************/
bool XYZ2BLH(XYZ xyz, ELLIPSOID ellipsoid, BLH &blh);

/********************************************
 * function: 地心地固转站心坐标系
 * @param station [in] station coordinate
 * @param obj     [in] 
 * @param neu     [out]
 * @return status code
********************************************/
bool XYZ2NEU(const XYZ station, const XYZ obj, ELLIPSOID type, XYZ &neu);
#endif
