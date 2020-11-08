#ifndef _SPP_H_
#define _SPP_H_

#include "satpos.h"

// 解算需要对流层改正（双频，不需要改正电离层）
// 对流层改正数需要高度角                    
// 高度角需要知道测站和卫星位置              
// 所以还需要转换到站心坐标系                    
// 卫星位置需要地球自转改正                   
// 地球自传改正需要知道信号传播时间           
// 信号传播时间需要知道测站位置

// 所以先用没有自转改正和无对流层改正计算测站位置
// 得到测站概率位置后将测站和卫星转换到站心坐标系计算高度角
// 根据高度角计算对流层改正、根据测站位置计算信号传播时间，计算地球自传改正
// 迭代直到测站坐标改正数变化不明显
class SPP
{
public:
    int solveSPP(Satellite* &&GPSPosAndVel, Satellite* &&BDSPosAndVel,
                 Obs* &&GPSObs, Obs* &&BDSObs, Ephemeris* &&GPSEph,
                 Ephemeris* &&BDSEph, ELLIPSOID type);


    /***************************************
     * to calculate Elevation Angle
     * @param neu satellite position in neu
     * @return EA in rad
    **************************************/
    double CalculateEA(const XYZ neu);

    /***************************************
     * function: to calculate azimuth angle
     * @param neu satellite position in neu
     * @return azimuth in rad
    ***************************************/
    double CalculateAzi(const XYZ neu);

    /***************************************************
     * function: to fix the influence by earth rotation
    ***************************************************/
    void EarthRotationFix(double deltat, XYZ SatPosition,
                          NavSys flag, XYZ &fixed);
    
    int solveSPV(Satellite* &&GPSPosAndVel, Satellite* &&BDSPosAndVel,
                 Obs* &&GPSObs, Obs* &&BDSObs, Ephemeris* &&GPSEph,
                 Ephemeris* &&BDSEph, ELLIPSOID type);
    
    void setRefPos(XYZ Ref);

    void resetResult();
    
public:
    SPPResult GetResult();
private:
    SPPResult result;
};

#endif