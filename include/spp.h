#ifndef _SPP_H_
#define _SPP_H_

#include "satpos.h"
#include "readdata.h"

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
    /*****************************************************************
     * function: Standard Point Positioning
     * @param   GPSPosAndVel    GPS satellite position and velocity
     * @param   BDSPosAndVel    BDS satellite position and velocity
     * @param   GPSObs          GPS observations
     * @param   BDSObs          BDS observations
     * @param   GPSEph          GPS ephemeris
     * @param   BDSEph          BDS ephemeris
     * @param   type            Ellipsoid parameters
     * @return                  status code
    *****************************************************************/
    int solveSPP(Satellite* &&GPSPosAndVel, Satellite* &&BDSPosAndVel,
                 Obs* &&GPSObs, Obs* &&BDSObs, Ephemeris* &&GPSEph,
                 Ephemeris* &&BDSEph, ELLIPSOID type);


    /***************************************
     * to calculate Elevation Angle
     * @param neu   satellite position in neu
     * @return      EA in rad
    **************************************/
    double CalculateEA(const XYZ neu);

    /***************************************
     * function: to calculate azimuth angle
     * @param   neu satellite position in neu
     * @return      azimuth in rad
    ***************************************/
    double CalculateAzi(const XYZ neu);

    /**********************************************************
     * function: to fix the influence by earth rotation
     * @param   deltat      [in]      signal transmitting time
     * @param   SatPosition [in]      satellite position
     * @param   flag        [in]      navigation system
     * @param   fixed       [out]     result
    **********************************************************/
    void EarthRotationFix(double deltat, XYZ SatPosition,
                          NavSys flag, XYZ &fixed);
    
    /*****************************************************************
     * function: Standard Positioning Velocity
     * @param   GPSPosAndVel    GPS satellite position and velocity
     * @param   BDSPosAndVel    BDS satellite position and velocity
     * @param   GPSObs          GPS observations
     * @param   BDSObs          BDS observations
     * @param   GPSEph          GPS ephemeris
     * @param   BDSEph          BDS ephemeris
     * @param   type            Ellipsoid parameters
     * @return                  status code
    *****************************************************************/
    int solveSPV(Satellite* &&GPSPosAndVel, Satellite* &&BDSPosAndVel,
                 Obs* &&GPSObs, Obs* &&BDSObs, Ephemeris* &&GPSEph,
                 Ephemeris* &&BDSEph, ELLIPSOID type);
    
    /*******************************************************
     * function: to set reference position of user receiver
     * @param   Ref
    *******************************************************/
    void setRefPos(XYZ Ref);

    /*******************************
     * function: solve SPP question
     * @param   decoder     data
     * @return  status code
    *******************************/
    int solve(ReadData decoder);

    /*************************
     * function: clear result
    *************************/
    void resetResult();
    
public:
    SPPResult GetResult();
private:
    SPPResult result;
};

#endif