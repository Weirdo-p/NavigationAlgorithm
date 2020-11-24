/******************************************************
 * this file is used to calculate values related to sat
 * @author XUZHUO WHU
 * 2020 10 08
******************************************************/

#ifndef _SATPOS_H_
#define _SATPOS_H_

#include "common.h"
#include "readdata.h"
#include "matrix.h"
#include "timetrans.h"
#include "coordinate.h"

/************************************************
 * function: to calculate sat pos  sat velocity
 *           clock difference  clock velocity
************************************************/
class SatPos
{
    /* main functions */
public:
    /**************************************
     * function: to calculate sat position
     * @param  GPSEph   Ephemeris of GPS
     * @return          status code
    **************************************/
    int CalculateGPSPos(Ephemeris* &&GPSEph, Obs* &&obs, const SPPResult result);

    /**************************************
     * function: to calculate sat position
     * @param  BDSEph       Ephemeris of BDS
     * @param  FileHead     Head Information
     * @return              status code
    **************************************/
    int CalculateBDSPos(Ephemeris* && BDSEph, Obs* &&obs, const SPPResult result);

    /******************************************************
     * function: to satellite velocity in earth coordinate
     * @param Eph      [in] Ephemris
     * @param flag     [in] Nav system
     * @param param    [in] calculated in position calculation
     * @return              status code
    ******************************************************/
    int CalculateSatVel(Ephemeris* &&Eph, const NavSys flag, SPPResult result);

    /************************************************************
     * function: to calculate the GEO satellite velocity
     * @param tk        Time from ephemeris reference epoch
     * @param SatPosGK  SatPosion in CGCS2000 before projection
     * @param prn       satellite prn
     * @return          status code
    ************************************************************/
    int CalculateGEOVel(const double tk, const XYZ SatPosGK, const int prn, const Matrix<double, 3, 1> velocity, SPPResult result);

/* helpers */
public:
    /*********************************
     * @return GPS Satellite position
    *********************************/
    SatPos GetGPSPos();

    /*********************************************************
     * function: to calculate the time of generating signals
     * @param ObsData  [in]     observation 
     * @param eph      [in]
     * @param prn      [in]     satellite prn number
     * @param sys      [in]     system flag
     * @param t        [out]    time of generating signals
     * @return status code
    *********************************************************/
    int Calculatet(const Obs* ObsData, const Ephemeris* Eph,
                   const int prn, const NavSys sys, const SPPResult result, SATTIME &t);

    /*****************************************************
     * function: to calculate satellite clock difference
     * @param BDSEph  [in]  BDS Ephemeris
     * @param prn     [in]  satellite prn number
     * @param tsv     [in]  卫星钟表面时
     * @param sys     [in]  system flag
     * @param ClkDif  [out] satellite clock difference
    *****************************************************/
    int CalculateClkDif(const Ephemeris* Eph, const int prn, const SATTIME tsv, const NavSys sys, double &ClkDif);


    /*****************************************************
     * function: to calculate satellite clock difference
     * @param BDSEph  [in]  BDS Ephemeris
     * @param prn     [in]  satellite prn number
     * @param tsv     [in]  卫星钟表面时
     * @param sys     [in]  system flag
     * @param ClkDif  [out] satellite clock difference
    *****************************************************/
    int CalculateClkDifdot(const Ephemeris* Eph, const int prn, const SATTIME tsv, const NavSys sys, double &ClkDif);

    /**********************************************
     * function: to calculate mean motion(rad/sec)
     * @param sqrtA   [in]  轨道长半轴的1/2次方
     * @param n0      [out] 卫星运动平均角速度
     * @param flag    [in]  nav sys type
    **********************************************/
    void Calculaten0(const double sqrtA, NavSys flag, double &n0);

    /*************************************************************
     * function: to calculate Time from ephemeris reference epoch
     * @param ObsTime [in]      observation time
     * @param RefTime [in]      Reference Time Ephemeris
     * @param tk      [out]     Time from ephemeris reference epoch
     * @param flag    [in]      nav sys type
     * @return                  true if it is not expired
    **************************************************************/
    bool Calculatetk(const SATTIME ObsTime, const SATTIME RefTime, NavSys flag, double &tk);

    /*******************************************************************
     * function: to calculate Corrected mean motion
     * @param n0      [in]      卫星运动平均角速度
     * @param deltan  [in]      Mean Motion Difference From Computed Value
     * @param n       [out]     Corrected mean motion
    *******************************************************************/
    void Calculaten(const double n0, const double deltan, double &n);

    /********************************************************
     * function: to calculate Mean anomaly
     * @param M0  [in]  Mean Anomaly at Reference Time
     * @param tk  [in]  Time from ephemeris reference epoch
     * @param n   [in]  Corrected mean motion
     * @param Mk  [out] Mean anomaly
    ********************************************************/
    void CalculateMk(const double M0, const double tk, const double n, double &Mk);

    /**********************************
     * function: to calculate 偏近点角
     * @param Mk     [in]   平近点角
     * @param Ek     [out]  偏近点角
     * @param ecc    [in]   轨道偏心率
    **********************************/
    void CalculateEk(const double Mk, const double ecc, double &Ek);

    /***********************************
     * function: to calculate 真近点角
     * @param Ek     [in]   偏近点角
     * @param ecc    [in]   轨道偏心率
     * @param vk     [out]  真近点角
    ***********************************/
    void Calculatevk(const double Ek, const double ecc, double &vk);

    /*********************************
     * function: ro calculate 升交角距
     * @param vk     [in]   真近点角
     * @param omega  [in]   近地点幅角
     * @param Phik   [out]  近地点角距
    *********************************/
    void CalculatePhik(const double vk, const double omega, double &Phik);

    /****************************************
     * function: to calculate 改正后的升交角距
     * @param Phik [in]     升交角距
     * @param cus  [in]     正弦改正
     * @param cuc  [in]     余弦改正
     * @param uk   [out]    改正后的升交角距
    ****************************************/
    void Calculateuk(const double Phik, const double cus, const double cuc, double &uk);
    
    /**************************************
     * function: to calculate 改正后的向径
     * @param Phik [in]     升交角距
     * @param crs  [in]     正弦改正
     * @param crc  [in]     余弦改正
     * @param rk   [out]    改正后的向径
    **************************************/
    void Calculaterk(const double Phik, const double crs, const double crc, const double sqrtA, const double ecc, const double Ek, double &rk);

    /**************************************
     * function: to calculate 改正后轨道倾角
     * @param Phik [in]     升交角距
     * @param crs  [in]     正弦改正
     * @param crc  [in]     余弦改正
     * @param ik   [out]    改正后轨道倾角
    **************************************/
    void Calculateik(const double Phik, const double cis, const double cic, const double i0, const double idot, const double tk, double &ik);

    /*************************************************
     * function: to calculate position in orbit coor
     * @param rk  [in]       改正后轨道倾角
     * @param uk  [in]       改正后的升交角距
     * @param posi [out]     position in orbit coor
    *************************************************/
    void CalculateOrbPos(const double rk, const double uk, XYZ &posi);

    /**************************************************************
     * function: to calculate 改正后升交点经度
     * @param omega0    [in]    升交点经度
     * @param omegadot  [in]    Rate of Right Ascension
     * @param tk        [in]    Time from ephemeris reference epoch
     * @param toe       [in]    Reference Time Ephemeris
     * @param flag      [in]
     * @param omegak    [out]   改正后升交点经度
    **************************************************************/
    void CalculateOmegak(const double omega0, const double omegadot, const double tk, const double toe, const NavSys flag, double &omegak);

    /********************************************************
     * function: to calculate position in earth coordinate
     * @param OrbPos    [in]    position in orbit coor
     * @param omegak    [in]    改正后升交点经度
     * @param ik        [in]    改正后轨道倾角
     * @param EPos      [out]   position in earth coordinate
    ********************************************************/
    void CalculateEPos(const XYZ OrbPos, const double omegak, const double ik, XYZ &EPos);

    /***************************************************************
     * function: to calculate 改正后升交点经度
     * @param omega0    [in]    升交点经度
     * @param omegadot  [in]    Rate of Right Ascension
     * @param tk        [in]    Time from ephemeris reference epoch
     * @param toe       [in]    Reference Time Ephemeris
     * @param omegak    [out]   改正后升交点经度
    ***************************************************************/
    void CalculateGEOOmegak(const double omega0, const double omegadot, const double tk, const double toe, double &omegak);

    /********************************************************
     * function: to calculate position in earth coordinate
     * @param OrbPos    [in]    position in orbit coor
     * @param omegak    [in]    改正后升交点经度
     * @param ik        [in]    改正后轨道倾角
     * @param EPos      [out]   position in earth coordinate
    ********************************************************/
    void CalculateGEOEPos(const double tk, const double omegak, const double ik, const XYZ OrbPos, int prn, XYZ &Epos);

    /***************************************************
     * function: to get satellite position and velocity
     * @param flag      system flag
     * @return          satellite position and velocity
    ***************************************************/
    Satellite* GetPosAndVel(NavSys flag);

    int solve();
/* calculate result */
protected:
    Satellite Gps[MAXGPSSRN];

    Satellite Bds[MAXBDSSRN];
};


#endif