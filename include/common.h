/******************************************
 * to declare some structures and functions
 * @author XUZHUO WHU
******************************************/
#ifndef _COMMON_H_
#define _COMMON_H_
#define PI 3.14159265358979323846
#define SECRAD  (180.0 * 3600 / PI)  // 206265
#define LIGHTSPEED 2.99792458e8

#define GPSMIU 3.986005e14
#define BDSMIU 3.986004418e14

#define GPSROTATIONRATE 7.2921151467e-5
#define BDSROTATIONRATE 7.2921150e-5

#define GPSL1 (10.23e6 * 154)
#define GPSL2 (10.23e6 * 120)
#define BDSB1 1561.098e6
#define BDSB3 1268.52e6

#define MAXGPSSRN 32
#define MAXBDSSRN 64
#define CRC32_POLYNOMIAL 0xEDB88320L

#define MAXBUFFLEN 40480
#define MAXDATALEN 30360

#include <iostream>
#include <iomanip>
#include "matrix.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>


using namespace std;

typedef Matrix<double, 3, 3> Matrix3d;
typedef Matrix<double, 3, 1> Vector3d;
typedef Matrix<double, Dynamic, Dynamic> MatrixXd;

/*******************************************************************************
 ****************************Time Related Structure*****************************
********************************************************************************/

/*************************
 * common time structure
 * @param Year  ushort
 * @param Month ushort
 * @param Day   ushort
 * @param Hour  ushort
 * @param Min   ushort
 * @param Sec   double
*************************/
struct COMMONTIME
{
    unsigned short int Year;
    unsigned short int Month;
    unsigned short int Day;
    unsigned short int Hour;
    unsigned short int Min;
    double Sec;

    friend ostream & operator<<(ostream &out, const COMMONTIME UT);
};

/**************************
 * Julian Day structure
 * @param Day      int
 * @param FracDay  double
**************************/
struct MJDTIME
{
    int Day;
    double FracDay;

    friend ostream & operator<<(ostream &out, const MJDTIME MJD);
};

/************************************
 * GPS Time structure
 * @param Week ushort
 * @param SOW  Second of Week double
************************************/
struct SATTIME
{
    int Week;
    double SOW = -1;

    friend ostream & operator<<(ostream &out, const SATTIME GPST);
    SATTIME operator-(const SATTIME &a) const;
    SATTIME operator-(const double &a) const;
    SATTIME operator+(const double &a) const;

};

/***************************
 * XYZ coordinate structure
 * 默认构造0
 * @param X    double
 * @param Y    double
 * @param Z    double
***************************/
struct XYZ
{
    double X = -1;
    double Y = -1;
    double Z = -1;

    XYZ();

    XYZ(double X_, double Y_, double Z_);

    friend ostream & operator<<(ostream &out, const XYZ xyz);


};

/********************************
 * BLH coordinate structure
 * 默认构造0
 * @param B    double  latitude
 * @param L    double  longitude
 * @param H    double  height
********************************/
struct BLH
{
    double B;
    double L;
    double H;

    BLH();

    BLH(double B_, double L_, double H_);

    friend ostream & operator<<(ostream &out, const BLH blh);
};

/*********************************
 * Ellipsoid parameters
 * 默认使用WGS84参数
 * @param a  长半轴
 * @param b  短半轴
 * @param c  简化书写而使用的符号
 * @param alpha  扁率
 * @param e2     第一偏心率的二次方
 * @param e1_2   第二偏心率的二次方
*********************************/
struct ELLIPSOID
{
    double a;
    double b;
    double c;
    double alpha;
    double e2;
    double e1_2;

    ELLIPSOID();
    
    ELLIPSOID(double a_, double b_);
};

/****************************************
 * to describe navigation system
 * GPS = 0, BDS = 4
 * to match the Reference Manual
 * for now I only deal with GPS and BDS
****************************************/
enum NavSys{GPS = 0, BDS = 4};

/**********************************************
 * store observation data
 * @param psr for GPS, psr[0] stores L1
 *            psr[1] stores L2. For BDS, 
 *            psr[0] stores B2, psr[1] stores
 *            B3. same as other lists.
 * @param CNo carrier to noise density ratio
 * @param LockTime [0] is the flag param
 *                 if it is -1, it means we do 
 *                 not have its Obs
**********************************************/
struct Obs
{
    SATTIME ObsTime;
    double psr[2];
    float psr_sigma[2];
    double adr[2];
    float adr_sigma[2];
    float dopp[2];
    float CNo[2];
    float LockTime[2];
    BLH psrpos;

    Obs() { psr[0] = -1; psr[1] = -1; }
};

/*********************************************
 * store satellite ephemeris
 * @param sys    navigation satellite system
 * @param PRN    satellite SRN
 * @param HealthStatus
 * @param RefTime reference time{week and toe}
 * @param M0     mean anomaly of reference
 * @param omega  Argument of Perigee(ridians)
 * @param cuc  
 * @param cus
 * @param crc
 * @param crs
 * @param cic
 * @param crs
 * @param I0
 * @param I0Rate
 * @param deltaN
 * @param ecc    eccentricty
 * @param omegaO
 * @param OmegaORate
 * @param iodc
 * @param toc
 * @param tgd
 * @param URA   the flag param----if it is -1,
 *              that means we do not have this
 *              sat's eph
*********************************************/

// typedef __uint64_t      Ulong;
// typedef __uint32_t      Ushort;
// typedef double          Double;
// typedef __int64_t       Bool;
// typedef unsigned char   Uchar;

// typedef struct GPSEPHEM_HDR {
//     Uchar   sync1;
//     Uchar   sync2;
//     Uchar   sync3;
//     Uchar   hdr_len;
//     Ushort  msg_id;
//     char    msg_type;
// };

struct Ephemeris
{
    NavSys          sys;
    unsigned short  PRN;
    int             HealthStatus;
    SATTIME         RefTime;
    double          deltaN;
    double          M0;
    double          ecc, sqrtA;
    double          w;
    double          cuc, cus;
    double          crc, crs;
    double          cic, cis;
    double          I0, I0Rate;
    double          omegaO, omegaORate;
    unsigned long   iodc;
    double          toc;
    double          tgd, tgd2;
    double          af0, af1, af2;
    double          URA = -1;
};

struct FileHead
{
    int     MessageID;
    int     MessageLength;
    int     TimeStatus;
    SATTIME GPST;
    int     HealthFlag; // 0 for healthy 1 for unhealthy
};

/******************************************
 * store satellites' position, velocity,
 * and useful values calculated during 
 * calculating the satellites' position
 * @param n  Corrected mean motion
 * @param vk 真近点角
 * @param Phik 近地点角距
 * @param Ek 偏近点角
 * @param uk 改正后的升交角距
 * @param ik   [out] 改正后轨道倾角
 * @param rk  [in] 改正后轨道倾角
*****************************************/
struct Satellite
{
    XYZ    SatPosition;
    XYZ    SatVelocity;
    XYZ    GK;
    double vk, phik, Ek, tk;
    double uk, rk, ik, Omegak;
    double clkdif = 0, n = -1;
    double deltat, t;
};

struct SPPResult
{
    SATTIME ObsTime;
    XYZ     UserPositionXYZ;
    BLH     UserPositionBLH;
    XYZ     UserRefPositionXYZ;
    XYZ     UserVelocity;
    XYZ     BDSPosi[MAXBDSSRN];
    XYZ     GPSPosi[MAXGPSSRN];
    XYZ     diffNeu;
    int     GPSObsNum = 0;
    int     BDSObsNum = 0;
    double  GPSR = -1;
    double  BDSR = -1;
    double  GPSDist[MAXGPSSRN];
    double  BDSDist[MAXBDSSRN];
    double  GPSelev[MAXGPSSRN];
    double  BDSelev[MAXBDSSRN];
    double  GPSAzimuth[MAXGPSSRN];
    double  BDSAzimuth[MAXBDSSRN];
    double  UserPositionSigma;
    double  UserVelocitySigma;
    double  PDop;

    SPPResult();
};

struct BUFF
{
    unsigned char buff[MAXBUFFLEN];
    int pos;
    int len;

    BUFF() { memset(buff, 0, MAXDATALEN); pos = 0; len = 0; }
};


/***********************************************
 * function: calculate distance of two vectors
 * @param a
 * @param b
 * @return distance
************************************************/
double dist(const Vector3d a, const Vector3d b);

double dist(const XYZ a, const XYZ b);

int WriteToFile(SPPResult result, string path = "./");

#endif
