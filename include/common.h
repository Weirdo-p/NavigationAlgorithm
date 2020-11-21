/******************************************
 * to declare some structures and functions
 * @author XUZHUO WHU
******************************************/
#ifndef _COMMON_H_
#define _COMMON_H_
#define PI                  3.14159265358979323846
#define SECRAD              (180.0 * 3600 / PI)
#define LIGHTSPEED          2.99792458e8

#define GPSMIU              3.986005e14
#define BDSMIU              3.986004418e14

#define GPSROTATIONRATE     7.2921151467e-5
#define BDSROTATIONRATE     7.2921150e-5

#define GPSL1               (10.23e6 * 154)
#define GPSL2               (10.23e6 * 120)
#define BDSB1               1561.098e6
#define BDSB3               1268.52e6

#define MAXGPSSRN           32
#define MAXBDSSRN           64
#define CRC32_POLYNOMIAL    0xEDB88320L

#define MAXBUFFLEN          40480
#define MAXDATALEN          30360

// status code
#define UNKNOWN_ERROR       -1
#define UNSUPPORTED_MSG     -2
#define NOT_ENOUGH_OBS      1
#define FILE_OR_BUFF_END    2
#define CRC_FAILED          3
#define INVALID_PRN         4
#define NOT_INVERTIBLE      5
#define CONFLICT_ID         6
#define OPEN_ERROR          7
#define CONNECTION_FAILED   8

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

typedef Matrix<double, 3, 3>                Matrix3d;
typedef Matrix<double, 3, 1>                Vector3d;
typedef Matrix<double, Dynamic, Dynamic>    MatrixXd;

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
    unsigned short int  Year;
    unsigned short int  Month;
    unsigned short int  Day;
    unsigned short int  Hour;
    unsigned short int  Min;
    double              Sec;

    friend ostream & operator<<(ostream &out, const COMMONTIME UT);
};

/**************************
 * Julian Day structure
 * @param Day      int
 * @param FracDay  double
**************************/
struct MJDTIME
{
    int     Day;
    double  FracDay;

    friend ostream & operator<<(ostream &out, const MJDTIME MJD);
};

/************************************
 * GPS Time structure
 * @param Week ushort
 * @param SOW  Second of Week double
************************************/
struct SATTIME
{
    int     Week;
    double  SOW = -1;

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
 * @param a         长半轴
 * @param b         短半轴
 * @param c         简化书写而使用的符号
 * @param alpha     扁率
 * @param e2        第一偏心率的二次方
 * @param e1_2      第二偏心率的二次方
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
 * to match the Reference Manual(OEM7)
 * for now I only deal with GPS and BDS
****************************************/
enum NavSys{GPS = 0, BDS = 4};

// observation information
struct Obs
{
    SATTIME ObsTime;        // GPS Time
    double  psr[2];         // psedorange for L/B1 L2 B3
    float   psr_sigma[2];   // sigma for psr observations
    double  adr[2];         // phase observations (not used)
    float   adr_sigma[2];   // sigma for adr observations(not used)
    float   dopp[2];        // doppler observations
    float   CNo[2];         // 信噪比
    float   LockTime[2];    // lock time(not used)
    BLH     psrpos;         // user position calculated by receiver

    Obs() { psr[0] = -1; psr[1] = -1; }
};

// ephemris infomation
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

// head infomation
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
 * @param n     Corrected mean motion
 * @param vk    真近点角
 * @param Phik  近地点角距
 * @param Ek    偏近点角
 * @param uk    改正后的升交角距
 * @param ik    [out] 改正后轨道倾角
 * @param rk    [in] 改正后轨道倾角
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

// copy data from socket buffer
struct BUFF
{
    unsigned char   buff[MAXBUFFLEN];   // buffer
    int             pos;                // position for now
    int             len;                // real total data length

    BUFF() { memset(buff, 0, MAXDATALEN); pos = 0; len = 0; }
};


/***********************************************
 * function: calculate distance of two vectors
 * @param a
 * @param b
 * @return distance
************************************************/
double dist(const Vector3d a, const Vector3d b);

/***********************************************
 * function: calculate distance of two points
 * @param a
 * @param b
 * @return distance
***********************************************/
double dist(const XYZ a, const XYZ b);

/*******************************************************************
 * function: write SPP and SPV result to specific file
 * @param result    SPP and SPV result
 * @param path      where it wiil be saved, default in workfolder
 * @return          status code
*******************************************************************/
int WriteToFile(SPPResult result, string path = "./");
// declare
class SatPos;
class SPP;
class ReadDataFromFile;
class ReadDataFromSocket;

int solve(SatPos &satposSolver, SPP &sppSolver, ReadDataFromFile &decoder, ELLIPSOID type, string path = "./");

int solve(SatPos &satposSolver, SPP &sppSolver, ReadDataFromSocket &decoder, ELLIPSOID type, string path = "./");

#endif
