/**************************************
 * @author XUZHUO WHU
 * defination of coordinate transform
 * 2020 09 20
**************************************/

#include "../include/coordinate.h"
#include "../include/common.h"
#include <math.h>
#include "../include/matrix.h"
using namespace std;

// WGS84基椭球参数
const double a_w = 6378137;
const double b_w = 6356752.3142;

// CGCS2000 椭球参数
const double a_c = 6378137;
const double b_c = 6356752.3141;

XYZ::XYZ()
{
    this->X = -1;
    this->Y = -1;
    this->Z = -1;
}

XYZ::XYZ(double X_, double Y_, double Z_)
{
    this->X = X_;
    this->Y = Y_;
    this->Z = Z_;
}

BLH::BLH()
{
    this->B = 0;
    this->L = 0;
    this->H = 0;
}

BLH::BLH(double B_, double L_, double H_)
{
    this->B = B_;
    this->L = L_;
    this->H = H_;
}

ostream & operator<<(ostream &out, const XYZ xyz)
{
    out << "(" <<  setprecision(15) << xyz.X << ", " << xyz.Y << ", " << xyz.Z << ")" << endl;
    return out;
}

ostream & operator<<(ostream &out, const BLH blh)
{
    out << "(" << blh.B << ", " << blh.L << ", " << blh.H << ")" << endl;
    return out;
}

ELLIPSOID::ELLIPSOID()
{
    this->a = a_w;
    this->b = b_w;
    this->c = this->a * this->a / this->b;
    this->alpha = (this->a - this->b) / this->a;
    this->e2 = (this->a * this->a - this->b * this->b) / (this->a * this->a);
    this->e1_2 = (this->a * this->a - this->b * this->b) / (this->b * this->b);
}

ELLIPSOID::ELLIPSOID(double a_, double b_)
{
    this->a = a_;
    this->b = b_;
    this->c = this->a * this->a / this->b;
    this->alpha = (this->a - this->b) / this->a;
    this->e2 = (this->a * this->a - this->b * this->b) / (this->a * this->a);
    this->e1_2 = (this->a * this->a - this->b * this->b) / (this->b * this->b);
}

double Deg2Rad(const double deg)
{
    double sec = deg * 3600.0;
    double rad = sec / SECRAD;
    
    return rad;
}

double Rad2Deg(const double rad)
{
    double sec = rad * SECRAD;
    double deg = sec / 3600.0;

    return deg;
}

bool GetN(const BLH blh, const ELLIPSOID ellipsoid, double &N)
{
    double sinB = sin(Deg2Rad(blh.B));
    double sub = sqrt(1 - ellipsoid.e2 * sinB * sinB);
    if(sub <= 1e-6)
        return false;
    N = ellipsoid.a / sub;

    return true;
}

bool BLH2XYZ(const BLH blh, const ELLIPSOID ellipsoid, XYZ &xyz)
{
    double B_rad = Deg2Rad(blh.B);
    double L_rad = Deg2Rad(blh.L);
    double sinB = sin(B_rad);
    double cosB = cos(B_rad);
    double sinL = sin(L_rad);
    double cosL = cos(L_rad);

    double N;
    bool flag = GetN(blh, ellipsoid, N);
    if(!flag)
        return flag;
    
    xyz.X = (N + blh.H) * cosB * cosL;
    xyz.Y = (N + blh.H) * cosB * sinL;
    xyz.Z = (N * (1 - ellipsoid.e2) + blh.H) * sinB;

    return true;
}

bool XYZ2BLH(XYZ xyz, ELLIPSOID ellipsoid, BLH &blh)
{
    blh.L = atan2(xyz.Y, xyz.X);
    blh.L = Rad2Deg(blh.L);
    // 迭代初值  度为单位
    unsigned short int iteration = 0;
    // double deltaZ = ellipsoid.e2 * xyz.Z;

    // in deg
    double B0 = 1;
    while(iteration != 100)
    {
        // double N;
        // double Dist2 = xyz.X * xyz.X + xyz.Y * xyz.Y;
        // if(Dist2 < 1e-4)
        //     return false;
        // double B_rad = atan((xyz.Z + deltaZ) / sqrt(Dist2));
        // blh.B = Rad2Deg(B_rad);
        // bool flag = GetN(blh, ellipsoid, N);
        // if(!flag)
        //     return false;
        // blh.H = sqrt(Dist2 + pow((xyz.Z + deltaZ), 2)) - N;
        // double sinB = (xyz.Z + deltaZ) / sqrt(Dist2 + pow((xyz.Z + deltaZ), 2));
        // // if(abs())
        // // 暂时不太清楚这里的阈值怎么设置
        // if(abs(N * ellipsoid.e2 * sinB) - deltaZ < 1e-10)
        //     break;
        // deltaZ = N * ellipsoid.e2 * sinB;
        // ++ iteration;
        // double dist = sqrt(xyz.X * xyz.X + xyz.Y * xyz.Y);
        // double t0 = xyz.Z / dist;
        // double ti;
        // if(iteration == 0)
        //     ti = t0;
        // double p = ellipsoid.c * ellipsoid.e2 / dist;
        // double k = 1 + ellipsoid.e1_2;
        // double ti1 = t0 + p * ti / sqrt(k + ti);
        // if(abs(ti - ti1) < 1e-10)
        // {
        //     ti = ti1;
        //     double B_rad = atan(ti);
        //     blh.B = Rad2Deg(B_rad);
            
        //     double N;
        //     GetN(blh, ellipsoid, N);
        //     blh.H = xyz.Z / sin(B_rad) - N * (1 - ellipsoid.e2);
        //     break;
        // }
        // ti = ti1;
        // double B_rad = atan(ti);
        // blh.B = Rad2Deg(B_rad);

        // double N;
        // GetN(blh, ellipsoid, N);
        // blh.H = xyz.Z / sin(B_rad) - N * (1 - ellipsoid.e2);
        // // cout << iteration << endl;
        // iteration ++;
        double sinB = sin(Deg2Rad(B0));
        double W = sqrt(1 - ellipsoid.e2 * sinB * sinB);
        double N = ellipsoid.a / W;
        double H = xyz.Z / sin(Deg2Rad(B0)) - N * (1 - ellipsoid.e2);
        double up = xyz.Z + N * ellipsoid.e2 * sinB;
        double down = sqrt(xyz.X * xyz.X + xyz.Y * xyz.Y);
        blh.B = atan(up / down);
        blh.B = Rad2Deg(blh.B);
        blh.H = H;
        double error = abs(blh.B - B0);
        if(error < 1e-20)
            break;
        B0 = blh.B;
        iteration ++;
    }
    
    return true;
}

bool XYZ2NEU(const XYZ station, const XYZ obj, ELLIPSOID type, XYZ &neu)
{
    BLH station_blh;

    XYZ2BLH(station, type, station_blh);
    // cout << station_blh << endl;
    double B = Deg2Rad(station_blh.B);
    double L = Deg2Rad(station_blh.L);
    Matrix3d rotation;
    rotation(0, 0) = -sin(B) * cos(L);
    rotation(0, 1) = -sin(B) * sin(L);
    rotation(0, 2) = cos(B);

    rotation(1, 0) = -sin(L);
    rotation(1, 1) = cos(L);
    rotation(1, 2) = 0;

    rotation(2, 0) = cos(B) * cos(L);
    rotation(2, 1) = cos(B) * sin(L);
    rotation(2, 2) = sin(B);

    Vector3d vec;
    vec(0, 0) = obj.X - station.X;
    vec(1, 0) = obj.Y - station.Y;
    vec(2, 0) = obj.Z - station.Z;

    Vector3d result = rotation * vec;
    neu.X = result(0, 0);
    neu.Y = result(1, 0);
    neu.Z = result(2, 0);
    vec.deleteMatrix();
    result.deleteMatrix();
    rotation.deleteMatrix();
}