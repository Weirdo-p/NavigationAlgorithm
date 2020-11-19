#include "spp.h"
#include "common.h"
#include "troposphere.h"
extern const int Dynamic;

/**
 * the format of B is:
 * [ l m n deltaGPSR deltaBDSR ]
*/
int SPP::solveSPP(Satellite* &&GPSPosAndVel, Satellite* &&BDSPosAndVel,
                  Obs* &&GPSObs, Obs* &&BDSObs, Ephemeris* &&GPSEph,
                  Ephemeris* &&BDSEph, ELLIPSOID type)
{
    int GPSObsNum = 0, BDSObsNum = 0;
    int cols = 5;
    for(int i = 0; i < MAXGPSSRN; ++i)
        if(GPSPosAndVel[i].n != -1)
            GPSObsNum ++;
    for(int i = 0; i < MAXBDSSRN; ++i)
        if(BDSPosAndVel[i].n != -1)
            BDSObsNum ++;

    this->result.BDSObsNum = BDSObsNum;
    this->result.GPSObsNum = GPSObsNum;

    if(GPSObsNum == 0 || BDSObsNum == 0)
        cols = 4;
    if(BDSObsNum ==0 && GPSObsNum <= 4)
        return 1;
    if(GPSObsNum == 0 && BDSObsNum <= 4)
        return 1;
    if(GPSObsNum != 0 && BDSObsNum != 0 && GPSObsNum + BDSObsNum < 5)
        return 1;
    
    // 测站位置
    Vector3d RefPos;
    RefPos.Zero();
    // n * m
    MatrixXd B(BDSObsNum + GPSObsNum, cols);
    // n * n
    MatrixXd P(B.row(), B.row());
    // n * 1
    MatrixXd w(B.row(), 1);
    B.Zero();
    P.Zero();
    int count = 0;
    double GPSR = this->result.GPSR, BDSR = this->result.BDSR;
    
    while (count <= 10)
    {
        int num = 0;
        XYZ station;
        for(int prn = 0; prn < MAXGPSSRN; ++prn)
        {
            if(GPSObsNum == 0)
                break;
            if(GPSPosAndVel[prn].n == -1){
                continue;
            }
            this->result.ObsTime = GPSObs[prn].ObsTime;

            // 地球自转改正
            XYZ fix;
            EarthRotationFix(GPSPosAndVel[prn].deltat, GPSPosAndVel[prn].SatPosition, GPS, fix);
            this->result.GPSPosi[prn] = fix;
            Vector3d SatPos;
            SatPos(0, 0) = fix.X;
            SatPos(1, 0) = fix.Y;
            SatPos(2, 0) = fix.Z;
            double distance = dist(RefPos, SatPos);
            double l = (RefPos(0, 0) - SatPos(0, 0)) / distance;
            double m = (RefPos(1, 0) - SatPos(1, 0)) / distance;
            double n = (RefPos(2, 0) - SatPos(2, 0)) / distance;
            this->result.GPSDist[prn] = distance;

            // 对流层改正
            double T = 0;
            XYZ satNEU;
            station.X = RefPos(0, 0);
            station.Y = RefPos(1, 0);
            station.Z = RefPos(2, 0);
            XYZ2NEU(station, fix, type, satNEU);

            double E = this->CalculateEA(satNEU);
            double A = this->CalculateAzi(satNEU);

            this->result.GPSelev[prn] = E;
            this->result.GPSAzimuth[prn] = A;

            T = Hopefield(E, result.UserPositionBLH.H);
            if(result.UserPositionBLH.H > 5e3)
                T = 0;
            double IFpsr = GPSL1 * GPSL1 / (GPSL1 * GPSL1 - GPSL2 * GPSL2) * GPSObs[prn].psr[0] - 
                           GPSL2 * GPSL2 / (GPSL1 * GPSL1 - GPSL2 * GPSL2) * GPSObs[prn].psr[1];
            // w矩阵
            w(num, 0) = IFpsr - distance + GPSPosAndVel[prn].clkdif * LIGHTSPEED - GPSR - T;
            // B矩阵
            B(num, 0) = l;
            B(num, 1) = m;
            B(num, 2) = n;
            B(num, 3) = 1;

            P(num, num) = 1.0;// / pow(GPSObs[prn].psr_sigma[0] + GPSObs[prn].psr_sigma[1], 2);
            num ++;
            SatPos.deleteMatrix();
        }
        // BDS
        for(int prn = 0; prn < MAXBDSSRN; ++prn)
        {
            if(BDSObsNum == 0)
                break;
            if(BDSPosAndVel[prn].n == -1){
                continue;
            }
            // 几何距离
            XYZ fix;
            EarthRotationFix(BDSPosAndVel[prn].deltat, BDSPosAndVel[prn].SatPosition, BDS, fix);
            this->result.BDSPosi[prn] = fix;
            this->result.ObsTime = BDSObs[prn].ObsTime;
            Vector3d SatPos;
            SatPos(0, 0) = fix.X;
            SatPos(1, 0) = fix.Y;
            SatPos(2, 0) = fix.Z;
            double distance = dist(RefPos, SatPos);
            double l = (RefPos(0, 0) - SatPos(0, 0)) / distance;
            double m = (RefPos(1, 0) - SatPos(1, 0)) / distance;
            double n = (RefPos(2, 0) - SatPos(2, 0)) / distance;
            this->result.BDSDist[prn] = distance;

            // 对流层改正
            double T = 0;
            XYZ satNEU;
            station.X = RefPos(0, 0);
            station.Y = RefPos(1, 0);
            station.Z = RefPos(2, 0);
            XYZ2NEU(station, fix, type, satNEU);

            double E = this->CalculateEA(satNEU);
            double A = this->CalculateAzi(satNEU);
            this->result.BDSelev[prn] = E;
            this->result.BDSAzimuth[prn] = A;

            T = Hopefield(E, result.UserPositionBLH.H);
            if(result.UserPositionBLH.H > 5e3)
                T = 0;
            double IFpsr = BDSB1 * BDSB1 / (BDSB1 * BDSB1 - BDSB3 * BDSB3) * BDSObs[prn].psr[0] - 
                           BDSB3 * BDSB3 / (BDSB1 * BDSB1 - BDSB3 * BDSB3) * BDSObs[prn].psr[1];
            
            // w矩阵
            w(num, 0) = IFpsr - distance + BDSPosAndVel[prn].clkdif * LIGHTSPEED - BDSR -
                        LIGHTSPEED * BDSB1 * BDSB1 * BDSEph[prn].tgd / ((BDSB1 * BDSB1 - BDSB3 * BDSB3)) - T;
            // B矩阵
            B(num, 0) = l;
            B(num, 1) = m;
            B(num, 2) = n;
            if(GPSObsNum == 0)
                B(num, 3) = 1;
            else
                B(num, 4) = 1;

            P(num, num) = 1.0;// / pow(BDSObs[prn].psr_sigma[0] + BDSObs[prn].psr_sigma[1], 2);
            num ++;
            SatPos.deleteMatrix();
        }
        int flag = 0;
        MatrixXd B_T = B.transpose();
        MatrixXd tmp1 = B_T * P;
        MatrixXd tmp2 = tmp1 * B;
        MatrixXd tmp3 = tmp2.inverse(flag);
        MatrixXd tmp4 = tmp3 * B_T;
        MatrixXd tmp5 = tmp4 * P;
        MatrixXd v = tmp5 * w;

        if(flag != 0){
            B_T.deleteMatrix();
            tmp1.deleteMatrix();
            tmp2.deleteMatrix();
            tmp3.deleteMatrix();
            tmp4.deleteMatrix();
            tmp5.deleteMatrix();
            P.deleteMatrix();
            w.deleteMatrix();
            RefPos.deleteMatrix();
            B.deleteMatrix();
            v.deleteMatrix();
            return 5;
        }
        RefPos(0, 0) += v(0, 0);
        RefPos(1, 0) += v(1, 0);
        RefPos(2, 0) += v(2, 0);
        GPSR += v(3, 0);
        if(GPSObsNum == 0 && BDSObsNum != 0)
            BDSR += v(3, 0);
        else if(BDSObsNum != 0 && GPSObsNum != 0)
            BDSR += v(4, 0);

        MatrixXd tmp6 = B * v;
        MatrixXd tmp7 = tmp6 + w;
        MatrixXd tmp8 = tmp7.transpose();
        MatrixXd tmp9 = tmp8 * P;
        MatrixXd tmp10 = tmp9 * tmp7;

        int Obsnum = 5;
        if(GPSObsNum ==0 || BDSObsNum ==0)
            Obsnum = 4;
        double sigma = sqrt(tmp10(0, 0) / (BDSObsNum + GPSObsNum - Obsnum));
        double pdop = sqrt(tmp3(0, 0) + tmp3(1, 1) + tmp3(2, 2));

        result.UserPositionSigma = sigma;
        result.PDop = pdop;
        tmp6.deleteMatrix();
        tmp7.deleteMatrix();
        tmp8.deleteMatrix();
        tmp9.deleteMatrix();
        tmp10.deleteMatrix();

        Vector3d dx;
        for(int i = 0; i < dx.row(); i++)
            dx(i, 0) = v(i, 0);
        if(dx.norm() < 1e-5)
        {
            B_T.deleteMatrix();
            tmp1.deleteMatrix();
            tmp2.deleteMatrix();
            tmp3.deleteMatrix();
            tmp4.deleteMatrix();
            tmp5.deleteMatrix();
            dx.deleteMatrix();
            v.deleteMatrix();
            break;        
        }
        count ++;

        B_T.deleteMatrix();
        tmp1.deleteMatrix();
        tmp2.deleteMatrix();
        tmp3.deleteMatrix();
        tmp4.deleteMatrix();
        tmp5.deleteMatrix();
        dx.deleteMatrix();
        v.deleteMatrix();
    }
    this->result.GPSR = GPSR;
    this->result.BDSR = BDSR;
    this->result.UserPositionXYZ.X = RefPos(0, 0);
    this->result.UserPositionXYZ.Y = RefPos(1, 0);
    this->result.UserPositionXYZ.Z = RefPos(2, 0);
    XYZ2BLH(result.UserPositionXYZ, type, result.UserPositionBLH);
    XYZ NEU;
    XYZ2NEU(result.UserRefPositionXYZ, result.UserPositionXYZ, type, NEU);
    result.diffNeu = NEU;

    P.deleteMatrix();
    w.deleteMatrix();
    RefPos.deleteMatrix();
    B.deleteMatrix();
    return 0;
}

int SPP::solveSPV(Satellite* &&GPSPosAndVel, Satellite* &&BDSPosAndVel,
             Obs* &&GPSObs, Obs* &&BDSObs, Ephemeris* &&GPSEph,
             Ephemeris* &&BDSEph, ELLIPSOID type)
{
    int GPSObsNum = 0, BDSObsNum = 0;
    int cols = 4;
    for(int i = 0; i < MAXGPSSRN; ++i)
        if(GPSPosAndVel[i].n != -1 && GPSObs[i].dopp[0] != -1)
            GPSObsNum ++;
    for(int i = 0; i < MAXBDSSRN; ++i)
        if(BDSPosAndVel[i].n != -1 && BDSObs[i].dopp[0] != -1)
            BDSObsNum ++;
    // BDSObsNum = 0;
    XYZ RefPos = this->result.UserPositionXYZ;
    if(RefPos.Z == -1)
        return 1;
    // n * m
    MatrixXd B(BDSObsNum + GPSObsNum, cols);
    // n * n
    MatrixXd P(B.row(), B.row());
    // n * 1
    MatrixXd w(B.row(), 1);
    XYZ RefV(0, 0, 0);
    B.Zero();
    P.Zero();
    int count = 0;
    // cout << GPSObsNum << endl;
    double R = 0;
    while(count < 10)
    {
        int num = 0;
        for(int prn = 0; prn < MAXGPSSRN; ++prn)
        {
            if(GPSObsNum == 0)
                break;
            if(GPSPosAndVel[prn].n == -1 ||
               GPSObs[prn].dopp[0] == -1)
                continue;

            XYZ fix = GPSPosAndVel[prn].SatVelocity;
            // 在SPP时已经自转改正  故不需要再次改正
            XYZ SatPosi = this->result.GPSPosi[prn];

            double rou = this->result.GPSDist[prn];
            double l = (-SatPosi.X + RefPos.X) / rou; 
            double m = (-SatPosi.Y + RefPos.Y) / rou;
            double n = (-SatPosi.Z + RefPos.Z) / rou;

            double deltatdot = GPSEph[prn].af1 + 2 * GPSEph[prn].af2 *
                               (GPSPosAndVel[prn].t - GPSPosAndVel[prn].deltat - GPSEph[prn].toc);

            double roudot = -l * (fix.X) - m * (fix.Y) - n * (fix.Z);

            B(num, 0) = l;
            B(num, 1) = m;
            B(num, 2) = n;
            B(num, 3) = 1;
            w(num, 0) = -GPSObs[prn].dopp[0] * LIGHTSPEED / GPSL1 -
                        roudot + deltatdot * LIGHTSPEED;
            P(num, num) = 1;
            num ++;
        }
        for(int prn = 0; prn < MAXBDSSRN; ++prn)
        {
            if(BDSObsNum == 0)
                break;
            if(BDSPosAndVel[prn].n == -1 ||
               BDSObs[prn].dopp[0] == -1)
                continue;
            // 地球自转改正
            XYZ fix = BDSPosAndVel[prn].SatVelocity;
            // EarthRotationFix(BDSPosAndVel[prn].deltat, BDSPosAndVel[prn].SatVelocity, BDS, fix);
            // 在SPP时已经自转改正  故不需要再次改正
            XYZ SatPosi = this->result.BDSPosi[prn];
            double rou = this->result.BDSDist[prn];
            double l = (-SatPosi.X + RefPos.X) / rou; 
            double m = (-SatPosi.Y + RefPos.Y) / rou;
            double n = (-SatPosi.Z + RefPos.Z) / rou;

            double deltatdot = BDSEph[prn].af1 + 2 * BDSEph[prn].af2 *
                               (BDSPosAndVel[prn].t - BDSPosAndVel[prn].deltat - BDSEph[prn].toc);

            double roudot = -l * (fix.X) - m * (fix.Y) - n * (fix.Z);

            B(num, 0) = l;
            B(num, 1) = m;
            B(num, 2) = n;
            B(num, 3) = 1;
            w(num, 0) = -BDSObs[prn].dopp[0] * LIGHTSPEED / BDSB1 -
                        roudot + deltatdot * LIGHTSPEED;
            P(num, num) = 1;
            num ++;
        }
        int flag = 0;
        MatrixXd B_T = B.transpose();
        MatrixXd tmp1 = B_T * P;
        MatrixXd tmp2 = tmp1 * B;
        MatrixXd tmp3 = tmp2.inverse(flag);
        MatrixXd tmp4 = tmp3 * B_T;
        MatrixXd tmp5 = tmp4 * P;
        MatrixXd v = tmp5 * w;

        if(flag != 0){
            B_T.deleteMatrix();
            tmp1.deleteMatrix();
            tmp2.deleteMatrix();
            tmp3.deleteMatrix();
            tmp4.deleteMatrix();
            tmp5.deleteMatrix();
            P.deleteMatrix();
            w.deleteMatrix();
            B.deleteMatrix();
            v.deleteMatrix();
            return 5;
        }
        RefV.X += v(0, 0);
        RefV.Y += v(1, 0);
        RefV.Z += v(2, 0);
        
        MatrixXd tmp6 = B * v;
        MatrixXd tmp7 = tmp6 - w;
        MatrixXd tmp8 = tmp7.transpose();
        MatrixXd tmp9 = tmp8 * P;
        MatrixXd tmp10 = tmp9 * tmp7;
        int Obsnum = 4;
        double sigma = sqrt(tmp10(0, 0) / (BDSObsNum + GPSObsNum - Obsnum));
        result.UserVelocitySigma = sigma;

        tmp6.deleteMatrix();
        tmp7.deleteMatrix();
        tmp8.deleteMatrix();
        tmp9.deleteMatrix();
        tmp10.deleteMatrix();

        Vector3d dx;
        for(int i = 0; i < dx.row(); i++)
            dx(i, 0) = v(i, 0);
        if(dx.norm() < 1e-5)
        {
            dx.deleteMatrix();
            B_T.deleteMatrix();
            tmp1.deleteMatrix();
            tmp2.deleteMatrix();
            tmp3.deleteMatrix();
            tmp4.deleteMatrix();
            tmp5.deleteMatrix();
            v.deleteMatrix();
            break;        
        }
        count ++;
        dx.deleteMatrix();
        B_T.deleteMatrix();
        tmp1.deleteMatrix();
        tmp2.deleteMatrix();
        tmp3.deleteMatrix();
        tmp4.deleteMatrix();
        tmp5.deleteMatrix();
        v.deleteMatrix();
    }
    this->result.UserVelocity = RefV;
    B.deleteMatrix();
    P.deleteMatrix();
    w.deleteMatrix();
}


double dist(const Vector3d a, const Vector3d b)
{
    double X2 = pow((a(0, 0) - b(0, 0)), 2);
    double Y2 = pow((a(1, 0) - b(1, 0)), 2);
    double Z2 = pow((a(2, 0) - b(2, 0)), 2);
    return (sqrt(X2 + Y2 + Z2));
}

double dist(const XYZ a, const XYZ b)
{
    double X2 = pow((a.X - b.X), 2);
    double Y2 = pow((a.Y - b.Y), 2);
    double Z2 = pow((a.Z - b.Z), 2);
    return (sqrt(X2 + Y2 + Z2));
}

double SPP::CalculateEA(const XYZ neu)
{
    return atan(neu.Z / sqrt(neu.X * neu.X + neu.Y * neu.Y));
}

double SPP::CalculateAzi(const XYZ neu)
{
    return atan2(neu.X, neu.Y);
}

void SPP::EarthRotationFix(double deltat, XYZ SatPosition, NavSys flag, XYZ &fixed)
{
    Vector3d pos;
    Matrix3d R;
    double rate;
    if(flag == GPS)
        rate = GPSROTATIONRATE;
    if(flag == BDS)
        rate = BDSROTATIONRATE;
    // GPS
    pos(0, 0) = SatPosition.X;
    pos(1, 0) = SatPosition.Y;
    pos(2, 0) = SatPosition.Z;

    R(0, 0) = cos(deltat * rate);
    R(0, 1) = sin(deltat * rate);
    R(0, 2) = 0;

    R(1, 0) = -sin(deltat * rate);
    R(1, 1) = cos(deltat * rate);
    R(1, 2) = 0;

    R(2, 0) = 0;
    R(2, 1) = 0;
    R(2, 2) = 1;

    Vector3d result = R * pos;
    fixed.X = result(0, 0);
    fixed.Y = result(1, 0);
    fixed.Z = result(2, 0);

    result.deleteMatrix();
    pos.deleteMatrix();
    R.deleteMatrix();
}

SPPResult SPP::GetResult()
{
    return this->result;
}

int WriteToFile(SPPResult result, string path){
    ifstream in;
    string fix = "result.txt";
    in.open(path + fix);
    bool isdata = false;
    if(in.eof() || !in)
        isdata = true;
    in.close();

    ofstream out;
    out.open(path + fix, ios::app);
    if(!out){
        cout << "error happened when writing file" << endl;
        return 1;
    }
    out.right;
    if(isdata)
        out << "Week      SOW         ECEF/X-m      ECEF/Y-m      ECEF/Z-m     REF-ECEF/X-m  REF-ECEF/Y-m    ECEF/Z-m      EAST/m  NORTH/m  UP/m    B/deg     L/deg     H/m      VX-m/s  VY-m/s  VZ-m/s     PDOP  Sigma-m  Sigma-m/s    BDSObsNum     GPSObsNum" << endl;

    out << fixed << setprecision(4);
    out << result.ObsTime.Week << "  " << result.ObsTime.SOW << "  ";
    out << result.UserPositionXYZ.X << "  " << result.UserPositionXYZ.Y << "  " << result.UserPositionXYZ.Z << "  ";
    out << result.UserRefPositionXYZ.X << "  " << result.UserRefPositionXYZ.Y << "  " << result.UserRefPositionXYZ.Z << "  ";
    out << setw(8) << result.diffNeu.X << setw(8) << result.diffNeu.Y << setw(8) << result.diffNeu.Z << "  ";
    out << result.UserPositionBLH.B << "  " << result.UserPositionBLH.L << "  " << result.UserPositionBLH.H << "  ";
    out << setw(8) << result.UserVelocity.X << setw(8) << result.UserVelocity.Y << setw(8) << result.UserVelocity.Z << "  ";
    out << setw(8) << result.PDop << setw(8) << result.UserPositionSigma << setw(8) << result.UserVelocitySigma << "  ";
    out << setw(10) << result.BDSObsNum << "  " << setw(10) << result.GPSObsNum << "       ";
    for(int prn = 0; prn < MAXBDSSRN; ++ prn) {
        if(result.BDSDist[prn] != 0 && result.BDSPosi[prn].Z != -1)
            out << "C" << setw(2) << setfill('0') << prn + 1;
    }
    for(int prn = 0; prn < MAXGPSSRN; ++ prn) {
        if(result.GPSDist[prn] != 0 && result.GPSPosi[prn].Z != -1)
            out << "G" << setw(2) << setfill('0') << prn + 1;
    }
    out << endl;
    out.close();

    fix = "sat.txt";
    out.open(path + fix, ios::app);
    out.right;
    out << fixed << setprecision(4);
    out << result.ObsTime.Week << "  " << result.ObsTime.SOW << "  ";
    out << setw(3) << result.BDSObsNum << "  " << setw(3) << result.GPSObsNum << endl;
    for(int prn = 0; prn < MAXBDSSRN; ++ prn) {
        if(result.BDSDist[prn] != 0 && result.BDSPosi[prn].Z != -1) {
            out << "C" << setw(2) << setfill('0') << prn + 1 << " ";
            out << setfill(' ');
            out << fixed << setprecision(4);
            out << setw(15) << result.BDSPosi[prn].X << "  " << setw(15) << result.BDSPosi[prn].Y << "  " << setw(15) << result.BDSPosi[prn].Z << "  ";
            out << fixed << setprecision(10);
            out << setw(15) << result.BDSAzimuth[prn] << "  " << setw(15) << result.BDSelev[prn] << endl;
        }
    }
    for(int prn = 0; prn < MAXGPSSRN; ++ prn) {
        if(result.GPSDist[prn] != 0 && result.GPSPosi[prn].Z != -1) {
            out << "G" << setw(2) << setfill('0') << prn + 1 << " ";
            out << setfill(' ');
            out << fixed << setprecision(4);
            out << setw(15) << result.GPSPosi[prn].X << "  " << setw(15) << result.GPSPosi[prn].Y << "  " << setw(15) << result.GPSPosi[prn].Z << "  ";
            out << fixed << setprecision(10);
            out << setw(15) << result.GPSAzimuth[prn] << "  " << setw(15) << result.GPSelev[prn] << endl;
        }
    }
}

void SPP::setRefPos(XYZ Ref) {
    this->result.UserRefPositionXYZ = Ref;
}

SPPResult::SPPResult() {
    memset(GPSDist, 0, MAXGPSSRN);
    memset(BDSDist, 0, MAXBDSSRN);
    memset(GPSAzimuth, 0, MAXGPSSRN);
    memset(BDSAzimuth, 0, MAXBDSSRN);
    memset(GPSelev, 0, MAXGPSSRN);
    memset(BDSelev, 0, MAXBDSSRN);
}