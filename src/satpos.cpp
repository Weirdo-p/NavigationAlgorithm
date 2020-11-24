#include "satpos.h"
#include <fstream>
#include <iomanip>

int SatPos::CalculateGPSPos(Ephemeris* &&GPSEph, Obs* &&obs, const SPPResult result)
{
    for(int prn = 0; prn < MAXGPSSRN; ++prn)
    {
        // 仅处理有星历和双频观测值的卫星
        if(GPSEph[prn].URA == -1){
            continue;
        }
        // 本次实习仅处理双频观测
        if(obs[prn].psr[0] == -1 || obs[prn].psr[1] == -1){
            continue;
        }
        try
        {
            SATTIME t;
            Calculatet(obs, GPSEph, prn, GPS, result, t);
            
            // 平均运动角速度
            double n0;
            Calculaten0(GPSEph[prn].sqrtA, GPS, n0);

            // 计算相对于星历的参考时间
            bool flag = Calculatetk(t, GPSEph[prn].RefTime, GPS, Gps[prn].tk);
            if(!flag)
                continue;
            this->Gps[prn].deltat = (obs[prn].ObsTime.SOW - t.SOW);
            this->Gps[prn].t = t.SOW;
            // 对平均运动角速度的改正
            Calculaten(n0, GPSEph[prn].deltaN, Gps[prn].n);

            // 平近点角
            double Mk;
            CalculateMk(GPSEph[prn].M0, Gps[prn].tk, Gps[prn].n, Mk);

            // 偏近点角
            CalculateEk(Mk, GPSEph[prn].ecc, Gps[prn].Ek);

            // 真近点角
            Calculatevk(Gps[prn].Ek, GPSEph[prn].ecc, Gps[prn].vk);

            // 近地点角距
            CalculatePhik(Gps[prn].vk, GPSEph[prn].w, Gps[prn].phik);

            // 改正后的升交角距
            Calculateuk(Gps[prn].phik, GPSEph[prn].cus, GPSEph[prn].cuc, Gps[prn].uk);
            

            // 改正后的向径
            Calculaterk(Gps[prn].phik, GPSEph[prn].crs, GPSEph[prn].crc,
                        GPSEph[prn].sqrtA, GPSEph[prn].ecc, Gps[prn].Ek, Gps[prn].rk);

            Calculateik(Gps[prn].phik, GPSEph[prn].cis, GPSEph[prn].cic,
                        GPSEph[prn].I0, GPSEph[prn].I0Rate, Gps[prn].tk, Gps[prn].ik);

            XYZ OrbPosi;
            CalculateOrbPos(Gps[prn].rk, Gps[prn].uk, OrbPosi);

            CalculateOmegak(GPSEph[prn].omegaO, GPSEph[prn].omegaORate,
                            Gps[prn].tk, GPSEph[prn].RefTime.SOW, GPS,
                            Gps[prn].Omegak);

            CalculateEPos(OrbPosi, Gps[prn].Omegak,
                          Gps[prn].ik, Gps[prn].SatPosition);
        }
        catch(...)
        {
            cout << "error happened when calculating GPS sat pos" << endl;
            return UNKNOWN_ERROR;
        }
    }
    return 0;
}

int SatPos::CalculateBDSPos(Ephemeris* && BDSEph, Obs* &&obs, const SPPResult result)
{
    for(int prn = 0; prn < MAXBDSSRN; ++prn)
    {
        if(BDSEph[prn].URA == -1)
            continue;
        if(obs[prn].psr[0] == -1 ||
           obs[prn].psr[1] == -1)
            continue;

        try
        {
            SATTIME t;
            Calculatet(obs, BDSEph, prn, BDS, result, t);
            
            // 平均运动角速度
            double n0;
            Calculaten0(BDSEph[prn].sqrtA, BDS, n0);

            // 计算相对于星历的参考时间
            bool flag = Calculatetk(t, BDSEph[prn].RefTime, BDS, Bds[prn].tk);
            if(!flag)
                continue;
            this->Bds[prn].deltat = (obs[prn].ObsTime.SOW - t.SOW);
            this->Bds[prn].t = t.SOW;
            // 对平均运动角速度的改正
            Calculaten(n0, BDSEph[prn].deltaN, Bds[prn].n);

            // 平近点角
            double Mk;
            CalculateMk(BDSEph[prn].M0, Bds[prn].tk, Bds[prn].n, Mk);

            // 偏近点角
            CalculateEk(Mk, BDSEph[prn].ecc, Bds[prn].Ek);

            // 真近点角
            Calculatevk(Bds[prn].Ek, BDSEph[prn].ecc, Bds[prn].vk);

            // 近地点角距
            CalculatePhik(Bds[prn].vk, BDSEph[prn].w, Bds[prn].phik);

            // 改正后的升交角距
            Calculateuk(Bds[prn].phik, BDSEph[prn].cus, BDSEph[prn].cuc, Bds[prn].uk);
            
            // 改正后的向径
            Calculaterk(Bds[prn].phik, BDSEph[prn].crs, BDSEph[prn].crc,
                        BDSEph[prn].sqrtA, BDSEph[prn].ecc, Bds[prn].Ek, Bds[prn].rk);

            Calculateik(Bds[prn].phik, BDSEph[prn].cis, BDSEph[prn].cic,
                        BDSEph[prn].I0, BDSEph[prn].I0Rate, Bds[prn].tk, Bds[prn].ik);

            XYZ OrbPosi;
            CalculateOrbPos(Bds[prn].rk, Bds[prn].uk, OrbPosi);
            // if it is GEO
            if(prn <= 4 || prn >= 58)
            {
                CalculateGEOOmegak(BDSEph[prn].omegaO, BDSEph[prn].omegaORate,
                                   Bds[prn].tk, BDSEph[prn].RefTime.SOW,
                                   Bds[prn].Omegak);
                CalculateGEOEPos(Bds[prn].tk, Bds[prn].Omegak,
                                 Bds[prn].ik, OrbPosi, prn, Bds[prn].SatPosition);
            }
            // if it is IGSO/MEO
            else
            {
                CalculateOmegak(BDSEph[prn].omegaO, BDSEph[prn].omegaORate,
                                Bds[prn].tk, BDSEph[prn].RefTime.SOW, BDS,
                                Bds[prn].Omegak);
                CalculateEPos(OrbPosi, Bds[prn].Omegak,
                              Bds[prn].ik, Bds[prn].SatPosition);
            }
        }
        catch(...)
        {
            cout << "error happened while calculating BDS sat pos" << endl;
            return UNKNOWN_ERROR;
        }
        
    }
    return 0;
}

void SatPos::Calculaten0(const double sqrtA, NavSys flag, double &n0)
{
    double A = sqrtA * sqrtA;
    if(flag == GPS)
        n0 = sqrt(GPSMIU) / sqrt(pow(A, 3));
    if(flag == BDS)
        n0 = sqrt(BDSMIU) / sqrt(pow(A, 3));
}

bool SatPos::Calculatetk(const SATTIME ObsTime, const SATTIME RefTime, NavSys flag, double &tk)
{
    SATTIME RefTime_GPST;
    if(flag == BDS)
        BDST2GPST(RefTime, RefTime_GPST);
    else if(flag == GPS)
        RefTime_GPST = RefTime;

    SATTIME tk_GPST = ObsTime - RefTime_GPST;
    tk = tk_GPST.Week * 604800 + tk_GPST.SOW;
    // 星历有效期
    double expired;
    if(flag == GPS)
        expired = 2 * 3600;
    else if(flag == BDS)
        expired = 3600;
    else
        expired = 0;
    
    if(abs(tk) > expired)
        return false;
    return true;
}

void SatPos::Calculaten(const double n0, const double deltan, double &n)
{
    n = deltan + n0;
}

void SatPos::CalculateMk(const double M0, const double tk, const double n, double &Mk)
{
    Mk = M0 + n * tk;
}

void SatPos::CalculateEk(const double Mk, const double ecc, double &Ek)
{
    double Ek0 = 1;
    int count = 0;
    while (count <= 10)
    {
        Ek = Mk + ecc * sin(Ek0);
        double error = abs(Ek - Ek0);
        if(error < 1e-7)
            break;
        Ek0 = Ek;
        count ++;
    }
}

void SatPos::Calculatevk(const double Ek, const double ecc, double &vk)
{
    double e2 = ecc * ecc;
    double up = sqrt(1 - e2) * sin(Ek);
    double down = cos(Ek) - ecc;
    vk =  atan2(up, down);
}

void SatPos::CalculatePhik(const double vk, const double omega, double &Phik)
{
    Phik = vk + omega;
}

void SatPos::Calculateuk(const double Phik, const double cus, const double cuc, double &uk)
{
    double Phik2 = Phik * 2;
    double deltauk = cus * sin(Phik2) + cuc * cos(Phik2);
    uk = Phik + deltauk;
}

void SatPos::Calculaterk(const double Phik, const double crs, const double crc, const double sqrtA, const double ecc, const double Ek, double &rk)
{
    double Phik2 = Phik * 2;
    double deltark = crs * sin(Phik2) + crc * cos(Phik2);
    rk = sqrtA * sqrtA * (1 - ecc * cos(Ek)) + deltark;
}

void SatPos::Calculateik(const double Phik, const double cis, const double cic, const double i0, const double idot, const double tk, double &ik)
{
    double Phik2 = Phik * 2;
    double deltaik = cis * sin(Phik2) + cic * cos(Phik2);
    ik = i0 + deltaik + idot * tk;
}

void SatPos::CalculateOrbPos(const double rk, const double uk, XYZ &posi)
{
    posi.Z = 0;
    posi.X = rk * cos(uk);
    posi.Y = rk * sin(uk);
}

void SatPos::CalculateOmegak(const double omega0, const double omegadot, const double tk, const double toe, const NavSys flag, double &omegak)
{
    if(flag == GPS)
        omegak = omega0 + (omegadot - GPSROTATIONRATE) * tk - GPSROTATIONRATE * toe;
    else if(flag == BDS)
        omegak = omega0 + (omegadot - BDSROTATIONRATE) * tk - BDSROTATIONRATE * toe;
}

void SatPos::CalculateEPos(const XYZ OrbPos, const double omegak, const double ik, XYZ &EPos)
{
    EPos.X = OrbPos.X * cos(omegak) - OrbPos.Y * cos(ik) * sin(omegak);
    EPos.Y = OrbPos.X * sin(omegak) + OrbPos.Y * cos(ik) * cos(omegak);
    EPos.Z = OrbPos.Y * sin(ik);
}

void SatPos::CalculateGEOOmegak(const double omega0, const double omegadot, const double tk, const double toe, double &omegak)
{
    omegak = omega0 + omegadot * tk - BDSROTATIONRATE * toe;
}

void SatPos::CalculateGEOEPos(const double tk, const double omegak, const double ik, const XYZ OrbPos, int prn, XYZ &Epos)
{
    CalculateEPos(OrbPos, omegak, ik, Epos);
    this->Bds[prn].GK = Epos;
    Matrix<double, 3, 1> GK;
    GK(0, 0) = Epos.X;
    GK(1, 0) = Epos.Y;
    GK(2, 0) = Epos.Z;

    double Zaxis = BDSROTATIONRATE * tk;
    double Xaxis = Deg2Rad(-5.0);

    Matrix<double, 3, 3> Rx;
    Rx(0, 0) = 1;
    Rx(0, 1) = 0;
    Rx(0, 2) = 0;

    Rx(1, 0) = 0;
    Rx(1, 1) = cos(Xaxis);
    Rx(1, 2) = sin(Xaxis);

    Rx(2, 0) = 0;
    Rx(2, 1) = -sin(Xaxis);
    Rx(2, 2) = cos(Xaxis);

    Matrix<double, 3, 3> Rz;
    Rz(0, 0) = cos(Zaxis);
    Rz(0, 1) = sin(Zaxis);
    Rz(0, 2) = 0;

    Rz(1, 0) = -sin(Zaxis);
    Rz(1, 1) = cos(Zaxis);
    Rz(1, 2) = 0;

    Rz(2, 0) = 0;
    Rz(2, 1) = 0;
    Rz(2, 2) = 1;

    Matrix3d tmp = Rz * Rx;
    Matrix<double, 3, 1> K = tmp * GK;

    Epos.X = K(0, 0);
    Epos.Y = K(1, 0);
    Epos.Z = K(2, 0);

    // cout << Epos << endl;
    tmp.deleteMatrix();
    K.deleteMatrix();
    Rz.deleteMatrix();
    Rx.deleteMatrix();
    GK.deleteMatrix();

}

int SatPos::CalculateSatVel(Ephemeris* &&Eph, const NavSys flag, SPPResult result)
{
    int maxprn;
    double RotationRate;
    Satellite* param;

    if(flag == GPS)
    {
        maxprn = MAXGPSSRN;
        RotationRate = GPSROTATIONRATE;
        param = this->Gps;
    }
    else if(flag == BDS)
    {
        maxprn = MAXBDSSRN;
        RotationRate = BDSROTATIONRATE;
        param = this->Bds;
    }
    else
        return UNSUPPORTED_MSG;

    for(int prn = 0; prn < maxprn; ++prn)
    {
        if(param[prn].n == -1)
            continue;

        try
        {   
            // temp
            double phik = param[prn].phik;
            double Ekdot = param[prn].n / (1 - Eph[prn].ecc * cos(param[prn].Ek));
            if(flag == GPS)
                this->Gps[prn].Ekdot = Ekdot;
            if(flag == BDS)
                this->Bds[prn].Ekdot = Ekdot;

            // temp param, to calculate long equation
            double left = sqrt((1 + Eph[prn].ecc) / (1 - Eph[prn].ecc));
            double right = cos(param[prn].vk / 2.0) / cos(param[prn].Ek / 2.0);
            double Phikdot = left * right * right * Ekdot;

            left = Eph[prn].cus * cos(2 * phik) - Eph[prn].cuc * sin(2 * phik);
            double ukdot = 2 * left * Phikdot + Phikdot;

            left = Eph[prn].sqrtA * Eph[prn].sqrtA * Eph[prn].ecc * sin(param[prn].Ek) * Ekdot;
            right = 2 * (Eph[prn].crs * cos(2 * phik) - Eph[prn].crc * sin(2 * phik));
            double rkdot = left + right * Phikdot;

            right = 2 * (Eph[prn].cis * cos(2 * phik) - Eph[prn].cic * sin(2 * phik));
            double Ikdot = Eph[prn].I0Rate + right * Phikdot;

            double Omegakdot = Eph[prn].omegaORate - RotationRate;

            if(flag == BDS && (prn < 5 || prn >= 58))
                Omegakdot = Eph[prn].omegaORate;
            
            double XK = param[prn].rk * cos(param[prn].uk);
            double YK = param[prn].rk * sin(param[prn].uk);
            double XKdot = rkdot * cos(param[prn].uk) - param[prn].rk * ukdot * sin(param[prn].uk);
            double YKdot = rkdot * sin(param[prn].uk) + param[prn].rk * ukdot * cos(param[prn].uk);

            // temp
            double Omegak = param[prn].Omegak;
            double ik = param[prn].ik;

            Matrix<double, 3, 4> Rdot;
            Rdot(0, 0) = cos(Omegak);
            Rdot(0, 1) = -sin(Omegak) * cos(ik);
            Rdot(0, 2) = -(XK * sin(Omegak) + YK * cos(Omegak) * cos(ik));
            Rdot(0, 3) = YK * sin(Omegak) * sin(ik);

            Rdot(1, 0) = sin(Omegak);
            Rdot(1, 1) = cos(Omegak) * cos(ik);
            Rdot(1, 2) = XK * cos(Omegak) - YK * sin(Omegak) * cos(ik);
            Rdot(1, 3) = YK * cos(Omegak) * sin(ik);

            Rdot(2, 0) = 0;
            Rdot(2, 1) = sin(ik);
            Rdot(2, 2) = 0;
            Rdot(2, 3) = YK * cos(ik);
            
            Matrix<double, 4, 1> p;
            p(0, 0) = XKdot;
            p(1, 0) = YKdot;
            p(2, 0) = Omegakdot;
            p(3, 0) = Ikdot;

            Vector3d velocity = Rdot * p;
            if(flag == GPS)
            {
                // 有些累赘  其实可以不用XYZ来存储
                // 可以全部存储为Matrix数据
                Gps[prn].SatVelocity.X = velocity(0, 0);
                Gps[prn].SatVelocity.Y = velocity(1, 0);
                Gps[prn].SatVelocity.Z = velocity(2, 0);
            }
            if(flag == BDS && (prn <5 || prn >= 58))
            {
                CalculateGEOVel(param[prn].tk, Bds[prn].GK, prn, velocity, result);
            }
            if(flag == BDS && (prn >= 5 && prn < 58))
            {
                Bds[prn].SatVelocity.X = velocity(0, 0);
                Bds[prn].SatVelocity.Y = velocity(1, 0);
                Bds[prn].SatVelocity.Z = velocity(2, 0);
            }
            // cout << Bds[prn].SatPosition << endl;

            velocity.deleteMatrix();
            p.deleteMatrix();
            Rdot.deleteMatrix();
        }
        catch(...)
        {
            cout << "error happened when calculate satellite velocity" << endl;
            return UNKNOWN_ERROR;
        }
        
    }
    return 0;
}

/****************
 * fix some bugs
 * 2020 10 20
****************/
int SatPos::CalculateGEOVel(const double tk, const XYZ SatPosGK, const int prn, const Matrix<double, 3, 1> velocity, SPPResult result)
{
    try
    {
        Vector3d GK;
        GK(0, 0) = SatPosGK.X;
        GK(1, 0) = SatPosGK.Y;
        GK(2, 0) = SatPosGK.Z;
        // GK(0, 0) = result.BDSPosi[prn].X;
        // GK(1, 0) = result.BDSPosi[prn].Y;
        // GK(2, 0) = result.BDSPosi[prn].Z;

        double Zaxis = BDSROTATIONRATE * tk;
        double Xaxis = Deg2Rad(-5.0);

        Matrix<double, 3, 3> Rx;
        Rx(0, 0) = 1;
        Rx(0, 1) = 0;
        Rx(0, 2) = 0;

        Rx(1, 0) = 0;
        Rx(1, 1) = cos(Xaxis);
        Rx(1, 2) = sin(Xaxis);

        Rx(2, 0) = 0;
        Rx(2, 1) = -sin(Xaxis);
        Rx(2, 2) = cos(Xaxis);

        Matrix<double, 3, 3> Rz;
        Rz(0, 0) = cos(Zaxis);
        Rz(0, 1) = sin(Zaxis);
        Rz(0, 2) = 0;

        Rz(1, 0) = -sin(Zaxis);
        Rz(1, 1) = cos(Zaxis);
        Rz(1, 2) = 0;

        Rz(2, 0) = 0;
        Rz(2, 1) = 0;
        Rz(2, 2) = 1;

        Matrix3d Rzdot;
        Rzdot(0, 0) = -sin(Zaxis) * BDSROTATIONRATE;
        Rzdot(0, 1) = cos(Zaxis) * BDSROTATIONRATE;
        Rzdot(0, 2) = 0;

        Rzdot(1, 0) = -cos(Zaxis) * BDSROTATIONRATE;
        Rzdot(1, 1) = -sin(Zaxis) * BDSROTATIONRATE;
        Rzdot(1, 2) = 0;

        Rzdot(2, 0) = 0;
        Rzdot(2, 1) = 0;
        Rzdot(2, 2) = 0;

        Matrix3d tmp1 = Rzdot * Rx;
        Vector3d left = tmp1 * GK;

        Matrix3d tmp2 = Rz * Rx;
        Vector3d right = tmp2 * velocity;
        Vector3d V = left + right;

        Bds[prn].SatVelocity.X = V(0, 0);
        Bds[prn].SatVelocity.Y = V(1, 0);
        Bds[prn].SatVelocity.Z = V(2, 0);

        tmp2.deleteMatrix();
        tmp1.deleteMatrix();
        left.deleteMatrix();
        right.deleteMatrix();
        V.deleteMatrix();
        Rz.deleteMatrix();
        Rx.deleteMatrix();
        GK.deleteMatrix();
        Rzdot.deleteMatrix();
    }
    catch(...)
    {
        cout << "error happened when calculate GEO vel" << endl;
        return UNKNOWN_ERROR;
    }
    
    return 0;
}


Satellite* SatPos::GetPosAndVel(NavSys flag)
{
    if(flag == GPS)
        return Gps;
    else if(flag == BDS)
        return Bds;
    else
        exit(2);
}

int SatPos::Calculatet(const Obs* ObsData, const Ephemeris* Eph,
                       const int prn, const NavSys sys, const SPPResult sppresult, SATTIME &t)
{
    if(sys == GPS)
    {
        t = ObsData[prn].ObsTime - sppresult.GPSDist[prn] / LIGHTSPEED + sppresult.GPSR / LIGHTSPEED;
    }
    else if(sys == BDS)
    {
        t = ObsData[prn].ObsTime - sppresult.BDSDist[prn] / LIGHTSPEED - sppresult.BDSR / LIGHTSPEED;
    }

    try
    {
        double clkdif, clkdot;
        CalculateClkDif(Eph, prn, t, sys, clkdif);
        CalculateClkDifdot(Eph, prn, t, sys, clkdot);
        if(sys == GPS) {
            Gps[prn].clkdif = clkdif;
            Gps[prn].clkdot = clkdot;
        }
        else if(sys == BDS) {
            Bds[prn].clkdif = clkdif;
            Bds[prn].clkdot = clkdot;
        }
    }
    catch(...)
    {
        cout << "error happened when calculating t" << endl;
        return UNKNOWN_ERROR;
    }
    return 0;
}

int SatPos::CalculateClkDif(const Ephemeris* Eph, const int prn, const SATTIME tsv,
                            const NavSys sys, double &ClkDif)
{
    double F, Ek;
    ClkDif = 0;
    try
    {
        if(sys == GPS){
            F = -2 * sqrt(GPSMIU) / LIGHTSPEED / LIGHTSPEED;
            Ek = Gps[prn].Ek;
        }
        else if(sys == BDS){
            F = -2 * sqrt(BDSMIU) / LIGHTSPEED / LIGHTSPEED;
            Ek = Bds[prn].Ek;
        }
    }
    catch(...)
    {
        cout << "error happened when calculation satellite clock difference" << endl;
        return UNKNOWN_ERROR;
    }
    
    double deltatr = F * Eph[prn].ecc * Eph[prn].sqrtA * sin(Ek);
    SATTIME t_now = tsv;
    ClkDif = Eph[prn].af0 + Eph[prn].af1 * (t_now.SOW - Eph[prn].toc) + Eph[prn].af2 * (t_now.SOW - Eph[prn].toc) * (t_now.SOW - Eph[prn].toc)+ deltatr; // 双频观测  不加TGD

    return 0;
}

int SatPos::CalculateClkDifdot(const Ephemeris* Eph, const int prn, const SATTIME tsv,
                               const NavSys sys, double &ClkDifdot) {
    double F, Ek, Ekdot;
    ClkDifdot = 0;
    try
    {
        if(sys == GPS){
            F = -2 * sqrt(GPSMIU) / LIGHTSPEED / LIGHTSPEED;
            Ek = Gps[prn].Ek;
            Ekdot =  Gps[prn].Ekdot;
        }
        else if(sys == BDS){
            F = -2 * sqrt(BDSMIU) / LIGHTSPEED / LIGHTSPEED;
            Ek = Bds[prn].Ek;
            Ekdot = Bds[prn].Ekdot;
        }
    }
    catch(...)
    {
        cout << "error happened when calculation satellite clock difference" << endl;
        return UNKNOWN_ERROR;
    }
    double deltatr = F * Eph[prn].ecc * Eph[prn].sqrtA * cos(Ek) * Ekdot;
    SATTIME t_now = tsv;
    ClkDifdot = Eph[prn].af1 + 2 * Eph[prn].af2 * (t_now.SOW - Eph[prn].toc) + deltatr; 

    return 0;
}
