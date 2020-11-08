/*************************************
 * @author XUZHUO WHU
 * definition of time transformation
 * 2020 09 19
*************************************/

#include "timetrans.h"
#include "common.h"
#include <iomanip>
const unsigned short LeapSec = 37;

ostream & operator<<(ostream &out, const COMMONTIME UT)
{

    out << UT.Year << "   " << UT.Month << "   "  << UT.Day
        <<  "   " << UT.Hour << "   "  << UT.Min << "   " << UT.Sec << endl;
    return out;
}

ostream & operator<<(ostream &out, const MJDTIME MJD)
{
    out << MJD.Day <<  "   " << MJD.FracDay << endl;
    return out;
}

ostream & operator<<(ostream &out, const SATTIME GPST)
{
    out << GPST.Week << "   " << setprecision(15) << GPST.SOW << endl;
    return out;
}

SATTIME SATTIME::operator-(const SATTIME &a) const
{      
    SATTIME r;
    r.Week = this->Week - a.Week;
    r.SOW = this->SOW - a.SOW;
    if(r.SOW < 0)
    {
        r.SOW += 604800;
        r.Week -= 1;
    }
    return r;
}

SATTIME SATTIME::operator-(const double &a) const
{
    SATTIME r;
    r.Week = this->Week;
    r.SOW = this->SOW - a;
    if(r.SOW < 0)
    {
        r.SOW += 604800.0;
        r.Week -= 1;
    }
    return r;
}

SATTIME SATTIME::operator+(const double &a) const
{
    SATTIME r;
    r.Week = this->Week;
    r.SOW = this->SOW + a;
    if(r.SOW >= 604800)
    {
        r.SOW -= 604800;
        r.Week += 1;
    }
    return r;
}


bool isLegal(const COMMONTIME CT)
{
    if(CT.Day > 31 || CT.Hour > 24 || CT.Min > 60 ||
       CT.Sec > 60 || CT.Day < 0 || CT.Hour < 0 || 
       CT.Min < 0 || CT.Sec < 0)
        return false;
    else
        return true;
}

bool isLegal(const SATTIME GPST)
{
    if(GPST.SOW < 0 || GPST.Week < 0)
        return false;
    else
        return true;
}

bool isLegal(const MJDTIME MJD)
{
    if(MJD.Day < 0 || MJD.FracDay >= 1)
        return false;
    else
        return true;
}

bool Common2Mjd(const COMMONTIME UT, MJDTIME &MJD)
{
    bool flag;
    unsigned short int y, m;

    // 输入合法性检测
    flag = isLegal(UT);
    if(!flag)
        return flag;

    if(UT.Month <= 2)
    {
        y = UT.Year - 1;
        m = UT.Month + 12;
    }
    else
    {
        y = UT.Year;
        m = UT.Month;
    }
    MJD.Day = int(365.25 * y) + int(30.6001 * (m + 1)) + UT.Day + 1720981;
    MJD.FracDay += (UT.Hour + UT.Min / 60.0 + (UT.Sec) / 3600.0) / 24.0;
    MJD.Day -= 2400000;
    // FracDay 合法性检测
    if(MJD.FracDay >= 1)
    {
        MJD.Day += int(MJD.FracDay);
        MJD.FracDay -= int(MJD.FracDay);
    }
    flag = true;

    return flag;
}

bool Mjd2Common(const MJDTIME MJD, COMMONTIME &UT)
{
    bool flag = isLegal(MJD);
    // 输入合法性判断
    if(!flag)
        return flag;

    // helpers
    double JD = MJD.Day + MJD.FracDay + 2400000.5;
    int a = int(JD + 0.5);
    int b = 1537 + a;
    int c = int((b - 122.1) / 365.25);
    int d = int(365.25 * c);
    int e = int((b - d) / 30.60001);
    double FracD = JD + 0.5;  // 小数天

    FracD -= int(FracD);
    UT.Day = b - d - int(30.60001 * e);
    UT.Month = e - 1 - 12 * int(e / 14.0);
    UT.Year = c - 4715 - int((7 + UT.Month) / 10.0);
    // 由小数天计算时分秒
    UT.Hour = int(FracD * 24.0);
    UT.Min = int((FracD * 24.0 - UT.Hour) * 60);
    UT.Sec = ((FracD * 24.0 - UT.Hour) * 60 - UT.Min) * 60;
    flag = true;

    return flag;
}

bool Mjd2Gps(const MJDTIME MJD, SATTIME &GPST)
{
    bool flag = isLegal(MJD);
    if(!flag)
        return flag;
    // 跳秒
    int LeapSec = 37;
    double DayofLeapSec = 37.0 / 24.0 / 3600.0;
    GPST.Week = int((MJD.Day + MJD.FracDay - 44244) / 7.0);
    GPST.SOW = (MJD.Day + MJD.FracDay + DayofLeapSec - GPST.Week * 7 - 44244) * 86400 - 19;  // TAI 与 GPST差19s
    // 合法性判断
    flag = isLegal(GPST);

    return flag;
}

bool Gps2Mjd(const SATTIME GPST, MJDTIME &MJD)
{
    bool flag = isLegal(GPST);
    if(!flag)
        return flag;
    int LeapSec = 37;
    double DayofLeapSec = 37.0 / 24.0 / 3600.0;

    MJD.Day = GPST.Week * 7 + 44244;
    MJD.FracDay = (GPST.SOW + 19) / 86400.0 + 44244 + GPST.Week * 7 - DayofLeapSec - MJD.Day;
    if(MJD.FracDay >= 1)
    {
        MJD.Day += int(MJD.FracDay);
        MJD.FracDay -= int(MJD.FracDay);
    }

    return true;
}

bool Common2Gps(const COMMONTIME CT, SATTIME &GPST)
{
    MJDTIME MJD;
    bool flag = Common2Mjd(CT, MJD);
    if(!flag)
        return flag;
    flag = Mjd2Gps(MJD, GPST);
    return flag;
}

bool Common2Doy(const COMMONTIME CT, unsigned short int &DOY)
{
    // 当前时间的儒略日
    MJDTIME MJD_Now;
    bool flag = Common2Mjd(CT, MJD_Now);
    if(!flag)
        return flag;

    // 当前年第一天的儒略日
    COMMONTIME NowYear;
    MJDTIME MJD_Beg;
    NowYear.Year = CT.Year;
    NowYear.Month = 1;
    NowYear.Day = 1;
    NowYear.Hour = 0;
    NowYear.Min = 0;
    NowYear.Sec = 0;
    flag = Common2Mjd(NowYear, MJD_Beg);
    if(!flag)
        return flag;
    
    DOY = int(MJD_Now.Day + MJD_Now.FracDay - (MJD_Beg.Day + MJD_Beg.FracDay)) + 1;

    return flag;
}

bool GPST2BDST(const SATTIME gpst, SATTIME &bdst)
{
    bdst.Week = gpst.Week - 1356;
    bdst.SOW = gpst.SOW - 14;
    if(bdst.SOW < 0)
    {
        bdst.SOW += 604800;
        bdst.Week -= 1;
    }
    return true;
}

bool BDST2GPST(const SATTIME bdst, SATTIME &gpst)
{
    gpst.Week = bdst.Week + 1356;
    gpst.SOW = bdst.SOW + 14;
    if(gpst.SOW >= 604800 )
    {
        gpst.SOW -= 604800;
        gpst.Week += 1;
    }
    return true;
}
