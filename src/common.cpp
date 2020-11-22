#include "common.h"
#include "satpos.h"
#include "spp.h"

int solve(SatPos &SatposSolver, SPP &SppSolver, ReadData &decoder, ELLIPSOID type, string path) {
    SPPResult result = SppSolver.GetResult();
    if(result.UserRefPositionXYZ.X == -1 && decoder.GetUserPos().H != 0) {
        BLH2XYZ(decoder.GetUserPos(), type, result.UserRefPositionXYZ);
        SppSolver.setRefPos(result.UserRefPositionXYZ);
    }
    // if(SppSolver.re)
    double dt0 = result.GPSR + result.BDSR;
    int count = 0;
    int flag;
    // SATTIME obs;
    while(count < 10)
    {
        flag = SatposSolver.CalculateGPSPos(decoder.GetGPSEph(), decoder.GetGPSObs(), result);
        if(flag != 0)
            break;
        flag = SatposSolver.CalculateSatVel(decoder.GetGPSEph(), GPS, result);
        if(flag != 0)
            break;
        flag = SatposSolver.CalculateBDSPos(decoder.GetBDSEph(), decoder.GetBDSObs(), result);
        if(flag != 0)
            break;
        flag = SatposSolver.CalculateSatVel(decoder.GetBDSEph(), BDS, result);
        if(flag != 0)
            break;
        flag = SppSolver.solveSPP(SatposSolver.GetPosAndVel(GPS), SatposSolver.GetPosAndVel(BDS), 
                            decoder.GetGPSObs(), decoder.GetBDSObs(), 
                            decoder.GetGPSEph(), decoder.GetBDSEph(), type);

        if(flag != 0)
            break;
        result = SppSolver.GetResult();
        double dt1 = result.GPSR + result.BDSR;
        if(abs(dt1 - dt0) < 1e-10)
        {
            break;
        }
        dt0 = dt1;
        count ++;
    } 
    SppSolver.solveSPV(SatposSolver.GetPosAndVel(GPS), SatposSolver.GetPosAndVel(BDS), 
                        decoder.GetGPSObs(), decoder.GetBDSObs(), 
                        decoder.GetGPSEph(), decoder.GetBDSEph(), type);
    result = SppSolver.GetResult();
    if(flag == 0) {
        SppSolver.solveSPV(SatposSolver.GetPosAndVel(GPS), SatposSolver.GetPosAndVel(BDS), 
                            decoder.GetGPSObs(), decoder.GetBDSObs(), 
                            decoder.GetGPSEph(), decoder.GetBDSEph(), type);
        result = SppSolver.GetResult();

        cout << result.UserPositionXYZ;
        WriteToFile(result, path);
        decoder.ResetObs();
    }
    return 0;
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
    if(isdata) {
        out << "Week      SOW         ECEF/X-m      ECEF/Y-m      ECEF/Z-m     ";
        out << "REF-ECEF/X-m  REF-ECEF/Y-m    ECEF/Z-m      ";
        out << "EAST/m  NORTH/m  UP/m    B/deg     L/deg     H/m      VX-m/s  VY-m/s  VZ-m/s     PDOP  Sigma-m  Sigma-m/s    BDSObsNum     GPSObsNum" << endl;
    }

    out << fixed << setprecision(4);
    out << result.ObsTime.Week << "  " << result.ObsTime.SOW << "  ";
    out << result.UserPositionXYZ.X << "  " << result.UserPositionXYZ.Y << "  " << result.UserPositionXYZ.Z << "  ";
    if(result.UserRefPositionXYZ.X != -1)
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
