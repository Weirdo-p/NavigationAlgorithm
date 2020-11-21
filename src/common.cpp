#include "common.h"
#include "satpos.h"
#include "spp.h"

int solve(SatPos &SatposSolver, SPP &SppSolver, ReadDataFromFile &decoder, ELLIPSOID type, string path) {
    SPPResult result = SppSolver.GetResult();
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

int solve(SatPos &SatposSolver, SPP &SppSolver, ReadDataFromSocket &decoder, ELLIPSOID type, string path) {
    SPPResult result = SppSolver.GetResult();
    double dt0 = result.GPSR + result.BDSR;
    int count = 0, flag = 0;

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
        if(abs(dt1 - dt0) < 1e-10) {
            break;
        }
        dt0 = dt1;
        count ++;
    } 
    if(flag == 0) {
        SppSolver.solveSPV(SatposSolver.GetPosAndVel(GPS), SatposSolver.GetPosAndVel(BDS), 
                            decoder.GetGPSObs(), decoder.GetBDSObs(), 
                            decoder.GetGPSEph(), decoder.GetBDSEph(), type);
        result = SppSolver.GetResult();
        cout << result.UserPositionXYZ;
        WriteToFile(result, path);
        decoder.ResetObs();
    }

}