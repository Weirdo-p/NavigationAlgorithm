/*******************************************
 * @author XUZHUO WHU
 * 本程序在Ubuntu18.04下测试通过
 * 比较疑惑的是 动态分配空间的数组 会自动delete???
 * 
 * 2020 09 11
 * 局部变量return以后与返回后的值共用一段地址
 * 故不需要释放局部变量
*******************************************/

#include <iostream>
#include <iomanip>
#include "matrix.h"
#include "timetrans.h"
#include "coordinate.h"
#include "readdata.h"
#include "satpos.h"
#include "spp.h"
#include <sstream>
#include <string.h>

using namespace std;

int main(int argc, char** argv)
{
    if(argc <= 1) {
        cout << "Usage: -i [IP address] [port]\n       -f [Binary File Path]" << endl;
        return 1;
    }
    string command = argv[1];

    if(command == "-f") {
        if(argc < 3 || argc >= 4) {
            cout << "Usage: -f [Binary File Path]" << endl;
            return 1;
        }
            
        char* filePath = argv[2];
        FILE* file = fopen(filePath, "rb");
        ELLIPSOID type;
        if(file == NULL)
        {
            cout << "open file error" << endl;
            exit(-1);
        }
        ReadDataFromFile decoder;
        bool findGPSEph = false, findBDSEph = false, findObs = false;
        while(!feof(file))
        {
            int flag;
            unsigned char HeadBuff[25];
            flag = decoder.ReadHead(file, HeadBuff);
            if(flag != 0)
            {
                cout << "error happened with code " << flag << endl;
                break;
            }
            flag = decoder.ReadMessage(file, HeadBuff);

            // find neither eph nor obs
            if(flag == -1)
                continue;
            else if(flag != 0 && flag > 0)
            {
                cout << "error happened with code " << flag << endl;
                continue;
            }

            SatPos Solver;
            SPP SppSolver;
            SPPResult result = SppSolver.GetResult();
            double dt0 = result.GPSR + result.BDSR;
            int count = 0;
            // SATTIME obs;
            while(count < 10)
            {
                flag = Solver.CalculateGPSPos(decoder.GetGPSEph(), decoder.GetGPSObs(), result);
                if(flag != 0)
                    break;
                flag = Solver.CalculateSatVel(decoder.GetGPSEph(), GPS, result);
                if(flag != 0)
                    break;
                flag = Solver.CalculateBDSPos(decoder.GetBDSEph(), decoder.GetBDSObs(), result);
                if(flag != 0)
                    break;
                flag = Solver.CalculateSatVel(decoder.GetBDSEph(), BDS, result);
                if(flag != 0)
                    break;
                flag = SppSolver.solveSPP(Solver.GetPosAndVel(GPS), Solver.GetPosAndVel(BDS), 
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
            SppSolver.solveSPV(Solver.GetPosAndVel(GPS), Solver.GetPosAndVel(BDS), 
                                decoder.GetGPSObs(), decoder.GetBDSObs(), 
                                decoder.GetGPSEph(), decoder.GetBDSEph(), type);
            result = SppSolver.GetResult();
            if(flag == 0) {
                SppSolver.solveSPV(Solver.GetPosAndVel(GPS), Solver.GetPosAndVel(BDS), 
                                    decoder.GetGPSObs(), decoder.GetBDSObs(), 
                                    decoder.GetGPSEph(), decoder.GetBDSEph(), type);
                result = SppSolver.GetResult();

                cout << result.UserPositionXYZ;
                WriteToFile(result);
                decoder.ResetObs();
            }
        }
        return 0;
    }

    if(command == "-i") {
        if(argc < 4 || argc >= 5) {
            cout << "Usage: -i [IP address] [port]\n" << endl;
            return 1;
        }
        string port_ = argv[3];
        stringstream tmp;
        tmp << port_;
        int port;
        tmp >> port;
        char* ip = argv[2];

        ReadDataFromSocket decoder;
        BUFF databuff;
        int pos = 0;
        int desc = 0;
        int flag = decoder.OpenSocket(ip, port, desc);
        if(flag == -1)
            return -1;
        cout << "waiting for data ..." << endl;
        while (true)
        {
            flag = decoder.ReadSocketData(databuff, desc);
            if(flag != 0)
                return -1;
            unsigned char headbuff[25];
            flag = decoder.ReadHead(databuff, headbuff);
            if(flag != 0)
                continue;
            flag = decoder.ReadMessage(databuff, headbuff);
            if(flag != 0)
                continue;
            else if(flag != 0 && flag > 0)
            {
                cout << "error happened with code " << flag << endl;
                continue;
            }

            ELLIPSOID type;
            SatPos Solver;
            SPP SppSolver;
            SppSolver.setRefPos(XYZ(-2267800.0690, 5009341.6472, 3220989.9179));
            SPPResult result = SppSolver.GetResult();
            
            double dt0 = result.GPSR + result.BDSR;
            int count = 0;
            // SATTIME obs;
            while(count < 10)
            {
                flag = Solver.CalculateGPSPos(decoder.GetGPSEph(), decoder.GetGPSObs(), result);
                if(flag != 0)
                    break;
                flag = Solver.CalculateSatVel(decoder.GetGPSEph(), GPS, result);
                if(flag != 0)
                    break;
                flag = Solver.CalculateBDSPos(decoder.GetBDSEph(), decoder.GetBDSObs(), result);
                if(flag != 0)
                    break;
                flag = Solver.CalculateSatVel(decoder.GetBDSEph(), BDS, result);
                if(flag != 0)
                    break;
                flag = SppSolver.solveSPP(Solver.GetPosAndVel(GPS), Solver.GetPosAndVel(BDS), 
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
                SppSolver.solveSPV(Solver.GetPosAndVel(GPS), Solver.GetPosAndVel(BDS), 
                                    decoder.GetGPSObs(), decoder.GetBDSObs(), 
                                    decoder.GetGPSEph(), decoder.GetBDSEph(), type);
                result = SppSolver.GetResult();
                cout << result.UserPositionXYZ;
                WriteToFile(result);
                decoder.ResetObs();
            }
        }

    }
    /* read data from socket */
    
}