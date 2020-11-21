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

int main(int argc, char** argv) {
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
        if(file == NULL) {
            cout << "open file error" << endl;
            exit(-1);
        }
        ReadDataFromFile decoder;
        while(!feof(file)) {
            int flag = decoder.decode(file);
            if(flag != 0) 
                continue;
            SatPos Solver;
            SPP SppSolver;
            solve(Solver, SppSolver, decoder, type);
        }
        return 0;
    }

    else if(command == "-i") {
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
        int desc = 0;
        int flag = decoder.OpenSocket(ip, port, desc);
        if(flag != 0)
            return flag;
        BUFF databuff;
        while (true) {
            flag = decoder.decode(desc);
            if(flag != 0)
                continue;
            ELLIPSOID type;
            SatPos Solver;
            SPP SppSolver;
            SppSolver.setRefPos(XYZ(-2267800.0690, 5009341.6472, 3220989.9179));
            solve(Solver, SppSolver, decoder, type);
        }
    }
    else {
        cout << "Usage: -i [IP address] [port]\n       -f [Binary File Path]" << endl;
        return 1;
    }
}