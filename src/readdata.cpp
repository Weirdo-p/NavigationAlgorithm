/****************************************
 * to read data from file
 * @author XUZHUO WHU
 * format of the data file should 
 * read the Reference Manual for OEM7
****************************************/
#include "readdata.h"
#include "coordinate.h"
#include <iostream>
#include <string.h>
#include <math.h>
#include <iomanip>

int ReadDataFromFile::decode(FILE* file) {
    int flag;
    unsigned char HeadBuff[25];
    flag = ReadHead(file, HeadBuff);
    if(flag != 0) {
        cout << "error happened with code " << flag << endl;
        return flag;
    }
    flag = ReadMessage(file, HeadBuff);

    // find neither eph nor obs
    return flag;
}

int ReadDataFromFile::ReadHead(FILE* file, unsigned char* buff)
{
    if(file == NULL)
    {
        cout << "open error" << endl;
        return OPEN_ERROR;
    }

    unsigned char HeadFlag[3];
    while(!feof(file))
    {
        if(!fread(&HeadFlag[2], 1, 1, file))
            return FILE_OR_BUFF_END;

        if(HeadFlag[0] != 0xaa || HeadFlag[1] != 0x44 || HeadFlag[2] != 0x12)
        {
            HeadFlag[0] = HeadFlag[1];
            HeadFlag[1] = HeadFlag[2];
            continue;
        }
        if(!fread(buff, 25, 1, file))
            return FILE_OR_BUFF_END;
        int HeadLength = U2I((buff + 3), 1);
        try
        {
            this->Head.MessageID = U2I((buff + 1), 2);
            this->Head.MessageLength = U2I((buff + 5), 2);
            this->Head.TimeStatus = U2I((buff + 10), 1);
            if(this->Head.MessageID == 43)
            {
                this->Head.GPST.Week = U2I((buff + 11), 2);
                this->Head.GPST.SOW = U2I((buff + 13), 4) / 1000.0;
            }
            this->Head.HealthFlag = (U2I((buff + 17), 4) & 1);
        }
        catch(...)
        {
            cout << "error happened when reading head" << endl;
            return UNKNOWN_ERROR;
        }
        break;
    }
    
    return 0;
}

int ReadDataFromFile::ReadMessage(FILE* file, unsigned char* HeadBuff)
{
    int flag = UNSUPPORTED_MSG;
    if(this->Head.MessageID == 7){
        // cout << "GPS   ";
        flag = this->ReadGPSEph(file, HeadBuff);
    }

    if(this->Head.MessageID == 1696){
        // cout << "BDS   ";
        flag = this->ReadBDSEph(file, HeadBuff);
    }
    
    if(Head.MessageID == 43){
        flag = this->ReadObs(file, HeadBuff);
    }

    if(Head.MessageID == 47) {
        flag = this->ReadRefPos(file, HeadBuff);
    }


    return flag;
}
int ReadDataFromFile::ReadRefPos(FILE* file, unsigned char* HeadBuff) {
    if(Head.MessageID != 47)
        return CONFLICT_ID;
    
    char* buff = new char[Head.MessageLength + 4];
    if(!fread(buff, this->Head.MessageLength + 4, 1, file))
        return FILE_OR_BUFF_END;

    try {
        if(!CRCCheck(HeadBuff, (unsigned char*)buff, Head.MessageLength, Head.MessageLength))
            return CRC_FAILED;
        UserPsrPos.B = C2D(buff + 8, 8);
        UserPsrPos.L = C2D(buff + 16, 8);
        UserPsrPos.H = C2D(buff + 24, 8);
    } 
    catch(...) {
        cout << "error happened when reading GPS ehpe" << endl;
        return UNKNOWN_ERROR;
    }
    
    delete [] buff;
   
}
int ReadDataFromFile::ReadGPSEph(FILE* file, unsigned char* HeadBuff)
{
    if(this->Head.MessageID != 7)
        return CONFLICT_ID;

    char* buff = new char[this->Head.MessageLength + 4];

    if(!fread(buff, this->Head.MessageLength + 4, 1, file))
        return FILE_OR_BUFF_END;

    try
    {
        if(!CRCCheck(HeadBuff, (unsigned char*)buff, Head.MessageLength, Head.MessageLength))
            return CRC_FAILED;

        unsigned short int prn = U2I((unsigned char*)buff, 4) - 1;
        if(prn < 0 || prn > MAXGPSSRN)
            return INVALID_PRN;
        this->GPSEph[prn].PRN = prn + 1;
        this->GPSEph[prn].sys = GPS;

        this->GPSEph[prn].HealthStatus = U2I((unsigned char*)(buff + 12), 4);
        this->GPSEph[prn].RefTime.Week = U2I((unsigned char*)(buff + 24), 4);
        this->GPSEph[prn].RefTime.SOW = C2D(buff + 32, 8);
        this->GPSEph[prn].deltaN = C2D(buff + 48, 8);
        this->GPSEph[prn].M0 = C2D((buff + 56), 8);
        this->GPSEph[prn].sqrtA = sqrt(C2D(buff + 40, 8));
        this->GPSEph[prn].ecc = C2D(buff + 64, 8);
        this->GPSEph[prn].w = C2D(buff + 72, 8);
        this->GPSEph[prn].cuc = C2D(buff + 80, 8);
        this->GPSEph[prn].cus = C2D(buff + 88, 8);
        this->GPSEph[prn].crc = C2D(buff + 96, 8);
        this->GPSEph[prn].crs = C2D(buff + 104, 8);
        this->GPSEph[prn].cic = C2D(buff + 112, 8);
        this->GPSEph[prn].cis = C2D(buff + 120, 8);
        this->GPSEph[prn].I0 = C2D(buff + 128, 8);
        this->GPSEph[prn].I0Rate = C2D(buff + 136, 8);
        this->GPSEph[prn].omegaO = C2D(buff + 144, 8);
        this->GPSEph[prn].omegaORate  = C2D(buff + 152, 8);
        this->GPSEph[prn].iodc = U2I((unsigned char*)buff + 160, 4);
        this->GPSEph[prn].toc = C2D(buff + 164, 8);
        this->GPSEph[prn].tgd = C2D(buff + 172, 8);
        this->GPSEph[prn].af0 = C2D(buff + 180, 8);
        this->GPSEph[prn].af1 = C2D(buff + 188, 8);
        this->GPSEph[prn].af2 = C2D(buff + 196, 8);
        this->GPSEph[prn].URA = C2D(buff + 216, 8);
    }
    catch(...)
    {
        cout << "error happened when reading GPS ehpe" << endl;
        return UNKNOWN_ERROR;
    }
    
    delete [] buff;

    return 0;
}

int ReadDataFromFile::ReadBDSEph(FILE* file, unsigned char* HeadBuff)
{
    if(Head.MessageID != 1696)
        return CONFLICT_ID;
    
    char* buff = new char [Head.MessageLength + 4];
    if(!fread(buff, this->Head.MessageLength + 4, 1, file))
        return FILE_OR_BUFF_END;

    if(!CRCCheck(HeadBuff, (unsigned char*)buff, Head.MessageLength, Head.MessageLength))
        return CRC_FAILED;

    try
    {
        unsigned short int prn = U2I((unsigned char*)buff, 4) - 1;
        if(prn < 0 || prn > MAXBDSSRN)
            return INVALID_PRN;
        this->BDSEph[prn].PRN = prn + 1;
        this->BDSEph[prn].sys = BDS;

        this->BDSEph[prn].RefTime.Week = U2I((unsigned char*)(buff + 4), 4);
        this->BDSEph[prn].URA = C2D(buff + 8, 8);
        this->BDSEph[prn].HealthStatus = U2I((unsigned char*)(buff + 16), 4);
        this->BDSEph[prn].tgd = C2D(buff + 20, 8);
        this->BDSEph[prn].tgd2 = C2D(buff + 28, 8);
        this->BDSEph[prn].toc = U2I((unsigned char*)(buff + 40), 4);
        this->BDSEph[prn].af0 = C2D(buff + 44, 8);
        this->BDSEph[prn].af1 = C2D(buff + 52, 8);
        this->BDSEph[prn].af2 = C2D(buff + 60, 8);
        this->BDSEph[prn].RefTime.SOW = U2I((unsigned char*)(buff + 72), 4);
        this->BDSEph[prn].ecc = C2D((buff + 84), 8);
        this->BDSEph[prn].w = C2D(buff + 92, 8);
        this->BDSEph[prn].sqrtA = C2D(buff + 76, 8);
        this->BDSEph[prn].deltaN = C2D(buff + 100, 8);
        this->BDSEph[prn].M0 = C2D(buff + 108, 8);
        this->BDSEph[prn].omegaO = C2D(buff + 116, 8);
        this->BDSEph[prn].omegaORate = C2D(buff + 124, 8);
        this->BDSEph[prn].I0 = C2D(buff + 132, 8);
        this->BDSEph[prn].I0Rate = C2D(buff + 140, 8);
        this->BDSEph[prn].cuc = C2D(buff + 148, 8);
        this->BDSEph[prn].cus = C2D(buff + 156, 8);
        this->BDSEph[prn].crc = C2D(buff + 164, 8);
        this->BDSEph[prn].crs = C2D(buff + 172, 8);
        this->BDSEph[prn].cic = C2D(buff + 180, 8);
        this->BDSEph[prn].cis = C2D(buff + 188, 8);
    }
    catch(...)
    {
        cout << "error happened when reading bds eph" << endl;
        return UNKNOWN_ERROR;
    }
    delete[] buff;
    return 0;
}

int ReadDataFromFile::ReadObs(FILE* file, unsigned char* HeadBuff)
{
    if(Head.MessageID != 43)
        return CONFLICT_ID;

    char* buff = new char [Head.MessageLength + 4];
    if(!fread(buff, this->Head.MessageLength + 4, 1, file))
        return FILE_OR_BUFF_END;

    int obsNum = U2I((unsigned char*)(buff), 4);
    if(!CRCCheck(HeadBuff, (unsigned char*)buff, Head.MessageLength, obsNum * 44 + 4))
        return CRC_FAILED;
        
    for(int prn = 0; prn < MAXGPSSRN; ++prn){
        this->GPSObs[prn].psr[0] = -1;
        this->GPSObs[prn].psr[1] = -1;
    }
    for(int prn = 0; prn < MAXBDSSRN; ++prn){
        this->BDSObs[prn].psr[0] = -1;
        this->BDSObs[prn].psr[1] = -1;
    }
    try
    {
        for(int i = 0; i < obsNum; ++i)
        {
            int ChannelState = U2I((unsigned char*)(buff + i * 44 + 44), 4);
            int sys = (ChannelState >> 16) & 7;
            switch(sys)
            {
                case BDS:
                {
                    int SignalType = (ChannelState >> 21) & 31;
                    unsigned short int PRN = U2I((unsigned char*)(buff + i * 44 + 4), 2) - 1;
                    this->BDSObs[PRN].ObsTime = Head.GPST;

                    // B1 signal
                    if(SignalType == 0 || SignalType == 4)
                    {
                        this->BDSObs[PRN].psr[0] = C2D(buff + i * 44 + 8, 8);
                        this->BDSObs[PRN].psr_sigma[0] = C2F(buff + i * 44 + 16, 4);
                        this->BDSObs[PRN].adr[0] = C2D(buff + i * 44 + 20, 8);
                        this->BDSObs[PRN].adr_sigma[0] = C2F(buff + i * 44 + 28, 4);
                        this->BDSObs[PRN].dopp[0] = C2F(buff + i * 44 + 32, 4);
                        this->BDSObs[PRN].CNo[0] = C2F(buff + i * 44 + 36, 4);
                        this->BDSObs[PRN].LockTime[0] = C2F(buff + i * 44 + 40, 4);
                    }
                    // B3 signal
                    if(SignalType == 2 || SignalType == 6)
                    {
                        this->BDSObs[PRN].psr[1] = C2D(buff + i * 44 + 8, 8);
                        this->BDSObs[PRN].psr_sigma[1] = C2F(buff + i * 44 + 16, 4);
                        this->BDSObs[PRN].adr[1] = C2D(buff + i * 44 + 20, 8);
                        this->BDSObs[PRN].adr_sigma[1] = C2F(buff + i * 44 + 28, 4);
                        this->BDSObs[PRN].dopp[1] = C2F(buff + i * 44 + 32, 4);
                        this->BDSObs[PRN].CNo[1] = C2F(buff + i * 44 + 36, 4);
                        this->BDSObs[PRN].LockTime[1] = C2F(buff + i * 44 + 40, 4);
                    }
                    break;
                }
                case GPS:
                {
                    int SignalType = (ChannelState >> 21) & 31;
                    unsigned short int PRN = U2I((unsigned char*)(buff + i * 44 + 4), 2) - 1;
                    this->GPSObs[PRN].ObsTime = Head.GPST;
                    // L1 C/A
                    if(SignalType == 0)
                    {
                        this->GPSObs[PRN].psr[0] = C2D(buff + i * 44 + 8, 8);
                        this->GPSObs[PRN].psr_sigma[0] = C2F(buff + i * 44 + 16, 4);
                        this->GPSObs[PRN].adr[0] = C2D(buff + i * 44 + 20, 8);
                        this->GPSObs[PRN].adr_sigma[0] = C2F(buff + i * 44 + 28, 4);
                        this->GPSObs[PRN].dopp[0] = C2F(buff + i * 44 + 32, 4);
                        this->GPSObs[PRN].CNo[0] = C2F(buff + i * 44 + 36, 4);
                        this->GPSObs[PRN].LockTime[0] = C2F(buff + i * 44 + 40, 4);
                    }
                    // L2P
                    if(SignalType == 9)
                    {
                        this->GPSObs[PRN].psr[1] = C2D(buff + i * 44 + 8, 8);
                        this->GPSObs[PRN].psr_sigma[1] = C2F(buff + i * 44 + 16, 4);
                        this->GPSObs[PRN].adr[1] = C2D(buff + i * 44 + 20, 8);
                        this->GPSObs[PRN].adr_sigma[1] = C2F(buff + i * 44 + 28, 4);
                        this->GPSObs[PRN].dopp[1] = C2F(buff + i * 44 + 32, 4);
                        this->GPSObs[PRN].CNo[1] = C2F(buff + i * 44 + 36, 4);
                        this->GPSObs[PRN].LockTime[1] = C2F(buff + i * 44 + 40, 4);
                    }
                    break;
                }
                default: break;
            }
        }
    }
    catch(...)
    {
        cout << "error happened when reading obs" << endl;
        return -1;
    }
    delete[] buff;
    return 0;
    
}

int ReadDataFromSocket::OpenSocket(char* ip, int port, int &desc) {
    // pos是当前数据流处理到的位置
    // 打开端口在循环开始，若此时原来buff中仍然有数据
    // 应该将剩余数据移动到buff开始
    // 新的数据存在之后
    int sock_cli = socket(AF_INET, SOCK_STREAM, 0);
    desc = sock_cli;
    sockaddr_in servaddr;
    memset(&servaddr, 0, sizeof(servaddr));
    servaddr.sin_family = AF_INET;
    servaddr.sin_port = htons(port);
    servaddr.sin_addr.s_addr = inet_addr(ip);
    if(connect(sock_cli, (sockaddr*)&servaddr, sizeof(servaddr)) < 0){
        perror("connect");
        return CONNECTION_FAILED;
    }
    return 0;
}

int ReadDataFromSocket::ReadSocketData(BUFF &buff, int desc) {
    // 将数据先读入缓冲区
    if(buff.pos != 0)
        reset(buff);
    int count = buff.pos;
    try
    {
        while (count < MAXDATALEN)
        {
            unsigned char tmp[1024];
            memset(tmp, 0x00, 1024);
            int num = recv(desc, tmp, 1024, 0);
            for(int i = 0; i < num; ++i)
                buff.buff[count++] = tmp[i];
            // buff[count] = tmp;
        }
        buff.len = count;
        buff.pos = 0;
        return 0;
    }
    catch(...)
    {
        cout << "error happened when getting data from socket" << endl;
        return UNKNOWN_ERROR;
    } 
}

int ReadDataFromSocket::ReadHead(BUFF &socketdata, unsigned char* buff) {
    unsigned char HeadFlag[3];
    for(int i = socketdata.pos; i < socketdata.len; ++i) {
        HeadFlag[2] = socketdata.buff[i];
        if(HeadFlag[0] != 0xaa || HeadFlag[1] != 0x44 || HeadFlag[2] != 0x12)
        {
            HeadFlag[0] = HeadFlag[1];
            HeadFlag[1] = HeadFlag[2];
            continue;
        }        
        memcpy(buff, socketdata.buff + i + 1, 25);

        try
        {
            this->Head.MessageID = U2I((buff + 1), 2);
            this->Head.MessageLength = U2I((buff + 5), 2);
            this->Head.TimeStatus = U2I((buff + 10), 1);
            if(this->Head.MessageID == 43)
            {
                this->Head.GPST.Week = U2I((buff + 11), 2);
                this->Head.GPST.SOW = U2I((buff + 13), 4) / 1000.0;
            }
            this->Head.HealthFlag = (U2I((buff + 17), 4) & 1);
            socketdata.pos += i + 25 + 1;
        }
        catch(...)
        {
            cout << "error happened when reading head" << endl;
            return UNKNOWN_ERROR;
        }
        break;
    }
    return 0;
}

int ReadDataFromSocket::ReadMessage(BUFF &socketdata, unsigned char* buff) {
    int flag = UNSUPPORTED_MSG;
    if(this->Head.MessageID == 7){
        flag = this->ReadGPSEph(socketdata, buff);
        return flag;
    }

    if(this->Head.MessageID == 1696){
        flag = this->ReadBDSEph(socketdata, buff);
        return flag;
    }
    
    if(Head.MessageID == 43){
        flag = this->ReadObs(socketdata, buff);
        return flag;
    }
    if(Head.MessageID == 47) {
        // flag = this->ReadRefPos(file, HeadBuff);
    }
    return flag;  
}

int ReadDataFromSocket::ReadObs(BUFF &socketdata, unsigned char* HeadBuff) {
    if(Head.MessageID != 43)
        return INVALID_PRN;
    if(socketdata.len - socketdata.pos < Head.MessageLength) {
        socketdata.pos -= 28;
        // reset(socketdata, pos);
        return FILE_OR_BUFF_END;
    }
    char* buff = new char [Head.MessageLength + 4];
    memcpy(buff, socketdata.buff + socketdata.pos, Head.MessageLength + 4);
    socketdata.pos += Head.MessageLength + 4;

    int obsNum = U2I((unsigned char*)(buff), 4);
    if(!CRCCheck(HeadBuff, (unsigned char*)buff, Head.MessageLength, Head.MessageLength)){
        delete[] buff;
        return CRC_FAILED;
    }

    for(int prn = 0; prn < MAXGPSSRN; ++prn){
        this->GPSObs[prn].psr[0] = -1;
        this->GPSObs[prn].psr[1] = -1;
    }
    for(int prn = 0; prn < MAXBDSSRN; ++prn){
        this->BDSObs[prn].psr[0] = -1;
        this->BDSObs[prn].psr[1] = -1;
    }
    try
    {
        for(int i = 0; i < obsNum; ++i)
        {
            int ChannelState = U2I((unsigned char*)(buff + i * 44 + 44), 4);
            int sys = (ChannelState >> 16) & 7;
            switch(sys)
            {
                case BDS:
                {
                    int SignalType = (ChannelState >> 21) & 31;
                    unsigned short int PRN = U2I((unsigned char*)(buff + i * 44 + 4), 2) - 1;
                    this->BDSObs[PRN].ObsTime = Head.GPST;

                    // B1 signal
                    if(SignalType == 0 || SignalType == 4)
                    {
                        // cout << "B1   " << PRN << endl;
                        this->BDSObs[PRN].psr[0] = C2D(buff + i * 44 + 8, 8);
                        this->BDSObs[PRN].psr_sigma[0] = C2F(buff + i * 44 + 16, 4);
                        this->BDSObs[PRN].adr[0] = C2D(buff + i * 44 + 20, 8);
                        this->BDSObs[PRN].adr_sigma[0] = C2F(buff + i * 44 + 28, 4);
                        this->BDSObs[PRN].dopp[0] = C2F(buff + i * 44 + 32, 4);
                        this->BDSObs[PRN].CNo[0] = C2F(buff + i * 44 + 36, 4);
                        this->BDSObs[PRN].LockTime[0] = C2F(buff + i * 44 + 40, 4);
                    }
                    // B3 signal
                    if(SignalType == 2 || SignalType == 6)
                    {
                        // cout << "B3   " << PRN << endl;
                        this->BDSObs[PRN].psr[1] = C2D(buff + i * 44 + 8, 8);
                        this->BDSObs[PRN].psr_sigma[1] = C2F(buff + i * 44 + 16, 4);
                        this->BDSObs[PRN].adr[1] = C2D(buff + i * 44 + 20, 8);
                        this->BDSObs[PRN].adr_sigma[1] = C2F(buff + i * 44 + 28, 4);
                        this->BDSObs[PRN].dopp[1] = C2F(buff + i * 44 + 32, 4);
                        this->BDSObs[PRN].CNo[1] = C2F(buff + i * 44 + 36, 4);
                        this->BDSObs[PRN].LockTime[1] = C2F(buff + i * 44 + 40, 4);
                    }
                    break;
                }
                case GPS:
                {
                    int SignalType = (ChannelState >> 21) & 31;
                    unsigned short int PRN = U2I((unsigned char*)(buff + i * 44 + 4), 2) - 1;
                    this->GPSObs[PRN].ObsTime = Head.GPST;
                    // L1 C/A
                    if(SignalType == 0)
                    {
                        this->GPSObs[PRN].psr[0] = C2D(buff + i * 44 + 8, 8);
                        this->GPSObs[PRN].psr_sigma[0] = C2F(buff + i * 44 + 16, 4);
                        this->GPSObs[PRN].adr[0] = C2D(buff + i * 44 + 20, 8);
                        this->GPSObs[PRN].adr_sigma[0] = C2F(buff + i * 44 + 28, 4);
                        this->GPSObs[PRN].dopp[0] = C2F(buff + i * 44 + 32, 4);
                        this->GPSObs[PRN].CNo[0] = C2F(buff + i * 44 + 36, 4);
                        this->GPSObs[PRN].LockTime[0] = C2F(buff + i * 44 + 40, 4);
                    }
                    // L2P
                    if(SignalType == 9)
                    {
                        this->GPSObs[PRN].psr[1] = C2D(buff + i * 44 + 8, 8);
                        this->GPSObs[PRN].psr_sigma[1] = C2F(buff + i * 44 + 16, 4);
                        this->GPSObs[PRN].adr[1] = C2D(buff + i * 44 + 20, 8);
                        this->GPSObs[PRN].adr_sigma[1] = C2F(buff + i * 44 + 28, 4);
                        this->GPSObs[PRN].dopp[1] = C2F(buff + i * 44 + 32, 4);
                        this->GPSObs[PRN].CNo[1] = C2F(buff + i * 44 + 36, 4);
                        this->GPSObs[PRN].LockTime[1] = C2F(buff + i * 44 + 40, 4);
                    }
                    break;
                }
                default: break;
            }
        }
    }
    catch(...)
    {
        cout << "error happened when reading obs" << endl;
        return UNKNOWN_ERROR;
    }
    delete[] buff;
    return 0;
    
}

int ReadDataFromSocket::ReadGPSEph(BUFF &socketdata, unsigned char* HeadBuff)
{
    if(this->Head.MessageID != 7)
        return CONFLICT_ID;

    if(socketdata.len - socketdata.pos < Head.MessageLength) {
        socketdata.pos -= 28;
        // reset(socketdata, pos);
        return FILE_OR_BUFF_END;
    }
    char* buff = new char [Head.MessageLength + 4];
    memcpy(buff, socketdata.buff + socketdata.pos, Head.MessageLength + 4);
    socketdata.pos += Head.MessageLength + 4;
    try
    {

        if(!CRCCheck(HeadBuff, (unsigned char*)buff, Head.MessageLength, Head.MessageLength)) {
            delete[] buff;
            return CRC_FAILED;
        }
        unsigned short int prn = U2I((unsigned char*)buff, 4) - 1;
        // cout << prn << endl;
        if(prn < 0 || prn > MAXGPSSRN)
            return INVALID_PRN;
        this->GPSEph[prn].PRN = prn + 1;
        this->GPSEph[prn].sys = GPS;

        this->GPSEph[prn].HealthStatus = U2I((unsigned char*)(buff + 12), 4);
        this->GPSEph[prn].RefTime.Week = U2I((unsigned char*)(buff + 24), 4);
        this->GPSEph[prn].RefTime.SOW = C2D(buff + 32, 8);
        this->GPSEph[prn].deltaN = C2D(buff + 48, 8);
        this->GPSEph[prn].M0 = C2D((buff + 56), 8);
        this->GPSEph[prn].sqrtA = sqrt(C2D(buff + 40, 8));
        this->GPSEph[prn].ecc = C2D(buff + 64, 8);
        this->GPSEph[prn].w = C2D(buff + 72, 8);
        this->GPSEph[prn].cuc = C2D(buff + 80, 8);
        this->GPSEph[prn].cus = C2D(buff + 88, 8);
        this->GPSEph[prn].crc = C2D(buff + 96, 8);
        this->GPSEph[prn].crs = C2D(buff + 104, 8);
        this->GPSEph[prn].cic = C2D(buff + 112, 8);
        this->GPSEph[prn].cis = C2D(buff + 120, 8);
        this->GPSEph[prn].I0 = C2D(buff + 128, 8);
        this->GPSEph[prn].I0Rate = C2D(buff + 136, 8);
        this->GPSEph[prn].omegaO = C2D(buff + 144, 8);
        this->GPSEph[prn].omegaORate  = C2D(buff + 152, 8);
        this->GPSEph[prn].iodc = U2I((unsigned char*)buff + 160, 4);
        this->GPSEph[prn].toc = C2D(buff + 164, 8);
        this->GPSEph[prn].tgd = C2D(buff + 172, 8);
        this->GPSEph[prn].af0 = C2D(buff + 180, 8);
        this->GPSEph[prn].af1 = C2D(buff + 188, 8);
        this->GPSEph[prn].af2 = C2D(buff + 196, 8);
        this->GPSEph[prn].URA = C2D(buff + 216, 8);
        // cout << setprecision(15) << GPSEph[prn].omega << endl;
    }
    catch(...)
    {
        cout << "error happened when reading GPS ehpe" << endl;
        return UNKNOWN_ERROR;
    }
    
    delete [] buff;

    return 0;
}

int ReadDataFromSocket::ReadBDSEph(BUFF &socketdata, unsigned char* HeadBuff) {
    if(Head.MessageID != 1696)
        return CONFLICT_ID;
    
    char* buff = new char [Head.MessageLength + 4];
    if(socketdata.len - socketdata.pos < Head.MessageLength) {
        socketdata.pos -= 28;
        // reset(socketdata, pos);
        return FILE_OR_BUFF_END;
    }
    memcpy(buff, socketdata.buff + socketdata.pos, Head.MessageLength + 4);
    socketdata.pos += Head.MessageLength + 4;
    if(!CRCCheck(HeadBuff, (unsigned char*)buff, Head.MessageLength, 196)) {
        delete[] buff;
        return CRC_FAILED;
    }
    // cout << "BDS   " << endl;
    try
    {
        unsigned short int prn = U2I((unsigned char*)buff, 4) - 1;
        // cout << prn << endl;
        if(prn < 0 || prn > MAXBDSSRN)
            return INVALID_PRN;
        this->BDSEph[prn].PRN = prn + 1;
        this->BDSEph[prn].sys = BDS;

        this->BDSEph[prn].RefTime.Week = U2I((unsigned char*)(buff + 4), 4);
        this->BDSEph[prn].URA = C2D(buff + 8, 8);
        this->BDSEph[prn].HealthStatus = U2I((unsigned char*)(buff + 16), 4);
        this->BDSEph[prn].tgd = C2D(buff + 20, 8);
        this->BDSEph[prn].tgd2 = C2D(buff + 28, 8);
        this->BDSEph[prn].toc = U2I((unsigned char*)(buff + 40), 4);
        this->BDSEph[prn].af0 = C2D(buff + 44, 8);
        this->BDSEph[prn].af1 = C2D(buff + 52, 8);
        this->BDSEph[prn].af2 = C2D(buff + 60, 8);
        this->BDSEph[prn].RefTime.SOW = U2I((unsigned char*)(buff + 72), 4);
        this->BDSEph[prn].ecc = C2D((buff + 84), 8);
        this->BDSEph[prn].w = C2D(buff + 92, 8);
        this->BDSEph[prn].sqrtA = C2D(buff + 76, 8);
        this->BDSEph[prn].deltaN = C2D(buff + 100, 8);
        this->BDSEph[prn].M0 = C2D(buff + 108, 8);
        this->BDSEph[prn].omegaO = C2D(buff + 116, 8);
        this->BDSEph[prn].omegaORate = C2D(buff + 124, 8);
        this->BDSEph[prn].I0 = C2D(buff + 132, 8);
        this->BDSEph[prn].I0Rate = C2D(buff + 140, 8);
        this->BDSEph[prn].cuc = C2D(buff + 148, 8);
        this->BDSEph[prn].cus = C2D(buff + 156, 8);
        this->BDSEph[prn].crc = C2D(buff + 164, 8);
        this->BDSEph[prn].crs = C2D(buff + 172, 8);
        this->BDSEph[prn].cic = C2D(buff + 180, 8);
        this->BDSEph[prn].cis = C2D(buff + 188, 8);
        // cout << BDSEph[prn].sqrtA << endl;
    }
    catch(...)
    {
        cout << "error happened when reading bds eph" << endl;
        return UNSUPPORTED_MSG;
    }
    delete[] buff;
    return 0;
}


int ReadDataFromSocket::reset(BUFF &buff) {
    for(int i = buff.pos; i < buff.len; ++i)
        buff.buff[i - buff.pos] = buff.buff[i];
    buff.pos = buff.len - buff.pos;
}

int U2I(unsigned char* src, int size)
{
    int dest = 0;;
    memcpy(&dest, src, size);
    return dest;
}

unsigned long U2L(unsigned char* src, int size)
{
    unsigned long dest;
    memcpy(&dest, src, size);
    return dest;
}

int C2I(char* src, int size)
{
    int dest;
    memcpy(&dest, src, size);
    return dest;
}

double C2D(char* src, int size)
{
    double dest;
    memcpy(&dest, src, size);
    return dest;
}

float C2F(char* src, int size)
{
    float dest;
    memcpy(&dest, src, size);
    return dest;
}

unsigned long CalculateBlockCRC32( unsigned long ulCount, unsigned char* ucBuffer )
{
    unsigned long ulTemp1;
    unsigned long ulTemp2;
    unsigned long ulCRC = 0;
    while ( ulCount-- != 0 )
    {
        ulTemp1 = ( ulCRC >> 8 ) & 0x00FFFFFFL;
        ulTemp2 = CRC32Value( ((int) ulCRC ^ *ucBuffer++ ) & 0xFF );
        ulCRC = ulTemp1 ^ ulTemp2;
    }
    return( ulCRC );
}

unsigned long CRC32Value(int i)
{
    int j;
    unsigned long ulCRC;
    ulCRC = i;
    for ( j = 8 ; j > 0; j-- )
    {
        if ( ulCRC & 1 )
            ulCRC = ( ulCRC >> 1 ) ^ CRC32_POLYNOMIAL;
        else
            ulCRC >>= 1;
    }
    return ulCRC;
}

bool CRCCheck(unsigned char* Head, unsigned char* Message, int MessageLength, int pos)
{
    unsigned char* CRCtest = new unsigned char[MessageLength + 28];
    CRCtest[0] = 0xAA;
    CRCtest[1] = 0x44;
    CRCtest[2] = 0x12;
    for(int i = 0; i < MessageLength + 25; ++i)
    {
        if(i < 25)
            CRCtest[i + 3] = Head[i];
        else
            CRCtest[i + 3] = Message[i - 25];
    }
    unsigned long CRC_Calcu = CalculateBlockCRC32(MessageLength + 28, CRCtest);
    unsigned long CRC = U2L((Message) + pos, 4);
    delete [] CRCtest;

    if(CRC == CRC_Calcu)
        return true;
    else
        return false;
}

int ReadDataFromSocket::ResetObs() {
    for(int i = 0; i < MAXBDSSRN; ++i) {
        this->BDSObs[i].psr[0] = -1;
        this->BDSObs[i].psr[1] = -1;
        this->BDSObs[i].ObsTime.Week = 0;
        this->BDSObs[i].ObsTime.SOW = -1;
    }
    for(int i = 0; i < MAXGPSSRN; ++i) {
        this->GPSObs[i].psr[0] = -1;
        this->GPSObs[i].psr[1] = -1;
        this->GPSObs[i].ObsTime.Week = 0;
        this->GPSObs[i].ObsTime.SOW = -1;
    }
    return 0;
}

int ReadDataFromFile::ResetObs() {
    for(int i = 0; i < MAXBDSSRN; ++i) {
        this->BDSObs[i].psr[0] = -1;
        this->BDSObs[i].psr[1] = -1;
        this->BDSObs[i].ObsTime.Week = 0;
        this->BDSObs[i].ObsTime.SOW = -1;
    }
    for(int i = 0; i < MAXGPSSRN; ++i) {
        this->GPSObs[i].psr[0] = -1;
        this->GPSObs[i].psr[1] = -1;
        this->GPSObs[i].ObsTime.Week = 0;
        this->GPSObs[i].ObsTime.SOW = -1;
    }
    return 0;
}

Obs* ReadDataFromFile::GetBDSObs()
{
    return BDSObs;
}

Obs* ReadDataFromFile::GetGPSObs()
{
    return GPSObs;
}

Ephemeris* ReadDataFromFile::GetBDSEph()
{
    return BDSEph;
}

Ephemeris* ReadDataFromFile::GetGPSEph()
{
    return GPSEph;
}

FileHead ReadDataFromFile::GetHead()
{
    return Head;
}


Obs* ReadDataFromSocket::GetBDSObs()
{
    return BDSObs;
}

Obs* ReadDataFromSocket::GetGPSObs()
{
    return GPSObs;
}

Ephemeris* ReadDataFromSocket::GetBDSEph()
{
    return BDSEph;
}

Ephemeris* ReadDataFromSocket::GetGPSEph()
{
    return GPSEph;
}

FileHead ReadDataFromSocket::GetHead()
{
    return Head;
}

int ReadDataFromSocket::decode(int desc) {
    BUFF databuff;
    int flag;

    cout << "waiting for data ..." << endl;
    
    flag = ReadSocketData(databuff, desc);
    if(flag != 0)
        return flag;

    unsigned char headbuff[25];
    flag = ReadHead(databuff, headbuff);
    if(flag != 0)
        return flag;

    flag = ReadMessage(databuff, headbuff);
    return flag;
}

int ReadDataFromSocket::ReadRefPos(BUFF &socketdata) {
    if(Head.MessageID != 47)
        return CONFLICT_ID;
    
    char* buff = new char [Head.MessageLength + 4];
    if(socketdata.len - socketdata.pos < Head.MessageLength) {
        socketdata.pos -= 28;
        // reset(socketdata, pos);
        return FILE_OR_BUFF_END;
    }
    memcpy(buff, socketdata.buff + socketdata.pos, Head.MessageLength + 4);
    socketdata.pos += Head.MessageLength + 4;
    if(!CRCCheck(HeadBuff, (unsigned char*)buff, Head.MessageLength, Head.MessageLength)) {
        delete[] buff;
        return CRC_FAILED;
    }

    try {
        UserPsrPos.B = C2D(buff + 8, 8);
        UserPsrPos.L = C2D(buff + 16, 8);
        UserPsrPos.H = C2D(buff + 24, 8);
    } 
    catch(...) {
        cout << "error happened when reading GPS ehpe" << endl;
        return UNKNOWN_ERROR;
    }
    
    delete [] buff;
    
}
