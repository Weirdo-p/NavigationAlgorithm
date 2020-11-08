/*************************************************
 * this file declare functions with data reading
 * @author XUZHUO WHU
 * 2020 09 27
*************************************************/

#ifndef _READDATA_H_
#define _READDATA_H_


#include "common.h"
#include <fstream>
#include <stdio.h>
#include <string.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
/*********************************
 * should position epoch by epoch
*********************************/
class ReadDataFromFile
{
    public:
    /*********************************************
     * function: a message must begin with
     * 0xAA 0x44 0x12 if finding the beginning,
     * return true.
     * else if meeting with eof or other error
     * return false
     * @param buff [out] to store head without aa4412
     * @return 0   successfully find the head
     *         1   open file error
     *         2   file ending
     *         3   crc wrong
     *         4   other errors
    **********************************************/
    int ReadHead(FILE* file, unsigned char* buff);

    /******************************************
     * to read bds eph
     * @param buff [in] using it to check CRC
     * @return 1   open file error
     *         2   file ending
     *         3   crc wrong
     *         4   other errors
     *         5   find Eph
     *         6   find Obs
    ******************************************/
    int ReadMessage(FILE* file, unsigned char* buff);

    /********************************************
     * to read gps eph
     * @param buff [in] using it to CRC
     * @return 0   successfully find the head
     *         1   open file error
     *         2   file ending
     *         3   crc wrong
     *         4   other errors
    ********************************************/
    int ReadGPSEph(FILE* file, unsigned char* buff);

    /******************************************
     * to read bds eph
     * @param buff [in] using it to check CRC
     * @return 0   successfully find the head
     *         1   open file error
     *         2   file ending
     *         3   crc wrong
     *         4   other errors
    ******************************************/
    int ReadBDSEph(FILE* file, unsigned char* buff);

    /******************************************
     * to read bds eph
     * @param buff [in] using it to check CRC
     * @return 1   open file error
     *         2   file ending
     *         3   crc wrong
     *         4   other errors
     *         5   find GPS OBS
     *         6   find BDS OBS
    ******************************************/
    int ReadObs(FILE* file, unsigned char* buff);

    /**************************
     * to get BDS observations
     * @return BDS Obs
    **************************/
    Obs* GetBDSObs();

    /**************************
     * to get GPS observations
     * @return GPS Obs
    **************************/
    Obs* GetGPSObs();

    /*************************
     * to get BDS Ephemris
     * @return BDS Ephemeris
    *************************/
    Ephemeris* GetBDSEph();

    /*************************
     * to get GPS Ephemris
     * @return GPS Ephemeris
    *************************/
    Ephemeris* GetGPSEph();

    /***************************
     * to get Head information
     * @return Head Infomation
    ***************************/
    FileHead GetHead();

    int ResetObs();

    protected:
    Obs BDSObs[MAXBDSSRN];
    Obs GPSObs[MAXGPSSRN];

    Ephemeris GPSEph[MAXGPSSRN];
    Ephemeris BDSEph[MAXBDSSRN];

    FileHead Head;

    unsigned char HeadBuff[25];
};

class ReadDataFromSocket {
public:
    int OpenSocket(char* ip, int port, int &test);

    int ReadSocketData(BUFF &buff, int desc);

    int ResetObs();

    /*********************************************
     * function: a message must begin with
     * 0xAA 0x44 0x12 if finding the beginning,
     * return true.
     * else if meeting with eof or other error
     * return false
     * @param buff [out] to store head without aa4412
     * @return 0   successfully find the head
     *         1   open file error
     *         2   file ending
     *         3   crc wrong
     *         4   other errors
    **********************************************/
    int ReadHead(BUFF &socketdata, unsigned char* head);

    /******************************************
     * to read bds eph
     * @param buff [in] using it to check CRC
     * @return 1   open file error
     *         2   file ending
     *         3   crc wrong
     *         4   other errors
     *         5   find Eph
     *         6   find Obs
    ******************************************/
    int ReadMessage(BUFF &socketdata, unsigned char* buff);

    /********************************************
     * to read gps eph
     * @param buff [in] using it to CRC
     * @return 0   successfully find the head
     *         1   open file error
     *         2   file ending
     *         3   crc wrong
     *         4   other errors
    ********************************************/
    int ReadGPSEph(BUFF &socketdata, unsigned char* buff);

    /******************************************
     * to read bds eph
     * @param buff [in] using it to check CRC
     * @return 0   successfully find the head
     *         1   open file error
     *         2   file ending
     *         3   crc wrong
     *         4   other errors
    ******************************************/
    int ReadBDSEph(BUFF &socketdata, unsigned char* buff);

    /******************************************
     * to read bds eph
     * @param buff [in] using it to check CRC
     * @return 1   open file error
     *         2   file ending
     *         3   crc wrong
     *         4   other errors
     *         5   find GPS OBS
     *         6   find BDS OBS
    ******************************************/
    int ReadObs(BUFF &socketdata, unsigned char* HeadBuff);

    /**
     * 
    */
    int reset(BUFF &socketdata);
public:
    /**************************
     * to get BDS observations
     * @return BDS Obs
    **************************/
    Obs* GetBDSObs();

    /**************************
     * to get GPS observations
     * @return GPS Obs
    **************************/
    Obs* GetGPSObs();

    /*************************
     * to get BDS Ephemris
     * @return BDS Ephemeris
    *************************/
    Ephemeris* GetBDSEph();

    /*************************
     * to get GPS Ephemris
     * @return GPS Ephemeris
    *************************/
    Ephemeris* GetGPSEph();

    /***************************
     * to get Head information
     * @return Head Infomation
    ***************************/
    FileHead GetHead();
protected:
    Obs BDSObs[MAXBDSSRN];
    Obs GPSObs[MAXGPSSRN];

    Ephemeris GPSEph[MAXGPSSRN];
    Ephemeris BDSEph[MAXBDSSRN];

    FileHead Head;

    unsigned char HeadBuff[25];
};

/********************************************
 * function: to decode unsigned char to int
 * @param src    want to be decoded
 * @param size   
 * @return dest
*******************************************/
int U2I(unsigned char* src, int size);

/********************************************
 * function: to decode char to int
 * @param src    want to be decoded
 * @param size   
 * @return dest
*******************************************/
int C2I(char* src, int size);

/********************************************
 * function: to decode char to double
 * @param src    want to be decoded
 * @param size   
 * @return dest
*******************************************/
double C2D(char* src, int size);

/********************************************
 * function: to decode unsigned char to long
 * @param src    want to be decoded
 * @param size   
 * @return dest
*******************************************/
unsigned long U2L(unsigned char* src, int size);

float C2F(char* src, int size);


/****************************************************************
 * Calculate a CRC value to be used by CRC calculation functions.
****************************************************************/
unsigned long CRC32Value(int i);

/****************************************************************
 * Calculates the CRC-32 of a block of data all at once
 * ulCount - Number of bytes in the data block
 * ucBuffer - Data block
****************************************************************/
unsigned long CalculateBlockCRC32(unsigned long ulCount, unsigned char* ucBuffer);

/**************************************************
 * function: to validate CRC
 * @param Head    [in] the Head stream
 * @param Message [in] the Message stream
 *                     include CRC information
 * @param MessageLength
 * @param pos     [in] position of CRC
 * @return true flase
**************************************************/
bool CRCCheck(unsigned char* Head, unsigned char* Message, int MessageLength, int pos);
#endif