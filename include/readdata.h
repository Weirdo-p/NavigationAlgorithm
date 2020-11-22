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
class ReadData
{
    // read data from socket
    public:
    /***************************************
     * function: to connect with web server
     * @param       ip      [in]  IP address
     * @param       port    [oin] port
     * @param       desc    [out] flag num
     * @return              status code
    ***************************************/
    int OpenSocket(char* ip, int port, int &desc);

    /********************************************
     * function: to copy data from socket buffer
     * @param       buff    [out] data buff
     * @param       desc    [in]  socket num
     * @return              status code
    ********************************************/
    int ReadSocketData(BUFF &buff, int desc);

    /*********************************************************
     * function: a message must begin with
     * 0xAA 0x44 0x12 if finding the beginning,
     * return true.
     * else if meeting with eof or other error
     * return false
     * @param socketdata [out] to store head without aa4412
     * @return 0         successfully find the head
     *         1         open file error
     *         2         file ending
     *         3         crc wrong
     *         4         other errors
    *********************************************************/
    int ReadHead(BUFF &socketdata, unsigned char* head);

    /******************************************************
     * to read bds eph
     * @param socketdata    [in] using it to check CRC
     * @return 1            open file error
     *         2            file ending
     *         3            crc wrong
     *         4            other errors
     *         5            find Eph
     *         6            find Obs
    *******************************************************/
    int ReadMessage(BUFF &socketdata, unsigned char* buff);

    /***************************************************
     * to read gps eph
     * @param socketdata    [in] using it to CRC
     * @return 0            successfully find the head
     *         1            open file error
     *         2            file ending
     *         3            crc wrong
     *         4            other errors
    ****************************************************/
    int ReadGPSEph(BUFF &socketdata, unsigned char* buff);

    /***************************************************
     * to read bds eph
     * @param socketdata    [in] using it to check CRC
     * @return 0            successfully find the head
     *         1            open file error
     *         2            file ending
     *         3            crc wrong
     *         4            other errors
    ***************************************************/
    int ReadBDSEph(BUFF &socketdata, unsigned char* buff);

    /***************************************************
     * to read bds eph
     * @param socketdata    [in] using it to check CRC
     * @return 1            open file error
     *         2            file ending
     *         3            crc wrong
     *         4            other errors
     *         5            find GPS OBS
     *         6            find BDS OBS
    ***************************************************/
    int ReadObs(BUFF &socketdata, unsigned char* HeadBuff);

    /*****************************************
     * function: to abandon old data stream
     * @param   socketdata
     * @return  status code
    *****************************************/
    int reset(BUFF &socketdata);

    /******************************
     * function: to read ref pos
     * @param   socketdata
     * @return  status code
    ******************************/
    int ReadRefPos(BUFF &socketdata, unsigned char* HeadBuff);

    /*************************************************
     * function: read data from specific ip address
     * @param desc  identifier with socket
     * @return      status code
    *************************************************/
    int decode(int desc);

    // read data from file
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
     * function: to read ref position
     * @param file  file pointer
     * @param buff  using it to CRC check
    ******************************************/
    int ReadRefPos(FILE* file, unsigned char* buff);

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

    /***********************************
     * to reset observations every time
     * SPP and SPV is done
    ***********************************/
    int ResetObs();

    /**********************************
     * function: to decode binary data
     * @param file  file point
     * @return      status code
    **********************************/
    int decode(FILE* file);

    BLH GetUserPos();

    protected:
    // observations
    Obs             BDSObs[MAXBDSSRN];
    Obs             GPSObs[MAXGPSSRN];

    // ephemeris
    Ephemeris       GPSEph[MAXGPSSRN];
    Ephemeris       BDSEph[MAXBDSSRN];

    // head infomation
    FileHead        Head;
    BUFF            databuff;

    unsigned char   HeadBuff[25];

    BLH             UserPsrPos;
};

// class ReadDataFromSocket {
// public:
//     /***************************************
//      * function: to connect with web server
//      * @param       ip      [in]  IP address
//      * @param       port    [oin] port
//      * @param       desc    [out] flag num
//      * @return              status code
//     ***************************************/
//     int OpenSocket(char* ip, int port, int &desc);

//     /********************************************
//      * function: to copy data from socket buffer
//      * @param       buff    [out] data buff
//      * @param       desc    [in]  socket num
//      * @return              status code
//     ********************************************/
//     int ReadSocketData(BUFF &buff, int desc);

//     /***********************************
//      * to reset observations every time
//      * SPP and SPV is done
//     ***********************************/
//     int ResetObs();
//     /*********************************************************
//      * function: a message must begin with
//      * 0xAA 0x44 0x12 if finding the beginning,
//      * return true.
//      * else if meeting with eof or other error
//      * return false
//      * @param socketdata [out] to store head without aa4412
//      * @return 0         successfully find the head
//      *         1         open file error
//      *         2         file ending
//      *         3         crc wrong
//      *         4         other errors
//     *********************************************************/
//     int ReadHead(BUFF &socketdata, unsigned char* head);

//     /******************************************************
//      * to read bds eph
//      * @param socketdata    [in] using it to check CRC
//      * @return 1            open file error
//      *         2            file ending
//      *         3            crc wrong
//      *         4            other errors
//      *         5            find Eph
//      *         6            find Obs
//     *******************************************************/
//     int ReadMessage(BUFF &socketdata, unsigned char* buff);

//     /***************************************************
//      * to read gps eph
//      * @param socketdata    [in] using it to CRC
//      * @return 0            successfully find the head
//      *         1            open file error
//      *         2            file ending
//      *         3            crc wrong
//      *         4            other errors
//     ****************************************************/
//     int ReadGPSEph(BUFF &socketdata, unsigned char* buff);

//     /***************************************************
//      * to read bds eph
//      * @param socketdata    [in] using it to check CRC
//      * @return 0            successfully find the head
//      *         1            open file error
//      *         2            file ending
//      *         3            crc wrong
//      *         4            other errors
//     ***************************************************/
//     int ReadBDSEph(BUFF &socketdata, unsigned char* buff);

//     /***************************************************
//      * to read bds eph
//      * @param socketdata    [in] using it to check CRC
//      * @return 1            open file error
//      *         2            file ending
//      *         3            crc wrong
//      *         4            other errors
//      *         5            find GPS OBS
//      *         6            find BDS OBS
//     ***************************************************/
//     int ReadObs(BUFF &socketdata, unsigned char* HeadBuff);

//     /*****************************************
//      * function: to abandon old data stream
//      * @param   socketdata
//      * @return  status code
//     *****************************************/
//     int reset(BUFF &socketdata);

//     /******************************
//      * function: to read ref pos
//      * @param   socketdata
//      * @return  status code
//     ******************************/
//     int ReadRefPos(BUFF &socketdata);

//     /*************************************************
//      * function: read data from specific ip address
//      * @param desc  identifier with socket
//      * @return      status code
//     *************************************************/
//     int decode(int desc);
// public:
//     /**************************
//      * to get BDS observations
//      * @return BDS Obs
//     **************************/
//     Obs* GetBDSObs();

//     /**************************
//      * to get GPS observations
//      * @return GPS Obs
//     **************************/
//     Obs* GetGPSObs();

//     /*************************
//      * to get BDS Ephemris
//      * @return BDS Ephemeris
//     *************************/
//     Ephemeris* GetBDSEph();

//     /*************************
//      * to get GPS Ephemris
//      * @return GPS Ephemeris
//     *************************/
//     Ephemeris* GetGPSEph();

//     /***************************
//      * to get Head information
//      * @return Head Infomation
//     ***************************/
//     FileHead GetHead();
// protected:
//     Obs             BDSObs[MAXBDSSRN];
//     Obs             GPSObs[MAXGPSSRN];

//     Ephemeris       GPSEph[MAXGPSSRN];
//     Ephemeris       BDSEph[MAXBDSSRN];

//     FileHead        Head;

//     unsigned char   HeadBuff[25];

//     BLH             UserPsrPos;
// };

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
 * @param ulCount       Number of bytes in the data block
 * @param ucBuffer      Data block
****************************************************************/
unsigned long CalculateBlockCRC32(unsigned long ulCount, unsigned char* ucBuffer);

/*********************************************************
 * function: to validate CRC
 * @param Head           [in]   the Head stream
 * @param Message        [in]   the Message stream
 *                              include CRC information
 * @param MessageLength
 * @param pos            [in]   position of CRC
 * @return                      true if success
*********************************************************/
bool CRCCheck(unsigned char* Head, unsigned char* Message, int MessageLength, int pos);
#endif