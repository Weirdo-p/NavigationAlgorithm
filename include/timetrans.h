/************************************************
 * to declare functions related to time transform
 * @author XUZHUO
************************************************/
#ifndef _TIMETRANS_H_
#define _TIMETRANS_H_

#include "common.h"

/**************************************************
 * function: to judge whether Common Time is legal
 * param:
 * @param   CT   Common Time
 * @return       flag 
 *      true     legal
 *      false   illegal      
**************************************************/
bool isLegal(const COMMONTIME CT);

/**************************************************
 * function: to judge whether GPS Time is legal
 * param:
 * @param   GPST   GPS Time
 * @return         flag 
 *       true     legal
 *       false   illegal      
**************************************************/
bool isLegal(const SATTIME GPST);

/**************************************************
 * function: to judge whether GPS Time is legal
 * param:
 * @param   MJD   GPS Time
 * @return        flag 
 *      true   -  legal
 *      false  - illegal      
**************************************************/
bool isLegal(const MJDTIME MJD);

/*****************************************
 * function: transform Common Time to MJD
 * param:
 * @param UTC  [in]     Universal Time
 * @param MJD  [out]    Modified Julian Day
 * @return              status code
*****************************************/
bool Common2Mjd(const COMMONTIME UT, MJDTIME &MJD);

/***********************************
 * function:
 * transform MJD to Common Time
 * param:
 * @param MJD    [in]    Modified Julian Day
 * @param UT     [out]   Universal Time
 * @return               status code
***********************************/
bool Mjd2Common(const MJDTIME MJD, COMMONTIME &UT);

/*******************************************
 * function:
 * transform MJD to GPST
 * param:
 * @param [in]    MJD  Modified Julian Day
 * @param [out]   GPST 
 * @return flag   status code
 *      true     success
 *      false  deadly error      
*******************************************/
bool Mjd2Gps(const MJDTIME MJD, SATTIME &GPST);

/*******************************************
 * function:
 * transform MJD to GPST
 * param:
 * @param [in]    GPST  
 * @param [out]   MJD  Modified Julian Day
 * @return flag   status code
 *      true     success
 *      false  deadly error      
*******************************************/
bool Gps2Mjd(const SATTIME GPST, MJDTIME &MJD);

/*******************************************
 * function:
 * transform MJD to GPST
 * param:
 * @param [in]    CT  Common Time
 * @param [out]   GPST 
 * @return flag   status code
 *      true     success
 *      false  deadly error      
*******************************************/
bool Common2Gps(const COMMONTIME CT, SATTIME &GPST);

/*******************************************
 * function:
 * transform MJD to Day Of Year
 * param:
 * @param  CT  [in] Common Time
 * @param  DOY [out]
 * @return flag   status code
*******************************************/
bool Common2Doy(const COMMONTIME CT, unsigned short int &DOY);

/***************************************
 * function: to transform gpst to bdst
 * @param gpst [in]   GPS time
 * @param bdst [out]  BDS time
 * @return flag   status code
***************************************/
bool GPST2BDST(const SATTIME gpst, SATTIME &bdst);

/***************************************
 * function: to transform gpst to bdst
 * @param gpst [in]   GPS time
 * @param bdst [out]  BDS time
 * @return flag   status code
 *         true     success
 *         false  deadly error      
***************************************/
bool BDST2GPST(const SATTIME bdst, SATTIME &gpst);


#endif