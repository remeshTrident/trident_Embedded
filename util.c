/******************************************************************************
*
* Filename: 	util.c  
* 
* 
******************************************************************************/

#include "util.h"



/** FHdr-beg *****************************************************
**
** Function name: Get_Time
**
** Anchor:   		   
**
** Purpose:        This fiunction will get the local time
**
**
** Inputs:        time in second , time in fraction of second    
**
**
** Outputs:       RPM radio  
**
** FPrj-beg *****************************************************
**
** FPrj-end *****************************************************
** FHdr-end *****************************************************/


void Get_Time(int32_t *Sec, int32_t *FracSec)
{
    struct timeval Tv; 
    gettimeofday(&Tv, NULL);           //system call to receive local host time       
    *Sec = Tv.tv_sec;
    *FracSec = Tv.tv_usec;
    
}


