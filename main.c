

/**
** MHdr beg *****************************************************************
**
** Copyright Trident infosol 2018
** All Rights Reserved
**
**
** Module name: main.c
**
** Purpose:     This is the main file . RRCU Initialization
**              activities are triggered in the main function.
**
** Modification History:
** PCR           Date          Name            Comment
** ---           ----          ----            -------
** RCS_SOW_001    4/6/2018   Harish     Updated to fix review comment. 
**
** MPrj beg *****************************************************************
**
** MPrj end *****************************************************************
** MHdr end *****************************************************************
**/
#include "main.h"
#include "global.h" 

int32_t main(int32_t argc, int8_t *argv[])
{
    mcs_ctrlr(HEX_ONE);   
    return HEX_ZERO;

}


