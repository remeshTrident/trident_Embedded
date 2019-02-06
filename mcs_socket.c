/******************************************************************************
*
* Filename: 	mcs_socket.c  
* 
* 
*******************************************************************************/

#include "mcs_socket.h"
#include "global.h"


/** FHdr-beg *****************************************************
**
** Function name: pfmbit_socket
**
** Anchor:        
**
** Purpose:       This function to create socket for pfmbit  
**				  
**				  
**							   
** Inputs:        NONE 
**
**
** Outputs:       NONE      
**
** FPrj-beg *****************************************************
**
** FPrj-end *****************************************************
** FHdr-end *****************************************************/ 

void pfmbit_socket(uint8_t pfmbit_no_of_retry)
{
    uint32_t pfmbit_len;
    struct sockaddr_in pfmbit_addr;        
    //while(TRUE)
    {
    	pfmbit_sockfd = socket(AF_INET, SOCK_STREAM, HEX_ZERO);   	
        pfmbit_addr.sin_family = AF_INET;
        pfmbit_addr.sin_addr.s_addr = inet_addr(MSC_PFMBIT_SERVER);        
    	//pfmbit_addr.sin_addr.s_addr = INADDR_ANY;
    	pfmbit_addr.sin_port = htons(PFMBIT_PORT);
    	pfmbit_len = sizeof(pfmbit_addr);
        if((connect(pfmbit_sockfd, (struct sockaddr *)&pfmbit_addr, pfmbit_len)) == ERROR)
        {
            close(pfmbit_sockfd);
            pfmbit_sockfd = ERROR;
            if(pfmbit_no_of_retry >= HEX_ONE)
            {
                break;
			}
            else
			{
                usleep(DEC_1000);
            } 
        }
        else
        {
            
            break;
        }
    } 
    
}



/** FHdr-beg *****************************************************
**
** Function name: ctrl_msc_socket
**
** Anchor:        
**
** Purpose:       This function to create socket for ctrlmsc 
**				  
**				  
**							   
** Inputs:        no of reentry  
**
**
** Outputs:       NONE      
**
** FPrj-beg *****************************************************
**
** FPrj-end *****************************************************
** FHdr-end *****************************************************/ 

void ctrl_msc_socket(uint8_t ctrl_msc_no_of_retry)
{
    uint32_t ctrl_msc_len; 
	uint32_t ctrl_msc_loop = HEX_ZERO;
    struct sockaddr_in ctrl_msc_addr;
    //while(TRUE)
	for(;;)
    {
        ctrl_msc_sockfd = socket(AF_INET, SOCK_STREAM, HEX_ZERO);    	
        //Prepare the sockaddr_in structure
        ctrl_msc_addr.sin_family = AF_INET;
        ctrl_msc_addr.sin_addr.s_addr = inet_addr(MSC_PFMBIT_SERVER);        
    	//ctrl_msc_addr.sin_addr.s_addr = INADDR_ANY;
    	ctrl_msc_addr.sin_port = htons(CTRLMSC_PORT);
    	ctrl_msc_len = sizeof(ctrl_msc_addr);
        if((connect(ctrl_msc_sockfd, (struct sockaddr *)&ctrl_msc_addr, ctrl_msc_len)) == ERROR)
        {           
            close(ctrl_msc_sockfd);
            ctrl_msc_sockfd = ERROR;
            if(ctrl_msc_no_of_retry == HEX_ONE)
            {
                break;
			}
            else
			{
                if(ctrl_msc_loop >= DEC_60)
                {
                    break;
                }
                else
				{
                    ; 
                }                 
                ctrl_msc_loop++; 
                usleep(DEC_1000);
            } 
        }
        else
        {
           
            break;
        }
    } 
    
}


/** FHdr-beg *****************************************************
**
** Function name: ctrl_mcs_socket
**
** Anchor:        
**
** Purpose:       This function to create socket for ctrlmcs 
**				  
**				  
**							   
** Inputs:        no of reentry  
**
**
** Outputs:       NONE      
**
** FPrj-beg *****************************************************
**
** FPrj-end *****************************************************
** FHdr-end *****************************************************/ 


void ctrl_mcs_socket(uint8_t ctrl_mcs_no_of_retry)
{
    uint32_t ctrl_mcs_len, ctrl_mcs_loop = HEX_ZERO;
    struct sockaddr_in ctrl_mcs_addr;
    //while(TRUE)
	for(;;)
    {       
        ctrl_mcs_sockfd = socket(AF_INET, SOCK_STREAM, HEX_ZERO); 
        ctrl_mcs_addr.sin_family = AF_INET;
        ctrl_mcs_addr.sin_addr.s_addr = inet_addr(MSC_PFMBIT_SERVER);
    	//ctrl_mcs_addr.sin_addr.s_addr = INADDR_ANY;
    	ctrl_mcs_addr.sin_port = htons(CTRLMCS_PORT);
    	ctrl_mcs_len = sizeof(ctrl_mcs_addr);
        if((connect(ctrl_mcs_sockfd, (struct sockaddr *)&ctrl_mcs_addr, ctrl_mcs_len)) == ERROR)
        {
            close(ctrl_mcs_sockfd);
            ctrl_mcs_sockfd = ERROR;
            if(ctrl_mcs_no_of_retry >= HEX_ONE)
            {
                break;
			}
            else
			{
                if(ctrl_mcs_loop >= DEC_60)
                {
                    break;
                }
                else
				{
                    ; 
                }                 
                ctrl_mcs_loop++; 
                usleep(DEC_1000);
            } 
        }
        else
        {
           
            break;
        }
    } 

}
