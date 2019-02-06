/******************************************************************************
*
* Filename: 	bit.c    
* 
* 
******************************************************************************/

#include "bit.h"
#include "global.h"

int8_t *PortName = "/dev/ttyXR";
int8_t *DiskName = "/dev/sda";

const int8_t *Pri_Eth_Mac = "70:b3:d5:c8:f0:00";
const int8_t *Sec_Eth_Mac = "00:e0:4b:4a:ba:ed";
const int8_t *PCI_Bridge_Vendor = "0x8086";
const int8_t *PCI_Exar_Vendor   = "0x13a8";

const int8_t *Pri_Eth_Carrier_Cmd   = "cat /sys/class/net/enp1s0/carrier";
const int8_t *Sec_Eth_Carrier_Cmd   = "cat /sys/class/net/enp0s31f6/carrier";
const int8_t *Pri_Eth_Address_Cmd   = "cat /sys/class/net/enp1s0/address";
const int8_t *Sec_Eth_Address_Cmd   = "cat /sys/class/net/enp0s31f6/address";
const int8_t *PCI_Bridge_Vendor_Cmd = "cat /sys/bus/pci/devices/0000:00:1c.0/vendor";
const int8_t *PCI_Bridge_Enable_Cmd = "cat /sys/bus/pci/devices/0000:00:1c.0/enable";
const int8_t *PCI_Exar_Vendor_Cmd   = "cat /sys/bus/pci/devices/0000:02:00.0/vendor";
const int8_t *PCI_Exar_Enable_Cmd   = "cat /sys/bus/pci/devices/0000:02:00.0/enable";


/** FHdr-beg *****************************************************
**
** Function name: open_dev
**
** Anchor:        RCS_SRS_REQ001
**
** Purpose:       function to open a device file
**
**
** Inputs:        name of the file to open 
**
**
** Outputs:       None.
**
** FPrj-beg *****************************************************
**
** FPrj-end *****************************************************
** FHdr-end *****************************************************/

int32_t open_dev(int8_t *dev_name)
{
    int32_t Fd;

    Fd = open(dev_name, O_RDWR | O_NOCTTY | O_NDELAY );  /* O_RDWR   - Read/Write access to serial port       */
							/* O_NOCTTY - No terminal will control the process   */
							/* Open in blocking mode,read will wait              */ 
    return Fd;
    
}
/** FHdr-beg *****************************************************
**
** Function name: mcs_ctrlr_post
**
** Anchor:        RCS_SRS_REQ001
**
** Purpose:       function to perform power on self test for mcs controller
**
**
** Inputs:        bit status  
**
**
** Outputs:       None.
**
** FPrj-beg *****************************************************
**
** FPrj-end *****************************************************
** FHdr-end *****************************************************/
void mcs_ctrlr_post(uint8_t *mcs_ctrlr_bit_status)
{
    int32_t i = HEX_ZERO;

    for(i = HEX_ZERO; i < TOTAL_MCS_CTRLR_BIT; i++)
    {
        mcs_ctrlr_bit_status[i] =  mcs_ctrlr_post_result(i);        
    } 
   
}

/** FHdr-beg *****************************************************
**
** Function name: mcs_ctrlr_post_result
**
** Anchor:        RCS_SRS_REQ001
**
** Purpose:       function to perform power on self test for mcs controller node
**
**
** Inputs:        bit number   
**
**
** Outputs:       ppower on self test result.
**
** FPrj-beg *****************************************************
**
** FPrj-end *****************************************************
** FHdr-end *****************************************************/
uint8_t mcs_ctrlr_post_result(uint8_t bit_num)
{
    int32_t Ret_Fd;           /* File Descriptor */
    int8_t  dev_name[DEC_20]; 
    int8_t  Buf[] = "01234567";
    int32_t i = HEX_ZERO;
    int32_t Result = HEX_ZERO; 
    int32_t *Arr;
    int32_t Limit = HEX_ONE;    
    int8_t  Str_Buffer[DEC_20];
    uint8_t Val = HEX_ZERO;
    FILE *Fp;

    switch(bit_num)
    {
        case ZERO: /* memory */
		        /*allocate memory for Limit elements dynamically*/
		    Arr = (int32_t *) malloc(Limit * sizeof(int32_t));    
			if(Arr == NULL)
			{
				
				Result = HEX_ZERO; 
			} 
			else
			{  
				
				Result = HEX_ONE;
			}
			free(Arr);
			break;

        case ONE: /* PCI bridge */ 
			Result = HEX_ZERO;	    
			/* If you want to read Output from command */
			Fp = popen(PCI_Bridge_Enable_Cmd, "r"); 
			fscanf(Fp, "%d", &Result);  		
			fclose(Fp);
			break;	  
        case TWO: /* PCI bus Interface */   
    		Result = HEX_ZERO;  
    		/* If you want to read Output from command */
    		Fp = popen(PCI_Exar_Enable_Cmd, "r"); 
    		fscanf(Fp, "%d", &Result);                
    		fclose(Fp);                 
            break;
        case THREE: /* hard disk */
	    	Ret_Fd = open_dev(DiskName);

	   		if(Ret_Fd == -HEX_ONE)
			{
			Result = HEX_ZERO;
				
			}
			else
			{
				Result = HEX_ONE;
				close(Ret_Fd);      
				
			} 
			break;

        case FOUR: /* Primary Ethernet */    

	    	    Result = HEX_ZERO;		        
		        i = system("ping -c1 -w1 192.168.100.100 > /dev/null 2>&1");
		        if(i == HEX_ZERO)
		        {
		            Result = HEX_ONE;
		        }
		        else
		        {
		            Result = HEX_ZERO;
		        }
				break;	  
        case FIVE: /* serial port */
                
			for(i = HEX_ZERO; i < HEX_EIGHT; i++)
			{
				strcpy(dev_name, PortName);        
				strncat(dev_name, &Buf[i], HEX_ONE);    
				Ret_Fd = open_dev(dev_name);

				if(Ret_Fd == -HEX_ONE)
				{
						
				} 
				else
				{
					Val = set_bit(Val, i); 
					close(Ret_Fd);       /* Close the serial port */
					
				}  

	    	} // end of for loop
			if(Val == HEX_FF)
			{
			Result = HEX_ONE;
			}
			else
			{
			Result = HEX_ZERO;
			}
			break;

        case SIX:  
        case SEVEN:
        case EIGHT:
        case NINE:
        case TEN:
        case ELEVEN: /* MPIOs */
			strcpy(dev_name, PortName);        
			strncat(dev_name, &Buf[HEX_ZERO], HEX_ONE); 	
			Ret_Fd = open_dev(dev_name);
			if(Ret_Fd == -HEX_ONE)
			{
				Result = HEX_ZERO;
				
			}
			else
			{
				Input.Reg = HEX_90;
				ioctl(Ret_Fd, EXAR_READ_REG, &Input);   
				Input.Reg = HEX_96;
				ioctl(Ret_Fd, EXAR_READ_REG, &Input);			  
				close(Ret_Fd);       /* Close the serial port */		  
				Result = HEX_ONE;
			} 
			break;	    
        case TWELVE: /* Secondary Ethernet */ 
			Result = HEX_ZERO;
			i = HEX_ZERO;
			i = system("ping -c1 -w1 192.168.100.101 > /dev/null 2>&1");
			if(i == HEX_ZERO)
			{
				Result = HEX_ONE;
			}
			else
			{
			   
				Result = HEX_ZERO;                                        
			}	    
			break;		
	       default: 
	       break;        

    } // end of switch    		
     
    return Result;
}
