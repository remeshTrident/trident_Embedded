/******************************************************************************
*
* Filename: 	radio_commands.c    
* 
* 
******************************************************************************/

#include "radio_commands.h"
#include "global.h"


/** FHdr-beg *****************************************************
**
** Function name: checksum
**
** Anchor:        
**
** Purpose:       funtion to calculate checksum
**
**
** Inputs:        CRC Packet,CRC Packet Length    
**
**
** Outputs:       checksum 
**
** FPrj-beg *****************************************************
**
** FPrj-end *****************************************************
** FHdr-end *****************************************************/

uint32_t checksum(uint8_t *CRC_Packet, uint32_t CRC_Packet_Length)
{
    uint32_t CRC = HEX_ZERO;
    uint32_t CRC_Loop = HEX_ZERO;

    uint8_t Saved_CRC_Byte1 = HEX_ZERO, Saved_CRC_Byte2 = HEX_ZERO;

    static uint32_t CRC_Table[TWOHUNDREDFIFTYSIX] =  {	HEX_0x0000, HEX_0xC0C1, HEX_0xC181, HEX_0x0140, HEX_0xC301, HEX_0x03C0, HEX_0x0280, HEX_0xC241,
						HEX_0xC601, HEX_0x06C0, HEX_0x0780, HEX_0xC741, HEX_0x0500, HEX_0xC5C1, HEX_0xC481, HEX_0x0440,
						HEX_0xCC01, HEX_0x0CC0, HEX_0x0D80, HEX_0xCD41, HEX_0x0F00, HEX_0xCFC1, HEX_0xCE81, HEX_0x0E40,
						HEX_0x0A00, HEX_0xCAC1, HEX_0xCB81, HEX_0x0B40, HEX_0xC901, HEX_0x09C0, HEX_0x0880, HEX_0xC841,
						HEX_0xD801, HEX_0x18C0, HEX_0x1980, HEX_0xD941, HEX_0x1B00, HEX_0xDBC1, HEX_0xDA81, HEX_0x1A40,
						HEX_0x1E00, HEX_0xDEC1, HEX_0xDF81, HEX_0x1F40, HEX_0xDD01, HEX_0x1DC0, HEX_0x1C80, HEX_0xDC41,
						HEX_0x1400, HEX_0xD4C1, HEX_0xD581, HEX_0x1540, HEX_0xD701, HEX_0x17C0, HEX_0x1680, HEX_0xD641,
						HEX_0xD201, HEX_0x12C0, HEX_0x1380, HEX_0xD341, HEX_0x1100, HEX_0xD1C1, HEX_0xD081, HEX_0x1040,
						HEX_0xF001, HEX_0x30C0, HEX_0x3180, HEX_0xF141, HEX_0x3300, HEX_0xF3C1, HEX_0xF281, HEX_0x3240,
						HEX_0x3600, HEX_0xF6C1, HEX_0xF781, HEX_0x3740, HEX_0xF501, HEX_0x35C0, HEX_0x3480, HEX_0xF441,
						HEX_0x3C00, HEX_0xFCC1, HEX_0xFD81, HEX_0x3D40, HEX_0xFF01, HEX_0x3FC0, HEX_0x3E80, HEX_0xFE41,
						HEX_0xFA01, HEX_0x3AC0, HEX_0x3B80, HEX_0xFB41, HEX_0x3900, HEX_0xF9C1, HEX_0xF881, HEX_0x3840,
						HEX_0x2800, HEX_0xE8C1, HEX_0xE981, HEX_0x2940, HEX_0xEB01, HEX_0x2BC0, HEX_0x2A80, HEX_0xEA41,
						HEX_0xEE01, HEX_0x2EC0, HEX_0x2F80, HEX_0xEF41, HEX_0x2D00, HEX_0xEDC1, HEX_0xEC81, HEX_0x2C40,
						HEX_0xE401, HEX_0x24C0, HEX_0x2580, HEX_0xE541, HEX_0x2700, HEX_0xE7C1, HEX_0xE681, HEX_0x2640,
						HEX_0x2200, HEX_0xE2C1, HEX_0xE381, HEX_0x2340, HEX_0xE101, HEX_0x21C0, HEX_0x2080, HEX_0xE041,
						HEX_0xA001, HEX_0x60C0, HEX_0x6180, HEX_0xA141, HEX_0x6300, HEX_0xA3C1, HEX_0xA281, HEX_0x6240,
						HEX_0x6600, HEX_0xA6C1, HEX_0xA781, HEX_0x6740, HEX_0xA501, HEX_0x65C0, HEX_0x6480, HEX_0xA441,
						HEX_0x6C00, HEX_0xACC1, HEX_0xAD81, HEX_0x6D40, HEX_0xAF01, HEX_0x6FC0, HEX_0x6E80, HEX_0xAE41,
						HEX_0xAA01, HEX_0x6AC0, HEX_0x6B80, HEX_0xAB41, HEX_0x6900, HEX_0xA9C1, HEX_0xA881, HEX_0x6840,
						HEX_0x7800, HEX_0xB8C1, HEX_0xB981, HEX_0x7940, HEX_0xBB01, HEX_0x7BC0, HEX_0x7A80, HEX_0xBA41,
						HEX_0xBE01, HEX_0x7EC0, HEX_0x7F80, HEX_0xBF41, HEX_0x7D00, HEX_0xBDC1, HEX_0xBC81, HEX_0x7C40,
						HEX_0xB401, HEX_0x74C0, HEX_0x7580, HEX_0xB541, HEX_0x7700, HEX_0xB7C1, HEX_0xB681, HEX_0x7640,
						HEX_0x7200, HEX_0xB2C1, HEX_0xB381, HEX_0x7340, HEX_0xB101, HEX_0x71C0, HEX_0x7080, HEX_0xB041,
						HEX_0x5000, HEX_0x90C1, HEX_0x9181, HEX_0x5140, HEX_0x9301, HEX_0x53C0, HEX_0x5280, HEX_0x9241,
						HEX_0x9601, HEX_0x56C0, HEX_0x5780, HEX_0x9741, HEX_0x5500, HEX_0x95C1, HEX_0x9481, HEX_0x5440,
						HEX_0x9C01, HEX_0x5CC0, HEX_0x5D80, HEX_0x9D41, HEX_0x5F00, HEX_0x9FC1, HEX_0x9E81, HEX_0x5E40,
						HEX_0x5A00, HEX_0x9AC1, HEX_0x9B81, HEX_0x5B40, HEX_0x9901, HEX_0x59C0, HEX_0x5880, HEX_0x9841,
						HEX_0x8801, HEX_0x48C0, HEX_0x4980, HEX_0x8941, HEX_0x4B00, HEX_0x8BC1, HEX_0x8A81, HEX_0x4A40,
						HEX_0x4E00, HEX_0x8EC1, HEX_0x8F81, HEX_0x4F40, HEX_0x8D01, HEX_0x4DC0, HEX_0x4C80, HEX_0x8C41,
						HEX_0x4400, HEX_0x84C1, HEX_0x8581, HEX_0x4540, HEX_0x8701, HEX_0x47C0, HEX_0x4680, HEX_0x8641,
						HEX_0x8201, HEX_0x42C0, HEX_0x4380, HEX_0x8341, HEX_0x4100, HEX_0x81C1, HEX_0x8081, HEX_0x4040
                                      };

    

    Saved_CRC_Byte1 = CRC_Packet[CRC_Packet_Length - HEX_ONE];
    Saved_CRC_Byte2 = CRC_Packet[CRC_Packet_Length - HEX_TWO];
    CRC_Packet[CRC_Packet_Length - HEX_ONE] = HEX_ZERO;
    CRC_Packet[CRC_Packet_Length - HEX_TWO] = HEX_ZERO; 
    for(CRC_Loop = HEX_ZERO; CRC_Loop < CRC_Packet_Length; CRC_Loop++)
    {
        CRC = (CRC >> HEX_EIGHT) ^ CRC_Table[(CRC ^ CRC_Packet[CRC_Loop]) & HEX_FF];     
    } 
    CRC_Packet[CRC_Packet_Length - HEX_ONE] = (uint8_t) (CRC & HEX_0x00FF);
    CRC_Packet[CRC_Packet_Length - HEX_TWO] = (uint8_t) ((CRC & HEX_0xFF00) >> HEX_EIGHT);
    return ((CRC_Packet[CRC_Packet_Length - HEX_ONE] ==  Saved_CRC_Byte1) && (CRC_Packet[CRC_Packet_Length - HEX_TWO] ==  Saved_CRC_Byte2));
    
}



/** FHdr-beg *****************************************************
**
** Function name: f_initialisation
**
** Anchor:   		RCS_SOW_001     
**
** Purpose:        This fiunction will initialize radio 
**
**
** Inputs:         radio initialize packet information , radio initialize packet    
**
**
** Outputs:       radio initialization status  
**
** FPrj-beg *****************************************************
**
** FPrj-end *****************************************************
** FHdr-end *****************************************************/

uint8_t *f_initialisation(uint8_t Init_RadioPackInfo, uint8_t *Init_RadioPack)
{
    //uint8_t Radio_Init_Buf[8] = {0xAA, 0x00, 0x00, 0x08, 7, 100, 0, 0};
    uint8_t Radio_Init_Buf[EIGHT] = {HEX_0xAA, HEX_ZERO, HEX_ZERO, HEX_EIGHT, HEX_ZERO, HEX_0x32, HEX_ZERO, HEX_ZERO};
    Radio_Init_Buf[FOUR] = Init_RadioPackInfo;
    checksum(Radio_Init_Buf, Radio_Init_Buf[HEX_THREE]);
    memcpy(Init_RadioPack, Radio_Init_Buf, HEX_EIGHT);
    return(Init_RadioPack);
}


/** FHdr-beg *****************************************************
**
** Function name: f_comm_check
**
** Anchor:   		RCS_SOW_001     
**
** Purpose:        This fiunction will check the radio communication  
**
**
** Inputs:         radio communication  packet information , radio commuinication packet    
**
**
** Outputs:       radio initialization status  
**
** FPrj-beg *****************************************************
**
** FPrj-end *****************************************************
** FHdr-end *****************************************************/
uint8_t *f_comm_check(uint8_t CommCheck_RadioPackInfo, uint8_t *CommCheck_RadioPack)
{
    uint8_t CommCheck_Buf[SIX] = {HEX_0xAA, HEX_SIX, HEX_ZERO, HEX_SIX, HEX_B, HEX_0x70};
    memcpy(CommCheck_RadioPack, CommCheck_Buf, HEX_SIX);			
    return(CommCheck_RadioPack);	
}


/** FHdr-beg *****************************************************
**
** Function name: f_clear_nvm
**
** Anchor:   		   
**
** Purpose:        This fiunction will clear NVM  
**
**
** Inputs:         radio NVM packet  information , radio NVM packet    
**
**
** Outputs:       radio initialization status  
**
** FPrj-beg *****************************************************
**
** FPrj-end *****************************************************
** FHdr-end *****************************************************/

uint8_t *f_clear_nvm(uint8_t CNVM_RadiopPackInfo, uint8_t *CNVM_RadiopPack)
{
    uint8_t ClearNVM_Buf[SEVEN] = {HEX_0xAA, HEX_0x82, HEX_ZERO, HEX_SEVEN, HEX_ONE, HEX_ZERO, HEX_ZERO};			
    checksum(ClearNVM_Buf, ClearNVM_Buf[HEX_THREE]);
    memcpy(CNVM_RadiopPack, ClearNVM_Buf, HEX_SEVEN);			
    return(CNVM_RadiopPack);	
}
/** FHdr-beg *****************************************************
**
** Function name: radiation_on_off
**
** Anchor:   		   
**
** Purpose:        This fiunction willon off radiation  
**
**
** Inputs:         radio NVM packet  information , radio NVM packet    
**
**
* *************** Values intialisation to the RADIATION_ON_OFF Packet	***************	
* Radn_Buf[0]  = 0xAA;					// Synchronization 			
* Radn_Buf[1]  = 0x05;					// Packet_id 				
* Radn_Buf[2]  = 00;					// Packet_length_MSB 
* Radn_Buf[3]  = 50;					// Packet_length_LSB 		
* Radn_Buf[4]  = 0;					// 1553 Monitor Mode		
* Radn_Buf[5]  = 0;					// Power-up BIT Control		
* Radn_Buf[6]  = 0;					// Output Squelch Status 	
* Radn_Buf[7]  = 1;					// Reserved 				
* Radn_Buf[8]  = 0;					// Reserved 				
* Radn_Buf[9]  = Radn_RadioPackInfo[1];		// Inhibit RF Transmission 	
* Radn_Buf[10] = 0;					// Key Fill Default 		
* Radn_Buf[11] = 0;					// Reserved 				
* Radn_Buf[12] = 0;					// Antenna Port Usage 		
* Radn_Buf[13] = 0;					// TX/RX Clock Output Style 
* Radn_Buf[14] = 0;					// Reserved 				
* Radn_Buf[15] = 0;					// Reserved 				
* Radn_Buf[16] = 0;					// Reserved					
* Radn_Buf[17] = 0;					// Reserved 				
* Radn_Buf[18] = 0;					// Reserved 				
* Radn_Buf[19] = 0;					// Reserved 				
* Radn_Buf[20] = 1;					// Reserved 				
* Radn_Buf[21] = 0;					// Reserved 				
* Radn_Buf[22] = 0;					// Reserved 				
* Radn_Buf[23] = 0;					// Reserved 				
* Radn_Buf[24] = 0;					// Reserved 				
* Radn_Buf[25] = 0;					// Reserved 				
* Radn_Buf[26] = 0;					// Reserved 				
* Radn_Buf[27] = 0;					// Reserved 				
* Radn_Buf[28] = 0;					// Reserved 				
* Radn_Buf[29] = 0;					// Reserved 				
* Radn_Buf[30] = 0;					// Reserved 				
* Radn_Buf[31] = 1;					// Unsolicited Packets 		
* Radn_Buf[32] = 1;					// Sidetone 				
* Radn_Buf[33] = 0;					// Tpwr Display 			
* Radn_Buf[34] = 0;					// HPA Fault Reporting 		
* Radn_Buf[35] = 0;					// Reserved 				
* Radn_Buf[36] = 0;					// Preset Operating Mode Enable 
* Radn_Buf[37] = 0;					// Current Mode Menu 		
* Radn_Buf[38] = 10;					// Sidetone Level 					
* Radn_Buf[39] = 0;					// Simplified LOS 			
* Radn_Buf[40] = 0;					// Preset Preview 			
* Radn_Buf[41] = 1;					// Radio Version Indicator 	
* Radn_Buf[42] = 0;					// Reserved 				
* Radn_Buf[43] = 0;					// Reserved 				
* Radn_Buf[44] = 0;					// Reserved 				
* Radn_Buf[45] = 0;					// Reserved 				
* Radn_Buf[46] = 0;					// Reserved 				
* Radn_Buf[47] = 0;					// Reserved 
* Radn_Buf[48] = 0;					// CRC 
* Radn_Buf[49] = 0;					// CRC 
** Outputs:       radio radiation packet   
**
** FPrj-beg *****************************************************
**
** FPrj-end *****************************************************
** FHdr-end *****************************************************/


uint8_t *f_radiation_on_off(uint8_t *Radn_RadioPackInfo, uint8_t *Radn_RadioPack)
{	
    uint8_t Radn_Buf[FIFTY] = {HEX_0xAA, HEX_FIVE, HEX_ZERO, DEC_50, HEX_ZERO, HEX_ZERO, HEX_ZERO, HEX_ONE, HEX_ZERO, HEX_ZERO, HEX_ZERO, HEX_ZERO, HEX_ZERO, HEX_ZERO, HEX_ZERO, HEX_ZERO, HEX_ZERO, HEX_ZERO, HEX_ZERO, HEX_ZERO, HEX_ONE, HEX_ZERO, HEX_ZERO, HEX_ZERO,  
                            HEX_ZERO, HEX_ZERO, HEX_ZERO, HEX_ZERO, HEX_ZERO, HEX_ZERO, HEX_ZERO, HEX_ONE, HEX_ONE, HEX_ZERO, HEX_ZERO, HEX_ZERO, HEX_ZERO, HEX_ZERO, DEC_10, HEX_ZERO, HEX_ZERO, HEX_ONE, HEX_ZERO, HEX_ZERO, HEX_ZERO, HEX_ZERO, HEX_ZERO, HEX_ZERO, HEX_ZERO, HEX_ZERO};

    Radn_Buf[NINE] = Radn_RadioPackInfo[ONE];		// Inhibit RF Transmission 
    checksum(Radn_Buf, Radn_Buf[THREE]);
    memcpy(Radn_RadioPack, Radn_Buf, DEC_50);	
    return(Radn_RadioPack);

}


/** FHdr-beg *****************************************************
**
** Function name: f_activate_setup
**
** Anchor:   		   
**
** Purpose:        This fiunction will activate radio set up
**
**
** Inputs:         radio activate   information , radio activate packet    
**
**
** Outputs:       radio initialization status  
**
** FPrj-beg *****************************************************
**
** FPrj-end *****************************************************
** FHdr-end *****************************************************/



uint8_t *f_activate_setup(uint8_t AS_RadioPackInfo, uint8_t *AS_RadioPack)
{	
    uint8_t ActivateSetup_Buf[NINE] = {HEX_0xAA, HEX_0X26, HEX_ZERO, HEX_NINE, HEX_ZERO, HEX_ZERO, HEX_ONE, HEX_ZERO, HEX_ZERO};	
    ActivateSetup_Buf[FIVE] = AS_RadioPackInfo;	       // Variant									
    checksum(ActivateSetup_Buf, ActivateSetup_Buf[THREE]);   // checksum	
    memcpy(AS_RadioPack, ActivateSetup_Buf, HEX_NINE);	
    return(AS_RadioPack);
}


/** FHdr-beg *****************************************************
**
** Function name: f_storesetup_rpwman
**
** Anchor:   		   
**
** Purpose:        This fiunction will store rpm set up
**
**
** Inputs:         RPM radio packet   information , RPM radio  packet    
**
**
** Outputs:       RPM radio  
**
** FPrj-beg *****************************************************
**
** FPrj-end *****************************************************
** FHdr-end *****************************************************/





uint8_t *f_storesetup_rpwman(uint8_t SS_RPWMan_RadioPackInfo, uint8_t *SS_RPWMan_RadioPack)
{
    uint8_t SS_RPWMan_Buf[HUNDREDANDSEVENTEEN] = ""; 
    uint8_t SS_RPWMan_NetMaster = HEX_ZERO;	
    if(SS_RPWMan_RadioPackInfo == HEX_ONE)
    {
        SS_RPWMan_NetMaster = HEX_ONE;
    }
    else
    {
        SS_RPWMan_NetMaster = HEX_ZERO;
    }	
    /* Store Setup for RPW-Manual */
    SS_RPWMan_Buf[ZERO]   			= HEX_0xAA;					// Synchronization
    SS_RPWMan_Buf[ONE]   			= HEX_0X25;					// Packet ID
    SS_RPWMan_Buf[TWO]   			= HEX_ZERO;					// Packet Length MSB
    SS_RPWMan_Buf[THREE]   			= DEC_117;					// Packet Length LSB
    SS_RPWMan_Buf[FOUR]   			= HEX_ZERO;					// Mode 0 = LOS
    SS_RPWMan_Buf[FIVE]   			= HEX_ONE;					// ID 0 = Current Preset
    SS_RPWMan_Buf[SIX]   			= DEC_10;					// Variant 3 = Normal,7-HQ , 10 -RPW
    SS_RPWMan_Buf[SEVEN]   			= HEX_ONE;					// Modulation 0 = AM and 1 = FM
    SS_RPWMan_Buf[EIGHT]   			= HEX_ZERO;					// Reserved1
    SS_RPWMan_Buf[NINE]   			= HEX_ONE;					// Reserved2
    SS_RPWMan_Buf[TEN]  			= HEX_ZERO;					// Source 0 = Voice
    SS_RPWMan_Buf[ELEVEN]  			= DEC_14;					// Preset I/O Data Rate 11 = 9600bps 14 = 16000bps
    SS_RPWMan_Buf[TWELVE]  			= HEX_0xF8;					// Transmit Freq LSB
    SS_RPWMan_Buf[THIRTEEN]  		= HEX_0x95;					// Transmit Freq LSB
    SS_RPWMan_Buf[FOURTEEN]  		= HEX_THREE;					// Transmit Freq MSB
    SS_RPWMan_Buf[FIFTEEN]  		= HEX_ZERO;					// Transmit Freq MSB
    SS_RPWMan_Buf[SIXTEEN]  		= HEX_0xF8;					// Receive Freq LSB
    SS_RPWMan_Buf[SEVENTEEN]  		= HEX_0x95;					// Receive Freq LSB
    SS_RPWMan_Buf[EIGHTEEN]  		= HEX_THREE;					// Receive Freq MSB
    SS_RPWMan_Buf[NINETEEN] 		= HEX_ZERO;					// Receive Freq MSB  
    SS_RPWMan_Buf[TWENTY]  			= HEX_ZERO;			   		// Transmit Power Level
    SS_RPWMan_Buf[TWENTYONE]  		= HEX_ONE;	                                // FM Decrement					//Transmit Power Decrement
    SS_RPWMan_Buf[TWENTYTWO]  		= HEX_ONE;					// Channel/Hopset LSB
    SS_RPWMan_Buf[TWENTYTHREE]  	= HEX_ZERO;					// Channel/Hopset MSB
    SS_RPWMan_Buf[TWENTYFOUR]  		= HEX_ZERO;					// Guard Receiver Control
    SS_RPWMan_Buf[TWENTYFIVE]  		= HEX_ONE;					// Reserved3
    SS_RPWMan_Buf[TWENTYSIX]  		= HEX_ONE;					// Reserved3
    SS_RPWMan_Buf[TWENTYSEVEN]  	= HEX_ONE;					// Reserved3
    SS_RPWMan_Buf[TWENTYEIGHT]  	= HEX_ONE;					// Reserved3
    SS_RPWMan_Buf[TWENTYNINE]  		= HEX_ONE;					// Reserved3
    SS_RPWMan_Buf[THIRTY]  			= HEX_ONE;					// Reserved3
    SS_RPWMan_Buf[THIRTYONE]  		= HEX_ONE;					// Reserved3
    SS_RPWMan_Buf[THIRTYTWO]  		= HEX_ONE;					// Reserved3
    SS_RPWMan_Buf[THIRTYTHREE]  	= HEX_ONE;					// Reserved3
    SS_RPWMan_Buf[THIRTYFOUR]  		= HEX_ONE;					// Reserved3
    SS_RPWMan_Buf[THIRTYFIVE]  		= HEX_ONE;					// Reserved3
    SS_RPWMan_Buf[THIRTYSIX]  		= HEX_ONE;					// Reserved3
    SS_RPWMan_Buf[THIRTYSEVEN]  	= HEX_ZERO;					// HQ2 Net no. LSB
    SS_RPWMan_Buf[THIRTYEIGHT]  	= HEX_ZERO;					// HQ2 Net no. MSB
    SS_RPWMan_Buf[THIRTYNINE]  		= HEX_ZERO;					// HQ2 Net no. LSB
    SS_RPWMan_Buf[FOURTY]  			= HEX_ZERO;					// HQ2 Net no. MSB
    SS_RPWMan_Buf[FOURTYONE]  		= HEX_ZERO;					// Reserved4
    SS_RPWMan_Buf[FOURTYTWO]  		= HEX_ZERO;					// Reserved4
    SS_RPWMan_Buf[FOURTYTHREE]  	= HEX_ZERO;					// Reserved4
    SS_RPWMan_Buf[FOURTYFOUR]  		= HEX_ZERO;					// Reserved4
    SS_RPWMan_Buf[FOURTYFIVE]  		= HEX_ZERO;					// Reserved4
    SS_RPWMan_Buf[FOURTYSIX]  		= HEX_ZERO;					// Reserved4
    SS_RPWMan_Buf[FOURTYSEVEN]  	= HEX_ZERO;					// Reserved4
    SS_RPWMan_Buf[FOURTYEIGHT]  	= HEX_ZERO;					// Reserved4
    SS_RPWMan_Buf[FOURTYNINE]  		= HEX_ZERO;					// Reserved4
    SS_RPWMan_Buf[FIFTY]  			= HEX_ZERO;					// Reserved4
    SS_RPWMan_Buf[FIFTYONE]  		= HEX_ZERO;					// Reserved4
    SS_RPWMan_Buf[FIFTYTWO]  		= HEX_ZERO;					// Reserved4
    SS_RPWMan_Buf[FIFTYTHREE]  		= HEX_ZERO;					// Reserved4
    SS_RPWMan_Buf[FIFTYFOUR]  		= HEX_ZERO;					// Reserved4
    SS_RPWMan_Buf[FIFTYFIVE]  		= HEX_ZERO;					// Maritime table:0 = USA 1 = International
    SS_RPWMan_Buf[FIFTYSIX]  		= HEX_ZERO;					// Maritime Station:0=Ship
    SS_RPWMan_Buf[FIFTYSEVEN]  		= HEX_0x80;					// RPW Cue Freq. LSB
    SS_RPWMan_Buf[FIFTYEIGHT]  		= HEX_0xA9;					// RPW Cue Freq. LSB
    SS_RPWMan_Buf[FIFTYNINE]  		= HEX_THREE;					// RPW Cue Freq. MSB
    SS_RPWMan_Buf[SIXTY]  			= HEX_ZERO;					// RPW Cue Freq. MSB
    SS_RPWMan_Buf[SIXTYONE]  		= HEX_ONE;					// RPW Band:1 = Not Loaded, 3-UHF
    SS_RPWMan_Buf[SIXTYTWO]  		= HEX_ZERO;					// RPW Net No. LSB					// 0xBC
    SS_RPWMan_Buf[SIXTYTHREE]  		= HEX_ZERO;					// RPW Net No. MSB					// 0x01
    SS_RPWMan_Buf[SIXTYFOUR]  		= HEX_ZERO;					// Private Mode:0 = None
    SS_RPWMan_Buf[SIXTYFIVE]  		= HEX_ZERO;					// Encode Tone Freq. LSB
    SS_RPWMan_Buf[SIXTYSIX]  		= HEX_ZERO;					// Encode Tone Freq. MSB
    SS_RPWMan_Buf[SIXTYSEVEN]  		= HEX_ZERO;					// Decode Tone Freq. LSB
    SS_RPWMan_Buf[SIXTYEIGHT]  		= HEX_ZERO;					// Decode Tone Freq. MSB
    SS_RPWMan_Buf[SIXTYNINE]  		= HEX_ZERO;					// Reserved5
    SS_RPWMan_Buf[SEVENTY]  		= HEX_ZERO;					// Reserved5
    SS_RPWMan_Buf[SEVENTYONE]  		= HEX_ZERO;					// Reserved5
    SS_RPWMan_Buf[SEVENTYTWO] 	 	= HEX_ZERO;					// Reserved5
    SS_RPWMan_Buf[SEVENTYTHREE]  	= HEX_ZERO;					// Reserved5
    SS_RPWMan_Buf[SEVENTYFOUR]  	= HEX_ZERO;					// Reserved5
    SS_RPWMan_Buf[SEVENTYFIVE]  	= HEX_ZERO;					// Reserved5
    SS_RPWMan_Buf[SEVENTYSIX]  		= HEX_ZERO;					// Reserved5
    SS_RPWMan_Buf[SEVENTYSEVEN]  	= HEX_ZERO;					// Reserved5
    SS_RPWMan_Buf[SEVENTYEIGHT]  	= HEX_ZERO;					// Reserved5
    SS_RPWMan_Buf[SEVENTYNINE]  	= HEX_ZERO;					// Reserved5
    SS_RPWMan_Buf[EIGHTY]  			= HEX_ZERO;					// Reserved5
    SS_RPWMan_Buf[EIGHTYONE]  		= HEX_ZERO;					// Reserved5
    SS_RPWMan_Buf[EIGHTYTWO]  		= HEX_ZERO;					// Reserved5
    SS_RPWMan_Buf[EIGHTYTHREE]  	= HEX_ZERO;					// Reserved5
    SS_RPWMan_Buf[EIGHTYFOUR]  		= HEX_ZERO;					// Reserved5
    SS_RPWMan_Buf[EIGHTYFIVE]  		= HEX_ZERO;					// Reserved5
    SS_RPWMan_Buf[EIGHTYSIX]  		= HEX_ZERO;					// Reserved5
    SS_RPWMan_Buf[EIGHTYSEVEN]  	= HEX_ZERO;					// Reserved5
    SS_RPWMan_Buf[EIGHTYEIGHT]  	= HEX_ZERO;					// Reserved5
    SS_RPWMan_Buf[EIGHTYNINE]  		= HEX_ZERO;					// Reserved5
    SS_RPWMan_Buf[NINTY]  			= HEX_ZERO;					// Reserved5
    SS_RPWMan_Buf[NINTYONE]  		= HEX_ZERO;					// Reserved5
    SS_RPWMan_Buf[NINTYTWO]  		= HEX_ZERO;					// Reserved5
    SS_RPWMan_Buf[NINTYTHREE]  		= HEX_0xF8;					// RPW ManualFreq. LSB
    SS_RPWMan_Buf[NINTYFOUR]  		= HEX_0x95;					// RPW ManualFreq. LSB
    SS_RPWMan_Buf[NINTYFIVE]  		= HEX_THREE;					// RPW ManualFreq. MSB
    SS_RPWMan_Buf[NINTYSIX]  		= HEX_ZERO;					// RPW ManualFreq. MSB
    SS_RPWMan_Buf[NINTYSEVEN]  		= SS_RPWMan_NetMaster;			// RPW Net Master 0: Disabled 1:Enabled
    SS_RPWMan_Buf[NINTYEIGHT]  		= HEX_ZERO;					// RPW Enable Cue:0 = Disabled
    SS_RPWMan_Buf[NINTYNINE]  		= HEX_ZERO;					// Reserved6
    SS_RPWMan_Buf[HUNDRED] 			= HEX_ONE;					// RPW Mode:0 = Cold Start,1 = manual,2-Cue,3-FH
    SS_RPWMan_Buf[HUNDREDANDONE] 	= HEX_ZERO;					// I/F Filter Selection:0 = Normal
    SS_RPWMan_Buf[HUNDREDANDTWO] 	= HEX_ONE;					// ADM Mode:0 = Enabled;1-Disabled(normal mode)
    SS_RPWMan_Buf[HUNDREDANDTHREE] 	= HEX_ONE;					// ADM ID
    SS_RPWMan_Buf[HUNDREDANDFOUR] 	= HEX_ONE;					// Reserved7
    SS_RPWMan_Buf[HUNDREDANDFIVE] 	= HEX_ZERO;					// ATC Mode Selected:0=Disabled
    SS_RPWMan_Buf[HUNDREDANDSIX] 	= HEX_ONE;					// ADM Quiet Mode:0 = Enabled
    SS_RPWMan_Buf[HUNDREDANDSEVEN] 	= HEX_ONE;					// CTCSS Tone Mode:0 = Non Std.
    SS_RPWMan_Buf[HUNDREDANDEIGHT] 	= HEX_0x58;					// Non. Std. Encode Tone Freq LSB
    SS_RPWMan_Buf[HUNDREDANDNINE] 	= HEX_TWO;					// Non. Std. Encode Tone Freq MSB
    SS_RPWMan_Buf[HUNDREDANDTEN] 	= HEX_0x58;					// Non. Std. Decode Tone Freq LSB
    SS_RPWMan_Buf[HUNDREDANDELEVEN] = HEX_TWO;					// Non. Std. Decode Tone Freq MSB
    SS_RPWMan_Buf[HUNDREDANDTWELVE] = HEX_ONE;					// Reserved8
    SS_RPWMan_Buf[HUNDREDANDTHIRTEEN] = HEX_ONE;				// Reserved8
    SS_RPWMan_Buf[HUNDREDANDFOURTEEN] = HEX_ONE;				// Reserved8									
    checksum(SS_RPWMan_Buf,SS_RPWMan_Buf[THREE]);	
    memcpy(SS_RPWMan_RadioPack, SS_RPWMan_Buf, DEC_117);	
    return(SS_RPWMan_RadioPack);	
}


/** FHdr-beg *****************************************************
**
** Function name: f_volume
**
** Anchor:   		   
**
** Purpose:        This fiunction will check the radio packet volume 
**
**
** Inputs:         Volume radio packet   information , volume  radio  packet   
* Vol_Buf[0]	= 0xAA;				// Synchronization 	
* Vol_Buf[1]	= 0x20;				// Packet ID 		
* Vol_Buf[2]	= 00;				// Packet Length_MSB 	
* Vol_Buf[3]	= 07;				// Packet Length_LSB 				
* Vol_Buf[4]	= 144;		                // Volume Level 
* Vol_Buf[5]	= 0;                            // CRC
* Vol_Buf[6]	= 0;    		        // CRC 


** Outputs:       volume radio packet  
**
** FPrj-beg *****************************************************
**
** FPrj-end *****************************************************
** FHdr-end *****************************************************/


uint8_t *f_volume(uint8_t Vol_RadioPackInfo, uint8_t *Vol_RadioPack)
{
    uint8_t Vol_Buf[SEVEN] = {HEX_0xAA, HEX_0x20, HEX_ZERO, HEX_SEVEN, DEC_144, HEX_ZERO, HEX_ZERO};
    checksum(Vol_Buf, Vol_Buf[THREE]);
    memcpy(Vol_RadioPack, Vol_Buf, HEX_SEVEN);
    return(Vol_RadioPack);				
}


/** FHdr-beg *****************************************************
**
** Function name: f_th_squelch
**
** Anchor:   		   
**
** Purpose:        This fiunction will sequence  the channel 
**
**
** Inputs:         Sq radio packet   information , Sq radio  packet    
** TH_Sq_Buf[0] = 0xAA;				// Synchronization 		
* TH_Sq_Buf[1] = 0x21;				// Packet ID 			
* TH_Sq_Buf[2] = 00;				// Packet Length_MSB 	
* TH_Sq_Buf[3] = 07;				// Packet Length_LSB 				
* TH_Sq_Buf[4] = TH_Sq_RadioPackInfo[1];		// TH_Sq_Buf Level 
* TH_Sq_Buf[5] = 0;                                // CRC	
* TH_Sq_Buf[6] = 0;	                        // CRC
** Outputs:       sq radio  packet 
**
** FPrj-beg *****************************************************
**
** FPrj-end *****************************************************
** FHdr-end *****************************************************/

uint8_t *f_th_squelch(uint8_t *TH_Sq_RadioPackInfo, uint8_t *TH_Sq_RadioPack)
{
    uint8_t TH_Sq_Buf[SEVEN] = {HEX_0xAA, HEX_0x21, HEX_ZERO, HEX_SEVEN, HEX_ZERO, HEX_ZERO, HEX_ZERO};	
    TH_Sq_Buf[FOUR] = TH_Sq_RadioPackInfo[ONE];		/* TH_Sq_Buf Level */			
    checksum(TH_Sq_Buf, TH_Sq_Buf[THREE]);	
    memcpy(TH_Sq_RadioPack, TH_Sq_Buf, HEX_SEVEN);	
    return(TH_Sq_RadioPack);				
}


/** FHdr-beg *****************************************************
**
** Function name: f_status_request 
**
** Anchor:   		   
**
** Purpose:        This fiunction  will request radio status 
**
**
** Inputs:         status request radio packet   information , status request radio  packet    
* StatusReq_Buf[0] = 0xAA;				// Synchronization 		
* StatusReq_Buf[1] = 0x22;				// Packet ID
* StatusReq_Buf[2] = 00;				// Packet Length_MSB
* StatusReq_Buf[3] = 07;				// Packet Length_LSB
* StatusReq_Buf[4] = StatusReq_RadioPackInfo[1];	        // StatusRequest Type  21: Sqelch settings, 17: Radio Config  
* StatusReq_Buf[5] = 0;                                // CRC
* StatusReq_Buf[6] = 0;                                // CRCCRC	

** Outputs:        radio  packet status 
**
** FPrj-beg *****************************************************
**
** FPrj-end *****************************************************
** FHdr-end *****************************************************/


uint8_t *f_status_request(uint8_t StatusReq_RadioPackInfo, uint8_t *StatusReq_RadioPack)
{	
    uint8_t StatusReq_Buf[SEVEN] = {HEX_0xAA, HEX_0x22, HEX_ZERO, HEX_SEVEN, HEX_ZERO, HEX_ZERO, HEX_ZERO};				
    StatusReq_Buf[FOUR] = StatusReq_RadioPackInfo;	            /* StatusRequest Type  21: Sqelch settings, 17: Radio Config */	
    checksum(StatusReq_Buf, StatusReq_Buf[THREE]);
    memcpy(StatusReq_RadioPack, StatusReq_Buf, HEX_SEVEN);
    return(StatusReq_RadioPack);
}


/** FHdr-beg *****************************************************
**
** Function name: f_ackpack_toradio 
**
** Anchor:   		   
**
** Purpose:        This fiunction  will ack to radio 
**
**
** Inputs:         ackt radio packet   information ,ack t radio  packet    
* Ack_Buf[0] = 0xAA;						
* Ack_Buf[1] = 0x01;				
* Ack_Buf[2] = 00;				
* Ack_Buf[3] = 07;				
* Ack_Buf[4] = Ack_RadioPackInfo[1];	        
* Ack_Buf[5] = 0;                            // CRC
* Ack_Buf[6] = 0;                            // CRC
** Outputs:        radio  packet status 
**
** FPrj-beg *****************************************************
**
** FPrj-end *****************************************************
** FHdr-end *****************************************************/


uint8_t *f_ackpack_toradio(uint8_t Ack_RadioPackInfo, uint8_t *Ack_RadioPack)
{
    uint8_t Ack_Buf[SEVEN] = {HEX_0xAA, HEX_ONE, HEX_ZERO, HEX_SEVEN, HEX_ZERO, HEX_ZERO, HEX_ZERO};
    Ack_Buf[FOUR] = Ack_RadioPackInfo;		
    checksum(Ack_Buf, Ack_Buf[THREE]);	
    memcpy(Ack_RadioPack, Ack_Buf, HEX_SEVEN);		
    return(Ack_RadioPack);	
}

/** FHdr-beg *****************************************************
**
** Function name: f_emergency_guard 
**
** Anchor:   		   
**
** Purpose:        This fiunction is f_emergency guard for radio 
**
**
** Inputs:         emergency radio packet   information ,emergency radio  packet    
* EMG_Buf[0] = 0xAA;				//synchronisation						
* EMG_Buf[1] = 0x27;				//Packet_id				
* EMG_Buf[2] = 00;				//Packet_length_MSB				
* EMG_Buf[3] = 7;					//Packet_length_LSB				
* EMG_Buf[4] = EMG_RadioPackInfo[1];		//Control: 0-Emergency Off;2-Emergency On
* EMG_Buf[4] = 0;                                 // CRC
* EMG_Buf[4] = 0;                                 // CRC
** Outputs:        emergency radio  packet  
**
** FPrj-beg *****************************************************
**
** FPrj-end *****************************************************
** FHdr-end *****************************************************/


uint8_t *f_emergency_guard(uint8_t *EMG_RadioPackInfo, uint8_t *EMG_RadioPack)
{
    uint8_t EMG_Buf[SEVEN] = {HEX_0xAA, HEX_0X27, HEX_ZERO, HEX_SEVEN, HEX_ZERO, HEX_ZERO, HEX_ZERO};	
    EMG_Buf[FOUR] = EMG_RadioPackInfo[ONE];		//Control: 0-Emergency Off;2-Emergency On		
    checksum(EMG_Buf,EMG_Buf[HEX_THREE]);	
    memcpy(EMG_RadioPack, EMG_Buf, HEX_SEVEN); 
    return(EMG_RadioPack);
}




/** FHdr-beg *****************************************************
**
** Function name: f_store_setup
**
** Anchor:   		   
**
** Purpose:        This fiunction will store setup information 
**
**
** Inputs:         SS radio packet   information , SS radio  packet    
**
**
** Outputs:       SS radio  packet
**
** FPrj-beg *****************************************************
**
** FPrj-end *****************************************************
** FHdr-end *****************************************************/



uint8_t *f_store_setup(uint8_t *SS_RadioPackInfo, uint8_t *SS_RadioPack)
{	
    uint8_t SS_Buf[HUNDREDANDSEVENTEEN]		 = "";
 	 
    /**************	Inialising values to the STORE_SETUP Packet **********/
	
    SS_Buf[ZERO]   					= HEX_0xAA;							//Synchronization
    SS_Buf[ONE]   					= HEX_0X25;							//Packet ID
    SS_Buf[TWO]   					= HEX_ZERO;							//Packet Length MSB
    SS_Buf[THREE]   				= DEC_117;							//Packet Length LSB	
    SS_Buf[FOUR]   					= HEX_ZERO;							//Mode 0=LOS
    SS_Buf[FIVE]   					= HEX_ONE;							//ID 0=Current Preset
    SS_Buf[SIX]   					= HEX_THREE;							//Variant 3=Normal 	
    SS_Buf[SEVEN]   				= SS_RadioPackInfo[HEX_ONE];		        		//Modulation 0=AM and 1=FM
    SS_Buf[EIGHT]   				= HEX_ZERO;							//Reserved1
    SS_Buf[NINE]   					= HEX_ONE;							//Reserved2
    SS_Buf[TEN]  					= HEX_ZERO;							//Source 0=Voice
    SS_Buf[ELEVEN]  				= HEX_E;					            	//Preset I/O Data Rate 11=9600bps	
    SS_Buf[TWELVE]  				= SS_RadioPackInfo[TWO];	//recvd_param[1];		//Transmit Freq LSB=260MHz
    SS_Buf[THIRTEEN]  				= SS_RadioPackInfo[THREE];	//recvd_param[2];		//Transmit Freq LSB
    SS_Buf[FOURTEEN]  				= SS_RadioPackInfo[FOUR];	//recvd_param[3];		//Transmit Freq MSB
    SS_Buf[FIFTEEN]  				= SS_RadioPackInfo[FIVE];	//recvd_param[4];		//Transmit Freq MSB
    SS_Buf[SIXTEEN]  				= SS_RadioPackInfo[TWO];	//recvd_param[1];		//Receive Freq LSB=260MHz
    SS_Buf[SEVENTEEN] 				= SS_RadioPackInfo[THREE];	//recvd_param[2];		//Receive Freq LSB
    SS_Buf[EIGHTEEN]  				= SS_RadioPackInfo[FOUR];	//recvd_param[3];		//Receive Freq MSB
    SS_Buf[NINETEEN]  				= SS_RadioPackInfo[FIVE];	//recvd_param[4];		//Receive Freq MSB  
    SS_Buf[TWENTY]  				= HEX_ZERO;			   				//Transmit Power Level
    SS_Buf[TWENTYONE]  				= SS_RadioPackInfo[SIX];		    			//Transmit Power Decrement  
    SS_Buf[TWENTYTWO]  				= HEX_ONE;							//Channel/Hopset LSB
    SS_Buf[TWENTYTHREE]  			= HEX_ZERO;							// Channel/Hopset MSB
    SS_Buf[TWENTYFOUR]  			= SS_RadioPackInfo[SEVEN];	//recvd_param[7];		//Guard Receiver Control:-0-off;1-on;
    SS_Buf[TWENTYFIVE]  			= HEX_ONE;							//Reserved3
    SS_Buf[TWENTYSIX]  				= HEX_ONE;							//Reserved3
    SS_Buf[TWENTYSEVEN]  			= HEX_ONE;							//Reserved3
    SS_Buf[TWENTYEIGHT]  			= HEX_ONE;							//Reserved3
    SS_Buf[TWENTYNINE]  			= HEX_ONE;							//Reserved3
    SS_Buf[THIRTY]  				= HEX_ONE;							//Reserved3
    SS_Buf[THIRTYONE]  				= HEX_ONE;							//Reserved3
    SS_Buf[THIRTYTWO]  				= HEX_ONE;							//Reserved3
    SS_Buf[THIRTYTHREE]  			= HEX_ONE;							//Reserved3
    SS_Buf[THIRTYFOUR]  			= HEX_ONE;							//Reserved3
    SS_Buf[THIRTYFIVE]  			= HEX_ONE;							//Reserved3
    SS_Buf[THIRTYSIX]  				= HEX_ONE;							//Reserved3
    SS_Buf[THIRTYSEVEN]  			= HEX_ZERO;							//HQ2 Net no. LSB
    SS_Buf[THIRTYEIGHT]  			= HEX_ZERO;							//HQ2 Net no. LSB
    SS_Buf[THIRTYNINE]  			= HEX_ZERO;							//HQ2 Net no. MSB
    SS_Buf[FOURTY]  				= HEX_ZERO;							//HQ2 Net no. MSB
    SS_Buf[FOURTYONE]  				= HEX_ZERO;							//Reserved4
    SS_Buf[FOURTYTWO]  				= HEX_ZERO;							//Reserved4
    SS_Buf[FOURTYTHREE]  			= HEX_ZERO;							//Reserved4
    SS_Buf[FOURTYFOUR]  			= HEX_ZERO;							//Reserved4
    SS_Buf[FOURTYFIVE]  			= HEX_ZERO;							//Reserved4
    SS_Buf[FOURTYSIX]  				= HEX_ZERO;							//Reserved4
    SS_Buf[FOURTYSEVEN]  			= HEX_ZERO;							//Reserved4
    SS_Buf[FOURTYEIGHT]  			= HEX_ZERO;							//Reserved4
    SS_Buf[FOURTYNINE]  			= HEX_ZERO;							//Reserved4
    SS_Buf[FIFTY]  					= HEX_ZERO;							//Reserved4
    SS_Buf[FIFTYONE]  				= HEX_ZERO;							//Reserved4
    SS_Buf[FIFTYTWO]  				= HEX_ZERO;							//Reserved4
    SS_Buf[FIFTYTHREE]  			= HEX_ZERO;							//Reserved4
    SS_Buf[FIFTYFOUR]  				= HEX_ZERO;							//Reserved4
    SS_Buf[FIFTYFIVE]  				= HEX_ONE;							//Maritime table:1=International
    SS_Buf[FIFTYSIX]  				= HEX_ZERO;							//Maritime Station:0=Ship
    SS_Buf[FIFTYSEVEN]  			= HEX_0x30;							//RPW Cue Freq. LSB
    SS_Buf[FIFTYEIGHT]  			= HEX_0x75;							//RPW Cue Freq. LSB
    SS_Buf[FIFTYNINE]  				= HEX_ZERO;							//RPW Cue Freq. MSB
    SS_Buf[SIXTY]  					= HEX_ZERO;							//RPW Cue Freq. MSB
    SS_Buf[SIXTYONE]  				= HEX_ONE;							//RPW Band:1=Not Loaded
    SS_Buf[SIXTYTWO]  				= HEX_ZERO;							//RPW Net No. MSB
    SS_Buf[SIXTYTHREE]  			= HEX_ZERO;							//RPW Net No. LSB
    SS_Buf[SIXTYFOUR]  				= HEX_ZERO;							//Private Mode:0=None
    SS_Buf[SIXTYFIVE]  				= HEX_ZERO;							//Encode Tone Freq. LSB
    SS_Buf[SIXTYSIX]  				= HEX_ZERO;							//Encode Tone Freq. LSB
    SS_Buf[SIXTYSEVEN]  			= HEX_ZERO;							//Decode Tone Freq. MSB
    SS_Buf[SIXTYEIGHT]  			= HEX_ZERO;							//Decode Tone Freq. MSB
    SS_Buf[SIXTYNINE]  				= HEX_ZERO;							//Reserved5
    SS_Buf[SEVENTY]  				= HEX_ZERO;							//Reserved5
    SS_Buf[SEVENTYONE]  			= HEX_ZERO;							//Reserved5
    SS_Buf[SEVENTYTWO]  			= HEX_ZERO;							//Reserved5
    SS_Buf[SEVENTYTHREE]  			= HEX_ZERO;							//Reserved5
    SS_Buf[SEVENTYFOUR]  			= HEX_ZERO;							//Reserved5
    SS_Buf[SEVENTYFIVE]  			= HEX_ZERO;							//Reserved5
    SS_Buf[SEVENTYSIX]  			= HEX_ZERO;							//Reserved5
    SS_Buf[SEVENTYSEVEN]  			= HEX_ZERO;							//Reserved5
    SS_Buf[SEVENTYEIGHT]  			= HEX_ZERO;							//Reserved5
    SS_Buf[SEVENTYNINE]  			= HEX_ZERO;							//Reserved5
    SS_Buf[EIGHTY]  				= HEX_ZERO;							//Reserved5
    SS_Buf[EIGHTYONE]  				= HEX_ZERO;							//Reserved5
    SS_Buf[EIGHTYTWO]  				= HEX_ZERO;							//Reserved5
    SS_Buf[EIGHTYTHREE]  			= HEX_ZERO;							//Reserved5
    SS_Buf[EIGHTYFOUR]  			= HEX_ZERO;							//Reserved5
    SS_Buf[EIGHTYFIVE]  			= HEX_ZERO;							//Reserved5
    SS_Buf[EIGHTYSIX]  				= HEX_ZERO;							//Reserved5
    SS_Buf[EIGHTYSEVEN]  			= HEX_ZERO;							//Reserved5
    SS_Buf[EIGHTYEIGHT]  			= HEX_ZERO;							//Reserved5
    SS_Buf[EIGHTYNINE]  			= HEX_ZERO;							//Reserved5
    SS_Buf[NINTY]  					= HEX_ZERO;							//Reserved5
    SS_Buf[NINTYONE]  				= HEX_ZERO;							//Reserved5
    SS_Buf[NINTYTWO]  				= HEX_ZERO;							//Reserved5
    SS_Buf[NINTYTHREE]  			= HEX_0x30;							//RPW ManualFreq. LSB
    SS_Buf[NINTYFOUR]  				= HEX_0x75;							//RPW ManualFreq. LSB
    SS_Buf[NINTYFIVE]  				= HEX_ZERO;							//RPW ManualFreq. MSB
    SS_Buf[NINTYSIX]  				= HEX_ZERO;							//RPW ManualFreq. MSB
    SS_Buf[NINTYSEVEN]  			= HEX_ZERO;							//RPW Net No. 0: Disabled
    SS_Buf[NINTYEIGHT]  			= HEX_ZERO;							//RPW Enable Cue:0=Disabled
    SS_Buf[NINTYNINE]  				= HEX_ZERO;							//Reserved6
    SS_Buf[HUNDRED] 				= HEX_ZERO;							//RPW Mode:0=Cold Start, 1=manual
    SS_Buf[HUNDREDANDONE] 			= HEX_ZERO;							//I/F Filter Selection:0=Normal
    SS_Buf[HUNDREDANDTWO] 			= HEX_ONE;							//ADM Mode:0=Enabled
    SS_Buf[HUNDREDANDTHREE] 		= HEX_ONE;							//ADM ID
    SS_Buf[HUNDREDANDFOUR] 			= HEX_ONE;							//Reserved7
    SS_Buf[HUNDREDANDFIVE] 			= SS_RadioPackInfo[EIGHT];                   			//ATC Mode Selected:0=Disabled
    SS_Buf[HUNDREDANDSIX] 			= HEX_ONE;					 		//ADM Quiet Mode:0=Enabled
    SS_Buf[HUNDREDANDSEVEN] 		= HEX_ONE;							//CTCSS Tone Mode:0=Non Std.
    SS_Buf[HUNDREDANDEIGHT] 		= HEX_0x58;							//Non. Std. Encode Tone Freq LSB
    SS_Buf[HUNDREDANDNINE] 			= HEX_TWO;							//Non. Std. Encode Tone Freq LSB
    SS_Buf[HUNDREDANDTEN] 			= HEX_0x58;							//Non. Std. Decode Tone Freq MSB
    SS_Buf[HUNDREDANDELEVEN] 		= HEX_TWO;							//Non. Std. Decode Tone Freq MSB
    SS_Buf[HUNDREDANDTWELVE] 		= HEX_ONE;							//Reserved8
    SS_Buf[HUNDREDANDTHIRTEEN] 		= HEX_ONE;							//Reserved8
    SS_Buf[HUNDREDANDFOURTEEN] 		= HEX_ONE;							//Reserved8								
    /*	calling Checksum() function for CRC values i.e SS_Buf[115], SS_Buf[116]	*/	
    checksum(SS_Buf, SS_Buf[THREE]);	
    memcpy(SS_RadioPack, SS_Buf, DEC_117);	
    return(SS_RadioPack);
}


/** FHdr-beg *****************************************************
**
** Function name: f_preset_request
**
** Anchor:   		   
**
** Purpose:        This fiunction will perform preset request 
**
**
** Inputs:         PR radio packet   information , PR radio  packet    
* PR_Buf[0] = 0xAA;				// synchronisation
* PR_Buf[1] = 0x2B;				// Packet_id
* PR_Buf[2] = 00;				// Packet_length_MSB
* PR_Buf[3] = 9;				// Packet_length_LSB
* PR_Buf[4] = 0;				// Mode ID
* PR_Buf[5] = PR_RadioPackInfo[1];		// Variant
* PR_Buf[6] = 1;				// Preset ID
* PR_Buf[7] = 0;				// CRC 
* PR_Buf[8] = 0;				// CRC
** Outputs:       PR radio  packet
**
** FPrj-beg *****************************************************
**
** FPrj-end *****************************************************
** FHdr-end *****************************************************/



uint8_t *f_preset_request(uint8_t PR_RadioPackInfo, uint8_t *PR_RadioPack)
{	
    uint8_t PR_Buf[NINE] = {HEX_0xAA, HEX_0X2B, HEX_ZERO, HEX_NINE, HEX_ZERO, HEX_ZERO, HEX_ONE, HEX_ZERO, HEX_ZERO};
    PR_Buf[FIVE] = PR_RadioPackInfo;														
    checksum(PR_Buf, PR_Buf[THREE]);	
    memcpy(PR_RadioPack, PR_Buf, HEX_NINE);		
    return(PR_RadioPack);
}


/** FHdr-beg *****************************************************
**
** Function name: f_hq_command
**
** Anchor:   		   
**
** Purpose:        This fiunction will perform have quick command 
**
**
** Inputs:         have quick  radio packet   information , have quick  radio  packet    
* HaveQuick_Buf[0] = 0xAA;       		    // SYNC
* HaveQuick_Buf[1] = 0x61;			    // Pkt ID 	
* HaveQuick_Buf[2] = 0;				    // Pkt len MSB 
* HaveQuick_Buf[3] = 8;			            // Pkt len LSB
* HaveQuick_Buf[4] = HQC_RadioPackInfo[1];	    // Type 
* HaveQuick_Buf[5] = HQC_RadioPackInfo[2];          // Data
* HaveQuick_Buf[6] = 0;                             // CRC
* HaveQuick_Buf[7] = 0;                             // CRC
** Outputs:       have quick  radio  packet
**
** FPrj-beg *****************************************************
**
** FPrj-end *****************************************************
** FHdr-end *****************************************************/




uint8_t *f_hq_command(uint8_t *HQC_RadioPackInfo, uint8_t *HaveQuick_RadioPack)
{
    uint8_t HaveQuick_Buf[EIGHT] = {HEX_0xAA, HEX_0x61, HEX_ZERO, HEX_EIGHT, HEX_EIGHT, HEX_ZERO, HEX_ZERO, HEX_ZERO};	
    HaveQuick_Buf[FOUR] = HQC_RadioPackInfo[ONE];					/* Type */
    HaveQuick_Buf[FIVE] = HQC_RadioPackInfo[TWO];
    checksum(HaveQuick_Buf, HaveQuick_Buf[THREE]);		
    memcpy(HaveQuick_RadioPack, HaveQuick_Buf, HEX_EIGHT);	
    return(HaveQuick_RadioPack);
}


/** FHdr-beg *****************************************************
**
** Function name: f_rpw_transec_erase
**
** Anchor:   		   
**
** Purpose:        This fiunction will perform rpw transec erase
**
**
** Inputs:         RPW radio packet   information , RPW radio  packet    
* RPW_TransecEarse_Buf[0]	= 0xAA;				// Synchronization 		
* RPW_TransecEarse_Buf[1]	= 0x6D;				// Packet ID 			
* RPW_TransecEarse_Buf[2]	= 0;				// Packet Length_MSB 	
* RPW_TransecEarse_Buf[3]	= 10;				// Packet Length_LSB 				
* RPW_TransecEarse_Buf[4]	= 6;				// Operating Mode  5-Time Equalization, 6-ERASE TRANSEC   	
* RPW_TransecEarse_Buf[5]	= 1;				// Hopset ID    	
* RPW_TransecEarse_Buf[6]	= 1;				// Band Identifier-> 1: Not Applicable, 2:VHF, 3: UHF  		
* RPW_TransecEarse_Buf[7]	= 1;		                // 0 = Reject, 1 = Accept 
* RPW_TransecEarse_Buf[8]	= 0;                            // CRC
* RPW_TransecEarse_Buf[9]	= 0;	                        // CRC
** Outputs:       RPW radio  packet
**
** FPrj-beg *****************************************************
**
** FPrj-end *****************************************************
** FHdr-end *****************************************************/


uint8_t *f_rpw_transec_erase(uint8_t RPW_Erase_RadioPackInfo, uint8_t *RPW_Erase_RadioPack)
{
    uint8_t RPW_TransecEarse_Buf[TEN] = {HEX_0xAA, HEX_0x6D, HEX_ZERO, DEC_10, HEX_SIX, HEX_ONE, HEX_ONE, HEX_ONE, HEX_ZERO, HEX_ZERO};	
    checksum(RPW_TransecEarse_Buf, RPW_TransecEarse_Buf[THREE]);	
    memcpy(RPW_Erase_RadioPack, RPW_TransecEarse_Buf, DEC_10);	
    return(RPW_Erase_RadioPack);
}


/** FHdr-beg *****************************************************
**
** Function name: f_pps_disable
**
** Anchor:   		   
**
** Purpose:        This fiunction will disable pps
**
**
** Inputs:        PPS disable  packet   
*  pps_disable_buf[0]  = 0xAA;   //  synchronization
*  pps_disable_buf[1]  = 0xAE;   //  packet-id
*  pps_disable_buf[2]  = 00;     //  packet length_msb
*  pps_disable_buf[3]  = 16;     //  packet length_lsb
*  pps_disable_buf[4]  = 0xC4;   //  year_lsb
*  pps_disable_buf[5]  = 0x07;   //  year_msb
*  pps_disable_buf[6]  = 0x3A;   //  days_lsb
*  pps_disable_buf[7]  = 0;      //  days_msb
*  pps_disable_buf[8]  = 22;     //  hours
*  pps_disable_buf[9]  = 40;     //  mins
*  pps_disable_buf[10] = 20;     //  seconds
*  pps_disable_buf[11] = 0;      //  mode
*  pps_disable_buf[12] = 0;      //  external 1 pps clock signal
*  pps_disable_buf[13] = 0;      //  reset time base
*  pps_disable_buf[14] = 0;      //  CRC
*  pps_disable_buf[15] = 0;      //  CRC
** Outputs:       PPS disable  radio packet   
**
** FPrj-beg *****************************************************
**
** FPrj-end *****************************************************
** FHdr-end *****************************************************/



uint8_t *f_pps_disable(uint8_t *pps_disable_radiopack)
{
    uint8_t pps_disable_buf[SIXTEEN] = {HEX_0xAA, HEX_0xAE, HEX_ZERO, DEC_16, HEX_0xC4, HEX_SEVEN, HEX_0x3A, HEX_ZERO, DEC_22, DEC_40, DEC_20, HEX_ZERO, HEX_ZERO, HEX_ZERO, HEX_ZERO, HEX_ZERO};
    checksum(pps_disable_buf, pps_disable_buf[THREE]);
    memcpy(pps_disable_radiopack, pps_disable_buf, DEC_16);
    return(pps_disable_radiopack);

}


/** FHdr-beg *****************************************************
**
** Function name: f_timeequalization
**
** Anchor:   		   
**
** Purpose:        This fiunction will perform timer equalization
**
**
** Inputs:         timer equalization radio  packet    
* time_equalize_buf[0] = 0xAA;    // synchronization
* time_equalize_buf[1] = 0x6D;    // packet id
* time_equalize_buf[2] = 0;       // packet length msb
* time_equalize_buf[3] = 10;      // packet length lsb
* time_equalize_buf[4] = 5;       // operate mode 5-time equalization, 6-erase transec
* time_equalize_buf[5] = 1;       // variant 3 for LOS
* time_equalize_buf[6] = 1;       // preset number
* time_equalize_buf[7] = 1;
* time_equalize_buf[8] = 0;       // CRC
* time_equalize_buf[9] = 0;       // CRC
** Outputs:       timer equalization radio packet 
**
** FPrj-beg *****************************************************
**
** FPrj-end *****************************************************
** FHdr-end *****************************************************/



uint8_t *f_timeequalization(uint8_t *time_equalise_radiopack)
{
    uint8_t time_equalize_buf[TEN] = {HEX_0xAA, HEX_0x6D, HEX_ZERO, DEC_10, HEX_FIVE, HEX_ONE, HEX_ONE, HEX_ONE, HEX_ZERO, HEX_ZERO};
    checksum(time_equalize_buf, time_equalize_buf[THREE]);
    memcpy(time_equalise_radiopack, time_equalize_buf, DEC_10);
    return(time_equalise_radiopack);
}


/** FHdr-beg *****************************************************
**
** Function name: f_rpw_load_kfd
**
** Anchor:   		   
**
** Purpose:        This fiunction will perform RPW laod function for KFD
**
**
** Inputs:         KFD radio packet   information , RPW radio  packet    
**
**
** Outputs:       RPM and KFD radio  
**
** FPrj-beg *****************************************************
**
** FPrj-end *****************************************************
** FHdr-end *****************************************************/



uint8_t *f_rpw_load_kfd(uint8_t *KFD_RadioPackInfo, uint8_t *RPW_KFD_RadioPack)
{
    uint8_t RPW_KFD_Buf[SEVENTYFOUR] = "";    	
    /* RPW - KeyFill Data - for Loading Hopsets */		
    RPW_KFD_Buf[ZERO]  			= HEX_0xAA;				// Synchronization 		
    RPW_KFD_Buf[ONE]  			= HEX_0x6F;				// Packet ID 			
    RPW_KFD_Buf[TWO]  			= HEX_ZERO;				// Packet Length_MSB 	
    RPW_KFD_Buf[THREE]  		= DEC_74;				// Packet Length_LSB 				
    RPW_KFD_Buf[FOUR]  			= HEX_ZERO;				// Type 0-Hopset    	
    RPW_KFD_Buf[FIVE]  			= KFD_RadioPackInfo[TWO];				// Position 0-Cold Start		     	
    RPW_KFD_Buf[SIX]  			= KFD_RadioPackInfo[THREE];				// No. of Cells     		
    RPW_KFD_Buf[SEVEN]  		= HEX_ZERO;                                //Reserved
    RPW_KFD_Buf[EIGHT]  		= KFD_RadioPackInfo[FOUR];             //cell 1 data (16 bytes)
    RPW_KFD_Buf[NINE]  			= KFD_RadioPackInfo[FIVE];
    RPW_KFD_Buf[TEN] 			= KFD_RadioPackInfo[SIX];
    RPW_KFD_Buf[ELEVEN] 		= KFD_RadioPackInfo[SEVEN];
    RPW_KFD_Buf[TWELVE] 		= KFD_RadioPackInfo[EIGHT];
    RPW_KFD_Buf[THIRTEEN] 		= KFD_RadioPackInfo[NINE];	
    RPW_KFD_Buf[FOURTEEN] 		= KFD_RadioPackInfo[TEN];
    RPW_KFD_Buf[FIFTEEN] 		= KFD_RadioPackInfo[ELEVEN];
    RPW_KFD_Buf[SIXTEEN] 		= KFD_RadioPackInfo[TWELVE];
    RPW_KFD_Buf[SEVENTEEN] 		= KFD_RadioPackInfo[THIRTEEN];
    RPW_KFD_Buf[EIGHTEEN] 		= KFD_RadioPackInfo[FOURTEEN];
    RPW_KFD_Buf[NINETEEN] 		= KFD_RadioPackInfo[FIFTEEN];
    RPW_KFD_Buf[TWENTY] 		= KFD_RadioPackInfo[SIXTEEN];
    RPW_KFD_Buf[TWENTYONE] 		= KFD_RadioPackInfo[SEVENTEEN];
    RPW_KFD_Buf[TWENTYTWO] 		= KFD_RadioPackInfo[EIGHTEEN];
    RPW_KFD_Buf[TWENTYTHREE] 	= KFD_RadioPackInfo[NINETEEN];
    RPW_KFD_Buf[TWENTYFOUR] 	= KFD_RadioPackInfo[TWENTY];                 // Cell 2 data (16 bytes)
    RPW_KFD_Buf[TWENTYFIVE] 	= KFD_RadioPackInfo[TWENTYONE];
    RPW_KFD_Buf[TWENTYSIX] 		= KFD_RadioPackInfo[TWENTYTWO];
    RPW_KFD_Buf[TWENTYSEVEN] 	= KFD_RadioPackInfo[TWENTYTHREE];
    RPW_KFD_Buf[TWENTYEIGHT] 	= KFD_RadioPackInfo[TWENTYFOUR];
    RPW_KFD_Buf[TWENTYNINE] 	= KFD_RadioPackInfo[TWENTYFIVE];
    RPW_KFD_Buf[THIRTY] 		= KFD_RadioPackInfo[TWENTYSIX];
    RPW_KFD_Buf[THIRTYONE] 		= KFD_RadioPackInfo[TWENTYSEVEN];
    RPW_KFD_Buf[THIRTYTWO] 		= KFD_RadioPackInfo[TWENTYEIGHT];
    RPW_KFD_Buf[THIRTYTHREE] 	= KFD_RadioPackInfo[TWENTYNINE];
    RPW_KFD_Buf[THIRTYFOUR] 	= KFD_RadioPackInfo[THIRTY];
    RPW_KFD_Buf[THIRTYFIVE] 	= KFD_RadioPackInfo[THIRTYONE];
    RPW_KFD_Buf[THIRTYSIX] 		= KFD_RadioPackInfo[THIRTYTWO];
    RPW_KFD_Buf[THIRTYSEVEN] 	= KFD_RadioPackInfo[THIRTYTHREE];
    RPW_KFD_Buf[THIRTYEIGHT] 	= KFD_RadioPackInfo[THIRTYFOUR];
    RPW_KFD_Buf[THIRTYNINE] 	= KFD_RadioPackInfo[THIRTYFIVE];
    RPW_KFD_Buf[FOURTY] 		= KFD_RadioPackInfo[THIRTYSIX];                  // Cell 3 Data (16 bytes)
    RPW_KFD_Buf[FOURTYONE] 		= KFD_RadioPackInfo[THIRTYSEVEN];
    RPW_KFD_Buf[FOURTYTWO] 		= KFD_RadioPackInfo[THIRTYEIGHT];
    RPW_KFD_Buf[FOURTYTHREE] 	= KFD_RadioPackInfo[THIRTYNINE];
    RPW_KFD_Buf[FOURTYFOUR] 	= KFD_RadioPackInfo[FOURTY];
    RPW_KFD_Buf[FOURTYFIVE] 	= KFD_RadioPackInfo[FOURTYONE];
    RPW_KFD_Buf[FOURTYSIX] 		= KFD_RadioPackInfo[FOURTYTWO];
    RPW_KFD_Buf[FOURTYSEVEN] 	= KFD_RadioPackInfo[FOURTYTHREE];
    RPW_KFD_Buf[FOURTYEIGHT] 	= KFD_RadioPackInfo[FOURTYFOUR];
    RPW_KFD_Buf[FOURTYNINE] 	= KFD_RadioPackInfo[FOURTYFIVE];
    RPW_KFD_Buf[FIFTY] 			= KFD_RadioPackInfo[FOURTYSIX];
    RPW_KFD_Buf[FIFTYONE] 		= KFD_RadioPackInfo[FOURTYSEVEN];
    RPW_KFD_Buf[FIFTYTWO] 		= KFD_RadioPackInfo[FOURTYEIGHT];
    RPW_KFD_Buf[FIFTYTHREE] 	= KFD_RadioPackInfo[FOURTYNINE];
    RPW_KFD_Buf[FIFTYFOUR] 		= KFD_RadioPackInfo[FIFTY];
    RPW_KFD_Buf[FIFTYFIVE] 		= KFD_RadioPackInfo[FIFTYONE];
    RPW_KFD_Buf[FIFTYSIX] 		= KFD_RadioPackInfo[FIFTYTWO];               // Cell 4 Data (16 bytes)
    RPW_KFD_Buf[FIFTYSEVEN] 	= KFD_RadioPackInfo[FIFTYTHREE];
    RPW_KFD_Buf[FIFTYEIGHT] 	= KFD_RadioPackInfo[FIFTYFOUR];
    RPW_KFD_Buf[FIFTYNINE] 		= KFD_RadioPackInfo[FIFTYFIVE];
    RPW_KFD_Buf[SIXTY] 			= KFD_RadioPackInfo[FIFTYSIX];
    RPW_KFD_Buf[SIXTYONE] 		= KFD_RadioPackInfo[FIFTYSEVEN];
    RPW_KFD_Buf[SIXTYTWO] 		= KFD_RadioPackInfo[FIFTYEIGHT];
    RPW_KFD_Buf[SIXTYTHREE] 	= KFD_RadioPackInfo[FIFTYNINE];
    RPW_KFD_Buf[SIXTYFOUR] 		= KFD_RadioPackInfo[SIXTY];
    RPW_KFD_Buf[SIXTYFIVE] 		= KFD_RadioPackInfo[SIXTYONE];
    RPW_KFD_Buf[SIXTYSIX] 		= KFD_RadioPackInfo[SIXTYTWO];
    RPW_KFD_Buf[SIXTYSEVEN] 	= KFD_RadioPackInfo[SIXTYTHREE];
    RPW_KFD_Buf[SIXTYEIGHT] 	= KFD_RadioPackInfo[SIXTYFOUR];
    RPW_KFD_Buf[SIXTYNINE] 		= KFD_RadioPackInfo[SIXTYFIVE];
    RPW_KFD_Buf[SEVENTY] 		= KFD_RadioPackInfo[SIXTYSIX];
    RPW_KFD_Buf[SEVENTYONE] 	= KFD_RadioPackInfo[SIXTYSEVEN];								
    checksum(RPW_KFD_Buf, RPW_KFD_Buf[THREE]);
    memcpy(RPW_KFD_RadioPack, RPW_KFD_Buf, DEC_74);	
    return(RPW_KFD_RadioPack);
}


/** FHdr-beg *****************************************************
**
** Function name: f_hq_load_ss
**
** Anchor:   		   
**
** Purpose:        This fiunction will perform have quick  load ss function 
**
**
** Inputs:         have quick SS radio packet   information , have quick SS radio  packet    
**
**
** Outputs:       have quick SS  radio  packet 
**
** FPrj-beg *****************************************************
**
** FPrj-end *****************************************************
** FHdr-end *****************************************************/



uint8_t *f_hq_load_ss(uint8_t *HQLSS_RadioPackInfo, uint8_t *HQLoadSS_RadioPack, uint8_t GuardControl)
{
	
    uint8_t SS_HQLoad_Buf[HUNDREDANDSEVENTEEN] = "";
   	
    SS_HQLoad_Buf[ZERO]   			= HEX_0xAA;					//Synchronization
    SS_HQLoad_Buf[ONE]   			= HEX_0X25;					//Packet ID
    SS_HQLoad_Buf[TWO]   			= HEX_ZERO;						//Packet Length MSB
    SS_HQLoad_Buf[THREE]   			= DEC_117;						//Packet Length LSB
    SS_HQLoad_Buf[FOUR]   			= HEX_ZERO;						//Mode 0=LOS
    SS_HQLoad_Buf[FIVE]   			= HEX_ONE;						//ID 0=Current Preset
    SS_HQLoad_Buf[SIX]   			= HEX_SEVEN;						//Variant 3=Normal,7-HQ 
    SS_HQLoad_Buf[SEVEN]   			= HEX_ZERO;						//Modulation 0=AM and 1=FM
    SS_HQLoad_Buf[EIGHT]   			= HEX_ONE;						//Reserved1
    SS_HQLoad_Buf[NINE]   			= HEX_ONE;						//Reserved2
    SS_HQLoad_Buf[TEN]  			= HEX_ZERO;						//Source 0=Voice
    SS_HQLoad_Buf[ELEVEN]  			= DEC_14;						//Preset I/O Data Rate 11=9600bps 14=16000bps
    SS_HQLoad_Buf[TWELVE]  			= HEX_0xE8;					//Transmit Freq LSB
    SS_HQLoad_Buf[THIRTEEN]  		= HEX_0x6E;					//Transmit Freq LSB
    SS_HQLoad_Buf[FOURTEEN]  		= HEX_THREE;						//Transmit Freq MSB
    SS_HQLoad_Buf[FIFTEEN]  		= HEX_ZERO;						//Transmit Freq MSB
    SS_HQLoad_Buf[SIXTEEN]  		= HEX_0xE8;					//Receive Freq LSB
    SS_HQLoad_Buf[SEVENTEEN]  		= HEX_0x6E;					//Receive Freq LSB
    SS_HQLoad_Buf[EIGHTEEN]  		= HEX_THREE;						//Receive Freq MSB
    SS_HQLoad_Buf[NINETEEN]  		= HEX_ZERO;						//Receive Freq MSB  
    SS_HQLoad_Buf[TWENTY]  			= HEX_ZERO;			   			//Transmit Power Level
    SS_HQLoad_Buf[TWENTYONE]  		= HEX_FOUR;						//Transmit Power Decrement
    SS_HQLoad_Buf[TWENTYTWO]  		= HEX_ONE;						//Channel/Hopset LSB
    SS_HQLoad_Buf[TWENTYTHREE]  	= HEX_ONE;						// Channel/Hopset MSB
    SS_HQLoad_Buf[TWENTYFOUR]  		= GuardControl;		                //Guard Receiver Control
    SS_HQLoad_Buf[TWENTYFIVE]  		= HEX_ONE;						//Reserved3			
    SS_HQLoad_Buf[TWENTYSIX]  		= HEX_ONE;						//Reserved3
    SS_HQLoad_Buf[TWENTYSEVEN]  	= HEX_ONE;						//Reserved3
    SS_HQLoad_Buf[TWENTYEIGHT]  	= HEX_ONE;						//Reserved3
    SS_HQLoad_Buf[TWENTYNINE]  		= HEX_ONE;						//Reserved3
    SS_HQLoad_Buf[THIRTY]			= HEX_ONE;						//Reserved3
    SS_HQLoad_Buf[THIRTYONE]  		= HEX_ONE;						//Reserved3
    SS_HQLoad_Buf[THIRTYTWO]  		= HEX_ONE;						//Reserved3
    SS_HQLoad_Buf[THIRTYTHREE]  	= HEX_ONE;						//Reserved3
    SS_HQLoad_Buf[THIRTYFOUR]  		= HEX_ONE;						//Reserved3
    SS_HQLoad_Buf[THIRTYFIVE]  		= HEX_ONE;						//Reserved3
    SS_HQLoad_Buf[THIRTYSIX]  		= HEX_ONE;						//Reserved3
    SS_HQLoad_Buf[THIRTYSEVEN]  	= HQLSS_RadioPackInfo[ONE];					//HQ2 Net no. LSB
    SS_HQLoad_Buf[THIRTYEIGHT]  	= HQLSS_RadioPackInfo[TWO];					//HQ2 Net no. MSB
    SS_HQLoad_Buf[THIRTYNINE]  		= HQLSS_RadioPackInfo[THREE];					//HQ2 Net no. LSB
    SS_HQLoad_Buf[FOURTY]  			= HQLSS_RadioPackInfo[FOUR];					//HQ2 Net no. MSB
    SS_HQLoad_Buf[FOURTYONE]  		= HEX_ZERO;						//Reserved4
    SS_HQLoad_Buf[FOURTYTWO]  		= HEX_ZERO;						//Reserved4
    SS_HQLoad_Buf[FOURTYTHREE]  	= HEX_ZERO;						//Reserved4
    SS_HQLoad_Buf[FOURTYFOUR]  		= HEX_ZERO;						//Reserved4
    SS_HQLoad_Buf[FOURTYFIVE]  		= HEX_ZERO;						//Reserved4
    SS_HQLoad_Buf[FOURTYSIX]  		= HEX_ZERO;						//Reserved4
    SS_HQLoad_Buf[FOURTYSEVEN]  	= HEX_ZERO;						//Reserved4
    SS_HQLoad_Buf[FOURTYEIGHT]  	= HEX_ZERO;						//Reserved4
    SS_HQLoad_Buf[FOURTYNINE]  		= HEX_ZERO;						//Reserved4
    SS_HQLoad_Buf[FIFTY]  			= HEX_ZERO;						//Reserved4
    SS_HQLoad_Buf[FIFTYONE]  		= HEX_ZERO;						//Reserved4
    SS_HQLoad_Buf[FIFTYTWO]  		= HEX_ZERO;						//Reserved4
    SS_HQLoad_Buf[FIFTYTHREE]  		= HEX_ZERO;						//Reserved4
    SS_HQLoad_Buf[FIFTYFOUR]  		= HEX_ZERO;						//Reserved4
    SS_HQLoad_Buf[FIFTYFIVE]  		= HEX_ZERO;						//Maritime table:0=USA 1=International
    SS_HQLoad_Buf[FIFTYSIX]  		= HEX_ZERO;						//Maritime Station:0=Ship
    SS_HQLoad_Buf[FIFTYSEVEN]  		= HEX_0x30;					//RPW Cue Freq. LSB
    SS_HQLoad_Buf[FIFTYEIGHT]  		= HEX_0x75;					//RPW Cue Freq. LSB
    SS_HQLoad_Buf[FIFTYNINE]  		= HEX_ZERO;					//RPW Cue Freq. MSB
    SS_HQLoad_Buf[SIXTY]  			= HEX_ZERO;					//RPW Cue Freq. MSB
    SS_HQLoad_Buf[SIXTYONE]  		= HEX_ONE;						//RPW Band:1=Not Loaded
    SS_HQLoad_Buf[SIXTYTWO]  		= HEX_ZERO;						//RPW Net No. MSB
    SS_HQLoad_Buf[SIXTYTHREE]  		= HEX_ZERO;						//RPW Net No. LSB
    SS_HQLoad_Buf[SIXTYFOUR]  		= HEX_ZERO;						//Private Mode:0=None
    SS_HQLoad_Buf[SIXTYFIVE]  		= HEX_ZERO;						//Encode Tone Freq. LSB
    SS_HQLoad_Buf[SIXTYSIX]  		= HEX_ZERO;						//Encode Tone Freq. MSB
    SS_HQLoad_Buf[SIXTYSEVEN]  		= HEX_ZERO;						//Decode Tone Freq. LSB
    SS_HQLoad_Buf[SIXTYEIGHT]  		= HEX_ZERO;						//Decode Tone Freq. MSB
    SS_HQLoad_Buf[SIXTYNINE]  		= HEX_ZERO;						//Reserved5
    SS_HQLoad_Buf[SEVENTY]  		= HEX_ZERO;						//Reserved5
    SS_HQLoad_Buf[SEVENTYONE]  		= HEX_ZERO;						//Reserved5
    SS_HQLoad_Buf[SEVENTYTWO]  		= HEX_ZERO;						//Reserved5
    SS_HQLoad_Buf[SEVENTYTHREE]  	= HEX_ZERO;						//Reserved5
    SS_HQLoad_Buf[SEVENTYFOUR]  	= HEX_ZERO;						//Reserved5
    SS_HQLoad_Buf[SEVENTYFIVE]  	= HEX_ZERO;						//Reserved5
    SS_HQLoad_Buf[SEVENTYSIX]  		= HEX_ZERO;						//Reserved5
    SS_HQLoad_Buf[SEVENTYSEVEN]  	= HEX_ZERO;						//Reserved5
    SS_HQLoad_Buf[SEVENTYEIGHT]  	= HEX_ZERO;						//Reserved5
    SS_HQLoad_Buf[SEVENTYNINE]  	= HEX_ZERO;						//Reserved5
    SS_HQLoad_Buf[EIGHTY]  			= HEX_ZERO;						//Reserved5
    SS_HQLoad_Buf[EIGHTYONE]  		= HEX_ZERO;						//Reserved5
    SS_HQLoad_Buf[EIGHTYTWO]  		= HEX_ZERO;						//Reserved5
    SS_HQLoad_Buf[EIGHTYTHREE]  	= HEX_ZERO;						//Reserved5
    SS_HQLoad_Buf[EIGHTYFOUR]  		= HEX_ZERO;						//Reserved5
    SS_HQLoad_Buf[EIGHTYFIVE]  		= HEX_ZERO;						//Reserved5
    SS_HQLoad_Buf[EIGHTYSIX]  		= HEX_ZERO;						//Reserved5
    SS_HQLoad_Buf[EIGHTYSEVEN]  	= HEX_ZERO;						//Reserved5
    SS_HQLoad_Buf[EIGHTYEIGHT]  	= HEX_ZERO;						//Reserved5
    SS_HQLoad_Buf[EIGHTYNINE]  		= HEX_ZERO;						//Reserved5
    SS_HQLoad_Buf[NINTY]  			= HEX_ZERO;						//Reserved5
    SS_HQLoad_Buf[NINTYONE]  		= HEX_ZERO;						//Reserved5
    SS_HQLoad_Buf[NINTYTWO]  		= HEX_ZERO;						//Reserved5
    SS_HQLoad_Buf[NINTYTHREE]  		= HEX_0x30;					//RPW ManualFreq. LSB
    SS_HQLoad_Buf[NINTYFOUR]  		= HEX_0x75;					//RPW ManualFreq. LSB
    SS_HQLoad_Buf[NINTYFIVE]  		= HEX_ZERO;					//RPW ManualFreq. MSB
    SS_HQLoad_Buf[NINTYSIX]  		= HEX_ZERO;					//RPW ManualFreq. MSB
    SS_HQLoad_Buf[NINTYSEVEN]  		= HEX_ZERO;						//RPW Net No. 0: Disabled
    SS_HQLoad_Buf[NINTYEIGHT]  		= HEX_ZERO;						//RPW Enable Cue:0=Disabled
    SS_HQLoad_Buf[NINTYNINE]  		= HEX_ZERO;						//Reserved6
    SS_HQLoad_Buf[HUNDRED] 			= HEX_ZERO;						//RPW Mode:0=Cold Start, 1=manual
    SS_HQLoad_Buf[HUNDREDANDONE] 	= HEX_ZERO;						//I/F Filter Selection:0=Normal
    SS_HQLoad_Buf[HUNDREDANDTWO] 	= HEX_ONE;						//ADM Mode:0=Enabled;1-Disabled(normal mode)
    SS_HQLoad_Buf[HUNDREDANDTHREE]  = HEX_ONE;						//ADM ID
    SS_HQLoad_Buf[HUNDREDANDFOUR] 	= HEX_ONE;						//Reserved7
    SS_HQLoad_Buf[HUNDREDANDFIVE] 	= HEX_ZERO;						//ATC Mode Selected:0=Disabled
    SS_HQLoad_Buf[HUNDREDANDSIX] 	= HEX_ONE;						//ADM Quiet Mode:0=Enabled
    SS_HQLoad_Buf[HUNDREDANDSEVEN] 	= HEX_ONE;						//CTCSS Tone Mode:0=Non Std.
    SS_HQLoad_Buf[HUNDREDANDEIGHT] 	= HEX_0x58;					//Non. Std. Encode Tone Freq LSB
    SS_HQLoad_Buf[HUNDREDANDNINE] 	= HEX_TWO;					//Non. Std. Encode Tone Freq MSB
    SS_HQLoad_Buf[HUNDREDANDTEN] 	= HEX_0x58;					//Non. Std. Decode Tone Freq LSB
    SS_HQLoad_Buf[HUNDREDANDELEVEN] = HEX_TWO;					//Non. Std. Decode Tone Freq MSB
    SS_HQLoad_Buf[HUNDREDANDTWELVE] = HEX_ONE;						//Reserved8
    SS_HQLoad_Buf[HUNDREDANDTHIRTEEN] = HEX_ONE;						//Reserved8
    SS_HQLoad_Buf[HUNDREDANDFOURTEEN] = HEX_ONE;						//Reserved8											
    checksum(SS_HQLoad_Buf, SS_HQLoad_Buf[THREE]);	
    memcpy(HQLoadSS_RadioPack, SS_HQLoad_Buf, DEC_117);	
    return(HQLoadSS_RadioPack);
}


/** FHdr-beg *****************************************************
**
** Function name: f_hq_load_ss
**
** Anchor:   		   
**
** Purpose:        This fiunction will perform have quick  load  mwod
**
**
** Inputs:         have quick mwod radio packet   information , have quick mwod radio  packet    
**
**
** Outputs:       have quick mwod  radio  packet 
**
** FPrj-beg *****************************************************
**
** FPrj-end *****************************************************
** FHdr-end *****************************************************/


uint8_t *f_hq_load_mwod(uint8_t *HQ_MWOD_RadioPackInfo, uint8_t *HQLoadMWOD_RadioPack)
{	
    uint8_t MWOD_HQLoad_Buf[THIRTYTWO] = "";
  	
    /*	Havequick WOD LOAD, send packet 63hex - for MWOD */
    MWOD_HQLoad_Buf[ZERO]  			= HEX_0xAA;    			  // Synchronisation
    MWOD_HQLoad_Buf[ONE]  			= HEX_0x63;       		  // Packet Id 
    MWOD_HQLoad_Buf[TWO]  			= HEX_ZERO;       			  // Packet length LSB  
    MWOD_HQLoad_Buf[THREE]  		= DEC_32;    				// Packet length LSB
    MWOD_HQLoad_Buf[FOUR]  			= HEX_ONE;   			  //  0=WOD;1=MWOD
    MWOD_HQLoad_Buf[FIVE]  			= HQ_MWOD_RadioPackInfo[FIVE]; 				// Channel 20 LSB;  280MHz
    MWOD_HQLoad_Buf[SIX]  			= HQ_MWOD_RadioPackInfo[SIX];
    MWOD_HQLoad_Buf[SEVEN]  		= HQ_MWOD_RadioPackInfo[SEVEN];    			
    MWOD_HQLoad_Buf[EIGHT]  		= HQ_MWOD_RadioPackInfo[EIGHT];                  			
    MWOD_HQLoad_Buf[NINE]  			= HQ_MWOD_RadioPackInfo[NINE];               // Channel 19 LSB;  278MHz
    MWOD_HQLoad_Buf[TEN] 			= HQ_MWOD_RadioPackInfo[TEN];
    MWOD_HQLoad_Buf[ELEVEN] 		= HQ_MWOD_RadioPackInfo[ELEVEN];  				
    MWOD_HQLoad_Buf[TWELVE] 		= HQ_MWOD_RadioPackInfo[TWELVE];  				
    MWOD_HQLoad_Buf[THIRTEEN] 		= HQ_MWOD_RadioPackInfo[THIRTEEN];                // Channel 18 LSB;  276MHz
    MWOD_HQLoad_Buf[FOURTEEN] 		= HQ_MWOD_RadioPackInfo[FOURTEEN];
    MWOD_HQLoad_Buf[FIFTEEN] 		= HQ_MWOD_RadioPackInfo[FIFTEEN];  				
    MWOD_HQLoad_Buf[SIXTEEN] 		= HQ_MWOD_RadioPackInfo[SIXTEEN];   			
    MWOD_HQLoad_Buf[SEVENTEEN] 		= HQ_MWOD_RadioPackInfo[SEVENTEEN];               // Channel 17 LSB;  274MHz
    MWOD_HQLoad_Buf[EIGHTEEN] 		= HQ_MWOD_RadioPackInfo[EIGHTEEN];
    MWOD_HQLoad_Buf[NINETEEN] 		= HQ_MWOD_RadioPackInfo[NINETEEN];  				
    MWOD_HQLoad_Buf[TWENTY] 		= HQ_MWOD_RadioPackInfo[TWENTY];   			
    MWOD_HQLoad_Buf[TWENTYONE] 		= HQ_MWOD_RadioPackInfo[TWENTYONE];               // Channel 16 LSB;  272MHz
    MWOD_HQLoad_Buf[TWENTYTWO] 		= HQ_MWOD_RadioPackInfo[TWENTYTWO];
    MWOD_HQLoad_Buf[TWENTYTHREE] 	= HQ_MWOD_RadioPackInfo[TWENTYTHREE];    			
    MWOD_HQLoad_Buf[TWENTYFOUR] 	= HQ_MWOD_RadioPackInfo[TWENTYFOUR];    			
    MWOD_HQLoad_Buf[TWENTYFIVE] 	= HQ_MWOD_RadioPackInfo[TWENTYFIVE];               // Channel 15 LSB;  270MHz
    MWOD_HQLoad_Buf[TWENTYSIX] 		= HQ_MWOD_RadioPackInfo[TWENTYSIX];
    MWOD_HQLoad_Buf[TWENTYSEVEN] 	= HQ_MWOD_RadioPackInfo[TWENTYSEVEN];  				
    MWOD_HQLoad_Buf[TWENTYEIGHT] 	= HQ_MWOD_RadioPackInfo[TWENTYEIGHT];
    MWOD_HQLoad_Buf[TWENTYNINE] 	= HQ_MWOD_RadioPackInfo[TWENTYNINE];                 // MWOD Day
    checksum(MWOD_HQLoad_Buf, MWOD_HQLoad_Buf[THREE]);
    memcpy(HQLoadMWOD_RadioPack, MWOD_HQLoad_Buf, DEC_32);  
    return(HQLoadMWOD_RadioPack);	
}


/** FHdr-beg *****************************************************
**
** Function name: f_rpw_ss_fh
**
** Anchor:   		   
**
** Purpose:        This fiunction will perform RPW SS FH 
**
**
** Inputs:         RPW FH radio packet   information ,RPW FH radio  packet    
**
**
** Outputs:       RPW FH  radio  packet 
**
** FPrj-beg *****************************************************
**
** FPrj-end *****************************************************
** FHdr-end *****************************************************/

uint8_t *f_rpw_ss_fh(uint8_t *RPW_FH_RadioPackInfo, uint8_t *RPW_FH_RadioPack)
{	
    uint8_t RPW_FH_SS_Buf[HUNDREDANDSEVENTEEN] = "";    
	uint8_t RPW_FH_NetMaster 				   = HEX_ZERO;	
    if(RPW_FH_RadioPackInfo[ZERO] == HEX_ONE)
    {
		RPW_FH_NetMaster = HEX_ONE;
    }
    else
    {
		RPW_FH_NetMaster = HEX_ZERO;
    }
	
    //	Store Setup for RPW- FH mode		
    RPW_FH_SS_Buf[ZERO]   			= HEX_0xAA;					//Synchronization
    RPW_FH_SS_Buf[ONE]   			= HEX_0X25;					//Packet ID
    RPW_FH_SS_Buf[TWO]   			= HEX_ZERO;						//Packet Length MSB
    RPW_FH_SS_Buf[THREE]   			= DEC_117;					//Packet Length LSB
    RPW_FH_SS_Buf[FOUR]   			= HEX_ZERO;						//Mode 0=LOS
    RPW_FH_SS_Buf[FIVE]   			= HEX_ONE;						//ID 0=Current Preset
    RPW_FH_SS_Buf[SIX]   			= DEC_10;						//Variant 3=Normal,7-HQ , 10 -RPW
    RPW_FH_SS_Buf[SEVEN]   			= HEX_ONE;						//Modulation 0=AM and 1=FM
    RPW_FH_SS_Buf[EIGHT]   			= HEX_ZERO;						//Reserved1
    RPW_FH_SS_Buf[NINE]   			= HEX_ONE;						//Reserved2
    RPW_FH_SS_Buf[TEN]  			= HEX_ZERO;						//Source 0=Voice
    RPW_FH_SS_Buf[ELEVEN]  			= DEC_14;						//Preset I/O Data Rate 11=9600bps 14=16000bps
    RPW_FH_SS_Buf[TWELVE]  			= HEX_0xF8;					//Transmit Freq LSB
    RPW_FH_SS_Buf[THIRTEEN]  		= HEX_0x95;					//Transmit Freq LSB
    RPW_FH_SS_Buf[FOURTEEN]  		= HEX_THREE;						//Transmit Freq MSB
    RPW_FH_SS_Buf[FIFTEEN]  		= HEX_ZERO;						//Transmit Freq MSB
    RPW_FH_SS_Buf[SIXTEEN]  		= HEX_0xF8;					//Receive Freq LSB
    RPW_FH_SS_Buf[SEVENTEEN]  		= HEX_0x95;					//Receive Freq LSB
    RPW_FH_SS_Buf[EIGHTEEN]  		= HEX_THREE;						//Receive Freq MSB
    RPW_FH_SS_Buf[NINETEEN]  		= HEX_ZERO;						//Receive Freq MSB  
    RPW_FH_SS_Buf[TWENTY]  			= HEX_ZERO;			   			//Transmit Power Level
    RPW_FH_SS_Buf[TWENTYONE]  		= HEX_ONE;						//Transmit Power Decrement
    RPW_FH_SS_Buf[TWENTYTWO]  		= HEX_ONE;						//Channel/Hopset LSB
    RPW_FH_SS_Buf[TWENTYTHREE]  	= HEX_ZERO;						// Channel/Hopset MSB
    RPW_FH_SS_Buf[TWENTYFOUR]  		= HEX_ZERO;						//Guard Receiver Control
    RPW_FH_SS_Buf[TWENTYFIVE]  		= HEX_ONE;						//Reserved3
    RPW_FH_SS_Buf[TWENTYSIX]  		= HEX_ONE;						//Reserved3
    RPW_FH_SS_Buf[TWENTYSEVEN]  	= HEX_ONE;						//Reserved3
    RPW_FH_SS_Buf[TWENTYEIGHT]  	= HEX_ONE;						//Reserved3
    RPW_FH_SS_Buf[TWENTYNINE]  		= HEX_ONE;						//Reserved3
    RPW_FH_SS_Buf[THIRTY]  			= HEX_ONE;						//Reserved3
    RPW_FH_SS_Buf[THIRTYONE]  		= HEX_ONE;						//Reserved3
    RPW_FH_SS_Buf[THIRTYTWO]  		= HEX_ONE;						//Reserved3
    RPW_FH_SS_Buf[THIRTYTHREE]  	= HEX_ONE;						//Reserved3
    RPW_FH_SS_Buf[THIRTYFOUR]  		= HEX_ONE;						//Reserved3
    RPW_FH_SS_Buf[THIRTYFIVE]  		= HEX_ONE;						//Reserved3
    RPW_FH_SS_Buf[THIRTYSIX]  		= HEX_ONE;						//Reserved3
    RPW_FH_SS_Buf[THIRTYSEVEN]  	= HEX_ZERO;					    //HQ2 Net no. LSB
    RPW_FH_SS_Buf[THIRTYEIGHT]  	= HEX_ZERO;					    //HQ2 Net no. MSB
    RPW_FH_SS_Buf[THIRTYNINE]  		= HEX_ZERO;						//HQ2 Net no. LSB
    RPW_FH_SS_Buf[FOURTY]  			= HEX_ZERO;						//HQ2 Net no. MSB
    RPW_FH_SS_Buf[FOURTYONE]  		= HEX_ZERO;						//Reserved4
    RPW_FH_SS_Buf[FOURTYTWO]  		= HEX_ZERO;						//Reserved4
    RPW_FH_SS_Buf[FOURTYTHREE]  	= HEX_ZERO;						//Reserved4
    RPW_FH_SS_Buf[FOURTYFOUR]  		= HEX_ZERO;						//Reserved4
    RPW_FH_SS_Buf[FOURTYFIVE]  		= HEX_ZERO;						//Reserved4
    RPW_FH_SS_Buf[FOURTYSIX]  		= HEX_ZERO;						//Reserved4
    RPW_FH_SS_Buf[FOURTYSEVEN]  	= HEX_ZERO;						//Reserved4
    RPW_FH_SS_Buf[FOURTYEIGHT]  	= HEX_ZERO;						//Reserved4
    RPW_FH_SS_Buf[FOURTYNINE]  		= HEX_ZERO;						//Reserved4
    RPW_FH_SS_Buf[FIFTY]  			= HEX_ZERO;						//Reserved4
    RPW_FH_SS_Buf[FIFTYONE]  		= HEX_ZERO;						//Reserved4
    RPW_FH_SS_Buf[FIFTYTWO]  		= HEX_ZERO;						//Reserved4
    RPW_FH_SS_Buf[FIFTYTHREE]  		= HEX_ZERO;						//Reserved4
    RPW_FH_SS_Buf[FIFTYFOUR]  		= HEX_ZERO;						//Reserved4
    RPW_FH_SS_Buf[FIFTYFIVE]  		= HEX_ZERO;						//Maritime table:0=USA 1=International
    RPW_FH_SS_Buf[FIFTYSIX]  		= HEX_ZERO;						//Maritime Station:0=Ship
    RPW_FH_SS_Buf[FIFTYSEVEN]  		= HEX_0x80;					//RPW Cue Freq. LSB
    RPW_FH_SS_Buf[FIFTYEIGHT]  		= HEX_0xA9;					//RPW Cue Freq. LSB
    RPW_FH_SS_Buf[FIFTYNINE]  		= HEX_THREE;					//RPW Cue Freq. MSB
    RPW_FH_SS_Buf[SIXTY]  			= HEX_ZERO;					//RPW Cue Freq. MSB
    RPW_FH_SS_Buf[SIXTYONE]  		= HEX_THREE;						//RPW Band:1=Not Loaded, 3-UHF
    RPW_FH_SS_Buf[SIXTYTWO]  		= RPW_FH_RadioPackInfo[ONE];					//RPW Net No. LSB					// 0x4D
    RPW_FH_SS_Buf[SIXTYTHREE]  		= RPW_FH_RadioPackInfo[TWO];					//RPW Net No. MSB					//0x01
    RPW_FH_SS_Buf[SIXTYFOUR] 		= HEX_ZERO;						//Private Mode:0=None
    RPW_FH_SS_Buf[SIXTYFIVE]  		= HEX_ZERO;						//Encode Tone Freq. LSB
    RPW_FH_SS_Buf[SIXTYSIX]  		= HEX_ZERO;						//Encode Tone Freq. MSB
    RPW_FH_SS_Buf[SIXTYSEVEN]  		= HEX_ZERO;						//Decode Tone Freq. LSB
    RPW_FH_SS_Buf[SIXTYEIGHT]  		= HEX_ZERO;						//Decode Tone Freq. MSB
    RPW_FH_SS_Buf[SIXTYNINE]  		= HEX_ZERO;						//Reserved5
    RPW_FH_SS_Buf[SEVENTY]  		= HEX_ZERO;						//Reserved5
    RPW_FH_SS_Buf[SEVENTYONE]  		= HEX_ZERO;						//Reserved5
    RPW_FH_SS_Buf[SEVENTYTWO]  		= HEX_ZERO;						//Reserved5
    RPW_FH_SS_Buf[SEVENTYTHREE]  	= HEX_ZERO;						//Reserved5
    RPW_FH_SS_Buf[SEVENTYFOUR]  	= HEX_ZERO;						//Reserved5
    RPW_FH_SS_Buf[SEVENTYFIVE]  	= HEX_ZERO;						//Reserved5
    RPW_FH_SS_Buf[SEVENTYSIX]  		= HEX_ZERO;						//Reserved5
    RPW_FH_SS_Buf[SEVENTYSEVEN]  	= HEX_ZERO;						//Reserved5
    RPW_FH_SS_Buf[SEVENTYEIGHT]  	= HEX_ZERO;						//Reserved5
    RPW_FH_SS_Buf[SEVENTYNINE]  	= HEX_ZERO;						//Reserved5
    RPW_FH_SS_Buf[EIGHTY]  			= HEX_ZERO;						//Reserved5
    RPW_FH_SS_Buf[EIGHTYONE]  		= HEX_ZERO;						//Reserved5
    RPW_FH_SS_Buf[EIGHTYTWO]  		= HEX_ZERO;						//Reserved5
    RPW_FH_SS_Buf[EIGHTYTHREE]  	= HEX_ZERO;						//Reserved5
    RPW_FH_SS_Buf[EIGHTYFOUR]  		= HEX_ZERO;						//Reserved5
    RPW_FH_SS_Buf[EIGHTYFIVE]  		= HEX_ZERO;						//Reserved5
    RPW_FH_SS_Buf[EIGHTYSIX]  		= HEX_ZERO;						//Reserved5
    RPW_FH_SS_Buf[EIGHTYSEVEN]  	= HEX_ZERO;						//Reserved5
    RPW_FH_SS_Buf[EIGHTYEIGHT]  	= HEX_ZERO;						//Reserved5
    RPW_FH_SS_Buf[EIGHTYNINE]  		= HEX_ZERO;						//Reserved5
    RPW_FH_SS_Buf[NINTY]  			= HEX_ZERO;						//Reserved5
    RPW_FH_SS_Buf[NINTYONE]  		= HEX_ZERO;						//Reserved5
    RPW_FH_SS_Buf[NINTYTWO]  		= HEX_ZERO;						//Reserved5
    RPW_FH_SS_Buf[NINTYTHREE]  		= HEX_0xF8;					//RPW ManualFreq. LSB
    RPW_FH_SS_Buf[NINTYFOUR]  		= HEX_0x95;					//RPW ManualFreq. LSB
    RPW_FH_SS_Buf[NINTYFIVE]  		= HEX_THREE;					//RPW ManualFreq. MSB
    RPW_FH_SS_Buf[NINTYSIX]  		= HEX_ZERO;					//RPW ManualFreq. MSB
    RPW_FH_SS_Buf[NINTYSEVEN]  		= RPW_FH_NetMaster;				//RPW Net Master 0: Disabled 1:Enabled
    RPW_FH_SS_Buf[NINTYEIGHT]  		= HEX_ZERO;						//RPW Enable Cue:0=Disabled
    RPW_FH_SS_Buf[NINTYNINE]  		= HEX_ZERO;						//Reserved6
    RPW_FH_SS_Buf[HUNDRED] 			= HEX_THREE;						//RPW Mode:0=Cold Start,1=manual,2-Cue,3-FH
    RPW_FH_SS_Buf[HUNDREDANDONE] 	= HEX_ZERO;						//I/F Filter Selection:0=Normal
    RPW_FH_SS_Buf[HUNDREDANDTWO] 	= HEX_ONE;						//ADM Mode:0=Enabled;1-Disabled(normal mode)
    RPW_FH_SS_Buf[HUNDREDANDTHREE] 	= HEX_ONE;						//ADM ID
    RPW_FH_SS_Buf[HUNDREDANDFOUR] 	= HEX_ONE;						//Reserved7
    RPW_FH_SS_Buf[HUNDREDANDFIVE] 	= HEX_ZERO;						//ATC Mode Selected:0=Disabled
    RPW_FH_SS_Buf[HUNDREDANDSIX] 	= HEX_ONE;						//ADM Quiet Mode:0=Enabled
    RPW_FH_SS_Buf[HUNDREDANDSEVEN] 	= HEX_ONE;						//CTCSS Tone Mode:0=Non Std.
    RPW_FH_SS_Buf[HUNDREDANDEIGHT] 	= HEX_0x58;					//Non. Std. Encode Tone Freq LSB
    RPW_FH_SS_Buf[HUNDREDANDNINE] 	= HEX_TWO;					//Non. Std. Encode Tone Freq MSB
    RPW_FH_SS_Buf[HUNDREDANDTEN] 	= HEX_0x58;					//Non. Std. Decode Tone Freq LSB
    RPW_FH_SS_Buf[HUNDREDANDELEVEN] = HEX_TWO;					//Non. Std. Decode Tone Freq MSB
    RPW_FH_SS_Buf[HUNDREDANDTWELVE] = HEX_ONE;						//Reserved8
    RPW_FH_SS_Buf[HUNDREDANDTHIRTEEN] = HEX_ONE;						//Reserved8
    RPW_FH_SS_Buf[HUNDREDANDFOURTEEN] = HEX_ONE;						//Reserved8			
    checksum(RPW_FH_SS_Buf, RPW_FH_SS_Buf[THREE]);
    memcpy(RPW_FH_RadioPack, RPW_FH_SS_Buf, DEC_117);	
    return(RPW_FH_RadioPack);
}


/** FHdr-beg *****************************************************
**
** Function name: f_gpsrecv_enable
**
** Anchor:   		   
**
** Purpose:        This fiunction will perform enable gps_recv to radio
**
**
** Inputs:          Reveive GPS radio  packet    
*  recvgps_buf[0] = 0xAA;  // synchronization
*  recvgps_buf[1] = 0x61;  // packet id         // have quick command 0x61 
*  recvgps_buf[2] = 0;     // packet length msb    
*  recvgps_buf[3] = 8;     // packet length lsb 
*  recvgps_buf[4] = 4;     // type 4 recv gps
*  recvgps_buf[5] = 0;
*  recvgps_buf[6] = 0;     // CRC
*  recvgps_buf[7] = 0;     // CRC
** Outputs:       GPS received  radio packet   
**
** FPrj-beg *****************************************************
**
** FPrj-end *****************************************************
** FHdr-end *****************************************************/

uint8_t *f_gpsrecv_enable(uint8_t *recvgps_radiopack)
{
    uint8_t recvgps_buf[EIGHT] = {HEX_0xAA, HEX_0x61, HEX_ZERO, HEX_EIGHT, HEX_FOUR, HEX_ZERO, HEX_ZERO, HEX_ZERO};  
    checksum(recvgps_buf, recvgps_buf[THREE]);
    memcpy(recvgps_radiopack, recvgps_buf, HEX_EIGHT);    
    return(recvgps_radiopack);

}


/** FHdr-beg *****************************************************
**
** Function name: f_rpw_stored_transec
**
** Anchor:   		   
**
** Purpose:        This fiunction will perform rpw stored transec
**
**
** Inputs:         RPW store  packet   information , RPW store radio  packet    
**
**
** Outputs:       RPW radio packet  
**
** FPrj-beg *****************************************************
**
** FPrj-end *****************************************************
** FHdr-end *****************************************************/




uint8_t *f_rpw_stored_transec(uint8_t RpwStoredTransec_RadioPackInfo, uint8_t *RpwStoredTransec_RadioPack)
{
    uint8_t RpwStoredTransec_Buf[NINE] = "";		
    /* RPW - RpwStoredTransecReq - for loading status request */	
    RpwStoredTransec_Buf[ZERO] 	= HEX_0xAA;
    RpwStoredTransec_Buf[ONE] 	= HEX_0x6E;
    RpwStoredTransec_Buf[TWO] 	= HEX_ZERO;
    RpwStoredTransec_Buf[THREE] = HEX_NINE;
    RpwStoredTransec_Buf[FOUR] 	= HEX_ZERO;
    RpwStoredTransec_Buf[FIVE] 	= HEX_ONE;
    RpwStoredTransec_Buf[SIX] 	= HEX_THREE;		
    checksum(RpwStoredTransec_Buf, RpwStoredTransec_Buf[THREE]);
    memcpy(RpwStoredTransec_RadioPack, RpwStoredTransec_Buf, HEX_NINE);	
    return(RpwStoredTransec_RadioPack);
}


