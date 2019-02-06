/******************************************************************************
*
* Filename: 	mcs_ctrlr.c  
* 
*
******************************************************************************/

#include "mcs_ctrlr.h"
#include "global.h"

const char charZero ='0'; 
const char charOne  ='1';
const char charTwo  ='2';
const char charThree='3';
const char charFour ='4'; 
const char charFive ='5'; 
const char charSix  ='6'; 
const char charEight='8'; 
const char charNine ='9'; 


/** FHdr-beg *****************************************************
**
** Function name: open_port
**
** Anchor:        RCS_SOW_REQ009
**
** Purpose:       funtion to open a serial port
**
**
** Inputs:        radio id,number of retry    
**
**
** Outputs:       serial port open status 
**
** FPrj-beg *****************************************************
**
** FPrj-end *****************************************************
** FHdr-end *****************************************************/
int32_t open_serial_port(uint8_t radio_id, int32_t *open_serialfd, uint8_t serial_numof_retry)                 
{
    struct termios options;    
    uint8_t dev_name[TWENTY];
    uint8_t buf[TOTAL_RADIOS_PLUS_ONE] = "001234";
    uint8_t serial_loopbreak = HEX_ONE;     
    double  serial_timetakento_recvpack = HEX_ZERO, serial_timetakento_recvpack1 = HEX_ZERO, serial_timetakento_recvpack2 = HEX_ZERO;
    struct  timespec serial_remtime_struct;    
    strcpy(dev_name, port_name);
    strncat(dev_name, &buf[radio_id], HEX_ONE);    
    clock_gettime(CLOCK_REALTIME, &serial_remtime_struct);
    serial_timetakento_recvpack1 = serial_remtime_struct.tv_sec * pow(HEX_TEN, HEX_NINE) + serial_remtime_struct.tv_nsec;    	
   //while(TRUE)
		for(;;)
    {         
        if(open_serialfd[radio_id] == ERROR)
        {
            open_serialfd[radio_id] = open(dev_name, O_RDWR | O_NOCTTY | O_NDELAY );            
            if(open_serialfd[radio_id] == ERROR)				         
            {
               
                open_serialfd[radio_id] = ERROR;
                clock_gettime(CLOCK_REALTIME, &serial_remtime_struct);
                serial_timetakento_recvpack2 = serial_remtime_struct.tv_sec * pow(HEX_TEN, HEX_NINE) + serial_remtime_struct.tv_nsec;
                serial_timetakento_recvpack  = (serial_timetakento_recvpack2 - serial_timetakento_recvpack1) / DEC_1000000000;

                if(serial_loopbreak >= serial_numof_retry || serial_timetakento_recvpack >= DEC_250)                {
                    
                    break;
                }
                else
                {
                    ;
                }
                serial_loopbreak++;
                usleep(SEC * HEX_TEN);                        /* taskdelay(sysClkRateGet() * 10); */
            }
            else
            {
                fcntl(open_serialfd[radio_id], F_SETFL, HEX_ZERO);                 
                /* ask whether to put below lines in else condition or outside */
                tcgetattr(open_serialfd[radio_id], &options);	/* Get the current attributes of the Serial port */    

                /* Setting the Baud rate */
                cfsetispeed(&options, B9600);        /* Set Read  Speed as 9600 */
                cfsetospeed(&options, B9600);        /* Set Write Speed as 9600 */
                /* 8N1 mode */
                options.c_cflag   &= ~PARENB;        /* Disables the Parity Enable bit(PARENB),So No Parity   */
                options.c_cflag   &= ~CSTOPB;        /* CSTOPB = 2 Stop bits,here it is cleared so 1 Stop bit */
                options.c_cflag   &= ~CSIZE;         /* Clears the mask for setting the data size             */
                options.c_cflag   |=  CS8;           /* Set the data bits = 8                                 */		
                options.c_cflag   &= ~CRTSCTS;       /* No Hardware flow Control                              */
                options.c_cflag   |= CREAD | CLOCAL; /* Enable receiver,Ignore Modem Control lines            */ 			
                options.c_iflag   &= ~(IXON | IXOFF | IXANY);          /* Disable XON/XOFF flow control both i/p and o/p */
                options.c_iflag   &= ~(ICANON | ECHO | ECHOE | ISIG);  /* Non Cannonical mode                         */
                options.c_oflag   &= ~OPOST;         /* No Output Processing */
                /* Setting Time outs */
                options.c_cc[VMIN]  = HEX_ZERO;            /* Read at least 5 characters */
                options.c_cc[VTIME] = DEC_50;            /* Wait indefinetly   */      
                tcgetattr(open_serialfd[radio_id], &options);	/* Get the current attributes of the Serial port */
			    /*------------------------------- Read data from serial port -----------------------------*/
                tcflush(open_serialfd[radio_id], TCIFLUSH);      /* Discards old data in the rx buffer */  
                break;
            }
        } 
        else
        {
            ;   
        }   
    }  
    return(OK);
}


/** FHdr-beg *****************************************************
**
** Function name: serial read
**
** Anchor:      RCS_SOW_REQ009  
**
** Purpose:       function to read serial data from radio
**
**
** Inputs:        serial_fd,recv_data, expected_packid   
**
**
** Outputs:       serial read status  
**
** FPrj-beg *****************************************************
**
** FPrj-end *****************************************************
** FHdr-end *****************************************************/


uint32_t serial_read(int32_t serial_fd, uint8_t *recv_data, uint8_t *expected_packid)
{    
    int32_t  num_bytes_recvd        	= HEX_ZERO;
	int32_t  n = HEX_ZERO;
    uint32_t   recv_timeoutset_nsec   	= HEX_ZERO;
	uint32_t   recv_timeoutset_sec    	= HEX_ZERO;
    double  time_takento_recvpack1 		= HEX_ZERO;
    double	time_takento_recvpack2 		= HEX_ZERO;
    double	recv_timeout_time         	= HEX_ZERO;
    uint8_t recv_data_new[DEC_255]    	= "";
	uint8_t radiopack_ackload[TWO]   	= "";
	uint8_t radio_ack_packto_radio[EIGHT] = "";
	uint8_t radiopack_recvchar  		= HEX_ZERO;
    uint8_t num_of_times_recvpack  		= HEX_ZERO;
	uint8_t radiopack_packlen 			= HEX_ZERO;
    uint8_t	radiopack_recvloop  		= HEX_ZERO;
    uint8_t	radiopack_recvstart 		= HEX_ZERO;
    uint8_t	recvd_or_timedout 			= HEX_ZERO;

    struct timespec recv_remtime_struct, recv_timeout_struct;    
    
    if(serial_fd != ERROR)
    {       
        recv_timeoutset_sec =  expected_packid[THREE] + HEX_ONE;
        recv_timeout_time = (recv_timeoutset_sec * pow(HEX_TEN, HEX_NINE)) + recv_timeoutset_nsec; 
        num_of_times_recvpack = HEX_ONE; 

        while(num_of_times_recvpack < LOOP_TERMINATE)
        {
            clock_gettime(CLOCK_REALTIME, &recv_remtime_struct);
            time_takento_recvpack1 = (recv_remtime_struct.tv_sec * pow(HEX_TEN, HEX_NINE)) + recv_remtime_struct.tv_nsec;
            recv_timeout_struct.tv_sec = recv_timeoutset_sec;
            recv_timeout_struct.tv_nsec = recv_timeoutset_nsec;

            memset(recv_data_new, HEX_ZERO, sizeof(recv_data_new));
            recvd_or_timedout = HEX_ZERO; 
            radiopack_recvstart = HEX_ZERO;
            radiopack_packlen = DEC_150;

            for(radiopack_recvloop = HEX_ZERO; ;radiopack_recvloop++) 
            {                
                num_bytes_recvd = read(serial_fd, (int8_t *)&radiopack_recvchar, 1);                      
                if(num_bytes_recvd == ERROR)
                {
                    
                    recvd_or_timedout = LOOP_TERMINATE;
                    num_of_times_recvpack = LOOP_TERMINATE;
                    break; 
                } 
                else if(num_bytes_recvd == HEX_ZERO)
                {
                    
                    close(serial_fd);                                  
                    serial_fd = ERROR;
                    recvd_or_timedout = LOOP_TERMINATE;
                    num_of_times_recvpack = LOOP_TERMINATE;
                    break; 
                }  

                if(radiopack_recvchar == HEX_0xAA && radiopack_recvstart == HEX_ZERO)
                {
					radiopack_recvloop = HEX_ZERO;
					radiopack_recvstart = HEX_ONE; 
                }                
                if(radiopack_recvstart == HEX_ONE)
                {
					recv_data_new[radiopack_recvloop] = radiopack_recvchar;
                    if(radiopack_recvloop == HEX_THREE)
                    {
						radiopack_packlen = radiopack_recvchar;
                    }
                    else if(radiopack_recvloop == (radiopack_packlen - HEX_ONE))
                    {
                        break;
                    }                   
                }  
            } 

            if(!recvd_or_timedout)
            {
                if((expected_packid[ZERO] == HEX_ONE && recv_data_new[ZERO] == HEX_0xAA && expected_packid[ZERO] == recv_data_new[ONE] &&
                    expected_packid[ONE] == recv_data_new[FOUR] && expected_packid[TWO] == recv_data_new[THREE]) ||
                   (expected_packid[ZERO] != HEX_ONE && recv_data_new[ZERO] == HEX_0xAA && expected_packid[ZERO] == recv_data_new[ONE] && 
                    expected_packid[TWO] == recv_data_new[THREE]))
                {
					memcpy(recv_data, recv_data_new, expected_packid[TWO]);
                    num_of_times_recvpack = LOOP_TERMINATE;                             
                    if((recv_data[ONE] == HEX_ONE && recv_data[FOUR] == HEX_SIX) || (recv_data[ONE] == HEX_ONE && recv_data[FOUR] == HEX_ZERO)) 
               	    {
                       
                    }
                    else
                    {
                        
                    }  
                }
                else
                {
                   

                    if(recv_data_new[ONE] == HEX_0xAE || recv_data_new[ONE] == HEX_0XA1 || recv_data_new[ONE] == HEX_0XED || recv_data_new[ONE] == HEX_0XE0 ||
                       recv_data_new[ONE] == HEX_0XA6 || recv_data_new[ONE] == HEX_0X89 || recv_data_new[ONE] == HEX_0XEE || recv_data_new[ONE] == HEX_0XA0)
                    {
                        bzero(radiopack_ackload, sizeof(radiopack_ackload));
                        radiopack_ackload[ZERO] = recv_data_new[ONE];
                                                 
                        f_ackpack_toradio(radiopack_ackload[ZERO], radio_ack_packto_radio);
                        n = write(serial_fd, radio_ack_packto_radio, HEX_SEVEN);
						usleep(DEC_10); 
					
                    }
                    clock_gettime(CLOCK_REALTIME, &recv_remtime_struct);
                    time_takento_recvpack2 = recv_remtime_struct.tv_sec * pow(HEX_TEN, HEX_NINE) + recv_remtime_struct.tv_nsec;
                    recv_timeout_time = recv_timeout_time - (time_takento_recvpack2 - time_takento_recvpack1);
                    if(recv_timeout_time >= RECEIVTIMEOUT)
                    {
                        recv_timeoutset_sec = recv_timeout_time / DEC_1000000000;
                        recv_timeoutset_nsec = recv_timeout_time / HEX_TEN;
                        //recv_timeoutset_nsec = HEX_ZERO; 
                        num_of_times_recvpack++;
                    }
                    else
                    {
                        memset(recv_data, HEX_ZERO, expected_packid[HEX_TWO]);    	
                        num_of_times_recvpack = LOOP_TERMINATE;
                    }                    
                }                
            } 
            else
            {
                memset(recv_data, HEX_ZERO, expected_packid[TWO]);    	
            }            
        }                 
    } 
    else
    {
        
        memset(recv_data, HEX_ZERO, expected_packid[TWO]);    	
    }
    
    return(OK); 

}


/** FHdr-beg *****************************************************
**
** Function name: serialfrequency_roundof
**
** Anchor:        RCS_SOW_REQ003
**
** Purpose:       this function will round of the retrievd frequency 
**
**
** Inputs:        recvd_freqvalue, rounded_freqvalue   
**
**
** Outputs:       round off frequency value  
**
** FPrj-beg *****************************************************
**
** FPrj-end *****************************************************
** FHdr-end *****************************************************/

uint8_t frequency_roundof(float recvd_freqvalue, uint32_t *rounded_freqvalue)
{
    uint8_t freqtostring_with_dot[HEX_12] 		= "";
    uint8_t freqtostring_without_dot[HEX_12]	= "";
	uint8_t freq_status 						= HEX_ZERO;
    uint32_t freq_to_integer 					= HEX_ZERO;
	uint32_t freq_to_new_integer 				= HEX_ZERO;
    
    sprintf(freqtostring_with_dot, "%.3f", recvd_freqvalue);    
    freqtostring_without_dot[ZERO]  = freqtostring_with_dot[ZERO];
    freqtostring_without_dot[ONE]   = freqtostring_with_dot[ONE];
    freqtostring_without_dot[TWO]   = freqtostring_with_dot[TWO];
    freqtostring_without_dot[THREE] = freqtostring_with_dot[FOUR];
    freqtostring_without_dot[FOUR]  = freqtostring_with_dot[FIVE];
    freqtostring_without_dot[FIVE]  = freqtostring_with_dot[SIX];
    
    freq_to_integer = atoi(freqtostring_without_dot);
    
    if(freq_to_integer % HEX_25 == HEX_ZERO)
    {
        freq_status = HEX_ZERO;
    }
    else if(recvd_freqvalue >= FLOAT_118_00 && recvd_freqvalue <= FLOAT_135_975)
    {
        freq_status = HEX_ONE;
        bzero(freqtostring_with_dot, sizeof(freqtostring_with_dot));
        bzero(freqtostring_without_dot, sizeof(freqtostring_without_dot));
        
        sprintf(freqtostring_with_dot, "%.6f", recvd_freqvalue);        
        freqtostring_without_dot[ZERO]   = freqtostring_with_dot[ZERO];
        freqtostring_without_dot[ONE] 	 = freqtostring_with_dot[ONE];
        freqtostring_without_dot[TWO] 	 = freqtostring_with_dot[TWO];
        freqtostring_without_dot[THREE]  = freqtostring_with_dot[FOUR];
        freqtostring_without_dot[FOUR]   = freqtostring_with_dot[FIVE];
        freqtostring_without_dot[FIVE]   = freqtostring_with_dot[SIX];
        freqtostring_without_dot[SIX]    = freqtostring_with_dot[SEVEN];
        freqtostring_without_dot[SEVEN]  = freqtostring_with_dot[EIGHT];
        freqtostring_without_dot[EIGHT]  = freqtostring_with_dot[NINE];
        
        freq_to_integer = atoi(freqtostring_without_dot);        
        freq_to_integer = freq_to_integer / DEC_100;
        freq_to_new_integer = freq_to_integer % HEX_TEN;
        freq_to_integer = freq_to_integer / HEX_TEN;
        freq_to_new_integer = freq_to_new_integer + (HEX_TEN * (freq_to_integer % HEX_TEN));       
        freq_to_integer = freq_to_integer / HEX_TEN;
        freq_to_new_integer = freq_to_new_integer + (DEC_100 * (freq_to_integer % HEX_TEN));        
        if(freq_to_new_integer == DEC_033 && ((freqtostring_without_dot[HEX_SEVEN] == HEX_0x32 || freqtostring_without_dot[SEVEN] == HEX_0x33)  || freqtostring_without_dot[SEVEN] == HEX_0x34))
        {
            freqtostring_without_dot[FOUR] = charOne;
            freqtostring_without_dot[FIVE] = charZero;        
        }
        else if(freq_to_new_integer == FREQUENCYTONEWINT && ((freqtostring_without_dot[SEVEN] ==HEX_0x35 || freqtostring_without_dot[HEX_SEVEN] == HEX_0x36)  || 
                                                                                       freqtostring_without_dot[SEVEN] == HEX_0x37))
        {
            freqtostring_without_dot[FOUR] = charZero;
            freqtostring_without_dot[FIVE] = charFive;        
        }
        else if(freq_to_new_integer == DEC_416 && ((freqtostring_without_dot[SEVEN] == HEX_0x35 || freqtostring_without_dot[SEVEN] == HEX_0x36)  || 
                                                                                       freqtostring_without_dot[SEVEN] == HEX_0x37))
        {
            freqtostring_without_dot[FOUR] = charFour;
            freqtostring_without_dot[FIVE] = charZero;        
        }
        else if(freq_to_new_integer == DEC_583 && ((freqtostring_without_dot[SEVEN] == HEX_0x32 || freqtostring_without_dot[SEVEN] == HEX_0x33)  || 
                                                                                       freqtostring_without_dot[SEVEN] == HEX_0x34))
        {
            freqtostring_without_dot[FOUR] = charSix;
            freqtostring_without_dot[FIVE] = charZero;        
        }
        else if(freq_to_new_integer == DEC_666 && ((freqtostring_without_dot[SEVEN] == HEX_0x35 || freqtostring_without_dot[SEVEN] == HEX_0x36)  || 
                                                                                       freqtostring_without_dot[SEVEN] == HEX_0x37))
        {
            freqtostring_without_dot[FOUR] = charSix;
            freqtostring_without_dot[FIVE] = charFive;        
        }
        else if(freq_to_new_integer == DEC_833 && ((freqtostring_without_dot[SEVEN] == HEX_0x32 || freqtostring_without_dot[SEVEN] == HEX_0x33)  || 
                                                                                       freqtostring_without_dot[SEVEN] == HEX_0x34))
        {
            freqtostring_without_dot[FOUR] = charEight;
            freqtostring_without_dot[FIVE] = charFive;        
        }
        else if(freq_to_new_integer == DEC_916 && ((freqtostring_without_dot[SEVEN] == HEX_0x35 || freqtostring_without_dot[SEVEN] == HEX_0x36)  || 
                                                                                       freqtostring_without_dot[SEVEN] == HEX_0x37))
        {
            freqtostring_without_dot[FOUR] = charNine;
            freqtostring_without_dot[FIVE] = charZero;        
        }
        else
        {
            freq_status = HEX_0XEE;
        }
        
        if(freq_status == HEX_ONE)
        {
            freqtostring_without_dot[SIX] = charZero;
            freqtostring_without_dot[SEVEN] = charZero;
            freqtostring_without_dot[EIGHT] = charZero;
            freq_to_integer = (atoi(freqtostring_without_dot) / DEC_1000);
        }
        else
        {
            freq_to_integer = HEX_ZERO;
        }
    }
    else
    {
        freq_status = HEX_0XEE;
        freq_to_integer = HEX_ZERO;
    }
    
    *rounded_freqvalue = freq_to_integer;
    return(freq_status);
}




/** FHdr-beg *****************************************************
**
** Function name: dec_to_bin
**
** Anchor:        RCS_SOW_REQ003
**
** Purpose:       function to perform decimal to binary conversion
**
**
** Inputs:        decimal value , buffer    
**
**
** Outputs:       binary value to corresponding decimal   
**
** FPrj-beg *****************************************************
**
** FPrj-end *****************************************************
** FHdr-end *****************************************************/

uint32_t dec_to_bin(uint32_t decimal_val, uint8_t *dec2bin_buf)
{
    uint8_t dec2bin_var = HEX_ZERO;
    bzero(dec2bin_buf, HEX_EIGHT);
    if(decimal_val != HEX_ZERO)
    {
        while(decimal_val > HEX_ZERO)
        {
			dec2bin_buf[dec2bin_var] = decimal_val % HEX_TWO;
            dec2bin_var++;
            decimal_val = decimal_val / HEX_TWO;
        }
    }
    else
    {
        ;
    }
 
    return OK;
    
}


/** FHdr-beg *****************************************************
**
** Function name: dec_to_hex
**
** Anchor:        RCS_SOW_REQ003
**
** Purpose:       function to perform decimal to hex conversion
**
**
** Inputs:        decimal value , buffer    
**
**
** Outputs:       decimal value   corresponding to binary value
**
** FPrj-beg *****************************************************
**
** FPrj-end *****************************************************
** FHdr-end *****************************************************/

uint32_t dec_to_hex(uint32_t decimal_val, uint8_t *dec2hex_buf)
{
    uint8_t dec2hex_tempbuf[EIGHT];
    uint8_t dec2hex_var1 			= HEX_ZERO;
	uint8_t dec2hex_var2			= HEX_ZERO;
	uint8_t dec2hex_var3 			= HEX_ZERO;
    uint8_t hex_buf[HEX_SIX] 		= {HEX_A, HEX_B, HEX_C, HEX_D, HEX_E, HEX_F};

    bzero(dec2hex_buf, HEX_FOUR);
    bzero(dec2hex_tempbuf, sizeof(dec2hex_tempbuf));

    if(decimal_val != HEX_ZERO)
    {
        while(decimal_val > DEC_15)
        {
            dec2hex_tempbuf[dec2hex_var1] = decimal_val % DEC_16;
            decimal_val = decimal_val / DEC_16;
            dec2hex_var1++;
        }  
        dec2hex_tempbuf[dec2hex_var1] = decimal_val;
        dec2hex_var2 = dec2hex_var1;
        for(dec2hex_var1 = HEX_ZERO; dec2hex_var1 <= dec2hex_var2; dec2hex_var1++)
        {
            for(dec2hex_var3 = HEX_TEN; dec2hex_var3 <= DEC_15; dec2hex_var3++)
            {
                if(dec2hex_tempbuf[dec2hex_var1] == dec2hex_var3)
                {
                    dec2hex_tempbuf[dec2hex_var1] = hex_buf[dec2hex_var3 - HEX_TEN];
                    break;
                }
                else
                {
                    ;
                }
            }
           
        } 
        dec2hex_var1 = HEX_ONE;
        dec2hex_var2 = HEX_ZERO;
        while(dec2hex_var1 < HEX_EIGHT)
        {
            dec2hex_buf[dec2hex_var2] = dec2hex_tempbuf[dec2hex_var1] * HEX_TEN1 + dec2hex_tempbuf[dec2hex_var1 - HEX_ONE];
            dec2hex_var2++;
            dec2hex_var1 = dec2hex_var1 + HEX_TWO;
        }    
    } 
    else
    {
        ;  
    }
    return OK;	

}


/** FHdr-beg *****************************************************
**
** Function name: send_ap_status
**
** Anchor:        RCS_SOW_REQ003
**
** Purpose:       send ap status 
**
**
** Inputs:        None    
**
**
** Outputs:       None
**
** FPrj-beg *****************************************************
**
** FPrj-end *****************************************************
** FHdr-end *****************************************************/


int32_t send_ap_status()
{
    uint8_t send_ap_loop = HEX_ZERO;
    bzero((int8_t *)&mcs_ap_presence,sizeof(mcs_ap_presence));

    mcs_ap_presence.message_code = MCS_MSC_ADDITIONAL_PARAMS_PRESENCE;
    mcs_ap_presence.message_size = sizeof(mcs_ap_presence)- HEX_FOUR;
    mcs_ap_presence.time_sec     = time_send.sec;
    mcs_ap_presence.time_fracsec = time_send.fracsec;
    for(send_ap_loop =HEX_ZERO; send_ap_loop < TOTAL_RADIOS ; send_ap_loop++)
    {
        mcs_ap_presence.ap_status[send_ap_loop] = radio_ap_present[send_ap_loop+ONE];
       
    }
    if((send(ctrl_mcs_sockfd, &mcs_ap_presence, sizeof(mcs_ap_presence), HEX_ZERO)) <= HEX_ZERO)
    { 
       
        
        return(ERROR); //NW_Reconnect_Fun();
    }
    return(OK);
}


/** FHdr-beg *****************************************************
**
** Function name: send_alert_Msg
**
** Anchor:       RCS_SOW_REQ003 
**
** Purpose:       send alert MSG 
**
**
** Inputs:        system id, alert msg types    
**
**
** Outputs:       connection details 
**
** FPrj-beg *****************************************************
**
** FPrj-end *****************************************************
** FHdr-end *****************************************************/

int32_t send_alert_Msg(uint8_t subsystem_id, uint8_t alert_msg_type)
{
    struct MCS_MSC_ALERT_MSG
    {
        uint16_t  message_code;
        uint16_t  message_size;
        uint32_t  time_sec;
        uint32_t  time_fracsec;
        uint8_t   subsystem_id;
        uint8_t   alert_type;
        
    } mcs_alert_msgs;
    
    bzero((int8_t *)&mcs_alert_msgs,sizeof(mcs_alert_msgs));
    
    mcs_alert_msgs.message_code = MCS_MSC_ALERTMSG;
    mcs_alert_msgs.message_size = sizeof(mcs_alert_msgs)-HEX_FOUR;
    mcs_alert_msgs.time_sec     = time_send.sec;
    mcs_alert_msgs.time_fracsec = time_send.fracsec;
    mcs_alert_msgs.subsystem_id = subsystem_id;
    mcs_alert_msgs.alert_type   = alert_msg_type;    
    if(ctrl_mcs_sockfd != ERROR)
    {       
        if(send(ctrl_mcs_sockfd,(int8_t *)&mcs_alert_msgs,sizeof(mcs_alert_msgs),HEX_ZERO) <= HEX_ZERO)
        {           
            
            return(ERROR);//NW_Reconnect_Fun();
        }
    }
    return(OK);
}



/** FHdr-beg *****************************************************
**
** Function name: send_mcs_status
**
** Anchor:        RCS_SOW_REQ003
**
** Purpose:       send mcs status
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
int32_t send_mcs_status()
{
    uint8_t send_tr_loop = HEX_ZERO;

    bzero((int8_t *)&mcs_status, sizeof(mcs_status));    
    mcs_status.message_code = MCS_MSC_STATUS;
    mcs_status.message_size = sizeof(mcs_status) - HEX_FOUR;
    mcs_status.time_sec     = time_send.sec;
    mcs_status.time_fracsec = time_send.fracsec;    
    mcs_status.intercom_Status = AMS_Present_State;      
    for(send_tr_loop = HEX_ZERO; send_tr_loop < TOTAL_RADIOS; send_tr_loop++)
    {
        mcs_status.tr_status[send_tr_loop] = radio_present_state[send_tr_loop + ONE];
       
    }    
    if((send(ctrl_mcs_sockfd, (int8_t *)&mcs_status, sizeof(mcs_status), HEX_ZERO)) <= HEX_ZERO)
    {
              
        return (ERROR);  //NW_ReConnect_Fun();
    }
    return OK;
}



/** FHdr-beg *****************************************************
**
** Function name: checking communication 
**
** Anchor:        RCS_SOW_REQ003
**
** Purpose:       radio id, serial fd
**
**
** Inputs:        communication check radio id , communication check serial fd    
**
**
** Outputs:       communication radio parameter set status 
**
** FPrj-beg *****************************************************
**
** FPrj-end *****************************************************
** FHdr-end *****************************************************/

uint8_t comm_check(uint8_t commchk_radio_id, int32_t commchk_serialfd)
{
    uint8_t commchk_packto_radio[HEX_SIX]        = "";
    uint8_t commchk_ack_packfromradio[HEX_SEVEN] = "";
    uint8_t commchk_expectedpackidset[HEX_FOUR]  = "";
    uint8_t commchk_radioparam_setstatus         = FAILURE_STATUS;
    uint8_t commchk_radiopack_sendcount          = HEX_ONE;
    while(commchk_radiopack_sendcount <= SENDPACK_MED)
    {
        f_comm_check(commchk_radio_id, commchk_packto_radio);         
        n = write(commchk_serialfd, &commchk_packto_radio, HEX_SIX);
        usleep(DEC_10); 
        commchk_expectedpackidset[ZERO]  = HEX_ONE;
        commchk_expectedpackidset[ONE]   = HEX_SIX;
        commchk_expectedpackidset[TWO]   = HEX_SEVEN;       
        commchk_expectedpackidset[THREE] = HEX_FOUR;        
        serial_read(commchk_serialfd, commchk_ack_packfromradio, commchk_expectedpackidset);        
        if(commchk_expectedpackidset[ZERO] == commchk_ack_packfromradio[ONE] && commchk_expectedpackidset[ONE] == commchk_ack_packfromradio[FOUR])
        {
            commchk_radiopack_sendcount  = LOOP_TERMINATE;
            commchk_radioparam_setstatus = SUCCESS_STATUS;
        }
        else
        {
            commchk_radiopack_sendcount++;
            usleep(MILISEC * DEC_10);                       //taskdelay(10); 
        }
    }
    return (commchk_radioparam_setstatus);

}


/** FHdr-beg *****************************************************
**
** Function name: radio_init 
**
** Anchor:     	RCS_SOW_REQ001   
**
** Purpose:       function to initialize radio
**
**
** Inputs:        radio id,serial fd    
**
**
** Outputs:      init status 
**
** FPrj-beg *****************************************************
**
** FPrj-end *****************************************************
** FHdr-end *****************************************************/


uint8_t radio_init(uint8_t init_radio_id, int32_t init_serialfd)
{
    uint8_t init_packto_radio[EIGHT]       = "";
    uint8_t init_radioparamload[TWO]       = "";
    uint8_t init_ack_packfromradio[SEVEN]  = "";
    uint8_t init_expectedpackidset[FOUR]   = "";
    uint8_t init_radioparam_setstatus      = FAILURE_STATUS;
    uint8_t init_radiopack_sendcount       = HEX_ONE;
    while(init_radiopack_sendcount <= SENDPACK_MED)
    {
        //init_radioparamload[0] = init_radio_id;
        init_radioparamload[ONE] = HEX_SEVEN;   
        f_initialisation(init_radioparamload[ONE], init_packto_radio);
        /* MCS Controller sending radio init packets to RIB */       
        n = write(init_serialfd, &init_packto_radio, HEX_EIGHT);
        usleep(DEC_10);   
        init_expectedpackidset[ZERO] = HEX_ONE;   /* ACK */
        init_expectedpackidset[ONE] = HEX_ZERO;   /* ID */
        init_expectedpackidset[TWO] = HEX_SEVEN;   /* LEN */             /* len changed from 0x08 to 0x07 */            
        init_expectedpackidset[THREE] = HEX_FOUR;   /* TIME OUT SEC */
        /* MCS Controller receiving ACK from radio through RIB */
        //RIB_recvpack(&init_serialfd, init_ack_packfromradio, init_expectedpackidset);
        serial_read(init_serialfd, init_ack_packfromradio, init_expectedpackidset);    
        if(init_expectedpackidset[ZERO] == init_ack_packfromradio[ONE] && init_expectedpackidset[ONE] == init_ack_packfromradio[FOUR])
        {               
            init_radiopack_sendcount  = LOOP_TERMINATE;
            init_radioparam_setstatus = SUCCESS_STATUS;  // 0xFF            
        }
        else
        {
             
            init_radiopack_sendcount++; 
            sleep(ONE);                                  //delay(); // verify delay in original code 
        }

    }  
     
    return(init_radioparam_setstatus);

}


/** FHdr-beg *****************************************************
**
** Function name: radio_tr_comm_check 
**
** Anchor:        RCS_SOW_REQ003
**
** Purpose:       radio transmission communication check
**
**
** Inputs:        radio id   
**
**
** Outputs:      None 
**
** FPrj-beg *****************************************************
**
** FPrj-end *****************************************************
** FHdr-end *****************************************************/
void *radio_tr_comm_check(void *arg)
{
    uint8_t trcc_radio_id = *(uint8_t *)arg;
    radio_commstatus[trcc_radio_id] = HEX_ONE;
    //while(TRUE)
	for(;;)
    {
        if(radio_tr_comm_stop[trcc_radio_id] == HEX_ZERO)
        {
            //semTake(radio_syncsem[trcc_radio_id], WAIT_FOREVER)
            sem_wait(&radio_syncsem[trcc_radio_id]);   
            if(radio_commstatus[trcc_radio_id] == HEX_ZERO)
            {                
                radio_param_setstatus[trcc_radio_id] = radio_init(trcc_radio_id, radio_serialfd[trcc_radio_id]);
            }
            else
            {
                radio_param_setstatus[trcc_radio_id] = comm_check(trcc_radio_id, radio_serialfd[trcc_radio_id]);
            }  
 
            if(radio_param_setstatus[trcc_radio_id] == SUCCESS_STATUS)
            {
                radio_commstatus[trcc_radio_id] == HEX_ONE; 
             
            }
            else
            {
                radio_commstatus[trcc_radio_id]   = HEX_ZERO;
                radio_present_state[trcc_radio_id] = OFF_STATE;  // 5 - OFF_STATE
            }   
            radio_isfirst_tr_complete[trcc_radio_id] = HEX_ONE;
            //semGive(radio_syncsem[trcc_radio_id]); 
            sem_post(&radio_syncsem[trcc_radio_id]);             
        }
        else
        { 
                   
            break;
        }  
        usleep(SEC * DEC_10);          //taskdelay(sysclkrateget() * 10);
    }      
    //radio_trcomm_threadid[trcc_radio_id] = ERROR;
    radio_tr_comm_thread[trcc_radio_id] = HEX_ZERO;
    return(OK);

}


/** FHdr-beg *****************************************************
**
** Function name: clear_nvm
**
** Anchor:        RCS_SOW_REQ003
**
** Purpose:       This function clear NVM 
**
**
** Inputs:        radio id  ,cnvm serial fd  
**
**
** Outputs:      cleat nvm status  
**
** FPrj-beg *****************************************************
**
** FPrj-end *****************************************************
** FHdr-end *****************************************************/

uint8_t clear_nvm(uint8_t cnvm_radio_id, int32_t cnvm_serialfd)
{
    uint8_t cnvm_packto_radio[SEVEN]       = "";
    uint8_t cnvm_ack_packfromradio[EIGHT]  = "";
    uint8_t cnvm_expectedpackidset[FOUR]   = "";
    uint8_t cnvm_radioparam_setstatus      = FAILURE_STATUS;
    uint8_t cnvm_radiopack_sendcount       = HEX_ONE;

    while(cnvm_radiopack_sendcount <= SENDPACK_MED)
    {
        f_clear_nvm(cnvm_radio_id, cnvm_packto_radio);
        n = write(cnvm_serialfd, &cnvm_packto_radio, HEX_SEVEN);
        usleep(DEC_10); 
        cnvm_expectedpackidset[ZERO]  = HEX_ONE;
        cnvm_expectedpackidset[ONE]   = HEX_0X82;
        cnvm_expectedpackidset[TWO]   = HEX_SEVEN;               /* len changed from 0x08 to 0x07 */
        cnvm_expectedpackidset[THREE] = HEX_FOUR;
        serial_read(cnvm_serialfd, cnvm_ack_packfromradio, cnvm_expectedpackidset);        
        if(cnvm_expectedpackidset[ZERO] == cnvm_ack_packfromradio[ONE] && cnvm_expectedpackidset[ONE] == cnvm_ack_packfromradio[FOUR])
        {           
            cnvm_radiopack_sendcount  = LOOP_TERMINATE;
            cnvm_radioparam_setstatus = SUCCESS_STATUS;            
        }
        else
        {            
            cnvm_radiopack_sendcount++;
            usleep(MILISEC * DEC_10);           
        }  
    }
    return (cnvm_radioparam_setstatus);

}


/** FHdr-beg *****************************************************
**
** Function name: radiation_status
**
** Anchor:        RCS_SOW_REQ003
**
** Purpose:       This function check the status of the radiation 
**
**
** Inputs:        radio id  ,radiation serial fd  
**
**
** Outputs:      cleat nvm status  
**
** FPrj-beg *****************************************************
**
** FPrj-end *****************************************************
** FHdr-end *****************************************************/

uint8_t radiation_status(uint8_t radn_radio_id, int32_t radn_serialfd)
{
    uint8_t radn_radioparamload             = HEX_ZERO;
    uint8_t radn_packfromradio[DEC_50]      = "";   
    uint8_t radn_sr_packto_radio[SEVEN]     = "";
    uint8_t radn_expectedpackidset[FOUR]    = "";
    uint8_t radn_ack_packfromradio[EIGHT]   = "";
    uint8_t radn_sr_ack_packto_radio[EIGHT] = "";
    uint8_t radn_radioparam_setstatus       = HEX_0XEE;
    uint8_t radn_radiopack_sendcount        = HEX_ZERO;    
    radn_radiopack_sendcount                = HEX_ONE;    
    while(radn_radiopack_sendcount <= SENDPACK_MED)
    {       
        
        radn_radioparamload = DEC_17;                
        f_status_request(radn_radioparamload, radn_sr_packto_radio);       
        n = write(radn_serialfd, &radn_sr_packto_radio, HEX_SEVEN);
        usleep(DEC_10);      
       //printf("Controller Receiving <RC-Status-Ack> Packet from <Radio-%d> \n\r", radn_radio_id);        
        radn_expectedpackidset[ZERO]   = HEX_ONE;
        radn_expectedpackidset[ONE]    = HEX_0x22;
        radn_expectedpackidset[TWO]    = HEX_SEVEN;   
        radn_expectedpackidset[THREE]  = HEX_FOUR;        
        serial_read(radn_serialfd, radn_ack_packfromradio, radn_expectedpackidset);         
        if(radn_expectedpackidset[ZERO] == radn_ack_packfromradio[ONE] && radn_expectedpackidset[ONE] == radn_ack_packfromradio[FOUR])
        {                      
            radn_expectedpackidset[ZERO]  = HEX_FIVE;
            radn_expectedpackidset[ONE]   = HEX_ZERO;
            radn_expectedpackidset[TWO]   = HEX_0x32;
            radn_expectedpackidset[THREE] = HEX_C;   
            serial_read(radn_serialfd, radn_packfromradio, radn_expectedpackidset);             
            if(radn_expectedpackidset[ZERO] == radn_packfromradio[ONE])  
            {               
                radn_radioparamload = radn_packfromradio[ONE];                 
                f_ackpack_toradio(radn_radioparamload, radn_sr_ack_packto_radio);                
                n = write(radn_serialfd, &radn_sr_ack_packto_radio, HEX_SEVEN);
                usleep(DEC_10);              
                radn_radiopack_sendcount = LOOP_TERMINATE;                
                radn_radioparam_setstatus = radn_packfromradio[NINE]; 
            }
            else
            {
               // printf("RC Status Failed for <Radio-%d> \n\r", radn_radio_id);
                radn_radiopack_sendcount++;
                usleep(MILISEC * DEC_30);   // taskDelay(30)
            }
        } 
        else
        {
            //printf("RC Status Failed for <Radio-%d> \n\r", radn_radio_id);
            radn_radiopack_sendcount++;
            usleep(MILISEC * DEC_30);   // taskDelay(30)
        }        
    }
    return(radn_radioparam_setstatus);
}


/** FHdr-beg *****************************************************
**
** Function name: radiation_on_off
**
** Anchor:        RCS_SOW_REQ003
**
** Purpose:       This function perform radioation ON OFF 
**
**
** Inputs:        radiation radio id  ,radiation serial fd  
**
**
** Outputs:      radiation ON OFF status  
**
** FPrj-beg *****************************************************
**
** FPrj-end *****************************************************
** FHdr-end *****************************************************/

uint8_t radiation_on_off(uint8_t radn_radio_id, int32_t radn_serialfd, uint8_t radn_radioparam_new[TOTAL_RADIOS_PLUS_ONE][TEN]) 
{
    uint8_t radn_radioparam[DEC_10]        = "";
    uint8_t radn_packto_radio[DEC_50]      = "";  
    uint8_t radn_radioparam_load[TWO]      = "";
    uint8_t radn_expectedpackidset[FOUR]   = "";
    uint8_t radn_ack_packfromradio[SEVEN]  = "";
    uint8_t radn_loop                      = HEX_ZERO;
    uint8_t radn_radioparam_setstatus      = FAILURE_STATUS;
    uint8_t radn_radiopack_sendcount       = HEX_ONE;

    for(radn_loop = HEX_ZERO; radn_loop < DEC_10; radn_loop++)
    {
        radn_radioparam[radn_loop] = radn_radioparam_new[radn_radio_id][radn_loop];
    }

    while(radn_radiopack_sendcount <= SENDPACK_MED)
    {        
        bzero(radn_radioparam_load, sizeof(radn_radioparam_load));
        radn_radioparam_load[ONE] = radn_radioparam[ZERO];
        f_radiation_on_off(radn_radioparam_load, radn_packto_radio);       
        n = write(radn_serialfd, &radn_packto_radio, DEC_50);
        usleep(DEC_10);  
        radn_expectedpackidset[ZERO]   = HEX_ONE;
        radn_expectedpackidset[ONE]    = HEX_FIVE;
        radn_expectedpackidset[TWO]    = HEX_SEVEN;  
        radn_expectedpackidset[THREE]  = HEX_FOUR;
        serial_read(radn_serialfd, radn_ack_packfromradio, radn_expectedpackidset);  
        if(radn_expectedpackidset[ZERO] == radn_ack_packfromradio[ONE] && radn_expectedpackidset[ONE] == radn_ack_packfromradio[FOUR]) 
        {
            radn_radiopack_sendcount  = LOOP_TERMINATE;
            radn_radioparam_setstatus = SUCCESS_STATUS;
        }
        else
        {
            radn_radiopack_sendcount++;
            usleep(MILISEC * DEC_10);   // taskDelay(10)
        }        
    }
    return (radn_radioparam_setstatus);
} 


/** FHdr-beg *****************************************************
**
** Function name: emergency_guard
**
** Anchor:        RCS_SOW_REQ003
**
** Purpose:       This function perform emergency guard for radio  
**
**
** Inputs:        emergency radio id  ,emergency serial fd  
**
**
** Outputs:      radiation ON OFF status  
**
** FPrj-beg *****************************************************
**
** FPrj-end *****************************************************
** FHdr-end *****************************************************/

uint8_t emergency_guard(uint8_t emg_radio_id, int32_t emg_serialfd, uint8_t emg_radioparam_new[TOTAL_RADIOS_PLUS_ONE][TEN])
{
    uint8_t emg_radioparam[DEC_10]       = "";
    uint8_t emg_packto_radio[SEVEN]      = "";  
    uint8_t emg_radioparam_load[TWO]     = "";
    uint8_t emg_ack_packfromradio[EIGHT] = "";
    uint8_t emg_expectedpackidset[FOUR]  = "";
    uint8_t emg_loop                     = HEX_ZERO;
    uint8_t emg_radioparam_setstatus     = FAILURE_STATUS;
    uint8_t emg_radiopack_sendcount      = HEX_ONE;
    for(emg_loop = HEX_ZERO; emg_loop < DEC_10; emg_loop++)
    {
        emg_radioparam[emg_loop] = emg_radioparam_new[emg_radio_id][emg_loop];
    }
    emg_radiopack_sendcount = HEX_ONE;
    while(emg_radiopack_sendcount <= SENDPACK_MED)
    {       
        bzero(emg_radioparam_load, sizeof(emg_radioparam_load));
        emg_radioparam_load[ONE] = emg_radioparam[ONE]; 
        f_emergency_guard(emg_radioparam_load, emg_packto_radio);
        n = write(emg_serialfd, &emg_packto_radio, HEX_SEVEN);
        usleep(DEC_10);  
        emg_expectedpackidset[ZERO]   = HEX_ONE;
        emg_expectedpackidset[ONE]    = HEX_0X27;
        emg_expectedpackidset[TWO]    = HEX_SEVEN;  
        emg_expectedpackidset[THREE]  = HEX_FOUR;
        serial_read(emg_serialfd, emg_ack_packfromradio, emg_expectedpackidset);     
        if(emg_expectedpackidset[ZERO] == emg_ack_packfromradio[ONE] && emg_expectedpackidset[ONE] == emg_ack_packfromradio[FOUR]) 
        {            
            emg_radiopack_sendcount  = LOOP_TERMINATE;
            emg_radioparam_setstatus = SUCCESS_STATUS;            
        }
        else
        {            
            emg_radiopack_sendcount++;
            usleep(MILISEC * DEC_10);   // taskDelay(10)
        }
    }
    return(emg_radioparam_setstatus);
}



/** FHdr-beg *****************************************************
**
** Function name: Activate_LOS_Preset
**
** Anchor:       RCS_SOW_REQ003 
**
** Purpose:       This function perform Activate LOS Preset 
**
**
** Inputs:        activate radio id  ,activate serial fd  
**
**
** Outputs:      Activate LOS preset status  
**
** FPrj-beg *****************************************************
**
** FPrj-end *****************************************************
** FHdr-end *****************************************************/

uint8_t activate_los_preset(uint8_t alosp_radio_id, int32_t alosp_serialfd)
{
    uint8_t alosp_packto_radio[NINE]        = "";  
    uint8_t alosp_radioparamload            = HEX_ZERO;
    uint8_t alosp_ss_packto_radio[DEC_117]  = "";
    uint8_t alosp_ack_packfromradio[EIGHT]  = "";
    uint8_t alosp_expectedpackidset[FOUR]   = "";
    uint8_t alosp_radiopack_sendcount       = HEX_ONE;
    uint8_t alosp_radiopack_sendcount1      = HEX_ONE;
    uint8_t alosp_radiopack_sendcount2      = HEX_ONE;
    uint8_t alosp_radiopack_sendcount3      = HEX_ONE;
    uint8_t alosp_radioparam_setstatus      = FAILURE_STATUS;
    
    while(alosp_radiopack_sendcount <= SENDPACK_MED)
    {
        alosp_radiopack_sendcount1 = HEX_ONE;
        while(alosp_radiopack_sendcount1 <= SENDPACK_LOW)
        {            
            alosp_radioparamload = DEC_10; //variant 10-RPW
            f_activate_setup(alosp_radioparamload, alosp_packto_radio);
            n = write(alosp_serialfd, &alosp_packto_radio, HEX_NINE);
            usleep(DEC_10);             
	        alosp_expectedpackidset[ZERO]  = HEX_ONE;
            alosp_expectedpackidset[ONE]   = HEX_0X26;
            alosp_expectedpackidset[TWO]   = HEX_SEVEN; 
            alosp_expectedpackidset[THREE] = HEX_FOUR;
            serial_read(alosp_serialfd, alosp_ack_packfromradio, alosp_expectedpackidset);
            if(alosp_expectedpackidset[ZERO] == alosp_ack_packfromradio[ONE] && alosp_expectedpackidset[ONE] == alosp_ack_packfromradio[FOUR]) 
            {
                alosp_radiopack_sendcount1 = LOOP_TERMINATE;
                alosp_radiopack_sendcount2 = HEX_ONE;    
                while(alosp_radiopack_sendcount2 <= SENDPACK_LOW)
                {
                    f_storesetup_rpwman(alosp_radio_id, alosp_ss_packto_radio);
                    n = write(alosp_serialfd, &alosp_ss_packto_radio, DEC_117);
                    usleep(DEC_10);                     
		            alosp_expectedpackidset[ZERO]  = HEX_ONE;
                    alosp_expectedpackidset[ONE]   = HEX_0X25;
                    alosp_expectedpackidset[TWO]   = HEX_SEVEN; 
                    alosp_expectedpackidset[THREE] = HEX_FOUR;    
                    serial_read(alosp_serialfd, alosp_ack_packfromradio, alosp_expectedpackidset);     
                    if(alosp_expectedpackidset[ZERO] == alosp_ack_packfromradio[ONE] && alosp_expectedpackidset[ONE] == alosp_ack_packfromradio[FOUR])
                    {
                        alosp_radiopack_sendcount2 = LOOP_TERMINATE;
                        alosp_radiopack_sendcount3 = HEX_ONE;
                        while(alosp_radiopack_sendcount3 <= SENDPACK_LOW)
                        {                           
                            alosp_radioparamload = HEX_THREE;  //variant 3-Normal
                            f_activate_setup(alosp_radioparamload, alosp_packto_radio);
                            n = write(alosp_serialfd, &alosp_packto_radio, HEX_NINE);
                            usleep(DEC_10);                              
			                alosp_expectedpackidset[ZERO]  = HEX_ONE;
            		        alosp_expectedpackidset[ONE]   = HEX_0X26;
                            alosp_expectedpackidset[TWO]   = HEX_SEVEN; 
                            alosp_expectedpackidset[THREE] = HEX_FOUR;
                            serial_read(alosp_serialfd, alosp_ack_packfromradio, alosp_expectedpackidset);                                                         
                            if(alosp_expectedpackidset[ZERO] == alosp_ack_packfromradio[ONE] && alosp_expectedpackidset[ONE] == alosp_ack_packfromradio[FOUR])
                            {
                                alosp_radiopack_sendcount3 = LOOP_TERMINATE;
                                alosp_radiopack_sendcount  = LOOP_TERMINATE;
                                alosp_radioparam_setstatus = SUCCESS_STATUS;
                            }
                            else
                            { 
                                if(alosp_radiopack_sendcount3 == SENDPACK_LOW)
                                {
                                    alosp_radiopack_sendcount++;
                                }
                                else
                                {
                                    ;
                                }
                                alosp_radiopack_sendcount3++;
                                usleep(MILISEC * DEC_10);   // taskDelay(10)
                            }
                        }
                    }
                    else
                    {                       
                        if(alosp_radiopack_sendcount2 == SENDPACK_LOW)
                        {
                            alosp_radiopack_sendcount++;
                        }
                        else
                        {
                            ;
                        }
                        alosp_radiopack_sendcount2++;
                        usleep(MILISEC * DEC_10);  // taskDelay(10)
                    }  
                }  
            }
            else
            {                
                if(alosp_radiopack_sendcount1 == SENDPACK_LOW)
                {
                    alosp_radiopack_sendcount++;
                }
                else
                {
                    ;
                }
                alosp_radiopack_sendcount1++;
                usleep(MILISEC * DEC_10);  // taskDelay(10)  
            }
        }  
    }  
    return (alosp_radioparam_setstatus);    
}


/** FHdr-beg *****************************************************
**
** Function name: store_setup_los
**
** Anchor:        RCS_SOW_REQ003
**
** Purpose:       This function perform store setup los 
**
**
** Inputs:        ss radio id  ,ss serial fd  
**
**
** Outputs:      LOS status  
**
** FPrj-beg *****************************************************
**
** FPrj-end *****************************************************
** FHdr-end *****************************************************/

uint8_t store_setup_los(uint8_t ss_radio_id, int32_t ss_serialfd, uint8_t ss_radioparam_new[TOTAL_RADIOS_PLUS_ONE][TEN], 
		       uint8_t ss_radioparam_setstatus[TOTAL_RADIOS_PLUS_ONE][FOUR], uint8_t atc_freq_mode)
{
    uint8_t ss_radioparam[DEC_10]        = "";
    uint8_t ss_packto_radio[DEC_117]     = "";  
    uint8_t ss_radioparam_load[DEC_10]   = "";
    uint8_t ss_as_packto_radio[NINE]     = "";  
    uint8_t ss_pr_packto_radio[NINE]     = "";   
    uint8_t ss_ack_packto_radio[SEVEN]   = "";  
    uint8_t ss_ack_packfromradio[SEVEN]  = "";  
    uint8_t ss_pr_packfromradio[DEC_117] = "";  
    uint8_t ss_expectedpackidset[FOUR]   = "";
    uint8_t ss_loop 					 = HEX_ZERO;
    uint8_t ss_radiopack_sendcount 		 = HEX_ONE;
    uint8_t ss_radiopack_sendcount1 	 = HEX_ONE;
    uint8_t ss_radiopack_sendcount2 	 = HEX_ONE;
    uint8_t ss_radiopack_sendcount3 	 = HEX_ONE;
    ss_radioparam_setstatus[ss_radio_id][ZERO]  = FAILURE_STATUS;
    ss_radioparam_setstatus[ss_radio_id][ONE]   = FAILURE_STATUS;
    ss_radioparam_setstatus[ss_radio_id][TWO]   = FAILURE_STATUS;
    ss_radioparam_setstatus[ss_radio_id][THREE] = FAILURE_STATUS;

    for(ss_loop = HEX_ZERO; ss_loop < DEC_10; ss_loop++)
    {
        ss_radioparam[ss_loop] = ss_radioparam_new[ss_radio_id][ss_loop];
    }           
    ss_radiopack_sendcount  = HEX_ONE;
    while(ss_radiopack_sendcount <= SENDPACK_MED)
    {
        ss_radiopack_sendcount1 = HEX_ONE;
        while(ss_radiopack_sendcount1 <= SENDPACK_LOW)
        {
            bzero(ss_radioparam_load, sizeof(ss_radioparam_load));
            ss_radioparam_load[ONE]    = ss_radioparam[TWO];
            ss_radioparam_load[TWO]    = ss_radioparam[THREE];
            ss_radioparam_load[THREE]  = ss_radioparam[FOUR];
            ss_radioparam_load[FOUR]   = ss_radioparam[FIVE];
            ss_radioparam_load[FIVE]   = ss_radioparam[SIX];
            ss_radioparam_load[SIX]    = ss_radioparam[SEVEN];
            ss_radioparam_load[SEVEN]  = ss_radioparam[EIGHT];
            ss_radioparam_load[EIGHT]  = atc_freq_mode;            
            f_store_setup(ss_radioparam_load, ss_packto_radio);
            n = write(ss_serialfd, &ss_packto_radio, DEC_117);
            usleep(DEC_10);  
            ss_expectedpackidset[ZERO] = HEX_ONE;
            ss_expectedpackidset[ONE]  = HEX_0X25;
            ss_expectedpackidset[TWO]  = HEX_SEVEN; 
            ss_expectedpackidset[THREE]= HEX_FOUR;
            serial_read(ss_serialfd, ss_ack_packfromradio, ss_expectedpackidset);     
            if(ss_expectedpackidset[ZERO] == ss_ack_packfromradio[ONE] && ss_expectedpackidset[ONE] == ss_ack_packfromradio[FOUR])
            {
                ss_radiopack_sendcount1 = LOOP_TERMINATE;
                ss_radiopack_sendcount2 = HEX_ONE;
                while(ss_radiopack_sendcount2 <= SENDPACK_LOW)
                {
                    bzero(ss_radioparam_load, sizeof(ss_radioparam_load));
                    ss_radioparam_load[ONE] = HEX_THREE;
                    f_activate_setup(ss_radioparam_load[ONE], ss_as_packto_radio);
                    n = write(ss_serialfd, &ss_as_packto_radio, HEX_NINE);
                    usleep(DEC_10);
                    ss_expectedpackidset[ZERO]  = HEX_ONE;
            	    ss_expectedpackidset[ONE]   = HEX_0X26;
            	    ss_expectedpackidset[TWO]   = HEX_SEVEN; 
		            ss_expectedpackidset[THREE] = HEX_FOUR;
                    serial_read(ss_serialfd, ss_ack_packfromradio, ss_expectedpackidset);    
                    if(ss_expectedpackidset[ZERO] == ss_ack_packfromradio[ONE] && ss_expectedpackidset[ONE] == ss_ack_packfromradio[FOUR])
                    {
                        ss_radiopack_sendcount2 = LOOP_TERMINATE;
                        if(SkipFirstPresetFreq[ss_radio_id] != HEX_ONE)
                        {
                            ss_radiopack_sendcount3 = HEX_ONE;
                            while(ss_radiopack_sendcount3 <= SENDPACK_HIGH)
                            {
                                usleep(MILISEC * DEC_30);  // taskDelay(30)                         
                                bzero(ss_radioparam_load, sizeof(ss_radioparam_load));
                                ss_radioparam_load[ONE] = HEX_THREE;
                                f_preset_request(ss_radioparam_load[HEX_ONE], ss_pr_packto_radio);
                                n = write(ss_serialfd, &ss_pr_packto_radio, HEX_NINE);
                                usleep(DEC_10);                               
			                    ss_expectedpackidset[ZERO]  = HEX_ONE;
            	    		    ss_expectedpackidset[ONE]   = HEX_0X2B;
            	    		    ss_expectedpackidset[TWO]   = HEX_SEVEN; 
		    		            ss_expectedpackidset[THREE] = HEX_FOUR;
                                serial_read(ss_serialfd, ss_ack_packfromradio, ss_expectedpackidset);     
                                if(ss_expectedpackidset[ZERO] == ss_ack_packfromradio[ONE] && ss_expectedpackidset[ONE] == ss_ack_packfromradio[FOUR])
                                {   
                                    ss_expectedpackidset[ZERO]  = HEX_0XA6;
                                    ss_expectedpackidset[ONE]   = HEX_ZERO;
                                    ss_expectedpackidset[TWO]   = HEX_0x75; 
                                    ss_expectedpackidset[THREE] = HEX_A; 
                                    serial_read(ss_serialfd, ss_pr_packfromradio, ss_expectedpackidset);      
                                    if(ss_expectedpackidset[ZERO] == ss_pr_packfromradio[ONE])  
                                    {
                                        ss_radiopack_sendcount3 = LOOP_TERMINATE;                                       
                                        bzero(ss_radioparam_load, sizeof(ss_radioparam_load));
                                        ss_radioparam_load[ONE] = ss_pr_packfromradio[ONE]; 
                                        f_ackpack_toradio(ss_radioparam_load[ONE], ss_ack_packto_radio);
                                        n = write(ss_serialfd, &ss_ack_packto_radio, HEX_SEVEN);
                                        usleep(DEC_10);   
                                        if(ss_radioparam[TWO]   == ss_pr_packfromradio[SEVEN]  && ss_radioparam[THREE] ==ss_pr_packfromradio[DEC_16] &&
                                           ss_radioparam[FOUR]  == ss_pr_packfromradio[DEC_17] && ss_radioparam[FIVE]  == ss_pr_packfromradio[DEC_18]  && 
                                           ss_radioparam[SIX]   == ss_pr_packfromradio[DEC_19] && ss_radioparam[SEVEN] == ss_pr_packfromradio[DEC_21] && 
                                           ss_radioparam[EIGHT] == ss_pr_packfromradio[DEC_24] && atc_freq_mode        == ss_pr_packfromradio[DEC_105])
                                        {
                                            ss_radiopack_sendcount  = LOOP_TERMINATE;
                                            ss_radioparam_setstatus[ss_radio_id][ZERO]  = HEX_FF;
                                            ss_radioparam_setstatus[ss_radio_id][ONE]   = HEX_FF;
                                            ss_radioparam_setstatus[ss_radio_id][TWO]   = HEX_FF;
                                            ss_radioparam_setstatus[ss_radio_id][THREE] = HEX_FF;
                                        }
                                        else if(ss_radioparam[TWO] == ss_pr_packfromradio[SEVEN]  || ss_radioparam[THREE] == ss_pr_packfromradio[DEC_16] ||
                                                ss_radioparam[FOUR] == ss_pr_packfromradio[DEC_17] || ss_radioparam[FIVE] == ss_pr_packfromradio[DEC_18] || 
                                                ss_radioparam[SIX] == ss_pr_packfromradio[DEC_19] || ss_radioparam[SEVEN] == ss_pr_packfromradio[DEC_21] || 
                                                ss_radioparam[EIGHT] == ss_pr_packfromradio[DEC_24] || atc_freq_mode    == ss_pr_packfromradio[DEC_105])
                                        {
                                            if(ss_pr_packfromradio[SEVEN] == ss_radioparam[TWO]) 
                                            {
                                                ss_radioparam_setstatus[ss_radio_id][ZERO] = HEX_FF;
                                            }
                                            else
 					                        {
                                                ss_radioparam_setstatus[ss_radio_id][ZERO] = HEX_ZERO;
                                            }                                            
                                            if(ss_radioparam[THREE] == ss_pr_packfromradio[DEC_16] && ss_radioparam[FOUR] == ss_pr_packfromradio[DEC_17] &&
                                               ss_radioparam[FIVE] == ss_pr_packfromradio[DEC_18] && ss_radioparam[SIX] == ss_pr_packfromradio[DEC_19] &&  
                                               atc_freq_mode    == ss_pr_packfromradio[DEC_105])
                                            {
                                                ss_radioparam_setstatus[ss_radio_id][ONE] = HEX_FF;
                                            }
                                            else
                                            {
                                                ss_radioparam_setstatus[ss_radio_id][ONE] = HEX_ZERO;
                                            }     
                                            if(ss_pr_packfromradio[DEC_21] == ss_radioparam[SEVEN])     
                                            {
                                                ss_radioparam_setstatus[ss_radio_id][TWO] = HEX_FF;
                                            }
                                            else
 					                        {
                                                ss_radioparam_setstatus[ss_radio_id][TWO] = HEX_ZERO;
                                            }
                                            if(ss_pr_packfromradio[DEC_24] == ss_radioparam[EIGHT])       
                                            {
                                                ss_radioparam_setstatus[ss_radio_id][THREE] = HEX_FF;
                                            }
                                            else
 					                        {
                                                ss_radioparam_setstatus[ss_radio_id][THREE] = HEX_ZERO;
                                            }
                                            ss_radiopack_sendcount++;
                                            usleep(MILISEC * DEC_10);  // taskDelay(10)                                            
                                        }
                                        else
                                        {                                            
                                            ss_radioparam_setstatus[ss_radio_id][ZERO] = HEX_ZERO;
                                            ss_radioparam_setstatus[ss_radio_id][ONE] = HEX_ZERO;
                                            ss_radioparam_setstatus[ss_radio_id][TWO] = HEX_ZERO;
                                            ss_radioparam_setstatus[ss_radio_id][THREE] = HEX_ZERO;
                                            ss_radiopack_sendcount++;
                                            usleep(MILISEC * DEC_10);  // taskDelay(10)                                         
                                        }   
                                    }
                                    else
                                    {                                       
                                        if(ss_radiopack_sendcount3 == SENDPACK_HIGH)
                                        {
                                            ss_radiopack_sendcount++;
                                        }
                                        ss_radiopack_sendcount3++;
                                        usleep(MILISEC * DEC_10);  // taskDelay(10)                                       
                                    }
                                }
                                else
                                {                                    
                                    if(ss_radiopack_sendcount3 == SENDPACK_HIGH)
                                    {
                                        ss_radiopack_sendcount++;
                                    }
                                    ss_radiopack_sendcount3++;
                                    usleep(MILISEC * DEC_10);  // taskDelay(10)                                    
                                }
                            }  
                        }
                        else
                        {
                            ss_radiopack_sendcount = LOOP_TERMINATE;
                            ss_radioparam_setstatus[ss_radio_id][ZERO]   = HEX_FF;
                            ss_radioparam_setstatus[ss_radio_id][ONE]    = HEX_FF;
                            ss_radioparam_setstatus[ss_radio_id][TWO]    = HEX_FF;
                            ss_radioparam_setstatus[ss_radio_id][THREE]  = HEX_FF;                  
                        }
                    }                                            
                    else
                    {                      
                        if(ss_radiopack_sendcount2 == SENDPACK_LOW)
                        {
                            ss_radiopack_sendcount++;
                        }
                        ss_radiopack_sendcount2++;
                        usleep(MILISEC * DEC_10);  // taskDelay(10)                        
                    }
                }  
            }
            else
            {
                if(ss_radiopack_sendcount1 == SENDPACK_LOW)
                {
                    ss_radiopack_sendcount++;
                }
                ss_radiopack_sendcount1++;
                usleep(MILISEC * DEC_10);  // taskDelay(10)                      
            }
        }  
    } 
    return (OK);
}


/** FHdr-beg *****************************************************
**
** Function name: squelch_th
**
** Anchor:        RCS_SOW_REQ003
**
** Purpose:       This function perform sequence  the channel 
**
**
** Inputs:        sq radio id  ,sq serial fd  
**
**
** Outputs:      sq status  
**
** FPrj-beg *****************************************************
**
** FPrj-end *****************************************************
** FHdr-end *****************************************************/
uint8_t squelch_th(uint8_t thsq_radio_id, int32_t thsq_serialfd, uint8_t thsq_radioparam_new[TOTAL_RADIOS_PLUS_ONE][TEN])
{
    uint8_t thsq_radioparam[DEC_10]          = "";
    uint8_t thsq_packto_radio[SEVEN]         = ""; 
    uint8_t thsq_packfromradio[SEVEN]        = ""; 
    uint8_t thsq_sr_packto_radio[SEVEN]      = ""; 
    uint8_t thsq_radioparam_load[TWO]        = "";
    uint8_t thsq_vol_packto_radio[SEVEN]     = ""; 
    uint8_t thsq_expectedpackidset[FOUR]     = "";
    uint8_t thsq_sr_ack_packto_radio[SEVEN]  = ""; 
    uint8_t thsq_sr_ack_packfromradio[SEVEN] = ""; 

    uint8_t thsq_loop                        = HEX_ZERO;
    uint8_t thsq_radioparam_setstatus        = FAILURE_STATUS;
    uint8_t thsq_radiopack_sendcount         = HEX_ONE;

    for(thsq_loop = HEX_ZERO; thsq_loop < DEC_10; thsq_loop++)
    {
        thsq_radioparam[thsq_loop] = thsq_radioparam_new[thsq_radio_id][thsq_loop];
    }
    f_volume(thsq_radio_id, thsq_vol_packto_radio);    
    n = write(thsq_serialfd, &thsq_vol_packto_radio, HEX_SEVEN);
    usleep(DEC_10);  
    while(thsq_radiopack_sendcount <= SENDPACK_MED)
    {
        bzero(thsq_radioparam_load, sizeof(thsq_radioparam_load));
        thsq_radioparam_load[ONE] = thsq_radioparam[NINE];
        f_th_squelch(thsq_radioparam_load, thsq_packto_radio);
        n = write(thsq_serialfd, &thsq_packto_radio, HEX_SEVEN);
        usleep(DEC_10);         
        usleep(MILISEC * DEC_30);  // taskDelay(30)     
        bzero(thsq_radioparam_load, sizeof(thsq_radioparam_load));
        thsq_radioparam_load[ONE] = DEC_21;
        f_status_request(thsq_radioparam_load[ONE], thsq_sr_packto_radio);
        n = write(thsq_serialfd, &thsq_sr_packto_radio, HEX_SEVEN);
        usleep(DEC_10);     
        thsq_expectedpackidset[ZERO]  = HEX_ONE;
        thsq_expectedpackidset[ONE]   = HEX_0x22;
        thsq_expectedpackidset[TWO]   = HEX_SEVEN; 
        thsq_expectedpackidset[THREE] = HEX_FOUR;
        serial_read(thsq_serialfd, thsq_sr_ack_packfromradio, thsq_expectedpackidset);   
        if(thsq_expectedpackidset[ZERO] == thsq_sr_ack_packfromradio[ONE] && thsq_expectedpackidset[ONE] == thsq_sr_ack_packfromradio[FOUR]) 
        {
            thsq_expectedpackidset[ZERO]   = HEX_0x21;
            thsq_expectedpackidset[ONE]    = HEX_ZERO;
            thsq_expectedpackidset[TWO]    = HEX_SEVEN;  
            thsq_expectedpackidset[THREE]  = HEX_C;
            serial_read(thsq_serialfd, thsq_packfromradio, thsq_expectedpackidset);   
            if(thsq_expectedpackidset[ZERO] == thsq_packfromradio[ONE]) 
            {
                bzero(thsq_radioparam_load, sizeof(thsq_radioparam_load));
                thsq_radioparam_load[ONE] = thsq_packfromradio[ONE]; 
                f_ackpack_toradio(thsq_radioparam_load[ONE], thsq_sr_ack_packto_radio);
                n = write(thsq_serialfd, &thsq_sr_ack_packto_radio, HEX_SEVEN);
                usleep(DEC_10);  
                if(thsq_radioparam[NINE] == thsq_packfromradio[FOUR]) 
                {
                    thsq_radiopack_sendcount  = LOOP_TERMINATE;
                    thsq_radioparam_setstatus = SUCCESS_STATUS;     
                }
                else
                {
                    thsq_radiopack_sendcount++;
                    usleep(MILISEC * DEC_10);  // taskDelay(10)                    
                }
            }
            else
            {
                thsq_radiopack_sendcount++;
                usleep(MILISEC * DEC_10);  // taskDelay(10)                
            }            
        }
        else
        { 
            thsq_radiopack_sendcount++;
            usleep(MILISEC * DEC_10);  // taskDelay(10)            
        }
    } 
    return (thsq_radioparam_setstatus);
}



/** FHdr-beg *****************************************************
**
** Function name: activate_setup
**
** Anchor:        RCS_SOW_REQ003
**
** Purpose:       This function perform activate raio set up 
**
**
** Inputs:        as radio id  ,as serial fd, radio parameter   
**
**
** Outputs:      radio  status  
**
** FPrj-beg *****************************************************
**
** FPrj-end *****************************************************
** FHdr-end *****************************************************/
uint8_t activate_setup(uint8_t as_radio_id, int32_t as_serialfd, uint8_t as_radioparam)
{
    uint8_t as_radioparamload           = HEX_ZERO;
    uint8_t as_packto_radio[NINE]       = "";        
    uint8_t as_expectedpackidset[FOUR]  = "";
    uint8_t as_ack_packfromradio[SEVEN] = "";      
    uint8_t as_radioparam_setstatus     = FAILURE_STATUS;
    uint8_t as_radiopack_sendcount      = HEX_ONE;    
    while(as_radiopack_sendcount <= SENDPACK_MED)
    {
        as_radioparamload = as_radioparam;                
        f_activate_setup(as_radioparamload, as_packto_radio);        
        n = write(as_serialfd, &as_packto_radio, HEX_NINE);
        usleep(DEC_10);  
        as_expectedpackidset[ZERO]   = HEX_ONE;
        as_expectedpackidset[ONE]    = HEX_0X26;
        as_expectedpackidset[TWO]    = HEX_SEVEN;  
        as_expectedpackidset[THREE]  = HEX_FOUR;       
        serial_read(as_serialfd, as_ack_packfromradio, as_expectedpackidset);  
        if(as_expectedpackidset[ZERO] == as_ack_packfromradio[ONE] && as_expectedpackidset[ONE] == as_ack_packfromradio[FOUR]) 
        {
            as_radiopack_sendcount = LOOP_TERMINATE;
            as_radioparam_setstatus = SUCCESS_STATUS;
        }
        else
        {
            as_radiopack_sendcount++;
            usleep(MILISEC * DEC_10);  // taskDelay(10)            
        }        
    }
    return(as_radioparam_setstatus);
    
}


/** FHdr-beg *****************************************************
**
** Function name: Preset_Request
**
** Anchor:        RCS_SOW_REQ003
**
** Purpose:       This function perform Preset_Request 
**
**
** Inputs:        pr radio id  ,pr serial fd, radio parameter   
**
**
** Outputs:      pr set radio  status  
**
** FPrj-beg *****************************************************
**
** FPrj-end *****************************************************
** FHdr-end *****************************************************/
uint8_t preset_request(uint8_t pr_radio_id , int32_t pr_serialfd , uint8_t *pr_radioparam_new)
{
    uint8_t pr_radioparam[DEC_10]       = "";
    uint8_t pr_packto_radio[NINE]       = "";  
    uint8_t pr_radioparamload[TWO]      = "";
    uint8_t pr_packfromradio[DEC_117]   = ""; 
    uint8_t pr_ack_packto_radio[SEVEN]  = ""; 
    uint8_t pr_ack_packfromradio[SEVEN] = ""; 
    uint8_t pr_expectedpackidset[FOUR]  = "";   
    uint8_t pr_radioparam_setstatus     = FAILURE_STATUS;
    uint8_t pr_loop                     = HEX_ZERO;
    uint8_t pr_radiopack_sendcount      = HEX_ONE;   
    for(pr_loop =HEX_ZERO ; pr_loop < DEC_10; pr_loop++)
    {
        pr_radioparam[pr_loop] = pr_radioparam_new[pr_loop];
    }
    pr_radiopack_sendcount = HEX_ONE;     
    while(pr_radiopack_sendcount <= SENDPACK_VHIGH)
    {        
        bzero(pr_radioparamload , sizeof(pr_radioparamload));
        pr_radioparamload[ONE] = pr_radioparam[ZERO];        
        f_preset_request(pr_radioparamload[ONE], pr_packto_radio);
        n = write(pr_serialfd, &pr_packto_radio, HEX_NINE);
        usleep(DEC_10); 
        pr_expectedpackidset[ZERO]   = HEX_ONE;
        pr_expectedpackidset[ONE]    = HEX_0X2B;
        pr_expectedpackidset[TWO]    = HEX_SEVEN; 
        pr_expectedpackidset[THREE]  = HEX_FOUR;        
        serial_read(pr_serialfd, pr_ack_packfromradio, pr_expectedpackidset);
        if(pr_expectedpackidset[ZERO] == pr_ack_packfromradio[ONE] && pr_expectedpackidset[ONE] == pr_ack_packfromradio[FOUR]) 
        {  
            pr_expectedpackidset[ZERO]  = HEX_0XA6;
            pr_expectedpackidset[ONE]   = HEX_ZERO;
            pr_expectedpackidset[TWO]   = HEX_0x75; 
            pr_expectedpackidset[THREE] = HEX_A;            
            serial_read(pr_serialfd, pr_packfromradio, pr_expectedpackidset);             
            if(pr_expectedpackidset[ZERO] == pr_packfromradio[ONE] ) 
            {
               // printf("Controller Sending < Preset_Request_Reply_Ack> Packet To < Radio%d>\n\r", pr_radio_id);
                bzero(pr_radioparamload , sizeof(pr_radioparamload));
                pr_radioparamload[ONE] = pr_packfromradio[ONE];                
                f_ackpack_toradio(pr_radioparamload[ONE], pr_ack_packto_radio);
                n = write(pr_serialfd, &pr_ack_packto_radio, HEX_SEVEN);
                usleep(DEC_10);            
                if(pr_packfromradio[FOUR] == HEX_ZERO && pr_packfromradio[FIVE] == HEX_ONE && pr_packfromradio[SIX] == pr_radioparam[ZERO]) 
                {
                    if(pr_radioparam[HEX_ZERO] == HEX_THREE)
                    {
                        if(pr_packfromradio[DEC_12] == pr_radioparam[ONE] && pr_packfromradio[DEC_13] == pr_radioparam[TWO] && 
                           pr_packfromradio[DEC_14] == pr_radioparam[THREE] && pr_packfromradio[DEC_15] == pr_radioparam[FOUR] && 
                           pr_packfromradio[DEC_16] == pr_radioparam[ONE] && pr_packfromradio[DEC_17] == pr_radioparam[TWO] && 
                           pr_packfromradio[DEC_18] == pr_radioparam[THREE] && pr_packfromradio[DEC_19] == pr_radioparam[FOUR] && 
                           pr_packfromradio[DEC_24] == pr_radioparam[FIVE])
                        {
                            pr_radioparam_setstatus = SUCCESS_STATUS;
                        }
                    }
                    else if(pr_radioparam[HEX_ZERO] == HEX_SEVEN)
                    {
                        if(pr_packfromradio[DEC_37] == pr_radioparam[ONE] && pr_packfromradio[DEC_38] == pr_radioparam[TWO] && 
                           pr_packfromradio[DEC_39] == pr_radioparam[THREE] && pr_packfromradio[DEC_40] == pr_radioparam[FOUR] && 
                           pr_packfromradio[DEC_24] == pr_radioparam[FIVE])
                        {                            
                            pr_radioparam_setstatus = SUCCESS_STATUS;
                        }
                    }
                    else if(pr_radioparam[HEX_ZERO] == DEC_10)
                    {
                        if(pr_packfromradio[DEC_100] == HEX_THREE)
                        {
                            pr_radioparam_setstatus = SUCCESS_STATUS;
                        }
                    }
                    pr_radiopack_sendcount = LOOP_TERMINATE;
                }
                else
                {
                    pr_radiopack_sendcount++;
                    usleep(MILISEC * DEC_10);  // taskDelay(10)                    
                }
            }
            else
            { 
                pr_radiopack_sendcount++;
                usleep(MILISEC * DEC_10);  // taskDelay(10)                
            }
        }
        else
        {
            pr_radiopack_sendcount++;
            usleep(MILISEC * DEC_10);  // taskDelay(10)            
        }
    }
    return (pr_radioparam_setstatus);  
}


/** FHdr-beg *****************************************************
**
** Function name: hq_erase
**
** Anchor:       RCS_SOW_REQ003 
**
** Purpose:       This function perform hq_erase 
**
**
** Inputs:        hq radio id  ,hq serial fd 
**
**
** Outputs:      hq  radio  status  
**
** FPrj-beg *****************************************************
**
** FPrj-end *****************************************************
** FHdr-end *****************************************************/

uint8_t hq_erase(uint8_t hqe_radio_id, int32_t hqe_serialfd)
{
    uint8_t hqe_packto_radio[EIGHT]      = "";  // changed from 9 to 8
    uint8_t hqe_radioparamload[THREE]    = "";
    uint8_t hqe_ack_packfromradio[EIGHT] = "";
    uint8_t hqe_expectedpackidset[FOUR]  = "";
    uint8_t hqe_radioparam_setstatus     = FAILURE_STATUS;
    uint8_t hqe_radiopack_sendcount      = HEX_ONE;    
    while(hqe_radiopack_sendcount <= SENDPACK_MED)
    {        
        bzero(hqe_radioparamload, sizeof(hqe_radioparamload));
        //hqe_radioparamload[0] = hqe_radio_id;
        hqe_radioparamload[ONE] = HEX_ONE;
        hqe_radioparamload[TWO] = HEX_ZERO;        
        f_hq_command(hqe_radioparamload, hqe_packto_radio);
        n = write(hqe_serialfd, &hqe_packto_radio, HEX_EIGHT);
        usleep(DEC_10);  
        hqe_expectedpackidset[ZERO]   = HEX_ONE;
        hqe_expectedpackidset[ONE]    = HEX_0x61;
        hqe_expectedpackidset[TWO]    = HEX_SEVEN;  // changed from 8 to 7
        hqe_expectedpackidset[THREE]  = HEX_FOUR;        
        serial_read(hqe_serialfd, hqe_ack_packfromradio, hqe_expectedpackidset);
        if(hqe_expectedpackidset[ZERO] == hqe_ack_packfromradio[ONE] && hqe_expectedpackidset[ONE] == hqe_ack_packfromradio[FOUR]) // changed from 2-1, 5-4
        {          
            hqe_radiopack_sendcount = LOOP_TERMINATE;
            hqe_radioparam_setstatus = SUCCESS_STATUS;
        }
        else
        {
            usleep(MILISEC * DEC_10); // taskDelay(10);
        }    
    }
    return (hqe_radioparam_setstatus);       
}


/** FHdr-beg *****************************************************
**
** Function name: rpw_erase
**
** Anchor:        RCS_SOW_REQ003
**
** Purpose:       This function perform rpw_erase 
**
**
** Inputs:        rpwe radio id  ,rpwe serial fd 
**
**
** Outputs:      rpwe  radio  status  
**
** FPrj-beg *****************************************************
**
** FPrj-end *****************************************************
** FHdr-end *****************************************************/

uint8_t rpw_erase(uint8_t rpwe_radio_id, int32_t rpwe_serialfd)
{
    uint8_t rpwe_packto_radio[DEC_10]     = "";   
    uint8_t rpwe_ack_packfromradio[EIGHT] = "";
    uint8_t rpwe_expectedpackidset[FOUR]  = "";    
    uint8_t rpwe_radioparam_setstatus     = FAILURE_STATUS;
    uint8_t rpwe_radiopack_sendcount      = HEX_ONE;    
    while(rpwe_radiopack_sendcount <= SENDPACK_MED)
    {        
        f_rpw_transec_erase(rpwe_radio_id, rpwe_packto_radio); 
        n = write(rpwe_serialfd, &rpwe_packto_radio, DEC_10);
        usleep(DEC_10); 
        rpwe_expectedpackidset[ZERO]  = HEX_ONE;
        rpwe_expectedpackidset[ONE]   = HEX_0x6D;
        rpwe_expectedpackidset[TWO]   = HEX_SEVEN; 
        rpwe_expectedpackidset[THREE] = HEX_FOUR;       
        serial_read(rpwe_serialfd, rpwe_ack_packfromradio, rpwe_expectedpackidset);
        if(rpwe_expectedpackidset[ZERO] == rpwe_ack_packfromradio[ONE] && rpwe_expectedpackidset[ONE] == rpwe_ack_packfromradio[FOUR]) 
        {
            rpwe_radiopack_sendcount  = LOOP_TERMINATE;
            rpwe_radioparam_setstatus = SUCCESS_STATUS;
        }
        else
        {
            usleep(MILISEC * DEC_10); // taskDelay(10);
        }    
    }    
    return(rpwe_radioparam_setstatus);        
}


/** FHdr-beg *****************************************************
**
** Function name: radio_tod_load
**
** Anchor:       RCS_SOW_REQ007 
**
** Purpose:       This function perform radio tod load 
**
**
** Inputs:        tod radio id  ,tod serial fd 
**
**
** Outputs:      tod  radio set   status  
**
** FPrj-beg *****************************************************
**
** FPrj-end *****************************************************
** FHdr-end *****************************************************/

uint8_t radio_tod_load(int32_t todl_serialfd, uint8_t todl_radio_id)
{
    uint8_t todl_pps_packto_radio[DEC_16]  = "";
    uint8_t todl_gpsr_packto_radio[EIGHT]  = "";
    uint8_t todl_ack_packfromradio[EIGHT]  = "";
    uint8_t todl_expectedpackidset[FOUR]   = "";
    uint8_t todl_radiopack_sendcount1      = HEX_ONE;
    uint8_t todl_radiopack_sendcount2      = HEX_ONE;
    uint8_t todl_radiotod_setstatus        = FAILURE_STATUS;
    while(todl_radiopack_sendcount1 <= SENDPACK_MED)
    {

        f_pps_disable(todl_pps_packto_radio);  //packets to disable pps      
        n = write(todl_serialfd, &todl_pps_packto_radio, DEC_16);
        usleep(DEC_10);        
        todl_expectedpackidset[ZERO]    = HEX_ONE;    
        todl_expectedpackidset[ONE]     = HEX_0xAE;     
        todl_expectedpackidset[TWO]     = HEX_SEVEN;     
		todl_expectedpackidset[THREE]   = HEX_FOUR;         
        serial_read(todl_serialfd, todl_ack_packfromradio, todl_expectedpackidset);
        if((todl_expectedpackidset[ZERO] == todl_ack_packfromradio[ONE]) && (todl_expectedpackidset[ONE] == todl_ack_packfromradio[FOUR]))
        {
			todl_radiopack_sendcount1 = LOOP_TERMINATE;
            todl_radiopack_sendcount2 = HEX_ONE;
            while(todl_radiopack_sendcount2 <= SENDPACK_MED)
            {
                f_gpsrecv_enable(todl_gpsr_packto_radio);  //function to enable gps receive
                n = write(todl_serialfd, &todl_gpsr_packto_radio, HEX_EIGHT);
                usleep(DEC_10);                
                todl_expectedpackidset[ZERO] = HEX_ONE;
                todl_expectedpackidset[ONE]  = HEX_0x61;
                todl_expectedpackidset[TWO]  = HEX_SEVEN;
                todl_expectedpackidset[THREE]= HEX_FOUR; 
                bzero(todl_ack_packfromradio, sizeof(todl_ack_packfromradio));                 
                serial_read(todl_serialfd, todl_ack_packfromradio, todl_expectedpackidset); 
                if((todl_expectedpackidset[ZERO] == todl_ack_packfromradio[ONE]) && (todl_expectedpackidset[ONE] == todl_ack_packfromradio[FOUR]))
                {
                    todl_radiopack_sendcount2 = LOOP_TERMINATE;
                    todl_radiotod_setstatus   = SUCCESS_STATUS;
                }
                else
                {
                    todl_radiopack_sendcount2++;
                    usleep(MILISEC * DEC_10); //nanosleep(&delay, NULL);
                }        
            } 
        } 
        else
        { 
            todl_radiopack_sendcount1++;            
            usleep(MILISEC * DEC_10);  //taskdelay(10) in original code
        }
    } 
    return(todl_radiotod_setstatus); 
}


/** FHdr-beg *****************************************************
**
** Function name: radio_tod_status
**
** Anchor:       RCS_SOW_REQ007 
**
** Purpose:       This function perform radio tod status check 
**
**
** Inputs:        tod radio id  ,tod serial fd 
**
**
** Outputs:      tod  radio set   status  
**
** FPrj-beg *****************************************************
**
** FPrj-end *****************************************************
** FHdr-end *****************************************************/ 
uint8_t radio_tod_status(int32_t tods_serialfd, uint8_t tods_radio_id)
{
    uint32_t radio_combyear         = HEX_ZERO;
	uint32_t radio_combday          = HEX_ZERO;
	uint32_t radio_combhour         = HEX_ZERO;   
    struct   timeval tv;
    struct   tm *time_buffer;    
    uint8_t tods_radioparamload[TWO]    = "";
    uint8_t tods_sr_packto_radio[SEVEN] = "";
    uint8_t tods_packfromradio[DEC_17]  = "";
    uint8_t tods_ack_packto_radio[SEVEN]= "";
    uint8_t tods_te_packto_radio[DEC_10]  = "";
    uint8_t tods_ack_packfromradio[EIGHT] = "";
    uint8_t tods_expectedpackidset[FOUR]  = "";
    uint8_t tods_radiopack_sendcount1 = HEX_ONE;
    uint8_t tods_radiopack_sendcount2 = HEX_ONE;
    uint8_t tods_radiotod_setstatus   = FAILURE_STATUS;
    
    while(tods_radiopack_sendcount1 <= SENDPACK_MED)
    {
        tods_radioparamload[ZERO] = DEC_12;
        f_status_request(tods_radioparamload[ZERO], tods_sr_packto_radio);
        n = write(tods_serialfd, &tods_sr_packto_radio, HEX_SEVEN);
        usleep(DEC_10);      
        tods_expectedpackidset[ZERO]   = HEX_ONE;    
        tods_expectedpackidset[ONE]    = HEX_0x22;
        tods_expectedpackidset[TWO]    = HEX_SEVEN;
        tods_expectedpackidset[THREE]  = HEX_FOUR;
        bzero(tods_ack_packfromradio, sizeof(tods_ack_packfromradio));
        serial_read(tods_serialfd, tods_ack_packfromradio, tods_expectedpackidset);                   
        if(tods_expectedpackidset[ZERO] == tods_ack_packfromradio[ONE] && tods_expectedpackidset[ONE] == tods_ack_packfromradio[FOUR])
        {
            tods_expectedpackidset[ZERO]   = HEX_0xAE;
            tods_expectedpackidset[ONE]    = HEX_ZERO;
            tods_expectedpackidset[TWO]    = HEX_TEN1;
            tods_expectedpackidset[THREE]  = HEX_C;
            bzero(tods_packfromradio, sizeof(tods_packfromradio));             
            serial_read(tods_serialfd, tods_packfromradio, tods_expectedpackidset);                        
            if(tods_expectedpackidset[ZERO] == tods_packfromradio[ONE])            
            {               
                tods_radiopack_sendcount1 = LOOP_TERMINATE;               
                gettimeofday(&tv, NULL);
                time_buffer = localtime(&tv.tv_sec);
                f_ackpack_toradio(tods_packfromradio[ONE], tods_ack_packto_radio);
                n = write(tods_serialfd, &tods_ack_packto_radio, HEX_SEVEN);
                usleep(DEC_10);                 
                radio_combyear = (tods_packfromradio[HEX_FIVE] << HEX_EIGHT);              
                radio_combyear = radio_combyear | tods_packfromradio[HEX_FOUR]; 
                radio_combday  = (tods_packfromradio[HEX_SEVEN] << HEX_EIGHT);              
                radio_combday  = radio_combday | tods_packfromradio[SIX];
                radio_combhour = tods_packfromradio[EIGHT];                   
                if(radio_combyear == (time_buffer->tm_year + GBDATE_1900) && radio_combday == (time_buffer->tm_yday) && radio_combhour == (time_buffer->tm_hour))
                {
                    tods_radiotod_setstatus = SUCCESS_STATUS;
                }
                else
                {
                    ;
                }
                if(tods_radiotod_setstatus == SUCCESS_STATUS)
                {
                    tods_radiopack_sendcount2 = HEX_ONE; 
                    while(tods_radiopack_sendcount2 <= SENDPACK_MED)
                    {
                        f_timeequalization(tods_te_packto_radio);
                        n = write(tods_serialfd, &tods_te_packto_radio, DEC_10);
                        usleep(DEC_10);                       
                        tods_expectedpackidset[ZERO]  = HEX_ONE;
                        tods_expectedpackidset[ONE]   = HEX_0x6D;
                        tods_expectedpackidset[TWO]   = HEX_SEVEN;
                        tods_expectedpackidset[THREE] = HEX_FIVE; 
                        bzero(tods_ack_packfromradio, sizeof(tods_ack_packfromradio)); 
                        serial_read(tods_serialfd, tods_ack_packfromradio, tods_expectedpackidset);                          
                        if(tods_expectedpackidset[ZERO] == tods_ack_packfromradio[ONE] && tods_expectedpackidset[ONE] == tods_ack_packfromradio[FOUR])
                        {
                            tods_radiopack_sendcount2 = LOOP_TERMINATE;                                                       
                            usleep(MILISEC * DEC_180); //taskdelay(180) in original code
                        } 
                        else
                        {
                            tods_radiopack_sendcount2++;                            
                            usleep(MILISEC * DEC_10);   //taskdelay(10) in original code
                        } 
                    }  
                } 
                else
                {
                   
                }   
            } 
            else
            {               
                tods_radiopack_sendcount1++;                                
                usleep(MILISEC * DEC_10);    //taskdelay(10) in original code            
            }           
        } 
        else
        {
            tods_radiopack_sendcount1++;            
            usleep(MILISEC * DEC_10);  //taskdelay(10) in original code
        }        
    } 
    return (tods_radiotod_setstatus);
}


/** FHdr-beg *****************************************************
**
** Function name: hq_param_load
**
** Anchor:        RCS_SOW_REQ003
**
** Purpose:       This function perform hq param load 
**
**
** Inputs:        hql radio id  ,hql serial fd 
**
**
** Outputs:      hql  radio set   status  
**
** FPrj-beg *****************************************************
**
** FPrj-end *****************************************************
** FHdr-end *****************************************************/ 

uint8_t hq_param_load(uint8_t hql_radio_id, int32_t hql_serialfd, uint8_t *hql_radioparam_new)
{
    uint8_t hql_radioparam[DEC_30]       = "";
    uint8_t hql_packto_radio[DEC_32]     = "";   
    uint8_t hql_radioparamload[DEC_31]   = "";
    uint8_t hql_ss_packto_radio[DEC_117] = "";   
    uint8_t hql_ack_packfromradio[SEVEN] = ""; 
    uint8_t hql_expectedpackidset[FOUR]  = "";   
    uint8_t hql_loop 					 = HEX_ZERO;
    uint8_t hql_radiopack_sendcount  	 = HEX_ONE;
    uint8_t hql_radiopack_sendcount1 	 = HEX_ONE;
    uint8_t hql_radiopack_sendcount2 	 = HEX_ONE;
    uint8_t hql_radioparam_setstatus 	 = FAILURE_STATUS;    
    bcopy(hql_radioparam_new, hql_radioparam, sizeof(hql_radioparam));   
    while(hql_radiopack_sendcount <= SENDPACK_MED)
    {
        hql_radiopack_sendcount1 = HEX_ONE;
        while(hql_radiopack_sendcount1 <= SENDPACK_LOW)
        {
            bzero(hql_radioparamload, sizeof(hql_radioparamload));
            hql_radioparamload[HEX_ZERO] = hql_radio_id;            
            for(hql_loop = HEX_ZERO; hql_loop < DEC_30; hql_loop++)
            {
                hql_radioparamload[hql_loop + HEX_ONE] = hql_radioparam[hql_loop];
            }            
            f_hq_load_ss(hql_radioparamload, hql_ss_packto_radio, hq_fha_guardcontrol[hql_radio_id]);            
            n = write(hql_serialfd, &hql_ss_packto_radio, DEC_117);
            usleep(DEC_10); 
            hql_expectedpackidset[ZERO]   = HEX_ONE;
            hql_expectedpackidset[ONE]    = HEX_0X25;
            hql_expectedpackidset[TWO]    = HEX_SEVEN;  
            hql_expectedpackidset[THREE]  = HEX_FOUR;            
            bzero(hql_ack_packfromradio, sizeof(hql_ack_packfromradio)); 
            serial_read(hql_serialfd, hql_ack_packfromradio, hql_expectedpackidset);
            if(hql_expectedpackidset[ZERO] == hql_ack_packfromradio[ONE] && hql_expectedpackidset[ONE] == hql_ack_packfromradio[FOUR]) 
            {
                hql_radiopack_sendcount1 = LOOP_TERMINATE;
                hql_radiopack_sendcount2 = HEX_ONE;                
                while(hql_radiopack_sendcount2 <= SENDPACK_LOW)
                {
                    
                    f_hq_load_mwod(hql_radioparamload, hql_packto_radio);                  
                    hql_expectedpackidset[ZERO]   = HEX_ONE;
                    hql_expectedpackidset[ONE]    = HEX_0x63;
                    hql_expectedpackidset[TWO]    = HEX_SEVEN;  
                    hql_expectedpackidset[THREE]  = HEX_FOUR;                    
                    bzero(hql_ack_packfromradio, sizeof(hql_ack_packfromradio));                     
                    serial_read(hql_serialfd, hql_ack_packfromradio, hql_expectedpackidset);
                    if(hql_expectedpackidset[ZERO] == hql_ack_packfromradio[ONE] && hql_expectedpackidset[ONE] == hql_ack_packfromradio[FOUR]) 
                    {
                        hql_radiopack_sendcount2 = LOOP_TERMINATE;
                        hql_radiopack_sendcount  = LOOP_TERMINATE;
                        hql_radioparam_setstatus = SUCCESS_STATUS;
                    }
                    else
                    {
                        if(hql_radiopack_sendcount2 == SENDPACK_LOW)
                        {
                            hql_radiopack_sendcount++;
                        }
                        hql_radiopack_sendcount2++;
                        usleep(MILISEC * DEC_10); //taskDelay(10);
                    }
                }
            }
            else
            {
                if(hql_radiopack_sendcount1 == SENDPACK_LOW)
                {
                    hql_radiopack_sendcount++;
                }
                hql_radiopack_sendcount1++;
                usleep(MILISEC * DEC_10); //taskDelay(10);
            }
        }
    }
    return (hql_radioparam_setstatus);
}


/** FHdr-beg *****************************************************
**
** Function name: rpw_param_load
**
** Anchor:        RCS_SOW_REQ003
**
** Purpose:       This function perform rpw param load 
**
**
** Inputs:        rpw radio id  ,rpw serial fd 
**
**
** Outputs:      rpw  radio set   status  
**
** FPrj-beg *****************************************************
**
** FPrj-end *****************************************************
** FHdr-end *****************************************************/ 

uint8_t rpw_param_load(uint8_t rpw_radio_id, int32_t rpw_serialfd, uint8_t *rpw_radioparam_new)
{
    uint8_t rpw_radioparam[DEC_70]         = "";
    uint8_t rpw_radioparamload[DEC_70]     = "";
    uint8_t rpw_as_packto_radio[HEX_NINE]  = "";     
    uint8_t rpw_kf_packto_radio[DEC_74]    = "";    
    uint8_t rpw_ss_packto_radio[DEC_117]   = "";   
    uint8_t rpw_init_packto_radio[EIGHT]   = "";
    uint8_t rpw_ack_packfromradio[SEVEN]   = "";  
    uint8_t rpw_expectedpackidset[FOUR]    = "";    
    uint8_t rpw_loop 					   = HEX_ZERO;
    uint8_t rpw_radiopack_sendcount  	   = HEX_ONE;
    uint8_t rpw_radiopack_sendcount1 	   = HEX_ONE;
    uint8_t rpw_radiopack_sendcount2 	   = HEX_ONE;
    uint8_t rpw_radiopack_sendcount3	   = HEX_ONE;
    uint8_t rpw_radiopack_sendcount4 	   = HEX_ONE;
    uint8_t rpw_radioparam_setstatus 	   = FAILURE_STATUS;    
    bcopy(rpw_radioparam_new, rpw_radioparam, sizeof(rpw_radioparam)); 
    while(rpw_radiopack_sendcount <= SENDPACK_LOW)
    {
        rpw_radiopack_sendcount1 = HEX_ONE;
        while(rpw_radiopack_sendcount1 <= SENDPACK_LOW)
        {
            
            bzero(rpw_radioparamload, sizeof(rpw_radioparamload));
            //rpw_radioparamload[0] = rpw_radio_id;
            rpw_radioparamload[HEX_ONE] = HEX_THREE;            
            f_activate_setup(rpw_radioparamload[HEX_ONE], rpw_as_packto_radio);
            n = write(rpw_serialfd, &rpw_as_packto_radio, HEX_NINE);
            usleep(DEC_10); 
            rpw_expectedpackidset[ZERO]  = HEX_ONE;
            rpw_expectedpackidset[ONE]   = HEX_0X26;
            rpw_expectedpackidset[TWO]   = HEX_SEVEN;  
            rpw_expectedpackidset[THREE] = HEX_FOUR;            
            bzero(rpw_ack_packfromradio, sizeof(rpw_ack_packfromradio)); 
            serial_read(rpw_serialfd, rpw_ack_packfromradio, rpw_expectedpackidset);
            if(rpw_expectedpackidset[ZERO] == rpw_ack_packfromradio[ONE] && rpw_expectedpackidset[ONE] == rpw_ack_packfromradio[FOUR]) 
            {
                rpw_radiopack_sendcount1 = LOOP_TERMINATE;
                rpw_radiopack_sendcount2 = HEX_ONE;
                while(rpw_radiopack_sendcount2 <= SENDPACK_LOW)
                {
                    f_storesetup_rpwman(rpw_radio_id, rpw_ss_packto_radio);                    
                    n = write(rpw_serialfd, &rpw_ss_packto_radio, DEC_117);
                    usleep(DEC_10);                   
                    rpw_expectedpackidset[ZERO]  = HEX_ONE;
                    rpw_expectedpackidset[ONE]   = HEX_0X25;
                    rpw_expectedpackidset[TWO]   = HEX_SEVEN;  
                    rpw_expectedpackidset[THREE] = HEX_FOUR;                   
                    bzero(rpw_ack_packfromradio, sizeof(rpw_ack_packfromradio)); 
                    serial_read(rpw_serialfd, rpw_ack_packfromradio, rpw_expectedpackidset);                    
                    if(rpw_expectedpackidset[ZERO] == rpw_ack_packfromradio[ONE] && rpw_expectedpackidset[ONE] == rpw_ack_packfromradio[FOUR])
                    {
                        rpw_radiopack_sendcount2 = LOOP_TERMINATE;
                        rpw_radiopack_sendcount3 = HEX_ONE;                        
                        while(rpw_radiopack_sendcount3 <= SENDPACK_LOW)
                        {
                            bzero(rpw_radioparamload, sizeof(rpw_radioparamload));
                            //rpw_radioparamload[0] = radio_id;
                            rpw_radioparamload[ONE] = HEX_B;
                            f_initialisation(rpw_radioparamload[ONE], rpw_init_packto_radio);                        
                            n = write(rpw_serialfd, &rpw_init_packto_radio, HEX_EIGHT);
                            usleep(DEC_10); 
                            rpw_expectedpackidset[ZERO]  = HEX_ONE;
                            rpw_expectedpackidset[ONE]   = HEX_ZERO;
                            rpw_expectedpackidset[TWO]   = HEX_SEVEN; 
                            rpw_expectedpackidset[THREE] = HEX_FOUR;                             
                            bzero(rpw_ack_packfromradio, sizeof(rpw_ack_packfromradio)); 
                            serial_read(rpw_serialfd, rpw_ack_packfromradio, rpw_expectedpackidset);                            
                            if(rpw_expectedpackidset[ZERO] == rpw_ack_packfromradio[ONE] && rpw_expectedpackidset[ONE] == rpw_ack_packfromradio[FOUR])
                            {
                                rpw_radiopack_sendcount3 = LOOP_TERMINATE;
                                rpw_radiopack_sendcount4 = HEX_ONE; 
                                while(rpw_radiopack_sendcount4 <= SENDPACK_LOW)
                                {      
                                    bzero(rpw_radioparamload, sizeof(rpw_radioparamload));
                                    //rpw_radioparamload[0] = rpw_radio_id;                                    
                                    for(rpw_loop = HEX_ZERO; rpw_loop < DEC_69; rpw_loop++)
                                    {
                                        rpw_radioparamload[rpw_loop + HEX_ONE] = rpw_radioparam[rpw_loop];
                                    }                                    
                                    f_rpw_load_kfd(rpw_radioparamload, rpw_kf_packto_radio);                                    
                                    n = write(rpw_serialfd, &rpw_kf_packto_radio, DEC_74);
                                    usleep(DEC_10);                                  
                                    rpw_expectedpackidset[ZERO]  = HEX_ONE;
                                    rpw_expectedpackidset[ONE]   = HEX_0x6F;
                                    rpw_expectedpackidset[TWO]   = HEX_SEVEN;  
                                    rpw_expectedpackidset[THREE] = HEX_FOUR;                                    
                                    bzero(rpw_ack_packfromradio, sizeof(rpw_ack_packfromradio)); 
                                    serial_read(rpw_serialfd, rpw_ack_packfromradio, rpw_expectedpackidset);                                    
                                    if(rpw_expectedpackidset[ZERO] == rpw_ack_packfromradio[ONE] && rpw_expectedpackidset[ONE] == rpw_ack_packfromradio[FOUR])
                                    {
                                        rpw_radiopack_sendcount4 = LOOP_TERMINATE;
                                        rpw_radiopack_sendcount  = LOOP_TERMINATE; 
                                        rpw_radioparam_setstatus = SUCCESS_STATUS;
                                    }                                              
                                    else
                                    {                                       
                                        if(rpw_radiopack_sendcount4 == SENDPACK_LOW)
                                        {
                                            rpw_radiopack_sendcount++;
                                        }
                                        rpw_radiopack_sendcount4++;
                                        usleep(MILISEC * DEC_10); //taskDelay(10);
                                    }                         
                                } 
                            }        
                            else
                            {
                                if(rpw_radiopack_sendcount3 == SENDPACK_LOW)
                                {
                                    rpw_radiopack_sendcount++;
                                }
                                rpw_radiopack_sendcount3++;
                                usleep(MILISEC * DEC_10); //taskDelay(10);
                            }                                         
                        }                    
                    }
                    else
                    {
                        if(rpw_radiopack_sendcount2 == SENDPACK_LOW)
                        {
                            rpw_radiopack_sendcount++;
                        }
                        rpw_radiopack_sendcount2++;
                        usleep(MILISEC * DEC_10); //taskDelay(10);
                    }
                }            
            }
            else
            {
                if(rpw_radiopack_sendcount1 == SENDPACK_LOW)
                {
                    rpw_radiopack_sendcount++;
                }
                rpw_radiopack_sendcount1++;
                usleep(MILISEC * DEC_10); //taskDelay(10);
            }
        }
    }    
    return(rpw_radioparam_setstatus);     
}



/** FHdr-beg *****************************************************
**
** Function name: rpw_param_load_reinit
**
** Anchor:       RCS_SOW_REQ003 
**
** Purpose:       This function perform rpw param load  re initialize 
**
**
** Inputs:        rpw radio id  ,rpw serial fd 
**
**
** Outputs:      rpw  radio set   status  
**
** FPrj-beg *****************************************************
**
** FPrj-end *****************************************************
** FHdr-end *****************************************************/ 

uint8_t rpw_param_load_reinit(uint8_t rpwrl_radio_id, int32_t rpwrl_serialfd)
{
    uint8_t rpwrl_radioparamload[TWO]   	 = "";
    uint8_t rpwrl_ack_packto_radio[SEVEN]    = "";   
    uint8_t rpwrl_str_packto_radio[NINE]  	 = "";   
    uint8_t rpwrl_init_packto_radio[EIGHT]   = "";
    uint8_t rpwrl_st_packfromradio[DEC_11] 	 = "";  
    uint8_t rpwrl_ack_packfromradio[NINE] 	 = "";
    uint8_t rpwrl_expectedpackidset[FOUR] 	 = "";    
    uint8_t rpwrl_radiopack_sendcount  		 = HEX_ONE;
    uint8_t rpwrl_radiopack_sendcount1 		 = HEX_ONE;
    uint8_t rpwrl_radiopack_sendcount2 		 = HEX_ONE;
    uint8_t rpwrl_radioparam_setstatus 		 = FAILURE_STATUS;      
    while(rpwrl_radiopack_sendcount <= SENDPACK_LOW)
    {
        rpwrl_radiopack_sendcount1 = HEX_ONE;
        while(rpwrl_radiopack_sendcount1 <= SENDPACK_LOW)
        {
           
            bzero(rpwrl_radioparamload ,sizeof(rpwrl_radioparamload));
            //rpwrl_radioparamload[0] = rpwrl_radio_id;
            rpwrl_radioparamload[ONE] = HEX_SEVEN;            
            f_initialisation(rpwrl_radioparamload[ONE], rpwrl_init_packto_radio);            
            n = write(rpwrl_serialfd, &rpwrl_init_packto_radio, HEX_EIGHT);
            usleep(DEC_10);   
            rpwrl_expectedpackidset[ZERO]   = HEX_ONE;
            rpwrl_expectedpackidset[ONE]    = HEX_ZERO;
            rpwrl_expectedpackidset[TWO]    = HEX_SEVEN;   
            rpwrl_expectedpackidset[THREE]  = HEX_FOUR;             
            bzero(rpwrl_ack_packfromradio, sizeof(rpwrl_ack_packfromradio)); 
            serial_read(rpwrl_serialfd, rpwrl_ack_packfromradio, rpwrl_expectedpackidset);            
            if(rpwrl_ack_packfromradio[ONE] == rpwrl_expectedpackidset[ZERO] && rpwrl_ack_packfromradio[FOUR] == rpwrl_expectedpackidset[ONE])
            {
				rpwrl_radiopack_sendcount1 = LOOP_TERMINATE;
                rpwrl_radiopack_sendcount2 = HEX_ONE;                
                while(rpwrl_radiopack_sendcount2 <= SENDPACK_MED)
                {
                    
                    f_rpw_stored_transec(rpwrl_radio_id, rpwrl_str_packto_radio);                    
                    n = write(rpwrl_serialfd, &rpwrl_str_packto_radio, HEX_NINE);
                    usleep(DEC_10);             
                    rpwrl_expectedpackidset[ZERO]   = HEX_ONE;
                    rpwrl_expectedpackidset[ONE]    = HEX_0x6E;
                    rpwrl_expectedpackidset[TWO]    = HEX_SEVEN;   
                    rpwrl_expectedpackidset[THREE]  = HEX_FOUR;                    
                    bzero(rpwrl_ack_packfromradio, sizeof(rpwrl_ack_packfromradio)); 
                    serial_read(rpwrl_serialfd, rpwrl_ack_packfromradio, rpwrl_expectedpackidset);                    
                    if(rpwrl_ack_packfromradio[ONE] == rpwrl_expectedpackidset[ZERO] && rpwrl_ack_packfromradio[FOUR] == rpwrl_expectedpackidset[ONE])
                    {
                        rpwrl_expectedpackidset[ZERO]   = HEX_0XEE;
                        rpwrl_expectedpackidset[ONE]    = HEX_ZERO;
                        rpwrl_expectedpackidset[TWO]    = HEX_B;  
                        rpwrl_expectedpackidset[THREE]  = HEX_C;                        
                        bzero(rpwrl_st_packfromradio, sizeof(rpwrl_st_packfromradio)); 
                        serial_read(rpwrl_serialfd, rpwrl_st_packfromradio, rpwrl_expectedpackidset);                    
                        if(rpwrl_st_packfromradio[ONE] == rpwrl_expectedpackidset[ZERO]) 
                        {
                           rpwrl_radiopack_sendcount2 = LOOP_TERMINATE;                   
                           bzero(rpwrl_radioparamload,sizeof(rpwrl_radioparamload));                         
                           rpwrl_radioparamload[ONE] = rpwrl_st_packfromradio[ONE];                           
                           f_ackpack_toradio(rpwrl_radioparamload[ONE], rpwrl_ack_packto_radio);                      
                           n = write(rpwrl_serialfd, &rpwrl_ack_packto_radio, HEX_SEVEN);
                           usleep(DEC_10); 
                           if(rpwrl_ack_packfromradio[SEVEN] == HEX_THREE && rpwrl_ack_packfromradio[EIGHT] == HEX_ONE)  
                           {
                               rpwrl_radiopack_sendcount  = LOOP_TERMINATE;
                               rpwrl_radioparam_setstatus = SUCCESS_STATUS;
                           }
                           else
                           {
                               rpwrl_radiopack_sendcount++;
                               usleep(MILISEC * DEC_60); //taskDelay(60);
                           }
                        }
                        else
                        {                           
                            if(rpwrl_radiopack_sendcount2 == SENDPACK_MED)
                            {
                               rpwrl_radiopack_sendcount++;
                            }
                            rpwrl_radiopack_sendcount2++;
                            usleep(MILISEC * DEC_10); //taskDelay(10);
                        }
                    }
                    else
                    {
                        if(rpwrl_radiopack_sendcount2 == SENDPACK_MED)
                        {
                            rpwrl_radiopack_sendcount++;
                        }
                        rpwrl_radiopack_sendcount2++;
                        usleep(MILISEC * DEC_10); //taskDelay(10);
                    }
                }
            }
            else
            {
                if(rpwrl_radiopack_sendcount1 == SENDPACK_LOW)
                {
                    rpwrl_radiopack_sendcount++;
                }
                rpwrl_radiopack_sendcount1++;
                usleep(MILISEC * DEC_10); //taskDelay(10);
            }
        }    
    }
    return(rpwrl_radioparam_setstatus);    
}


/** FHdr-beg *****************************************************
**
** Function name: hq_mode_operation
**
** Anchor:     RCS_SOW_REQ003   
**
** Purpose:       This function perform hq_mode_operation 
**
**
** Inputs:        hqop radio id  ,hqop serial fd 
**
**
** Outputs:      hq  radio   status  
**
** FPrj-beg *****************************************************
**
** FPrj-end *****************************************************
** FHdr-end *****************************************************/ 


uint8_t hq_mode_operation(uint8_t hqop_radio_id, int32_t hqop_serialfd, uint8_t *hqop_radioparam_new)
{
    uint8_t hqop_radioparam[DEC_30]           = "";
    uint8_t hqop_sr_packto_radio[SEVEN]       = "";  
    uint8_t hqop_radioparamload[DEC_31]       = "";
    uint8_t hqop_al_packto_radio[NINE]        = "";  
    uint8_t hqop_ack_packto_radio[SEVEN]      = ""; 
    uint8_t hqop_ss_packto_radio[DEC_117]     = "";  
    uint8_t hqop_hql_packto_radio[DEC_32]     = "";  
    uint8_t hqop_mwod_packto_radio[EIGHT]     = "";  
    uint8_t hqop_ack_packfromradio[SEVEN]     = ""; 
    uint8_t hqop_expectedpackidset[FOUR]      = "";
    uint8_t hqop_status_packfromradio[DEC_16] = "";    
    uint8_t hqop_loop 						  = HEX_ZERO;
    uint8_t hqop_radiopack_sendcount  		  = HEX_ONE;
    uint8_t hqop_radiopack_sendcount1 		  = HEX_ONE;
    uint8_t hqop_radiopack_sendcount2 		  = HEX_ONE;
    uint8_t hqop_radiopack_sendcount3 		  = HEX_ONE;
    uint8_t hqop_radiopack_sendcount4 		  = HEX_ONE;
    uint8_t hqop_radiopack_sendcount5 		  = HEX_ONE;
    uint8_t hqop_radioparam_setstatus 		  = FAILURE_STATUS;     
    bcopy(hqop_radioparam_new, hqop_radioparam, sizeof(hqop_radioparam));   
    while(hqop_radiopack_sendcount <= SENDPACK_MED)
    {
        hqop_radiopack_sendcount1 = HEX_ONE;
        while(hqop_radiopack_sendcount <= SENDPACK_LOW)
        {
            bzero(hqop_radioparamload,sizeof(hqop_radioparamload));       
            for(hqop_loop = HEX_ZERO; hqop_loop < DEC_30; hqop_loop++)
            {
                hqop_radioparamload[hqop_loop + HEX_ONE] = hqop_radioparam[hqop_loop];
            }            
            f_hq_load_ss(hqop_radioparamload, hqop_ss_packto_radio, hq_fha_guardcontrol[hqop_radio_id]);            
            n = write(hqop_serialfd, &hqop_ss_packto_radio, DEC_117);
            usleep(DEC_10);        
            hqop_expectedpackidset[ZERO]  = HEX_ONE;
            hqop_expectedpackidset[ONE]   = HEX_0X25;
            hqop_expectedpackidset[TWO]   = HEX_SEVEN;  
            hqop_expectedpackidset[THREE] = HEX_FOUR;            
            bzero(hqop_ack_packfromradio, sizeof(hqop_ack_packfromradio)); 
            serial_read(hqop_serialfd, hqop_ack_packfromradio, hqop_expectedpackidset);
            if(hqop_ack_packfromradio[ONE] == hqop_expectedpackidset[ZERO] && hqop_ack_packfromradio[FOUR] == hqop_expectedpackidset[ONE]) 
            {
                hqop_radiopack_sendcount1 = LOOP_TERMINATE;
                hqop_radiopack_sendcount2 = HEX_ONE;                
                while(hqop_radiopack_sendcount2 <= SENDPACK_LOW)
                {
                    bzero(hqop_radioparamload,sizeof(hqop_radioparamload));          
                    for(hqop_loop = HEX_ZERO; hqop_loop < DEC_30; hqop_loop++)
                    {
                        hqop_radioparamload[hqop_loop + HEX_ONE] = hqop_radioparam[hqop_loop];
                    }                    
                    f_hq_load_mwod(hqop_radioparamload, hqop_hql_packto_radio);                    
                    n = write(hqop_serialfd, &hqop_hql_packto_radio, DEC_32);
                    usleep(DEC_10); 
                    hqop_expectedpackidset[ZERO]   = HEX_ONE;
                    hqop_expectedpackidset[ONE]    = HEX_0x63;
                    hqop_expectedpackidset[TWO]    = HEX_SEVEN;  
                    hqop_expectedpackidset[THREE]  = HEX_FOUR;             
                    bzero(hqop_ack_packfromradio, sizeof(hqop_ack_packfromradio)); 
                    serial_read(hqop_serialfd, hqop_ack_packfromradio, hqop_expectedpackidset);                    
                    if(hqop_ack_packfromradio[ONE] == hqop_expectedpackidset[ZERO] && hqop_ack_packfromradio[FOUR] == hqop_expectedpackidset[ONE])
                    {
                        hqop_radiopack_sendcount2 = LOOP_TERMINATE;
                        hqop_radiopack_sendcount3 = HEX_ONE;                        
                        while(hqop_radiopack_sendcount3 <= SENDPACK_LOW)
                        {                           
                            bzero(hqop_radioparamload,sizeof(hqop_radioparamload));                           
                            hqop_radioparamload[ONE] = HEX_SEVEN;                            
                            f_activate_setup(hqop_radioparamload[HEX_ONE], hqop_al_packto_radio);                            
                            n = write(hqop_serialfd, &hqop_al_packto_radio, DEC_32);
                            usleep(DEC_10);                          
                            hqop_expectedpackidset[ZERO]   = HEX_ONE;
                            hqop_expectedpackidset[ONE]    = HEX_0X26;
                            hqop_expectedpackidset[TWO]    = HEX_SEVEN;   
                            hqop_expectedpackidset[THREE]  = HEX_FOUR;                            
                            bzero(hqop_ack_packfromradio, sizeof(hqop_ack_packfromradio)); 
                            serial_read(hqop_serialfd, hqop_ack_packfromradio, hqop_expectedpackidset);                            
                            if(hqop_ack_packfromradio[ONE] == hqop_expectedpackidset[ZERO] && hqop_ack_packfromradio[FOUR] == hqop_expectedpackidset[ONE])
                            {
                                hqop_radiopack_sendcount3 = LOOP_TERMINATE;
                                hqop_radiopack_sendcount4 = HEX_ONE;                                
                                while(hqop_radiopack_sendcount4 <= SENDPACK_LOW)
                                {                                    
                                    bzero(hqop_radioparamload,sizeof(hqop_radioparamload));                                   
                                    hqop_radioparamload[ONE] = HEX_ZERO;
                                    hqop_radioparamload[TWO] = hqop_radioparam[DEC_28];                                    
                                    f_hq_command(hqop_radioparamload, hqop_mwod_packto_radio);                                    
                                    n = write(hqop_serialfd, &hqop_mwod_packto_radio, HEX_EIGHT);
                                    usleep(DEC_10); 
                                    hqop_expectedpackidset[ZERO]  = HEX_ONE;
                                    hqop_expectedpackidset[ONE]   = HEX_0x61;
                                    hqop_expectedpackidset[TWO]   = HEX_SEVEN;  
                                    hqop_expectedpackidset[THREE] = HEX_FOUR;                                    
                                    bzero(hqop_ack_packfromradio, sizeof(hqop_ack_packfromradio)); 
                                    serial_read(hqop_serialfd, hqop_ack_packfromradio, hqop_expectedpackidset);                                    
                                    if(hqop_ack_packfromradio[ONE] == hqop_expectedpackidset[ZERO] && hqop_ack_packfromradio[FOUR] == hqop_expectedpackidset[ONE])
                                    {
                                        hqop_radiopack_sendcount4 = LOOP_TERMINATE;
                                        hqop_radiopack_sendcount5 = HEX_ONE;                                        
                                        while(hqop_radiopack_sendcount5 <= SENDPACK_LOW)
                                        {
                                            bzero(hqop_radioparamload,sizeof(hqop_radioparamload));
                                            hqop_radioparamload[ZERO] = hqop_radio_id;
                                            hqop_radioparamload[ONE]  = HEX_SEVEN;
                                            hqop_radioparamload[TWO]  = HEX_ZERO;                                            
                                            f_hq_command(hqop_radioparamload, hqop_mwod_packto_radio);                                            
                                            n = write(hqop_serialfd, &hqop_mwod_packto_radio, HEX_EIGHT);
                                            usleep(DEC_10);
                                            hqop_expectedpackidset[ZERO]  = HEX_ONE;
                                            hqop_expectedpackidset[ONE]   = HEX_0x61;
                                            hqop_expectedpackidset[TWO]   = HEX_SEVEN;  
                                            hqop_expectedpackidset[THREE] = HEX_FOUR;                                            
                                            bzero(hqop_ack_packfromradio, sizeof(hqop_ack_packfromradio)); 
                                            serial_read(hqop_serialfd, hqop_ack_packfromradio, hqop_expectedpackidset);                                           
                                            if(hqop_ack_packfromradio[ONE]  == hqop_expectedpackidset[ZERO] && 
                                               hqop_ack_packfromradio[FOUR] == hqop_expectedpackidset[ONE])
                                            {
                                                hqop_radiopack_sendcount5 = LOOP_TERMINATE;
                                                hqop_radiopack_sendcount  = LOOP_TERMINATE;
                                            }
                                            else
                                            {
                                                if(hqop_radiopack_sendcount5 == SENDPACK_LOW)
                                                {
                                                    hqop_radiopack_sendcount++;
                                                }
                                                hqop_radiopack_sendcount5++;
                                                usleep(MILISEC * DEC_10); //taskDelay(10);
                                            }                                                                                                                                
                                        }
                                    }
                                    else
                                    {
                                        if(hqop_radiopack_sendcount4 == SENDPACK_LOW)
                                        {
                                            hqop_radiopack_sendcount++;
                                        }
                                        hqop_radiopack_sendcount4++;
                                        usleep(MILISEC * DEC_10); //taskDelay(10);
                                    }                            
                                }
                            }
                            else
                            {
                                if(hqop_radiopack_sendcount3 == SENDPACK_LOW)
                                {
                                    hqop_radiopack_sendcount++;
                                }
                                hqop_radiopack_sendcount3++;
                                usleep(MILISEC * DEC_10); //taskDelay(10);
                            }
                        }
                    }        
                    else
                    {
                        if(hqop_radiopack_sendcount2 == SENDPACK_LOW)
                        {
                            hqop_radiopack_sendcount++;
                        }
                        hqop_radiopack_sendcount2++;
                        usleep(MILISEC * DEC_10); //taskDelay(10);
                    }            
                }
            }        
            else
            {
                if(hqop_radiopack_sendcount1 == SENDPACK_LOW)
                {
                    hqop_radiopack_sendcount++;
                }
                hqop_radiopack_sendcount1++;
                usleep(MILISEC * DEC_10); //taskDelay(10);
            }             
        }
    } // end of while(hqop_radiopack_sendcount <= SENDPACK_MED)
    
    if(hqop_radiopack_sendcount == LOOP_TERMINATE)
    {
        hqop_radiopack_sendcount = HEX_ONE;
        while(hqop_radiopack_sendcount <= SENDPACK_HIGH)
        {
            bzero(hqop_radioparamload,sizeof(hqop_radioparamload));            
            hqop_radioparamload[ONE] = HEX_NINE;            
            f_status_request(hqop_radioparamload[HEX_ONE], hqop_sr_packto_radio);            
            n = write(hqop_serialfd, &hqop_sr_packto_radio, HEX_SEVEN);
            usleep(DEC_10);             
            hqop_expectedpackidset[ZERO]   = HEX_ONE;
            hqop_expectedpackidset[ONE]    = HEX_0x22;
            hqop_expectedpackidset[TWO]    = HEX_SEVEN;  
            hqop_expectedpackidset[THREE]  = HEX_FOUR;            
            bzero(hqop_ack_packfromradio, sizeof(hqop_ack_packfromradio)); 
            serial_read(hqop_serialfd, hqop_ack_packfromradio, hqop_expectedpackidset);            
            if(hqop_ack_packfromradio[ONE] == hqop_expectedpackidset[ZERO] && hqop_ack_packfromradio[FOUR] == hqop_expectedpackidset[ONE])
            {
                hqop_expectedpackidset[ZERO]   = HEX_0XE0;
                hqop_expectedpackidset[ONE]    = HEX_ZERO;
                hqop_expectedpackidset[TWO]    = HEX_TEN1;  
                hqop_expectedpackidset[THREE]  = HEX_C;                
                bzero(hqop_status_packfromradio, sizeof(hqop_status_packfromradio)); 
                serial_read(hqop_serialfd, hqop_status_packfromradio, hqop_expectedpackidset);                
                if(hqop_expectedpackidset[ZERO] == hqop_status_packfromradio[ONE])  
                {
                    bzero(hqop_radioparamload,sizeof(hqop_radioparamload));                   
                    hqop_radioparamload[ONE] = hqop_status_packfromradio[ONE];                   
                    f_ackpack_toradio(hqop_radioparamload[ONE], hqop_ack_packto_radio);                   
                    n = write(hqop_serialfd, &hqop_ack_packto_radio, HEX_SEVEN);
                    usleep(DEC_10);  
                    if(hqop_status_packfromradio[FIVE] == HEX_ONE)  
                    {                       
                        hqop_radiopack_sendcount  = LOOP_TERMINATE;
                        hqop_radioparam_setstatus = SUCCESS_STATUS;                        
                        radio_cbitfirsttime_after_fh[hqop_radio_id] = HEX_ONE;
                    }                    
                    else
                    {
                        hqop_radiopack_sendcount++;
                        usleep(MILISEC * DEC_10); //taskDelay(10);
                    }
                }
                else
                {
                    hqop_radiopack_sendcount++;
                    usleep(MILISEC * DEC_10); //taskDelay(10);
                }
            }    
            else
            {
                hqop_radiopack_sendcount++;
                usleep(MILISEC * DEC_10); //taskDelay(10);
            }        
        }
    }
    else
    {
       
    }
    
    return(hqop_radioparam_setstatus);
}


/** FHdr-beg *****************************************************
**
** Function name: rpw_mode_operation
**
** Anchor:        RCS_SOW_REQ003
**
** Purpose:       This function perform rpw mode operation 
**
**
** Inputs:        hqop radio id  ,hqop serial fd 
**
**
** Outputs:      rpwop  radio parameter set   status  
**
** FPrj-beg *****************************************************
**
** FPrj-end *****************************************************
** FHdr-end *****************************************************/ 

uint8_t rpw_mode_operation(uint8_t rpwop_radio_id, int32_t rpwop_serialfd, uint8_t *rpwop_radioparam_new)
{
    uint8_t rpwop_radioparam[DEC_70]           = "";
    uint8_t rpwop_radioparamload[THREE]        = "";
    uint8_t rpwop_sr_packto_radio[SEVEN]       = "";  
    uint8_t rpwop_al_packto_radio[NINE]        = "";  
    uint8_t rpwop_ack_packto_radio[SEVEN]      = "";  
    uint8_t rpwop_ss_packto_radio[DEC_117]     = "";  
    uint8_t rpwop_expectedpackidset[FOUR]      = "";
    uint8_t rpwop_status_packfromradio[DEC_17] = ""; 
    uint8_t rpwop_ack_packfromradio[HEX_EIGHT] = "";    
    uint8_t rpw_re_operation                   = HEX_ZERO;
    uint8_t rpwop_radiopack_sendcount          = HEX_ONE;
    uint8_t rpwop_radiopack_sendcount1         = HEX_ONE;
    uint8_t rpwop_radiopack_sendcount2         = HEX_ONE;
    uint8_t rpwop_radiopack_sendcount3         = HEX_ONE;
    uint8_t rpwop_radioparam_setstatus         = FAILURE_STATUS;    
    bcopy(rpwop_radioparam_new, rpwop_radioparam, sizeof(rpwop_radioparam));    
    if(rpw_re_operation_complete[rpwop_radio_id] == HEX_ONE)
    {
        rpw_re_operation = HEX_ZERO;
    }
    else
    {
        rpw_re_operation = HEX_ONE;
    }
    
    if(rpw_re_operation == HEX_ONE)
    {
        rpwop_radiopack_sendcount = HEX_ONE;
        while(rpwop_radiopack_sendcount <= SENDPACK_MED)
        {
            rpwop_radiopack_sendcount1 = HEX_ONE;
            while(rpwop_radiopack_sendcount1 <= SENDPACK_LOW)
            {
                bzero(rpwop_radioparamload,sizeof(rpwop_radioparamload));               
                rpwop_radioparamload[ONE] = DEC_10;                
                f_activate_setup(rpwop_radioparamload[HEX_ONE], rpwop_al_packto_radio);
                n = write(rpwop_serialfd, &rpwop_al_packto_radio, HEX_NINE);
                usleep(DEC_10);  
                rpwop_expectedpackidset[ZERO]   = HEX_ONE;
                rpwop_expectedpackidset[ONE]    = HEX_0X26;
                rpwop_expectedpackidset[TWO]    = HEX_SEVEN;  
                rpwop_expectedpackidset[THREE]  = HEX_FOUR;
                bzero(rpwop_ack_packfromradio, sizeof(rpwop_ack_packfromradio)); 
                serial_read(rpwop_serialfd, rpwop_ack_packfromradio, rpwop_expectedpackidset);                
                if(rpwop_ack_packfromradio[ONE] == rpwop_expectedpackidset[ZERO] && rpwop_ack_packfromradio[FOUR] == rpwop_expectedpackidset[ONE]) 
                {
                    rpwop_radiopack_sendcount1 = LOOP_TERMINATE;
                    rpwop_radiopack_sendcount2 = HEX_ONE;
                    while(rpwop_radiopack_sendcount2 <= SENDPACK_LOW)
                    {
                        bzero(rpwop_radioparamload, sizeof(rpwop_radioparamload));
                        rpwop_radioparamload[ZERO] = rpwop_radio_id;
                        rpwop_radioparamload[ONE]  = rpwop_radioparam[ZERO];
                        rpwop_radioparamload[TWO]  = HEX_ZERO;                        
                        f_rpw_ss_fh(rpwop_radioparamload, rpwop_ss_packto_radio);                      
                        n = write(rpwop_serialfd, &rpwop_ss_packto_radio, DEC_117);
                        usleep(DEC_10);                        
                        rpwop_expectedpackidset[ZERO]  = HEX_ONE;
                        rpwop_expectedpackidset[ONE]   = HEX_0X25;
                        rpwop_expectedpackidset[TWO]   = HEX_SEVEN;  
                        rpwop_expectedpackidset[THREE] = HEX_FOUR;                        
                        bzero(rpwop_ack_packfromradio, sizeof(rpwop_ack_packfromradio)); 
                        serial_read(rpwop_serialfd, rpwop_ack_packfromradio, rpwop_expectedpackidset);                        
                        if(rpwop_ack_packfromradio[ONE] == rpwop_expectedpackidset[ZERO] && rpwop_ack_packfromradio[FOUR] == rpwop_expectedpackidset[ONE])
                        {
                            rpwop_radiopack_sendcount2 = LOOP_TERMINATE;
                            rpwop_radiopack_sendcount3 = HEX_ONE;
                            while(rpwop_radiopack_sendcount3 <= SENDPACK_LOW)
                            {
                                bzero(rpwop_radioparamload,sizeof(rpwop_radioparamload));                               
                                rpwop_radioparamload[ONE] = DEC_10;
                                f_activate_setup(rpwop_radioparamload[ONE], rpwop_al_packto_radio);                                
                                n = write(rpwop_serialfd, &rpwop_al_packto_radio, HEX_NINE);
                                usleep(DEC_10);                                 
                                rpwop_expectedpackidset[ZERO]  = HEX_ONE;
                                rpwop_expectedpackidset[ONE]   = HEX_0X26;
                                rpwop_expectedpackidset[TWO]   = HEX_SEVEN;  
                                rpwop_expectedpackidset[THREE] = HEX_FOUR;                               
                                bzero(rpwop_ack_packfromradio, sizeof(rpwop_ack_packfromradio)); 
                                serial_read(rpwop_serialfd, rpwop_ack_packfromradio, rpwop_expectedpackidset);                               
                                if(rpwop_ack_packfromradio[ONE] == rpwop_expectedpackidset[ZERO] && rpwop_ack_packfromradio[FOUR] == rpwop_expectedpackidset[ONE])
                                {
                                    rpwop_radiopack_sendcount3 = LOOP_TERMINATE;
                                    rpwop_radiopack_sendcount  = LOOP_TERMINATE;
                                } 
                                else
                                {
                                    if(rpwop_radiopack_sendcount3 == SENDPACK_LOW)
                                    {
                                        rpwop_radiopack_sendcount++;
                                    }
                                    rpwop_radiopack_sendcount3++;
                                    usleep(MILISEC * DEC_10); //taskDelay(10);
                                }
                            }                           
                        }
                        else
                        {
                            if(rpwop_radiopack_sendcount2 == SENDPACK_LOW)
                            {
                                rpwop_radiopack_sendcount++;
                            }
                            rpwop_radiopack_sendcount2++;
                            usleep(MILISEC * DEC_10); //taskDelay(10);
                        }
                    }
                }                            
                else
                { 
                    if(rpwop_radiopack_sendcount1 == SENDPACK_LOW)
                    {
                        rpwop_radiopack_sendcount++;
                    }
                    rpwop_radiopack_sendcount1++;
                    usleep(MILISEC * DEC_10); //taskDelay(10);
                }
            }
        }
    }  
    else
    {
        rpwop_radiopack_sendcount = HEX_ONE;
        while(rpwop_radiopack_sendcount <= SENDPACK_MED)
        {
            bzero(rpwop_radioparamload,sizeof(rpwop_radioparamload));           
            rpwop_radioparamload[HEX_ONE] = DEC_10;            
            f_activate_setup( rpwop_radioparamload[HEX_ONE], rpwop_al_packto_radio);           
            n = write(rpwop_serialfd, &rpwop_al_packto_radio, HEX_NINE);
            usleep(DEC_10);             
            rpwop_expectedpackidset[ZERO]  = HEX_ONE;
            rpwop_expectedpackidset[ONE]   = HEX_0X26;
            rpwop_expectedpackidset[TWO]   = HEX_SEVEN;  
            rpwop_expectedpackidset[THREE] = HEX_FOUR;            
            bzero(rpwop_ack_packfromradio, sizeof(rpwop_ack_packfromradio)); 
            serial_read(rpwop_serialfd, rpwop_ack_packfromradio, rpwop_expectedpackidset);           
            if(rpwop_ack_packfromradio[ONE] == rpwop_expectedpackidset[ZERO] && rpwop_ack_packfromradio[FOUR] == rpwop_expectedpackidset[ONE])
            {
                rpwop_radiopack_sendcount = LOOP_TERMINATE;
            }
            else
            {
                rpwop_radiopack_sendcount++;
                usleep(MILISEC * DEC_10); //taskDelay(10);
            }
        }        
    }
    
    if(rpwop_radiopack_sendcount == LOOP_TERMINATE)
    {
        rpwop_radiopack_sendcount = HEX_ONE;
        while(rpwop_radiopack_sendcount <= SENDPACK_HIGH)
        {
            bzero(rpwop_radioparamload, sizeof(rpwop_radioparamload));         
            rpwop_radioparamload[ONE] = DEC_27;            
            f_status_request(rpwop_radioparamload[ONE], rpwop_sr_packto_radio);          
            n = write(rpwop_serialfd, &rpwop_sr_packto_radio, HEX_SEVEN);
            usleep(DEC_10);  
            rpwop_expectedpackidset[ZERO]  = HEX_ONE;
            rpwop_expectedpackidset[ONE]   = HEX_0x22;
            rpwop_expectedpackidset[TWO]   = HEX_SEVEN;
            rpwop_expectedpackidset[THREE] = HEX_FOUR;            
            bzero(rpwop_ack_packfromradio, sizeof(rpwop_ack_packfromradio)); 
            serial_read(rpwop_serialfd, rpwop_ack_packfromradio, rpwop_expectedpackidset);         
            if(rpwop_ack_packfromradio[ONE] == rpwop_expectedpackidset[HEX_ZERO] && rpwop_ack_packfromradio[FOUR] == rpwop_expectedpackidset[ONE])
            {                
                rpwop_expectedpackidset[ZERO]  = HEX_0XED;
                rpwop_expectedpackidset[ONE]   = HEX_ZERO;
                rpwop_expectedpackidset[TWO]   = HEX_0x11; 
                rpwop_expectedpackidset[THREE] = HEX_C;               
                bzero(rpwop_status_packfromradio, sizeof(rpwop_status_packfromradio)); 
                serial_read(rpwop_serialfd, rpwop_status_packfromradio, rpwop_expectedpackidset);                
                if(rpwop_expectedpackidset[ZERO] == rpwop_status_packfromradio[ONE]) 
                {
                    bzero(rpwop_radioparamload, sizeof(rpwop_radioparamload));                    
                    rpwop_radioparamload[ONE] = rpwop_status_packfromradio[ONE];                     
                    f_ackpack_toradio(rpwop_radioparamload[ONE], rpwop_ack_packto_radio);                    
                    n = write(rpwop_serialfd, &rpwop_ack_packto_radio, HEX_SEVEN);
                    usleep(DEC_10);  
                    if(rpwop_radioparam[ZERO] == rpwop_status_packfromradio[SIX]) 
                    { 
                        rpwop_radiopack_sendcount  = LOOP_TERMINATE;
                        rpwop_radioparam_setstatus = SUCCESS_STATUS;                        
                        radio_cbitfirsttime_after_fh[rpwop_radio_id] = HEX_ONE;
                        rpw_re_operation_complete[rpwop_radio_id]    = HEX_ONE;
                    }  
                    else
                    {
                        rpwop_radiopack_sendcount++;
                        usleep(MILISEC * DEC_10);//taskDelay(10);
                    }                  
                }                
                else
                {
                    rpwop_radiopack_sendcount++;
                    usleep(MILISEC * DEC_10); //taskDelay(10);
                }
            }
            else
            {
                rpwop_radiopack_sendcount++;
                usleep(MILISEC * DEC_10); //taskDelay(10);
            }
        }
    }    
    return(rpwop_radioparam_setstatus);        
}





/** FHdr-beg *****************************************************
**
** Function name: get_ams_post
**
** Anchor:        RCS_SOW_REQ001
**
** Purpose:       This function get the AMS post results
**
**
** Inputs:        ams bit send msg  ,ams bit recv msg 
**
**
** Outputs:      AMS post result   
**
** FPrj-beg *****************************************************
**
** FPrj-end *****************************************************
** FHdr-end *****************************************************/ 
int32_t get_ams_post(uint8_t *ams_bit_sendmsg, uint8_t *ams_bit_recvmsg)
{
    int32_t vaiu_tx_serialfd             = ERROR;
    struct sockaddr_in servaddr_tx;
    int32_t sock_addrsize_tx;

    int32_t vaiu_rx_serialfd = ERROR;
    struct sockaddr_in servaddr_rx;
    struct sockaddr_in clientaddr_rx;
    int32_t client_addrsize_rx;

    uint8_t power_report[FIVE]          = "";
    uint8_t bus_A_report[FIVE]          = "";
    uint8_t bus_B_report[FIVE]          = "";
    uint8_t E1_line_report[FIVE]        = "";
    uint8_t ams_recvd_relay[DEC_10]     = "";
    uint8_t bit_binary_report[EIGHT]    = "";
    uint8_t vaiu_cbit_sendbuf[DEC_17]   = "";
    uint8_t vaiu_ibit_sendbuf[DEC_17]   = "";
    uint8_t relay_params_recv[DEC_700]  = "";
    uint8_t relay_request_send[DEC_16]  = "";
    uint8_t relay_setbuf_send[DEC_661]  = "";
    uint8_t bit_report_recvbuf[DEC_127] = "";
    uint32_t bit_report_loop 			= HEX_ZERO; 
	uint32_t   bit_reportloop_new 	    = HEX_ZERO;
    uint8_t  relay_radioid   			= HEX_ZERO;
    uint8_t  checksum        			= HEX_ZERO;
    uint8_t  send_res_set    			= HEX_ZERO; 
	uint8_t   num_times_resend          = HEX_ZERO;
    struct timespec ams_recv_timeout_struct;
    if((vaiu_tx_serialfd = socket(AF_INET, SOCK_DGRAM, HEX_ZERO)) == ERROR)
    {
	    return(ERROR);
    }
    sock_addrsize_tx = sizeof(servaddr_tx);
    bzero((int8_t *)&servaddr_tx, sock_addrsize_tx);    
    servaddr_tx.sin_family = AF_INET;
    servaddr_tx.sin_port   = htons(VAIU_BIT_SEND_PORT);
    if((servaddr_tx.sin_addr.s_addr = inet_addr(VAIU_BIT_SERVER_NAME)) == ERROR) 
    {        
        close(vaiu_tx_serialfd);
		return(ERROR);
    }
    if((vaiu_rx_serialfd = socket(AF_INET, SOCK_DGRAM, HEX_ZERO)) == ERROR)
    {
        return(ERROR);
    }
    client_addrsize_rx = sizeof(clientaddr_rx);
    bzero((int8_t *)&servaddr_rx, client_addrsize_rx);    
    servaddr_rx.sin_family      = AF_INET;
    servaddr_rx.sin_port        = htons(VAIU_BIT_RECV_PORT);
    servaddr_rx.sin_addr.s_addr = INADDR_ANY;  /*htonl(INADDR_ANY);*/ /*org*/
    if((bind(vaiu_rx_serialfd, (struct sockaddr *)&servaddr_rx, client_addrsize_rx)) == ERROR)
    {
        close(vaiu_rx_serialfd);
        return(ERROR);
    }        
    /* store these values in one buffer and use memcpy */
    vaiu_cbit_sendbuf[ZERO]     = ONE;
    vaiu_cbit_sendbuf[ONE]      = ONE;
    vaiu_cbit_sendbuf[TWO]      = ZERO;
    vaiu_cbit_sendbuf[THREE]    = ZERO;
    vaiu_cbit_sendbuf[FOUR]     = ONE;
    vaiu_cbit_sendbuf[FIVE]     = ONE;
    vaiu_cbit_sendbuf[SIX]      = ZERO;
    vaiu_cbit_sendbuf[SEVEN]    = ZERO;
    vaiu_cbit_sendbuf[EIGHT]    = HEX_0x55;
    vaiu_cbit_sendbuf[NINE]     = HEX_0xAA;
    vaiu_cbit_sendbuf[TEN]      = HEX_0x30;
    vaiu_cbit_sendbuf[ELEVEN]   = NINE;
    vaiu_cbit_sendbuf[TWELVE]   = ZERO;
    vaiu_cbit_sendbuf[THIRTEEN] = HEX_0x55;
    vaiu_cbit_sendbuf[FOURTEEN] = ZERO;
    vaiu_cbit_sendbuf[FIFTEEN]  = HEX_0x36;
    vaiu_cbit_sendbuf[SIXTEEN]  = HEX_0XA5;
    vaiu_ibit_sendbuf[ZERO]     = ONE;
    vaiu_ibit_sendbuf[ONE]      = ONE;
    vaiu_ibit_sendbuf[TWO]      = ZERO;
    vaiu_ibit_sendbuf[THREE]    = ZERO;
    vaiu_ibit_sendbuf[FOUR]     = ONE;
    vaiu_ibit_sendbuf[FIVE]     = ONE;
    vaiu_ibit_sendbuf[SIX]      = ZERO;
    vaiu_ibit_sendbuf[SEVEN]    = ZERO;
    vaiu_ibit_sendbuf[EIGHT]    = HEX_0x55;
    vaiu_ibit_sendbuf[NINE]     = HEX_0xAA;
    vaiu_ibit_sendbuf[TEN]      = HEX_0xC0;
    vaiu_ibit_sendbuf[ELEVEN]   = NINE;
    vaiu_ibit_sendbuf[TWELVE]   = ZERO;
    vaiu_ibit_sendbuf[THIRTEEN] = HEX_0x64;
    vaiu_ibit_sendbuf[FOURTEEN] = ZERO;
    vaiu_ibit_sendbuf[FIFTEEN]  = HEX_F7;
    vaiu_ibit_sendbuf[SIXTEEN]  = HEX_0XA5;
    relay_request_send[ZERO]    = ONE;
    relay_request_send[ONE]     = ONE;
    relay_request_send[TWO]     = ZERO;
    relay_request_send[THREE]   = ZERO;
    relay_request_send[FOUR]    = ONE;
    relay_request_send[FIVE]    = ONE;
    relay_request_send[SIX]     = ZERO;
    relay_request_send[SEVEN]   = ZERO;
    relay_request_send[EIGHT]   = HEX_0x55;
    relay_request_send[NINE]    = HEX_0xAA;
    relay_request_send[TEN]     = HEX_0x30;
    relay_request_send[ELEVEN]  = EIGHT;
    relay_request_send[TWELVE]  = ZERO;
    relay_request_send[THIRTEEN]= HEX_0x51;
    relay_request_send[FOURTEEN]= HEX_0x33;
    relay_request_send[FIFTEEN] = HEX_0XA5;
    if(ams_bit_sendmsg[ZERO] == ONE || ams_bit_sendmsg[ZERO] == TWO)
    {
        send_res_set = ZERO;
        if(ams_bit_sendmsg[ZERO] == ONE)
        {
            num_times_resend = ONE;
            while(num_times_resend <= TWO)
            {
                if((sendto(vaiu_tx_serialfd, vaiu_ibit_sendbuf, sizeof(vaiu_ibit_sendbuf), ZERO, (struct sockaddr *)&servaddr_tx, sock_addrsize_tx)) == ERROR)
                {
                    if(num_times_resend == TWO)
                    {
                        ams_bit_sendmsg[ZERO] = ZERO;
                    }
                    num_times_resend++;
                }
                else
                {
                    bzero(bit_report_recvbuf, sizeof(bit_report_recvbuf));
                    ams_recv_timeout_struct.tv_sec  = EIGHT;
                    ams_recv_timeout_struct.tv_nsec = ZERO;                    
                    if((recvfrom(vaiu_rx_serialfd, bit_report_recvbuf, sizeof(bit_report_recvbuf), 0, (struct sockaddr *)&clientaddr_rx, &client_addrsize_rx))
                        == ERROR)
                    {                        
                        if(num_times_resend == HEX_TWO)
                        {
                            ams_bit_recvmsg[ZERO] = ZERO;
                            send_res_set = ONE;
                        }
                        num_times_resend++;
                    }
                    else
                    {
                        usleep(SEC * DEC_60);   // taskdelay(sysClkRateGet() * 60);
                        send_res_set = ZERO;
                        num_times_resend = HEX_0xDD;
                    }
                }
            }
        }
        if(send_res_set == ZERO)
        {
            num_times_resend = ONE;
            while(num_times_resend <= HEX_TWO)
            {
                if((sendto(vaiu_tx_serialfd, vaiu_cbit_sendbuf, sizeof(vaiu_cbit_sendbuf), HEX_ZERO, (struct sockaddr *)&servaddr_tx, sock_addrsize_tx)) == ERROR)
                {                    
                    if(num_times_resend == HEX_TWO)
                    {
                        ams_bit_recvmsg[ZERO] = HEX_ZERO;
                    }
                    num_times_resend++;                
                }
                else
                {
                    bzero(bit_report_recvbuf, sizeof(bit_report_recvbuf));
                    ams_recv_timeout_struct.tv_sec  = HEX_EIGHT;
                    ams_recv_timeout_struct.tv_nsec = HEX_ZERO;        
                    if((recvfrom(vaiu_rx_serialfd, bit_report_recvbuf, sizeof(bit_report_recvbuf), HEX_ZERO, (struct sockaddr *)&clientaddr_rx, &client_addrsize_rx))
                        == ERROR)
                    {
                        if(num_times_resend == HEX_TWO)
                        {
                            ams_bit_recvmsg[ZERO] = HEX_ZERO;
                        }
                        num_times_resend++;
                    }
                    else
                    {
                        num_times_resend       = HEX_0xDD;                        
                        ams_bit_recvmsg[ZERO]  = HEX_FF;
                        ams_bit_recvmsg[ONE]   = bit_report_recvbuf[FOURTEEN];
                        ams_bit_recvmsg[TWO]   = bit_report_recvbuf[FIFTEEN];
                        ams_bit_recvmsg[THREE] = bit_report_recvbuf[TWENTYFIVE];
                        ams_bit_recvmsg[FOUR]  = bit_report_recvbuf[FOURTYONE];
                        ams_bit_recvmsg[FIVE]  = bit_report_recvbuf[TWENTYONE];
                        ams_bit_recvmsg[SIX]   = bit_report_recvbuf[TWENTYTHREE];
                        ams_bit_recvmsg[SEVEN] = bit_report_recvbuf[TWENTYFOUR];                        
                        if(bit_report_recvbuf[FIFTEEN] == HEX_0x2F) 
                        {
                            ;
                        }
                        else
                        {
                            dec_to_bin(bit_report_recvbuf[FOURTEEN], bit_binary_report);
                            
                        }                        
                    }
                }
            }
        }
    }
    else if(ams_bit_sendmsg[ZERO] == HEX_THREE)
    {
        num_times_resend = HEX_ONE;
        while(num_times_resend <= HEX_THREE)
        {
            if((sendto(vaiu_tx_serialfd, relay_request_send, sizeof(relay_request_send), HEX_ZERO, (struct sockaddr *)&servaddr_tx, sock_addrsize_tx)) == ERROR)
            {
                if(num_times_resend >= HEX_THREE)
                {
                    ams_bit_recvmsg[ZERO] = HEX_ZERO;
                }
                num_times_resend++;
            }
            else
            {
                bzero(relay_params_recv, sizeof(relay_params_recv));
                ams_recv_timeout_struct.tv_sec  = DEC_10;
                ams_recv_timeout_struct.tv_nsec = HEX_ZERO;             
                if((recvfrom(vaiu_rx_serialfd, relay_params_recv, sizeof(relay_params_recv), 0, (struct sockaddr *)&clientaddr_rx, &client_addrsize_rx))
                    == ERROR)
                {
                    if(num_times_resend == HEX_THREE)
                    {
                        ams_bit_recvmsg[ZERO] = HEX_ZERO;
                    }
                    num_times_resend++;
                }
                else
                {
                    bzero(relay_setbuf_send, sizeof(relay_setbuf_send));                    
                    relay_setbuf_send[ZERO]     = HEX_ONE;
                    relay_setbuf_send[ONE]      = HEX_ONE;                    
                    relay_setbuf_send[TWO]      = HEX_ZERO;
                    relay_setbuf_send[THREE]    = HEX_ZERO;
                    relay_setbuf_send[FOUR]     = HEX_ONE;
                    relay_setbuf_send[FIVE]     = HEX_ONE;
                    relay_setbuf_send[SIX]      = HEX_ZERO;
                    relay_setbuf_send[SEVEN]    = HEX_ZERO;
                    relay_setbuf_send[EIGHT]    = HEX_0x55;
                    relay_setbuf_send[NINE]     = HEX_0xAA;
                    relay_setbuf_send[TEN]      = HEX_0xC0;
                    relay_setbuf_send[ELEVEN]   = HEX_0x8D;
                    relay_setbuf_send[TWELVE]   = HEX_TWO;
                    relay_setbuf_send[THIRTEEN] = HEX_0x51;
                    relay_setbuf_send[FOURTEEN] = HEX_NINE;
                    relay_setbuf_send[FIFTEEN]  = HEX_FF;                    
                    for(bit_report_loop = DEC_16; bit_report_loop <= DEC_163; bit_report_loop++)
                    {
                        relay_setbuf_send[bit_report_loop] = relay_params_recv[bit_report_loop];
                    }
                    if(relay_setbuf_send[DEC_162] == HEX_ZERO)
                    {
                        relay_setbuf_send[DEC_162] = HEX_TWO;
                    }
                    else
                    {
                        relay_setbuf_send[DEC_163] = HEX_TWO;
                    }
                    
                    for(bit_report_loop = DEC_164; bit_report_loop <= DEC_178; bit_report_loop++)
                    {
                        relay_setbuf_send[bit_report_loop] = relay_params_recv[bit_report_loop];
                    }                    
                    relay_setbuf_send[DEC_179] = HEX_0x23;
                    relay_setbuf_send[DEC_180] = HEX_0X25;
                    relay_setbuf_send[DEC_181] = HEX_0X27;
                    relay_setbuf_send[DEC_182] = HEX_0x23;
                    relay_setbuf_send[DEC_183] = HEX_0x24;
                    relay_setbuf_send[DEC_184] = HEX_0X26;
                    relay_setbuf_send[DEC_185] = HEX_0x28;
                    relay_setbuf_send[DEC_186] = HEX_0x23;                    
                    for(bit_report_loop = HEX_ZERO; bit_report_loop < HEX_SIX; bit_report_loop++)
                    {
                        if(ams_bit_sendmsg[bit_report_loop + HEX_ONE] == HEX_ZERO)
                        {
                            for(relay_radioid = HEX_ONE; relay_radioid <= HEX_SIX; relay_radioid++)
                            {
                                send_res_set =HEX_ONE;
                                for(bit_reportloop_new = HEX_ZERO; bit_reportloop_new < HEX_SIX; bit_reportloop_new++)
                                {
                                    if(relay_radioid == ams_recvd_relay[bit_reportloop_new])
                                    {
                                        send_res_set = HEX_ZERO;
                                        break;
                                    }
                                }
                                if(send_res_set != HEX_ZERO)
                                {
                                    ams_recvd_relay[bit_report_loop] = relay_radioid;
                                    break;
                                }
                            }
                        }
                        else
                        {
                            ams_recvd_relay[bit_report_loop] = ams_bit_sendmsg[bit_report_loop + HEX_ONE];
                        }
                    }
                    bit_report_loop = HEX_ZERO;
                    for(relay_radioid = HEX_ZERO; relay_radioid < HEX_SIX; relay_radioid = relay_radioid + HEX_TWO)
                    {
                        relay_setbuf_send[DEC_179 + bit_report_loop] = HEX_0x22 + ams_recvd_relay[relay_radioid];
                        relay_setbuf_send[DEC_183 + bit_report_loop] = HEX_0x22 + ams_recvd_relay[relay_radioid + HEX_ONE];
                        bit_report_loop++;
                    }                    
                    for(bit_report_loop = DEC_187; bit_report_loop <= DEC_657; bit_report_loop++)
                    {
                        relay_setbuf_send[bit_report_loop] = relay_params_recv[bit_report_loop];
                    }
                    relay_setbuf_send[DEC_658] = HEX_ZERO;                    
                    for(bit_report_loop = HEX_EIGHT; bit_report_loop <= DEC_658; bit_report_loop++)
                    {
                        checksum = checksum ^ relay_setbuf_send[bit_report_loop];
                    }
                    checksum 				   = checksum ^ HEX_0XA5;                    
                    relay_setbuf_send[DEC_659] = checksum;
                    relay_setbuf_send[DEC_660] = HEX_0XA5;                    
                    if(relay_setbuf_send[DEC_179] == relay_params_recv[DEC_179] && relay_setbuf_send[DEC_180] == relay_params_recv[DEC_180] &&
                       relay_setbuf_send[DEC_181] == relay_params_recv[DEC_181] && relay_setbuf_send[DEC_183] == relay_params_recv[DEC_183] &&
                       relay_setbuf_send[DEC_184] == relay_params_recv[DEC_184] && relay_setbuf_send[DEC_185] == relay_params_recv[DEC_185])
                    {
                        ams_bit_recvmsg[ZERO] = HEX_FF;
                        num_times_resend = HEX_0xDD;
                    }
                    else
                    {
                        usleep(SEC);   // taskDelay(sysClkRateGet());                        
                        if((sendto(vaiu_tx_serialfd, relay_setbuf_send, sizeof(relay_setbuf_send), HEX_ZERO, (struct sockaddr *)&servaddr_tx, sock_addrsize_tx))   				    == ERROR)
                        {
                            if(num_times_resend >= HEX_THREE)
                            {
                                ams_bit_recvmsg[ZERO] = HEX_ZERO;
                            }
                            num_times_resend++;
                        }
                        ams_recv_timeout_struct.tv_sec  = DEC_15;
                        ams_recv_timeout_struct.tv_nsec = HEX_ZERO;            
                       if((recvfrom(vaiu_rx_serialfd, bit_report_recvbuf, sizeof(bit_report_recvbuf), HEX_ZERO, 
                                                   (struct sockaddr *)&clientaddr_rx, &client_addrsize_rx))== ERROR)
                        {
                            if(num_times_resend >= HEX_THREE)
                            {
                                ams_bit_recvmsg[ZERO] = HEX_ZERO;
                            }
                            num_times_resend++;
                        }
                        else
                        {
                            ams_bit_recvmsg[ZERO] = HEX_FF;
                            num_times_resend = HEX_0xDD;
                        }
                    }
                }
            }
        }
    }
    else
    {
		;
    }
    close(vaiu_tx_serialfd);
    close(vaiu_rx_serialfd);    
    vaiu_tx_serialfd = ERROR;
    vaiu_rx_serialfd = ERROR;
    return(OK);
}


/** FHdr-beg *****************************************************
**
** Function name: ams_post
**
** Anchor:        RCS_SOW_REQ001
**
** Purpose:       This function to perform the AMS post 
**
**
** Inputs:        NONE 
**
**
** Outputs:      NONE   
**
** FPrj-beg *****************************************************
**
** FPrj-end *****************************************************
** FHdr-end *****************************************************/ 
void *ams_post()
{
    
    mqd_t AMS_BIT_Tx_MsgQ            = ERROR;    
    struct  sockaddr_in ams_bit_servaddr;
    int32_t ams_bit_sockaddr_size;
    int32_t ams_bit_serialfd         = ERROR;
    uint8_t ams_bit_sendmsg[SIXTEEN] = ""; 
	uint8_t ams_bit_recvmsg[TEN]     = "";  
    int32_t bit_status_retval        = HEX_ZERO; 
	int32_t bit_report_loop 		 = HEX_ZERO;    
    uint8_t power_report[NUM_ACP]    = "";
    uint8_t bus_A_report[NUM_ACP]    = "";
    uint8_t bus_B_report[NUM_ACP]    = "";
    uint8_t E1_line_report[NUM_ACP]  = "";    
    uint8_t ams_numbers 			 = HEX_ZERO; 
	uint8_t ams_numbers1			 = HEX_ZERO;
    uint8_t binary_buf[FOUR][NUM_ACP]; 
	uint8_t ams_bit_ifstatementcheck = HEX_ZERO;
    uint8_t bit_binary_report[EIGHT] = "";
    int32_t prio;
    struct mq_attr new_attr, old_attr;   
    /* create read write queue that is blocking */
    new_attr.mq_flags    = HEX_ZERO;
    new_attr.mq_maxmsg   = HEX_ONE;
    new_attr.mq_msgsize  = DEC_16;	    
    struct timespec ams_recv_timeout_struct;    
    while(mcs_present_stateload != HEX_FIVE)
    {    
        
        //while(TRUE)
		for(;;)
        {            
			bzero(ams_bit_sendmsg, sizeof(ams_bit_sendmsg));
            if(mq_receive(AMS_BIT_Tx_MsgQ, ams_bit_sendmsg, sizeof(ams_bit_sendmsg), &prio) == ERROR)
            {
                //mq_unlink(msgQname); 
            	mq_close(AMS_BIT_Tx_MsgQ);
                AMS_BIT_Tx_MsgQ = ERROR;
                break;
            }
            else
            {
                get_ams_post(ams_bit_sendmsg, ams_bit_recvmsg);
				if(ams_bit_recvmsg[ZERO] ==  HEX_FF)
                {
                    if(ams_bit_sendmsg[ZERO] == HEX_ONE)
                    {
                        intercom_res_amscc = HEX_ONE;                        
                    }
                    else if(ams_bit_sendmsg[ZERO] == HEX_TWO)
                    {
                        intercom_res_amscc = HEX_ONE;                       
                    }
                    else if(ams_bit_sendmsg[ZERO] == HEX_THREE)
                    {
                        relay_set_status = HEX_FF;                       
                    }
                    else
                    {
						;
                    }
                    if(ams_bit_sendmsg[ZERO] == HEX_ONE|| ams_bit_sendmsg[ZERO] == HEX_TWO)
                    {
                        if(ams_bit_recvmsg[ONE] ==  HEX_0x2F)
                        {
                            ;
                        }
                        else
                        {                            
                            dec_to_bin(ams_bit_recvmsg[ONE], bit_binary_report);
                            
                        }                        
                        if(ams_bit_recvmsg[TWO] == ACP_POWER_VALUE[NUM_ACP])
                        {
							;
                        }
                        else
                        {                        
                            dec_to_bin(ams_bit_recvmsg[THREE], bit_binary_report); //ACP Power Supply  
                            for(bit_report_loop = HEX_ZERO; bit_report_loop < NUM_ACP; bit_report_loop++)
                            {
                                power_report[bit_report_loop] = bit_binary_report[bit_report_loop];
                            }                        
                            dec_to_bin(ams_bit_recvmsg[FOUR], bit_binary_report); //ACP E1 Line Signal                                
                            for(bit_report_loop = HEX_ZERO; bit_report_loop < NUM_ACP; bit_report_loop++)
                            {
                                E1_line_report[bit_report_loop] = bit_binary_report[bit_report_loop];
                            }                        
                            dec_to_bin(ams_bit_recvmsg[FIVE], bit_binary_report); //ACP BUS A Signal                                
                            for(bit_report_loop = HEX_ZERO; bit_report_loop < NUM_ACP; bit_report_loop++)
                            {
                                bus_A_report[bit_report_loop] = bit_binary_report[bit_report_loop];
                            }                        
                            dec_to_bin(ams_bit_recvmsg[SIX], bit_binary_report); //ACP BUS B Signal                                
                            for(bit_report_loop = HEX_ZERO; bit_report_loop < NUM_ACP; bit_report_loop++)
                            {
                                bus_B_report[bit_report_loop] = bit_binary_report[bit_report_loop];
                            }                        
                            
                            
                        }
                    }
                    else
                    {
						;
					}
                }
                else
                { 
                    if(ams_bit_sendmsg[ZERO] == HEX_ONE || ams_bit_sendmsg[ZERO] == HEX_TWO)
                    {
                        intercom_res_amscc = HEX_ZERO;
                    }
                    else
                    {
                        relay_set_status = HEX_ZERO;
                    }
                }        
            }            
            if(intercom_res_amscc == HEX_ONE)
            {
                if(ams_bit_sendmsg[ZERO] == HEX_ONE || ams_bit_sendmsg[ZERO] == HEX_TWO)
                {
                    sem_wait(&Ams_Cbit_SyncSem); 
                    ams_num_faults = HEX_ONE;                
                    bzero((int8_t *)&amsbit_faultstatus, sizeof(amsbit_faultstatus));
                    bzero(amsbit_faultcode, sizeof(amsbit_faultcode));
                    bzero(amsbit_curstatus, sizeof(amsbit_curstatus));                    
                    if(ams_bit_recvmsg[1] != HEX_0x2F)
                    {
                        amsbit_faultstatus[(ams_num_faults * HEX_FOUR) + HEX_ZERO] = ams_nodeidbuf[ZERO];
                        amsbit_faultstatus[(ams_num_faults * HEX_FOUR) + HEX_ONE]  = ams_nodeidbuf[ONE];
                        amsbit_faultstatus[(ams_num_faults * HEX_FOUR) + HEX_TWO]  = ams_nodeidbuf[TWO];
                        amsbit_faultstatus[(ams_num_faults * HEX_FOUR) + THREE]    = ams_nodeidbuf[THREE];                    
                        amsbit_faultcode[ams_num_faults] = ams_nodefaultcode[ZERO];
                        amsbit_curstatus[ams_num_faults] = HEX_FF;
                        ams_num_faults++;
                    }
                    else
					{
						;
					}                
                    if(ams_bit_recvmsg[TWO] != ACP_POWER_VALUE[NUM_ACP])
                    {
                        bzero(bit_binary_report, sizeof(bit_binary_report));
                        bzero((int8_t *)&binary_buf, sizeof(binary_buf));                    
                        dec_to_bin(ams_bit_recvmsg[FOUR], bit_binary_report);
                        for(ams_numbers = HEX_ZERO; ams_numbers < NUM_ACP; ams_numbers++)
                        {
                            binary_buf[ZERO][ams_numbers] = bit_binary_report[ams_numbers];
                        }                    
                        dec_to_bin(ams_bit_recvmsg[FIVE], bit_binary_report);
                        for(ams_numbers = HEX_ZERO; ams_numbers < NUM_ACP; ams_numbers++)
                        {
                            binary_buf[ONE][ams_numbers] = bit_binary_report[ams_numbers];
                        }                    
                        dec_to_bin(ams_bit_recvmsg[SIX], bit_binary_report);
                        for(ams_numbers = HEX_ZERO; ams_numbers < NUM_ACP; ams_numbers++)
                        {
                            binary_buf[TWO][ams_numbers] = bit_binary_report[ams_numbers];
                        }                    
                        dec_to_bin(ams_bit_recvmsg[THREE], bit_binary_report);
                        for(ams_numbers = HEX_ZERO; ams_numbers < NUM_ACP; ams_numbers++)
                        { 
                            binary_buf[THREE][ams_numbers] = bit_binary_report[ams_numbers];
                        }                    
                        ams_bit_ifstatementcheck = HEX_ONE;                    
                        for(ams_numbers = HEX_ZERO; ams_numbers < NUM_ACP; ams_numbers++)
                        {
                            bit_status_retval = HEX_FF;                                                  
                            if(binary_buf[THREE][ams_numbers] == HEX_ONE)
                            {
                                amsbit_faultstatus[(ams_num_faults * HEX_FOUR) + HEX_ZERO] = HEX_ZERO;
                                amsbit_faultstatus[(ams_num_faults * HEX_FOUR) +HEX_ONE]   = HEX_ZERO;
                                amsbit_faultstatus[(ams_num_faults * HEX_FOUR) + HEX_TWO]  = ams_numbers + HEX_TWO;
                                amsbit_faultstatus[(ams_num_faults * HEX_FOUR) + HEX_THREE]= AMS_NODE_ID;                            
                                amsbit_faultcode[ams_num_faults] = HEX_ONE;
                                amsbit_curstatus[ams_num_faults] = HEX_FF;
                                ams_num_faults++; 
                                ams_bit_ifstatementcheck = ams_bit_ifstatementcheck & HEX_ONE;                                                      
                            }
                            else
                            {
                                for(ams_numbers1 = HEX_ZERO; ams_numbers1 < HEX_THREE; ams_numbers1++)
                                {
                                    if(binary_buf[ams_numbers1][ams_numbers] == HEX_ONE)
                                    {
                                        if(bit_status_retval != ams_numbers)
                                        {
                                            amsbit_faultstatus[(ams_num_faults * HEX_FOUR) + HEX_ZERO]  = HEX_ZERO;
                                            amsbit_faultstatus[(ams_num_faults * HEX_FOUR) + HEX_ONE]   = HEX_ZERO;
                                            amsbit_faultstatus[(ams_num_faults * HEX_FOUR) + HEX_TWO]   = ams_numbers + HEX_ONE;
                                            amsbit_faultstatus[(ams_num_faults * HEX_FOUR) + HEX_THREE] = AMS_NODE_ID;                            
                                            amsbit_curstatus[ams_num_faults] = HEX_FF;                                    
                                            if(binary_buf[ZERO][ams_numbers] == HEX_ONE)
                                            {
                                                amsbit_faultcode[ams_num_faults] = HEX_ONE;
                                            }
                                            else
                                            {
                                                amsbit_faultcode[ams_num_faults] = HEX_TWO;
                                            }
                                            ams_num_faults++;
                                            bit_status_retval = ams_numbers;
                                        }
                                        else
					                    {
					                          ;
                                        }								
                                        amsbit_faultstatus[(ams_num_faults * HEX_FOUR) + HEX_ZERO] = ams_nodeidbuf[(ams_numbers * DEC_16)  + ((ams_numbers1 + HEX_ONE) * HEX_FOUR) + HEX_ZERO];
                                        amsbit_faultstatus[(ams_num_faults * HEX_FOUR) + HEX_ONE] = ams_nodeidbuf[(ams_numbers * DEC_16)   + ((ams_numbers1 + HEX_ONE) * HEX_FOUR) + HEX_ONE];
                                        amsbit_faultstatus[(ams_num_faults * HEX_FOUR) + HEX_TWO] = ams_nodeidbuf[(ams_numbers * DEC_16)   + ((ams_numbers1 + HEX_ONE) * HEX_FOUR) + HEX_TWO];
                                        amsbit_faultstatus[(ams_num_faults * HEX_FOUR) + HEX_THREE] = ams_nodeidbuf[(ams_numbers * DEC_16) + ((ams_numbers1 + HEX_ONE) * HEX_FOUR) + HEX_THREE];                                        
                                        amsbit_faultcode[ams_num_faults] = ams_nodefaultcode[(ams_numbers * HEX_FOUR) + ams_numbers1 + HEX_ONE];
                                        amsbit_curstatus[ams_num_faults] = HEX_FF;
                                        ams_num_faults++;
                                    }
                                    else
				                    {
					                     ;
				                    } 
                                }
                            
                                if(binary_buf[ZERO][ams_numbers] == HEX_ONE)
                                {
                                    ams_bit_ifstatementcheck = ams_bit_ifstatementcheck & HEX_ONE;
                                }
                                else
                                {
                                    ams_bit_ifstatementcheck = ams_bit_ifstatementcheck & HEX_ZERO;
                                }
                            }
                        }
                    }
                    else
					{
						;
					}                
                    if(ams_bit_ifstatementcheck == HEX_ONE)
                    {
                        amsbit_faultstatus[ZERO] = HEX_ZERO;
                        amsbit_faultstatus[ONE] = HEX_ZERO;
                        amsbit_faultstatus[TWO] = HEX_ZERO;
                        amsbit_faultstatus[THREE] = AMS_NODE_ID;                    
                        amsbit_faultcode[ZERO] = HEX_ONE;
                        amsbit_curstatus[ZERO] = HEX_FF;
                    }                
                    else
                    {
                        if(ams_num_faults == HEX_ONE)
                        {
                            ams_num_faults = HEX_ZERO;
                        }
                        else
                        {
                            amsbit_faultstatus[ZERO]   = HEX_ZERO;
                            amsbit_faultstatus[ONE]    = HEX_ZERO;
                            amsbit_faultstatus[TWO]    = HEX_ZERO;
                            amsbit_faultstatus[THREE]  = AMS_NODE_ID;                    
                            amsbit_faultcode[ZERO]     = HEX_TWO;
                            amsbit_curstatus[ZERO]     = HEX_FF;
                        }
                    }
                    sem_post(&Ams_Cbit_SyncSem);
                }
                else
                {
					;
				}        
            }
            else
            {
                ams_num_faults = SUBSYSTEM_NORESPONSE;
            } 
        
            ams_msg_sent_status = HEX_ZERO; 
            
        } /* end of while(1) */          

    } /* end of while(mcs_present_stateload != 5) */  
    ams_post_thread = HEX_ZERO;
    return(OK);     
}


/** FHdr-beg *****************************************************
**
** Function name: ams_bit_relay
**
** Anchor:        
**
** Purpose:       function to send the AMS BIT Type 
**				  ams_bit_type -> 1 = IBit
**				  ams_bit_type -> 2 = CBit
**				  ams_bit_type -> 3 = relaypairconfig
**				   
** Inputs:        ams bit type,ams bit relay 
**
**
** Outputs:      ams bit relay status    
**
** FPrj-beg *****************************************************
**
** FPrj-end *****************************************************
** FHdr-end *****************************************************/ 

int32_t ams_bit_relay(uint8_t ams_bit_type, uint8_t *ams_bit_relay)
{
    mqd_t AMS_BIT_Tx_MsgQ               = ERROR;
    struct mq_attr new_attr, old_attr;
    uint8_t ams_bit_relay_txrx[SIXTEEN] = ""; 
	uint8_t abr_radio_id 				= HEX_ZERO;  
	uint8_t timeout_target 				= HEX_ZERO;
    ams_bit_relay_txrx[ZERO] 			= ams_bit_type;
    /* create read write queue that is blocking */
    new_attr.mq_flags    = HEX_ZERO;
    new_attr.mq_maxmsg   = HEX_ONE;
    new_attr.mq_msgsize  = DEC_16;
    if(ams_bit_type == HEX_ONE)
    {
		timeout_target = DEC_85;
    }    	
    else if(ams_bit_type == HEX_TWO)
    {
		timeout_target = DEC_25;
    }
    else if(ams_bit_type == HEX_THREE)
    {
		timeout_target = DEC_55;	
        for(abr_radio_id = HEX_ZERO; abr_radio_id < ((TOTAL_RADIOS / HEX_TWO) * HEX_TWO); abr_radio_id++)
        {
			ams_bit_relay_txrx[abr_radio_id + HEX_ONE] = ams_bit_relay[abr_radio_id];
        }	
    }
    else
    {
		;
    }
    abr_radio_id = HEX_ONE;
    if(AMS_BIT_Tx_MsgQ != ERROR)
    {
		ams_msg_sent_status = HEX_ONE;
        if(mq_send(AMS_BIT_Tx_MsgQ, ams_bit_relay_txrx, sizeof (ams_bit_relay_txrx), HI_PRIO) == ERROR)
        {            
            //mq_unlink(msgQname); 
            mq_close(AMS_BIT_Tx_MsgQ);            
            AMS_BIT_Tx_MsgQ = ERROR;
            if(ams_bit_type == HEX_ONE || ams_bit_type == HEX_TWO)
            {
				intercom_res_amscc = HEX_ZERO;
            }
            else
            {
				relay_set_status = HEX_ZERO;
            }
        }
        else
        {
	    //while(TRUE)
		for(;;)
            {
			if(ams_msg_sent_status == HEX_ZERO)
			{
				break;
            }
            else
            {
				if(abr_radio_id >= timeout_target)
                {
					if(ams_bit_type == HEX_ONE || ams_bit_type == HEX_TWO)
            		{
						intercom_res_amscc = HEX_ZERO;
            		}
            		else
            		{	
						relay_set_status = HEX_ZERO;
            		}

                        ams_msg_sent_status = HEX_ZERO;
                        break;                       
                    }
                    else
					{
						;
					}                    
					usleep(SEC);        //taskDelay(sysClkRateGet());
                    abr_radio_id++; 

                }
            } // end of while	
        } // end of else
    }
    else
    {
		relay_set_status = HEX_ZERO;
        intercom_res_amscc = HEX_ZERO;  
    }
    if(intercom_res_amscc == HEX_ZERO)
    {
    	ams_num_faults = SUBSYSTEM_NORESPONSE;
    }
   
    return(OK);
}


/** FHdr-beg *****************************************************
**
** Function name: radio_post
**
** Anchor:     RCS_SOW_REQ001   
**
** Purpose:       function to get the radio post results 
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
void *radio_post(void *arg)
{
    //int8_t post_tasknameset[20] = "";

    uint8_t post_radio_id = *(uint8_t *)arg; 
    uint8_t radio_post_reload 			= HEX_ZERO; 
	uint8_t radio_num_postfaults 		= HEX_ZERO;
	uint8_t radiopost_node_num 			= HEX_ZERO;
    uint8_t radio_post_faultstatus[THIRTYSIX] = ""; 
	uint8_t radiopost_faultcode[NINE]	= ""; 
	uint8_t radiopost_curstatus[NINE] 	= "";
    int32_t res 						= HEX_ZERO;
    radio_tr_comm_stop[post_radio_id]   = HEX_ZERO;
    radio_isfirst_tr_complete[post_radio_id] = HEX_ZERO;
    radio_init_status[post_radio_id]    = FAILURE_STATUS;
    if(radio_serialfd[post_radio_id] != ERROR)
    {
   
        radio_init(post_radio_id, radio_serialfd[post_radio_id]);        
        usleep(SEC * HEX_TWO);    /* taskdelay(sysclkrateget() * 2);  */        
        //radiotr_commcheck function for calculating radiotr_commcheck of MCS  
        res = pthread_create(&radio_tr_comm_thread[post_radio_id], NULL, radio_tr_comm_check, &post_radio_id);       
        //while(TRUE)
		for(;;)
        {
            if(radio_isfirst_tr_complete[post_radio_id] == HEX_ONE)
            {
                break;
            }
            else
            {
                usleep(SEC * HEX_TWO);         /* taskdelay(sysclkrateget() * 2); */
            }
        }        
        if(radio_commstatus[post_radio_id] == HEX_ONE)
        {            
            sem_wait(&radio_syncsem[post_radio_id]);
            radio_param_setstatus[post_radio_id] = clear_nvm(post_radio_id, radio_serialfd[post_radio_id]);            
            sem_post(&radio_syncsem[post_radio_id]);
            usleep(SEC * HEX_FIVE);                                  /* taskdelay(sysclkrateget() * 5); */
        }
        else
        {
            ;
        }
        if(radio_commstatus[post_radio_id] == HEX_ONE)
        {
            radio_present_state[post_radio_id] = INIT_STATE;              /* 1 = Radio in Init state */
            radio_init_status[post_radio_id]   = SUCCESS_STATUS;
        }
        else
        {
            ;
        }    
    } 
    else
    {
        radio_present_state[post_radio_id] = OFF_STATE;            /* 5 = Radio in OFF state */
    }    
    /* This needs to be checked with radio_post_thread */
    radio_post_thread[post_radio_id] = HEX_ZERO;
    return(OK);
}


/** FHdr-beg *****************************************************
**
** Function name: pfm_bit_command
**
** Anchor:        RCS_SOW_REQ002
**
** Purpose:       function perform PFM Bit Command 
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

void *pfm_bit_command()
{
    //uint8_t PFMBIT_TaskNameSet[20]   = "";
    int32_t pfm_numbytes_recv         		= HEX_ZERO,  pfm_ctrlloop = HEX_ZERO;
    uint8_t pfmbit_cmd_recvbuf[DEC_1000]    = "";
    uint8_t pfmbit_cmd_recvbufmod[DEC_500]  = "";
    uint8_t bit_nodeidentifier        		= HEX_ZERO,  proc_B_talkmsg[FIVE] = "";
    uint8_t res 							= HEX_ZERO;   
    //while(TRUE)
	for(;;)
    {
        pfm_numbytes_recv = HEX_ZERO;
        bzero(pfmbit_cmd_recvbuf, sizeof(pfmbit_cmd_recvbuf));        
        if((pfm_numbytes_recv = recv(pfmbit_sockfd, (int8_t *)&pfmbit_cmd_recvbuf, DEC_12, HEX_ZERO)) <= HEX_ZERO)
        {
            break;
        }       
        else
        {                               
            switch(pfmbit_cmd_recvbuf[ZERO])
            {
                case THREE:
                          {
                              /* CBit Periodicity */
                             
                              break;
                          }
                case FIVE:
                          {
                              /* IBit Command */
                         
                              break;
                          }       
                case SEVEN:
                          {
                              /* MBit Command */
                              
                              break;
                          }
                case NINE:
                        {                             
                            is_firstpfm_recvd = HEX_ONE;                                                           
                            if(mcs_pfm_thread == HEX_ZERO)
                            {  
                                //while(TRUE)
								for(;;)
                                    {
                                        if(stop_pfm_cmd == HEX_ZERO)
                                        {
											break;
                                        }
                                        else
                                        {                                          
                                           usleep(SEC * HEX_TWO);  // taskDelay(sysClkRateGet() * 2);
                                        }
                                    }
                                    pfm_numbytes_recv = HEX_ZERO;
                                    bzero(pfmbit_cmd_recvbufmod, sizeof(pfmbit_cmd_recvbufmod));
                                    recv(pfmbit_sockfd, (int8_t *)&pfmbit_cmd_recvbufmod, DEC_408, HEX_ZERO);
                                    for(pfm_ctrlloop = HEX_ZERO; pfm_ctrlloop < DEC_408; pfm_ctrlloop++)
                                    {
                                        pfmbit_cmd_recvbuf[DEC_12 + pfm_ctrlloop] = pfmbit_cmd_recvbufmod[pfm_ctrlloop];
                                    }                                    
                                    mcs_pfmload_struct_main = (struct MSC_MCS_PFM_LOAD_MAIN *)&pfmbit_cmd_recvbuf[ZERO];
                                    pfm_numbytes_recv = pfm_numbytes_recv + DEC_420;                                  
                                    recv(pfmbit_sockfd, (int8_t *)&pfmbit_cmd_recvbufmod, (pfmbit_cmd_recvbuf[DEC_416] * HEX_EIGHT), HEX_ZERO);                                    
                                    for(pfm_ctrlloop = HEX_ZERO; pfm_ctrlloop < (pfmbit_cmd_recvbuf[DEC_416] * HEX_EIGHT); pfm_ctrlloop++)
                                    {
                                        pfmbit_cmd_recvbuf[pfm_numbytes_recv + pfm_ctrlloop] = pfmbit_cmd_recvbufmod[pfm_ctrlloop];
                                    }                                    
                                    mcs_pfmload_struct_blank = (struct MSC_MCS_PFM_LOAD_BLANK *)&pfmbit_cmd_recvbuf[pfm_numbytes_recv];
                                    pfm_numbytes_recv = pfm_numbytes_recv + (pfmbit_cmd_recvbuf[DEC_416] * HEX_EIGHT);                                  
                                    bzero(pfmbit_cmd_recvbufmod, sizeof(pfmbit_cmd_recvbufmod));
                                    recv(pfmbit_sockfd, (int8_t *)&pfmbit_cmd_recvbufmod, DEC_100, HEX_ZERO);
                                    for(pfm_ctrlloop = HEX_ZERO; pfm_ctrlloop < DEC_100; pfm_ctrlloop++)
                                    {
                                       pfmbit_cmd_recvbuf[pfm_numbytes_recv + pfm_ctrlloop] = pfmbit_cmd_recvbufmod[pfm_ctrlloop];
                                    }                                  
                                    bzero(pfmbit_cmd_recvbufmod, sizeof(pfmbit_cmd_recvbufmod));
                                    recv(pfmbit_sockfd, (int8_t *)&pfmbit_cmd_recvbufmod, (DEC_20 * pfmbit_cmd_recvbuf[pfm_numbytes_recv + DEC_96]), HEX_ZERO);
                                    for(pfm_ctrlloop = HEX_ZERO; pfm_ctrlloop < (DEC_20 * pfmbit_cmd_recvbuf[pfm_numbytes_recv + DEC_96]); pfm_ctrlloop++)
                                    {
                                       pfmbit_cmd_recvbuf[pfm_numbytes_recv + DEC_100 + pfm_ctrlloop] = pfmbit_cmd_recvbufmod[pfm_ctrlloop];
                                    }
                                    mcs_pfmload_struct_full = (struct MSC_MCS_PFM_LOAD_FULL *)&pfmbit_cmd_recvbuf[pfm_numbytes_recv];                                  
                                    pfm_numbytes_recv = pfm_numbytes_recv + DEC_100 + (DEC_20 * pfmbit_cmd_recvbuf[pfm_numbytes_recv + DEC_96]);
                                    bzero(pfmbit_cmd_recvbufmod, sizeof(pfmbit_cmd_recvbufmod));
                                    recv(pfmbit_sockfd, (int8_t *)&pfmbit_cmd_recvbufmod, HEX_FOUR, HEX_ZERO);
                                    for(pfm_ctrlloop = HEX_ZERO; pfm_ctrlloop < HEX_FOUR; pfm_ctrlloop++)
                                    {
                                       pfmbit_cmd_recvbuf[pfm_numbytes_recv + pfm_ctrlloop] = pfmbit_cmd_recvbufmod[pfm_ctrlloop];
                                    }
                                    mcs_pfmload_struct_relay = (struct MSC_MCS_PFM_LOAD_RELAY *)&pfmbit_cmd_recvbuf[pfm_numbytes_recv];                          
                                    usleep(SEC * HEX_ONE);  //taskDelay(sysClkRateGet());                                   
                                    res = pthread_create(&mcs_pfm_thread, NULL, mcs_pfmload, NULL);                         
                                    pthread_join(mcs_pfm_thread, NULL);   
                                                                                                                             
                            }
                            else
                            {
                                pfm_numbytes_recv = HEX_ZERO;
                                bzero(pfmbit_cmd_recvbufmod, sizeof(pfmbit_cmd_recvbufmod));
                                recv(pfmbit_sockfd, (int8_t *)&pfmbit_cmd_recvbufmod, DEC_408, HEX_ZERO);
                                for(pfm_ctrlloop = HEX_ZERO; pfm_ctrlloop < DEC_408; pfm_ctrlloop++)
                                {
                                    pfmbit_cmd_recvbuf[DEC_12 + pfm_ctrlloop] = pfmbit_cmd_recvbufmod[pfm_ctrlloop];
                                }
                                mcs_pfmload_struct_main = (struct MSC_MCS_PFM_LOAD_MAIN *)&pfmbit_cmd_recvbuf[ZERO];
                                pfm_numbytes_recv = pfm_numbytes_recv + DEC_420;
                                bzero(pfmbit_cmd_recvbufmod, sizeof(pfmbit_cmd_recvbufmod));
                                recv(pfmbit_sockfd, (int8_t *)&pfmbit_cmd_recvbufmod, (pfmbit_cmd_recvbuf[DEC_416] * HEX_EIGHT), HEX_ZERO);
                                for(pfm_ctrlloop = HEX_ZERO; pfm_ctrlloop < (pfmbit_cmd_recvbuf[DEC_416] * HEX_EIGHT); pfm_ctrlloop++)
                                {
                                    pfmbit_cmd_recvbuf[pfm_numbytes_recv + pfm_ctrlloop] = pfmbit_cmd_recvbufmod[pfm_ctrlloop];
                                }                                
                                mcs_pfmload_struct_blank = (struct MSC_MCS_PFM_LOAD_BLANK *)&pfmbit_cmd_recvbuf[pfm_numbytes_recv];
                                pfm_numbytes_recv = pfm_numbytes_recv + (pfmbit_cmd_recvbuf[DEC_416] * HEX_EIGHT);                                
                                bzero(pfmbit_cmd_recvbufmod, sizeof(pfmbit_cmd_recvbufmod));
                                recv(pfmbit_sockfd, (int8_t *)&pfmbit_cmd_recvbufmod, DEC_100, HEX_ZERO);
                                for(pfm_ctrlloop = HEX_ZERO; pfm_ctrlloop < DEC_100; pfm_ctrlloop++)
                                {
                                    pfmbit_cmd_recvbuf[pfm_numbytes_recv + pfm_ctrlloop] = pfmbit_cmd_recvbufmod[pfm_ctrlloop];
                                }                                
                                bzero(pfmbit_cmd_recvbufmod, sizeof(pfmbit_cmd_recvbufmod));
                                recv(pfmbit_sockfd, (int8_t *)&pfmbit_cmd_recvbufmod, (DEC_20 * pfmbit_cmd_recvbuf[pfm_numbytes_recv + DEC_96]), HEX_ZERO);
                                for(pfm_ctrlloop = HEX_ZERO; pfm_ctrlloop < (DEC_20 * pfmbit_cmd_recvbuf[pfm_numbytes_recv + DEC_96]); pfm_ctrlloop++)
                                {
                                    pfmbit_cmd_recvbuf[pfm_numbytes_recv + DEC_100 + pfm_ctrlloop] = pfmbit_cmd_recvbufmod[pfm_ctrlloop];
                                }                                
                                mcs_pfmload_struct_full = (struct MSC_MCS_PFM_LOAD_FULL *)&pfmbit_cmd_recvbuf[pfm_numbytes_recv]; 
                                pfm_numbytes_recv = pfm_numbytes_recv + DEC_100 + (DEC_20 * pfmbit_cmd_recvbuf[pfm_numbytes_recv + DEC_96]);                                
                                bzero(pfmbit_cmd_recvbufmod, sizeof(pfmbit_cmd_recvbufmod));
                                recv(pfmbit_sockfd, (int8_t *)&pfmbit_cmd_recvbufmod, HEX_FOUR, HEX_ZERO);
                                for(pfm_ctrlloop = HEX_ZERO; pfm_ctrlloop < HEX_FOUR; pfm_ctrlloop++)
                                {
                                    pfmbit_cmd_recvbuf[pfm_numbytes_recv + pfm_ctrlloop] = pfmbit_cmd_recvbufmod[pfm_ctrlloop];
                                }                                
                                mcs_pfmload_struct_relay = (struct MSC_MCS_PFM_LOAD_RELAY *)&pfmbit_cmd_recvbuf[pfm_numbytes_recv];                      
                                
                            } 
                            break;
                        }
                case SIXTEEN:
                        {
                            /* State Change */  
                            bzero(pfmbit_cmd_recvbufmod, sizeof(pfmbit_cmd_recvbufmod));                              
                            recv(pfmbit_sockfd, (int8_t *)&pfmbit_cmd_recvbufmod, HEX_FOUR, HEX_ZERO);                              
                            for(pfm_ctrlloop = HEX_ZERO; pfm_ctrlloop < HEX_FOUR; pfm_ctrlloop ++)
                            {
                                pfmbit_cmd_recvbuf[DEC_12 + pfm_ctrlloop] = pfmbit_cmd_recvbufmod[pfm_ctrlloop];
                            }
                            msc_mcs_vuhf_state_change_p = (struct MSC_MCS_VUHF_STATE_CHANGE *)&pfmbit_cmd_recvbuf[ZERO];                                                                           
                            if(msc_mcs_vuhf_state_change_p->vuhf_state == HEX_FIVE)
                            {
                                mcs_present_stateload = HEX_FIVE;
                                bzero((int8_t*)&mcs_state_change_ack, sizeof(mcs_state_change_ack));                                  
                                mcs_state_change_ack.message_code    = MCS_MSC_V_UHF_STATE_CHANGE_ACK;
                                mcs_state_change_ack.message_size    = sizeof(mcs_state_change_ack) - HEX_FOUR;                                   
                                mcs_state_change_ack.time_sec        = time_send.sec; 
                                mcs_state_change_ack.time_fracsec    = time_send.fracsec; 
                                mcs_state_change_ack.vuhf_change_ack = HEX_0xFE;                        
                                bzero((int8_t*)&mcs_state_change_ack, sizeof(mcs_state_change_ack));
                                mcs_state_change_ack.message_size    = MCS_MSC_V_UHF_STATE_CHANGE_ACK;
                                mcs_state_change_ack.message_size    = sizeof(mcs_state_change_ack) - HEX_FOUR;
                                mcs_state_change_ack.time_sec        = time_send.sec; 
                                mcs_state_change_ack.time_fracsec    = time_send.fracsec; 
                                mcs_state_change_ack.vuhf_change_ack = HEX_FF;                               
                                close(pfmbit_sockfd);
                                close(ctrl_msc_sockfd);
                                close(ctrl_mcs_sockfd);
                                pfmbit_sockfd   = ERROR;
                                ctrl_msc_sockfd = ERROR;
                                ctrl_mcs_sockfd = ERROR;                                   
                            }
                            else
                            {                                                                                                       
                                if(mcs_state_change_thread == HEX_ZERO)
                                {
                                    mcs_present_stateload = msc_mcs_vuhf_state_change_p->vuhf_state;
                                    res = pthread_create(&mcs_state_change_thread, NULL, mcs_state_change, NULL);                             
                                    pthread_join(mcs_state_change_thread, NULL); 
                                }
                                else
                                {
                                    ;
                                }                                  
                            }                              
                              usleep(DEC_10); //taskDelay(SysClkRateGet());                                                    
                              break;                        
                          }
                    default:
                        { 
                             
                            break;
                        }            
            } //end of switch
        }
           
    }  
    pfmbit_thread = HEX_ZERO;
    return (OK);
}


/** FHdr-beg *****************************************************
**
** Function name: mcs_pfmload
**
** Anchor:        RCS_SOW_REQ002
**
** Purpose:       function perform MCS PFM Load  
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

void *mcs_pfmload()
{
    //int8_t PFM_TaskNameSet[20] = "";
    uint32_t radiopfm_freqvalue[TOTAL_RADIOS_PLUS_ONE];
	uint32_t blankfreq_validation 			= HEX_ZERO;
    uint8_t  radiopfm_ctrlloop 				= HEX_ZERO; 
	uint8_t pfm_radio_id 					= HEX_ZERO; 
	uint8_t pfmvalidation_fail 				= HEX_ZERO; 
	uint8_t pfm_ifstatementcheck 			= HEX_ZERO;
    uint8_t  pfmradio_numbers[TOTAL_RADIOS_PLUS_ONE] = "";
    uint8_t  radiopfm_hexfreq[FOUR]			 = ""; 
	uint8_t radiopfm_los_param[TOTAL_RADIOS_PLUS_ONE][TEN]; 
	uint8_t radiopfm_833_freqstatus[TOTAL_RADIOS_PLUS_ONE] = "";
    uint8_t  radio_id_buf[SIX] 				= {ZERO,ONE,TWO,THREE,FOUR,FIVE};   // not in original code
    uint8_t  res 							= HEX_ZERO;                         // not in original code  
    bzero((int8_t *)&mcs_pfmload_ack, sizeof(mcs_pfmload_ack));    
    mcs_pfmload_ack.message_code = MCS_MSC_PFM_LOAD_ACK;
    mcs_pfmload_ack.message_size = sizeof(mcs_pfmload_ack) - HEX_FOUR;
    mcs_pfmload_ack.time_sec = time_send.sec; 
    mcs_pfmload_ack.time_fracsec = time_send.fracsec; 
    mcs_pfmload_ack.pfm_load_ack = HEX_0xFE;   
    pfm_ifstatementcheck = HEX_ONE;   
    if(pfm_ifstatementcheck == HEX_ONE && /* taskIdVerify(AmsMBitTId) == ERROR && taskIdVerify(McsMBitId) == ERROR &&*/ ap_erase_thread == 0)
    {
        bzero(pfmradio_numbers, sizeof(pfmradio_numbers));
        bzero(radiopfm_securemode, sizeof(radiopfm_securemode));
        bzero(radiopfm_guardchannel, sizeof(radiopfm_guardchannel));
        bzero((int8_t *)&radiopfm_los_param, sizeof(radiopfm_los_param));
        bzero((int8_t *)&radiopfm_freqvalue, sizeof(radiopfm_freqvalue));        
        pfmvalidation_fail = HEX_ZERO;        
        for(radiopfm_ctrlloop = HEX_ZERO; radiopfm_ctrlloop < mcs_pfmload_struct_full->num_radios; radiopfm_ctrlloop++)
        {
            pfm_radio_id = mcs_pfmload_struct_full->pfm_radio_params[radiopfm_ctrlloop].radio_id;
            
            pfmradio_numbers[pfm_radio_id] 			= HEX_ONE;
            radiopfm_los_param[pfm_radio_id][ZERO]  = HEX_ZERO;
            /*** allocation and Guard Channel ***/            
            if(mcs_pfmload_struct_full->pfm_radio_params[radiopfm_ctrlloop].allocation == HEX_ONE)
            {
                radiopfm_los_param[pfm_radio_id][ONE] = HEX_ZERO;
                radiopfm_guardchannel[pfm_radio_id]   = HEX_ZERO;                
            }
            else if(mcs_pfmload_struct_full->pfm_radio_params[radiopfm_ctrlloop].allocation == HEX_TWO)
            {
                
                radiopfm_los_param[pfm_radio_id][ONE] = HEX_TWO;
                if(mcs_pfmload_struct_full->pfm_radio_params[radiopfm_ctrlloop].channel_guard == HEX_ONE)
                {
                    radiopfm_guardchannel[pfm_radio_id] = HEX_ONE;
                }
                else if(mcs_pfmload_struct_full->pfm_radio_params[radiopfm_ctrlloop].channel_guard == HEX_TWO)
                {
                    radiopfm_guardchannel[pfm_radio_id] = HEX_TWO;
                }
                else if(mcs_pfmload_struct_full->pfm_radio_params[radiopfm_ctrlloop].channel_guard == HEX_THREE)
                {
                    radiopfm_guardchannel[pfm_radio_id] = HEX_THREE;
                }
                else
                {                   
                    pfmvalidation_fail = HEX_ONE;
                }
            }
            else
            {                
                pfmvalidation_fail = HEX_ONE;
            }
            
            /*** mode * Frequency * tx_power ***/
            radiopfm_833_freqstatus[pfm_radio_id] = frequency_roundof(mcs_pfmload_struct_full->pfm_radio_params[radiopfm_ctrlloop].freq_main,
                                                                   &radiopfm_freqvalue[pfm_radio_id]);
                                                                   
            if(mcs_pfmload_struct_full->pfm_radio_params[radiopfm_ctrlloop].mode == HEX_ONE && 
              ((radiopfm_freqvalue[pfm_radio_id] >= DEC_108000 && radiopfm_freqvalue[pfm_radio_id] <= DEC_155975) || 
               (radiopfm_freqvalue[pfm_radio_id] >= DEC_225000 && radiopfm_freqvalue[pfm_radio_id] <= DEC_399975)))
            {
                radiopfm_los_param[pfm_radio_id][TWO]   = HEX_ZERO;
                dec_to_hex(radiopfm_freqvalue[pfm_radio_id], radiopfm_hexfreq);
                
                radiopfm_los_param[pfm_radio_id][THREE] = *(radiopfm_hexfreq);
                radiopfm_los_param[pfm_radio_id][FOUR]  = *(radiopfm_hexfreq + HEX_ONE);
                radiopfm_los_param[pfm_radio_id][FIVE]  = *(radiopfm_hexfreq + HEX_TWO);
                radiopfm_los_param[pfm_radio_id][SIX]   = *(radiopfm_hexfreq + HEX_THREE);                                
                radiopfm_los_param[pfm_radio_id][SEVEN] = HEX_FOUR;
            }
            else if(mcs_pfmload_struct_full->pfm_radio_params[radiopfm_ctrlloop].mode == HEX_TWO && radiopfm_833_freqstatus[pfm_radio_id] == HEX_ZERO &&
                   ((radiopfm_freqvalue[pfm_radio_id] >= DEC_130000 && radiopfm_freqvalue[pfm_radio_id] <= DEC_173975) || 
                    (radiopfm_freqvalue[pfm_radio_id] >= DEC_225000 && radiopfm_freqvalue[pfm_radio_id] <= DEC_399975)))
            {                
                radiopfm_los_param[pfm_radio_id][TWO]   = HEX_ONE;
                dec_to_hex(radiopfm_freqvalue[pfm_radio_id], radiopfm_hexfreq);                
                radiopfm_los_param[pfm_radio_id][THREE] = *(radiopfm_hexfreq);
                radiopfm_los_param[pfm_radio_id][FOUR]  = *(radiopfm_hexfreq + HEX_ONE);
                radiopfm_los_param[pfm_radio_id][FIVE]  = *(radiopfm_hexfreq + HEX_TWO);
                radiopfm_los_param[pfm_radio_id][SIX]   = *(radiopfm_hexfreq + HEX_THREE);
                radiopfm_los_param[pfm_radio_id][SEVEN] = HEX_ONE;
            }
            else if(mcs_pfmload_struct_full->pfm_radio_params[radiopfm_ctrlloop].mode == HEX_THREE && radiopfm_833_freqstatus[pfm_radio_id] == HEX_ZERO &&
                   ((radiopfm_freqvalue[pfm_radio_id] >= DEC_108000 && radiopfm_freqvalue[pfm_radio_id] <= DEC_173975) || 
                    (radiopfm_freqvalue[pfm_radio_id] >= DEC_225000 && radiopfm_freqvalue[pfm_radio_id] <= DEC_399975)))
            {
                radiopfm_securemode[pfm_radio_id]       = HEX_ONE;
                radiopfm_los_param[pfm_radio_id][TWO]   = HEX_ZERO;
                dec_to_hex(radiopfm_freqvalue[pfm_radio_id], radiopfm_hexfreq);                
                radiopfm_los_param[pfm_radio_id][THREE] = *(radiopfm_hexfreq);
                radiopfm_los_param[pfm_radio_id][FOUR]  = *(radiopfm_hexfreq + HEX_ONE);
                radiopfm_los_param[pfm_radio_id][FIVE]  = *(radiopfm_hexfreq + HEX_TWO);
                radiopfm_los_param[pfm_radio_id][SIX]   = *(radiopfm_hexfreq + HEX_THREE);
                radiopfm_los_param[pfm_radio_id][SEVEN] = HEX_FOUR;
            }
            else if(mcs_pfmload_struct_full->pfm_radio_params[radiopfm_ctrlloop].mode == HEX_FOUR && radiopfm_833_freqstatus[pfm_radio_id] == HEX_ZERO &&
                   ((radiopfm_freqvalue[pfm_radio_id] >= DEC_108000 && radiopfm_freqvalue[pfm_radio_id] <= DEC_173975) || 
                    (radiopfm_freqvalue[pfm_radio_id] >= DEC_225000 && radiopfm_freqvalue[pfm_radio_id] <= DEC_399975)))
            {
                radiopfm_securemode[pfm_radio_id]         = HEX_TWO;
                radiopfm_los_param[pfm_radio_id][HEX_TWO] = HEX_ZERO;
                dec_to_hex(radiopfm_freqvalue[pfm_radio_id], radiopfm_hexfreq);
                
                radiopfm_los_param[pfm_radio_id][THREE]   = *(radiopfm_hexfreq);
                radiopfm_los_param[pfm_radio_id][FOUR]    = *(radiopfm_hexfreq + HEX_ONE);
                radiopfm_los_param[pfm_radio_id][FIVE]    = *(radiopfm_hexfreq + HEX_TWO);
                radiopfm_los_param[pfm_radio_id][SIX]     = *(radiopfm_hexfreq + HEX_THREE);                
                radiopfm_los_param[pfm_radio_id][SEVEN] = HEX_FOUR;
            }
            else
            {               
                pfmvalidation_fail = HEX_ONE;
            }
            /*** sub_state ***/
            if(mcs_pfmload_struct_full->pfm_radio_params[radiopfm_ctrlloop].sub_state == HEX_ONE)
            {
                radiopfm_los_param[pfm_radio_id][EIGHT] = HEX_ZERO;
            }
            else if(mcs_pfmload_struct_full->pfm_radio_params[radiopfm_ctrlloop].sub_state == HEX_TWO)
            {
                radiopfm_los_param[pfm_radio_id][EIGHT] = HEX_ONE;
            }
            else
            {               
                pfmvalidation_fail = HEX_ONE;
            }
            /*** Squelch ***/
            if(mcs_pfmload_struct_full->pfm_radio_params[radiopfm_ctrlloop].squelch >= HEX_ZERO && 
               mcs_pfmload_struct_full->pfm_radio_params[radiopfm_ctrlloop].squelch <= DEC_45)
            {
                radiopfm_los_param[pfm_radio_id][NINE] = mcs_pfmload_struct_full->pfm_radio_params[radiopfm_ctrlloop].squelch;
            }
            else
            {              
                pfmvalidation_fail = HEX_ONE;
            }
        }  /* end of for loop */
        
        bzero(relaypairconfig_pfmstore, sizeof(relaypairconfig_pfmstore));        
        for(pfm_radio_id = HEX_ZERO; pfm_radio_id < ((TOTAL_RADIOS / HEX_TWO) * HEX_TWO); pfm_radio_id++)
        {
            relaypairconfig_pfmstore[pfm_radio_id] = mcs_pfmload_struct_relay->relay_tr[pfm_radio_id];
            
        }
        pfm_ifstatementcheck = HEX_ONE;       
        for(pfm_radio_id = HEX_ZERO; pfm_radio_id < HEX_ONE /* ((TOTAL_RADIOS / 2) * 2) */; pfm_radio_id++)
        {
            if(relaypairconfig_pfmstore[pfm_radio_id] <= TOTAL_RADIOS && relaypairconfig_pfmstore[pfm_radio_id] >= HEX_ONE)
            {
                for(radiopfm_ctrlloop = pfm_radio_id + HEX_ONE; radiopfm_ctrlloop < HEX_ONE /* ((TOTAL_RADIOS / 2) * 2)*/; radiopfm_ctrlloop++)
                {
                    if(relaypairconfig_pfmstore[pfm_radio_id] != relaypairconfig_pfmstore[pfm_radio_id + radiopfm_ctrlloop])
                    {
                        pfm_ifstatementcheck = pfm_ifstatementcheck & HEX_ONE;
                    }
                    else
                    {
                        pfm_ifstatementcheck = pfm_ifstatementcheck & HEX_ZERO;
                    }
                }
            }
            else
            {
                pfm_ifstatementcheck = pfm_ifstatementcheck & HEX_ZERO;
            }
        }
        
        if(pfm_ifstatementcheck != HEX_ONE)
        {           
            pfmvalidation_fail = HEX_ONE;
        }        
        for(radiopfm_ctrlloop = HEX_ZERO; radiopfm_ctrlloop < DEC_100; radiopfm_ctrlloop++)
        {
            radiopfm_833_freqstatus[ZERO] = frequency_roundof(mcs_pfmload_struct_main->freq_preset_mem.freq_preset[radiopfm_ctrlloop], &radiopfm_freqvalue[ZERO]);
            if(radiopfm_freqvalue[ZERO] <= DEC_108000 || radiopfm_freqvalue[ZERO] >= DEC_399975)
            {               
                pfmvalidation_fail = HEX_ONE;
            }        
        }
        for(radiopfm_ctrlloop = HEX_TWO; radiopfm_ctrlloop < (mcs_pfmload_struct_main->num_blank_freqs * HEX_TWO) + HEX_TWO; radiopfm_ctrlloop++)
        {
            radiopfm_833_freqstatus[ZERO] = frequency_roundof(mcs_pfmload_struct_blank->blank_freq_preset[radiopfm_ctrlloop - HEX_TWO], &radiopfm_freqvalue[ZERO]);
            if(radiopfm_freqvalue[ZERO] <= DEC_108000 || radiopfm_freqvalue[ZERO] >= DEC_399975)
            {               
                pfmvalidation_fail = HEX_ONE;
                if(radiopfm_ctrlloop % HEX_TWO == HEX_ZERO)
                {
                    radiopfm_ctrlloop++;
                }
            }
            else
            {
                if(radiopfm_ctrlloop % HEX_TWO != HEX_ZERO)
                {
                    for(pfm_radio_id = HEX_ONE; pfm_radio_id <= HEX_ONE/*TOTAL_RADIOS*/; pfm_radio_id++)
                    {
                        if(radiopfm_freqvalue[pfm_radio_id] >= blankfreq_validation && radiopfm_freqvalue[pfm_radio_id] <= radiopfm_freqvalue[ZERO])
                        {                           
                            pfmvalidation_fail = HEX_ONE;
                        }                    
                    }
                }
                else
                {
                    blankfreq_validation = radiopfm_freqvalue[ZERO];
                }
            }        
        }
        for(radiopfm_ctrlloop = HEX_ZERO; radiopfm_ctrlloop < HEX_SIX; radiopfm_ctrlloop++)
        {
            radiopfm_833_freqstatus[ZERO] = frequency_roundof(mcs_pfmload_struct_full->fha_freq[radiopfm_ctrlloop], &radiopfm_freqvalue[ZERO]);
            if(radiopfm_freqvalue[ZERO] <= DEC_225000 || radiopfm_freqvalue[ZERO] >= DEC_399975)
            {               
                pfmvalidation_fail = HEX_ONE;
            }
        }
        
        if(pfmvalidation_fail == HEX_ZERO)
        {           
            mcspfm_underprogess = HEX_ONE;
            memset(radiopfm_underprogress, HEX_ONE, sizeof(radiopfm_underprogress));    
            numof_blankfreq = HEX_ZERO;
            bzero(freqchan_buf, sizeof(freqchan_buf));
            bzero(blankfreq_chanbuf, sizeof(blankfreq_chanbuf));            
            for(radiopfm_ctrlloop = HEX_ZERO; radiopfm_ctrlloop < DEC_100; radiopfm_ctrlloop++)
            {
                radiopfm_833_freqstatus[ZERO] = frequency_roundof(mcs_pfmload_struct_main->freq_preset_mem.freq_preset[radiopfm_ctrlloop], &radiopfm_freqvalue[ZERO]);
                freqchan_buf[radiopfm_ctrlloop] = radiopfm_freqvalue[ZERO];  //Preset Frequency
            }            
            numof_blankfreq = mcs_pfmload_struct_main->num_blank_freqs;
            for(radiopfm_ctrlloop = HEX_ZERO; radiopfm_ctrlloop < (numof_blankfreq * HEX_TWO); radiopfm_ctrlloop++)
            {
                radiopfm_833_freqstatus[ZERO] = frequency_roundof(mcs_pfmload_struct_blank->blank_freq_preset[radiopfm_ctrlloop], &radiopfm_freqvalue[ZERO]);
                blankfreq_chanbuf[radiopfm_ctrlloop] = radiopfm_freqvalue[ZERO];  // Blank Frequency
            }            
            bzero(hq_param_fha, sizeof(hq_param_fha));
            bzero(rpw_param_fhb, sizeof(rpw_param_fhb));            
            dec_to_hex(mcs_pfmload_struct_full->fha_index * DEC_1000, radiopfm_hexfreq);            
            hq_param_fha[ZERO]   = *(radiopfm_hexfreq);
            hq_param_fha[ONE]    = *(radiopfm_hexfreq + HEX_ONE);
            hq_param_fha[TWO]    = *(radiopfm_hexfreq + HEX_TWO);
            hq_param_fha[THREE]  = *(radiopfm_hexfreq + HEX_THREE);            
            radiopfm_833_freqstatus[ZERO] = frequency_roundof(mcs_pfmload_struct_full->fha_freq[ZERO], &radiopfm_freqvalue[ZERO]);
            dec_to_hex(radiopfm_freqvalue[ZERO], radiopfm_hexfreq);            
            hq_param_fha[FOUR]  = *(radiopfm_hexfreq);
            hq_param_fha[FIVE]  = *(radiopfm_hexfreq + HEX_ONE);
            hq_param_fha[SIX]   = *(radiopfm_hexfreq + HEX_TWO);
            hq_param_fha[SEVEN] = *(radiopfm_hexfreq + HEX_THREE);            
            radiopfm_833_freqstatus[ZERO] = frequency_roundof(mcs_pfmload_struct_full->fha_freq[ONE], &radiopfm_freqvalue[ZERO]);
            dec_to_hex(radiopfm_freqvalue[ZERO], radiopfm_hexfreq);            
            hq_param_fha[EIGHT]  = *(radiopfm_hexfreq);
            hq_param_fha[NINE]   = *(radiopfm_hexfreq + HEX_ONE);
            hq_param_fha[TEN]    = *(radiopfm_hexfreq + HEX_TWO);
            hq_param_fha[ELEVEN] = *(radiopfm_hexfreq + HEX_THREE);            
            radiopfm_833_freqstatus[ZERO] = frequency_roundof(mcs_pfmload_struct_full->fha_freq[TWO], &radiopfm_freqvalue[ZERO]);
            dec_to_hex(radiopfm_freqvalue[ZERO], radiopfm_hexfreq);            
            hq_param_fha[TWELVE]    = *(radiopfm_hexfreq);
            hq_param_fha[THIRTEEN]  = *(radiopfm_hexfreq + HEX_ONE);
            hq_param_fha[FOURTEEN]  = *(radiopfm_hexfreq + HEX_TWO);
            hq_param_fha[FIFTEEN]   = *(radiopfm_hexfreq + HEX_THREE);            
            radiopfm_833_freqstatus[ZERO] = frequency_roundof(mcs_pfmload_struct_full->fha_freq[THREE], &radiopfm_freqvalue[ZERO]);
            dec_to_hex(radiopfm_freqvalue[ZERO], radiopfm_hexfreq);            
            hq_param_fha[SIXTEEN]    = *(radiopfm_hexfreq);
            hq_param_fha[SIXTEEN]    = *(radiopfm_hexfreq + HEX_ONE);
            hq_param_fha[EIGHTEEN]   = *(radiopfm_hexfreq + HEX_TWO);
            hq_param_fha[NINETEEN]   = *(radiopfm_hexfreq + HEX_THREE);            
            radiopfm_833_freqstatus[ZERO] = frequency_roundof(mcs_pfmload_struct_full->fha_freq[FOUR], &radiopfm_freqvalue[ZERO]);
            dec_to_hex(radiopfm_freqvalue[ZERO], radiopfm_hexfreq);            
            hq_param_fha[TWENTY]      = *(radiopfm_hexfreq);
            hq_param_fha[TWENTYONE]   = *(radiopfm_hexfreq + HEX_ONE);
            hq_param_fha[TWENTYTWO]   = *(radiopfm_hexfreq + HEX_TWO);
            hq_param_fha[TWENTYTHREE] = *(radiopfm_hexfreq + HEX_THREE);            
            radiopfm_833_freqstatus[ZERO] = frequency_roundof(mcs_pfmload_struct_full->fha_freq[FIVE], &radiopfm_freqvalue[ZERO]);
            dec_to_hex(radiopfm_freqvalue[ZERO], radiopfm_hexfreq);            
            hq_param_fha[TWENTYFOUR]   = *(radiopfm_hexfreq);
            hq_param_fha[TWENTYFIVE]   = *(radiopfm_hexfreq + HEX_ONE);
            hq_param_fha[TWENTYFIVE]   = *(radiopfm_hexfreq + HEX_TWO);
            hq_param_fha[TWENTYSEVEN]  = *(radiopfm_hexfreq + HEX_THREE);            
            hq_param_fha[TWENTYEIGHT]  = mcs_pfmload_struct_full->fha_mwod;            
            rpw_param_fhb[ZERO]        = mcs_pfmload_struct_full->fhb_index;
            rpw_param_fhb[ONE]         = mcs_pfmload_struct_full->fhb_hopset_position;
            rpw_param_fhb[TWO]         = mcs_pfmload_struct_full->fhb_num_cells;            
            for(radiopfm_ctrlloop = HEX_ZERO; radiopfm_ctrlloop < DEC_64; radiopfm_ctrlloop++)
            {
                rpw_param_fhb[radiopfm_ctrlloop + HEX_THREE] = mcs_pfmload_struct_full->fhb_cell[radiopfm_ctrlloop];
            }                        
            // Default all buffers before PFM Loading Except Radiation Status
            for(pfm_radio_id = HEX_ONE; pfm_radio_id <= HEX_ONE/*TOTAL_RADIOS*/; pfm_radio_id++)
            {
                if(pfmradio_numbers[pfm_radio_id] == HEX_ONE)
                {
                    for(radiopfm_ctrlloop = HEX_ZERO; radiopfm_ctrlloop < DEC_10; radiopfm_ctrlloop++)
                    {
                        radiostored_los_param[pfm_radio_id][radiopfm_ctrlloop] = radiopfm_los_param[pfm_radio_id][radiopfm_ctrlloop];                      
                    }                    
                    radio_loadedfreq[pfm_radio_id]           = radiopfm_freqvalue[pfm_radio_id];
                    radio_securemode[pfm_radio_id]           = radiopfm_securemode[pfm_radio_id];
                    radio_guardchannel[pfm_radio_id]         = radiopfm_guardchannel[pfm_radio_id];                                        
                    radioloaded_833_freqstatus[pfm_radio_id] = radiopfm_833_freqstatus[pfm_radio_id];
                    is_radio_firstpfm_present[pfm_radio_id]  = HEX_ONE;
                    hq_fha_guardcontrol[pfm_radio_id]        = radiostored_los_param[pfm_radio_id][EIGHT];
                }
            }           
            for(pfm_radio_id = HEX_ONE; pfm_radio_id <= HEX_ONE/*TOTAL_RADIOS*/; pfm_radio_id++)
            {
                if(radio_present_state[pfm_radio_id] == HEX_TWO || radio_present_state[pfm_radio_id] == HEX_THREE || radio_present_state[pfm_radio_id] == HEX_ONE)
                {
                    radiopfm_underprogress[pfm_radio_id] = HEX_ZERO;
                    pthread_create(&radio_pfmload_thread[pfm_radio_id], NULL, radio_pfmload, &radio_id_buf[pfm_radio_id]);                 
             
                }
            }
            /* wait till function get over */ 
            for(pfm_radio_id = HEX_ONE; pfm_radio_id <= HEX_ONE /*TOTAL_RADIOS*/; pfm_radio_id++)
            {                
                pthread_join(radio_pfmload_thread[pfm_radio_id], NULL);
            }           
                       
            mcspfm_underprogess = HEX_ZERO;
            usleep(SEC * HEX_TWO);             //taskDelay(sysClkRateGet() * 2);            
            mcs_pfmload_ack.pfm_load_ack = HEX_FF;
            
           
        }
        else
        {
            printf("PFM Validation Fail\n\r");
            mcs_pfmload_ack.pfm_load_ack = HEX_ZERO;
        }
    }
    else
    {       
        pfm_ifstatementcheck 		 = HEX_ZERO;        
        mcs_pfmload_ack.pfm_load_ack = HEX_ZERO;
    }
    mcs_pfmload_ack.message_code = MCS_MSC_PFM_LOAD_ACK;
    mcs_pfmload_ack.message_size = sizeof(mcs_pfmload_ack) - HEX_FOUR;   
    mcs_pfmload_ack.time_sec     = time_send.sec; 
    mcs_pfmload_ack.time_fracsec = time_send.fracsec;   
    is_firstpfm_loaded = HEX_ONE;
    if(mcs_pfmload_ack.pfm_load_ack == HEX_FF)
    {
        usleep(MILISEC * DEC_10);  // taskDelay(10)
        send_ap_status();
    } 
    mcs_pfm_thread = HEX_ZERO;    
}   


/** FHdr-beg *****************************************************
**
** Function name: radio_pfmload
**
** Anchor:        RCS_SOW_REQ003
**
** Purpose:       function perform radio pfm load  
**				  
**				  
**							   
** Inputs:        radio pfm laod
**
**
** Outputs:       radio pfm laod status      
**
** FPrj-beg *****************************************************
**
** FPrj-end *****************************************************
** FHdr-end *****************************************************/ 
void *radio_pfmload(void *radio_pfmload_arg)
{
    uint8_t radiopfm_hexfreq[FOUR]       = "";
    uint8_t radiopfm_reload              = HEX_ZERO;   
    uint8_t radio_pfmload_status         = HEX_ZERO;
    uint8_t temp_data_store              = HEX_ZERO;
    uint8_t radiostart_modestatus[TEN]   = "";
    uint8_t pfml_radio_id                = HEX_ZERO;     
    pfml_radio_id = *(uint8_t *)radio_pfmload_arg;    
    if(radio_present_state[pfml_radio_id] == HEX_ONE)
    {
        radiopfm_reload = HEX_ZERO;
        while(radiopfm_reload <= HEX_FOUR)
        {
            radio_param_setstatus[pfml_radio_id] = radiation_status(pfml_radio_id, radio_serialfd[pfml_radio_id]);        
            if(radio_param_setstatus[pfml_radio_id] == HEX_0XEE)
            {                
                if(radiopfm_reload == HEX_THREE)
                {
                    radio_radiation_status[pfml_radio_id] = radiostored_los_param[pfml_radio_id][ZERO] = HEX_ZERO;
                }
                radiopfm_reload++;
                usleep(SEC * HEX_ONE);    // taskDelay(sysClkRateGet());
            }
            else
            {                
                radiopfm_reload = LOOP_TERMINATE;
                radio_radiation_status[pfml_radio_id] = radiostored_los_param[pfml_radio_id][ZERO] = radio_param_setstatus[pfml_radio_id];
            }
        }       
        radio_ap_present[pfml_radio_id]             = HEX_ZERO;
        hq_fha_guardcontrol[pfml_radio_id]          = HEX_ZERO;
        radio_fh_notpossible[pfml_radio_id]         = HEX_ZERO;
        radio_emgactive_aftersecure[pfml_radio_id]  = HEX_ZERO;
        radio_cbitfirsttime_after_fh[pfml_radio_id] = HEX_ZERO;
        rpw_re_operation_complete[pfml_radio_id]    = HEX_ZERO;        
        radio_pfmload_status = HEX_FF;        
        if(radiostored_los_param[pfml_radio_id][ONE] == HEX_TWO)    /* Loading Guard Frequency */
        {         
            
            radiopfm_reload = HEX_ZERO;
            while(radiopfm_reload <= HEX_FOUR)  
            {
                temp_data_store = radiostored_los_param[pfml_radio_id][ZERO];
                radiostored_los_param[pfml_radio_id][ZERO] = HEX_ONE;                
                radio_param_setstatus[pfml_radio_id] = radiation_on_off(pfml_radio_id, radio_serialfd[pfml_radio_id], radiostored_los_param);                
                radio_param_setstatus[pfml_radio_id] = radiation_status(pfml_radio_id, radio_serialfd[pfml_radio_id]);                
                if(radio_param_setstatus[pfml_radio_id] == HEX_FF || radio_param_setstatus[pfml_radio_id] == HEX_ZERO)
                {                  
                    radiopfm_reload++;
                    usleep(SEC * HEX_ONE);    // taskDelay(sysClkRateGet());
                }
                else
                {                  
                    radiopfm_reload = LOOP_TERMINATE;
                }
                radiostored_los_param[pfml_radio_id][ZERO] = temp_data_store;
            }         
            radio_param_setstatus[pfml_radio_id] = activate_los_preset(pfml_radio_id, radio_serialfd[pfml_radio_id]);               
            usleep(MILISEC * DEC_30);    // taskDelay(30);             
            temp_data_store = radiostored_los_param[pfml_radio_id][ONE];
            radiostored_los_param[pfml_radio_id][ONE] = HEX_ZERO;            
            radio_param_setstatus[pfml_radio_id] = emergency_guard(pfml_radio_id, radio_serialfd[pfml_radio_id], radiostored_los_param);              
            radiostored_los_param[pfml_radio_id][ONE] = temp_data_store;   
            usleep(MILISEC * DEC_30);    // taskDelay(30);             
            if(radio_securemode[pfml_radio_id] != HEX_ZERO)
            {
                radio_securemode[pfml_radio_id] = HEX_ZERO;
            }
			else 
			{
					;
			}
            //Emg Guard            
            if(radio_guardchannel[pfml_radio_id] == HEX_THREE)              /* Maritime Freq */
            {    
                activate_setup(pfml_radio_id, radio_serialfd[pfml_radio_id], HEX_FIVE);
            }
            else
            {
                if(radio_guardchannel[pfml_radio_id] == HEX_TWO)
                {                    
                    dec_to_hex(UHFGUARD_RANFREQ, radiopfm_hexfreq);
                }
                else
                {                   
                    dec_to_hex(VHFGUARD_RANFREQ, radiopfm_hexfreq);
                }                
                radiostored_los_param[pfml_radio_id][THREE] = *(radiopfm_hexfreq);
                radiostored_los_param[pfml_radio_id][FOUR]  = *(radiopfm_hexfreq + HEX_ONE);
                radiostored_los_param[pfml_radio_id][FIVE]  = *(radiopfm_hexfreq + HEX_TWO);
                radiostored_los_param[pfml_radio_id][SIX]   = *(radiopfm_hexfreq + THREE);          
                skip_first_presetreq[pfml_radio_id]         = HEX_ONE;                
                store_setup_los(pfml_radio_id, radio_serialfd[pfml_radio_id], radiostored_los_param, storesetup_setstatus, 
                               radioloaded_833_freqstatus [pfml_radio_id]);                
                skip_first_presetreq[pfml_radio_id] = HEX_ZERO;                
                if(storesetup_setstatus[pfml_radio_id][ZERO] != SUCCESS_STATUS || storesetup_setstatus[pfml_radio_id][ONE] != SUCCESS_STATUS ||
                   storesetup_setstatus[pfml_radio_id][TWO] != SUCCESS_STATUS || storesetup_setstatus[pfml_radio_id][THREE] != SUCCESS_STATUS)
                {                   
                    radio_pfmload_status = HEX_ZERO;
                }
                else
                {
                    ;
                }                              
            }        
            radio_param_setstatus[pfml_radio_id] = emergency_guard(pfml_radio_id, radio_serialfd[pfml_radio_id], radiostored_los_param);
            if(radio_param_setstatus[pfml_radio_id] != SUCCESS_STATUS)
            {              
                radio_pfmload_status = HEX_ZERO;
            }       
            else
            {
               ;
            }         
        }  
        else                                                                 /* Loading Main Frequency */
        {
            if(radio_securemode[pfml_radio_id] == HEX_ZERO)
            {
                bzero((int8_t *)&radiostart_modestatus, sizeof(radiostart_modestatus));                
                radiostart_modestatus[ZERO]    = HEX_THREE;
                radiostart_modestatus[ONE]     = radiostored_los_param[pfml_radio_id][THREE];
                radiostart_modestatus[TWO]     = radiostored_los_param[pfml_radio_id][FOUR];
                radiostart_modestatus[THREE]   = radiostored_los_param[pfml_radio_id][FIVE];
                radiostart_modestatus[FOUR]    = radiostored_los_param[pfml_radio_id][SIX];                
                radiostart_modestatus[FIVE]    = radiostored_los_param[pfml_radio_id][EIGHT];             
                radio_param_setstatus[pfml_radio_id] = preset_request(pfml_radio_id, radio_serialfd[pfml_radio_id], radiostart_modestatus);                                 
                if(radio_param_setstatus[pfml_radio_id] != SUCCESS_STATUS)
                {                   
                    while(radiopfm_reload <= HEX_FOUR)  
                    {
                        temp_data_store = radiostored_los_param[pfml_radio_id][ZERO];
                        radiostored_los_param[pfml_radio_id][ZERO] = HEX_ONE;                
                        radio_param_setstatus[pfml_radio_id] = radiation_on_off(pfml_radio_id, radio_serialfd[pfml_radio_id], radiostored_los_param);
                        radio_param_setstatus[pfml_radio_id] = radiation_status(pfml_radio_id, radio_serialfd[pfml_radio_id]);                          
                        if(radio_param_setstatus[pfml_radio_id] == HEX_FF || radio_param_setstatus[pfml_radio_id] == HEX_ZERO)
                        {                           
                            radiopfm_reload++;
                            usleep(SEC * HEX_ONE);    // taskDelay(sysClkRateGet());                            
                        }
                        else
                        {                           
                            radiopfm_reload = LOOP_TERMINATE;
                        }
                        radiostored_los_param[pfml_radio_id][ZERO] = temp_data_store;
                    }        
                    radio_param_setstatus[pfml_radio_id] = activate_los_preset(pfml_radio_id, radio_serialfd[pfml_radio_id]);                     
                    usleep(MILISEC * DEC_30);     //taskDelay(30);                                               
                    temp_data_store = radiostored_los_param[pfml_radio_id][ONE];
                    radiostored_los_param[pfml_radio_id][ONE] = HEX_ZERO;            
                    radio_param_setstatus[pfml_radio_id] = emergency_guard(pfml_radio_id, radio_serialfd[pfml_radio_id], radiostored_los_param);                     
                    radiostored_los_param[pfml_radio_id][ONE] = temp_data_store;              
                    radio_param_setstatus[pfml_radio_id] = squelch_th(pfml_radio_id, radio_serialfd[pfml_radio_id], radiostored_los_param);
                    if(radio_param_setstatus[pfml_radio_id] != SUCCESS_STATUS)
                    {
                        radio_pfmload_status = HEX_ZERO;
                    } 
					else
					{
							;
                    }                    
                    skip_first_presetreq[pfml_radio_id] = HEX_ONE;                    
                    store_setup_los(pfml_radio_id, radio_serialfd[pfml_radio_id], radiostored_los_param, storesetup_setstatus, 
                                                                              radioloaded_833_freqstatus [pfml_radio_id]);                
                    skip_first_presetreq[pfml_radio_id] = HEX_ZERO;                
                    if(storesetup_setstatus[pfml_radio_id][ONE] != SUCCESS_STATUS || storesetup_setstatus[pfml_radio_id][ONE] != SUCCESS_STATUS ||
                       storesetup_setstatus[pfml_radio_id][TWO] != SUCCESS_STATUS || storesetup_setstatus[pfml_radio_id][THREE] != SUCCESS_STATUS)
                    {                       
                        radio_pfmload_status = HEX_ONE;
                    }
                    else
                    {
                        ;
                    }  
                                   
                } /* end of if(radio_param_setstatus[pfml_radio_id] != SUCCESS_STATUS) */ 
                else
                {                    
                    activate_setup(pfml_radio_id, radio_serialfd[pfml_radio_id], HEX_THREE);
                }
            } /* end of if(radio_securemode[pfml_radio_id] == 0) */
            else
            {
                radio_pfmload_status = HEX_ZERO;     
                radiopfm_reload = HEX_ZERO;
                while(radiopfm_reload <= HEX_FOUR)  
                {
                    temp_data_store = radiostored_los_param[pfml_radio_id][ZERO];
                    radiostored_los_param[pfml_radio_id][ZERO] = HEX_ONE;                
                    radio_param_setstatus[pfml_radio_id] = radiation_on_off(pfml_radio_id, radio_serialfd[pfml_radio_id], radiostored_los_param);                
                    radio_param_setstatus[pfml_radio_id] = radiation_status(pfml_radio_id, radio_serialfd[pfml_radio_id]);                
                    if(radio_param_setstatus[pfml_radio_id] == HEX_FF || radio_param_setstatus[pfml_radio_id] == HEX_ZERO)
                    {                       
                        radiopfm_reload++;
                        usleep(SEC * HEX_ONE);    // taskDelay(sysClkRateGet());
                    }
                    else
                    {                      
                        radiopfm_reload = LOOP_TERMINATE;
                    }
                    radiostored_los_param[pfml_radio_id][ZERO] = temp_data_store;
                } 
                radio_param_setstatus[pfml_radio_id] = activate_los_preset(pfml_radio_id, radio_serialfd[pfml_radio_id]);                  
                usleep(MILISEC * DEC_30);    // taskDelay(30);    
                temp_data_store = radiostored_los_param[pfml_radio_id][ONE];
                radiostored_los_param[pfml_radio_id][ONE] = HEX_ZERO;
                radio_param_setstatus[pfml_radio_id] = emergency_guard(pfml_radio_id, radio_serialfd[pfml_radio_id], radiostored_los_param);                 
                radiostored_los_param[pfml_radio_id][ONE] = temp_data_store;  
                usleep(MILISEC * DEC_30);    //taskDelay(30);         
                radio_param_setstatus[pfml_radio_id] = squelch_th(pfml_radio_id, radio_serialfd[pfml_radio_id], radiostored_los_param);
                if(radio_param_setstatus[pfml_radio_id] != SUCCESS_STATUS)
                {
                    radio_pfmload_status = HEX_ZERO;
                }
				else
				{
						;
				}
                usleep(MILISEC * DEC_30);    //taskDelay(30);             
                skip_first_presetreq[pfml_radio_id] = HEX_ONE;                
                store_setup_los(pfml_radio_id, radio_serialfd[pfml_radio_id], radiostored_los_param, storesetup_setstatus, 
                               radioloaded_833_freqstatus[pfml_radio_id]);                
                skip_first_presetreq[pfml_radio_id] = HEX_ZERO;                
                if(storesetup_setstatus[pfml_radio_id][ZERO] != SUCCESS_STATUS || storesetup_setstatus[pfml_radio_id][ONE] != SUCCESS_STATUS ||
                   storesetup_setstatus[pfml_radio_id][TWO] != SUCCESS_STATUS || storesetup_setstatus[pfml_radio_id][THREE] != SUCCESS_STATUS)
                {                  
                    radio_pfmload_status = HEX_ZERO;
                }
                else
                {
                   ; 
                }            
                radio_securemode[pfml_radio_id] = HEX_ZERO;
                radio_fh_notpossible[pfml_radio_id] = HEX_ONE;
            }
        } /* end of else of if(radiostored_los_param[pfml_radio_id][1] == 2) */        
        if(radio_fh_notpossible[pfml_radio_id] == HEX_ONE)
        {
            radio_standby_status[pfml_radio_id] = HEX_ONE;
        }
		else
		{
		   ; 
		}         
        /* TR Status */
        if(radio_fh_notpossible[pfml_radio_id] == HEX_ONE || mcs_present_stateload == HEX_TWO)
        {
            radiostored_los_param[pfml_radio_id][ZERO] = HEX_ONE;           
        }
		else
		{
		   ; 
		}         
        if(radiostored_los_param[pfml_radio_id][ZERO] == HEX_ONE)
        {
            radio_present_state[pfml_radio_id] = radio_present_statestore[pfml_radio_id] = HEX_TWO;
        }
        else
        {
            radio_present_state[pfml_radio_id] = radio_present_statestore[pfml_radio_id] = mcs_present_stateload;
        }        
        /** Radiation_on_off **/    
        radio_param_setstatus[pfml_radio_id] = radiation_on_off(pfml_radio_id, radio_serialfd[pfml_radio_id], radiostored_los_param);
        if(radio_param_setstatus[pfml_radio_id] != SUCCESS_STATUS)
        {
           ;
        }
        else
        {           
            radio_radiation_status[pfml_radio_id] = radiostored_los_param[pfml_radio_id][ZERO];
        }            
	     
    } /* end of if(radio_present_state[pfml_radio_id] == 1 || radio_present_state[pfml_radio_id] == 2 radio_present_state[pfml_radio_id] == 3) */
    else
    {
        ;
    }    
    if(radio_pfmload_status == HEX_ZERO || is_radio_firstpfm_present[pfml_radio_id] == HEX_ZERO)
    {
        send_alert_Msg(pfml_radio_id, PFM_NOT_LOADED_TO_RADIO);
    }
    usleep(MILISEC * DEC_30);    //taskDelay(30);    
    if(radio_fh_notpossible[pfml_radio_id] == HEX_ONE)
    {
        send_alert_Msg(pfml_radio_id, RADIO_NOT_AVAILABLE_FOR_FHA_FHB_MODE);
    }
	else
	{
	   ; 
	}     
    if(mcs_sent_status == HEX_ONE)
    {
        send_mcs_status();
    } 
	else
	{
	   ; 
	}     
    //Radio_PfmTId[pfml_radio_id] = ERROR;    
    //radio_pfmload_threadid[pfml_radio_id] = ERROR;  /* use pthread_exit */
    radio_pfmload_thread[pfml_radio_id] = HEX_ZERO;
    return(OK);
}


/** FHdr-beg *****************************************************
**
** Function name: mcs_state_change
**
** Anchor:        RCS_SOW_REQ003
**
** Purpose:       This function perform mcs state change  
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

void *mcs_state_change()
{
    int8_t  sc_tasknameset[TWENTY]		 = "";
    uint8_t sc_radio_id 				 = HEX_ZERO;
	uint8_t sc_ifstatementcheck 		 = HEX_ZERO;
    uint8_t radio_id_buf[SIX]			 = {HEX_ZERO,HEX_ONE,HEX_TWO,HEX_THREE,HEX_FOUR,HEX_FIVE};
    uint8_t res 						 = HEX_ZERO;
    bzero((int8_t*) &mcs_state_change_ack,sizeof(mcs_state_change_ack));
    mcs_state_change_ack.message_code    = MCS_MSC_V_UHF_STATE_CHANGE_ACK;  
    mcs_state_change_ack.message_size    = sizeof(mcs_state_change_ack)-HEX_FOUR;
    mcs_state_change_ack.time_sec        = time_send.sec;
    mcs_state_change_ack.time_fracsec    = time_send.fracsec;
    mcs_state_change_ack.vuhf_change_ack = HEX_0xFE;
    sc_ifstatementcheck					 = HEX_ONE;   
    // original if condition
    /*if(sc_ifstatementcheck == 1 && taskIdVerify(McsMbitTId) == ERROR && taskIdVerify(Mcs_PfmTId) == ERROR && 
       taskIdVerify(AP_EraseTId) == ERROR && (mcs_present_stateload ==2 || mcs_present_stateload == 3))*/
    if(sc_ifstatementcheck == HEX_ONE && /*taskIdVerify(McsMbitTId) == ERROR &&*/ mcs_pfm_thread == HEX_ZERO && 
       ap_erase_thread == HEX_ZERO && (mcs_present_stateload ==HEX_TWO || mcs_present_stateload == HEX_THREE))
    {
        
        for(sc_radio_id = HEX_ONE; sc_radio_id <= HEX_ONE/*TOTAL_RADIOS*/; sc_radio_id++)
        {
            res = pthread_create(&radio_state_change_thread[sc_radio_id], NULL, radio_state_change, &radio_id_buf[sc_radio_id]);
           
        }
        //while(TRUE)
		for(;;)
        {
            sc_ifstatementcheck = HEX_ONE;            
            for(sc_radio_id = HEX_ONE; sc_radio_id <= HEX_ONE /*TOTAL_RADIOS*/; sc_radio_id++)
            {
                //if(taskIdVerify(Radio_StateTId[sc_radio_id]) == ERROR)
                if(radio_state_change_thread[sc_radio_id] == HEX_ZERO)
                {
                    sc_ifstatementcheck = sc_ifstatementcheck & HEX_ONE;
                }
                else
                {
                    sc_ifstatementcheck = sc_ifstatementcheck & HEX_ZERO;
                }
            }            
            if(sc_ifstatementcheck == HEX_ONE)
            {
                send_mcs_status();
                mcs_state_change_ack.vuhf_change_ack = HEX_FF;
                break;
            }
            else
            {
                usleep(SEC * HEX_ONE);   //taskDelay(SysClkRateGet());
            }
        }
    }
    else
    {
        sc_ifstatementcheck = HEX_ZERO;
        
        for(sc_radio_id = HEX_ONE; sc_radio_id <= HEX_ONE/*TOTAL_RADIOS*/; sc_radio_id++)
        {
            //if(taskIdVerify(RadioMbitTId[sc_radio_id]) != ERROR)
            if(radio_state_change_thread[sc_radio_id] != HEX_ZERO)
            {
                sc_ifstatementcheck = sc_ifstatementcheck | HEX_ONE;
            }
            else
            {
                sc_ifstatementcheck = sc_ifstatementcheck | HEX_ZERO;
            }
        }        
        mcs_state_change_ack.vuhf_change_ack = HEX_ZERO;        
    }
    mcs_state_change_ack.message_code   = MCS_MSC_V_UHF_STATE_CHANGE_ACK;  
    mcs_state_change_ack.message_size   = sizeof(mcs_state_change_ack) - HEX_FOUR;
    mcs_state_change_ack.time_sec       = time_send.sec;
    mcs_state_change_ack.time_fracsec   = time_send.fracsec;
    mcs_state_change_thread = HEX_ZERO;
    return(OK);
}


/** FHdr-beg *****************************************************
**
** Function name: radio_state_change
**
** Anchor:        RCS_SOW_REQ003
**
** Purpose:       This function perform radio state change  
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

void *radio_state_change(void *arg)
{
    uint8_t state_ch_radio_id;
    state_ch_radio_id = *(uint8_t *)arg;    
    //semTake(Radio_SynSem[state_ch_radio_id], WAIT_FOREVER);
    sem_wait(&radio_syncsem[state_ch_radio_id]);
    if(radio_standby_status[state_ch_radio_id] == HEX_ZERO)
    {
        radio_present_stateload[state_ch_radio_id] = mcs_present_stateload;
        if(radio_present_state[state_ch_radio_id] == HEX_TWO || radio_present_state[state_ch_radio_id] == HEX_THREE)
        {
            if(radio_present_stateload[state_ch_radio_id] == HEX_THREE)
            {
                radiostored_los_param[state_ch_radio_id][ZERO] = HEX_ZERO;
            }
            else
            {
                radiostored_los_param[state_ch_radio_id][ZERO] = HEX_ONE;
            }
            radio_param_setstatus[state_ch_radio_id] = radiation_on_off(state_ch_radio_id, radio_serialfd[state_ch_radio_id], radiostored_los_param);            
            if(radio_param_setstatus[state_ch_radio_id] == SUCCESS_STATUS)
            {
                radio_present_state[state_ch_radio_id] = radio_present_statestore[state_ch_radio_id] = radio_present_stateload[state_ch_radio_id];
                radio_radiation_status[state_ch_radio_id] = radiostored_los_param[state_ch_radio_id][0];
            }
            else
            {
                ;
            }
        }
    }
    else
    {
        radio_present_stateload[state_ch_radio_id] = HEX_TWO;
    }
    //semGive(Radio_SynSem[state_ch_radio_id]);
    sem_post(&radio_syncsem[state_ch_radio_id]);  
    radio_state_change_thread[state_ch_radio_id] = HEX_ZERO;
    return(OK);
}




/** FHdr-beg *****************************************************
**
** Function name: control_ap_relay
**
** Anchor:        RCS_SOW_REQ002
**
** Purpose:       This function perform control ap relay  
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

void *control_ap_relay()
{
    
    int32_t ctrl_numbytesrecv 				= HEX_ZERO;
	int32_t ctrlcmdloop 					= HEX_ZERO;
    uint8_t ctap_radio_id 					= HEX_ZERO;
	uint8_t radio_config_validationalfail   = HEX_ZERO; 
    uint8_t ctrlcmd_recvbuf[DEC_100] 		= "";
	uint8_t ctrlcmd_recvbufmod[DEC_100] 	= "";
	uint8_t config_ack_status[DEC_10] 		= "";
	uint8_t radioctrl_cmd_hexfreq[FOUR]		= ""; 
    int32_t res 							= HEX_ZERO;    
    for( ctap_radio_id = HEX_ONE; ctap_radio_id <= TOTAL_RADIOS; ctap_radio_id++)
    {        
        radioctrl_thread[ctap_radio_id] = HEX_ZERO;
    } 
    //while(TRUE)
	for(;;)
    {
        ctrl_numbytesrecv = HEX_ZERO;
        bzero(ctrlcmd_recvbuf,sizeof(ctrlcmd_recvbuf));        
        if(ctrl_numbytesrecv = recv(ctrl_msc_sockfd, (int8_t *) &ctrlcmd_recvbuf, DEC_12, HEX_ZERO) <= HEX_ZERO)
        {           
            break;
        }
        else
        {            
            switch(ctrlcmd_recvbuf[ZERO])
            {
                case TWENTYONE:
                    {
                        bzero(ctrlcmd_recvbufmod, sizeof(ctrlcmd_recvbufmod));
                        recv(ctrl_msc_sockfd, (int8_t *)&ctrlcmd_recvbufmod, HEX_FOUR, HEX_ZERO);
						for(ctrlcmdloop = HEX_ZERO; ctrlcmdloop < HEX_FOUR; ctrlcmdloop++)
						{
						  ctrlcmd_recvbuf[DEC_12 + ctrlcmdloop] = ctrlcmd_recvbufmod[ctrlcmdloop];
						}
						mcs_ap_erase_p = (struct MCS_MSC_ADDITIONAL_PARAMS_ERASE *) &ctrlcmd_recvbuf[ZERO];     
					    if(ap_erase_thread == HEX_ZERO)
                        {
                            res = pthread_create(&ap_erase_thread, NULL, ap_erase, NULL);
                            
						}
						else
						{
							;
						}
                        break;
                    }
                case THIRTYTWO: /*  Radio Configuration  */
                    {
					    bzero(ctrlcmd_recvbufmod , sizeof(ctrlcmd_recvbufmod));
					    recv(ctrl_msc_sockfd ,(int8_t *) &ctrlcmd_recvbufmod, DEC_24, HEX_ZERO );
					    for(ctrlcmdloop = HEX_ZERO; ctrlcmdloop < DEC_24; ctrlcmdloop++)
					    {
						   ctrlcmd_recvbuf[DEC_12 + ctrlcmdloop] = ctrlcmd_recvbufmod[ctrlcmdloop];
					    }
					    mcs_radio_config_p = (struct MCS_MSC_RADIO_CONFIGURATION *)&ctrlcmd_recvbuf[ZERO];
						radio_config_validationalfail = HEX_ZERO;
					    config_ack_status[ZERO] = ctap_radio_id = mcs_radio_config_p->radio_id;
					    config_ack_status[ONE]  = config_ack_status[TWO] = config_ack_status[THREE] = config_ack_status[FOUR]  = HEX_ZERO;
					    config_ack_status[FIVE] = config_ack_status[SIX] = config_ack_status[SEVEN] = config_ack_status[EIGHT] = HEX_ZERO;                              
                        if(ctap_radio_id >= HEX_ONE && ctap_radio_id <= TOTAL_RADIOS)
                        {
                                  //if(taskIdVerify(RadioCtrlTId[ctap_radio_id]) ==  ERROR)
                            if(radioctrl_thread[ctap_radio_id] == HEX_ZERO)
                            {  
							    radioload_securemode[ctap_radio_id]   = HEX_ZERO;
							    radioctrl_freqvalue[ctap_radio_id]    = HEX_ZERO;
							    radioload_guardchannel[ctap_radio_id] = HEX_ZERO;
                                for(ctrlcmdloop = HEX_ZERO; ctrlcmdloop < DEC_10; ctrlcmdloop++)
                                {
                                    radioctrl_los_param[ctap_radio_id][ctrlcmdloop] = HEX_ZERO;
                                }
                                /*** Radiation ***/
                                if(mcs_radio_config_p->radiation_on_off == HEX_ZERO)           /* 0 - Radiation off */
                                {
                                    radioctrl_los_param[ctap_radio_id][ZERO] = HEX_ONE;           
                                }
                                else if(mcs_radio_config_p->radiation_on_off == HEX_ONE)      /* 1 - Radiation on */
                                {
                                    radioctrl_los_param[ctap_radio_id][ZERO] = HEX_ZERO; 
                                }
                                else
                                {
                                          
                                   radio_config_validationalfail = HEX_ONE;
                                }   
                                    /*** allocation ***/                                   
                                if(radio_config_validationalfail == HEX_ZERO)
                                {
                                    if(mcs_radio_config_p->allocation == HEX_ONE)          /* 1 - main freq */              
                                    {
                                        radioctrl_los_param[ctap_radio_id][ONE] = HEX_ZERO;
                                        radioload_guardchannel[ctap_radio_id] = HEX_ZERO;
                                    }
                                    else if(mcs_radio_config_p->allocation == HEX_TWO)     /* 2 - guard mode */
                                    {
                                        radioctrl_los_param[ctap_radio_id][ONE] = HEX_TWO;
                                        if(mcs_radio_config_p->channel_guard == HEX_ONE )
                                        {
                                            radioload_guardchannel[ctap_radio_id] = HEX_ONE;     /* store VHFGUARD_RANFREQ */
                                        }       
                                        else if(mcs_radio_config_p->channel_guard == HEX_TWO )
                                        {
                                            radioload_guardchannel[ctap_radio_id] = HEX_TWO;     /* store UHFGUARD_RANFREQ */
                                        }
                                        else if (mcs_radio_config_p->channel_guard == THREE)
                                        {
                                            radioload_guardchannel[ctap_radio_id] = HEX_THREE;    /* store Maritime Freq */
                                        } 
                                        else
                                        {
                                            radio_config_validationalfail = HEX_ONE;
                                        } 
                                    }
                                    else
                                    {                                              
                                        radio_config_validationalfail = HEX_ONE;
                                    }
                                }
                                     
                                      /*** Blank Frequency Check ***/                                      
                                radioctrl_833_freqstatus[ctap_radio_id] = frequency_roundof(mcs_radio_config_p->freq_main, 
                                                                                                &radioctrl_freqvalue[ctap_radio_id]);
                                                                                                                                      
                                if(radio_config_validationalfail == HEX_ZERO && (mcs_radio_config_p->mode == HEX_ONE || mcs_radio_config_p->mode == HEX_TWO) &&     						(radioctrl_freqvalue[ctap_radio_id] >= 108000 && radioctrl_freqvalue[ctap_radio_id] <= 399975))
                                {
                                    for(ctrlcmdloop = HEX_ZERO; ctrlcmdloop < numof_blankfreq; ctrlcmdloop = ctrlcmdloop + HEX_TWO)
                                    {
                                        if(radioctrl_freqvalue[ctap_radio_id] >= blankfreq_chanbuf[ctrlcmdloop] && 
                                                 radioctrl_freqvalue[ctap_radio_id] <= blankfreq_chanbuf[ctrlcmdloop + HEX_ONE])
            
                                        {                                                  
                                            radio_config_validationalfail = HEX_ONE;
                                            break;
                                        }
                                    }
                                }                                      
                                      /*** mode Frequency ** tx_power ***/
                                if(radio_config_validationalfail == HEX_ZERO)
                                {
									if(mcs_radio_config_p->mode == HEX_ZERO && (radioctrl_freqvalue[ctap_radio_id] >= DEC_108000 && 
                                             radioctrl_freqvalue[ctap_radio_id] <= DEC_155975) || (radioctrl_freqvalue[ctap_radio_id] >= DEC_225000 && 
                                             radioctrl_freqvalue[ctap_radio_id] <= DEC_399975))
                                    {
                                        radioctrl_los_param[ctap_radio_id][TWO] = HEX_ZERO;
                                        dec_to_hex(radioctrl_freqvalue[ctap_radio_id] ,radioctrl_cmd_hexfreq);                                               
                                        radioctrl_los_param[ctap_radio_id][THREE] = *(radioctrl_cmd_hexfreq );
                                        radioctrl_los_param[ctap_radio_id][FOUR]  = *(radioctrl_cmd_hexfreq + HEX_ONE);
                                        radioctrl_los_param[ctap_radio_id][FIVE]  = *(radioctrl_cmd_hexfreq + HEX_TWO);
                                        radioctrl_los_param[ctap_radio_id][SIX]   = *(radioctrl_cmd_hexfreq + THREE);
                                        radioctrl_los_param[ctap_radio_id][SEVEN] = HEX_FOUR;                                               
                                        radioload_securemode[ctap_radio_id]  = HEX_ZERO;
                                    }
                                    else if(mcs_radio_config_p->mode == HEX_TWO && ((radioctrl_freqvalue[ctap_radio_id] >= DEC_130000 && 
                                          radioctrl_freqvalue[ctap_radio_id] <= DEC_173975) || (radioctrl_freqvalue[ctap_radio_id] >= DEC_225000 && 
                                          radioctrl_freqvalue[ctap_radio_id] <= DEC_399975)) && radioctrl_833_freqstatus[ctap_radio_id] ==HEX_ZERO)
                                    {
                                        radioctrl_los_param[ctap_radio_id][TWO]     = HEX_ONE;                                              
                                        dec_to_hex(radioctrl_freqvalue[ctap_radio_id], radioctrl_cmd_hexfreq);                                              
                                        radioctrl_los_param[ctap_radio_id][THREE] = *(radioctrl_cmd_hexfreq);
                                        radioctrl_los_param[ctap_radio_id][FOUR]  = *(radioctrl_cmd_hexfreq + HEX_ONE);
                                        radioctrl_los_param[ctap_radio_id][FIVE]  = *(radioctrl_cmd_hexfreq + HEX_TWO);
                                        radioctrl_los_param[ctap_radio_id][SIX]   = *(radioctrl_cmd_hexfreq + HEX_THREE);
                                        radioctrl_los_param[ctap_radio_id][SEVEN] = HEX_ONE;                                               
                                        radioload_securemode[ctap_radio_id]       = HEX_ZERO;
                                    }
                                    else if(mcs_radio_config_p->mode == HEX_THREE)
                                    {
										radioload_securemode[ctap_radio_id]       = HEX_ONE;                                                
										radioctrl_los_param[ctap_radio_id][TWO]   = radiostored_los_param[ctap_radio_id][TWO];
										radioctrl_los_param[ctap_radio_id][THREE] = radiostored_los_param[ctap_radio_id][THREE]; 
										radioctrl_los_param[ctap_radio_id][FOUR]  = radiostored_los_param[ctap_radio_id][FOUR];    
										radioctrl_los_param[ctap_radio_id][FIVE]  = radiostored_los_param[ctap_radio_id][FIVE];    
										radioctrl_los_param[ctap_radio_id][SIX]   = radiostored_los_param[ctap_radio_id][SIX];
										radioctrl_los_param[ctap_radio_id][SEVEN] = radiostored_los_param[ctap_radio_id][SEVEN];
                                    }       
                                    else if(mcs_radio_config_p->mode == HEX_FOUR)
                                    {
										radioload_securemode[ctap_radio_id]       = HEX_TWO;                                                
										radioctrl_los_param[ctap_radio_id][TWO]   = radiostored_los_param[ctap_radio_id][TWO];
										radioctrl_los_param[ctap_radio_id][THREE] = radiostored_los_param[ctap_radio_id][THREE]; 
										radioctrl_los_param[ctap_radio_id][FOUR]  = radiostored_los_param[ctap_radio_id][FOUR];    
										radioctrl_los_param[ctap_radio_id][FIVE]  = radiostored_los_param[ctap_radio_id][FIVE];    
										radioctrl_los_param[ctap_radio_id][SIX]   = radiostored_los_param[ctap_radio_id][SIX];
										radioctrl_los_param[ctap_radio_id][SEVEN] = radiostored_los_param[ctap_radio_id][SEVEN];
                                    } 
                                    else
                                    {                                               //printf("\n $$---Config Failed ---4 --$$ \n");
                                               radio_config_validationalfail = HEX_ONE;
                                    }
                                }
                                      /*** sub_state ***/
                                if(radio_config_validationalfail == HEX_ZERO)
                                {
									if(mcs_radio_config_p->sub_state == HEX_ONE)                    /* 1 – T/R */
									{
										radioctrl_los_param[ctap_radio_id][EIGHT] = HEX_ZERO;
									}
									else if(mcs_radio_config_p->sub_state == HEX_TWO)               /* 2 – T/R+G */
									{
										  radioctrl_los_param[ctap_radio_id][EIGHT] = HEX_ONE;
									}
									else
									{										  
										  radio_config_validationalfail = HEX_ONE;
									}
                                }                                      /*** Squelch ***/   
                                if(radio_config_validationalfail == HEX_ZERO)
                                {
									if((mcs_radio_config_p->th_squelch >= HEX_ZERO) && (mcs_radio_config_p->th_squelch <= DEC_45))
									{
										radioctrl_los_param[ctap_radio_id][NINE] = mcs_radio_config_p->th_squelch;
									}
									else
									{                                              
										radio_config_validationalfail = HEX_ONE;
									}
                                }                                     
                                if( radio_config_validationalfail == HEX_ZERO)
                                {
                                    //sprintf(CTRL_TaskNameSet,"w_R %d CTRL ", ctap_radio_id);                             
                                     
                                    res = pthread_create(&radioctrl_thread[ctap_radio_id], NULL, radio_ctrlcmd, &ctap_radio_id);		                           				           
				                    pthread_join(radioctrl_thread[ctap_radio_id], NULL);			           
                                }
                                else
                                {
                                    send_radioconfig_ack(config_ack_status);
                                }                                  
                            }
                            else
                            {
                                    ;
                            }
                        }
                        else
                        {
                            send_radioconfig_ack(config_ack_status);
                        }                
                              break;
                    }
                case THIRTYFOUR: /*  Relay Configuration  */
                    {
						bzero(ctrlcmd_recvbufmod, sizeof(ctrlcmd_recvbufmod));
						recv(ctrl_msc_sockfd, (int8_t *)&ctrlcmd_recvbufmod ,HEX_FOUR ,HEX_ZERO);
						for(ctrlcmdloop = HEX_ZERO; ctrlcmdloop < HEX_FOUR ;ctrlcmdloop++)
						{
							  ctrlcmd_recvbuf[ DEC_12 + ctrlcmdloop] = ctrlcmd_recvbufmod[ctrlcmdloop];
						}
						mcs_relay_config_p = (struct MSC_MCS_RELAY_CONFIGURATION *)ctrlcmd_recvbuf;                            
						for(ctrlcmdloop =HEX_ZERO; ctrlcmdloop< (TOTAL_RADIOS/HEX_TWO) * HEX_TWO; ctrlcmdloop++)
						{
							relay_pairconfig_store[ctrlcmdloop] = mcs_relay_config_p->relay_tr[ctrlcmdloop];
							  
						}                              
                        break;                              
                    }
                  default:
                    {
                             
                        break;
                    }
            }               
        }
    } 
}


/** FHdr-beg *****************************************************
**
** Function name: ap_erase
**
** Anchor:       RCS_SOW_REQ003 
**
** Purpose:       This function perform ap erase 
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


void *ap_erase()
{
   
    uint8_t ape_radio_id 				= HEX_ZERO;
	uint8_t ape_ifstatementcheck 		= HEX_ZERO; 
    uint8_t radio_id_buf[SIX] 			= {HEX_ZERO,HEX_ONE,HEX_TWO,HEX_THREE,HEX_FOUR,HEX_FIVE};
    uint8_t res 						= HEX_ZERO;       
    bzero((int8_t *)&mcs_ap_erase_ack, sizeof(mcs_ap_erase_ack));    
    mcs_ap_erase_ack.message_code 		= MCS_MSC_ADDITIONAL_PARAMS_ERASE_ACK;  
    mcs_ap_erase_ack.message_size 		= sizeof(mcs_ap_erase_ack) - HEX_FOUR;    
    mcs_ap_erase_ack.time_sec     		= time_send.sec;
    mcs_ap_erase_ack.time_fracsec 		= time_send.fracsec;
    mcs_ap_erase_ack.ap_erase_ack 		= HEX_0xFE;    
   
    if(send(ctrl_mcs_sockfd, (int8_t *)&mcs_ap_erase_ack, sizeof(mcs_ap_erase_ack), HEX_ZERO) <= HEX_ZERO)
    {
       ;
    }
	else
	{
		;
	}        
    ape_ifstatementcheck = HEX_ONE;   
    if(ape_ifstatementcheck == HEX_ONE /*&& taskIdVerify(McsMbitTId) == ERROR */ && mcs_pfm_thread == HEX_ZERO)
    {            
        //while(TRUE)
		for(;;)
        {
            ape_ifstatementcheck = HEX_ONE;
            for(ape_radio_id = HEX_ONE; ape_radio_id <= HEX_ONE/*TOTAL_RADIOS*/; ape_radio_id++)
            {  
               
                if((radio_present_state[ape_radio_id] == HEX_ONE /*&& radio_reinit_process[ape_radio_id] == 0 */) || (radio_present_state[ape_radio_id] == HEX_FIVE ||
                  ((radio_present_state[ape_radio_id] == HEX_TWO || radio_present_state[ape_radio_id] == HEX_THREE) && radioctrl_thread[ape_radio_id] == HEX_ZERO) 
                    /*&& taskIdVerify(RadioIbitTId[ape_radio_id]) == ERROR*/))
                {                    
                    ape_ifstatementcheck = ape_ifstatementcheck & HEX_ONE;
                }
                else
                {
                    
                    ape_ifstatementcheck = ape_ifstatementcheck & HEX_ZERO;
                }
            }            
            //if(ape_ifstatementcheck == 1 && taskIdVerify(Mcs_StateTId) == ERROR)
            if(ape_ifstatementcheck == HEX_ONE && mcs_state_change_thread == HEX_ZERO)
            {
                break;
            }     
            else
            {
                usleep(SEC * DEC_2); // taskDelay(sysclkRateGet()*2);
            }
        }        
        ape_ifstatementcheck = HEX_ONE;
        for(ape_radio_id = HEX_ONE; ape_radio_id <= HEX_ONE/*TOTAL_RADIOS*/; ape_radio_id++)
        {
            if(radio_present_state[ape_radio_id] ==HEX_ONE || radio_present_state[ape_radio_id] == HEX_FIVE ||
              ((radio_present_state[ape_radio_id] == HEX_TWO || radio_present_state [ape_radio_id] == HEX_THREE) && radio_securemode[ape_radio_id] == HEX_ZERO))
            {
                ape_ifstatementcheck = ape_ifstatementcheck & HEX_ONE;
            }
            else
            {
                ape_ifstatementcheck = ape_ifstatementcheck & HEX_ZERO;
            }   
        }          
        if(ape_ifstatementcheck == HEX_ONE)
        {
            
            for(ape_radio_id = HEX_ONE; ape_radio_id <= HEX_ONE/*TOTAL_RADIOS*/;  ape_radio_id++)
            {
                res = pthread_create(&radio_ap_erase_thread[ape_radio_id], NULL, radio_ap_erase, &radio_id_buf[ape_radio_id]);		 		
				sleep(HEX_ONE);
            }
            for(ape_radio_id = HEX_ONE; ape_radio_id <= HEX_ONE/*TOTAL_RADIOS*/;  ape_radio_id++)
            {
                pthread_join(radio_ap_erase_thread[ape_radio_id], NULL);
            }            
            //while(TRUE)
			for(;;)
            {
                ape_ifstatementcheck = HEX_ONE;                
                for(ape_radio_id = HEX_ONE; ape_radio_id <= HEX_ONE/*TOTAL_RADIOS*/;  ape_radio_id++) 
                { 
                    if((radio_ap_erase_thread[ape_radio_id]) == HEX_ZERO)
                    {
                        ape_ifstatementcheck = ape_ifstatementcheck & HEX_ONE;
                    }
                    else
                    {
                        ape_ifstatementcheck = ape_ifstatementcheck & HEX_ZERO;
                    }
                }              
                if(ape_ifstatementcheck == HEX_ONE)
                {
                    mcs_ap_erase_ack.ap_erase_ack = HEX_FF; 
                    break;
                }
                else
                {
                    usleep(SEC * HEX_TWO);  // taskDelay(sysclkRateGet() * 2);                    
                } 
            }
        }
        else
        {
            
            mcs_ap_erase_ack.ap_erase_ack = HEX_ZERO;
        }
    }
    else
    {
        ape_ifstatementcheck = HEX_ZERO;        
        mcs_ap_erase_ack.ap_erase_ack = HEX_ZERO;
    } 
    mcs_ap_erase_ack.message_code = MCS_MSC_ADDITIONAL_PARAMS_ERASE_ACK;
    mcs_ap_erase_ack.message_size = sizeof(mcs_ap_erase_ack) - HEX_FOUR;    
    mcs_ap_erase_ack.time_sec     = time_send.sec;
    mcs_ap_erase_ack.time_fracsec = time_send.fracsec;         
    if(send(ctrl_mcs_sockfd, (int8_t *)&mcs_ap_erase_ack, sizeof(mcs_ap_erase_ack), HEX_ZERO) <= HEX_ZERO)
    {
        ;
    }
	else
	{
		;
	}
    if(mcs_ap_erase_ack.ap_erase_ack == HEX_FF)
    {
        usleep(MILISEC * DEC_10);  //taskDelay(10);
        send_ap_status();
    }
	else
	{
		;
	}
    //AP_EraseTId = ERROR;
    ap_erase_thread = HEX_ZERO;
    //return(OK);
}


/** FHdr-beg *****************************************************
**
** Function name: radio_ap_erase
**
** Anchor:        RCS_SOW_REQ003
**
** Purpose:       This function perform radio  ap erase 
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

void *radio_ap_erase(void *arg)
{
    uint8_t ap_erase_radio_id = *(uint8_t *)arg;
    uint8_t res = HEX_ZERO;       
	radio_param_setstatus[ap_erase_radio_id] = hq_erase(ap_erase_radio_id, radio_serialfd[ap_erase_radio_id]);	
	if(radio_param_setstatus[ap_erase_radio_id] == SUCCESS_STATUS)
	{		
		sleep(DEC_10);  //usleep(SEC * 10);  //taskDelay(sysClkRateGet() * 10);                
	}
	else
	{
		;
	}
	radio_param_setstatus[ap_erase_radio_id] = rpw_erase(ap_erase_radio_id, radio_serialfd[ap_erase_radio_id]);
	if(radio_param_setstatus[ap_erase_radio_id] == SUCCESS_STATUS)
	{		
		sleep(DEC_20); //usleep(SEC * 20); //taskDelay(sysClkRateGet() * 20);                
	}
	else
	{
		;
	}    
    radio_ap_erase_thread[ap_erase_radio_id] = HEX_ZERO;
    return(OK);
} 


/** FHdr-beg *****************************************************
**
** Function name: send_radioconfig_ack
**
** Anchor:        RCS_SOW_REQ003
**
** Purpose:       This function perform Send radio configuration acknowledgment   
**				  
**				  
**							   
** Inputs:        radio ack status 
**
**
** Outputs:       NONE      
**
** FPrj-beg *****************************************************
**
** FPrj-end *****************************************************
** FHdr-end *****************************************************/  

int32_t send_radioconfig_ack(uint8_t *SendingAckStatus)
{
    struct MCS_MSC_RADIO_CONFIGURATION_ACK
    {
        uint16_t message_code;
        uint16_t message_size;
        uint32_t time_sec;
        uint32_t time_fracsec;
        uint8_t  radio_id;
        uint8_t  radiationonoff_ack;
        uint8_t  allocation_ack;
        uint8_t  mode_ack;
        uint8_t  channelmain_ack;
        uint8_t  frequencymain_ack;
        uint8_t  channelguard_ack;
        uint8_t  frequencyguard_ack;
        uint8_t  substate_ack;
        uint8_t  txpower_ack;
        uint8_t  th_squelch_ack;
        
    } mcs_radio_configack;
     
    bzero((int8_t *)&mcs_radio_configack, sizeof(mcs_radio_configack));     
    mcs_radio_configack.message_code       = MCS_MSC_RADIO_CONFIGURATION_ACK;
    mcs_radio_configack.message_size       = sizeof(mcs_radio_configack) -HEX_FOUR;    
    mcs_radio_configack.time_sec           = time_send.sec;
    mcs_radio_configack.time_fracsec       = time_send.fracsec;
    mcs_radio_configack.radio_id           = SendingAckStatus[ZERO];
    mcs_radio_configack.radiationonoff_ack = SendingAckStatus[ONE];
    mcs_radio_configack.allocation_ack     = SendingAckStatus[TWO];
    mcs_radio_configack.mode_ack           = SendingAckStatus[THREE];
    mcs_radio_configack.channelmain_ack    = SendingAckStatus[FOUR];
    mcs_radio_configack.frequencymain_ack  = SendingAckStatus[FOUR];
    mcs_radio_configack.channelguard_ack   = SendingAckStatus[EIGHT];
    mcs_radio_configack.frequencyguard_ack = SendingAckStatus[EIGHT];
    mcs_radio_configack.substate_ack       = SendingAckStatus[FIVE];
    mcs_radio_configack.txpower_ack        = SendingAckStatus[SIX];
    mcs_radio_configack.th_squelch_ack      = SendingAckStatus[SEVEN];                                                   
    if((send(ctrl_mcs_sockfd, (int8_t *)&mcs_radio_configack, sizeof(mcs_radio_configack), HEX_ZERO)) <= HEX_ZERO)
    {
         
        return(ERROR);
    }
     return(OK);
}



/** FHdr-beg *****************************************************
**
** Function name: radio_ctrlcmd
**
** Anchor:        RCS_SOW_REQ002
**
** Purpose:       This function perform  radio control command  
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

void *radio_ctrlcmd(void *arg)
{
    uint8_t ctl_radio_id = *(uint8_t *)arg;    
    uint8_t radio_configack_status[DEC_10]     = "";
	uint8_t store_setup_paramstore[DEC_10]     = ""; 
	uint8_t radioctrl_cmd_hexfreq[FOUR]		   = ""; 
	uint8_t fh_loadfail_ind 				   = HEX_ZERO;
    uint8_t radio_ctrl_process 				   = HEX_ZERO; 
	uint8_t radio_ss_sendset 				   = HEX_ZERO; 
	uint8_t radio_emg_sendset 				   = HEX_ZERO; 
	uint8_t radio_fh_sendset 				   = HEX_ZERO;  
	uint8_t radio_thsq_sendset 				   = HEX_ZERO;  
	uint8_t store_setupafter_emg 			   = HEX_ZERO;
    uint8_t radio_ss_sendset_new 			   = HEX_ZERO; 
	uint8_t radio_emg_sendset_new 			   = HEX_ZERO; 
	uint8_t radio_fh_sendset_new 			   = HEX_ZERO; 
	uint8_t radio_thsq_sendset_new 			   = HEX_ZERO; 
	uint8_t radioctrl_reload 			 	   = HEX_ZERO;
    uint32_t i 								   = HEX_ZERO;    
    radio_configack_status[ZERO] 			   = ctl_radio_id;    
    radio_configack_status[ONE]  = radio_configack_status[TWO] = radio_configack_status[THREE] = radio_configack_status[FOUR]  = HEX_0xFE;
    radio_configack_status[FIVE] = radio_configack_status[SIX] = radio_configack_status[SEVEN] = radio_configack_status[EIGHT] = HEX_0xFE;    
    send_radioconfig_ack(radio_configack_status);    
    radio_configack_status[ONE]  = radio_configack_status[TWO] = radio_configack_status[THREE] = radio_configack_status[FOUR]  = HEX_FF;
    radio_configack_status[FIVE] = radio_configack_status[SIX] = radio_configack_status[SEVEN] = radio_configack_status[EIGHT] = HEX_FF;
    if(mcs_pfm_thread == HEX_ZERO && ap_erase_thread == HEX_ZERO)  
    {        //semTake(radio_syncsem[ctl_radio_id], WAIT_FOREVER);
        sem_wait(&radio_syncsem[ctl_radio_id]);        
        if(radio_present_state[ctl_radio_id] == HEX_TWO || radio_present_state[ctl_radio_id] == HEX_THREE)
        {
            fh_loadfail_ind			 = HEX_ZERO;
            radio_ss_sendset_new 	 = HEX_ZERO;
            radio_fh_sendset_new 	 = HEX_ZERO;
            radio_emg_sendset_new 	 = HEX_ZERO;
            radio_thsq_sendset_new  = HEX_ZERO;            
            radio_standby_status[ctl_radio_id] = HEX_ZERO;
            //while(TRUE)
			for(;;)
            {
                radio_ss_sendset	  = HEX_ZERO;
                radio_fh_sendset 	  = HEX_ZERO;
                radio_emg_sendset 	  = HEX_ZERO;
                radio_thsq_sendset 	  = HEX_ZERO;
                radio_ctrl_process    = HEX_ZERO;                
                /* Checking conditions between PFM values and Present Radio Config Values */
                /* checking for substate, secure mode and Guard freq */
                if(radioctrl_los_param[ctl_radio_id][NINE] != radiostored_los_param[ctl_radio_id][NINE] && radio_thsq_sendset_new == HEX_ZERO && 
                   radio_securemode[ctl_radio_id] == HEX_ZERO && radio_guardchannel[ctl_radio_id] == HEX_ZERO)
                {
                    radio_thsq_sendset = HEX_ONE;
                    radio_ctrl_process = HEX_ONE;
                    
                }
                
                /* checking for mode, freq, guard freq */
                if((((radioctrl_los_param[ctl_radio_id][TWO]   != radiostored_los_param[ctl_radio_id][TWO]   || 
                      radioctrl_los_param[ctl_radio_id][THREE] != radiostored_los_param[ctl_radio_id][THREE] ||
                      radioctrl_los_param[ctl_radio_id][FOUR]  != radiostored_los_param[ctl_radio_id][FOUR]  ||
                      radioctrl_los_param[ctl_radio_id][FIVE]  != radiostored_los_param[ctl_radio_id][FIVE]  ||
                      radioctrl_los_param[ctl_radio_id][SIX]   != radiostored_los_param[ctl_radio_id][SIX]   ||
                      radioctrl_los_param[ctl_radio_id][SEVEN] != radiostored_los_param[ctl_radio_id][SEVEN] ||
                      radioctrl_los_param[ctl_radio_id][EIGHT] != radiostored_los_param[ctl_radio_id][EIGHT])&&
                      radio_guardchannel[ctl_radio_id]         == HEX_ZERO && radioload_securemode[ctl_radio_id] == HEX_ZERO) ||
                     (radioload_securemode[ctl_radio_id] 	   == HEX_ZERO && radio_securemode[ctl_radio_id] != HEX_ZERO) || 
                       store_setupafter_emg == HEX_ONE) && radio_ss_sendset_new == HEX_ZERO)
                {
                    radio_ss_sendset     = HEX_ONE;
                    radio_ctrl_process   = HEX_ONE;
                    store_setupafter_emg = HEX_ZERO;                    
                }
				else
				{
					;
				}                
                /* checking for guard freq, secure mode */
                if(radio_guardchannel[ctl_radio_id]   == HEX_ZERO && ((radioload_securemode[ctl_radio_id] != HEX_ZERO &&
                   radio_securemode[ctl_radio_id]     != radioload_securemode[ctl_radio_id]) ||
                  (radioload_securemode[ctl_radio_id] == HEX_ONE && hq_fha_guardcontrol[ctl_radio_id] != radioctrl_los_param[ctl_radio_id][EIGHT])) && 
                   fh_loadfail_ind == HEX_ZERO)
                {
                    radio_fh_sendset   = HEX_ONE;
                    radio_ctrl_process = HEX_ONE;                                       
                }
				else
				{
					;
				}  				
                /* checking for emg guard condition and guard freq */
                if((radiostored_los_param[ctl_radio_id][ONE] != radioctrl_los_param[ctl_radio_id][ONE] || 
                    radio_guardchannel[ctl_radio_id] != radioload_guardchannel[ctl_radio_id]) && radio_emg_sendset_new == HEX_ZERO)
                {
                    radio_emg_sendset  = HEX_ONE;
                    radio_ctrl_process = HEX_ONE;
                }
                
                if(radio_ctrl_process)
                {
                    if(radio_thsq_sendset)
                    {  
                        radio_param_setstatus[ctl_radio_id] = squelch_th(ctl_radio_id, radio_serialfd[ctl_radio_id], radioctrl_los_param);
                        if(radio_param_setstatus[ctl_radio_id] == SUCCESS_STATUS)
                        {
                            printf("** Squelch-Setting Success for <Radio-%d> **\n\r", ctl_radio_id);
                            radiostored_los_param[ctl_radio_id][NINE] = radioctrl_los_param[ctl_radio_id][NINE];
                            radio_configack_status[SEVEN] = HEX_FF;
                        }
                        else
                        {                          
                            radio_thsq_sendset_new = HEX_ONE;
                            radio_configack_status[SEVEN] = HEX_ZERO;
                        }
                    }
                    
                    if(radio_ss_sendset)
                    {                       
                        activate_setup(ctl_radio_id, radio_serialfd[ctl_radio_id], HEX_THREE);
                        radio_securemode[ctl_radio_id] = HEX_ZERO;                 
                        store_setup_los(ctl_radio_id, radio_serialfd[ctl_radio_id], radioctrl_los_param, storesetup_setstatus, 
                        radioctrl_833_freqstatus[ctl_radio_id]);                                        
                        if(storesetup_setstatus[ctl_radio_id][ZERO] == SUCCESS_STATUS && storesetup_setstatus[ctl_radio_id][ONE]   == SUCCESS_STATUS &&
                           storesetup_setstatus[ctl_radio_id][TWO]  == SUCCESS_STATUS && storesetup_setstatus[ctl_radio_id][THREE] == SUCCESS_STATUS)
                        {                         
                            
                            radio_loadedfreq[ctl_radio_id] =  radioctrl_freqvalue[ctl_radio_id];                            
                            radiostored_los_param[ctl_radio_id][TWO]   = radioctrl_los_param[ctl_radio_id][TWO];
                            radiostored_los_param[ctl_radio_id][THREE] = radioctrl_los_param[ctl_radio_id][THREE];
                            radiostored_los_param[ctl_radio_id][FOUR]  = radioctrl_los_param[ctl_radio_id][FOUR];
                            radiostored_los_param[ctl_radio_id][FIVE]  = radioctrl_los_param[ctl_radio_id][FIVE];
                            radiostored_los_param[ctl_radio_id][SIX]   = radioctrl_los_param[ctl_radio_id][SIX];
                            radiostored_los_param[ctl_radio_id][SEVEN] = radioctrl_los_param[ctl_radio_id][SEVEN];
                            radiostored_los_param[ctl_radio_id][EIGHT] = radioctrl_los_param[ctl_radio_id][EIGHT];                                                                         
                            radioloaded_833_freqstatus[ctl_radio_id]   = radioctrl_833_freqstatus[ctl_radio_id];                            
                            radio_configack_status[THREE] 			   = HEX_FF;
                            radio_configack_status[FOUR] 			   = HEX_FF;
                            radio_configack_status[FIVE] 			   = HEX_FF;
                            radio_configack_status[SIX] 			   = HEX_FF;
                        }
                        else
                        {
                            radio_ss_sendset_new = HEX_ONE; 
                            if(storesetup_setstatus[ctl_radio_id][ZERO] == SUCCESS_STATUS)
                            {
                                radiostored_los_param[ctl_radio_id][TWO] = radioctrl_los_param[ctl_radio_id][TWO];
                                radio_configack_status[THREE] = HEX_FF;
                            }
                            else
                            {
                                radio_configack_status[THREE] = HEX_ZERO;
                            }                            
                            if(storesetup_setstatus[ctl_radio_id][ONE] == SUCCESS_STATUS)
                            {
                                radio_loadedfreq[ctl_radio_id]             =  radioctrl_freqvalue[ctl_radio_id];                                
                                radiostored_los_param[ctl_radio_id][THREE] = radioctrl_los_param[ctl_radio_id][THREE];
                                radiostored_los_param[ctl_radio_id][FOUR]  = radioctrl_los_param[ctl_radio_id][FOUR];
                                radiostored_los_param[ctl_radio_id][FIVE]  = radioctrl_los_param[ctl_radio_id][FIVE];
                                radiostored_los_param[ctl_radio_id][SIX]   = radioctrl_los_param[ctl_radio_id][SIX];                            
                                radioloaded_833_freqstatus[ctl_radio_id] = radioctrl_833_freqstatus[ctl_radio_id];
                                radio_configack_status[FOUR] = HEX_FF;
                            }
                            else
                            {
                                radio_configack_status[FOUR] = HEX_ZERO;
                            }                            
                            if(storesetup_setstatus[ctl_radio_id][TWO] == SUCCESS_STATUS)
                            {
                                radiostored_los_param[ctl_radio_id][SEVEN] = radioctrl_los_param[ctl_radio_id][SEVEN];
                                radio_configack_status[FIVE] = HEX_FF;
                            }
                            else
                            {
                                radio_configack_status[FIVE] = HEX_ZERO;
                            }                            
                            if(storesetup_setstatus[ctl_radio_id][THREE] == SUCCESS_STATUS)
                            {
                                radiostored_los_param[ctl_radio_id][EIGHT] = radioctrl_los_param[ctl_radio_id][EIGHT];
                                radio_configack_status[SIX] = HEX_FF;
                            }
                            else
                            {
                                radio_configack_status[SIX] = HEX_ZERO;
                            }
                        }
                    } // end of if(radio_ss_sendset)                    
                    if(radio_fh_sendset)
                    {
                        if(radio_ap_present[ctl_radio_id] != HEX_ONE)
                        {
                            rpw_re_operation_complete[ctl_radio_id] = HEX_ZERO;                            
                            radio_param_setstatus[ctl_radio_id] = hq_erase(ctl_radio_id, radio_serialfd[ctl_radio_id]);                            
                            sleep(DEC_10);
                            if(radio_tod_present_status[ctl_radio_id] == HEX_ZERO)
                            {
                                radioctrl_reload = HEX_ONE;
                                while(radioctrl_reload <= SENDPACK_MED)
                                {
                                    radio_tod_setstatus[ctl_radio_id] = FAILURE_STATUS;
                                    radio_tod_setstatus[ctl_radio_id] = radio_tod_load(ctl_radio_id, radio_serialfd[ctl_radio_id]);                                    
                                    if(radio_tod_setstatus[ctl_radio_id] == SUCCESS_STATUS)
                                    {                                        
                                        sleep(DEC_22);                                        
                                        radio_tod_setstatus[ctl_radio_id] = FAILURE_STATUS;
                                        radio_tod_setstatus[ctl_radio_id] = radio_tod_status(ctl_radio_id, radio_serialfd[ctl_radio_id]);
                                        if(radio_tod_setstatus[ctl_radio_id] == SUCCESS_STATUS)
                                        {
                                            radio_tod_present_status[ctl_radio_id] = HEX_ONE;       
                                            radioctrl_reload = LOOP_TERMINATE;                                 
                                        }
                                        else
                                        {
                                            if(radioctrl_reload == SENDPACK_MED)
                                            {
                                                radio_tod_present_status[ctl_radio_id] = HEX_ZERO;
                                            }
                                            radioctrl_reload++;                                          
                                        }  
                                    }                                    
                                }
                            }  // end of if(radio_tod_present_status[ctl_radio_id] == 0)
                            
                            if(radio_tod_present_status[ctl_radio_id] == HEX_ONE)
                            {
                                radio_param_setstatus[ctl_radio_id] = hq_param_load(ctl_radio_id, radio_serialfd[ctl_radio_id], hq_param_fha);                                
                                if(radio_param_setstatus[ctl_radio_id] == SUCCESS_STATUS)
                                {
                                    radio_ap_present[ctl_radio_id] = HEX_ONE;
                                }
                                else
                                {
                                    radio_ap_present[ctl_radio_id] = HEX_ZERO;
                                }                                
                                usleep(SEC * DEC_2);  // taskDelay(sysClkRateGet() * 2);
                                radioctrl_reload = HEX_ONE;                                
                                while(radioctrl_reload <= SENDPACK_MED)
                                {
                                    if(radio_ap_present[ctl_radio_id] == HEX_ONE)
                                    {
                                        radio_rpw_setstatus[ctl_radio_id] = FAILURE_STATUS;
                                        radio_rpw_setstatus[ctl_radio_id] = rpw_param_load(ctl_radio_id, radio_serialfd[ctl_radio_id], rpw_param_fhb);
                                        
                                        if(radio_rpw_setstatus[ctl_radio_id] == SUCCESS_STATUS)
                                        {
                                            usleep(SEC * DEC_20); //taskDelay(sysClkRateGet() * 20);
                                            radio_rpw_setstatus[ctl_radio_id] = FAILURE_STATUS;
                                            radio_rpw_setstatus[ctl_radio_id] = rpw_param_load_reinit(ctl_radio_id, radio_serialfd[ctl_radio_id]);
                                            if(radio_rpw_setstatus[ctl_radio_id] == SUCCESS_STATUS)
                                            {
                                                radioctrl_reload = LOOP_TERMINATE;
                                                radio_ap_present[ctl_radio_id] = HEX_ONE;
                                            }
                                            else
                                            {
                                                if(radioctrl_reload == SENDPACK_MED)
                                                {
                                                    radio_ap_present[ctl_radio_id] = HEX_ZERO;
                                                }
                                                radioctrl_reload++;
                                            }
                                        }
                                        else
                                        {
                                            if(radioctrl_reload == SENDPACK_MED)
                                            {
                                                radio_ap_present[ctl_radio_id] = HEX_ZERO;                                            
                                            }
                                            radioctrl_reload++;
                                        }
                                    }
                                    else
                                    {
                                        radioctrl_reload++;
                                    }      
                                }
                            } // end of if(radio_tod_present_status[ctl_radio_id] == 1)
                            else
                            {
                                send_alert_Msg(ctl_radio_id, RADIO_TOD_LOAD_FAILURE);
                            }
                            
                            if(radio_ap_present[ctl_radio_id] == HEX_ONE)
                            {
                                send_ap_status();
                            }
                            usleep(SEC * DEC_3); //taskDelay(sysClkRateGet() * 3);
                        } // end of if(radio_ap_present[ctl_radio_id] != 1)
                        
                        if(radio_ap_present[ctl_radio_id] == HEX_ONE)
                        {
                            radio_emgactive_aftersecure[ctl_radio_id] = HEX_ZERO;
                            if(radioload_securemode[ctl_radio_id] == HEX_ONE && (radio_securemode[ctl_radio_id] != HEX_ONE || 
                               hq_fha_guardcontrol[ctl_radio_id] != radioctrl_los_param[ctl_radio_id][EIGHT]))  // Have Quick
                            {                                
                                radio_param_setstatus[ctl_radio_id] = hq_mode_operation(ctl_radio_id, radio_serialfd[ctl_radio_id], hq_param_fha);                                
                                if(radio_param_setstatus[ctl_radio_id] == SUCCESS_STATUS)
                                {
                                    radio_securemode[ctl_radio_id] = HEX_ONE;
                                    radio_configack_status[THREE] = HEX_FF;
                                    hq_fha_guardcontrol[ctl_radio_id] = radioctrl_los_param[ctl_radio_id][EIGHT];
                                }
                                else
                                {
                                    radio_fh_sendset_new = HEX_ONE;
                                    radio_configack_status[THREE] = HEX_ZERO;
                                }                                
                            }
                            else if(radioload_securemode[ctl_radio_id] == HEX_TWO && radio_securemode[ctl_radio_id] != HEX_TWO) // RPW
                            {
                                radio_param_setstatus[ctl_radio_id] = rpw_mode_operation(ctl_radio_id, radio_serialfd[ctl_radio_id], rpw_param_fhb);                                
                                if(radio_param_setstatus[ctl_radio_id] == SUCCESS_STATUS)
                                {
                                    radio_securemode[ctl_radio_id] = HEX_TWO;
                                    radio_configack_status[THREE] = HEX_FF;                                    
                                }
                                else
                                {
                                    radio_fh_sendset_new = HEX_ONE;
                                    radio_configack_status[THREE] = HEX_ZERO;
                                }          
                            }                            
                        } // end of if(radio_ap_present[ctl_radio_id] == 1)
                        else
                        {
                            radio_securemode[ctl_radio_id] = HEX_ZERO;
                            radio_configack_status[THREE] = HEX_ZERO;
                            fh_loadfail_ind = HEX_ONE;
                            radio_fh_sendset_new = HEX_ONE;
                            send_alert_Msg(ctl_radio_id, RADIO_NOT_AVAILABLE_FOR_FHA_FHB_MODE);
                        }
                    } // end of if(radio_fh_sendset)
                    
                    if(radio_emg_sendset)
                    {                        
                        if(radioctrl_los_param[ctl_radio_id][ONE] == HEX_TWO)
                        {
                            //Emg Guard                            
                            if(radio_securemode[ctl_radio_id] != HEX_ZERO)
                            {
                                activate_setup(ctl_radio_id, radio_serialfd[ctl_radio_id], HEX_THREE);                                
                                radio_emgactive_aftersecure[ctl_radio_id] = radio_securemode[ctl_radio_id];
                                radio_securemode[ctl_radio_id]            = HEX_ZERO;
                            }
                            
                            if(radioload_guardchannel[ctl_radio_id] == HEX_THREE)
                            {
                                activate_setup(ctl_radio_id, radio_serialfd[ctl_radio_id], HEX_FIVE);                                
                                if(radio_param_setstatus[ctl_radio_id] == SUCCESS_STATUS)
                                {
                                    radio_guardchannel[ctl_radio_id] = HEX_THREE;
                                    radio_configack_status[EIGHT] = HEX_FF;
                                }
                                else
                                {
                                    radio_emg_sendset_new = HEX_ONE;
                                    radio_configack_status[EIGHT] = HEX_ZERO;
                                }
                            }
                            else
                            {
                                if(radioload_guardchannel[ctl_radio_id] == HEX_TWO)
                                {
                                    dec_to_hex(UHFGUARD_RANFREQ, radioctrl_cmd_hexfreq);
                                    radio_guardchannel[ctl_radio_id] = HEX_TWO;
                                }
                                else
                                {
                                    dec_to_hex(VHFGUARD_RANFREQ, radioctrl_cmd_hexfreq);
                                    radio_guardchannel[ctl_radio_id] = HEX_ONE;
                                }                                
                                storesetup_paramstore[TWO]  		 	= radioctrl_los_param[ctl_radio_id][TWO];
                                storesetup_paramstore[THREE] 		 	= radioctrl_los_param[ctl_radio_id][THREE];
                                storesetup_paramstore[FOUR]  		 	= radioctrl_los_param[ctl_radio_id][FOUR];
                                storesetup_paramstore[FIVE]  		 	= radioctrl_los_param[ctl_radio_id][FIVE];
                                storesetup_paramstore[SIX]   		 	= radioctrl_los_param[ctl_radio_id][SIX];
                                storesetup_paramstore[SEVEN] 		 	= radioctrl_los_param[ctl_radio_id][SEVEN];
                                storesetup_paramstore[EIGHT] 		 	= radioctrl_los_param[ctl_radio_id][EIGHT];                                
                                radioctrl_los_param[ctl_radio_id][TWO]  = HEX_ZERO;
                                radioctrl_los_param[ctl_radio_id][THREE]= *(radioctrl_cmd_hexfreq);
                                radioctrl_los_param[ctl_radio_id][FOUR] = *(radioctrl_cmd_hexfreq + HEX_ONE);
                                radioctrl_los_param[ctl_radio_id][FIVE] = *(radioctrl_cmd_hexfreq + HEX_TWO);
                                radioctrl_los_param[ctl_radio_id][SIX]  = *(radioctrl_cmd_hexfreq + HEX_THREE);
                                radioctrl_los_param[ctl_radio_id][SEVEN]= HEX_FOUR;
                                radioctrl_los_param[ctl_radio_id][EIGHT]= HEX_ONE;  
                                store_setup_los(ctl_radio_id, radio_serialfd[ctl_radio_id], radioctrl_los_param, storesetup_setstatus, 0);
                                
                                if(storesetup_setstatus[ctl_radio_id][ONE] == SUCCESS_STATUS)
                                {
                                    radio_configack_status[EIGHT] = HEX_FF;                                    
                                }
                                else
                                {
                                    radio_emg_sendset_new = HEX_ONE;
                                    radio_configack_status[EIGHT] = HEX_ZERO;
                                }                                    
                                radioctrl_los_param[ctl_radio_id][TWO]   = storesetup_paramstore[TWO];
                                radioctrl_los_param[ctl_radio_id][THREE] = storesetup_paramstore[THREE];
                                radioctrl_los_param[ctl_radio_id][FOUR]  = storesetup_paramstore[FOUR];
                                radioctrl_los_param[ctl_radio_id][FIVE]  = storesetup_paramstore[FIVE];
                                radioctrl_los_param[ctl_radio_id][SIX]   = storesetup_paramstore[SIX];
                                radioctrl_los_param[ctl_radio_id][SEVEN] = storesetup_paramstore[SEVEN];
                                radioctrl_los_param[ctl_radio_id][EIGHT] = storesetup_paramstore[EIGHT];
                                
                            }
                        }
                        radio_param_setstatus[ctl_radio_id] = emergency_guard(ctl_radio_id, radio_serialfd[ctl_radio_id], radioctrl_los_param);                           
                        if(radio_param_setstatus[ctl_radio_id] == SUCCESS_STATUS)
                        {
                            if(radio_guardchannel[ctl_radio_id] != HEX_ZERO && radioctrl_los_param[ctl_radio_id][ONE] == HEX_ZERO)    
                            {
                                store_setupafter_emg = HEX_ONE;
                                radio_guardchannel[ctl_radio_id] = HEX_ZERO;
                            }     
                            radiostored_los_param[ctl_radio_id][ONE] = radioctrl_los_param[ctl_radio_id][ONE];                        
                            radio_configack_status[TWO] = HEX_FF;                                 
                        }
                        else
                        {
                            radio_emg_sendset_new = HEX_ONE;
                            radio_configack_status[TWO] = HEX_ZERO;                          
                        }                          
                    }  // end of if(radio_emg_sendset)
                    usleep(SEC * DEC_2); //taskDelay(sysClkRateet() * 2);                    
                }
                else
                {
                    break;
                }
            } // end of FOREVER            
            if(radio_present_state[ctl_radio_id] == HEX_TWO || radio_present_state[ctl_radio_id] == HEX_THREE)
            {
                if(radioctrl_los_param[ctl_radio_id][ZERO] != radiostored_los_param[ctl_radio_id][ZERO])
                {                    
                    radio_param_setstatus[ctl_radio_id] = radiation_on_off(ctl_radio_id, radio_serialfd[ctl_radio_id], radioctrl_los_param);
                    if(radio_param_setstatus[ctl_radio_id] == SUCCESS_STATUS)
                    {
                        radiostored_los_param[ctl_radio_id][ZERO] = radio_radiation_status[ctl_radio_id] = radioctrl_los_param[ctl_radio_id][ZERO]; 
                        radio_configack_status[ONE] = HEX_FF;                            
                    }
                    else
                    {
                        radio_configack_status[ONE] = HEX_ZERO;                           
                    } 
                    if(radiostored_los_param[ctl_radio_id][ZERO] == HEX_ONE)
                    {
                        radio_present_state[ctl_radio_id] = radio_present_statestore[ctl_radio_id] = radio_present_stateload[ctl_radio_id] = HEX_TWO;
                        send_mcs_status();
                    }           
                    else if(radiostored_los_param[ctl_radio_id][ZERO] == HEX_ZERO)
                    {
                        radio_present_state[ctl_radio_id] = radio_present_statestore[ctl_radio_id] = radio_present_stateload[ctl_radio_id] = HEX_THREE;
                        send_mcs_status();
                    }
                }
            }
            else
            {
                radio_configack_status[ONE] = HEX_ZERO;
            }
        }
        else
        {
            radio_configack_status[ONE] = radio_configack_status[TWO] = radio_configack_status[THREE] = radio_configack_status[FOUR] = HEX_ZERO;
            radio_configack_status[FIVE] = radio_configack_status[SIX] = radio_configack_status[SEVEN] = radio_configack_status[EIGHT] = HEX_ZERO;      
        }       
        sem_post(&radio_syncsem[ctl_radio_id]);
    }
    else
    {
        ; 
           
    }
    
    send_radioconfig_ack(radio_configack_status); 
    radioctrl_thread[ctl_radio_id] = HEX_ZERO;
    return (OK);  
}
/******************************************************************************
* los_params_store
* 
* RETURNS: int32_t
******************************************************************************/
/** FHdr-beg *****************************************************
**
** Function name: los_params_store
**
** Anchor:        
**
** Purpose:       This function will store the LOS parameters 
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
int32_t los_params_store(void)
{
    uint8_t los_store_loop = HEX_ZERO; 
	uint8_t los_store_hexfreq[FOUR];   
    cbit_period_recv = HEX_SIX;
    for(los_store_loop = HEX_ONE; los_store_loop <= TOTAL_RADIOS; los_store_loop++)
    {
        radiostored_los_param[los_store_loop][ZERO] = HEX_ONE;   /* Radiation ON/OFF: 0-ON,  1-OFF */
        radiostored_los_param[los_store_loop][ONE]  = HEX_ZERO;   /* Emergency Guard : 0-OFF, 2-ON  */
        if(los_store_loop == HEX_ONE)
        {
            radiostored_los_param[los_store_loop][TWO] =  RADIO1_DEFAULT_MODE;  /* mode */
            dec_to_hex(RADIO1_LOS_DEFAULT_FREQ, los_store_hexfreq);             
            radio_loadedfreq[los_store_loop] = RADIO1_LOS_DEFAULT_FREQ;            
        }
        else if(los_store_loop == HEX_TWO)
        {
            radiostored_los_param[los_store_loop][TWO] =  RADIO2_DEFAULT_MODE;  /* mode */
            dec_to_hex(RADIO2_LOS_DEFAULT_FREQ, los_store_hexfreq);           
            radio_loadedfreq[los_store_loop] = RADIO2_LOS_DEFAULT_FREQ;  
        }  
        else if(los_store_loop == HEX_THREE)
        {
            radiostored_los_param[los_store_loop][TWO] =  RADIO3_DEFAULT_MODE;  /* mode */
            dec_to_hex(RADIO3_LOS_DEFAULT_FREQ, los_store_hexfreq);             
            radio_loadedfreq[los_store_loop] = RADIO3_LOS_DEFAULT_FREQ;  
        } 
        else if(los_store_loop == HEX_FOUR)
        {
            radiostored_los_param[los_store_loop][TWO] =  RADIO4_DEFAULT_MODE;  /* mode */
            dec_to_hex(RADIO4_LOS_DEFAULT_FREQ, los_store_hexfreq);             
            radio_loadedfreq[los_store_loop] = RADIO4_LOS_DEFAULT_FREQ;  
        }   
        else if(los_store_loop == HEX_FIVE)
        {
            radiostored_los_param[los_store_loop][TWO] =  RADIO5_DEFAULT_MODE;  /* mode */
            dec_to_hex(RADIO5_LOS_DEFAULT_FREQ, los_store_hexfreq);             
            radio_loadedfreq[los_store_loop] = RADIO5_LOS_DEFAULT_FREQ;  
        } 
        else
        {
            ;
        }         
        radiostored_los_param[los_store_loop][THREE] = *(los_store_hexfreq + HEX_ZERO);
        radiostored_los_param[los_store_loop][FOUR]  = *(los_store_hexfreq + HEX_ONE);
        radiostored_los_param[los_store_loop][FIVE]  = *(los_store_hexfreq + HEX_TWO);
        radiostored_los_param[los_store_loop][SIX]   = *(los_store_hexfreq + HEX_THREE);
        radiostored_los_param[los_store_loop][SEVEN] = HEX_FOUR - (HEX_THREE * (radiostored_los_param[los_store_loop][2]));  /* Tx power decrement */
        radiostored_los_param[los_store_loop][EIGHT] = HEX_ZERO;  /* Sub State */
        radiostored_los_param[los_store_loop][NINE] = DEC_20; /* Squelch */
        hq_fha_guardcontrol[los_store_loop] 		= radiostored_los_param[los_store_loop][EIGHT];
        radio_radiation_status[los_store_loop]		= radiostored_los_param[los_store_loop][ZERO];

    } // end of for 

    bzero(radio_securemode, sizeof(radio_securemode)); /* presently loaded secure mode: 0-Not loaded, 1-Have quick, 2-RPW */
    bzero(radio_guardchannel, sizeof(radio_guardchannel)); /* presently loaded guard channel: 0-Not loaded, 1-VHF, 2-UHF, 3-MARITIME */
    bzero(radio_emg_active_after_secure, sizeof(radio_emg_active_after_secure)); /* 0-No, 1-Yes */
    bzero(radio_cbitfirsttime_after_fh, sizeof(radio_cbitfirsttime_after_fh)); /* 0-No, 1-Yes */
    bzero(radio_rpw_operation_complete, sizeof(radio_rpw_operation_complete)); /* 0-No, 1-Yes */
    bzero(relaypairconfig, sizeof(relaypairconfig));
    for(los_store_loop = HEX_ZERO; los_store_loop < ((TOTAL_RADIOS / HEX_TWO) * HEX_TWO); los_store_loop++)
    {
		relaypairconfig[los_store_loop] = los_store_loop + HEX_ONE;
    }

    return OK;
}


/** FHdr-beg *****************************************************
**
** Function name: mcs_ctrlr_node_post
**
** Anchor:        RCS_SOW_REQ001
**
** Purpose:       This function perform mcs controller node post 
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
void mcs_ctrlr_node_post(void)
{
    uint8_t mcs_ctrlr_post_buf[TOTAL_MCS_CTRLR_BIT] = ""; 
	uint8_t mcs_ctrlr_post_num 						= HEX_ZERO; 
	uint8_t mcs_ctrlr_post_num_temp 				= HEX_ZERO;     
    bzero(mcs_ctrlr_bit_faultstatus, sizeof(mcs_ctrlr_bit_faultstatus));
    bzero(mcs_ctrlr_bit_curstatus, sizeof(mcs_ctrlr_bit_curstatus));
    bzero(mcs_ctrlr_bit_faultcode, sizeof(mcs_ctrlr_bit_faultcode));
    bzero(mcs_ctrlr_post_buf, sizeof(mcs_ctrlr_post_buf));
    mcs_ctrlr_post(mcs_ctrlr_post_buf);    // performing mcs controller post 
    mcs_ctrlr_post_num_temp = HEX_FF;
    for(mcs_ctrlr_post_num = HEX_ZERO; mcs_ctrlr_post_num < TOTAL_MCS_CTRLR_BIT; mcs_ctrlr_post_num++) 
    {
        if(mcs_ctrlr_post_buf[mcs_ctrlr_post_num] == HEX_ZERO)
        {
            if(mcs_ctrlr_post_num_temp == HEX_FF)
            {
                mcs_ctrlr_bit_faultstatus[ZERO]   = HEX_ZERO;
            	mcs_ctrlr_bit_faultstatus[ONE]    = HEX_ZERO;
            	mcs_ctrlr_bit_faultstatus[TWO]    = HEX_ONE;
            	mcs_ctrlr_bit_faultstatus[THREE]  = MCS_CONTROLLER_NODE_ID;
            	mcs_ctrlr_bit_faultcode[mcs_ctrlr_bit_faults] = HEX_ONE;
            	mcs_ctrlr_bit_curstatus[mcs_ctrlr_bit_faults] = HEX_FF;
                mcs_ctrlr_bit_faults++;
                mcs_ctrlr_post_num_temp          = HEX_ZERO;
                
            }
            else
            {
                ;
            } 
            mcs_ctrlr_bit_faultstatus[(mcs_ctrlr_bit_faults * HEX_FOUR) + HEX_ZERO] = mcs_cntlr_nodeidbuf[(mcs_ctrlr_post_num * HEX_FOUR) + HEX_ZERO];
            mcs_ctrlr_bit_faultstatus[(mcs_ctrlr_bit_faults * HEX_FOUR) + HEX_ONE] = mcs_cntlr_nodeidbuf[(mcs_ctrlr_post_num * HEX_FOUR)  + HEX_ONE];
            mcs_ctrlr_bit_faultstatus[(mcs_ctrlr_bit_faults * HEX_FOUR) + HEX_TWO] = mcs_cntlr_nodeidbuf[(mcs_ctrlr_post_num * HEX_FOUR)  + HEX_TWO];
            mcs_ctrlr_bit_faultstatus[(mcs_ctrlr_bit_faults * HEX_FOUR) + HEX_THREE] = mcs_cntlr_nodeidbuf[(mcs_ctrlr_post_num * HEX_FOUR)+ HEX_THREE];
            mcs_ctrlr_bit_faultcode[mcs_ctrlr_bit_faults] = mcs_cntlr_nodefaultcode[mcs_ctrlr_post_num];
            mcs_ctrlr_bit_curstatus[mcs_ctrlr_bit_faults] = HEX_FF;
            mcs_ctrlr_bit_faults++;
        }
        else
        {
            ;
        } 
    } 
}


/** FHdr-beg *****************************************************
**
** Function name: mcs_ctrlr
**
** Anchor:        RCS_SOW_REQ002
**
** Purpose:       This function perform mcs controller function  
**				  
**				  
**							   
** Inputs:        MCS controller start state  
**
**
** Outputs:       NONE      
**
** FPrj-beg *****************************************************
**
** FPrj-end *****************************************************
** FHdr-end *****************************************************/ 

uint32_t mcs_ctrlr(uint32_t mcs_ctrlr_start_state)
{    
    uint8_t  mcs_ctrlr_ifstatementcheck 		= HEX_ZERO;
    uint8_t  mcs_post_faultcode[TOTAL_NODES] 	= ""; 
	uint8_t mcs_post_curstatus[TOTAL_NODES] 	= ""; 
	uint8_t mcs_post_faultstatus[TOTAL_NODES * HEX_FOUR] = "";
    uint32_t mcs_post_node_numbers 				= HEX_ZERO; 
	uint32_t number_of_nodes 					= HEX_ZERO; 
	uint32_t postresults_size 					= HEX_ZERO;
    uint8_t  pfm_break_loop 					= HEX_ZERO;
    uint8_t  radio_id_buf[SIX] 					= {HEX_ZERO,HEX_ONE,HEX_TWO,HEX_THREE,HEX_FOUR,HEX_FIVE};
    uint8_t  radio_id 							= HEX_ZERO;
    uint8_t  bin_buf[EIGHT];
    int32_t  res 								= HEX_ZERO;   
    mcs_present_stateload 						= HEX_ONE;    
    if(mcs_ctrlr_start_state == HEX_ONE)
    {
        for(radio_id = HEX_ZERO; radio_id < TOTAL_RADIOS_PLUS_ONE; radio_id++)
        {
	    radio_serialfd[radio_id] = ERROR;
        }
    }
    else
    {
        ;
    }
    for(radio_id = HEX_ZERO; radio_id < TOTAL_RADIOS_PLUS_ONE; radio_id++)
    {
        Radio_IBit_TId[radio_id]         		= ERROR;
        Radio_CBit_TId[radio_id]         		= ERROR;
        Radio_MBit_TId[radio_id]         		= ERROR;        
        Radio_State_TId[radio_id]        		= ERROR;       
        Radio_CommCheck_TId[radio_id]    		= ERROR;
        Radio_ReInit_TId[radio_id]       		= ERROR;     
        radio_post_thread[radio_id]    			= HEX_ZERO;
        radio_tr_comm_thread[radio_id] 			= HEX_ZERO;
        radio_pfmload_thread[radio_id] 			= HEX_ZERO;
        radio_ap_erase_thread[radio_id]			= HEX_ZERO; 
        radio_loadedfreq[radio_id]           	= HEX_ZERO;
        radio_ctrl_freq_val[radio_id]        	= HEX_ZERO;
        radio_present_state[radio_id]        	= OFF_STATE; // 5 - OFF_STATE
        radio_present_statestore[radio_id]   	= HEX_FIVE;
        radioloaded_833_freqstatus[radio_id] 	= HEX_ZERO;

    }
    bzero(freqchan_buf, sizeof(freqchan_buf));
    bzero(blankfreq_chanbuf, sizeof(blankfreq_chanbuf));
    bzero((int8_t *)&time_send, sizeof(time_send));
    intercom_res_amscc = HEX_ZERO;
    /* Opening serial ports for communication with radios */
    for(radio_id = HEX_ONE; radio_id <= HEX_ONE/*TOTAL_RADIOS*/; radio_id++)
    {
		if(radio_serialfd[radio_id] == ERROR)
		{
			open_serial_port(radio_id, radio_serialfd, DEC_20);
			usleep(SEC * DEC_16);                                     // taskdelay(sysClkRateGet() * 16);
		}
			else
		{
			;
		}
    }   
    for(radio_id = HEX_ONE; radio_id <= HEX_ONE /*TOTAL_RADIOS*/; radio_id++)
    {
        res = sem_init(&radio_syncsem[radio_id], HEX_ZERO, HEX_ONE);  

    }           
    los_params_store();  // default parameters      
    /* Falcon post results */        
    mcs_ctrlr_node_post(); 
    for(radio_id = HEX_ONE; radio_id <= HEX_ONE /*TOTAL_RADIOS */; radio_id++)
    {
		res = pthread_create(&radio_post_thread[radio_id], NULL, radio_post, &radio_id_buf[radio_id]);       
        usleep(SEC * HEX_ONE);              
    }
    if(pfmbit_sockfd == ERROR)     
    {
        pfmbit_socket(HEX_ZERO);          // open socket to connect with MSC
    }
    else
    {
       ;
    }      
    if(intercom_res_amscc == HEX_ZERO)
    {
        ams_num_faults = SUBSYSTEM_NORESPONSE; 
    } 
    for(radio_id = HEX_ONE; radio_id <= TOTAL_RADIOS; radio_id++)
    {
        if(radio_present_state[radio_id] == OFF_STATE)
        {
            radionumfaults[radio_id] = SUBSYSTEM_NORESPONSE;
        }
        else
        {
            ;
        }
    }
    number_of_nodes = HEX_ONE;    
    /* AMS Faults Filling */
    if(ams_num_faults == SUBSYSTEM_NORESPONSE)
    {
    	mcs_post_faultstatus[(number_of_nodes * HEX_FOUR) + HEX_ZERO]  = HEX_ZERO;
        mcs_post_faultstatus[(number_of_nodes * HEX_FOUR) + HEX_ZERO]  = HEX_ZERO;
        mcs_post_faultstatus[(number_of_nodes * HEX_FOUR) + HEX_TWO]   = HEX_ZERO;
        mcs_post_faultstatus[(number_of_nodes * HEX_FOUR) + HEX_THREE] = AMS_NODE_ID;
        mcs_post_faultcode[number_of_nodes] = HEX_F1;
        mcs_post_curstatus[number_of_nodes] = HEX_FF;
        number_of_nodes++;	
    }
    else
    {
    	for(mcs_post_node_numbers = HEX_ZERO; mcs_post_node_numbers < ams_num_faults; mcs_post_node_numbers++)
        {
            mcs_post_faultstatus[(number_of_nodes * HEX_FOUR) + HEX_ZERO] = amsbit_faultstatus[(mcs_post_node_numbers * HEX_FOUR) + HEX_ZERO];
            mcs_post_faultstatus[(number_of_nodes * HEX_FOUR) + HEX_ONE] = amsbit_faultstatus[(mcs_post_node_numbers * HEX_FOUR)  + HEX_ONE];
            mcs_post_faultstatus[(number_of_nodes * HEX_FOUR) + HEX_TWO] = amsbit_faultstatus[(mcs_post_node_numbers * HEX_FOUR)  + HEX_TWO];
            mcs_post_faultstatus[(number_of_nodes * HEX_FOUR) + HEX_THREE] = amsbit_faultstatus[(mcs_post_node_numbers * HEX_FOUR)+ HEX_THREE]; 
            mcs_post_faultcode[number_of_nodes] = amsbit_faultcode[mcs_post_node_numbers];
            mcs_post_curstatus[number_of_nodes] = amsbit_curstatus[mcs_post_node_numbers];
            number_of_nodes++;
        }
    }
    for(radio_id = HEX_ONE; radio_id <= TOTAL_RADIOS; radio_id++)
    {
        if(radionumfaults[radio_id] == SUBSYSTEM_NORESPONSE)
        {
            mcs_post_faultstatus[(number_of_nodes * HEX_FOUR) + HEX_ZERO]  = HEX_ZERO;
            mcs_post_faultstatus[(number_of_nodes * HEX_FOUR) + HEX_ONE]   = HEX_ZERO;
            mcs_post_faultstatus[(number_of_nodes * HEX_FOUR) + HEX_TWO]   = HEX_ZERO;
            mcs_post_faultstatus[(number_of_nodes * HEX_FOUR) + HEX_THREE] = Radio_NodeId[radio_id];
            mcs_post_faultcode[number_of_nodes] = HEX_F1;
            mcs_post_curstatus[number_of_nodes] = HEX_FF;
            number_of_nodes++;
        }
        else
        {
            for(mcs_post_node_numbers = HEX_ZERO; mcs_post_node_numbers < radionumfaults[radio_id]; mcs_post_node_numbers++)
            {
                mcs_post_faultstatus[(number_of_nodes * HEX_FOUR) + HEX_ZERO] = Radio_Bit_FaultStatus[radio_id][(mcs_post_node_numbers * HEX_FOUR) + HEX_ZERO];
                mcs_post_faultstatus[(number_of_nodes * HEX_FOUR) + HEX_ONE]  = Radio_Bit_FaultStatus[radio_id][(mcs_post_node_numbers * HEX_FOUR) + HEX_ONE];
                mcs_post_faultstatus[(number_of_nodes * HEX_FOUR) + HEX_TWO]  = Radio_Bit_FaultStatus[radio_id][(mcs_post_node_numbers * HEX_FOUR) + HEX_TWO];
                mcs_post_faultstatus[(number_of_nodes * HEX_FOUR) + HEX_THREE]= Radio_Bit_FaultStatus[radio_id][(mcs_post_node_numbers * HEX_FOUR) + HEX_THREE]; 
                mcs_post_faultcode[number_of_nodes] = Radio_Bit_FaultCode[radio_id][mcs_post_node_numbers];
                mcs_post_curstatus[number_of_nodes] = Radio_Bit_CurStatus[radio_id][mcs_post_node_numbers];
                number_of_nodes++;
            }
        }
    }

    for(mcs_post_node_numbers = HEX_ZERO; mcs_post_node_numbers < mcs_ctrlr_bit_faults; mcs_post_node_numbers++)
    {
        mcs_post_faultstatus[(number_of_nodes * HEX_FOUR) + HEX_ZERO] = mcs_ctrlr_bit_faultstatus[(mcs_post_node_numbers * HEX_FOUR) + HEX_ZERO];
        mcs_post_faultstatus[(number_of_nodes * HEX_FOUR) + HEX_ONE]  = mcs_ctrlr_bit_faultstatus[(mcs_post_node_numbers * HEX_FOUR) + HEX_ONE];
        mcs_post_faultstatus[(number_of_nodes * HEX_FOUR) + HEX_TWO]  = mcs_ctrlr_bit_faultstatus[(mcs_post_node_numbers * HEX_FOUR) + HEX_TWO];
        mcs_post_faultstatus[(number_of_nodes * HEX_FOUR) + HEX_THREE]= mcs_ctrlr_bit_faultstatus[(mcs_post_node_numbers * HEX_FOUR) + HEX_THREE]; 
        mcs_post_faultcode[number_of_nodes] = mcs_ctrlr_bit_faultcode[mcs_post_node_numbers];
        mcs_post_curstatus[number_of_nodes] = mcs_ctrlr_bit_curstatus[mcs_post_node_numbers];
        number_of_nodes++;
    }

    if(number_of_nodes == HEX_ONE)
    {
		number_of_nodes = HEX_ZERO;
    } 
    else
    {
		mcs_post_faultstatus[ZERO]  = HEX_ZERO;
        mcs_post_faultstatus[ONE]   = HEX_ZERO;
        mcs_post_faultstatus[TWO]   = HEX_ZERO;
        mcs_post_faultstatus[THREE] = MCS_NODE_ID;   
        mcs_post_curstatus[ZERO] = mcs_present_stateload;
        if((amsbit_faultstatus[ZERO] == HEX_ZERO) && (amsbit_faultstatus[ONE] == HEX_ZERO) && (amsbit_faultstatus[TWO] == HEX_ZERO)
           && (amsbit_faultstatus[THREE] == AMS_NODE_ID) && (amsbit_faultcode[ZERO] == HEX_ONE)) 
        {
            mcs_post_faultcode[ZERO] = HEX_ONE;
        }
        else
        { 
            mcs_post_faultcode[ZERO] = HEX_TWO;
        }
        
    }
    bzero(&mcs_post_results, sizeof(mcs_post_results));
    Get_Time(&time_send.sec, &time_send.fracsec);
    if(number_of_nodes == HEX_ZERO)
    {
        number_of_nodes = HEX_ONE;
        mcs_post_results.node_info[ZERO].node_identifier[ZERO] 		= HEX_ZERO;
        mcs_post_results.node_info[ZERO].node_identifier[ONE] 		= HEX_ZERO;
        mcs_post_results.node_info[ZERO].node_identifier[TWO] 		= HEX_ZERO;
        mcs_post_results.node_info[ZERO].node_identifier[THREE] 	= MCS_NODE_ID;
        mcs_post_results.node_info[ZERO].no_of_faults 				= HEX_ZERO;
        mcs_post_results.node_info[ZERO].node_fault_details 		= HEX_ZERO;
        mcs_post_results.node_info[ZERO].time_sec 					= time_send.sec;          // needs to be filled
        mcs_post_results.node_info[ZERO].time_fracsec 				= time_send.fracsec;  // needs to be filled 
        mcs_post_results.node_info[ZERO].current_system_state			 = mcs_present_stateload;
        mcs_post_results.node_info[ZERO].current_state_transition_reason = HEX_ZERO;
        
    }
    else
    {
        for(mcs_post_node_numbers = HEX_ZERO; mcs_post_node_numbers < number_of_nodes; mcs_post_node_numbers++)
        {
            mcs_post_results.node_info[mcs_post_node_numbers].node_identifier[ZERO] = mcs_post_faultstatus[(mcs_post_node_numbers * HEX_FOUR) + HEX_ZERO];
            mcs_post_results.node_info[mcs_post_node_numbers].node_identifier[ONE] 	= mcs_post_faultstatus[(mcs_post_node_numbers * HEX_FOUR) + HEX_ONE];
            mcs_post_results.node_info[mcs_post_node_numbers].node_identifier[TWO] 	= mcs_post_faultstatus[(mcs_post_node_numbers * HEX_FOUR) + HEX_TWO];
            mcs_post_results.node_info[mcs_post_node_numbers].node_identifier[THREE]= mcs_post_faultstatus[(mcs_post_node_numbers * HEX_FOUR) + HEX_THREE];
            mcs_post_results.node_info[mcs_post_node_numbers].no_of_faults			 = HEX_ONE;
            mcs_post_results.node_info[mcs_post_node_numbers].node_fault_details 	 = mcs_post_faultcode[mcs_post_node_numbers];
            mcs_post_results.node_info[mcs_post_node_numbers].time_sec 				 = time_send.sec;          
            mcs_post_results.node_info[mcs_post_node_numbers].time_fracsec 			 = time_send.fracsec;  
            mcs_post_results.node_info[mcs_post_node_numbers].current_system_state   = mcs_post_curstatus[mcs_post_node_numbers];
            mcs_post_results.node_info[mcs_post_node_numbers].current_state_transition_reason = HEX_ZERO;
        }
    }
    mcs_post_results.message_code 			= MCS_MSC_POST_RESULT;
    mcs_post_results.time_sec 				= time_send.sec;          
    mcs_post_results.time_fracsec 			= time_send.fracsec;  
    mcs_post_results.hardware_version 		= CONTROLLER_HW_VERSION;
    strcpy(mcs_post_results.software_version, CONTROLLER_SW_VERSION);
    mcs_post_results.no_of_nodes = number_of_nodes;
    postresults_size = DEC_44 + (mcs_post_results.no_of_nodes * DEC_20); // 20 in the original code    
    mcs_post_results.message_size = postresults_size - HEX_FOUR;
    /* sending post results to MSC */
    if((send(pfmbit_sockfd, (void *)&mcs_post_results, sizeof(mcs_post_results), HEX_ZERO)) == ERROR)
    {
		;
    }
    else
    {   
        ;          
    }
    bzero(&mcs_post_ack, sizeof(mcs_post_ack));
    /* Receive POST Acknowledgement from MSC */
    if((recv(pfmbit_sockfd, (void *)&mcs_post_ack, sizeof(mcs_post_ack), HEX_ZERO)) == ERROR)
    {
        ;
    }
    else 
    {
		;              
    }    
    is_firstpfm_loaded = HEX_ZERO;
    is_firstpfm_recvd  = HEX_ZERO;     
    //while(TRUE)
	for(;;)
    {
        if(mcs_post_ack.post_status == HEX_FF)     
        {            
            /* Control commands socket connection with MSC */
            sleep(HEX_THREE);                       
            ctrl_msc_socket(HEX_ONE);
            if(pfmbit_sockfd == ERROR || ctrl_msc_sockfd == ERROR)
            {
 		
                break;
            }
            else
	        { 
               ;
            } 
            //usleep(SEC * 3);  // not in original code  
            sleep(HEX_THREE);          
            /* Control commands socket connection with MSC */
            ctrl_mcs_socket(HEX_ONE);
            if(pfmbit_sockfd == ERROR || ctrl_msc_sockfd == ERROR || ctrl_mcs_sockfd == ERROR)
            {
 		
                break;
            }
            else
	        { 
               ;
            }             
            
            mcs_present_stateload = HEX_ONE;
            //bfill(radio_present_stateload, sizeof(radio_present_stateload), 1);
            memset(radio_present_stateload, HEX_ONE, sizeof(radio_present_stateload));
            
            for(radio_id = HEX_ONE; radio_id <= HEX_ONE/*TOTAL_RADIOS*/; radio_id++)
            {
                if(radio_present_state[radio_id] ==  HEX_ONE && radio_commstatus[radio_id] == HEX_ONE)  // Radio is On and Post Success-->Init State
                {
                    radio_present_state[radio_id] = radio_present_statestore[radio_id] = mcs_present_stateload;
                }
                else if(radio_present_state[radio_id] ==  HEX_FIVE && radio_commstatus[radio_id] == HEX_ZERO)  // Radio is Off --> Off state
                {
                    radio_present_state[radio_id] = radio_present_statestore[radio_id] = HEX_FIVE;
                }
            } 
            /* Intercom and TR Status */
            send_mcs_status();        /* sending init state */   
            res = pthread_create(&pfmbit_thread, NULL, pfm_bit_command, NULL);                     
            usleep(SEC * HEX_TWO);       // taskDelay(sysClkRateGet() * 2);
            pfm_break_loop = HEX_ZERO;    
            //while(TRUE)
			for(;;)
            {
                if(is_firstpfm_recvd == HEX_ONE)
                {                   
                    memset(radio_tr_comm_stop, HEX_ONE, sizeof(radio_tr_comm_stop));
                    usleep(SEC * HEX_ONE);  //taskDelay(sysClkRateGet());
                    usleep(SEC * HEX_ONE);  //taskDelay(sysClkRateGet());                   
                    break;                                      
                }
                else
                {
                    if(pfmbit_sockfd == ERROR || ctrl_msc_sockfd == ERROR || ctrl_mcs_sockfd == ERROR)
                    {                       
                        break;
                    }
                    
                    if(pfm_break_loop == DEC_18)
                    {                       
                        stop_pfm_cmd = HEX_ONE;
                        send_alert_Msg(HEX_ZERO, PFM_NOT_LOADED_TO_RADIO);
                        usleep(SEC * DEC_500);   //taskDelay(sysClkRateGet() * 0.5);                        
                        for(radio_id = HEX_ONE; radio_id <= TOTAL_RADIOS; radio_id)
                        {
                            if(is_radio_firstpfm_present[radio_id] == HEX_ZERO && radio_present_state[radio_id] == HEX_ONE)
                            {
                                send_alert_Msg(radio_id, PFM_NOT_LOADED_TO_RADIO);
                                usleep(SEC * DEC_500);   //taskDelay(sysClkRateGet() * 0.5);
                            }
                            is_radio_firstpfm_present[radio_id] = HEX_ONE;
                        }                        
                        is_firstpfm_recvd = HEX_ONE;
                        is_firstpfm_loaded = HEX_ONE;                        
                    }
                    else
                    {
                        usleep(SEC * DEC_10);  //taskDelay(sysClkRateGet() * 10);
                        pfm_break_loop++;
                    }
                }
            }
            //while(TRUE)
			for(;;)
            {
                if(is_firstpfm_loaded == HEX_ONE)
                {                
                   
                    res = pthread_create(&ctrl_ap_thread, NULL, control_ap_relay, NULL);  
                    mcs_present_stateload = HEX_THREE;
                    //bfill(radio_present_stateload, sizeof(radio_present_stateload), 3);  //changed from 1 to 3
                    memset(radio_present_stateload, HEX_THREE, sizeof(radio_present_stateload));                    
                    for(radio_id = HEX_ONE; radio_id <= HEX_ONE/*TOTAL_RADIOS*/; radio_id)
                    {
                        if(radio_present_state[radio_id] == HEX_ONE && firsttime_reinit_set[radio_id] == HEX_ZERO)
                        {
                            radio_present_state[radio_id] = radio_present_statestore[radio_id] = mcs_present_stateload;
                        }
                    } 
                    /* InterCom and TR Status */                    
                    mcs_sent_status = HEX_ZERO;                    
                    send_mcs_status();        /* sending operational state */
                    mcs_sent_status = HEX_ONE;
                    usleep(SEC * HEX_ONE);   // taskDelay(sysClkRateGet());            
                    stop_pfm_cmd = HEX_ZERO;
                    break;
                }
                else
                {
                    if(pfmbit_sockfd == ERROR || ctrl_msc_sockfd == ERROR || ctrl_mcs_sockfd == ERROR)
                    {
                        break;
                    }
                    usleep(SEC * HEX_ONE);  //taskDelay(sysClkRateGet());                    
                }
            }            
            break;                                 
        }
        else
        {
            if(pfmbit_sockfd == ERROR)
            {
                break;
            }
            else
            {
                ;
            } 
            
            usleep(DEC_225000);  // verify delay in original code 
            bzero(&mcs_post_results, sizeof(mcs_post_results));            
            if(number_of_nodes == HEX_ZERO)
    	    {
        	number_of_nodes 										= HEX_ONE;
        	mcs_post_results.node_info[ZERO].node_identifier[ZERO]  = HEX_ZERO;
        	mcs_post_results.node_info[ZERO].node_identifier[ONE]   = HEX_ZERO;
        	mcs_post_results.node_info[ZERO].node_identifier[TWO]   = HEX_ZERO;
        	mcs_post_results.node_info[ZERO].node_identifier[THREE] = MCS_NODE_ID;
        	mcs_post_results.node_info[ZERO].no_of_faults 			= HEX_ZERO;
        	mcs_post_results.node_info[ZERO].node_fault_details 	= HEX_ZERO;
        	mcs_post_results.node_info[ZERO].time_sec 				= time_send.sec;          // needs to be filled
        	mcs_post_results.node_info[ZERO].time_fracsec 			= time_send.fracsec;  // needs to be filled 
        	mcs_post_results.node_info[ZERO].current_system_state 	= mcs_present_stateload;
        	mcs_post_results.node_info[ZERO].current_state_transition_reason = HEX_ZERO;
        
    	    }
    	    else
            {
        	for(mcs_post_node_numbers = HEX_ZERO; mcs_post_node_numbers < number_of_nodes; mcs_post_node_numbers++)
        	{
            	    mcs_post_results.node_info[mcs_post_node_numbers].node_identifier[ZERO] = mcs_post_faultstatus[(mcs_post_node_numbers * HEX_FOUR) + HEX_ZERO];
             	    mcs_post_results.node_info[mcs_post_node_numbers].node_identifier[ONE]  = mcs_post_faultstatus[(mcs_post_node_numbers * HEX_FOUR) + HEX_ONE];
					mcs_post_results.node_info[mcs_post_node_numbers].node_identifier[TWO]  = mcs_post_faultstatus[(mcs_post_node_numbers * HEX_FOUR) + HEX_TWO];
            	    mcs_post_results.node_info[mcs_post_node_numbers].node_identifier[THREE]= mcs_post_faultstatus[(mcs_post_node_numbers * HEX_FOUR) + HEX_THREE];
            	    mcs_post_results.node_info[mcs_post_node_numbers].no_of_faults			= HEX_ONE;
            	    mcs_post_results.node_info[mcs_post_node_numbers].node_fault_details 	= mcs_post_faultcode[mcs_post_node_numbers];
             	    mcs_post_results.node_info[mcs_post_node_numbers].time_sec 				= time_send.sec;          // needs to be filled
            	    mcs_post_results.node_info[mcs_post_node_numbers].time_fracsec 			= time_send.fracsec;  // needs to be filled 
            	    mcs_post_results.node_info[mcs_post_node_numbers].current_system_state  = mcs_post_curstatus[mcs_post_node_numbers];
            	    mcs_post_results.node_info[mcs_post_node_numbers].current_state_transition_reason = HEX_ZERO;
                }
            }
    	    mcs_post_results.message_code 				= MCS_MSC_POST_RESULT;
            mcs_post_results.time_sec 					= time_send.sec;          // needs to be filled
    	    mcs_post_results.time_fracsec 				= time_send.fracsec;  // needs to be filled 
    	    mcs_post_results.hardware_version 			= CONTROLLER_HW_VERSION;
    	    strcpy(mcs_post_results.software_version, CONTROLLER_SW_VERSION);
    	    mcs_post_results.no_of_nodes 				= number_of_nodes;    
    	    postresults_size = DEC_44 + (mcs_post_results.no_of_nodes * DEC_20); //20 in the original code    
    	    mcs_post_results.message_size =  postresults_size - HEX_FOUR;    
       	    /* sending post results to MSC */
    	    if((send(pfmbit_sockfd, (void *)&mcs_post_results, sizeof(mcs_post_results), HEX_ZERO)) == ERROR)
    	    {
				;
    	    }
    	    else
    	    {   
				;          
    	    }
    	    bzero(&mcs_post_ack, sizeof(mcs_post_ack));
    	    //Receive acknowledgement from server
    	    if((recv(pfmbit_sockfd, (void *)&mcs_post_ack, sizeof(mcs_post_ack), HEX_ZERO)) == ERROR)
       	    {
       	        ;
    	    }
    	    else 
    	    {
				;              
    	    }               
        } 
    }     
    return OK;
                
}
