
/*
 ** HHdr-beg *****************************************************
 **
 ** Copyright Trident Infosol 2018
 ** All Rights Reserved
 **
 ** Trident infosol Proprietary
 **
 ** Include File name: mcs_ctrlr.h
 **
 ** Purpose:           
 **
 ** Modification History:
 ** PCR                 Date     Name       Comment
 ** ---                 ----     ----       -------
 ** HPrj-beg *****************************************************
 **
 ** HPrj-end *****************************************************
 ** HHdr-end *****************************************************
 */

#ifndef MCS_CTRLR_H_
#define MCS_CTRLR_H_

#include <stdio.h>
#include <string.h>    
#include <stdlib.h>
#include <fcntl.h> 
#include <termios.h>
#include <unistd.h>
#include <time.h>
#include <pthread.h>
#include <sys/select.h>
#include <math.h>
#include <sys/ioctl.h>
#include <ctype.h>
#include <stdint.h>
#include <sys/time.h>
#include <signal.h>
#include <semaphore.h>
#include <mqueue.h>
#include <sys/socket.h>
#include <arpa/inet.h>

#include "config.h"
#include "bit.h"
#include "util.h"
#include "radio_commands.h"

char *port_name = "/dev/ttyXR";

uint32_t n = 0, i = 0;
struct timespec delay;

/* mcs controller fault codes */
uint8_t mcs_cntlr_nodefaultcode[TOTAL_MCS_CTRLR_NODES] = {0x71, 0x72, 0x73, 0x74, 0xE1, 0xE2, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7A, 0xE3};

/* mcs controller node identifier */
uint8_t mcs_cntlr_nodeidbuf[TOTAL_MCS_CTRLR_NODES * 4] =  
                               {0, MEMORY, BOARD, MCS_CONTROLLER_NODE_ID,      // sub node level3, sub node level2, sub node level1, main node
                                0, PCI_BRIDGE, BOARD, MCS_CONTROLLER_NODE_ID,
				0, PCI_BUS_INTERFACE, BOARD, MCS_CONTROLLER_NODE_ID,
				0, HARDDISK, STORAGE_MODULE, MCS_CONTROLLER_NODE_ID,	
				0, PRI_ETHERNET, ETHERNET_INTERFACE, MCS_CONTROLLER_NODE_ID,
                                0, UART, RADIO_INTERFACE, MCS_CONTROLLER_NODE_ID,
                                0, TOD, RADIO_INTERFACE, MCS_CONTROLLER_NODE_ID,
				0, PTT1, RADIO_INTERFACE, MCS_CONTROLLER_NODE_ID,
                                0, PTT2, RADIO_INTERFACE, MCS_CONTROLLER_NODE_ID,
                                0, PTT3, RADIO_INTERFACE, MCS_CONTROLLER_NODE_ID,
                                0, PTT4, RADIO_INTERFACE, MCS_CONTROLLER_NODE_ID,
                                0, PTT5, RADIO_INTERFACE, MCS_CONTROLLER_NODE_ID,
                                0, SEC_ETHERNET, RADIO_INTERFACE, MCS_CONTROLLER_NODE_ID};

/* radio controller fault codes */
uint8_t Radio_NodeFaultCode[TOTAL_RADIOS][TOTAL_RADIO_NODES] =
					       {{0x21, 0x22, 0x23, 0x24, 0x25, 0xC1, 0xC2, 0x26, 0xC3},  // V_UHF_RADIO_SYSTEM_1 fault codes
                                             	{0x31, 0x32, 0x33, 0x34, 0x35, 0xC4, 0xC5, 0x36, 0xC6},  // V_UHF_RADIO_SYSTEM_2 fault codes
                                             	{0x41, 0x42, 0x43, 0x44, 0x45, 0xC7, 0xC8, 0x46, 0xC9},  // V_UHF_RADIO_SYSTEM_3 fault codes
                                             	{0x51, 0x52, 0x53, 0x54, 0x55, 0xCA, 0xCB, 0x56, 0xCC},  // V_UHF_RADIO_SYSTEM_4 fault codes
                                             	{0x61, 0x62, 0x63, 0x64, 0x65, 0xCD, 0xCE, 0x66, 0xCF}}; // V_UHF_RADIO_SYSTEM_5 fault codes

/* radio controller node identifier */
uint8_t Radio_NodeIdBuf[TOTAL_RADIOS][TOTAL_RADIO_NODES * 4] = 
					{{0, 0, CONTROLLER, V_UHF_RADIO_SYSTEM_1_NODE_ID,   // sub node level3, sub node level2, sub node level1, main node
                                    	  0, 0, SYNTHESIZER, V_UHF_RADIO_SYSTEM_1_NODE_ID,
                                    	  0, 0, TRANSMITTER, V_UHF_RADIO_SYSTEM_1_NODE_ID,
                                    	  0, 0, MAIN_RECEIVER, V_UHF_RADIO_SYSTEM_1_NODE_ID,
                                          0, 0, GUARD_RECEIVER, V_UHF_RADIO_SYSTEM_1_NODE_ID,
                                       	  0, 0, AUDIO_SECTION, V_UHF_RADIO_SYSTEM_1_NODE_ID,
                                       	  0, 0, POWER_REGULATOR, V_UHF_RADIO_SYSTEM_1_NODE_ID,
                                       	  0, 0, HPA_ICFPA, V_UHF_RADIO_SYSTEM_1_NODE_ID,
                                       	  0, 0, RF_CABLE_ANTENNA, V_UHF_RADIO_SYSTEM_1_NODE_ID},
                                      	 {0, 0, CONTROLLER, V_UHF_RADIO_SYSTEM_2_NODE_ID,   // sub node level3, sub node level2, sub node level1, main node
                                       	  0, 0, SYNTHESIZER, V_UHF_RADIO_SYSTEM_2_NODE_ID,
                                       	  0, 0, TRANSMITTER, V_UHF_RADIO_SYSTEM_2_NODE_ID,
                                       	  0, 0, MAIN_RECEIVER, V_UHF_RADIO_SYSTEM_2_NODE_ID,
                                       	  0, 0, GUARD_RECEIVER, V_UHF_RADIO_SYSTEM_2_NODE_ID,
                                       	  0, 0, AUDIO_SECTION, V_UHF_RADIO_SYSTEM_2_NODE_ID,
                                       	  0, 0, POWER_REGULATOR, V_UHF_RADIO_SYSTEM_2_NODE_ID,
                                       	  0, 0, HPA_ICFPA, V_UHF_RADIO_SYSTEM_2_NODE_ID,
                                       	  0, 0, RF_CABLE_ANTENNA, V_UHF_RADIO_SYSTEM_2_NODE_ID},
                                      	 {0, 0, CONTROLLER, V_UHF_RADIO_SYSTEM_3_NODE_ID,   // sub node level3, sub node level2, sub node level1, main node
                                       	  0, 0, SYNTHESIZER, V_UHF_RADIO_SYSTEM_3_NODE_ID,
                                       	  0, 0, TRANSMITTER, V_UHF_RADIO_SYSTEM_3_NODE_ID,
                                       	  0, 0, MAIN_RECEIVER, V_UHF_RADIO_SYSTEM_3_NODE_ID,
                                       	  0, 0, GUARD_RECEIVER, V_UHF_RADIO_SYSTEM_3_NODE_ID,
                                       	  0, 0, AUDIO_SECTION, V_UHF_RADIO_SYSTEM_3_NODE_ID,
                                       	  0, 0, POWER_REGULATOR, V_UHF_RADIO_SYSTEM_3_NODE_ID,
                                       	  0, 0, HPA_ICFPA, V_UHF_RADIO_SYSTEM_3_NODE_ID,
                                          0, 0, RF_CABLE_ANTENNA, V_UHF_RADIO_SYSTEM_3_NODE_ID},
                                      	 {0, 0, CONTROLLER, V_UHF_RADIO_SYSTEM_4_NODE_ID,   // sub node level3, sub node level2, sub node level1, main node
                                       	  0, 0, SYNTHESIZER, V_UHF_RADIO_SYSTEM_4_NODE_ID,
                                       	  0, 0, TRANSMITTER, V_UHF_RADIO_SYSTEM_4_NODE_ID,
                                       	  0, 0, MAIN_RECEIVER, V_UHF_RADIO_SYSTEM_4_NODE_ID,
                                       	  0, 0, GUARD_RECEIVER, V_UHF_RADIO_SYSTEM_4_NODE_ID,
                                       	  0, 0, AUDIO_SECTION, V_UHF_RADIO_SYSTEM_4_NODE_ID,
                                       	  0, 0, POWER_REGULATOR, V_UHF_RADIO_SYSTEM_4_NODE_ID,
                                       	  0, 0, HPA_ICFPA, V_UHF_RADIO_SYSTEM_4_NODE_ID,
                                       	  0, 0, RF_CABLE_ANTENNA, V_UHF_RADIO_SYSTEM_4_NODE_ID},
                                      	 {0, 0, CONTROLLER, V_UHF_RADIO_SYSTEM_5_NODE_ID,   // sub node level3, sub node level2, sub node level1, main node
                                       	  0, 0, SYNTHESIZER, V_UHF_RADIO_SYSTEM_5_NODE_ID,
                                       	  0, 0, TRANSMITTER, V_UHF_RADIO_SYSTEM_5_NODE_ID,
                                       	  0, 0, MAIN_RECEIVER, V_UHF_RADIO_SYSTEM_5_NODE_ID,
                                       	  0, 0, GUARD_RECEIVER, V_UHF_RADIO_SYSTEM_5_NODE_ID,
                                       	  0, 0, AUDIO_SECTION, V_UHF_RADIO_SYSTEM_5_NODE_ID,
                                       	  0, 0, POWER_REGULATOR, V_UHF_RADIO_SYSTEM_5_NODE_ID,
                                       	  0, 0, HPA_ICFPA, V_UHF_RADIO_SYSTEM_5_NODE_ID,
                                          0, 0, RF_CABLE_ANTENNA, V_UHF_RADIO_SYSTEM_5_NODE_ID}};  

/* ams fault codes */
uint8_t ams_nodefaultcode[21] = 
{
    0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08,0x09,0x0A,0x0B,0x0C,0x0D,0x0E,0x0F,0x10,0x11,0x12,0x13,0x14,0x15	
};

/* ams node identifier */
uint8_t ams_nodeidbuf[84] = 
{
    0,0,1,AMS_NODE_ID, 0,1,2,AMS_NODE_ID, 0,2,2,AMS_NODE_ID, 0,3,2,AMS_NODE_ID, 0,4,2,AMS_NODE_ID, 0,1,3,AMS_NODE_ID,
    0,2,3,AMS_NODE_ID, 0,3,3,AMS_NODE_ID, 0,4,3,AMS_NODE_ID, 0,1,4,AMS_NODE_ID, 0,2,4,AMS_NODE_ID, 0,3,4,AMS_NODE_ID,
    0,4,4,AMS_NODE_ID, 0,1,5,AMS_NODE_ID, 0,2,5,AMS_NODE_ID, 0,3,5,AMS_NODE_ID, 0,4,5,AMS_NODE_ID, 0,1,6,AMS_NODE_ID,
    0,2,6,AMS_NODE_ID, 0,3,6,AMS_NODE_ID, 0,4,6,AMS_NODE_ID
};

uint8_t ACP_POWER_VALUE[8] = 
{
    0x00,0xFE,0xFC,0xF8,0xF0,0XE0,0xC0,0x80
};


uint8_t mcs_ctrlr_bit_faults = 0, mcs_ctrlr_bit_faultstatus[TOTAL_MCS_CTRLR_NODES * 4] = "", mcs_ctrlr_bit_faultcode[TOTAL_MCS_CTRLR_NODES] = "",
      mcs_ctrlr_bit_curstatus[TOTAL_MCS_CTRLR_NODES] = "";

uint8_t mcs_present_stateload = 0;

//Global declaration of socket id
int32_t pfmbit_sockfd = ERROR, ctrl_msc_sockfd = ERROR, ctrl_mcs_sockfd = ERROR;   

uint8_t intercom_res_amscc = 0;

uint8_t cbit_period_recv = 0;

/*Radio 1-5*/
uint8_t radio_securemode[TOTAL_RADIOS_PLUS_ONE] = "", radio_guardchannel[TOTAL_RADIOS_PLUS_ONE] = "";
uint8_t radiopfm_guardchannel[TOTAL_RADIOS_PLUS_ONE] = "";
uint8_t relaypairconfig_pfmstore[10] = "";
uint8_t mcspfm_underprogess = 0;
uint8_t radiopfm_underprogress[TOTAL_RADIOS_PLUS_ONE] = "";
uint8_t numof_blankfreq = 0;
uint8_t hq_param_fha[30] = "";
uint8_t rpw_param_fhb[70] = "";
uint8_t is_radio_firstpfm_present[TOTAL_RADIOS_PLUS_ONE] = "";
uint8_t is_firstpfm_loaded = 0;
//int32_t MCS_PfmTId = ERROR;
//int32_t PfmBitTId = ERROR;
uint8_t radiopfm_securemode[TOTAL_RADIOS_PLUS_ONE] = "";
uint8_t radio_ap_present[TOTAL_RADIOS_PLUS_ONE] = "";
uint8_t radio_fh_notpossible[TOTAL_RADIOS_PLUS_ONE] = "";
uint8_t radio_emgactive_aftersecure[TOTAL_RADIOS_PLUS_ONE] = "";
uint8_t rpw_re_operation_complete[TOTAL_RADIOS_PLUS_ONE] = "";
uint8_t storesetup_setstatus[TOTAL_RADIOS_PLUS_ONE][4];
uint8_t radio_standby_status[TOTAL_RADIOS_PLUS_ONE] = "";
uint8_t mcs_sent_status = 0;
uint8_t SkipFirstPresetFreq[TOTAL_RADIOS_PLUS_ONE] = "";

uint8_t stop_pfm_cmd = 0;

uint8_t radio_emg_active_after_secure[TOTAL_RADIOS_PLUS_ONE] = "", radio_cbitfirsttime_after_fh[TOTAL_RADIOS_PLUS_ONE] = "";
uint8_t radio_rpw_operation_complete[TOTAL_RADIOS_PLUS_ONE] = "";

// int32_t radio_sockid[TOTAL_RADIOS_PLUS_ONE];
int32_t radio_serialfd[TOTAL_RADIOS_PLUS_ONE];

uint8_t radionumfaults[TOTAL_RADIOS_PLUS_ONE];
uint8_t Radio_NodeId[TOTAL_RADIOS_PLUS_ONE] = {0, V_UHF_RADIO_SYSTEM_1_NODE_ID, V_UHF_RADIO_SYSTEM_2_NODE_ID, V_UHF_RADIO_SYSTEM_3_NODE_ID,
                                                  V_UHF_RADIO_SYSTEM_4_NODE_ID, V_UHF_RADIO_SYSTEM_5_NODE_ID };

uint8_t Radio_Bit_FaultStatus[TOTAL_RADIOS_PLUS_ONE][36];
uint8_t Radio_Bit_FaultCode[TOTAL_RADIOS_PLUS_ONE][9];
uint8_t Radio_Bit_CurStatus[TOTAL_RADIOS_PLUS_ONE][36];

uint8_t cbit_period_recv;
uint8_t bit_nodeidentifier;
uint8_t ams_ibit_node[4] = "";
uint8_t radio_ibit_node[TOTAL_RADIOS_PLUS_ONE][4];
uint8_t controller_ibit_node[4] = "";

int32_t Radio_Ctrl_TId[TOTAL_RADIOS_PLUS_ONE], Radio_PfmTId[TOTAL_RADIOS_PLUS_ONE], Radio_IBit_TId[TOTAL_RADIOS_PLUS_ONE];
int32_t Radio_CBit_TId[TOTAL_RADIOS_PLUS_ONE], Radio_MBit_TId[TOTAL_RADIOS_PLUS_ONE], Radio_Post_TId[TOTAL_RADIOS_PLUS_ONE];
int32_t Radio_State_TId[TOTAL_RADIOS_PLUS_ONE], Radio_TrComm_TId[TOTAL_RADIOS_PLUS_ONE], Radio_CommCheck_TId[TOTAL_RADIOS_PLUS_ONE];
int32_t Radio_ReInit_TId[TOTAL_RADIOS_PLUS_ONE], Radio_Ap_Erase_TId[TOTAL_RADIOS_PLUS_ONE];

pthread_t radio_post_thread[TOTAL_RADIOS_PLUS_ONE];     /* equivalent to Radio_Post_TId[TOTAL_RADIOS_PLUS_ONE] in org code   */
pthread_t radio_tr_comm_thread[TOTAL_RADIOS_PLUS_ONE];   /* equivalent to Radio_TrComm_TId[TOTAL_RADIOS_PLUS_ONE] in org code */
pthread_t radio_pfmload_thread[TOTAL_RADIOS_PLUS_ONE];
pthread_t radioctrl_thread[TOTAL_RADIOS_PLUS_ONE];
pthread_t radio_ap_erase_thread[TOTAL_RADIOS_PLUS_ONE];
pthread_t radio_state_change_thread[TOTAL_RADIOS_PLUS_ONE];

pthread_t pfmbit_thread   = 0;
pthread_t mcs_pfm_thread  = 0;
pthread_t ctrl_ap_thread  = 0;
pthread_t ap_erase_thread = 0;
pthread_t ams_post_thread  = 0;
pthread_t mcs_state_change_thread = 0;


//int32_t RadioPost_ThreadId[TOTAL_RADIOS_PLUS_ONE];
//int32_t radio_trcomm_threadid[TOTAL_RADIOS_PLUS_ONE];  
//int32_t pfmbit_threadid  = ERROR;
//int32_t mcs_pfm_threadid = ERROR;
//int32_t radio_pfmload_threadid[TOTAL_RADIOS_PLUS_ONE];
//int32_t ctrl_ap_threadid = ERROR;
//int32_t ap_erase_threadid = ERROR;
//int32_t radioctrl_threadid[TOTAL_RADIOS_PLUS_ONE];
//int32_t radio_ap_erase_threadid[TOTAL_RADIOS_PLUS_ONE];
//int32_t mcs_state_change_threadid = ERROR;
//int32_t radio_state_change_threadid[TOTAL_RADIOS_PLUS_ONE];

uint32_t radio_loadedfreq[TOTAL_RADIOS_PLUS_ONE];
int32_t  radio_ctrl_freq_val[TOTAL_RADIOS_PLUS_ONE], radio_present_state[TOTAL_RADIOS_PLUS_ONE];
int32_t  radio_present_statestore[TOTAL_RADIOS_PLUS_ONE], radioloaded_833_freqstatus[TOTAL_RADIOS_PLUS_ONE]; 

uint32_t freqchan_buf[100], blankfreq_chanbuf[200];

uint8_t radiostored_los_param[TOTAL_RADIOS_PLUS_ONE][10];
uint8_t radio_present_stateload[TOTAL_RADIOS_PLUS_ONE] = "";

uint8_t hq_fha_guardcontrol[TOTAL_RADIOS_PLUS_ONE], radio_radiation_status[TOTAL_RADIOS_PLUS_ONE];

uint8_t radio_tr_comm_stop[TOTAL_RADIOS_PLUS_ONE], radio_isfirst_tr_complete[TOTAL_RADIOS_PLUS_ONE], radio_init_status[TOTAL_RADIOS_PLUS_ONE];
uint8_t radio_commstatus[TOTAL_RADIOS_PLUS_ONE], radio_param_setstatus[TOTAL_RADIOS_PLUS_ONE];  

uint8_t  radioload_securemode[TOTAL_RADIOS_PLUS_ONE] = "";
uint32_t radioctrl_freqvalue[TOTAL_RADIOS_PLUS_ONE];
uint8_t  radioload_guardchannel[TOTAL_RADIOS_PLUS_ONE] = "";
uint8_t  radioctrl_los_param[TOTAL_RADIOS_PLUS_ONE][10];
uint8_t  radioctrl_833_freqstatus[TOTAL_RADIOS_PLUS_ONE] = "";
uint8_t  radio_tod_present_status[TOTAL_RADIOS_PLUS_ONE] = "";  
uint8_t  radio_tod_setstatus[TOTAL_RADIOS_PLUS_ONE] = "";  
uint8_t  radio_rpw_setstatus[TOTAL_RADIOS_PLUS_ONE] = "";
uint8_t  storesetup_paramstore[TOTAL_RADIOS_PLUS_ONE+2] = "";
int32_t  RadioCtrlTId[TOTAL_RADIOS_PLUS_ONE];

uint8_t ntp_task_close = 0;

uint8_t is_firstpfm_recvd = 0;
uint8_t firsttime_reinit_set[TOTAL_RADIOS_PLUS_ONE] = "";


uint8_t skip_first_presetreq[TOTAL_RADIOS_PLUS_ONE];

int32_t NTP_Time_TId = ERROR;
int32_t IPC_TId      = ERROR;
//int32_t ams_post_threadid = ERROR;

int32_t AmsRelayTId = ERROR;


int8_t *msgQname = "/msgqueue";

uint8_t ams_msg_sent_status = 0;
uint8_t AMS_Present_State = 0;
uint8_t AMS_Present_StateStore = 0;

int32_t MCS_Maintenance_TId = ERROR;

uint8_t amsbit_faultstatus[84] = "", amsbit_faultcode[21] = "", amsbit_curstatus[21] = "", AmsBIT_FaultCompare[84] = "";
uint8_t ams_num_faults = 0;
int32_t relay_set_status = 0;

//SEM_ID restart_syncsem = NULL, ams_syncsem = NULL, ams_cbit_syncsem = NULL;
//SEM_ID mcs_cbit_syncsem = NULL, ntp_time_sync_sem = NULL, ntp_time_sync_sem = NULL; 

sem_t Ams_Cbit_SyncSem;

sem_t radio_syncsem[TOTAL_RADIOS_PLUS_ONE], radio_cbit_syncsem[TOTAL_RADIOS_PLUS_ONE];

/* Relay */
uint8_t relaypairconfig[10] = "";
uint8_t relay_pairconfig_store[10] = "";

//int32_t control_ap_relay();
void *control_ap_relay();
void *radio_ap_erase(void *);
void *radio_ctrlcmd(void *);
int32_t send_radioconfig_ack(uint8_t *);
void *ap_erase();
int32_t send_ap_status();
int32_t send_alert_Msg(uint8_t, uint8_t);
int32_t send_mcs_status();

void *mcs_state_change();
void *radio_state_change(void *);

uint32_t mcs_ctrlr(uint32_t);
int32_t los_params_store(void);
void mcs_ctrlr_node_post();

void *ams_post();                                 /* thread */
int32_t get_ams_post(uint8_t *, uint8_t *);

/* radio post */
void *radio_post(void *);                         /* thread */
void *pfm_bit_command();
void *mcs_pfmload();
void *radio_pfmload();

int32_t  Open_Serial_Port(uint8_t, int32_t *, uint8_t);
uint32_t serial_read(int32_t, uint8_t *, uint8_t *);
uint8_t  frequency_roundof(float, uint32_t *);
uint32_t dec_to_hex(uint32_t, uint8_t *);
uint32_t dec_to_bin(uint32_t, uint8_t *);

uint8_t comm_check(uint8_t, int32_t);

uint8_t  radio_init(uint8_t, int32_t);
void    *radio_tr_comm_check(void *);                   /* thread */
uint8_t  clear_nvm(uint8_t, int32_t);
uint8_t  radiation_status(uint8_t, int32_t);
uint8_t  radiation_on_off(uint8_t , int32_t , uint8_t (*)[]);
uint8_t  emergency_guard(uint8_t, int32_t, uint8_t (*)[]);
uint8_t  activate_los_preset(uint8_t , int32_t );
uint8_t  store_setup_los(uint8_t, int32_t, uint8_t (*)[], uint8_t (*)[], uint8_t);
uint8_t  squelch_th(uint8_t , int32_t , uint8_t (*)[]);
uint8_t  activate_setup(uint8_t, int32_t, uint8_t);
uint8_t  preset_request(uint8_t  , int32_t  , uint8_t *);
uint8_t  hq_erase(uint8_t , int32_t );
uint8_t  rpw_erase(uint8_t, int32_t);
uint8_t  radio_tod_load(int32_t, uint8_t);
uint8_t  radio_tod_status(int32_t, uint8_t);
uint8_t  hq_param_load(uint8_t, int32_t, uint8_t *);
uint8_t  rpw_param_load(uint8_t, int32_t, uint8_t *);
uint8_t  rpw_param_load_reinit(uint8_t, int32_t);
uint8_t  hq_mode_operation(uint8_t, int32_t, uint8_t *);
uint8_t  rpw_mode_operation(uint8_t, int32_t, uint8_t *);


#endif 
