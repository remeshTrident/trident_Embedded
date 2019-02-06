
/*
 ** HHdr-beg *****************************************************
 **
 ** Copyright Trident Infosol 2018
 ** All Rights Reserved
 **
 ** Trident infosol Proprietary
 **
 ** Include File name: config.h
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


#ifndef CONFIG_H_
#define CONFIG_H_



#define DEBUG 
#define HI_PRIO 31 
#define ERROR -1 
#define SEC      100000
#define MILISEC  100

#define VAIU_BIT_SEND_PORT 1101
#define VAIU_BIT_RECV_PORT 1100
#define VAIU_BIT_SERVER_NAME "10.10.10.3"
#define OK 0
#define FAILURE_STATUS 0x00
#define SUCCESS_STATUS 0xFF
#define TOTAL_RADIOS           5
#define TOTAL_RADIOS_PLUS_ONE  7
#define TOTAL_NODES            80
#define TOTAL_MCS_CTRLR_NODES  13
#define TOTAL_RADIO_NODES      9

/* Alert Messages */
#define RADIO_OFF                            0X01
#define RADIO_NOT_AVAILABLE_FOR_CONTROL      0X02
#define RADIO_IN_RECV_ONLY_FREQUENCY         0X03
#define RADIO_NO_TX_CONDITION                0X04
#define RADIO_NOT_AVAILABLE_FOR_FHA_FHB_MODE 0X05
#define RADIO_IN_EMG_MODE                    0X06
#define RADIO_AP_LOAD_FAILURE                0X07
#define RADIO_TOD_LOAD_FAILURE               0X08
#define IBIT_MBIT_UNDER_PROGRESS             0X09
#define NTP_FAILURE                          0X0A
#define VAIU_POWER_SUPPLY_FAILURE            0X0B
#define PFM_NOT_LOADED_TO_RADIO              0X0C
#define NUM_ACP 0x05
#define PFMBIT_PORT  6205
#define CTRLMSC_PORT 6206
#define CTRLMCS_PORT 6210 
#define MSC_PFMBIT_SERVER "192.168.100.100" 
#define MSC_CTRL_SERVER   "192.168.100.103"
#define CONTROLLER_HW_VERSION 1
#define CONTROLLER_SW_VERSION "MCS_CONTOLLER_0_0_0_0:05/04/18"
#define RADIO1_DEFAULT_MODE 0
#define RADIO2_DEFAULT_MODE 0
#define RADIO3_DEFAULT_MODE 0
#define RADIO4_DEFAULT_MODE 0
#define RADIO5_DEFAULT_MODE 0
#define INIT_STATE        1
#define STANDBY_STATE     2
#define OPERATIONAL_STATE 3
#define MAINTENANCE_STATE 4
#define OFF_STATE         5
#define RADIO1_LOS_DEFAULT_FREQ  230000
#define RADIO2_LOS_DEFAULT_FREQ  260000
#define RADIO3_LOS_DEFAULT_FREQ  280000
#define RADIO4_LOS_DEFAULT_FREQ  300000
#define RADIO5_LOS_DEFAULT_FREQ  360000
#define SENDPACK_LOW   0X02
#define SENDPACK_MED   0X03
#define SENDPACK_HIGH  0X04
#define SENDPACK_VHIGH 0X05
#define LOOP_TERMINATE 0XDD
#define VHFGUARD_RANFREQ   145000
#define UHFGUARD_RANFREQ   255000
#define SUBSYSTEM_NORESPONSE 0XEE

/* Message Code List */
enum message_code_list
{

    MCS_MSC_POST_RESULT      = 0x0D01,
    MSC_MCS_POST_RESULT_ACK  = 0x0D02,
    MSC_MCS_CBIT_PERIODICITY = 0x0D03,
    MCS_MSC_CBIT_RESULTS     = 0x0D04,
    MSC_MCS_IBIT_CMD         = 0x0D05,
    MCS_MSC_IBIT_RESULTS     = 0x0D06,
    MSC_MCS_MBIT_CMD         = 0x0D07,
    MCS_MSC_MBIT_RESULTS     = 0x0D08,
    MSC_MCS_PFM_LOAD     	 = 0x0D09,
    MCS_MSC_PFM_LOAD_ACK 	 = 0x0D0A,
    MSC_MCS_V_UHF_STATE_CHANGE     = 0x0D10,
    MCS_MSC_V_UHF_STATE_CHANGE_ACK = 0x0D11,
    MCS_MSC_STATUS  		= 0x0D12,
    MCS_MSC_TXRXIND 		= 0x0D2A,
    MCS_MSC_ADDITIONAL_PARAMS_PRESENCE  = 0x0D14,
    MSC_MCS_ADDITIONAL_PARAMS_ERASE     = 0x0D15,
    MCS_MSC_ADDITIONAL_PARAMS_ERASE_ACK = 0x0D16,
    MCS_MSC_ALERTMSG = 0x0D17,
    MSC_MCS_RADIO_CONFIGURATION     = 0x0D20,
    MCS_MSC_RADIO_CONFIGURATION_ACK = 0x0D21,
    MSC_MCS_RELAY_CONFIGURATION     = 0x0D22,
    MCS_MSC_RELAY_CONFIGURATION_ACK = 0x0D23,
    MSC_MCS_GET_BACK_UP_FILE_DETAILS          = 0x0D41,
    MCS_MSC_GET_BACK_UP_FILE_DETAILS_RESPONSE = 0x0D42,
    MSC_MCS_AUDIO_FILE_BACKUP_STATE          = 0x0D43,
    MCS_MSC_AUDIO_FILE_BACKUP_STATE_RESPONSE = 0x0D44

};

/* acknowledgement return values */
enum ack_values
{
    FAILURE = 0,
    UNDER_PROGESS = 0xFE,   //for IBIT and MBIT Results
    SUCCESS = 0xFF
      
};
    

/* MCS System state status */
enum system_state_list
{
    SHUTDOWN = 0,
    OPERATE,
    MAINTANENCE
};

/* Main Node List */
enum main_node_list
{
    MCS_NODE_ID = 150,              //hex value - 0x96
    AMS_NODE_ID,                    //hex value - 0x97
    V_UHF_RADIO_SYSTEM_1_NODE_ID,   //hex value - 0x98
    V_UHF_RADIO_SYSTEM_2_NODE_ID,   //hex value - 0x99
    V_UHF_RADIO_SYSTEM_3_NODE_ID,   //hex value - 0x9A
    V_UHF_RADIO_SYSTEM_4_NODE_ID,   //hex value - 0x9B
    V_UHF_RADIO_SYSTEM_5_NODE_ID,   //hex value - 0x9C
    MCS_CONTROLLER_NODE_ID          //hex value - 0x9D

};

/* AMS Sub Node Level_1 List */
enum ams_sub_node_level_1_list
{
    VAIU = 1,
    ACP_1,
    ACP_2,
    ACP_3,
    ACP_4,
    ACP_5

};

enum ams_vaiu_sub_node_level_2_list
{
    PWR_SUPPLY = 1
};

enum ams_acp_sub_node_level_2_list
{
    E1_BUS = 1,
    RS485_A,
    RS485_B,
    POWER_SUPPLY

};

/* Radio System Sub Node Level_1 List */
enum radio_system_sub_node_level_1_list
{
    CONTROLLER = 1,
    SYNTHESIZER,
    TRANSMITTER,
    MAIN_RECEIVER,
    GUARD_RECEIVER,
    AUDIO_SECTION,
    POWER_REGULATOR,
    HPA_ICFPA,
    RF_CABLE_ANTENNA

};

/* MCS Controller Sub Node Level_1 List */
enum mcs_controller_sub_node_level_1_list
{
    BOARD = 1,
    STORAGE_MODULE,
    ETHERNET_INTERFACE,
    RADIO_INTERFACE

};

/* MCS Controller Sub Node Level_2 List */
enum mcs_controller_board_sub_node_level_2_list
{
    MEMORY = 1,
    PCI_BRIDGE,
    PCI_BUS_INTERFACE 

};

enum mcs_controller_storage_module_sub_node_level_2_list
{
    HARDDISK = 1

};

enum mcs_controller_ethernet_interface_sub_node_level_2_list
{
    PRI_ETHERNET = 1

};

enum mcs_controller_radio_interface_sub_node_level_2_list
{
    UART = 1,
    TOD,
    PTT1,
    PTT2,
    PTT3,
    PTT4,
    PTT5,
    SEC_ETHERNET

};



/* node health information structure */ 
typedef struct node_info
{
    uint8_t  node_identifier[4];
    int8_t   no_of_faults;                     // 0-No Faults, 1-One fault on this node 
    int8_t   node_fault_details;
    uint32_t time_sec;
    uint32_t time_fracsec;
    int8_t   current_system_state;             
    int8_t   current_state_transition_reason;

} node_info_t;

/* POST message structure */
struct MCS_MSC_POST_RESULT_STRUCT
{
    uint16_t message_code;
    uint16_t message_size;
    uint32_t time_sec;
    uint32_t time_fracsec;
    int8_t   hardware_version;
    uint8_t  software_version[30];
    uint8_t  no_of_nodes;
    node_info_t node_info[80];

} mcs_post_results;

/* POST message ack structure */
struct MSC_MCS_POST_ACK_STRUCT
{
    uint16_t message_code;
    uint16_t message_size;
    uint32_t time_sec;
    uint32_t time_fracsec;
    uint8_t  post_status;    //0 = Failure, 0xFF = Success

} mcs_post_ack;

/* CBit Cmd Message Structure */
struct MSC_MCS_CBIT_CMD_STRUCT
{
    uint16_t message_code;
    uint16_t message_size;
    uint32_t time_sec;
    uint32_t time_fracsec;
    uint8_t  cbit_periodicity;
    
} *msc_mcs_cbit_cmd_p;

/* CBit Ack Message Structure */
struct MCS_MSC_CBIT_CMD_ACK_STRUCT
{
    uint16_t message_code;
    uint16_t message_size;
    uint32_t time_sec;
    uint32_t time_fracsec;
    uint8_t  num_of_nodes;
    node_info_t node_info[80];
    
} mcs_msc_cbit_cmd_ack;

/* IBit Cmd Message Structure */
struct MSC_MCS_IBIT_CMD_STRUCT
{
    uint16_t message_code;
    uint16_t message_size;
    uint32_t time_sec;
    uint32_t time_fracsec;
    uint8_t  node_identifier[4];
    uint8_t  test_type;         // specified in SDD but not in IRS
    
} *msc_mcs_ibit_cmd_p;

/* IBit Ack Message Structure */
struct MCS_MSC_IBIT_CMD_ACK_STRUCT
{
    uint16_t message_code;
    uint16_t message_size;
    uint32_t time_sec;
    uint32_t time_fracsec;
    uint8_t  ack_result;      // specified in IRS but not in SDD
    uint8_t  num_of_nodes;
    node_info_t node_info[80];
    
} mcs_msc_ibit_cmd_ack;

/* MBit Cmd Message Structure */
struct MSC_MCS_MBIT_CMD_STRUCT
{
    uint16_t message_code;
    uint16_t message_size;
    uint32_t time_sec;
    uint32_t time_fracsec;
    uint8_t  node_identifier[4];
    uint8_t  test_type;         // specified in SDD but not in IRS

} *msc_mcs_mbit_cmd_p;

/* MBit Ack Message Structure */
struct MCS_MSC_MBIT_CMD_ACK_STRUCT
{
    uint16_t message_code;
    uint16_t message_size;
    uint32_t time_sec;
    uint32_t time_fracsec;
    uint8_t  ack_result;         // specified in IRS but not in SDD
    uint8_t  num_of_nodes;
    node_info_t node_info[80];
    
} mcs_msc_mbit_cmd_ack;

/* PFM structure */
struct MSC_MCS_PFM_LOAD_MAIN
{
    uint16_t message_code;
    uint16_t message_size;
    uint32_t time_sec;
    uint32_t time_fracsec;            
    struct FREQ_PRESET
    {
        float freq_preset[100];  
    }freq_preset_mem;
    
    float freq_default;          
    uint8_t num_blank_freqs;     
   
} *mcs_pfmload_struct_main;     

struct MSC_MCS_PFM_LOAD_BLANK
{
    float blank_freq_preset[200];  

} *mcs_pfmload_struct_blank;

struct MSC_MCS_PFM_LOAD_FULL
{
    uint8_t fha_index;               
    float   fha_freq[6];             
    uint8_t fha_mwod;               
    uint8_t fhb_index;
    uint8_t fhb_hopset_position;
    uint8_t fhb_num_cells;           
    uint8_t fhb_cell[64];            
    uint8_t num_radios;        
    
    struct RADIO_CONTROL_PARAMS
    {
        uint8_t radio_id;
        uint8_t allocation;
        uint8_t mode;
        uint8_t channel_main;       
        float   freq_main;          
        uint8_t channel_guard;      
        float   freq_guard;         
        uint8_t sub_state;
        uint8_t tx_power;
        uint8_t squelch;               
    }  pfm_radio_params[TOTAL_RADIOS]; // changed from TOTAL_RADIOS to 1

} *mcs_pfmload_struct_full;

struct MSC_MCS_PFM_LOAD_RELAY
{
    uint8_t relay_tr[((TOTAL_RADIOS / 2) * 2)];  

} *mcs_pfmload_struct_relay;


/* PFM Load Ack Message Structure */
struct MCS_MSC_PFM_LOAD_ACK_STRUCT
{
    uint16_t message_code;
    uint16_t message_size;
    uint32_t time_sec;
    uint32_t time_fracsec;
    uint8_t  pfm_load_ack;
    
} mcs_pfmload_ack;

/* MCS MSC Additional Params Presence */
struct MCS_MSC_ADDITIONAL_PARAMS_PRESENCE
{
    uint16_t message_code;
    uint16_t message_size;
    uint32_t time_sec;
    uint32_t time_fracsec;
    uint8_t  ap_status[TOTAL_RADIOS];
    
} mcs_ap_presence;

/* MCS MSC Additional Params Erase */
struct MCS_MSC_ADDITIONAL_PARAMS_ERASE
{
    uint16_t message_code;
    uint16_t message_size;
    uint32_t time_sec;
    uint32_t time_fracsec;
    uint8_t  ap_erase;
    
} *mcs_ap_erase_p;

/* MCS MSC Additional Params Erase Ack */
struct MCS_MSC_ADDITIONAL_PARAMS_ERASE_ACK
{
    uint16_t message_code;
    uint16_t message_size;
    uint32_t time_sec;
    uint32_t time_fracsec;
    uint8_t  ap_erase_ack;
    
} mcs_ap_erase_ack;

/* Radio Config */
struct MCS_MSC_RADIO_CONFIGURATION
{
    uint16_t message_code;
    uint16_t message_size;
    uint32_t time_sec;
    uint32_t time_fracsec;
    uint8_t  radio_id;
    uint8_t  radiation_on_off;
    uint8_t  allocation;
    uint8_t  mode;
    uint8_t  channel_main;
    float    freq_main;
    uint8_t  channel_guard;
    float    freq_guard;
    uint8_t  sub_state;
    uint8_t  tx_power;
    uint8_t  th_squelch;

} *mcs_radio_config_p;

/* Relay Config */
struct MSC_MCS_RELAY_CONFIGURATION
{
    uint16_t message_code;
    uint16_t message_size;
    uint32_t time_sec;
    uint32_t time_fracsec;
    uint8_t  relay_tr[((TOTAL_RADIOS / 2) * 2)];   

} *mcs_relay_config_p;

/* Relay Config Ack */
struct MCS_MSC_RELAY_CONFIGURATION_ACK
{
    uint16_t message_code;
    uint16_t message_size;
    uint32_t time_sec;
    uint32_t time_fracsec;
    uint8_t  relay_config_ack;
    
} mcs_relay_config_ack;


/*********************** MCS STATUS MESSAGES *********************/

/* MSC_MCS_V_UHF_STATE_CHANGE */
struct MSC_MCS_VUHF_STATE_CHANGE
{
    uint16_t message_code;
    uint16_t message_size;
    uint32_t time_sec;
    uint32_t time_fracsec;
    uint8_t  vuhf_state;
    
} *msc_mcs_vuhf_state_change_p;

struct MSC_MCS_VUHF_STATE_CHANGE_ACK
{
    uint16_t message_code;
    uint16_t message_size;
    uint32_t time_sec;
    uint32_t time_fracsec;
    uint8_t  vuhf_change_ack;

} mcs_state_change_ack;

/* MCS MSC Status */
struct MCS_MSC_STATUS_STRUCT
{
    uint16_t message_code;
    uint16_t message_size;
    uint32_t time_sec;
    uint32_t time_fracsec;
    uint8_t  intercom_Status;
    uint8_t  tr_status[TOTAL_RADIOS];

} mcs_status;

struct TIME_TO_SEND
{
    uint32_t sec;
    uint32_t fracsec;

} time_send;
#endif 
