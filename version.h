/******************************************************************************
*
* Filename: 	version.h
* 
*
******************************************************************************/

// ======================================================================================================================================================
//                            VERSION HISTORY
// ======================================================================================================================================================
// Date                Version            Engineer              Changes
// 6th  april 2018     Betarelease1       Harish/Nagaraj        Sending POST results(no failure) to MSC sucessfully 
// 11th april 2018     Betarelease2       Harish/Nagaraj        make file created 
// 11th april 2018     Betarelease3       Harish/Nagaraj        hour and min variables changed to sec and fsec
// 13th april 2018     Betarelease4       Harish/Nagaraj        dynamically allocating memory for node info successfull
// 16th april 2018     Betarelease5       Harish/Nagaraj        sending of two node info(no failure) to MSC successfull
// 09th may   2018     Betarelease6       Harish/Nagaraj        program implemented to send radio controller post results to MSC and
//                                                              receive post ack from MSC
// 04th july  2018     Betarelease7       Harish/Nagaraj        Adding of functions los_params_store, radio_post, radio_init, radiotr_com_check,clear_nvm 
//                                                              radiation_on_off,activate_los_preset,squelch_th,emergency_guard,store_setup_los,comm_check
//                                                              dec_to_hex & dec_to_bin(but not tested any function). Adding of radio_commands.c file.
//                                                              Change of datatypes.
// 14th Aug   2018     Betarelease8       Harish/Nagaraj        Able to read Radio Post and AMS Post results and update to MSC successfully.
// 21st Sep   2018     Betarelease9       Harish/Nagaraj        Received PFM Command from MSC and loaded PFM Parameters to radio and successfully 
//                                                              sent ack to MSC.
// 11th Oct   2018     Betarelease10      Harish/Nagaraj        Received Radio Config, AP Erase Cmd from MSC and performed the function succesfully with radio
//                                                              and sent ack to MSC. AP_Status, Alert_Messages and MCS Status also successfully sent to radio.
//                                                              Change of variable names from Upper to Lower case.
// 12th Oct   2018     Betarelease11      Harish/Nagaraj        Receiving of V/UHF State Change Command from MSC and Updating to MCS Controller/Radio
//                                                              accordingly and sending back ack to MSC.
// ======================================================================================================================================================
#ifndef VERSION_H_
#define VERSION_H_

#define VERSION "Betarelease11"


#endif /* VERSION_H_ */
