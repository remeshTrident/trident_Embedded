
/*
 ** HHdr-beg *****************************************************
 **
 ** Copyright Trident Infosol 2018
 ** All Rights Reserved
 **
 ** Trident infosol Proprietary
 **
 ** Include File name: radio_commands.h
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

#ifndef RADIO_COMMANDS_H_
#define RADIO_COMMANDS_H_

#include <stdio.h>
#include <stdint.h>

#include "config.h"
#include "bit.h"
#include "util.h"

uint32_t checksum(uint8_t *, uint32_t);

uint8_t *f_initialisation(uint8_t, uint8_t *);
uint8_t *f_comm_check(uint8_t, uint8_t *);
uint8_t *f_clear_nvm(uint8_t, uint8_t *);
uint8_t *f_radiation_on_off(uint8_t *, uint8_t *);
uint8_t *f_activate_setup(uint8_t, uint8_t *);
uint8_t *f_storesetup_rpwman(uint8_t, uint8_t *);
uint8_t *f_volume(uint8_t, uint8_t *);
uint8_t *f_th_squelch(uint8_t *, uint8_t *);
uint8_t *f_status_request(uint8_t, uint8_t *);
uint8_t *f_ackpack_toradio(uint8_t, uint8_t *);
uint8_t *f_emergency_guard(uint8_t *, uint8_t *);
uint8_t *f_store_setup(uint8_t *, uint8_t *);
uint8_t *f_preset_request(uint8_t, uint8_t *);

uint8_t *f_hq_command(uint8_t *, uint8_t *);
uint8_t *f_rpw_transec_erase(uint8_t, uint8_t *);
uint8_t *f_pps_disable(uint8_t *pps_disable_radiopack);
uint8_t *f_timeequalization(uint8_t *);
uint8_t *f_rpw_load_kfd(uint8_t *, uint8_t *);
uint8_t *f_hq_load_ss(uint8_t *, uint8_t *, uint8_t);
uint8_t *f_hq_load_mwod(uint8_t *, uint8_t *);
uint8_t *f_rpw_stored_transec(uint8_t, uint8_t *);
uint8_t *f_rpw_ss_fh(uint8_t *, uint8_t *);
uint8_t *f_gpsrecv_enable(uint8_t *);

#endif
