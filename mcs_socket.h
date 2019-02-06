

/*
 ** HHdr-beg *****************************************************
 **
 ** Copyright Trident Infosol 2018
 ** All Rights Reserved
 **
 ** Trident infosol Proprietary
 **
 ** Include File name: mcs_socket.h
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

#ifndef MCS_SOCKET_H_
#define MCS_SOCKET_H_

#include <stdio.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <stdint.h>

#include "config.h"

void pfmbit_socket(uint8_t);
void ctrl_msc_socket(uint8_t);
void ctrl_mcs_socket(uint8_t);

//Global declaration of socket id
extern int32_t pfmbit_sockfd, ctrl_msc_sockfd, ctrl_mcs_sockfd;

#endif 
