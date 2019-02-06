
/*
 ** HHdr-beg *****************************************************
 **
 ** Copyright Trident Infosol 2018
 ** All Rights Reserved
 **
 ** Trident infosol Proprietary
 **
 ** Include File name: bit.h
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
#ifndef BIT_H_
#define BIT_H_

#include <stdio.h>
#include <string.h>    
#include <stdlib.h>
#include <fcntl.h> 
#include <termios.h>
#include <unistd.h>
#include <stdint.h>

// #define DEBUG
#define set_bit(num, bit_pos) (num | (1 << bit_pos))
#define clr_bit(num, bit_pos) (num & ~(1 << bit_pos))
#define check_bit(num, bit_pos) (num & (1 << bit_pos))
#define FIOQSIZE 0x5460
#define EXAR_READ_REG (FIOQSIZE + 1)
#define EXAR_WRITE_REG (FIOQSIZE + 2)
#define PCI_BRIDGE_VENDOR 0X8086
#define PCI_EXAR_VENDOR   0X13A8
#define TOTAL_MCS_CTRLR_BIT 13

struct xrioctl_rw_reg {

    uint8_t Reg;
    uint8_t RegValue;	

} Output, Input;

int32_t open_dev(int8_t *);
void    mcs_ctrlr_post(uint8_t *);
uint8_t mcs_ctrlr_post_result(uint8_t);
#endif 

