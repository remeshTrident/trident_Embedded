/******************************************************************************
*
* Filename: 	global.h
* 
*
******************************************************************************/
#ifndef GLOBAL_H_
#define GLOBAL_H_

#define MAX_BUFF_SIZE  01234567
#define TOTAL_RADIOS_PLUS_BUFF_SIZE 001234
#define RECEIVTIMEOUT 		2000000000
#define HEX_MINUS_ONE        -1
#define HEX_ZERO              0X00
#define HEX_ONE               0X1
#define HEX_TWO               0X2
#define HEX_THREE             0X3
#define HEX_FOUR              0X4
#define HEX_FIVE              0X5
#define HEX_SIX               0X6
#define HEX_SEVEN             0X7
#define HEX_EIGHT             0X8
#define HEX_NINE              0X9
#define HEX_TEN		      10
#define HEX_TEN1              0x10 
#define HEX_A		      0X0A
#define HEX_B	              0X0B
#define HEX_C   	      0X0C
#define HEX_D	              0X0D
#define HEX_E	              0X0E
#define HEX_F        	      0X0F
#define HEX_0X27              0X27

#define HEX_FF		   0XFF
#define HEX_90		   0X90
#define HEX_96		   0X96

#define HEX_0xAA            0xAA
#define HEX_0x01            0x01
#define HEX_0x11            0x11
#define HEX_0x06            0x06
#define HEX_0xAE	    0xAE
#define HEX_0XA1	    0XA1
#define HEX_0XED            0XED
#define HEX_0XE0 	    0XE0

#define HEX_0XA5            0XA5
#define HEX_0XA6 	    0XA6
#define HEX_0xC0	    0xC0
#define HEX_0x51	    0x51
#define HEX_0x64 	    0x64
#define HEX_0X82 	    0X82
#define HEX_0X89 	    0X89
#define HEX_0xDD	    0xDD
#define HEX_0XEE 	    0XEE
#define HEX_0XA0 	    0XA0
#define HEX_0x00            0x00
#define HEX_0x00FF          0x00FF
#define HEX_0xFF00          0xFF00
#define HEX_2000000000	    2000000000
#define HEX_12                12
#define HEX_25	 	      25
#define HEX_F7		      0xF7
#define HEX_F1		      0xF1

#define HEX_0x22            0x22
#define HEX_0X25            0x25
#define HEX_0X26            0x26
#define HEX_0x24    	    0x23 
#define HEX_0x23    	    0x24 
#define HEX_0x28    	    0x28
#define HEX_0X2B            0x2B
#define HEX_0x2F	    0x2F	 
#define HEX_0x30            0x30
#define HEX_0x32            0x32
#define HEX_0x33            0x33
#define HEX_0x34            0x34
#define HEX_0x35            0x35
#define HEX_0x36            0x36
#define HEX_0x37            0x37
#define HEX_0x3F       	    0x3F
#define HEX_0x50            0x50
#define HEX_0x55            0x55
#define HEX_0x70            0x70
#define HEX_0x75            0x75
#define HEX_0x82	    0x82
#define HEX_0xFE	    0xFE

/////

#define HEX_0x0000	0x0000
#define HEX_0xC0C1	0xC0C1 
#define HEX_0xC181	0xC181 
#define HEX_0x0140	0x0140
#define HEX_0xC301	0xC301 
#define HEX_0x03C0	0x03C0 
#define HEX_0x0280	0x0280		 
#define HEX_0xC241	0xC241	
#define HEX_0xC601	0xC601 
#define HEX_0x06C0	0x06C0 
#define HEX_0x0780	0x0780	 
#define HEX_0xC741	0xC741 
#define HEX_0x0500	0x0500 
#define HEX_0xC5C1	0xC5C1
#define HEX_0xC481	0xC481	 
#define HEX_0x0440	0x0440
#define HEX_0xCC01	0xCC01 
#define HEX_0x0CC0	0x0CC0 
#define HEX_0x0D80	0x0D80 
#define HEX_0xCD41	0xCD41 
#define HEX_0x0F00	0x0F00 
#define HEX_0xCFC1	0xCFC1 
#define HEX_0xCE81	0xCE81 
#define HEX_0x0E40	0x0E40
#define HEX_0x0A00	0x0A00 
#define HEX_0xCAC1	0xCAC1 
#define HEX_0xCB81	0xCB81 
#define HEX_0x0B40	0x0B40 
#define HEX_0xC901	0xC901 
#define HEX_0x09C0	0x09C0 
#define HEX_0x0880	0x0880 
#define HEX_0xC841	0xC841
#define HEX_0xD801	0xD801 
#define HEX_0x18C0 	0x18C0
#define HEX_0x1980	0x1980 
#define HEX_0xD941 	0xD941
#define HEX_0x1B00	0x1B00 
#define HEX_0xDBC1	0xDBC1 
#define HEX_0xDA81	0xDA81 
#define HEX_0x1A40	0x1A40
#define HEX_0x1E00	0x1E00 
#define HEX_0xDEC1	0xDEC1 
#define HEX_0xDF81	0xDF81 
#define HEX_0x1F40	0x1F40 
#define HEX_0xDD01	0xDD01 
#define HEX_0x1DC0	0x1DC0 
#define HEX_0x1C80	0x1C80 
#define HEX_0xDC41	0xDC41
#define HEX_0x1400	0x1400 
#define HEX_0xD4C1	0xD4C1 
#define HEX_0xD581	0xD581 
#define HEX_0x1540	0x1540 
#define HEX_0xD701	0xD701 
#define HEX_0x17C0	0x17C0 
#define HEX_0x1680	0x1680 
#define HEX_0xD641	0xD641	
#define HEX_0xD201	0xD201 
#define HEX_0x12C0	0x12C0 
#define HEX_0x1380	0x1380 
#define HEX_0xD341	0xD341 
#define HEX_0x1100	0x1100 
#define HEX_0xD1C1	0xD1C1 
#define HEX_0xD081	0xD081 
#define HEX_0x1040	0x1040
#define HEX_0xF001	0xF001 
#define HEX_0x30C0	0x30C0 
#define HEX_0x3180	0x3180 
#define HEX_0xF141	0xF141 
#define HEX_0x3300	0x3300 
#define HEX_0xF3C1	0xF3C1 
#define HEX_0xF281	0xF281 
#define HEX_0x3240	0x3240
#define HEX_0x3600	0x3600 
#define HEX_0xF6C1	0xF6C1 
#define HEX_0xF781	0xF781 
#define HEX_0x3740	0x3740 
#define HEX_0xF501	0xF501 
#define HEX_0x35C0	0x35C0 
#define HEX_0x3480	0x3480 
#define HEX_0xF441	0xF441
#define HEX_0x3C00	0x3C00 
#define HEX_0xFCC1	0xFCC1 
#define HEX_0xFD81	0xFD81 
#define HEX_0x3D40	0x3D40 
#define HEX_0xFF01	0xFF01 
#define HEX_0x3FC0	0x3FC0 
#define HEX_0x3E80	0x3E80 
#define HEX_0xFE41	0xFE41
#define HEX_0xFA01	0xFA01 
#define HEX_0x3AC0	0x3AC0 
#define HEX_0x3B80	0x3B80 
#define HEX_0xFB41	0xFB41 
#define HEX_0x3900	0x3900 
#define HEX_0xF9C1	0xF9C1 
#define HEX_0xF881	0xF881 
#define HEX_0x3840	0x3840
#define HEX_0x2800	0x2800 
#define HEX_0xE8C1	0xE8C1 
#define HEX_0xE981	0xE981 
#define HEX_0x2940	0x2940 
#define HEX_0xEB01	0xEB01 
#define HEX_0x2BC0	0x2BC0 
#define HEX_0x2A80	0x2A80
#define HEX_0xEA41	0xEA41
#define HEX_0xEE01	0xEE01 
#define HEX_0x2EC0	0x2EC0 
#define HEX_0x2F80	0x2F80 
#define HEX_0xEF41	0xEF41 
#define HEX_0x2D00	0x2D00 
#define HEX_0xEDC1	0xEDC1 
#define HEX_0xEC81	0xEC81 
#define HEX_0x2C40	0x2C40
#define HEX_0xE401	0xE401 
#define HEX_0x24C0	0x24C0 
#define HEX_0x2580	0x2580
#define HEX_0xE541	0xE541 
#define HEX_0x2700	0x2700 
#define HEX_0xE7C1	0xE7C1 
#define HEX_0xE681	0xE681 
#define HEX_0x2640	0x2640
#define HEX_0x2200	0x2200 
#define HEX_0xE2C1	0xE2C1 
#define HEX_0xE381	0xE381 
#define HEX_0x2340	0x2340 
#define HEX_0xE101	0xE101 
#define HEX_0x21C0	0x21C0 
#define HEX_0x2080	0x2080 
#define HEX_0xE041	0xE041
#define HEX_0xA001 	0xA001 
#define HEX_0x60C0	0x60C0 
#define HEX_0x6180	0x6180 
#define HEX_0xA141	0xA141 
#define HEX_0x6300	0x6300 
#define HEX_0xA3C1	0xA3C1 
#define HEX_0xA281	0xA281 
#define HEX_0x6240	0x6240
#define HEX_0x6600	0x6600 
#define HEX_0xA6C1	0xA6C1 
#define HEX_0xA781	0xA781 
#define HEX_0x6740	0x6740 
#define HEX_0xA501	0xA501 
#define HEX_0x65C0	0x65C0 
#define HEX_0x6480	0x6480 
#define HEX_0xA441	0xA441
#define HEX_0x6C00	0x6C00 
#define HEX_0xACC1	0xACC1 
#define HEX_0xAD81	0xAD81 
#define HEX_0x6D40	0x6D40 
#define HEX_0xAF01	0xAF01 
#define HEX_0x6FC0	0x6FC0 
#define HEX_0x6E80	0x6E80 
#define HEX_0xAE41	0xAE41
#define HEX_0xAA01	0xAA01 
#define HEX_0x6AC0	0x6AC0 
#define HEX_0x6B80	0x6B80 
#define HEX_0xAB41	0xAB41 
#define HEX_0x6900	0x6900 
#define HEX_0xA9C1	0xA9C1 
#define HEX_0xA881	0xA881 
#define HEX_0x6840	0x6840
#define HEX_0x7800	0x7800 
#define HEX_0xB8C1	0xB8C1 
#define HEX_0xB981	0xB981 
#define HEX_0x7940	0x7940 
#define HEX_0xBB01	0xBB01 
#define HEX_0x7BC0	0x7BC0 
#define HEX_0x7A80	0x7A80 
#define HEX_0xBA41	0xBA41
#define HEX_0xBE01	0xBE01 
#define HEX_0x7EC0	0x7EC0 
#define HEX_0x7F80	0x7F80 
#define HEX_0xBF41	0xBF41 
#define HEX_0x7D00	0x7D00 
#define HEX_0xBDC1	0xBDC1 
#define HEX_0xBC81	0xBC81 
#define HEX_0x7C40	0x7C40
#define HEX_0xB401	0xB401 
#define HEX_0x74C0	0x74C0 
#define HEX_0x7580	0x7580 
#define HEX_0xB541	0xB541 
#define HEX_0x7700	0x7700 
#define HEX_0xB7C1	0xB7C1 
#define HEX_0xB681	0xB681 
#define HEX_0x7640	0x7640
#define HEX_0x7200	0x7200 
#define HEX_0xB2C1	0xB2C1 
#define HEX_0xB381	0xB381 
#define HEX_0x7340	0x7340 
#define HEX_0xB101	0xB101 
#define HEX_0x71C0	0x71C0 
#define HEX_0x7080	0x7080	 
#define HEX_0xB041	0xB041
#define HEX_0x5000	0x5000 
#define HEX_0x90C1	0x90C1 
#define HEX_0x9181	0x9181 
#define HEX_0x5140	0x5140 
#define HEX_0x9301	0x9301 
#define HEX_0x53C0	0x53C0
#define HEX_0x5280	0x5280 
#define HEX_0x9241	0x9241
#define HEX_0x9601	0x9601 
#define HEX_0x56C0	0x56C0 
#define HEX_0x5780	0x5780 
#define HEX_0x9741	0x9741 
#define HEX_0x5500	0x5500 
#define HEX_0x95C1	0x95C1 
#define HEX_0x9481	0x9481 
#define HEX_0x5440	0x5440
#define HEX_0x9C01	0x9C01 
#define HEX_0x5CC0	0x5CC0 
#define HEX_0x5D80	0x5D80 
#define HEX_0x9D41	0x9D41 
#define HEX_0x5F00	0x5F00 
#define HEX_0x9FC1	0x9FC1 
#define HEX_0x9E81	0x9E81 
#define HEX_0x5E40	0x5E40
#define HEX_0x5A00	0x5A00 
#define HEX_0x9AC1	0x9AC1 
#define HEX_0x9B81	0x9B81 
#define HEX_0x5B40	0x5B40 
#define HEX_0x9901	0x9901 
#define HEX_0x59C0	0x59C0 
#define HEX_0x5880	0x5880 
#define HEX_0x9841	0x9841
#define HEX_0x8801	0x8801 
#define HEX_0x48C0	0x48C0 
#define HEX_0x4980	0x4980 
#define HEX_0x8941	0x8941 
#define HEX_0x4B00	0x4B00 
#define HEX_0x8BC1	0x8BC1 
#define HEX_0x8A81	0x8A81 
#define HEX_0x4A40	0x4A40
#define HEX_0x4E00	0x4E00 
#define HEX_0x8EC1	0x8EC1
#define HEX_0x8F81	0x8F81
#define HEX_0x4F40	0x4F40 
#define HEX_0x8D01	0x8D01 
#define HEX_0x4DC0	0x4DC0 
#define HEX_0x4C80	0x4C80 
#define HEX_0x8C41	0x8C41
#define HEX_0x4400	0x4400 
#define HEX_0x84C1	0x84C1 
#define HEX_0x8581	0x8581 
#define HEX_0x4540	0x4540 
#define HEX_0x8701	0x8701 
#define HEX_0x47C0	0x47C0 
#define HEX_0x4680	0x4680 
#define HEX_0x8641	0x8641
#define HEX_0x8201	0x8201 
#define HEX_0x42C0	0x42C0 
#define HEX_0x4380	0x4380 
#define HEX_0x8341	0x8341 
#define HEX_0x4100	0x4100 
#define HEX_0x81C1	0x81C1 
#define HEX_0x8081	0x8081 
#define HEX_0x4040      0x4040

/////

#define	HEX_0x20	0x20
#define	HEX_0x21	0x21
#define	HEX_0x52	0x52
#define	HEX_0x58	0x58
#define HEX_0x6E  	0x6E
#define HEX_0x61	0x61
#define HEX_0x63	0x63
#define HEX_0x6D	0x6D
#define HEX_0x6F	0x6F
#define HEX_0x95	0x95
#define HEX_0xF8	0xF8
#define HEX_0xA9	0xA9
#define HEX_0x80	0x80
#define HEX_0x8D	0x8D
#define HEX_0xC4	0xC4
#define HEX_0x3A	0x3A
#define HEX_0xE8	0xE8

/////

#define DEC_2		     2
#define DEC_3		     3
#define DEC_10		     10
#define DEC_11		     11
#define DEC_12		     12
#define DEC_13		     13
#define DEC_14		     14
#define DEC_15		     15
#define DEC_16		     16
#define DEC_17		     17
#define DEC_18		     18
#define DEC_19		     19
#define DEC_20		     20
#define DEC_21		     21
#define DEC_22		     22
#define DEC_23		     23
#define DEC_24		     24
#define DEC_25		     25
#define DEC_26		     26
#define DEC_27		     27
#define DEC_28		     28
#define DEC_29		     29
#define DEC_30		     30
#define DEC_31		     31
#define DEC_32		     32
#define DEC_033              033
#define DEC_34		     34
#define DEC_35		     35
#define DEC_36		     36
#define DEC_37		     37
#define DEC_38		     38
#define DEC_39		     39
#define DEC_40		     40
#define DEC_41		     41
#define DEC_42		     42
#define DEC_43		     43
#define DEC_44		     44
#define DEC_45		     45
#define DEC_46		     46
#define DEC_47		     47
#define DEC_48		     48
#define DEC_49		     49

#define DEC_50		     50
#define DEC_51		     51
#define DEC_52		     52
#define DEC_53		     53
#define DEC_54		     54
#define DEC_55		     55
#define DEC_56		     56
#define DEC_57		     57
#define DEC_58		     58
#define DEC_59		     59


#define DEC_60		     60
#define DEC_61		     61
#define DEC_62		     62
#define DEC_63		     63
#define DEC_64		     64
#define DEC_65		     65
#define DEC_66		     66
#define DEC_67		     67
#define DEC_68		     68
#define DEC_69		     69

#define DEC_70		     70
#define DEC_71		     71
#define DEC_72		     72
#define DEC_73		     73
#define DEC_74		     74
#define DEC_75		     75
#define DEC_76		     76
#define DEC_77		     77
#define DEC_78		     78
#define DEC_79		     79

#define DEC_80		     80
#define DEC_81		     81
#define DEC_82		     82
#define DEC_83		     83
#define DEC_84		     84
#define DEC_85		     85
#define DEC_86		     86
#define DEC_87		     87
#define DEC_88		     88
#define DEC_89		     89

#define DEC_90		     90
#define DEC_91		     91
#define DEC_92		     92
#define DEC_93		     93
#define DEC_94		     94
#define DEC_95		     95
#define DEC_96		     96
#define DEC_97		     97
#define DEC_98		     98
#define DEC_99		     99

#define DEC_100              100
#define DEC_101              101
#define DEC_102              102
#define DEC_103              103
#define DEC_104              104
#define DEC_105              105
#define DEC_106              106
#define DEC_107              107
#define DEC_108              108
#define DEC_109              109

#define DEC_110              110
#define DEC_111              111
#define DEC_112              112
#define DEC_113              113
#define DEC_114              114
#define DEC_115              115
#define DEC_116              116
#define DEC_117              117
#define DEC_127              127
//#define DEC_100		     100
#define DEC_144		     144
#define DEC_150		     150
#define DEC_162		     162
#define DEC_163		     163
#define DEC_164		     164
#define DEC_178		     178
#define DEC_179		     179
#define DEC_181	             181
#define DEC_182	             182
#define DEC_183	             183
#define DEC_184	             184
#define DEC_185	             185
#define DEC_186	             186
#define DEC_187	             187
#define DEC_250		     250
#define DEC_255		     255
#define DEC_256		     256
#define DEC_150		     150
#define DEC_166              166
#define DEC_180              180
#define DEC_416              416
#define DEC_408              408
#define DEC_420              420
#define DEC_500		     500
#define DEC_583		     583
#define DEC_661              661
#define DEC_657              657
#define DEC_658              658
#define DEC_659              659
#define DEC_666 	     666
#define DEC_660              660
#define DEC_700              700
#define DEC_833		     833
#define DEC_916              916
#define DEC_1000             1000
#define DEC_108000           108000
#define DEC_225000	     225000
#define DEC_173975	     173975
#define DEC_155975           155975         
#define DEC_130000	     130000
#define DEC_399975	     399975
#define DEC_225000           225000
#define DEC_1000000000 1000000000
#define FREQUENCYTONEWINT 166

#define FLOAT_118_00        118.00
#define FLOAT_135_975      135.975

typedef enum _SWITCH_CASE
{
 ZERO=0,
ONE,
TWO,
THREE,
FOUR,
FIVE,
SIX,
SEVEN,
EIGHT,
NINE,
TEN,
ELEVEN,
TWELVE,
THIRTEEN,
FOURTEEN,
FIFTEEN,
SIXTEEN,
SEVENTEEN,
EIGHTEEN=18,
NINETEEN,
TWENTY,
TWENTYONE =21,
TWENTYTWO =22,
TWENTYTHREE =23,
TWENTYFOUR =24,
TWENTYFIVE =25,
TWENTYSIX,
TWENTYSEVEN,
TWENTYEIGHT,
TWENTYNINE,
THIRTY,
THIRTYONE,
THIRTYTWO,
THIRTYTHREE,
THIRTYFOUR,
THIRTYFIVE,
THIRTYSIX,
THIRTYSEVEN,
THIRTYEIGHT,
THIRTYNINE,
FOURTY,
FOURTYONE,
FOURTYTWO,
FOURTYTHREE,
FOURTYFOUR,
FOURTYFIVE,
FOURTYSIX,
FOURTYSEVEN,
FOURTYEIGHT,
FOURTYNINE,
FIFTY=50,
FIFTYONE,
FIFTYTWO,
FIFTYTHREE,
FIFTYFOUR,
FIFTYFIVE,
FIFTYSIX,
FIFTYSEVEN,
FIFTYEIGHT,
FIFTYNINE,
SIXTY,
SIXTYONE,
SIXTYTWO,
SIXTYTHREE,
SIXTYFOUR,
SIXTYFIVE,
SIXTYSIX,
SIXTYSEVEN,
SIXTYEIGHT,
SIXTYNINE,
SEVENTY,
SEVENTYONE,
SEVENTYTWO,
SEVENTYTHREE,
SEVENTYFOUR,
SEVENTYFIVE,
SEVENTYSIX,
SEVENTYSEVEN,
SEVENTYEIGHT,
SEVENTYNINE,
EIGHTY,
EIGHTYONE,
EIGHTYTWO,
EIGHTYTHREE,
EIGHTYFOUR,
EIGHTYFIVE,
EIGHTYSIX,
EIGHTYSEVEN,
EIGHTYEIGHT,
EIGHTYNINE,
NINTY,
NINTYONE,
NINTYTWO,
NINTYTHREE,
NINTYFOUR,
NINTYFIVE,
NINTYSIX,
NINTYSEVEN,
NINTYEIGHT,
NINTYNINE,
HUNDRED,
HUNDREDANDONE,
HUNDREDANDTWO,
HUNDREDANDTHREE,
HUNDREDANDFOUR,
HUNDREDANDFIVE,
HUNDREDANDSIX,
HUNDREDANDSEVEN,
HUNDREDANDEIGHT,
HUNDREDANDNINE,
HUNDREDANDTEN,
HUNDREDANDELEVEN,
HUNDREDANDTWELVE,
HUNDREDANDTHIRTEEN,
HUNDREDANDFOURTEEN,
HUNDREDANDFIFTEEN,
HUNDREDANDSIXTEEN,
HUNDREDANDSEVENTEEN=117,
TWOHUNDREDFIFTYSIX=256

}switchcase;




typedef enum _TRUEFALSE
{
 FALSE=0,
 TRUE=1


}truefalse;

typedef enum _GREGORIANDATES 
{


GBDATE_1900 = 1900,
GBDATE_1800 = 1800,
 GBDATE_1700 = 1700,
GBDATE_2000 = 2000


}gregorianDates;

//typedef enum _SWITCH_CASE switchcase;






#endif 

