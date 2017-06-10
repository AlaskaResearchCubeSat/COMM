#ifndef __temp_H
#define __temp_H

// for SPI pin def. check "Radio_functions.h"
#define Temp_Sensor1_CS BIT7; // CC1101
#define Temp_Sensor2_CS BIT6; // CC2500


// function prototypes 
int temp_SPI_sel (int temp_select);
int temp_SPI_desel(int temp_select);
int temp_read_reg(int temp_select);


#endif
