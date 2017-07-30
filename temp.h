#ifndef __TEMP_H
#define __TEMP_H

#define Temp_Sensor1_CS BIT6; // Temp CS select lines on P5
#define Temp_Sensor2_CS BIT7;

extern int temp_select;  // global var to select temp sens 

int set_temp_sel(char *temp);
int temp_SPI_sel (int temp_select);
int temp_SPI_desel(int temp_select);
int temp_read_reg(int temp_select);

#endif
