#include <msp430.h>
#include <stdio.h>
#include <string.h>
#include "temp.h"

/* NOTES
The ADC 12 bit result onto the SO @ all 0 --> 0C and @ all 1's 1023.75 C
D15 -->D0, where D15 is a dummy bit and always zero. D14-->D3, contain the converted temp.D2-->D0, 
*/

//global temp vars
int temp_select;  // keep track of witch temp sens we are talking to either temp1 (CC1101 channel) or temp2 (CC2500 channel)

//set temp path
int set_temp_sel(char *temp){
  if(strcmp(temp,"temp1")){ 
    temp_select = 1;
  }
  else if(strcmp(temp,"temp2")){
    temp_select = 2;
  }
  else{
    return -1;
  }
}

//temp chip select function. CS active low, this will initiate a new conversion
int temp_SPI_sel (int temp_select){
  switch (temp_select){
    case 1:
     P5OUT &= ~Temp_Sensor1_CS;
     break;
    case 2:
     P5OUT &= ~Temp_Sensor2_CS;
     break;
    default:
     return-1;
  }
}

//temp chip de-select function
int temp_SPI_desel(int temp_select){
    switch (temp_select){
    case 1:
     P5OUT |= Temp_Sensor1_CS;
     break;
    case 2:
     P5OUT |= Temp_Sensor2_CS;
     break;
    default:
     return-1;
  }
}

//temp read function 
int temp_read_reg(int temp_select){
  int temp_data[2];
  int i;

  temp_SPI_sel(temp_select);  // initiate temp read 

  for(i=0;i<2;i++){
    while(!(UCB1IFG & UCTXIFG));
      UCB1TXBUF = 0XAA;   // dummy write 
    while (UCB1STAT & UCBUSY);
      temp_data[i] = UCB1RXBUF; // this will be MSP
  }
  temp_SPI_desel(temp_select);
  return *temp_data; 
  
}










