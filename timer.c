#include <msp430.h>
#include <ctl.h>

//use majority function so the timer
//can be read while it is running
short readTA(void){
  int a=TAR,b=TAR,c=TAR;
  return (a&b)|(a&c)|(b&c);
}

//setup timer A to run off 32.768kHz xtal
void init_timerA(void){
  //setup timer A 
  TACTL=TASSEL_1|ID_0|TACLR;
  //init CCR0 for tick interrupt
  TACCR0=32;
  TACCTL0=CCIE;
}

//start timer A in continuous mode
void start_timerA(void){
//start timer A
  TACTL|=MC_2;
}
