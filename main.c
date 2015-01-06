//COMM FLIGHT
//Denise Thorsen
//2015-1-3

#include <string.h>
#include <ctl_api.h>
#include <msp430.h>
#include <ARCbus.h>
#include <SDlib.h>
#include <stdio.h>
#include <Error.h>
#include "COMM.h"
#include "Radio_functions.h"

//Define three task structures in array tasks (what are these tasks?)
CTL_TASK_t tasks[2];

//Define stacks for each tasks, STACKSize is the middle number
unsigned stack1[1+256+1];
unsigned stack2[1+256+1];

//make printf and friends to send chars out UCA1 uart
int __putchar(int c){
    return async_TxChar(c);
}

//set scanf and friends to read chars from UAC1 uart
int __getchar(void){
    return async_Getc();
}

int main(void)
{
  //Do this first
  ARC_setup();

  //setup SD card peripherals
  mmcInit_msp();

  //setup subsystem specific peripherals
  COMM_Setup();

  //setup buss interface - COMM
  initARCbus(BUS_ADDR_COMM);

  //setup P7 I/O output
  P7OUT = BUS_ADDR_COMM;
  P7DIR = 0xFF;
  P7SEL = 0;

  //initialize stacks
  memset(stack1, 0xcd, sizeof(stack1));  // write known values into the stack
  stack1[0]=stack1[sizeof(stack1)/sizeof(stack1[0])-1]=0xfeed; // put marker values at the words before/after the stack

  memset(stack2, 0xcd, sizeof(stack2));  // write known values into the stack
  stack2[0]=stack2[sizeof(stack2)/sizeof(stack2[0])-1]=0xfeed; // put marker values at the words before/after the stack

  //create tasks
  ctl_task_run(&tasks[0], BUS_PRI_LOW, COMM_events, NULL, "COMM_events", sizeof(stack1)/sizeof(stack1[0])-2,stack1+1,0);
  ctl_task_run(&tasks[1], BUS_PRI_HIGH, sub_events, NULL, "sub_events", sizeof(stack2)/sizeof(stack2[0])-2,stack2+1,0);
   
 //Call mainLoop to initialize the ARCbus task and drop the idle task priority to zero allowing other tasks to run.  This is the idle loop.
  mainLoop();

}


void Port2_ISR (void) __ctl_interrupt[PORT2_VECTOR]
{
   if (P2IFG & CC1101_GDO0) // GDO0 is set up to assert when RX FIFO is greater than FIFO_THR.  This is an RX function only
    {
        P2IFG &= ~CC1101_GDO0;
        ctl_events_set_clear(&COMM_evt,CC1101_EV_RX_READ,0);
    } 

    if (P2IFG & CC1101_GDO2) //GDO2 is set up to assert when TX FIFO is above FIFO_THR threshold.  
                             //Interrups on falling edge, i.e. when TX FIFO falls below FIFO_THR
    {
        switch(state)
        {
            case IDLE:
                 P2IFG &= ~CC1101_GDO2;
                 break;

            case TX_START:  //Called on falling edge of GDO2, Tx FIFO < threshold, Radio in TX mode, Packet in progress
                 state = TX_RUNNING;
                 P2IFG &= ~CC1101_GDO2;
                 ctl_events_set_clear(&COMM_evt,CC1101_EV_TX_THR,0);
                 break;
            
            case TX_RUNNING: //Called on falling edge of GDO2, Tx FIFO < threshold, Radio in TX mode, Packet in progress
                 P2IFG &= ~CC1101_GDO2;
                 ctl_events_set_clear(&COMM_evt,CC1101_EV_TX_THR,0);
                 break;

            case TX_END:  //Called on falling edge of GDO2, Tx FIFO < threshold, Radio in TX mode, Last part of packet to transmit
                 state = IDLE;
                 P2IFG &= ~CC1101_GDO2;
                 ctl_events_set_clear(&COMM_evt,CC1101_EV_TX_END,0);
                 break;

            default:
              P2IFG &= ~CC1101_GDO2;
              break;          
  
        }
    }

}



