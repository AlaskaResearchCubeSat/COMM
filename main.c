#include <msp430.h>
#include <ctl.h>
#include <ARCbus.h>
#include <Error.h>
#include <terminal.h>      
#include <string.h>         // added for memeset function
#include <UCA2_uart.h>        // UART setup 
#include "pins.h"         
#include "Radio_functions.h"      
#include "COMM.h"  


CTL_TASK_t terminal_task,sub_task,comm_task; // name your task (first thing to do when setting up a new task (1))

//********************************************* allocate mem for tasks (2)
//stack for terminal
unsigned terminal_stack[2000];
//stack for subsystem events
unsigned COMM_sys_stack[1000];
//stack for bus events
unsigned sub_stack[1000];

//******************************************** redefine putchar and getchar 
//make printf and friends use async
int __putchar(int c){
  return UCA2_TxChar(c);
}

//make printf and friends use async
int __getchar(void){
  return UCA2_Getc();
}

//******************************************* Main loop****************************************************************************************************************
void main(void){
  //turn on LED's this will flash the LED's during startup
  P7OUT=0x01;
  P7DIR=0xFF;
  
  //Initlize the comm communication port for the SPI 
  radio_SPI_setup(); // Do this before ARC_setup because of PM 

  //DO this first (but not before PM)
  ARC_setup(); 

  //TESTING: set log level to report everything by default
  set_error_level(0);

  //initialize UART
  UCA2_init_UART(UART_PORT,UART_TX_PIN_NUM,UART_RX_PIN_NUM);

  //setup bus interface
  initARCbus(0x1F);   // Default addr for "SYS" subsystem, should be changed for specific subsystems.

   
  // initialize stacks (3) 
  memset(terminal_stack,0xcd,sizeof(terminal_stack));                                                     // write known values into the stack 
  terminal_stack[0]=terminal_stack[sizeof(terminal_stack)/sizeof(terminal_stack[0])-1]=0xfeed;            // put marker values at the words before/after the stack
  memset(COMM_sys_stack,0xcd,sizeof(COMM_sys_stack));                                                     // write known values into the stack 
  COMM_sys_stack[0]=COMM_sys_stack[sizeof(COMM_sys_stack)/sizeof(COMM_sys_stack[0])-1]=0xfeed;            // put marker values at the words before/after the stack
  memset(sub_stack,0xcd,sizeof(sub_stack));                                                               // write known values into the stack 
  sub_stack[0]=sub_stack[sizeof(sub_stack)/sizeof(sub_stack[0])-1]=0xfeed;                                // put marker values at the words before/after the stack


  // creating the tasks
  ctl_task_run(&terminal_task, BUS_PRI_LOW,    terminal,    "Mike's Awesome COMM Code", "terminal",        sizeof(terminal_stack)/sizeof(terminal_stack[0])-2,   terminal_stack-1,0);
  ctl_task_run(&comm_task,     BUS_PRI_NORMAL, COMM_events, NULL,                       "COMM_SYS_events", sizeof(COMM_sys_stack)/sizeof(COMM_sys_stack[0])-2,   COMM_sys_stack-1,0);
  ctl_task_run(&sub_task,      BUS_PRI_HIGH,   sub_events,  NULL,                       "SUB_events",      sizeof(sub_stack)/sizeof(sub_stack[0])-2,             sub_stack-1,0);

  //main loop <-- this is an ARCbus function 
  mainLoop(); 

}

//decode errors
char *err_decode(char buf[150], unsigned short source,int err, unsigned short argument){
  sprintf(buf,"source = %i, error = %i, argument = %i",source,err,argument);
  return buf;
}

