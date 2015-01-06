#include <msp430.h>
#include <string.h>
#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctl.h>
#include <terminal.h>
#include <ARCbus.h>
#include <SDlib.h>
#include <crc.h>
#include "COMM.h"
#include <Error.h>
#include "COMM_errors.h"
#include "Radio_functions.h"

//Turn on COMM
int onCmd(char *argv[],unsigned short argc){
  //output lower four bits COMM address (Ox13) to P7 LED's

  P7OUT=BIT1|BIT0;

  //Perhaps should set a register here that says we are commanded on.
  
  printf("COMM On.  Check LEDs: 0bxxxx0011\r\n");
}

//Turn off COMM
int offCmd(char *argv[],unsigned short argc){
   //output lower four bits COMM address (Ox13) to P7 LED's
  P7OUT=0;

  //Perhaps should set a register here that says we are commanded off.
  
  printf("COMM Off.  Check LEDs: 0bxxxx0000\r\n");
}

//Retreive status COMM
int statusCmd(char *argv[],unsigned short argc){

  int i;
   //flash lower four bits COMM address (Ox13) to P7 LED's 10 times
   P7OUT=BIT1|BIT0;
   for (i=0;i<10;i++){
	ctl_timeout_wait(ctl_get_current_time()+102);
	P7OUT=~(BIT1|BIT0);
	ctl_timeout_wait(ctl_get_current_time()+102);
	P7OUT=(BIT1|BIT0);
   }
  //Need to send back status through terminal.
  
  P7OUT=BIT1|BIT0; //finish present CDH address
  printf("COMM On.  Check LEDs: flashing 0bxxxx0011 - 0bxxxx1100\r\n");
}

//reset a MSP430 on command
int resetCmd(char **argv,unsigned short argc){
  //force user to pass no arguments to prevent unwanted resets
  if(argc!=0){
    printf("Error : %s takes no arguments\r\n",argv[0]);
    return -1;
  }
  //print reset message
  puts("Initiating reset\r\n");
  //write to WDTCTL without password causes PUC
  WDTCTL=0;
  //Never reached due to reset
  puts("Error : Reset Failed!\r");
  return 0;
}

int beaconCmd(char **argv,unsigned short argc){
  if(argc>1){
    printf("Error : Too many arguments\r\n");
    return -1;
  }
  if(argc==1){
    if(!strcmp(argv[1],"on")){
      beacon_on=1;
    }else if(!strcmp(argv[1],"off")){
      beacon_on=0;
    }else{
      printf("Error : Unknown argument \"%s\"\r\n",argv[1]);
      return -2;
    }
  }
  printf("Beacon : %s\r\n",beacon_on?"on":"off");
  return 0;
}

int writeReg(char **argv,unsigned short argc){
  char radio, regaddr, regdata;
  if(argc>3){
    printf("Error : Too many arguments\r\n");
    return -1;
  }
  if(argc==3){
    if(!strcmp(argv[1],"CC1101")){
      radio = CC1101;
    } else if(!strcmp(argv[1],"CC2500")){
      radio = CC2500;
    } else {
      printf("Error: Unknown radio \"%s\"\r\n",argv[1]);
      return -2;
    }
    regaddr=strtoul(argv[2],NULL,0);
    regdata = strtoul(argv[3],NULL,0);
    Radio_Write_Registers(regaddr, regdata, radio);
    printf("Wrote register 0x%02x = 0x%02x [regaddr, regdata]\r\n", regaddr, regdata);
  }
  return 0;
}

int readReg(char **argv,unsigned short argc){
  char result, radio, regaddr;
  if(argc>2){
    printf("Error : Too many arguments\r\n");
    return -1;
  }
  if(argc==2){
    if(!strcmp(argv[1],"CC1101")){
      radio = CC1101;
    } else if(!strcmp(argv[1],"CC2500")){
      radio = CC2500;
    } else {
      printf("Error: Unknown radio \"%s\"\r\n",argv[1]);
      return -2;
    }
    printf("Radio = %i\r\n",radio);
    regaddr=strtoul(argv[2],NULL,0);
    result= Radio_Read_Registers(regaddr, radio);
    printf("Register 0x%02x = 0x%02x [regaddr, regdata]\r\n", regaddr, result);
  }
  return 0;
}

int regtstCmd(char **argv,unsigned short argc){
  ctl_events_set_clear(&COMM_evt,COMM_EVT_LEDL_DAT,0);
}


//table of commands with help
const CMD_SPEC cmd_tbl[]={{"help"," [command]",helpCmd},
                    {"reset","\r\n\t""Reset the MSP430",resetCmd},
                    {"OnCOMM","[bgnd|stop]\r\n\t""Command ON COMM",onCmd},
                    {"OffCOMM","port [port ...]\r\n\t""Command OFF COMM",offCmd},
                    {"StatusCOMM","\r\n\t""Get CDH status",statusCmd},
                    {"WriteRadioReg","[radio regaddr data]", writeReg},
                    {"ReadRadioReg","[radio regaddr]", readReg},
                    {"beacon","[on|off]\r\n\t""Turn on/off status requests and beacon\r\n",beaconCmd},
                    {"regtst","\r\n\t""Test Read of register",regtstCmd},
                   //end of list

                   {NULL,NULL,NULL}};

