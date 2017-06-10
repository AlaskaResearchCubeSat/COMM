/**********************************************************************************************************************************************
The commands.c file is for commands that will be displayed through the serial terminal. 
In order to add a command you must create a function as seen below.
Then function must be added to the "const CMD_SPEC cmd_tbl[]={{"help"," [command]",helpCmd}" table at the end of the file.
**********************************************************************************************************************************************/
#include <msp430.h>
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <terminal.h>
#include <commandLib.h>
#include <stdlib.h>
#include <ARCbus.h>
#include <SDlib.h>
#include <i2c.h>
#include <Radio_functions.h>
#include <UCA2_uart.h>  
#include "COMM.h"
#include "AX25_EncodeDecode.h"
#include "COMM_Events.h"
#include "temp.h"



extern CTL_EVENT_SET_t COMM_evt; // define because this lives in COMM.c

//*********************************************************************************** RADIO UART COMMANDS *****************************************************

int writeReg(char **argv,unsigned short argc){
  char radio_select, regaddr, regdata;  // expecting [radio] [address] [data]
  int radio_check;

  if(argc>3){ // input checking and set radio address 
    printf("Error : Too many arguments\r\n");
    return -1;
  }

  radio_check = set_radio_path(argv[1]);  // set radio_select  

   if (radio_check==-1) {
      printf("Error: Unknown radio \"%s\"\r\n",argv[1]);
      return -2;
    }
  else{
    regaddr=strtoul(argv[2],NULL,0);
    regdata = strtoul(argv[3],NULL,0);
    Radio_Write_Registers(regaddr, regdata, radio_select);
    printf("Wrote register 0x%02x = 0x%02x [regaddr, regdata]\r\n", regaddr, regdata);
    return 0;
  }

  printf("Error : %s requires 3 arguments but %u given\r\n",argv[0],argc);
  return -2;
}

int readReg(char **argv,unsigned short argc){
  char result, radio, regaddr;  // expecting [radio]  [address]
  int radio_check;

  if(argc>2){
    printf("Error : Too many arguments\r\n");
    return -1;
  }

  radio_check = set_radio_path(argv[1]);  // set radio_select  

   if (radio_check==-1) {
      printf("Error: Unknown radio \"%s\"\r\n",argv[1]);
      return -2;
    }
  else {
    printf("Radio = %i\r\n",radio_select);
    regaddr=strtoul(argv[2],NULL,0);
    result= Radio_Read_Registers(regaddr, radio_select);
    printf("Register 0x%02x = 0x%02x [regaddr, regdata]\r\n", regaddr, result);
  }
  return 0;
}

int status_Cmd(char **argv,unsigned short argc){
char status1, status2, radio, state1, state2;
// state info
 const char* statetbl[32]={"SLEEP","IDLE","XOFF","VCOON_MC","REGON_MC","MANCAL","VCOON","REGON","STARTCAL","BWBOOST","FS_LOCK","IFADCON","ENDCAL","RX","RX_END","RX_RST","TXRX_SWITCH","RXFIFO_OVERFLOW","FSTXON","TX","TX_END","RXTX_SWITCH","TXFIFO_UNDERFLOW"};
// read 0x00 --> 0x2E
 status1=Radio_Read_Status(TI_CCxxx0_MARCSTATE,CC1101); // get status of CC1101
 status2=Radio_Read_Status(TI_CCxxx0_MARCSTATE,CC2500); // get status of CC2500
 state1=status1&(~(BIT7|BIT6|BIT5)); //get state of CC1101
 state2=status2&(~(BIT7|BIT6|BIT5)); //get state of CC2500
  if(0x00==state1){
   printf("Radio CC1011 is in the SLEEP state or may be unconnected");
  }
  else if(0x00==state2){
   printf("Radio CC2500 is in the SLEEP state or may be unconnected");
  }
  else{
  // store stat stuff
    printf("The status of the CC1101 is %s.\r\n",statetbl[status1]);
    printf("The state of the CC1101 is %i.\r\n",state1);
    printf("The status of the CC2500 is %s.\r\n",statetbl[status2]);
    printf("The state of the CC2500 is %i.\r\n",state2);
  }
return 0;
} 

// streams data from radio argv[1]=ADR 
//TODO   (update for second radio)
int streamCmd(char **argv,unsigned short argc){
// input checking 
  if(!strcmp(argv[1],"value")){
    data_mode=TX_DATA_PATTERN;
    data_seed=atoi(argv[2]); // arg to stream (0xXX)
  }
  else if(!strcmp(argv[1],"random")){
    data_mode=TX_DATA_RANDOM;
    if(argc==2){
      data_seed=atoi(argv[2]);
      if(data_seed==0){
        data_seed=1;
      }
    }
    else{
      data_seed=1;
    }
  }
 
  // input case statment to pick from enum table in COMM.h
  ctl_events_set_clear(&COMM_evt,COMM_EVT_CC1101_TX_START,0); 
  
  printf("Push any key to stop\r\n");
  getchar(); // waits for any char 
  
  Radio_Write_Registers(TI_CCxxx0_PKTCTRL0, 0x00, radio_select);         // Fixed byte mode
  state = TX_END;

  return 0;
}


int transmit_test(char **argv,unsigned short argc){
  int i=0;
  data_mode=TX_DATA_BUFFER;
  for(i=0;i<19;i++){
    Tx1Buffer[i]=Packet_NoBit[i];
  }
 while(UCA2_CheckKey()==EOF){
  P7OUT ^= BIT1;  // flip a led every loop 
  ctl_events_set_clear(&COMM_evt,COMM_EVT_CC2500_TX_START,0);
  BUS_delay_usec(5000);  // delay in ms

 }
}

//Select power output from the radio chip
//TODO (update for second radio)
int power_Cmd(char **argv,unsigned short argc){
  const int power_dbm[8]=             { -30, -20, -15, -10,   0,   5,   7,  10};
  const unsigned char power_PTABLE[8]={0x12,0x0E,0x1D,0x34,0x60,0x84,0xC8,0xC0};
  unsigned long input;
  int idx,i;
  int pwr;
  char *end;
  unsigned char read;

  if(argc>0){
    input=strtol(argv[1],&end,0);
    if(*end=='\0' || !strcmp(end,"dBm")){
      pwr=input;
    }else{
      printf("Error : unknown suffix \"%s\" for power \"%s\"\r\n",end,argv[1]);
      return -1;
    }
    for(i=0,idx=0;i<8;i++){
      //find the power that is closest to desired
      if(abs(power_dbm[i]-pwr)<abs(power_dbm[idx]-pwr)){
        idx=i;
      }
    }
    printf("Setting radio to %idBm\r\n",power_dbm[idx]);
    Radio_Write_Registers(TI_CCxxx0_PATABLE,power_PTABLE[idx],CC1101);
  }
  read=Radio_Read_Registers(TI_CCxxx0_PATABLE,CC1101);
  for(i=0,idx=-1;i<8;i++){
    if(power_PTABLE[i]==read){
      idx=i;
      break;
    }
  }
  if(idx==-1){
    printf("PTABLE = 0x%02X\r\n",read);
  }else{
    printf("PTABLE = %idBm = 0x%02X\r\n",power_dbm[idx],read);
  }
  return 0;
}

//LED strobe on plugin (done)
LED_cmd(char** argv, unsigned short argc){
  P7DIR=0xFF;
  P7OUT=0x00;
  while(async_CheckKey()==EOF){
    P7OUT=P7OUT+1;
    ctl_timeout_wait(ctl_get_current_time()+50);
  }
  P7OUT = BUS_ADDR_COMM; // re-set LED to COMM addr
return 0;
}

// read temp data from SPI connected IC's
temp_cmd(char** argv, unsigned short argc){
  int *read;
  int addr;

    if(!(strcmp(argv[1],"CC1101"))){  //pick temp sens 
      addr = 1;
    }
    else if(!(strcmp(argv[1],"CC2500"))){
      addr = 2;
    }
    else{
     addr = 1;  //defalt for extraneous input 
    }
    *read = temp_read_reg(addr); // read from temp sens
    printf("Temp sens for the %s radio reads %i .\r\n",argv[1],read);
}

//table of commands with help
const CMD_SPEC cmd_tbl[]={{"help"," [command]",helpCmd},
                   {"radio_status","",status_Cmd},
                   {"stream","[zeros|ones|[value [val]]]\r\n""Stream data from radio\n\r",streamCmd},
                   {"writereg","Writes data to radio register\r\n [radio] [adress] [data]",writeReg},
                   {"readreg","reads data from a radio register\r\n [radio] [adrss]",readReg},
                   {"power","[power]\r\n""get/set the radio output power\n\r",power_Cmd},
                   {"transmit_test","Testing tranmission of data\r\n [data][event] ", transmit_test},
                   {"LED","pulses LED's as binary counter",LED_cmd},
                   {"temp","temp [CC1101/CC2500] .\r\n",temp_cmd},

                   //ARC_COMMANDS,CTL_COMMANDS,// ERROR_COMMANDS
                   //end of list
                   {NULL,NULL,NULL}};

