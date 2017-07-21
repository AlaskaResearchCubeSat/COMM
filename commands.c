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
#include "AX25_EncodeDecode.h"
#include "COMM_Events.h"
#include "COMM.h"
#include "temp.h"
#include "Radio_functions.h"

extern CTL_EVENT_SET_t COMM_evt; // define because this lives in COMM.c

//*********************************************************************************** RADIO COMMANDS *****************************************************
//NOTE do not define global vars in a local function ie "radio_select"
int writeReg(char **argv,unsigned short argc){
  char regaddr, regdata;  // expecting [radio] [address] [data]
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
    //printf("Radio = %i\r\n",radio_select);
    regaddr=strtoul(argv[2],NULL,0);
    regdata = strtoul(argv[3],NULL,0);
    Radio_Write_Registers(regaddr, regdata, radio_select);
    //Radio_Write_Registers(regaddr, regdata, 1);
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
 status1=Radio_Read_Status(TI_CCxxx0_MARCSTATE,CC1101);   // get status of CC1101
 status2=Radio_Read_Status(TI_CCxxx0_MARCSTATE,CC2500_1); // get status of CC2500
 state1=status1&(~(BIT7|BIT6|BIT5)); //get state of CC2500_1
 state2=status2&(~(BIT7|BIT6|BIT5)); //get state of CC2500_2
  if(0x00==state1){
   printf("The CC1101 is in the SLEEP state or may be unconnected.\r\n");
  }
  else if(0x00==state2){
   printf("The CC2500 is in the SLEEP state or may be unconnected.\r\n");
  }
  else{
  // store stat stuff
    printf("The status of the CC1101 is %s.\r\n",statetbl[status1]);
    printf("The state of the CC1101 is %i.\r\n",state1);
    printf("The status of the CC2500_1 is %s.\r\n",statetbl[status2]);
    printf("The state of the CC2500_1 is %i.\r\n",state2);
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
  ctl_events_set_clear(&COMM_evt,COMM_EVT_CC2500_1_TX_START,0); 
  printf("Push any key to stop\r\n");
  getchar(); // waits for any char 
  Radio_Write_Registers(TI_CCxxx0_PKTCTRL0, 0x00, radio_select);         // Fixed byte mode
  state = TX_END;
  return 0;
}

//Select power output from the radio chip
//TODO (update for second radio)
int powerCmd(char **argv,unsigned short argc){
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
    Radio_Write_Registers(TI_CCxxx0_PATABLE,power_PTABLE[idx],radio_select);
  }
  read=Radio_Read_Registers(TI_CCxxx0_PATABLE,radio_select);
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


// reset selected radio, if un-specified all radios rest
//TODO gets stuck in set_radio_path.
int radio_resetCmd(char **argv,unsigned short argc){
  int radio_check;

  if ( argc < 1){           // reset all radios if no args passed 
    Reset_Radio(CC1101);
    Reset_Radio(CC2500_1);
    __delay_cycles(800);                         // Wait for radio to be ready before writing registers.cc1101.pdf Table 13 indicates a power-on start-up time of 150 us for the crystal to be stable
    
    Write_RF_Settings(CC1101);                // Write radios Settings
    Write_RF_Settings(CC2500_1);                // Write radios Settings

    Radio_Strobe(TI_CCxxx0_SRX, CC1101);          //Initialize CCxxxx in Rx mode
    Radio_Strobe(TI_CCxxx0_SRX, CC2500_1);          //Initialize CCxxxx in Rx mode

  }
  else{                     
    radio_check = set_radio_path(argv[1]);       // reset specified radio
    printf("radio path set to %d.\r\nradio_check = %d.\r\n",radio_select,radio_check);
    if (radio_check && -1){
      printf("Plese enter a valid radio, ex. \"CC1101, CC2500_1, CC2500_2\".\r\n");
      return -1;
    }
    printf("The %s radio has been reset.\r\n");
    Reset_Radio(radio_select);
    __delay_cycles(800);                         // Wait for radio to be ready before writing registers.cc1101.pdf Table 13 indicates a power-on start-up time of 150 us for the crystal to be stable
    Write_RF_Settings(radio_select);             // Write radios Settings
  }
  return 0;
}

// beacon_on COMM's beacon arbiter var 1 = send 
int beacon_onCmd(char **argv,unsigned short argc){
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

// sets COMM's beacon_flag "hello" beacon var 1 = beacon("hello")
int beacon_flagCmd(char **argv,unsigned short argc){
  if(argc>1){
    printf("Error : Too many arguments\r\n");
    return -1;
  }
  if(argc==1){
    if(!strcmp(argv[1],"on")){
      beacon_flag=1;
    }else if(!strcmp(argv[1],"off")){
      beacon_flag=0;
    }else{
      printf("Error : Unknown argument \"%s\"\r\n",argv[1]);
      return -2;
    }
  }
  printf("Beacon_flag : %s\r\n",beacon_flag?"on":"off");
  return 0;
}

int TestCmd(char **argv,unsigned short argc){
  
  set_radio_path(argv[1]);
  if(argc > 1){
   radio_SPI_sel (radio_select); 
   printf("The %s radio has been selected.\r\n", argv[1]);
   }
   else{
   radio_SPI_desel(radio_select);
   printf("The %s radio has been deselected.\r\n", argv[1]);
   }
  return 0;
}

//table of commands with help
const CMD_SPEC cmd_tbl[]={{"help"," [command]",helpCmd},
                   {"status","",status_Cmd},
                   {"stream","[zeros|ones|[value [val]]]\r\n""Stream data from radio.\n\r",streamCmd},
                   {"writereg","Writes data to radio register\r\n [radio] [adress] [data].\n\r",writeReg},
                   {"readreg","reads data from a radio register\r\n [radio] [adrss].\n\r",readReg},
                   {"power","Changes the transmit power of the radio [radio][power].\n\rex. CC2500_1 -24\n\r",powerCmd},
                   {"radio_reset","Reset radios on COMM SPI bus.\n\rradio_reset [radio]. Note if no radio addr included all radios will be reset",radio_resetCmd},
                   {"beacon","Toggles the COMM beacon on or off.\n\rCurrently targeting the CC2500_1",beacon_onCmd},
                   {"beacon_flag","Toggles the COMM beacon \"hello\" packet on or off.\n\rCurrently targeting the CC2500_1",beacon_flagCmd},
                   {"test","for testing things in code",TestCmd},
                  // ARC_COMMANDS,CTL_COMMANDS, ERROR_COMMANDS
                   //end of list
                   {NULL,NULL,NULL}};

