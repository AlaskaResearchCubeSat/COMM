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



extern CTL_EVENT_SET_t COMM_evt; // define because this lives in COMM.c
//********************************************************************  Example commands (not really comm related ) *********************************************
int example_command(char **argv,unsigned short argc){
  int i,j;
  //TODO replace printf with puts ? 
  printf("This is an example command that shows how arguments are passed to commands.\r\n""The values in the argv array are as follows : \r\n");
  for(i=0;i<=argc;i++){
    printf("argv[%i] = 0x%p\r\n\t""string = \"%s\"\r\n",i,argv[i],argv[i]);
    //print out the string as an array of hex values
    j=0;
    printf("\t""hex = {");
    do{
      //check if this is the first element
      if(j!='\0'){
        //print a space and a comma
        printf(", ");
      }
      //print out value
      printf("0x%02hhX",argv[i][j]);
    }while(argv[i][j++]!='\0');
    //print a closing bracket and couple of newlines
    printf("}\r\n\r\n");
    printf("argc = %i\r\n",argc);
  }
  i=strtol(argv[1],0,0);
  printf("the string says %p\r\n",i);
  return 0;
}
/*********************************************************** Using the Timer_A1 ***************************************************************
* DONT USE TIMER0_Ax_VECTOR !!! this interrupt is use in library code and will cause a collision 
* Use TIMERx_Ay_VECTOR x=2,3 & y=0,1
* TIMER0_Ax_VECTOR used in ARClib ? 
* TIMER1_Ax_VECTOR used in ????
**********************************************************************************************************************************************/

int example_timer_IR(char **argv,unsigned short argc){
  int timer_check;
  WDTCTL = WDTPW+WDTHOLD;                                   // Stop WDT
  P7DIR |= 0xFF;                                            // Setting port 7 to drive LED's (0xFF ==1111 1111)
  P7OUT = 0x00;                                             // Set all LED's on port 7 to start all off
//************************************ Set up clock [0] 
  TA2CTL |= TASSEL__ACLK | MC_2;                            // Setting Timer_A to ACLK(TASSEL_1) to continuous mode(MC_2)

//*********************************** Set timer interrupt enable [1] 
  TA2CCTL0 |= CCIE;                                          // Capture/compare interrupt enable #0
  TA2CCTL1 |= CCIE;                                          // Capture/compare interrupt enable #1

//*********************************** Set the timer count IR value [2] 
  TA2CCR0 = 10000;                                           // Timer0_A3 Capture/Compare @ 10000 counts
  TA2CCR1 = 1000;                                            // TA0IV_1 Capture/Compare @ 1000 counts

   while (1)                                                // poll in while loop until a key press
   {
      if ((timer_check=getchar()) != EOF)
     {
      break;                                                 // break out of loop if a key is pressed
     }
    ctl_events_wait(CTL_EVENT_WAIT_ANY_EVENTS,0, 1<<15, CTL_TIMEOUT_DELAY, 1000); // wait in loop 
   }
  reset(0,ERR_SRC_CMD,CMD_ERR_RESET,0);                     // reset clock registers 
  return 0;
}

// ********************************* Timer_A0 interrupt code 
void Timer_A2_A0(void)__interrupt[TIMER2_A0_VECTOR]{     // Timer A0 interrupt service routine TA0IV_TA0IFG. 
  P7OUT^=BIT0; // toggle LEDs when IR is called
}

void Timer_A2_A1(void)__interrupt[TIMER2_A1_VECTOR]{     // Timer A0 interrupt service routine for capture comp 1 and 2
        P7OUT^=BIT1; // light LEDs
}
//*********************************************************************************** RADIO COMMANDS *****************************************************

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
 status1=Radio_Read_Status(TI_CCxxx0_MARCSTATE,CC2500_1); // get status of CC1101
 status2=Radio_Read_Status(TI_CCxxx0_MARCSTATE,CC2500_2); // get status of CC2500
 state1=status1&(~(BIT7|BIT6|BIT5)); //get state of CC2500_1
 state2=status2&(~(BIT7|BIT6|BIT5)); //get state of CC2500_2
  if(0x00==state1){
   printf("Radio CC1011 is in the SLEEP state or may be unconnected");
  }
  else if(0x00==state2){
   printf("Radio CC2500 is in the SLEEP state or may be unconnected");
  }
  else{
  // store stat stuff
    printf("The status of the CC2500_1 is %s.\r\n",statetbl[status1]);
    printf("The state of the CC2500_1 is %i.\r\n",state1);
    printf("The status of the CC2500_2 is %s.\r\n",statetbl[status2]);
    printf("The state of the CC2500_2 is %i.\r\n",state2);
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

int power(char **argv,unsigned short argc)
{
int radio_power;
radio_power = strtoul(argv[2],NULL,0); //Converts the inputed string in argv[2] to an int or hex value
set_radio_path(argv[1]); //select the radio
Radio_Write_Registers(TI_CCxxx0_PATABLE, radio_power, radio_select); //Write to the power register
printf("The power selected is %x\r\n", radio_power);
printf("The radio selected is radio CC2500_%i radio  \r\n", radio_select);
}


//Better power function using enum(set up in radio radiofunctions.h)

int powerbetter(char **argv,unsigned short argc)
{
int radio_level;
enum power_level power;
set_radio_path(argv[1]); //select the radio
power=strtoul(argv[2],NULL,0);
 if (power ==  power1){
  Radio_Write_Registers(TI_CCxxx0_PATABLE, 0x00, radio_select);
  printf("The power selected is -55 dBm\r\n" );
  printf("The radio selected is radio CC2500_%i radio \r\n", radio_select);
  }
  else if(power ==  power2){
  Radio_Write_Registers(TI_CCxxx0_PATABLE, 0x50, radio_select);
  printf("The power selected is -30 dBm\r\n" );
  printf("The radio selected is radio CC2500_%i radio \r\n", radio_select);
  }
    else if(power ==  power3){
  Radio_Write_Registers(TI_CCxxx0_PATABLE, 0x44, radio_select);
  printf("The power selected is -28 dBm\r\n" );
  printf("The radio selected is radio CC2500_%i radio \r\n", radio_select);
  }
    else if(power ==  power4){
  Radio_Write_Registers(TI_CCxxx0_PATABLE, 0xC0, radio_select);
  printf("The power selected is -26 dBm\r\n" );
  printf("The radio selected is radio CC2500_%i radio \r\n", radio_select);
  }
    else if(power ==  power5){
  Radio_Write_Registers(TI_CCxxx0_PATABLE, 0x84, radio_select);
  printf("The power selected is -24 dBm\r\n" );
  printf("The radio selected is radio CC2500_%i radio \r\n", radio_select);
  }
    else if(power ==  power6){
  Radio_Write_Registers(TI_CCxxx0_PATABLE, 0x81, radio_select);
  printf("The power selected is -22 dBm\r\n" );
  printf("The radio selected is radio CC2500_%i radio \r\n", radio_select);
  }
    else if(power ==  power7){
  Radio_Write_Registers(TI_CCxxx0_PATABLE, 0x46, radio_select);
  printf("The power selected is -20 dBm\r\n" );
  printf("The radio selected is radio CC2500_%i radio \r\n", radio_select);
  }
    else if(power ==  power8){
  Radio_Write_Registers(TI_CCxxx0_PATABLE, 0x93, radio_select);
  printf("The power selected is -18 dBm\r\n" );
  printf("The radio selected is radio CC2500_%i radio \r\n", radio_select);
  }
    else if(power ==  power9){
  Radio_Write_Registers(TI_CCxxx0_PATABLE, 0x55, radio_select);
  printf("The power selected is -16 dBm\r\n" );
  printf("The radio selected is radio CC2500_%i radio \r\n", radio_select);
  }
    else if(power ==  power10){
  Radio_Write_Registers(TI_CCxxx0_PATABLE, 0x8D, radio_select);
  printf("The power selected is -14 dBm\r\n" );
  printf("The radio selected is radio CC2500_%i radio \r\n", radio_select);
  }
    else if(power ==  power11){
  Radio_Write_Registers(TI_CCxxx0_PATABLE, 0xC6, radio_select);
  printf("The power selected is -12 dBm\r\n" );
  printf("The radio selected is radio CC2500_%i radio \r\n", radio_select);
  }
    else if(power ==  power12){
  Radio_Write_Registers(TI_CCxxx0_PATABLE, 0x97, radio_select);
  printf("The power selected is -10 dBm\r\n" );
  printf("The radio selected is radio CC2500_%i radio \r\n", radio_select);
  }
    else if(power ==  power13){
  Radio_Write_Registers(TI_CCxxx0_PATABLE, 0x6E, radio_select);
  printf("The power selected is -8 dBm\r\n" );
  printf("The radio selected is radio CC2500_%i radio \r\n", radio_select);
  }
    else if(power ==  power14){
  Radio_Write_Registers(TI_CCxxx0_PATABLE, 0x7F, radio_select);
  printf("The power selected is -6 dBm\r\n" );
  printf("The radio selected is radio CC2500_%i radio \r\n", radio_select);
  }
  else if(power ==  power15){
  Radio_Write_Registers(TI_CCxxx0_PATABLE, 0xA9, radio_select);
  printf("The power selected is -4 dBm\r\n" );
  printf("The radio selected is radio CC2500_%i radio \r\n", radio_select);
  }
    else if(power ==  power16){
  Radio_Write_Registers(TI_CCxxx0_PATABLE, 0xBB, radio_select);
  printf("The power selected is -2 dBm\r\n" );
  printf("The radio selected is radio CC2500_%i radio \r\n", radio_select);
  }
    else if(power ==  power17){
  Radio_Write_Registers(TI_CCxxx0_PATABLE, 0xFE, radio_select);
  printf("The power selected is -0 dBm\r\n" );
  printf("The radio selected is radio CC2500_%i radio \r\n", radio_select);
  }
    else if(power ==  power18){
  Radio_Write_Registers(TI_CCxxx0_PATABLE, 0xFF, radio_select);
  printf("The power selected is +1 dBm\r\n" );
  printf("The radio selected is radio CC2500_%i radio \r\n", radio_select);
  }
  else{
  printf("error not a valid power");
  }
}

int transmit_test(char **argv,unsigned short argc){
  int i=0;
  data_mode=TX_DATA_BUFFER;
  for(i=0;i<19;i++){
    Tx1Buffer[i]=Packet_NoBit[i];
  }
 while(UCA2_CheckKey()==EOF){
  P7OUT ^= BIT1;  // flip a led every loop 
  ctl_events_set_clear(&COMM_evt,COMM_EVT_CC2500_1_TX_START,0);
  BUS_delay_usec(5000);  // delay in ms

 }
}

int transmit_test2(char **argv,unsigned short argc){

  int i=0;
  for(i=0;i<19;i++){
    Tx1Buffer[i]=Packet_NoBit[i];
  }
 while(UCA2_CheckKey()==EOF){
  RF_Send_Packet(Tx1Buffer, 19, CC2500_1);
  BUS_delay_usec(50);  // delay in ms 
 }
}

readtemp(char **argv,unsigned short argc)
{
// TODO voltage sampling and then use the conversion 2.43 mV/degrees C
int i= 0; 
Radio_Strobe(TI_CCxxx0_SIDLE,CC2500_1);
Radio_Write_Registers(TI_CCxxx0_PTEST,0xBF,CC2500_1); //Writing 0xBF to this register makes the on-chip temperature sensor available in the IDLE state. The default 0x7F value
Radio_Write_Registers(TI_CCxxx0_IOCFG0,0x01,CC2500_1);
//printf("The temperature is %i \r\n", );
Radio_Write_Registers(TI_CCxxx0_PTEST,0x7F,CC2500_1);
}


//table of commands with help
const CMD_SPEC cmd_tbl[]={{"help"," [command]",helpCmd},
                   {"timer_IR","[time]...\r\n\tExample command to show how the timer can be used as an interupt",example_timer_IR},
                   {"ex","[arg1] [arg2] ...\r\n\t""Example command to show how arguments are passed",example_command},
                   {"radio_status","",status_Cmd},
                   {"stream","[zeros|ones|[value [val]]]\r\n""Stream data from radio\n\r",streamCmd},
                   {"writereg","Writes data to radio register\r\n [radio] [adress] [data]",writeReg},
                   {"readreg","reads data from a radio register\r\n [radio] [adrss]",readReg},
                   {"transmit_test","Testing tranmission of data\r\n [data][event] ", transmit_test},
                   {"power","Changes the transmit power of the radio [radio][PATABLE Value] ex. CC2500_1 0x8D",power},
                   {"powerbetter","Changes the transmit power of the radio [radio][power] ex. CC2500_1 -24",powerbetter},
                   {"transmit_test2","Testing tranmission of data\r\n [data][event] ", transmit_test2},
                   {"readtemp","reads the temperature", readtemp},
                   ARC_COMMANDS,CTL_COMMANDS,// ERROR_COMMANDS
                   //end of list
                   {NULL,NULL,NULL}};

