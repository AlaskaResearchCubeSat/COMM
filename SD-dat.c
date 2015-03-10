
#include <msp430.h>
#include <ctl_api.h>
#include <stdio.h>
#include <ARCbus.h>
#include <string.h>
#include <SDlib.h>
#include "COMM_errors.h"
#include <crc.h>
#include <Error.h>
#include "SD-dat.h"

DATA_TABLE sd_data_table;

int data_setup(void){
    SD_DATA_TABLE *block;
    int resp;
    unsigned short crc;
    //read data table from SD card
    block=BUS_get_buffer(CTL_TIMEOUT_DELAY,5000);
    //check if timeout expired
    if(block!=NULL){
        //read block
        resp=mmcReadBlock(SD_DAT_TABLE,(unsigned char*)block);
        //check for errors
        if(resp==RET_SUCCESS){
            //check magic
            if(block->magic==DATA_TABLE_MAGIC){
                //calculate CRC
                crc=crc16(&block->dat,sizeof(block->dat));
                //check CRC
                if(crc==block->CRC){
                    //copy data table from SD card
                    memcpy(&sd_data_table,&block->dat.table,sizeof(sd_data_table));
                    //free buffer                    
                    BUS_free_buffer();
                    //success
                    return RET_SUCCESS;
                }else{
                    //CRC is not correct, report error
                    report_error(ERR_LEV_INFO,COMM_ERR_SRC_DAT_STORE,COMM_ERR_DAT_INIT_CRC,0);
                }
            }else{
                //wrong magic number, report error
                report_error(ERR_LEV_INFO,COMM_ERR_SRC_DAT_STORE,COMM_ERR_DAT_INIT_MAGIC,block->magic);
            }
            //This is probably the first boot, initialize structure to all zeros
            memset(&sd_data_table,0,sizeof(sd_data_table));
            //prepare values for SD card
            block->magic=DATA_TABLE_MAGIC;
            //copy data to SD structure
            memcpy(&sd_data_table,&block->dat.table,sizeof(sd_data_table));
            //calculate CRC
            block->CRC=crc16(&block->dat,sizeof(block->dat));
            //write data table to SD card
            resp=mmcWriteBlock(SD_DAT_TABLE,(unsigned char*)block);
            //check result
            if(resp!=RET_SUCCESS){
                //write failed, report error
                report_error(ERR_LEV_ERROR,COMM_ERR_SRC_DAT_STORE,COMM_ERR_DAT_INIT_WRITE,resp);
                //free buffer                
                BUS_free_buffer();
                //return error
                return SD_DAT_INIT_WRITE_ERR;
            }
            //free bufer            
            BUS_free_buffer();
            //return initilization error code
            return SD_DAT_INIT;
        }else{
            //SD card read failed, report error
            report_error(ERR_LEV_ERROR,COMM_ERR_SRC_DAT_STORE,COMM_ERR_DAT_INIT_READ,resp);
            //TODO: handle this some how
        }
        //free buffer
        BUS_free_buffer();
    }else{
        //buffer busy, report error
        report_error(ERR_LEV_ERROR,COMM_ERR_SRC_DAT_STORE,COMM_ERR_DAT_INIT_BUFFER,0);
        //TODO: handle this some how
    }
    //return error reading from card setup data not loaded
    return SD_DAT_INIT_ERR;
}

int writeSD_Data(unsigned char subsystem,unsigned char type,const unsigned char *dat){
    SD_block_addr dest;
    SD_DATA_TABLE *block;
    int resp;
    //check which subsystem is being used
    switch(subsystem){
        case BUS_ADDR_LEDL:
            dest=SD_LEDL_DAT_START+sd_data_table.next_LEDL++;
            //wraparound so that we don't write out of the LEDL area
            if(sd_data_table.next_LEDL>=SD_LEDL_DAT_SIZE){
                sd_data_table.next_IMG=0;
            }
        break;
        case BUS_ADDR_ACDS:
            dest=SD_ACDS_DAT_START+sd_data_table.next_ACDS++;
            //wraparound so that we don't write out of the ACDS area
            if(sd_data_table.next_ACDS>=SD_ACDS_DAT_SIZE){
                sd_data_table.next_ACDS=0;
            }
        break;
        case BUS_ADDR_IMG:
            dest=SD_IMG_DAT_START+sd_data_table.next_IMG++;
            //wraparound so that we don't write out of the IMG area
            if(sd_data_table.next_IMG>=SD_IMG_DAT_SIZE){
                sd_data_table.next_IMG=0;
            }
        break;
        default:
            //unknown subystem report error
            report_error(ERR_LEV_ERROR,COMM_ERR_SRC_DAT_STORE,COMM_ERR_DAT_STORE_BAD_SUBSYSTEM,subsystem);
            //return error
            return SD_DAT_BAD_SUBSYSTEM;
    }
    //write data to SD card
    resp=mmcWriteBlock(dest,dat);
    //check for errors
    if(resp!=RET_SUCCESS){
        //TODO: handle error and return
        report_error(ERR_LEV_ERROR,COMM_ERR_SRC_DAT_STORE,COMM_ERR_DAT_STORE_BLOCK_WRITE,resp);
        //return error
        return SD_DAT_BLOCK_WRITE_ERROR;
    }
    //store data table in *dat for SD card write
    block=(SD_DATA_TABLE*)dat;
    //copy data to SD structure
    memcpy(&sd_data_table,&block->dat.table,sizeof(sd_data_table));
    //set block ID
    block->magic=DATA_TABLE_MAGIC;
    //calculate CRC
    block->CRC=crc16(&block->dat,sizeof(block->dat));
    //write data table to SD card
    resp=mmcWriteBlock(SD_DAT_TABLE,(unsigned char*)block);
    //check for errors
    if(resp!=RET_SUCCESS){
        //TODO: handle error and return
        report_error(ERR_LEV_ERROR,COMM_ERR_SRC_DAT_STORE,COMM_ERR_DAT_STORE_TABLE_WRITE,resp);
        //return error
        return SD_DAT_TABLE_WRITE_ERROR;
    }
    return RET_SUCCESS;
}

//read data from SD card
int readSD_Data(unsigned char subsystem,unsigned long index,unsigned char *dat){
    SD_block_addr src;
    SD_DATA_TABLE *block; 
    int resp;
    //check which subsystem is being used
    switch(subsystem){
        case BUS_ADDR_LEDL:
            src=SD_LEDL_DAT_START+index;
        break;
        case BUS_ADDR_ACDS:
            src=SD_ACDS_DAT_START+index;
        break;
        case BUS_ADDR_IMG:
            src=SD_IMG_DAT_START+index;
        break;
        default:
            //unknown subystem report error
            report_error(ERR_LEV_ERROR,COMM_ERR_SRC_DAT_STORE,COMM_ERR_DAT_READ_BAD_SUBSYSTEM,subsystem);
            //return error
            return SD_DAT_BAD_SUBSYSTEM;
    }
    resp=mmcReadBlock(src,block);
    //check for errors
    if(resp!=RET_SUCCESS){
        //TODO: handle error and return
        report_error(ERR_LEV_ERROR,COMM_ERR_SRC_DAT_STORE,COMM_ERR_DAT_STORE_BLOCK_READ,resp);
        //return error
        return SD_DAT_BLOCK_READ_ERROR;
    }
    return RET_SUCCESS;
}

