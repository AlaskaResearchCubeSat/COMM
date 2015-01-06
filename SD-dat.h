#ifndef __SD_DAT_H
    #define __SD_DAT_H
    //needed for ERR_ADDR_END
    #include <Error.h>

    //sizes for data areas
    #define SD_LEDL_DAT_SIZE      (100000)
    #define SD_IMG_DAT_SIZE       (20000)
    #define SD_ACDS_DAT_SIZE      (20000)

    //Identifier for the data table
    #define DATA_TABLE_MAGIC      (0xAFCA)

    //storage location definitions for SD card
    enum{SD_DAT_TABLE=ERR_ADDR_END+1,SD_LEDL_DAT_START,SD_LEDL_DAT_END=SD_LEDL_DAT_START+SD_LEDL_DAT_SIZE,
         SD_IMG_DAT_START,SD_IMG_DAT_END=SD_IMG_DAT_START+SD_IMG_DAT_SIZE,
         SD_ACDS_DAT_START,SD_ACDS_DAT_END=SD_ACDS_DAT_START+SD_ACDS_DAT_SIZE};

    //return values for data_setup
    enum{SD_DAT_INIT=1,SD_DAT_INIT_WRITE_ERR=2,SD_DAT_INIT_ERR=-1,SD_DAT_BAD_SUBSYSTEM=-2,SD_DAT_BLOCK_WRITE_ERROR=-3,SD_DAT_TABLE_WRITE_ERROR=-4};

    typedef struct{
        //saved data values
        unsigned short next_LEDL;
        unsigned short next_IMG;
        unsigned short next_ACDS;
    }DATA_TABLE;

    //structure for keeping track of data in memory
    typedef struct{
        //ID for data
        unsigned short magic;
        //use a union to add padding
        union{
            DATA_TABLE table;
            //ensure a 512 byte structure
            unsigned char fill[508];
        }dat;
        //CRC from crc16
        unsigned short CRC;
    }SD_DATA_TABLE;
    
    
    
    extern DATA_TABLE sd_data_table;
    
    //initialize data writing
    int data_setup(void);
    //write data to SD card in the proper area
    int writeData(unsigned char subsystem,unsigned char type,unsigned char *dat);

#endif
    