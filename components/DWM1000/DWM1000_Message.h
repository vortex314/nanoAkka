#include <stdint.h>
#ifndef DWM1000_Message_H_
#define DWM1000_Message_H_

#define FC_1_BLINK 0xC5
//#define FC_1_BLINK 0xC5
#define FC_1_SHORT 0x41
#define FC_2 0x8C
#define FC_2_SHORT 0x88

typedef  union {
    uint8_t buffer[1];
    struct {
        uint8_t fc[1];
        uint8_t sequence;
        uint8_t sourceLong[8];
        uint8_t sourceShort[2];
        uint8_t x[4];
        uint8_t y[4];
        uint8_t distance[4];
        uint8_t crc[2];
    } ;
    uint16_t getSrc() {
    	return sourceShort[0]+(sourceShort[1]<<8);
    };
} __attribute__ ((packed)) BlinkMsg ;



typedef  union {
    uint8_t buffer[1];
    struct {
        uint8_t fc[2];
        uint8_t sequence;
        uint8_t panId[2];
        uint8_t dst[2];
        uint8_t src[2];
        uint8_t function;
        uint8_t crc[2];
    } ;
    uint16_t getSrc() {
    	return src[0]+(src[1]<<8);
    };
    uint16_t getDst() {
    	return dst[0]+(dst[1]<<8);
    }
} __attribute__ ((packed)) PollMsg ;

typedef  union {
    uint8_t buffer[1];
    struct {
        uint8_t fc[2];
        uint8_t sequence;
        uint8_t panId[2];
        uint8_t dst[2];
        uint8_t src[2];
        uint8_t function;
        uint8_t activity;
        uint8_t activityParameter[2];
        uint8_t crc[2];
    } ;
    uint16_t getSrc() {
    	return src[0]+(src[1]<<8);
    };
    uint16_t getDst() {
    	return dst[0]+(dst[1]<<8);
    }
} __attribute__ ((packed)) RespMsg ;

typedef  union {
    uint8_t buffer[1];
    struct {
        uint8_t fc[2];
        uint8_t sequence;
        uint8_t panId[2];
        uint8_t dst[2];
        uint8_t src[2];
        uint8_t function;
        uint8_t pollTimestamp[4];
        uint8_t respTimestamp[4];
        uint8_t finalTimestamp[4];
        uint8_t crc[2];
    } ;
    uint16_t getSrc() {
    	return src[0]+(src[1]<<8);
    };
    uint16_t getDst() {
    	return dst[0]+(dst[1]<<8);
    }
} __attribute__ ((packed)) FinalMsg ;

typedef union {
    uint8_t buffer[1];
    BlinkMsg blinkMsg;
    FinalMsg finalMsg;
    PollMsg pollMsg;
    RespMsg respMsg;
    
    struct {
        uint8_t fc[2];
        uint8_t sequence;
        uint8_t panId[2];
        uint8_t dst[2];
        uint8_t src[2];
        uint8_t function;
 /*      union  {
            struct {
                uint8_t crc[2];
            } __attribute__ ((packed)) poll;
            struct {
                uint8_t activity;
                uint8_t activityParameter[2];
                uint8_t crc[2];
            } __attribute__ ((packed)) resp;
            struct {
                uint8_t pollTimestamp[4];
                uint8_t respTimestamp[4];
                uint8_t finalTimestamp[4];
                uint8_t crc[2];
            } __attribute__ ((packed)) finalMsg;
        };*/
    } __attribute__ ((packed)) ;
    uint16_t getSrc() {
    	return src[0]+(src[1]<<8);
    };
    uint16_t getDst() {
    	return dst[0]+(dst[1]<<8);
    }
} __attribute__ ((packed)) DwmMsg;


#define FUNC_POLL_MSG  0x21
#define FUNC_RESP_MSG  0x10
#define FUNC_FINAL_MSG  0x23
#define ACT_CODE 0x2

#endif
