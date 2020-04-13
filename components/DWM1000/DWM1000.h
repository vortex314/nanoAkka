/*
 * DWM1000_Tag.h
 *
 *  Created on: Feb 12, 2016
 *      Author: lieven
 */

#ifndef DWM1000_H_
#define DWM1000_H_
#include <NanoAkka.h>
#include <Log.h>
#include <Hardware.h>
#include <DWM1000_Message.h>
#include <Register.h>

extern "C" {

#include "deca_device_api.h"
#include "deca_regs.h"
#include "deca_sleep.h"
}

#define MAX_MESSAGE 5

typedef void (*InterruptHandler)(void* instance);
typedef enum {
    FT_BLINK,
    FT_POLL,
    FT_RESP,
    FT_FINAL,
    FT_UNKNOWN
} FrameType;

#define DWM_PIN_RESET D1
#define DWM_PIN_IRQ 4


template <typename T> void little_endian(uint8_t* dst,T v)
{
    for(int i=0; i<sizeof(T); i++) {
        dst[i]=v & 0xFF;
        v >>=8;
    }
}


template <typename T> void little_endian(T& v,uint8_t* src)
{
    v=0;
    for(int i=sizeof(T)-1; i>=0; i--) {
        v <<=8;
        v += src[i];
    }
}

class Timeout
{
public:
    uint32_t _interval;
    uint64_t _nextTimeout;
    Timeout(uint32_t interval)
    {
        _interval = interval;
        _nextTimeout = UINT64_MAX;
    }
    void setInterval(uint32_t interval);
    void reset()
    {
        _nextTimeout = Sys::millis() + _interval;
    }
    bool expired()
    {
        return Sys::millis() > _nextTimeout;
    };
};

class DWM1000
{
public:
    uint32_t _count;
    Spi& _spi;
    DigitalIn& _irq;
    DigitalOut& _reset;
    /*    DWM1000* _me;
     uint32_t _interrupts;
     uint32_t _polls;
     bool interrupt_detected;
     */

    uint8_t _longAddress[8];
    uint16_t _shortAddress;

    dwt_config_t _config;
    uint8_t _channel;
    uint8_t _prf;
    uint8_t _preambleLength;
    uint8_t _dataRate;
    uint8_t _pacSize;
    uint8_t _sequence;
    int32_t _x;             // abs pos in cm
    int32_t _y;
    uint32_t _distance;  // dist in cm
    typedef enum  {
        RCV_ANY, RCV_RESP
    } State;


public:
    DWM1000(Spi& spi,DigitalIn& irq,DigitalOut& reset,uint16_t shortAddress,uint8_t* longAddress);
    virtual ~DWM1000();
    void mode(uint32_t m);
    void init();
    void setup();
    void resetChip();

    bool isRespMsg();
    void sendFinalMsg();
    void setInterruptFunction(FunctionPointer fp,void* obj);
    void setShortAddress(uint16_t address);
    void setLongAddress(uint8_t address[]);
    static FrameType getFrameType(DwmMsg& msg);
    static FrameType getFrameType(uint8_t fc[]);
    void createBlinkFrame(BlinkMsg& blink);
    void createPollMsg(PollMsg& pollMsg, uint16_t address,uint8_t sequence);
    void createPollMsg(PollMsg& pollMsg, BlinkMsg& blinkMsg);
    void createRespMsg(RespMsg& respMsg, PollMsg& pollMsg);
    void createFinalMsg(FinalMsg& respMsg, RespMsg& pollMsg);
    bool isForMe(DwmMsg& dmwMsg);
    void tune();
    uint8_t sequence()
    {
        return _sequence;
    }
    void status();


private:

    DwmMsg _rcvMsg[MAX_MESSAGE];

};

#endif /* DWM1000_H_ */
