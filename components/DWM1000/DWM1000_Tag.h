/*
 * DWM1000_Tag.h
 *
 *  Created on: Feb 12, 2016
 *      Author: lieven
 */

#ifndef DWM1000_Tag_H_
#define DWM1000_Tag_H_

#include <NanoAkka.h>
#include <Mqtt.h>
#include <Hardware.h>
#include <DWM1000.h>
#include <DWM1000_Message.h>
#include <LogIsr.h>

#define ANCHOR_EXPIRE_TIME 30000
class Anchor
{
public:
    uint16_t _address;
    uint64_t _expires;
    uint8_t _sequence;
    int32_t _x;
    int32_t _y;
    uint32_t _distance;

    Anchor(uint16_t address, uint8_t sequence)
    {
        _address = address;
        _expires = Sys::millis() + ANCHOR_EXPIRE_TIME;
        _sequence = sequence;
        _x = 0;
        _y = 0;
        _distance = 0;
    }

    Anchor()
        : Anchor(0, 0)
    {
    }

    bool expired()
    {
        return Sys::millis() > _expires;
    }

    void update(uint8_t sequence)
    {
        _expires = Sys::millis() + ANCHOR_EXPIRE_TIME;
        if (sequence > (_sequence + 1))
            INFO_ISR(" dropped %d frames from %d", sequence - _sequence - 1, _address);
        _sequence = sequence;
    }

    void update(BlinkMsg& blinkMsg)
    {
        uint8_t sequence = blinkMsg.sequence;
        _expires = Sys::millis() + ANCHOR_EXPIRE_TIME;
        if (sequence > (_sequence + 1))
            INFO_ISR(" dropped %d frames from %d", sequence - _sequence - 1, _address);
        little_endian(_x, blinkMsg.x);
        little_endian(_y, blinkMsg.y);
        little_endian(_distance, blinkMsg.distance);
        _sequence = sequence;
    }

    void remove()
    {
        _address = 0;
    }

};

class DWM1000_Tag: public Actor, public DWM1000
{
    uint32_t _count;

    uint32_t _polls;
    uint32_t _resps;
    uint32_t _blinks;
    uint32_t _finals;
    uint32_t _errs;
    uint32_t _missed;
    uint32_t _timeouts;
    uint32_t _interruptDelay;
    BlinkMsg _blinkMsg;
    PollMsg _pollMsg;
    RespMsg _respMsg;
    FinalMsg _finalMsg;
    DwmMsg _dwmMsg;
    std::string _anchors;
    uint32_t _anchorIndex;
    std::string _panAddress;

    typedef enum {
        RCV_ANY,
        RCV_RESP,
        RCV_FINAL
    } State;
    Anchor* _currentAnchor;
    State _state;
    bool _pollTimerExpired;
    DigitalIn& _irq;
    const char* stateString();

public:
    TimerSource pollTimer;
    TimerSource expireTimer;
    TimerSource checkTimer;
    TimerSource logTimer;
    TimerSource pulseTimer;


    ValueSource<MqttMessage> mqttMsg;
    ValueSource<bool> blink;
    RefSource<uint32_t> polls;
    RefSource<uint32_t> resps;
    RefSource<uint32_t> blinks;
    RefSource<uint32_t> finals;
    RefSource<uint32_t> errs;
    RefSource<uint32_t> missed;
    RefSource<uint32_t> timeouts;
    RefSource<uint32_t> interruptCount;
    static DWM1000_Tag* _tag;
    uint32_t _interrupts;
    uint64_t _interruptStart;
    DWM1000_Tag(Thread& thr,Spi& spi, DigitalIn& irq, DigitalOut& reset,
                uint16_t shortAddress, uint8_t longAddress[6]);
    DWM1000_Tag(Thread&, Connector*  );
    ~DWM1000_Tag();
    void mode(uint32_t m);
    void init();
    void preStart();
    void wiring();
    void run();

    int sendFinalMsg();
    int sendPollMsg();
    static void rxcallback(const dwt_callback_data_t* event);
    static void txcallback(const dwt_callback_data_t* event);
    void FSM(const dwt_callback_data_t* signal);
    void onDWEvent(const dwt_callback_data_t* signal);
    FrameType readMsg(const dwt_callback_data_t* signal);
    void updateAnchors(uint16_t address, uint8_t sequence);
    void updateAnchors(BlinkMsg& blinkMsg);
    void expireAnchors();
    bool pollAnchor();
    void listAnchors(std::string& output);
    void handleBlinkMsg();
    void handleRespMsg();
    void enableRxd();
    void diag(const char* msg);
private:

};

#endif /* DWM1000_Tag_H_ */
