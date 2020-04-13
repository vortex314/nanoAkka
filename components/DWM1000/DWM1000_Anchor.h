/*
 * DWM1000_Anchor_Tag.h
 *
 *  Created on: Feb 12, 2016
 *      Author: lieven
 */

#ifndef DWM1000_Anchor_H_
#define DWM1000_Anchor_H_

#include <NanoAkka.h>
#include <Mqtt.h>
#include <Hardware.h>
#include <DWM1000_Message.h>
#include <DWM1000.h>
#include <LogIsr.h>
#include <Register.h>

class DWM1000_Anchor: public Actor, public DWM1000
{

    uint32_t _count;
    uint32_t _polls;
    uint32_t _finals;
    uint32_t _blinks;
    uint32_t _resps;
    uint32_t _errs;
    uint32_t _missed;
    uint32_t _timeouts;
    uint32_t _interruptDelay;
    uint8_t _lastSequence;
    PollMsg _pollMsg;
    RespMsg _respMsg;
    FinalMsg _finalMsg;
    BlinkMsg _blinkMsg;
    DwmMsg _dwmMsg;
    std::string _panAddress;
    bool _hasIrqEvent;
    float _distance;
    typedef enum {
        RCV_ANY,
        RCV_POLL,
        RCV_FINAL
    } State;
    State _state;
    bool _blinkTimerExpired;
    DigitalIn& _irq;
    const char* stateString();

public:
    TimerSource blinkTimer;
    TimerSource checkTimer;
    TimerSource logTimer;
    TimerSource pulseTimer;
    ValueSource<bool> poll;
    ValueSource<MqttMessage> mqttMsg;
    RefSource<uint32_t> polls;
    RefSource<uint32_t> blinks;
    RefSource<uint32_t> finals;
    RefSource<uint32_t> errs;
    RefSource<uint32_t> missed;
    RefSource<uint32_t> timeouts;
    RefSource<uint32_t> interruptCount;
    RefSource<float> distanceRef;
    ValueSource<uint16_t> address;
    static DWM1000_Anchor* _anchor;
    uint64_t _interruptStart;
    uint32_t _interrupts;

    DWM1000_Anchor(Thread& thr,Spi& spi, DigitalIn& irq, DigitalOut& reset,
                   uint16_t shortAddress, uint8_t longAddress[6]);
    ~DWM1000_Anchor();
    void mode(uint32_t m);
    void init();
    void preStart();
    void wiring();

    void run();

    void sendReply();
    void calcFinalMsg();
    int sendRespMsg();

    void update(uint16_t srcAddress, uint8_t sequence);
    static void rxcallback(const dwt_callback_data_t* event);
    static void txcallback(const dwt_callback_data_t* event);

    void FSM(const dwt_callback_data_t* signal);
//		void onDWEvent(const dwt_callback_data_t* event);
    FrameType readMsg(const dwt_callback_data_t* signal);
    void sendBlinkMsg();
    void handleFinalMsg();
    void enableRxd();
    void showRegs();
};

#endif /* DWM1000_Anchor_Tag_H_ */
