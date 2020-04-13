/*
 * DWM1000_Anchor.cpp
 *
 *  Created on: Feb 12, 2016
 *      Author: lieven
 */

#include <DWM1000_Anchor.h>
#include <Log.h>
#include <decaSpi.h>
#include <Config.h>

extern "C" {

#include "deca_device_api.h"
#include "deca_regs.h"
#include "deca_sleep.h"
}

#define RNG_DELAY_MS 1000 /* Inter-ranging delay period, in milliseconds. */
#define TX_ANT_DLY 16436 /* Default antenna delay values for 64 MHz PRF. See NOTE 1 below. */
#define RX_ANT_DLY 16436

/* Indexes to access some of the fields in the frames defined above. */
#define ALL_MSG_SN_IDX 2
#define FINAL_MSG_POLL_TX_TS_IDX 10
#define FINAL_MSG_RESP_RX_TS_IDX 14
#define FINAL_MSG_FINAL_TX_TS_IDX 18
#define FINAL_MSG_TS_LEN 4
/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
 * 1 uus = 512 / 499.2 µs and 1 µs = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 65536
/* Delay between frames, in UWB microseconds. See NOTE 4 below. */
/* This is the delay from the end of the frame transmission to the enable of the receiver, as programmed for the DW1000's wait for response feature. */
//#define POLL_TX_TO_RESP_RX_DLY_UUS 150
/* This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW1000's delayed TX function. This includes the
 * frame length of approximately 2.66 ms with above configuration. */
//
#define RESP_RX_TO_FINAL_TX_DLY_UUS 3100
/* Receive response timeout. See NOTE 5 below. */
#define RESP_RX_TIMEOUT_UUS 2700

/* Time-stamps of frames transmission/reception, expressed in device time units.
 * As they are 40-bit wide, we need to define a 64-bit int type to handle them. */
typedef unsigned long long uint64;

/* Delay between frames, in UWB microseconds. See NOTE 4 below. */
/* This is the delay from Frame RX timestamp to TX reply timestamp used for calculating/setting the DW1000's delayed TX function. This includes the
 * frame length of approximately 2.46 ms with above configuration. */
#define POLL_RX_TO_RESP_TX_DLY_UUS 2600
#define RESP_TX_TO_FINAL_RX_DLY_UUS 500 /* This is the delay from the end of the frame transmission to the enable of the receiver, as programmed for the DW1000's wait for response feature. */
#define FINAL_RX_TIMEOUT_UUS 3300   /* Receive final timeout. See NOTE 5 below. */

/* Timestamps of frames transmission/reception.
 * As they are 40-bit wide, we need to define a 64-bit int type to handle them. */
typedef signed long long int64;

static uint64 poll_rx_ts;
static uint64 resp_tx_ts;
static uint64 final_rx_ts;

/* Speed of light in air, in metres per second. */
#define SPEED_OF_LIGHT 299702547

/* Hold copies of computed time of flight and distance here for reference, so reader can examine it at a breakpoint. */
static double tof;
static double distance;

/* Declaration of static functions. */
static uint64 get_tx_timestamp_u64(void);
static uint64 get_rx_timestamp_u64(void);
static void final_msg_get_ts(const uint8 *ts_field, uint32 *ts);

static uint32_t lastStatus = 0;
static uint32_t lastEvent = 0;

const char* DWM1000_Anchor::stateString()
{
    return _state == RCV_ANY ? "RCV_ANY" :
           ( _state== RCV_POLL  ? "RCV_POLL" :
             ( _state== RCV_FINAL ? "RCV_FINAL" : "unknown state"));

}
DWM1000_Anchor* DWM1000_Anchor::_anchor;

void anchorInterruptHandler(void* obj)
{
    DWM1000_Anchor::_anchor->_interruptStart = Sys::micros();
    DWM1000_Anchor::_anchor->_interrupts++;
    dwt_isr();
}

//_________________________________________________ IRQ handler
void DWM1000_Anchor::rxcallback(const dwt_callback_data_t* signal)
{
    _anchor->FSM(signal);
}

void DWM1000_Anchor::txcallback(const dwt_callback_data_t* signal)
{
    _anchor->FSM(signal);
}

DWM1000_Anchor::DWM1000_Anchor(Thread& thr, Spi& spi, DigitalIn& irq,
                               DigitalOut& reset, uint16_t shortAddress, uint8_t longAddress[6])
    : Actor(thr),
      DWM1000(spi, irq, reset, shortAddress, longAddress), _irq(irq),
      blinkTimer(thr,1, 1000,true),
      checkTimer(thr,3,5000,true),
      logTimer(thr,4,1000,true),
      pulseTimer(thr,5,10,true),
      polls(_polls),
      blinks(_blinks),
      finals(_finals),
      errs(_errs),
      missed(_missed),
      timeouts(_timeouts),
      interruptCount(_interrupts),
      distanceRef(_distance)
//int pin = 5;   // RESET PIN == D1 == GPIO5

// PIN_IRQ_IN 4// PIN == D2 == GPIO4
{
    setLongAddress(longAddress);
    setShortAddress(shortAddress);
    _count = 0;
    _interrupts = 0;
    _polls = 0;
    _finals = 0;
    _blinks = 0;
    _resps = 0;
    _errs = 0;
    _missed = 0;
    _timeouts = 0;
    _anchor = this;
    _hasIrqEvent = false;
    _state = RCV_ANY;
    _interruptDelay = 0;
    _interruptStart = 0;
    _lastSequence = 0;
    _blinkTimerExpired = false;
    _distance = 0;
}

DWM1000_Anchor::~DWM1000_Anchor()
{

}

void DWM1000_Anchor::init()
{
    dwt_setcallbacks(txcallback, rxcallback);
    dwt_setdblrxbuffmode(false);
    dwt_enableframefilter(DWT_FF_DATA_EN | DWT_FF_BEACON_EN);
    dwt_setinterrupt(DWT_INT_RFCG | DWT_INT_RFCE | DWT_INT_RFTO | DWT_INT_TFRS, 1); // enable

    _blinkTimerExpired = false;
    dwt_setautorxreenable(true);
    dwt_setrxtimeout(60000);
    dwt_rxenable(0);

}

void DWM1000_Anchor::preStart()
{
//_________________________________________________INIT SPI ESP8266
    INFO("DWM1000 ANCHOR started.");
    _count = 0;
    wiring();

    _irq.onChange(DigitalIn::DIN_RAISE, anchorInterruptHandler, this);
    DWM1000::setup();
    init();

}

static Register reg_sys_status2("SYS_STATUS", "ICRBP HSRBP AFFREJ TXBERR HPDWARN RXSFDTO CLKPLL_LL RFPLL_LL "
                                "SLP2INIT GPIOIRQ RXPTO RXOVRR F LDEERR RXRFTO RXRFSL RXFCE RXFCG "
                                "RXDFR RXPHE RXPHD LDEDONE RXSFDD RXPRD TXFRS TXPHS TXPRS TXFRB AAT "
                                "ESYNCR CPLOCK IRQSD");

void DWM1000_Anchor::wiring()
{
    logTimer >> ([&](const TimerMsg& tm) {
        INFO(" int: %d to:%d blk: %d pol: %d rsp: %d fin: %d dist: %.1f delay: %d usec", _interrupts, _timeouts, _blinks, _polls, _resps, _finals, _distance, _interruptDelay);
        std::string topic="anchor/poller";
        std::string message;
        string_format(message,"%u:%u",_pollMsg.getSrc(),_pollMsg.getDst());
        mqttMsg.emit({topic,message});
    });

    pulseTimer >> ([&](const TimerMsg& tm) {
        static uint32_t _oldPolls=0;
        if ( _polls > _oldPolls) {
            poll=true;
        }
        _oldPolls=_polls;
        static float oldDistance=0.0;
        if ( _distance !=  oldDistance ) {
            distanceRef=_distance; // force pub
        }
        oldDistance=_distance;
    });

    blinkTimer >> ([&](const TimerMsg& tm) {
        _blinkTimerExpired=true;
    });

    checkTimer >> ([&](const TimerMsg& tm) {
        static uint32_t oldInterrupts=0;
        static uint32_t oldPolls=0;
        if ( _polls == oldPolls || _interrupts==oldInterrupts) {
            INFO(" missing interrupts or polls ");
            reg_sys_status2.value(lastStatus);
            reg_sys_status2.show();
            status();
        }
        oldInterrupts=_interrupts;
        oldPolls=_polls;
    });


}

void DWM1000_Anchor::run()
{
    static uint32_t oldInterrupts;

    if (oldInterrupts == _interrupts) {
        status();
        enableRxd();
    }
    oldInterrupts = _interrupts;

}

void DWM1000_Anchor::sendBlinkMsg()
{
    int erc;
    uint32_t dist = _distance * 100;
    createBlinkFrame(_blinkMsg);
    little_endian(_blinkMsg.distance, dist);
    dwt_writetxdata(sizeof(_blinkMsg), _blinkMsg.buffer, 0);
    dwt_writetxfctrl(sizeof(_blinkMsg), 0);
    erc = dwt_starttx(DWT_START_TX_IMMEDIATE);
    if (erc < 0) WARN_ISR("WARN blink Txd failed");
//    INFO("blink ");
}

int DWM1000_Anchor::sendRespMsg()
{
    uint32 resp_tx_time;
    poll_rx_ts = get_rx_timestamp_u64(); /* Retrieve poll reception timestamp. */
    /* Set send time for response. See NOTE 8 below. */
    resp_tx_time = (poll_rx_ts + (POLL_RX_TO_RESP_TX_DLY_UUS * UUS_TO_DWT_TIME))
                   >> 8;
    dwt_setdelayedtrxtime(resp_tx_time);
    dwt_setrxaftertxdelay(RESP_TX_TO_FINAL_RX_DLY_UUS);
    dwt_setrxtimeout(FINAL_RX_TIMEOUT_UUS);
    dwt_writetxdata(sizeof(_respMsg), _respMsg.buffer, 0);
    dwt_writetxfctrl(sizeof(_respMsg), 0);
    if (dwt_starttx(DWT_START_TX_DELAYED | DWT_RESPONSE_EXPECTED) < 0) {
        _missed++;
        return -1;
    }
    return 0;
}

//===================================================================================

void DWM1000_Anchor::calcFinalMsg()
{
    uint32 poll_tx_ts, resp_rx_ts, final_tx_ts;
    uint32 poll_rx_ts_32, resp_tx_ts_32, final_rx_ts_32;
    double Ra, Rb, Da, Db;
    int64 tof_dtu;

    /* Retrieve response transmission and final reception timestamps. */
    resp_tx_ts = get_tx_timestamp_u64();
    final_rx_ts = get_rx_timestamp_u64();

    /* Get timestamps embedded in the final message. */
    final_msg_get_ts(&_dwmMsg.buffer[FINAL_MSG_POLL_TX_TS_IDX], &poll_tx_ts);
    final_msg_get_ts(&_dwmMsg.buffer[FINAL_MSG_RESP_RX_TS_IDX], &resp_rx_ts);
    final_msg_get_ts(&_dwmMsg.buffer[FINAL_MSG_FINAL_TX_TS_IDX], &final_tx_ts);

    /* Compute time of flight. 32-bit subtractions give correct answers even if clock has wrapped. See NOTE 10 below. */
    poll_rx_ts_32 = (uint32) poll_rx_ts;
    resp_tx_ts_32 = (uint32) resp_tx_ts;
    final_rx_ts_32 = (uint32) final_rx_ts;
    Ra = (double) (resp_rx_ts - poll_tx_ts);
    Rb = (double) (final_rx_ts_32 - resp_tx_ts_32);
    Da = (double) (final_tx_ts - resp_rx_ts);
    Db = (double) (resp_tx_ts_32 - poll_rx_ts_32);
    tof_dtu = (int64) ((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));

    tof = tof_dtu * DWT_TIME_UNITS;
    distance = tof * SPEED_OF_LIGHT;
    _distance = distance * 100.0;
    /*    INFO(" >>>>>>>>> distance : %f for TAG : %X ", distance,
     (_finalMsg.src[0] << 8) + _finalMsg.src[1]); */

}

//===================================================================================

FrameType DWM1000_Anchor::readMsg(const dwt_callback_data_t* signal)
{
    uint32_t frameLength = signal->datalength;
    if (frameLength <= sizeof(_dwmMsg)) {
        dwt_readrxdata(_dwmMsg.buffer, frameLength, 0);

        FrameType ft = DWM1000::getFrameType(_dwmMsg);
        if (ft == FT_BLINK) {
            memcpy(_blinkMsg.buffer, _dwmMsg.buffer, sizeof(_blinkMsg));
            DEBUG_ISR(" blink %d : %d : %s", _blinkMsg.getSrc(), _blinkMsg
                      .sequence, stateString());
            _blinks++;
        } else if (ft == FT_POLL) {
            memcpy(_pollMsg.buffer, _dwmMsg.buffer, sizeof(_pollMsg));
            DEBUG_ISR(" poll %d : %d : %s", _pollMsg.getSrc(), _pollMsg.sequence, stateString());
            _polls++;
        } else if (ft == FT_RESP) {
            memcpy(_respMsg.buffer, _dwmMsg.buffer, sizeof(_respMsg));
            DEBUG_ISR(" resp %d : %d : %s ", _respMsg.getSrc(), _respMsg
                      .sequence, stateString());
            _resps++;
        } else if (ft == FT_FINAL) {
            memcpy(_finalMsg.buffer, _dwmMsg.buffer, sizeof(_finalMsg));
            DEBUG_ISR(" final %d : %d : %s", _finalMsg.getSrc(), _finalMsg
                      .sequence, stateString());
            _finals++;
        } else {
            WARN_ISR("WARN unknown frame type %X:%X : %s", _dwmMsg.fc[0], _dwmMsg
                     .fc[1],stateString());
        }
        return ft;
    } else {
        WARN_ISR("WARN invalid length %d : hdr %X:%X : %s", frameLength, _dwmMsg
                 .fc[0], _dwmMsg.fc[1], stateString());
        return FT_UNKNOWN;
    }
}

//===================================================================================
void DWM1000_Anchor::update(uint16_t src, uint8_t sequence)
{
    if (sequence > (_lastSequence + 1)) {
        WARN_ISR("dropped frames : %d", sequence - _lastSequence - 1);
    }
    _lastSequence = sequence;
}

void DWM1000_Anchor::enableRxd()
{
    dwt_setautorxreenable(true);
    dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_GOOD | SYS_STATUS_ALL_RX_ERR | SYS_STATUS_ALL_TX);
    dwt_setrxtimeout(60000); // 60 msec ?
    dwt_rxenable(0);
}

void DWM1000_Anchor::FSM(const dwt_callback_data_t* signal)
{
    lastStatus = signal->status;
    lastEvent = signal->event;
    if (signal->event == DWT_SIG_RX_OKAY) {
        FrameType ft = readMsg(signal);
        if (ft == FT_POLL && _dwmMsg.getDst() == _shortAddress) {
            update(_dwmMsg.getSrc(), _dwmMsg.sequence);
            createRespMsg(_respMsg, _pollMsg);
            _resps++;
            if (sendRespMsg() != 0) {
                WARN_ISR(" sendRespMsg fails ");
            } else {
                enableRxd();
            }
        } else if (ft == FT_FINAL) {
            calcFinalMsg();
//			sendBlinkMsg();
//			_blinks++;
            enableRxd();
        } else {
            WARN_ISR("WARN unexpected frame type %d", ft);
            enableRxd();
        }
    } else if (signal->event == DWT_SIG_RX_TIMEOUT) {
        _timeouts++;
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXRFTO); // Clear RX timeout event bit
        if (_blinkTimerExpired) {
            sendBlinkMsg();
            _blinks++;
            _blinkTimerExpired = false;
        }
        enableRxd();
    } else if (signal->event == DWT_SIG_TX_DONE) {
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_TX); // Clear TX event bit
    } else {
        WARN_ISR("WARN unhandled event %d", signal->event);
        enableRxd();
    }
    _interruptDelay = Sys::micros() - _interruptStart;
}

//===================================================================================

//Timer opsTime("DW1000 Action", 10);
//uint64_t _startTime;

#include <malloc.h>

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_tx_timestamp_u64()
 *
 * @brief Get the TX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
static uint64 get_tx_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64 ts = 0;
    int i;
    dwt_readtxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--) {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_rx_timestamp_u64()
 *
 * @brief Get the RX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
static uint64 get_rx_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64 ts = 0;
    int i;
    dwt_readrxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--) {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn final_msg_get_ts()
 *
 * @brief Read a given timestamp value from the final message. In the timestamp fields of the final message, the least
 *        significant byte is at the lower address.
 *
 * @param  ts_field  pointer on the first byte of the timestamp field to read
 *         ts  timestamp value
 *
 * @return none
 */
static void final_msg_get_ts(const uint8 *ts_field, uint32 *ts)
{
    int i;
    *ts = 0;
    for (i = 0; i < FINAL_MSG_TS_LEN; i++) {
        *ts += ts_field[i] << (i * 8);
    }
}

/*****************************************************************************************************************************************************
 * NOTES:
 *
 * 1. The sum of the values is the TX to RX antenna delay, experimentally determined by a calibration process. Here we use a hard coded typical value
 *    but, in a real application, each device should have its own antenna delay properly calibrated to get the best possible precision when performing
 *    range measurements.
 * 2. The messages here are similar to those used in the DecaRanging ARM application (shipped with EVK1000 kit). They comply with the IEEE
 *    802.15.4 standard MAC data frame encoding and they are following the ISO/IEC:24730-62:2013 standard. The messages used are:
 *     - a poll message sent by the initiator to trigger the ranging exchange.
 *     - a response message sent by the responder allowing the initiator to go on with the process
 *     - a final message sent by the initiator to complete the exchange and provide all information needed by the responder to compute the
 *       time-of-flight (distance) estimate.
 *    The first 10 bytes of those frame are common and are composed of the following fields:
 *     - byte 0/1: frame control (0x8841 to indicate a data frame using 16-bit addressing).
 *     - byte 2: sequence number, incremented for each new frame.
 *     - byte 3/4: PAN ID (0xDECA).
 *     - byte 5/6: destination address, see NOTE 3 below.
 *     - byte 7/8: source address, see NOTE 3 below.
 *     - byte 9: function code (specific values to indicate which message it is in the ranging process).
 *    The remaining bytes are specific to each message as follows:
 *    Poll message:
 *     - no more data
 *    Response message:
 *     - byte 10: activity code (0x02 to tell the initiator to go on with the ranging exchange).
 *     - byte 11/12: activity parameter, not used for activity code 0x02.
 *    Final message:
 *     - byte 10 -> 13: poll message transmission timestamp.
 *     - byte 14 -> 17: response message reception timestamp.
 *     - byte 18 -> 21: final message transmission timestamp.
 *    All messages end with a 2-byte checksum automatically set by DW1000.
 * 3. Source and destination addresses are hard coded constants in this example to keep it simple but for a real product every device should have a
 *    unique ID. Here, 16-bit addressing is used to keep the messages as short as possible but, in an actual application, this should be done only
 *    after an exchange of specific messages used to define those short addresses for each device participating to the ranging exchange.
 * 4. Delays between frames have been chosen here to ensure proper synchronisation of transmission and reception of the frames between the initiator
 *    and the responder and to ensure a correct accuracy of the computed distance. The user is referred to DecaRanging ARM Source Code Guide for more
 *    details about the timings involved in the ranging process.
 * 5. This timeout is for complete reception of a frame, i.e. timeout duration must take into account the length of the expected frame. Here the value
 *    is arbitrary but chosen large enough to make sure that there is enough time to receive the complete final frame sent by the responder at the
 *    110k data rate used (around 3.5 ms).
 * 6. In a real application, for optimum performance within regulatory limits, it may be necessary to set TX pulse bandwidth and TX power, (using
 *    the dwt_configuretxrf API call) to per device calibrated values saved in the target system or the DW1000 OTP memory.
 * 7. We use polled mode of operation here to keep the example as simple as possible but all status events can be used to generate interrupts. Please
 *    refer to DW1000 User Manual for more details on "interrupts". It is also to be noted that STATUS register is 5 bytes long but, as the event we
 *    use are all in the first bytes of the register, we can use the simple dwt_read32bitreg() API call to access it instead of reading the whole 5
 *    bytes.
 * 8. Timestamps and delayed transmission time are both expressed in device time units so we just have to add the desired response delay to poll RX
 *    timestamp to get response transmission time. The delayed transmission time resolution is 512 device time units which means that the lower 9 bits
 *    of the obtained value must be zeroed. This also allows to encode the 40-bit value in a 32-bit words by shifting the all-zero lower 8 bits.
 * 9. dwt_writetxdata() takes the full size of the message as a parameter but only copies (size - 2) bytes as the check-sum at the end of the frame is
 *    automatically appended by the DW1000. This means that our variable could be two bytes shorter without losing any data (but the sizeof would not
 *    work anymore then as we would still have to indicate the full length of the frame to dwt_writetxdata()). It is also to be noted that, when using
 *    delayed send, the time set for transmission must be far enough in the future so that the DW1000 IC has the time to process and start the
 *    transmission of the frame at the wanted time. If the transmission command is issued too late compared to when the frame is supposed to be sent,
 *    this is indicated by an error code returned by dwt_starttx() API call. Here it is not tested, as the values of the delays between frames have
 *    been carefully defined to avoid this situation.
 * 10. The high order byte of each 40-bit time-stamps is discarded here. This is acceptable as, on each device, those time-stamps are not separated by
 *     more than 2**32 device time units (which is around 67 ms) which means that the calculation of the round-trip delays can be handled by a 32-bit
 *     subtraction.
 * 11. The user is referred to DecaRanging ARM application (distributed with EVK1000 product) for additional practical example of usage, and to the
 *     DW1000 API Guide for more details on the DW1000 driver functions.
 ****************************************************************************************************************************************************/
