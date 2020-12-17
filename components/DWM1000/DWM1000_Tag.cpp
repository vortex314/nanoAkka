/*
 * DWM1000_Tag.cpp
 *
 *  Created on: Feb 12, 2016
 *      Author: lieven
 */

#include <DWM1000_Tag.h>
#include <Log.h>
#include <decaSpi.h>

extern "C" {
#include "deca_device_api.h"
#include "deca_regs.h"
#include "deca_sleep.h"
}
#define MAX_ANCHORS 10
Anchor anchors[MAX_ANCHORS];

uint32_t anchorsCount()
{
    uint32_t count = 0;
    for (uint32_t i = 0; i < MAX_ANCHORS; i++)
        if (anchors[i]._address != 0)
            count++;
    return count;
}

Anchor* anchorsFind(uint16_t address)
{
    for (uint32_t i = 0; i < MAX_ANCHORS; i++)
        if (anchors[i]._address == address)
            return &anchors[i];
    return 0;
}

Anchor* newAnchor(uint16_t address, uint32_t sequence)
{
    uint32_t nextFree = 0;
    for (uint32_t i = 0; i < MAX_ANCHORS; i++)
        if (anchors[i]._address == 0) {
            nextFree = i;
            break;
        }
    anchors[nextFree]._sequence = sequence;
    anchors[nextFree]._address = address;
    anchors[nextFree]._expires = Sys::millis() + ANCHOR_EXPIRE_TIME;
    return &anchors[nextFree];
}

/* Inter-ranging delay period, in milliseconds. */
#define RNG_DELAY_MS 1000
/* Default antenna delay values for 64 MHz PRF. See NOTE 1 below. */
#define TX_ANT_DLY 16436
#define RX_ANT_DLY 16436
#define FINAL_MSG_TS_LEN 4

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion
 * factor.
 * 1 uus = 512 / 499.2 µs and 1 µs = 499.2 * 128 dtu. */
#define UUS_TO_DWT_TIME 65536

/* Delay between frames, in UWB microseconds. See NOTE 4 below. */
/* This is the delay from the end of the frame transmission to the enable of the
 * receiver, as programmed for the DW1000's wait for response feature. */
#define POLL_TX_TO_RESP_RX_DLY_UUS 150
/* This is the delay from Frame RX timestamp to TX reply timestamp used for
 * calculating/setting the DW1000's delayed TX function. This includes the
 * frame length of approximately 2.66 ms with above configuration. */
#define RESP_RX_TO_FINAL_TX_DLY_UUS 3100
/* Receive response timeout. See NOTE 5 below. */
#define RESP_RX_TIMEOUT_UUS 2700

/* Time-stamps of frames transmission/reception, expressed in device time units.
 * As they are 40-bit wide, we need to define a 64-bit int type to handle them.
 */
typedef unsigned long long uint64;
static uint64 poll_tx_ts;
static uint64 resp_rx_ts;
static uint64 final_tx_ts;

/* Declaration of static functions. */
static uint64 get_tx_timestamp_u64(void);
static uint64 get_rx_timestamp_u64(void);
static void final_msg_set_ts(uint8* ts_field, uint64 ts);

static uint32_t lastStatus = 0;
static uint32_t lastEvent = 0;

static Register reg_sys_status2(
    "SYS_STATUS",
    "ICRBP HSRBP AFFREJ TXBERR HPDWARN RXSFDTO CLKPLL_LL RFPLL_LL "
    "SLP2INIT GPIOIRQ RXPTO RXOVRR F LDEERR RXRFTO RXRFSL RXFCE RXFCG "
    "RXDFR RXPHE RXPHD LDEDONE RXSFDD RXPRD TXFRS TXPHS TXPRS TXFRB AAT "
    "ESYNCR CPLOCK IRQSD");
//_________________________________________________  IRQ Handler
void tagInterruptHandler(void* obj)
{
    DWM1000_Tag::_tag->_interruptStart = Sys::micros();
    DWM1000_Tag::_tag->_interrupts++;
    dwt_isr();
}
void DWM1000_Tag::rxcallback(const dwt_callback_data_t* signal)
{
    _tag->FSM(signal);
}

void DWM1000_Tag::txcallback(const dwt_callback_data_t* signal)
{
    _tag->FSM(signal);
}
//__________________________________________________
DWM1000_Tag* DWM1000_Tag::_tag = 0;

DWM1000_Tag::DWM1000_Tag(Thread& thr, Spi& spi, DigitalIn& irq,
                         DigitalOut& reset, uint16_t shortAddress,
                         uint8_t longAddress[6])
    :  Actor(thr),DWM1000(spi, irq, reset, shortAddress, longAddress),
       _irq(irq),
       pollTimer(thr, 100,true),
       expireTimer(thr,1000,true),
       checkTimer(thr,5000,true),
       logTimer(thr,1000,true),
       pulseTimer(thr,10,true),
       polls(_polls),
       resps(_resps),
       blinks(_blinks),
       finals(_finals),
       errs(_errs),
       missed(_missed),
       timeouts(_timeouts),
       interruptCount(_interrupts)

{
    _state = RCV_ANY;
    _count = 0;
    _interrupts = 0;
    _polls = 0;
    _finals = 0;
    _blinks = 0;
    _resps = 0;
    _errs = 0;
    _missed = 0;
    _timeouts = 0;
    _state = RCV_ANY;
    _tag = this;
    _currentAnchor = &anchors[0];
    _anchorIndex = 0;
    _interruptDelay = 0;
    _interruptStart = 0;
    _pollTimerExpired = false;
    _panAddress = "ABC";
    _count = 0;
}

DWM1000_Tag::DWM1000_Tag(Thread& thread,Connector* connector )
    : DWM1000_Tag(thread, connector->getSPI(), connector->getDigitalIn(LP_RXD),
                  connector->getDigitalOut(LP_TXD), 0xABCD,
                  (uint8_t*)"ABCDEF") {}

DWM1000_Tag::~DWM1000_Tag() {}

void DWM1000_Tag::preStart()
{
    INFO("DWM1000 TAG started.");
    wiring();

    _irq.onChange(DigitalIn::DIN_RAISE, tagInterruptHandler, this);
    DWM1000::setup();
    init();
}

void DWM1000_Tag::wiring()
{
    INFO("");
    pulseTimer >> ([&](const TimerMsg& tm) {
        static uint32_t _oldBlinks=0;
        if ( _blinks > _oldBlinks) {
            blink=true;
        }
        _oldBlinks=_blinks;
    });
    pollTimer >> ([&](const TimerMsg& tm) {
        _pollTimerExpired=true;
    });
    logTimer >> ([&](const TimerMsg& tm) {
        INFO("interr: %d TO:%d blink: %d poll: %d resp: %d final:%d anchors: %d delay:%d usec", _interrupts, _timeouts, _blinks, _polls, _resps, _finals, anchorsCount(), _interruptDelay);
        for(int i=0; i< MAX_ANCHORS; i++) {
            if ( anchors[i]._address!=0) {
                std::string topic;
                std::string message;

                string_format(topic,"anchors/%d",anchors[i]._address);
                string_format(message,"{\"x\":%d,\"y\":%d,\"distance\":%d}",
                              anchors[i]._x,
                              anchors[i]._y,
                              anchors[i]._distance);
                mqttMsg.emit({topic,message});
            }
        }

    });

    checkTimer >> ([&](const TimerMsg& tm) {
        static uint32_t oldInterrupts = 0;
        if (oldInterrupts == _interrupts) {
            INFO(" missing interrupts , lastEvent : %d lastStatus : 0x%X",lastEvent,lastStatus);
            reg_sys_status2.value(lastStatus);
            reg_sys_status2.show();
            status();
            enableRxd();
        }
        oldInterrupts = _interrupts;
    });

    expireTimer >> ([&](const TimerMsg& tm) {
        expireAnchors();
    });
}

void DWM1000_Tag::init()
{
    dwt_setcallbacks(txcallback, rxcallback);
    dwt_setinterrupt(DWT_INT_RFCG | DWT_INT_RFCE | DWT_INT_RFTO | DWT_INT_TFRS |
                     DWT_INT_SFDT | DWT_INT_RPHE | DWT_INT_RFSL,
                     1);

    /*	dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
     dwt_setdblrxbuffmode(false);
     dwt_setinterrupt(DWT_INT_RFCG | DWT_INT_RFCE | DWT_INT_RFTO | DWT_INT_TFRS,
     1);
     // enable receive frame good, receive frame bad fcs, receive TO
     dwt_setautorxreenable(true);
     dwt_setrxtimeout(5000);
     dwt_rxenable(0);*/

    dwt_setrxantennadelay(RX_ANT_DLY);
    dwt_settxantennadelay(TX_ANT_DLY);

    /* Set expected response's delay and timeout. See NOTE 4, 5 and 6 below.
     * As this example only handles one incoming frame with always the same
     * delay and timeout, those values can be set here once for all. */
    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
    //   dwt_setpreambledetecttimeout(PRE_TIMEOUT);
}

void DWM1000_Tag::diag(const char* msg)
{
    uint32_t sys_mask, sys_status, sys_state;

    sys_mask = dwt_read32bitreg(SYS_MASK_ID);
    sys_status = dwt_read32bitreg(SYS_STATUS_ID);
    sys_state = dwt_read32bitreg(SYS_STATE_ID);
    INFO(" %s SYS_MASK : %X SYS_STATUS : %X SYS_STATE: %X state : %s IRQ : %d", msg, sys_mask, sys_status, sys_state, stateString(), _irq
         .read());
}

void DWM1000_Tag::run() {}

int DWM1000_Tag::sendPollMsg()
{
    _polls++;
    dwt_writetxdata(sizeof(_pollMsg), _pollMsg.buffer, 0);
    dwt_writetxfctrl(sizeof(_pollMsg), 0);
    dwt_setrxaftertxdelay(POLL_TX_TO_RESP_RX_DLY_UUS);
    dwt_setrxtimeout(RESP_RX_TIMEOUT_UUS);
    return dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);
}

int DWM1000_Tag::sendFinalMsg()
{
    _finals++;
    uint32 final_tx_time;

    /* Retrieve poll transmission and response reception timestamp. */
    poll_tx_ts = get_tx_timestamp_u64();
    resp_rx_ts = get_rx_timestamp_u64();

    /* Compute final message transmission time. See NOTE 9 below. */
    final_tx_time =
        (resp_rx_ts + (RESP_RX_TO_FINAL_TX_DLY_UUS * UUS_TO_DWT_TIME)) >> 8;

    /* Final TX timestamp is the transmission time we programmed plus the TX
     * antenna delay. */
    final_tx_ts = (((uint64)(final_tx_time & 0xFFFFFFFE)) << 8) + TX_ANT_DLY;

    final_msg_set_ts(_finalMsg.pollTimestamp, poll_tx_ts);
    final_msg_set_ts(_finalMsg.respTimestamp, resp_rx_ts);
    final_msg_set_ts(_finalMsg.finalTimestamp, final_tx_ts);

    dwt_setdelayedtrxtime(final_tx_time);
    dwt_writetxdata(sizeof(_finalMsg), _finalMsg.buffer, 0);
    dwt_writetxfctrl(sizeof(_finalMsg), 0);

    return dwt_starttx(DWT_START_TX_DELAYED);
}

void DWM1000_Tag::handleBlinkMsg()
{
    updateAnchors(_blinkMsg);
}
/*
 std::string strLogTag;

 void logTag(const char* s, uint32_t state, uint8_t* buffer, uint32_t length) {
 strLogTag = ":";
 for (int i = 0; i < length; i++)
 strLogTag += buffer[i] + ":";
 INFO("%s %s %s", s, Uid::label(state), strLogTag.c_str());
 }*/

void DWM1000_Tag::updateAnchors(BlinkMsg& blinkMsg)
{
    uint16_t address = blinkMsg.getSrc();
    Anchor* rap;

    if ((rap = anchorsFind(address)) == 0) {
        rap = newAnchor(address, blinkMsg.sequence);
        little_endian(rap->_x, blinkMsg.x);
        little_endian(rap->_y, blinkMsg.y);
        little_endian(rap->_distance, blinkMsg.distance);
        INFO_ISR(" +1 anchor : %d x:%d y:%d dist: %d", address, rap->_x,
                 rap->_y, rap->_distance);
    } else {
        rap->update(blinkMsg);
        DEBUG_ISR(" upd anchor : %d x:%d y:%d dist: %d", address, rap->_x,
                  rap->_y, rap->_distance);
    }
}

void DWM1000_Tag::listAnchors(std::string& output)
{
    output.clear();
    std::string anchor;
    for (uint32_t i = 0; i < MAX_ANCHORS; i++)
        if (anchors[i]._address != 0) {
            string_format(anchor,
                          "{ address=%d, x=%d , y = %d, distance = %d  } ",
                          anchors[i]._address, anchors[i]._x, anchors[i]._y,
                          anchors[i]._distance);
            output += anchor;
        }
}

void DWM1000_Tag::updateAnchors(uint16_t address, uint8_t sequence)
{
    Anchor* rap;
    if ((rap = anchorsFind(address)) == 0) {
        newAnchor(address, sequence);
    } else {
        rap->_sequence = sequence;
        rap->_expires = Sys::millis() + ANCHOR_EXPIRE_TIME;
    }
}

void DWM1000_Tag::expireAnchors()
{

    for (uint32_t i = 0; i < MAX_ANCHORS; i++)
        if (anchors[i]._address != 0) {
            if (anchors[i].expired()) {
                INFO_ISR(" expire anchor : %d ", anchors[i]._address);
                anchors[i].remove();
            }
        }
}

bool DWM1000_Tag::pollAnchor()
{
    //    INFO(" anchors : %d in %d ",_anchorIndex,anchors.size());
    for (int i = 0; i < MAX_ANCHORS; i++) {
        _anchorIndex++;
        _anchorIndex %= MAX_ANCHORS;
        if (anchors[_anchorIndex]._address != 0) {
            break;
        }
    }
    INFO_ISR(" poll anchor[%d].addr= %d ", _anchorIndex,
             anchors[_anchorIndex]._address);
    _currentAnchor = &anchors[_anchorIndex];
    if (_currentAnchor->_address != 0) {
        _currentAnchor->_sequence++;
        createPollMsg(_pollMsg, _currentAnchor->_address,
                      _currentAnchor->_sequence);
        if (sendPollMsg() < 0) {
            WARN_ISR(" sendPollMsg failed ");
            return false;
            ;
        }
    }
    return true;
}
#include <pt.h>
/**
 * FSM for handling reception
 * in receive state : wait for data or timeout
 * on timeout , check if poll-timeout expired, if so : poll one of the known
 * anchors and wait for response
 * measure distance
 * 				tag =============> 	anchor
 * 				POLL ===>
 * 							<====	RESP
 * 				FINAL ===> :
 * calc
 * distance
 *
 * blink message
 * 					<===== ( x,y,distance )
 *
 */
const char* DWM1000_Tag::stateString()
{
    return _state == RCV_ANY ? "RCV_ANY" :
           ( _state== RCV_FINAL  ? "RCV_FINAL" :
             ( _state== RCV_RESP ? "RCV_RESP" : "unknown state"));

}
void DWM1000_Tag::enableRxd()
{
    //	dwt_setautorxreenable(true);
    dwt_write32bitreg(SYS_STATUS_ID,
                      SYS_STATUS_ALL_RX_GOOD | SYS_STATUS_ALL_RX_ERR |
                      SYS_STATUS_ALL_TX | SYS_STATUS_ALL_DBLBUFF);
    dwt_setrxtimeout(60000); // 60 msec ?
    if (dwt_rxenable(0) < 0)
        WARN_ISR("WARN dwt_rxenable() failed ");
}

void DWM1000_Tag::FSM(const dwt_callback_data_t* signal)
{
    lastStatus = signal->status;
    lastEvent = signal->event;
    if (signal->event == DWT_SIG_RX_OKAY) {
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_GOOD);
        FrameType ft = readMsg(signal);
        if (ft == FT_BLINK) {
            handleBlinkMsg();
        } else if (ft == FT_RESP && _dwmMsg.getDst() == _shortAddress &&
                   _dwmMsg.getSrc() == _currentAnchor->_address) {
            createFinalMsg(_finalMsg, _respMsg);
            if (sendFinalMsg() < 0) {
                WARN_ISR("WARN sendFinalMsg failed");
            }
        } else {
            WARN_ISR("WARN unexpected frame type %d", ft);
        }
        dwt_rxenable(0);
    } else if (signal->event == DWT_SIG_RX_TIMEOUT ||
               signal->event == DWT_SIG_RX_SFDTIMEOUT ||
               signal->event == DWT_SIG_RX_PHR_ERROR ||
               signal->event == DWT_SIG_RX_ERROR ||
               signal->event == DWT_SIG_RX_SYNCLOSS) {
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
        _timeouts++;
        if (_pollTimerExpired) {
            _pollTimerExpired = false;
            if (anchorsCount() > 0) {
                if (!pollAnchor()) {
                    WARN_ISR("WARN pollAnchor failed");
                }
            } else {
                dwt_rxenable(0);
            }
        } else {
            dwt_rxenable(0);
        }
    } else if (signal->event == DWT_SIG_TX_DONE) {
        // apparently irq cannot be suppressed
        dwt_write32bitreg(SYS_STATUS_ID,
                          SYS_STATUS_ALL_TX); // Clear TX event bit
    } else {
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
        WARN_ISR("WARN unhandled event %d", signal->event);
    }
    _interruptDelay = Sys::micros() - _interruptStart;
}

FrameType DWM1000_Tag::readMsg(const dwt_callback_data_t* signal)
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
            WARN_ISR(" unknown frame type %X:%X : %s", _dwmMsg.fc[0], _dwmMsg.fc[1], stateString());
        }
        return ft;
    } else {
        WARN_ISR("WARN invalid length %d : hdr %X:%X : %s", frameLength, _dwmMsg
                 .fc[0], _dwmMsg.fc[1], stateString());
        return FT_UNKNOWN;
    }
}

/*!
 * ------------------------------------------------------------------------------------------------------------------
 * @fn get_tx_timestamp_u64()
 *
 * @brief Get the TX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for
 * both TX and RX!
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

/*!
 * ------------------------------------------------------------------------------------------------------------------
 * @fn get_rx_timestamp_u64()
 *
 * @brief Get the RX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for
 * both TX and RX!
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

/*!
 * ------------------------------------------------------------------------------------------------------------------
 * @fn final_msg_get_ts()
 *
 * @brief Read a given timestamp value from the final message. In the timestamp
 * fields of the final message, the least
 *        significant byte is at the lower address.
 *
 * @param  ts_field  pointer on the first byte of the timestamp field to read
 *        ts  timestamp value
 *
 * @return none
 */
/*static void final_msg_get_ts(const uint8* ts_field, uint32* ts) {
	int i;
	*ts = 0;
	for (i = 0; i < FINAL_MSG_TS_LEN; i++) {
		*ts += ts_field[i] << (i * 8);
	}
}*/

/*!
 * ------------------------------------------------------------------------------------------------------------------
 * @fn final_msg_set_ts()
 *
 * @brief Fill a given timestamp field in the final message with the given
 * value. In the timestamp fields of the final
 *        message, the least significant byte is at the lower address.
 *
 * @param  ts_field  pointer on the first byte of the timestamp field to fill
 *        ts  timestamp value
 *
 * @return none
 */
static void final_msg_set_ts(uint8* ts_field, uint64 ts)
{
    int i;
    for (i = 0; i < FINAL_MSG_TS_LEN; i++) {
        ts_field[i] = (uint8)ts;
        ts >>= 8;
    }
}

/*****************************************************************************************************************************************************
 * NOTES:
 *
 * 1. The sum of the values is the TX to RX antenna delay, experimentally
 *determined by a calibration process. Here we use a hard coded typical value
 *    but, in a real application, each device should have its own antenna delay
 *properly calibrated to get the best possible precision when performing
 *    range measurements.
 * 2. The messages here are similar to those used in the DecaRanging ARM
 *application (shipped with EVK1000 kit). They comply with the IEEE
 *    802.15.4 standard MAC data frame encoding and they are following the
 *ISO/IEC:24730-62:2013 standard. The messages used are:
 *    - a poll message sent by the initiator to trigger the ranging exchange.
 *    - a response message sent by the responder allowing the initiator to go on
 *with the process
 *    - a final message sent by the initiator to complete the exchange and
 *provide all information needed by the responder to compute the
 *      time-of-flight (distance) estimate.
 *    The first 10 bytes of those frame are common and are composed of the
 *following fields:
 *    - byte 0/1: frame control (0x8841 to indicate a data frame using 16-bit
 *addressing).
 *    - byte 2: sequence number, incremented for each new frame.
 *    - byte 3/4: PAN ID (0xDECA).
 *    - byte 5/6: destination address, see NOTE 3 below.
 *    - byte 7/8: source address, see NOTE 3 below.
 *    - byte 9: function code (specific values to indicate which message it is
 *in the ranging process).
 *    The remaining bytes are specific to each message as follows:
 *    Poll message:
 *    - no more data
 *    Response message:
 *    - byte 10: activity code (0x02 to tell the initiator to go on with the
 *ranging exchange).
 *    - byte 11/12: activity parameter, not used for activity code 0x02.
 *    Final message:
 *    - byte 10 -> 13: poll message transmission timestamp.
 *    - byte 14 -> 17: response message reception timestamp.
 *    - byte 18 -> 21: final message transmission timestamp.
 *    All messages end with a 2-byte checksum automatically set by DW1000.
 * 3. Source and destination addresses are hard coded constants in this example
 *to keep it simple but for a real product every device should have a
 *    unique ID. Here, 16-bit addressing is used to keep the messages as short
 *as possible but, in an actual application, this should be done only
 *    after an exchange of specific messages used to define those short
 *addresses for each device participating to the ranging exchange.
 * 4. Delays between frames have been chosen here to ensure proper
 *synchronisation of transmission and reception of the frames between the
 *initiator
 *    and the responder and to ensure a correct accuracy of the computed
 *distance. The user is referred to DecaRanging ARM Source Code Guide for more
 *    details about the timings involved in the ranging process.
 * 5. This timeout is for complete reception of a frame, i.e. timeout duration
 *must take into account the length of the expected frame. Here the value
 *    is arbitrary but chosen large enough to make sure that there is enough
 *time to receive the complete final frame sent by the responder at the
 *    110k data rate used (around 3.5 ms).
 * 6. In a real application, for optimum performance within regulatory limits,
 *it may be necessary to set TX pulse bandwidth and TX power, (using
 *    the dwt_configuretxrf API call) to per device calibrated values saved in
 *the target system or the lastEvDW1000 OTP memory.
 * 7. We use polled mode of operation here to keep the example as simple as
 *possible but all status events can be used to generate interrupts. Please
 *    refer to DW1000 User Manual for more details on "interrupts". It is also
 *to be noted that STATUS register is 5 bytes long but, as the event we
 *    use are all in the first bytes of the register, we can use the simple
 *dwt_read32bitreg() API call to access it instead of reading the whole 5
 *    bytes.
 * 8. Timestamps and delayed transmission time are both expressed in device time
 *units so we just have to add the desired response delay to poll RX
 *    timestamp to get response transmission time. The delayed transmission time
 *resolution is 512 device time units which means that the lower 9 bits
 *    of the obtained value must be zeroed. This also allows to encode the
 *40-bit value in a 32-bit words by shifting the all-zero lower 8 bits.
 * 9. dwt_writetxdata() takes the full size of the message as a parameter but
 *only copies (size - 2) bytes as the check-sum at the end of the frame is
 *    automatically appended by the DW1000. This means that our variable could
 *be two bytes shorter without losing any data (but the sizeof would not
 *    work anymore then as we would still have to indicate the full length of
 *the frame to dwt_writetxdata()). It is also to be noted that, when using
 *    delayed send, the time set for transmission must be far enough in the
 *future so that the DW1000 IC has the time to process and start the
 *    transmission of the frame at the wanted time. If the transmission command
 *is issued too late compared to when the frame is supposed to be sent,
 *    this is indicated by an error code returned by dwt_starttx() API call.
 *Here it is not tested, as the values of the delays between frames have
 *    been carefully defined to avoid this situation.
 * 10. The high order byte of each 40-bit time-stamps is discarded here. This is
 *acceptable as, on each device, those time-stamps are not separated by
 *    more than 2**32 device time units (which is around 67 ms) which means that
 *the calculation of the round-trip delays can be handled by a 32-bit
 *    subtraction.
 * 11. The user is referred to DecaRanging ARM application (distributed with
 *EVK1000 product) for additional practical example of usage, and to the
 *    DW1000 API Guide for more details on the DW1000 driver functions.
 ************************************************************************************************************************************************/
