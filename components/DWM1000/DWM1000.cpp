#include <stdint.h>
#include <DWM1000.h>
#include <Hardware.h>

#include <decaSpi.h>

/* Default antenna delay values for 64 MHz PRF. See NOTE 1 below. */
#define TX_ANT_DLY 16436
#define RX_ANT_DLY 16436

/* Default communication configuration. We use here EVK1000's default mode (mode 3). */

//_________________________________________________ SETUP  DWM1000
//
#include <stdint.h>
#include <DWM1000.h>
#include <Config.h>
#include <decaSpi.h>

/* Default antenna delay values for 64 MHz PRF. See NOTE 1 below. */
#define TX_ANT_DLY 16436
#define RX_ANT_DLY 16436

/* Default communication configuration. We use here EVK1000's default mode (mode 3). */
/*
static dwt_config_t dwt2_config = {  //
		2, // Channel number.
				DWT_PRF_64M, // Pulse repetition frequency.
				DWT_PLEN_1024, // Preamble length.
				DWT_PAC32, // Preamble acquisition chunk size Used in RX only.
				9, // TX preamble code. Used in TX only.
				9, // RX preamble code. Used in RX only.
				1, // Use non-standard SFD (Boolean)
				DWT_BR_850K, // Data rate.
				DWT_PHRMODE_STD, // PHY header mode.
				(1025 + 64 - 32) // SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only.
};
*/
const static struct {
    uint16_t preambleLength;
    uint8_t pacSize;
} preamblePacSize[] = { { 64, 8 }, { 128, 8 }, { 256, 16 }, { 512, 16 }, {
        1024,
        32
    }, { 1536, 64 }, { 2048, 64 }, { 4096, 64 }
};

DWM1000::DWM1000(Spi& spi, DigitalIn& irq, DigitalOut& reset,
                 uint16_t shortAddress, uint8_t* longAddress)
    : _spi(spi), _irq(irq), _reset(reset)
{
    _shortAddress = shortAddress;
    memcpy(_longAddress, longAddress, 6);
    _sequence = 0;
    _channel = 2;
    _prf = DWT_PRF_64M; // was DWT_PRF_64M
    _preambleLength = DWT_PLEN_1024; // was DWT_PLEN_1024
    _dataRate = DWT_BR_850K; // was DWT_BR_850K
    _pacSize = DWT_PAC32; // was 32

    _config = {  //129 + 64 - 8
        _channel,// Channel number.
        _prf,// Pulse repetition frequency.
        _preambleLength, /* Preamble length. */
        _pacSize, /* Preamble acquisition chunk size. Used in RX only. */
        9, /* TX preamble code. Used in TX only. */
        9, /* RX preamble code. Used in RX only. */
        1, /* Use non-standard SFD (Boolean) */
        _dataRate, /* Data rate. */
        DWT_PHRMODE_EXT, /* PHY header mode. */
        (1025+64-32) /* SFD timeout (preamble length + 1 + SFD length - PAC size). Used in RX only. */ // changed due to above was (1025+64-32)
    };
    _distance = 0.0;
    _x = 0;
    _y = 0;
    _count = 0;
}

DWM1000::~DWM1000()
{

}

void DWM1000::resetChip()
{
    INFO("Reset DWM1000");

    _reset.write(0);
    Sys::delay(10);
    _reset.write(1);
}

Register reg_sys_status("SYS_STATUS", "ICRBP HSRBP AFFREJ TXBERR HPDWARN RXSFDTO CLKPLL_LL RFPLL_LL "
                        "SLP2INIT GPIOIRQ RXPTO RXOVRR F LDEERR RXRFTO RXRFSL RXFCE RXFCG "
                        "RXDFR RXPHE RXPHD LDEDONE RXSFDD RXPRD TXFRS TXPHS TXPRS TXFRB AAT "
                        "ESYNCR CPLOCK IRQSD");
Register ref_any("ANY", "31 30 29 28 27 26 25 24 23 22 21 20 19 18 17 16 15 14 13 12 11 10 9 8 7 6 5 4 3 2 1 0");

Register reg_sys_state("SYS_STATE", "31 30 29 28 27 26 25 24 23 22 21 20 19 18 17 16 15 14 13 12 11 10 9 8 7 6 5 4 3 2 1 0");
Register reg_sys_state2("SYS_STATE", "PMSC_STATE:23:16:INIT,IDLE,TX_WAIT,RX_WAIT,TX,RX "
                        "RX_STATE:15:8:IDLE,START_ANALOG,2,3,RX_READY,PREAMBLE_FIND,PREAMBLE_TO,SFD_FOUND,CONFIGURE_PHR_RX,PHR_RX_START,DATA_STATE_READY,DATA_RX_SEQ,CONFIG_DATA,PHR_NOT_OK,LAST_SYMBOL,WAIT_RSD_DONE,RSD_OK,RSD_NOT_OK,RECONFIG_110,WAIT110_PHR "
                        "TX_STATE:7:0:IDLE,PREAMBLE,SFD,PHR,SDE,DATA,RSP_DATA");
Register reg_sys_state3("SYS_STATE", "+ + + + + + + filler + + + + + + + PMSC_STATE + + + + + + + RX_STATE + + + + + + + TX_STATE");

Register reg_sys_mask("SYS_MASK", "- - MAFFREJ MTXBERR MHPDWARN MRXSFDTO MPLLHILO MRFPLLLL MSLP2INIT MGPIOIRQ MRXPTO "
                      "MRXOVRR - MLDEERR MRXRFTO MRXRFSL MRXFCE MRXFCG MRXDFR MRXPHE MRXPHD MLDEDON "
                      "MRXSFDD MRXPRD MTXFRS MTXPHS MTXPRS MTXFRB MAAT MESYNCR MCPLOCK -");

void DWM1000::status()
{
    uint32_t sys_mask, sys_status, sys_state;

    sys_mask = dwt_read32bitreg(SYS_MASK_ID);
    sys_status = dwt_read32bitreg(SYS_STATUS_ID);
    sys_state = dwt_read32bitreg(SYS_STATE_ID);
    reg_sys_status.value(sys_status);
    reg_sys_status.show();
    reg_sys_mask.value(sys_mask);
    reg_sys_mask.show();
    reg_sys_state3.value(sys_state);
    reg_sys_state3.show();
    INFO("SYS_MASK : %X SYS_STATUS : %X SYS_STATE: %X IRQ : %d", sys_mask, sys_status, sys_state, _irq
         .read());
}

//_________________________________________________ SETUP  DWM1000
//

static const uint8_t pulseGeneratorDelay[] = { 0xFF, 0xC9, 0xC2, 0xC5, 0x95,
                                               0xC0, 0xFF, 0x93
                                             };
static const struct {
    uint8_t channel;
    uint32_t txPower16MHZ;
    uint32_t txPower64MHZ;
} txPowerSmart[] = { { 0, 0x00, 0x00 }, //
    { 1, 0x15355575, 0x07274767 }, //
    { 2, 0x15355575, 0x07274767 }, //
    { 3, 0x0F2F4F6F, 0x2B4B6B8B }, //
    { 4, 0x1F1F3F5F, 0x3A5A7A9A }, //
    { 5, 0x0E082848, 0x25456585 }, //
    { 0, 0x00, 0x00 }, //
    { 7, 0x32527292, 0x5171B1D1 } //
};

extern void dwt_isr();
/*
 #include <DWM1000/DWM1000_Anchor.h>
 void dwt_isr_void(void* obj)
 {
 vPortEnterCritical();
 DWM1000_Anchor* ptr=(DWM1000_Anchor*)obj;
 ptr->_interruptStart = Sys::micros();
 dwt_isr();
 portEXIT_CRITICAL();
 }*/

void DWM1000::init()
{
    resetChip();

    spi_set_rate_low();
    dwt_setpanid(0xDECA);
    dwt_setaddress16(_shortAddress);
//    dwt_enableframefilter(DWT_FF_DATA_EN);
//    dwt_setpanid(0xDECA);
//    dwt_setaddress16(((uint16_t)'E'<<8)+'V');
    std::string strAddress;

    dwt_seteui(_longAddress);
    dwt_geteui(_longAddress);

    INFO(" EUID = %X:%X:%X:%X:%X:%X", _longAddress[0], _longAddress[1], _longAddress[2], _longAddress[3], _longAddress[4], _longAddress[5]);

//  dwt_softreset();
//    deca_sleep(100);
    for (int i = 0; i < 20; i++) {
        if (dwt_initialise(DWT_LOADUCODE)) {
            INFO("dwt_initialise failed.");
            Sys::delay(10);
        } else {
            INFO("dwt_initialise done.");
            break;
        }
    }

    dwt_txconfig_t txConfig;
    txConfig.PGdly = pulseGeneratorDelay[_config.chan];
    if (_config.dataRate == DWT_PRF_64M)
        txConfig.power = txPowerSmart[_config.chan].txPower64MHZ;
    else
        txConfig.power = txPowerSmart[_config.chan].txPower16MHZ;
    txConfig = {0xC0,0x1F1F1F1F};
    dwt_configuretxrf(&txConfig);
    dwt_setsmarttxpower(false); // was true
    dwt_setrxmode(DWT_RX_NORMAL, 0, 0);

    if (dwt_configure(&_config)) {
        INFO(" dwt_configure failed ");
    } else
        INFO(" dwt_configure done.");

    spi_set_rate_high();

    uint32_t device_id = dwt_readdevid();
    uint32_t part_id = dwt_getpartid();
    uint32_t lot_id = dwt_getlotid();

    INFO(" device id : %X , part id : %X , lot_id : %X", device_id, part_id, lot_id);

    dwt_setrxantennadelay(RX_ANT_DLY); /* Apply default antenna delay value. See NOTE 1 below. */
    dwt_settxantennadelay(TX_ANT_DLY);
}

void DWM1000::setup()
{

    config.setNameSpace("dwm1000");
//_________________________________________________INIT SPI, IRQ,RESET  DWM1000
    spi_set_global(&_spi); // to support deca spi routines, to handle also irq's
    _spi.setClock(Spi::SPI_CLOCK_500K);
    _spi.setHwSelect(true);
    _spi.setMode(Spi::SPI_MODE_PHASE0_POL0);
    _spi.setLsbFirst(false);
    _spi.init();

//    _irq.onChange(DigitalIn::DIN_RAISE,dwt_isr_void,this); // has been set by anchor
    _irq.init();
    _reset.init();

    init();

}

FrameType DWM1000::getFrameType(DwmMsg& msg)
{
    if (msg.fc[0] == FC_1_BLINK) return FT_BLINK;
    if (msg.fc[0] == FC_1_SHORT && msg.fc[1] == FC_2_SHORT) {
        if (msg.function == FUNC_POLL_MSG) return FT_POLL;
        if (msg.function == FUNC_RESP_MSG) return FT_RESP;
        if (msg.function == FUNC_FINAL_MSG) return FT_FINAL;
    }
    return FT_UNKNOWN;
}

//=======================================================================

void DWM1000::createBlinkFrame(BlinkMsg& blink)
{
    blink.fc[0] = FC_1_BLINK;
    blink.sourceShort[1] = (_shortAddress >> 8) & 0xFF;
    blink.sourceShort[0] = (_shortAddress & 0xFF);
    blink.sequence = _sequence++;
    for (int i = 0; i < 8; i++)
        blink.sourceLong[i] = _longAddress[7 - i];
    little_endian(blink.x, _x);
    little_endian(blink.y, _y);
    little_endian(blink.distance, _distance);
}

//=======================================================================

void DWM1000::createPollMsg(PollMsg& pollMsg, uint16_t address,
                            uint8_t sequence)
{
    pollMsg.fc[0] = FC_1_SHORT;
    pollMsg.fc[1] = FC_2_SHORT;
    pollMsg.function = FUNC_POLL_MSG;
    pollMsg.sequence = sequence;
    pollMsg.panId[0] = 0xCA;
    pollMsg.panId[1] = 0xDE;
    pollMsg.dst[0] = address & 0xFF;
    pollMsg.dst[1] = address >> 8;
    pollMsg.src[0] = _shortAddress & 0xFF;
    pollMsg.src[1] = _shortAddress >> 8;
}

//=======================================================================

void DWM1000::createPollMsg(PollMsg& pollMsg, BlinkMsg& blinkMsg)
{
    pollMsg.fc[0] = FC_1_SHORT;
    pollMsg.fc[1] = FC_2_SHORT;
    pollMsg.function = FUNC_POLL_MSG;
    pollMsg.sequence = blinkMsg.sequence;
    pollMsg.panId[0] = 0xCA;
    pollMsg.panId[1] = 0xDE;
    memcpy(pollMsg.dst, blinkMsg.sourceShort, 2);
    pollMsg.src[0] = _shortAddress & 0xFF;
    pollMsg.src[1] = _shortAddress >> 8;
}

//=======================================================================

void DWM1000::createRespMsg(RespMsg& respMsg, PollMsg& pollMsg)
{
    respMsg.fc[0] = FC_1_SHORT;
    respMsg.fc[1] = FC_2_SHORT;
    respMsg.function = FUNC_RESP_MSG;
    respMsg.activity = 2;
    respMsg.sequence = pollMsg.sequence;
    respMsg.panId[0] = 0xCA;
    respMsg.panId[1] = 0xDE;
    memcpy(respMsg.dst, pollMsg.src, 2);
    memcpy(respMsg.src, pollMsg.dst, 2);
}

void DWM1000::createFinalMsg(FinalMsg& finalMsg, RespMsg& respMsg)
{
    finalMsg.fc[0] = FC_1_SHORT;
    finalMsg.fc[1] = FC_2_SHORT;
    finalMsg.function = FUNC_FINAL_MSG;
    finalMsg.sequence = respMsg.sequence;
    finalMsg.panId[0] = 0xCA;
    finalMsg.panId[1] = 0xDE;
    memcpy(finalMsg.dst, respMsg.src, 2);
    memcpy(finalMsg.src, respMsg.dst, 2);
}

void DWM1000::setShortAddress(uint16_t address)
{
    INFO(" DWM1000 short address : %X ", address);
    _shortAddress = address;
}

void DWM1000::setLongAddress(uint8_t address[])
{
    memcpy(_longAddress, address, 8);
}
