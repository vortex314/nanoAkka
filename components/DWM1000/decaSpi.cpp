#include <Hardware.h>
#include <Log.h>
#include <deca_device_api.h>
#ifdef ARDUINO
#include <Arduino.h>
#endif
#ifdef ESP8266_OPEN_RTOS
#endif

Spi* _gSpi;     // to support irq 


void spi_set_global(Spi* spi) {
    _gSpi = spi;
}

Bytes out(100);
Bytes in(100);

//////////////////////////////////////////////////////////////////////////////////
//
//
//
/////////////////////////////////////////////////////////////////////////////////
extern "C" int writetospi(uint16 hLen, const uint8 *hbuff, uint32 bLen,
                          const uint8 *buffer)
{

    out.clear();
    out.write((uint8_t*)hbuff,0,hLen);
    out.write((uint8_t*)buffer,0,bLen);
    _gSpi->exchange(in,out);
    return 0;
}
//////////////////////////////////////////////////////////////////////////////////
//
//
//
/////////////////////////////////////////////////////////////////////////////////

extern "C" int readfromspi(uint16 hLen, const uint8 *hbuff, uint32 bLen, uint8 *buffer)
{
    out.clear();
    out.write((uint8_t*)hbuff,0,hLen);
    out.write((uint8_t*)buffer,0,bLen);
    _gSpi->exchange(in,out);
    in.offset(hLen);
    while(in.hasData()) {
        *buffer++ = in.read();
    }
    return 0;
}
//////////////////////////////////////////////////////////////////////////////////
//
//
//
/////////////////////////////////////////////////////////////////////////////////
extern "C" void spi_set_rate_low()
{
    _gSpi->setClock(Spi::SPI_CLOCK_1M);
    _gSpi->deInit();
    _gSpi->init();
}
//////////////////////////////////////////////////////////////////////////////////
//
//
//
/////////////////////////////////////////////////////////////////////////////////
extern "C" void spi_set_rate_high()
{
   _gSpi->setClock(Spi::SPI_CLOCK_10M);
   _gSpi->deInit();
   _gSpi->init();
}

#include <FreeRTOS.h>
#include <task.h>

extern "C" decaIrqStatus_t decamutexon() {
//    noInterrupts();

return 0;
}

extern "C" void decamutexoff(decaIrqStatus_t s) {
//    interrupts();

}



