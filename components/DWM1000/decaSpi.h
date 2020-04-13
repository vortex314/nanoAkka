
#ifndef DECA_SPI_H
#define DECA_SPI_H
void spi_set_global(Spi* spi) ;
extern "C"  {
    void spi_set_rate_low();
    void spi_set_rate_high();
}
#endif
