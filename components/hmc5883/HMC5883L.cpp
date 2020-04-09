/*
HMC5883L.cpp - Class file for the HMC5883L Triple Axis Digital Compass Arduino
Library.

Version: 1.1.0
(c) 2014 Korneliusz Jarzebski
www.jarzebski.pl

This program is free software: you can redistribute it and/or modify
it under the terms of the version 3 GNU General Public License as
published by the Free Software Foundation.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "HMC5883L.h"
#include <Log.h>

HMC5883L::HMC5883L(I2C& i2c) : _i2c(i2c) {}
HMC5883L::HMC5883L(Connector& connector) : _i2c(connector.getI2C()) {}

bool HMC5883L::init()
{
    _i2c.init();
    _i2c.setSlaveAddress(HMC5883L_ADDRESS);

    if ((fastRegister8(HMC5883L_REG_IDENT_A) != 0x48) ||
        (fastRegister8(HMC5883L_REG_IDENT_B) != 0x34) ||
        (fastRegister8(HMC5883L_REG_IDENT_C) != 0x33)) {
        return false;
    }


    setRange(HMC5883L_RANGE_8_1GA);

//    setRange(HMC5883L_RANGE_1_3GA);
    setMeasurementMode(HMC5883L_CONTINOUS);
    setDataRate(HMC5883L_DATARATE_15HZ);
    setSamples(HMC5883L_SAMPLES_1);

    mgPerDigit = 0.92f;

    return true;
}

void HMC5883L::printReg()
{
    uint8_t data[14];
    //  _i2c.write(HMC5883L_REG_CONFIG_A);
    /*	for (int i = 0; i < 13; i++) data[i] = fastRegister8(i);
    	INFO(
    	    " HMC5883L regs :  0x%x 0x%x 0x%x  0x%x 0x%x 0x%x 0x%x 0x%x  0x%x 0x%x "
    	    "0x%x 0x%x 0x%x",
    	    data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7],
    	    data[8], data[9], data[10], data[11], data[12], data[13]);*/
    _i2c.setSlaveAddress(HMC5883L_ADDRESS);
    _i2c.write(0);
    _i2c.read(data, 13);
    INFO(
        " HMC5883L regs :  0x%x 0x%x 0x%x  0x%x 0x%x 0x%x 0x%x 0x%x  0x%x 0x%x "
        "0x%x 0x%x 0x%x",
        data[0], data[1], data[2], data[3], data[4], data[5], data[6], data[7],
        data[8], data[9], data[10], data[11], data[12], data[13]);
}

Vector<int16_t> HMC5883L::readRaw(void)
{
    uint8_t buffer[6];
    Vector<int16_t> v;
    _i2c.setSlaveAddress(HMC5883L_ADDRESS);
    if ( _i2c.write(HMC5883L_REG_OUT_X_M) ) ERROR("I2C write failed");
    _i2c.read(buffer, 6);
    v.x = (buffer[0] << 8) + (buffer[1]);
    v.z = (buffer[2] << 8) + (buffer[3]);
    v.y = (buffer[4] << 8) + (buffer[5]);
    return v;
}

Vector<float> HMC5883L::readNormalize(void)
{
    Vector<float> v;
    v.x =
        ((float)readRegister16(HMC5883L_REG_OUT_X_M) - xOffset) * mgPerDigit;
    v.y =
        ((float)readRegister16(HMC5883L_REG_OUT_Y_M) - yOffset) * mgPerDigit;
    v.z = (float)readRegister16(HMC5883L_REG_OUT_Z_M) * mgPerDigit;

    return v;
}

void HMC5883L::setOffset(int xo, int yo)
{
    xOffset = xo;
    yOffset = yo;
}

void HMC5883L::setRange(hmc5883l_range_t range)
{
    switch (range) {
    case HMC5883L_RANGE_0_88GA:
        mgPerDigit = 0.073f;
        break;

    case HMC5883L_RANGE_1_3GA:
        mgPerDigit = 0.92f;
        break;

    case HMC5883L_RANGE_1_9GA:
        mgPerDigit = 1.22f;
        break;

    case HMC5883L_RANGE_2_5GA:
        mgPerDigit = 1.52f;
        break;

    case HMC5883L_RANGE_4GA:
        mgPerDigit = 2.27f;
        break;

    case HMC5883L_RANGE_4_7GA:
        mgPerDigit = 2.56f;
        break;

    case HMC5883L_RANGE_5_6GA:
        mgPerDigit = 3.03f;
        break;

    case HMC5883L_RANGE_8_1GA:
        mgPerDigit = 4.35f;
        break;

    default:
        break;
    }

    writeRegister8(HMC5883L_REG_CONFIG_B, range << 5);
}

hmc5883l_range_t HMC5883L::getRange(void)
{
    return (hmc5883l_range_t)((readRegister8(HMC5883L_REG_CONFIG_B) >> 5));
}

void HMC5883L::setMeasurementMode(hmc5883l_mode_t mode)
{
    uint8_t value;

    value = readRegister8(HMC5883L_REG_MODE);
    value &= 0b11111100;
    value |= mode;

    writeRegister8(HMC5883L_REG_MODE, value);
}

hmc5883l_mode_t HMC5883L::getMeasurementMode(void)
{
    uint8_t value;

    value = readRegister8(HMC5883L_REG_MODE);
    value &= 0b00000011;

    return (hmc5883l_mode_t)value;
}

void HMC5883L::setDataRate(hmc5883l_dataRate_t dataRate)
{
    uint8_t value;

    value = readRegister8(HMC5883L_REG_CONFIG_A);
    value &= 0b11100011;
    value |= (dataRate << 2);

    writeRegister8(HMC5883L_REG_CONFIG_A, value);
}

hmc5883l_dataRate_t HMC5883L::getDataRate(void)
{
    uint8_t value;

    value = readRegister8(HMC5883L_REG_CONFIG_A);
    value &= 0b00011100;
    value >>= 2;

    return (hmc5883l_dataRate_t)value;
}

void HMC5883L::setSamples(hmc5883l_samples_t samples)
{
    uint8_t value;

    value = readRegister8(HMC5883L_REG_CONFIG_A);
    value &= 0b10011111;
    value |= (samples << 5);

    writeRegister8(HMC5883L_REG_CONFIG_A, value);
}

hmc5883l_samples_t HMC5883L::getSamples(void)
{
    uint8_t value;
    value = readRegister8(HMC5883L_REG_CONFIG_A);
    value &= 0b01100000;
    value >>= 5;

    return (hmc5883l_samples_t)value;
}

// Write byte to register
void HMC5883L::writeRegister8(uint8_t reg, uint8_t value)
{
    uint8_t arr[2] = {reg, value};
    _i2c.setSlaveAddress(HMC5883L_ADDRESS);
    _i2c.write(arr, 2);
}

// Read byte to register
uint8_t HMC5883L::fastRegister8(uint8_t reg)
{
    uint8_t value;
    _i2c.setSlaveAddress(HMC5883L_ADDRESS);
    _i2c.write(reg);
    _i2c.read(&value, 1);
    return value;
}

// Read byte from register
uint8_t HMC5883L::readRegister8(uint8_t reg)
{
    uint8_t value;
    _i2c.setSlaveAddress(HMC5883L_ADDRESS);
    _i2c.write(reg);
    _i2c.read(&value, 1);
    //  INFO(" reg %d : 0x%x", reg, value);
    return value;
}

// Read word from register
int16_t HMC5883L::readRegister16(uint8_t reg)
{
    int16_t value;
    _i2c.setSlaveAddress(HMC5883L_ADDRESS);
    _i2c.write(reg);
    uint8_t vha, vla;
//#define READ1 1
#ifdef READ1

    _i2c.read(&vha, 1);
    _i2c.read(&vla, 1);

    value = vha << 8 | vla;
#else
    vha = fastRegister8(reg);
    vla = fastRegister8(reg + 1);
    value = vha << 8 | vla;
#endif
    return value;
}
