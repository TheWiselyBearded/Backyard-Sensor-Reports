/**
 * Copyright (c) 2018 Alireza Bahremand
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include <avr/io.h>
#include <util/delay.h>
#include <avr/power.h>
#include <stdio.h>
#include "pinDefines.h"
#include "USART.h"
#include "i2c.h"
#include "math.h"

/**
 * Luminosity Sensor (TSL2561): 0x72 (w) 0x73 (r)
 * ADDR pin can be used if i2c address conflict, change address:
 * Connect to ground sets address to 0x29.
 * Connect to 3.3V (vcc) sets address to 0x49
 * Leave it (unconnected) address 0x39 (used).
 * 7 bits, shift it 1 left, then || address with 0/1 for w/r.
 *
 *
 * Temp & Humidity Sensor:
 * The HTU21D-F has a default I2C address of 0x40 and cannot be changed!
 *
 */

void printReadValue(uint16_t sensorVal, char sensorName[]);
void inline checkIntruder();
void formatADXL345();
void formatTSL2561();
void formatHTU21D();
void readADXL345_sensor(uint16_t *x_axis, uint16_t *y_axis, uint16_t *z_axis);

/**
 * Method configures:
 *     - The mux via AVCC ref volt.
 *     - Sets clock prescaler.
 *     - Enables ADC to receive input after config.
 */
static inline void initADC(void) {
    ADMUX |= (1 << REFS0);
    ADCSRA |= (1 << ADPS1) | (1 << ADPS0);
    ADCSRA |= (1 << ADEN);
}

/**
 * Given a 4 bit channel selection from ADMUX, we can switch between
 * channels. Apply bitmask to change channel and begin conversion, wait
 * for result, then return result.
 */
uint16_t readADC(uint8_t channel) {
    ADMUX = (0xf0 & ADMUX) | channel;   // Clear 4 lower mux bits, set chnnl.
    ADCSRA |= (1 << ADSC);                      // New ADC conversion.
    loop_until_bit_is_clear(ADCSRA, ADSC);      // Wait conversion.
    return (ADC);
}


/**
 * Word protocol read.
 * Address the Ch0 lower data register and configure for Read Word
 *
 * Command = 0xAC //Set Command bit and Word bit
 *
 * Reads two bytes from sequential registers 0x0C and 0x0D
 * Results are returned in DataLow and DataHigh variables
 *
 * ReadWord (Address, Command, DataLow,
 *
 * Higher value means less illuminance, lower means more illuminance.
 */
void readTSL2561_sensor(uint16_t *channel_zero, uint16_t *channel_one) {
    // We read words, therefore data low & data high per respective channel.
    i2cStart();
    i2cSend(TSL2561_I2C_W);     // Addr + r/w bit
    // command bit and word bit, ch0 lower data reg, 0x8C
    i2cSend((TSL2561_CMD_BIT | TSL2561_REG_CH0_LOW));   // 0x8C
    i2cStop();
    
    i2cStart();
    i2cSend(TSL2561_I2C_R);
    // 16 bits, lets keep communication line by ack.
    (*channel_zero) = i2cReadAck();
    // Retrieve remaining bits, end multi-byte read.
	(*channel_zero) |= (i2cReadNoAck() << 8);
    i2cStop();

    i2cStart();
    i2cSend(TSL2561_I2C_W);     // Addr + r/w bit
    // command bit and word bit, ch0 lower data reg, 0x8C
    i2cSend((TSL2561_CMD_BIT | TSL2561_REG_CH1_LOW));   // 0x8C
    i2cStop();
    
    i2cStart();
    i2cSend(TSL2561_I2C_R);
    // 16 bits, lets keep communication line by ack.
	(*channel_one) = i2cReadAck();
    // Retrieve remaining bits, end multi-byte read.
	(*channel_one) |= (i2cReadNoAck() << 8);
    i2cStop();
}

/**
 * The command register specifies the address of the target register
 * for subsequent read and write operations.
 * The Send Byte protocol is used to configure the COMMAND register.
 * The command register contains eight bits as described in Table 3.
 * The command register defaults to 00h at power on.
 *
 * No Hold Master mode allows for processing other I²C communication
 * tasks on a bus while the HTU21D(F) sensor is measuring.
 */
float readHTU21D_humidity_sensor() {
    i2cStart();
    i2cSend(HTU21D_ADDRESS_W);     // Addr + r/w bit
    i2cWaitForComplete();
    i2cSend(HTU21D_HUMID_CMD);      // E5
    i2cWaitForComplete();
    
    i2cStart();
    i2cSend(HTU21D_ADDRESS_R);
    _delay_ms(1000);      // 1 second wait.
    i2cWaitForComplete();
    unsigned char most_sig_bit = i2cReadAck();
    i2cWaitForComplete();
    unsigned char least_sig_bit = i2cReadNoAck();
    i2cStop();
    // apply formula.
    unsigned int return_humid = most_sig_bit;
    return_humid = return_humid << 8;
    return_humid = return_humid | (least_sig_bit & 0xFC);
    float humidity = -6 + 125*(float) return_humid/65536;
    return humidity;
}

// TODO: See if we can consecutively read both temp and humidity
// back to back instead of divying it into 2 methods.
//uint16_t readHTU21D_temp_sensor() {
uint16_t readHTU21D_temp_sensor() {
    i2cStart();
    i2cSend(HTU21D_ADDRESS_W);     // Addr + r/w bit
    i2cWaitForComplete();
    i2cSend(HTU21D_TEMP_CMD); // pass command
    i2cWaitForComplete();
    
    i2cStart();
    i2cSend(HTU21D_ADDRESS_R);
    _delay_ms(1000);      // 1 second wait.
//    if (i2cReadAck() == 0x40) { printString("Matching"); }
    i2cWaitForComplete();
    unsigned char most_sig_bit = i2cReadAck(); // most significant bits
    i2cWaitForComplete();
    unsigned char least_sig_bit = i2cReadNoAck();	// least significant bits.
    i2cStop();
    // Apply formula from datasheet.
    unsigned int temp = most_sig_bit;
    temp = temp << 8;
    temp = temp | (least_sig_bit & 0xFC);
    uint16_t tempC = -46.85 + 175.72*((uint16_t)temp/65536.0);
    uint16_t tempF = tempC*9.0/5.0 + 32.0; //  Convert to Fahrenheit
    return tempF;
}

int main(void) {
    // Full Result is 16 bits
    uint16_t x_acc = 0;
    uint16_t y_acc = 0;
    uint16_t z_acc = 0;
    uint16_t lum_reading = 0;
	uint16_t lum_reading_next = 0;
	// 16 bit for adc so we can fit the 10 bits
	uint16_t soil_sensor_a = 0;
	uint16_t soil_sensor_b = 0;
    uint16_t temp;				// 32 bits, could switch to 16 use less.
    float humidity;
 
    clock_prescale_set(clock_div_1);            /* 8MHz */
    initUSART();
    initI2C();
    initADC();
    formatADXL345();
    formatTSL2561();
    
    DDRB = 0x00;            // Port B input
    PORTB = 0x00;           // disable pull ups
    
  while (1) {
      // read values
      readADXL345_sensor(&x_acc, &y_acc, &z_acc);
	  readTSL2561_sensor(&lum_reading, &lum_reading_next);
      temp = readHTU21D_temp_sensor();
      humidity = readHTU21D_humidity_sensor();
      soil_sensor_a = readADC(SOIL_SENSOR_ANLG_1);
      soil_sensor_b = readADC(SOIL_SENSOR_ANLG_2);
      
      // print values.
      printReadValue(x_acc, "X:\t");
      printReadValue(y_acc, "Y:\t");
      printReadValue(z_acc, "Z:\t");
      printReadValue(soil_sensor_a, "SOIL SENSOR #1:\t");
      printReadValue(soil_sensor_b, "SOIL SENSOR #2:\t");
      // Laser Detector
      checkIntruder();
      printReadValue(lum_reading, "LUM CH0:\t");
	  printReadValue(lum_reading_next, "LUM CH1:\t");
      
      printReadValue(temp, "TEMP:\t");
      printReadValue(humidity, "HUMIDITY:\t");
      printString("\n\r");          // end iteration
      
      _delay_ms(3000);      // 3 second wait.
  }
    return 0;
}


void formatTSL2561() {
	i2cStart();
	i2cSend(TSL2561_I2C_W);     // Addr + r/w bit
	i2cSend((TSL2561_CMD_BIT | TSL2561_REG_CTRL));  // enable
	i2cSend(TSL2561_CTRL_PWR_ON);                   // power
	i2cStop();
	i2cStart();
	i2cSend(TSL2561_I2C_W);     // Addr + r/w bit
								// integration time
	i2cSend((TSL2561_CMD_BIT | TSL2561_REG_TIMING));
	// gain
	i2cSend((TSL2561_INTEGRATION_TIME_402MS | TSL2561_GAIN_0X));
	i2cStop();
}

void formatHTU21D() {
	uint8_t resolutionVal = 0;

	i2cStart();
	i2cSend(HTU21D_ADDRESS_R);     // Addr + r/w bit
	i2cSend(HTU21D_READ_USER_REG);  // cmd read
	resolutionVal = i2cReadAck();   // retrieve res val
	resolutionVal &= 0x7E;  // clear current res val
	resolutionVal |= HTU21D_RES_RH8_TEMP12;
	i2cStop();

	i2cStart();
	i2cSend(TSL2561_I2C_W);     // Addr + r/w bit
								// integration time
	i2cSend(HTU21D_WRITE_USER_REG);  // cmd read
	i2cSend(resolutionVal);
	i2cStop();
}


void printReadValue(uint16_t sensorVal, char sensorName[]) {
    printString(sensorName);
    printByte(sensorVal);
    printString("\n");
}

void inline checkIntruder() {
    printString("INTRUDER:\t");
    if (!(bit_is_clear(PINB, LASER_DETECTOR))) {
        printString("false");
    } else {
        printString("true");
    }
    printString("\n\r");
}


/**
 * i2c single byte write starts:
 * master_addr + register_addr + data + stop (ack inbetween)
 */
void formatADXL345() {
    i2cStart();
    i2cSend(ADXL345_ADDRESS_W);             // Write
    i2cSend(ADXL345_DATA_FORMAT);           // DATA format register
    i2cSend(1 << 3);                        // Full Result enabled
    i2cStop();
    
    i2cStart();
    i2cSend(ADXL345_ADDRESS_W);
    i2cSend(ADXL345_POWER_CTL);             // Enable measurement, wake it up
    i2cSend(1 << 3);
    i2cStop();
}

/**
 * Multiple byte read is:
 * slave_addr_w + register_addr + slave_addr_r +
 * readByte + ack + readByte + ack + read... + nack
 */
void readADXL345_sensor(uint16_t *x_axis, uint16_t *y_axis, uint16_t *z_axis) {
    i2cStart();
    i2cSend(ADXL345_ADDRESS_W);     // Master
    i2cSend(ADXL345_X0);      // Write to lower bits addr.
    i2cStop();              // Read multiple-byte read values from register.
    i2cStart();
    i2cSend(ADXL345_ADDRESS_R);     // Setup send addr with read.
    (*x_axis) = i2cReadAck();       // 16 bits, keep communication line by ack.
    (*x_axis) |= (i2cReadAck() << 8);  // End communication after byte.
    (*y_axis) = i2cReadAck();       // 16 bits, keep communication line by ack.
    (*y_axis) |= (i2cReadAck() << 8);  // End communication after byte.
    (*z_axis) = i2cReadAck();       // 16 bits, keep communication line by ack.
    (*z_axis) |= (i2cReadNoAck() << 8);  // End communication after byte.
    i2cStop();
}
