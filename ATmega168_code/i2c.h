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

// Functions for i2c communication
#include <avr/io.h>
#include "pinDefines.h"

void initI2C(void);
    /* Sets pullups and initializes bus speed to 100kHz (at FCPU=8MHz) */

void i2cWaitForComplete(void);
                       /* Waits until the hardware sets the TWINT flag */

void i2cStart(void);
                               /* Sends a start condition (sets TWSTA) */
void i2cStop(void);
                                /* Sends a stop condition (sets TWSTO) */

void i2cSend(uint8_t data);
                   /* Loads data, sends it out, waiting for completion */

uint8_t i2cReadAck(void);
              /* Read in from slave, sending ACK when done (sets TWEA) */
uint8_t i2cReadNoAck(void);
              /* Read in from slave, sending NOACK when done (no TWEA) */

 
