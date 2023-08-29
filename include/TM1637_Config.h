/* mbed TM1637 Library, for TM1637 LEDcontroller
 * Copyright (c) 2016, v01: WH, Initial version
 *               2017, v02: WH, Added RobotDyn 6 Digit module,
 *                          Added Eyewink 6 Digit + 6 Keys module, 
 *                          Constructor adapted to 2 pins: dio, clk 
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef TM1637_CONFIG_H
#define TM1637_CONFIG_H

// Select one of the testboards for TM1637 LED controller
#define TM1637_TEST   1
#define CATALEX_TEST  0 
#define ROBOTDYN_TEST 0 
#define EYEWINK_TEST  0 

// Select when you wish to keep the DEPRECATED Constructor with 3 Pins for mosi, miso and sclk
// The new component Constructor uses only 2 pins: dio and clk
#define SPI           0 

// Select the display mode: only digits and hex or ASCII
#define SHOW_ASCII    0 

#endif