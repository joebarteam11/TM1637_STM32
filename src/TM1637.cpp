/* mbed TM1637 Library, for TM1637 LED controller
 * Copyright (c) 2016, v01: WH, Initial version
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, inclumosig without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUmosiG BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#include "mbed.h" 
#include "TM1637.h"
#include "TM1637_Config.h"
#include "Font_7Seg.h"

/** Constructor for class for driving TM1637 LED controller with Serial bus interface device. 
 *  @brief Supports 6 digits @ 8 segments. 
 *   
 *  @param  PinName mosi Serial bus MOSI pin
 *  @param  PinName miso Serial bus MISO pin 
 *  @param  PinName sclk Serial bus SCLK pin
*/

TM1637::TM1637(PinName dio, PinName clk) : _dio(dio), _clk(clk) {

  _init();
}

/** Init the Serial interface and the controller
  * @param  none
  * @return none
  */ 
 
void TM1637::_init(){
  
//TM1637 uses a Serial bus that looks like I2C, but really is not.
//It has Start and Stop conditions like I2C and an Ack pulse, but instead of slaveaddresses and a RW bit it just transmits commands and data.

//init Serial bus
  _dio.output();
//  _dio.mode(PullUp);
  wait_us(1);

  _dio=1;  
  _clk=1;  

//init controller  
  _display = TM1637_DSP_ON;
  _bright  = TM1637_BRT_DEF; 
  _writeCmd(TM1637_DSP_CTRL_CMD, _display | _bright );                                 // Display control cmd, display on/off, brightness   
  
  _writeCmd(TM1637_DATA_SET_CMD, TM1637_DATA_WR | TM1637_ADDR_INC | TM1637_MODE_NORM); // Data set cmd, normal mode, auto incr, write data  
}  


/** Clear the screen and locate to 0
 */  
void TM1637::cls() {

  _start();  

  _write(TM1637_ADDR_SET_CMD | 0x00); // Address set cmd, 0      
  for (int cnt=0; cnt<TM1637_DISPLAY_MEM; cnt++) {
    _write(0x00); // data 
  }

  _stop();  
}  

/** Set Brightness
  *
  * @param  char brightness (3 significant bits, valid range 0..7 (1/16 .. 14/14 dutycycle)  
  * @return none
  */
void TM1637::setBrightness(char brightness){

  _bright = brightness & TM1637_BRT_MSK; // mask invalid bits
  
  _writeCmd(TM1637_DSP_CTRL_CMD, _display | _bright );  // Display control cmd, display on/off, brightness  
}

/** Set the Display mode On/off
  *
  * @param bool display mode
  */
void TM1637::setDisplay(bool on) {
  
  if (on) {
    _display = TM1637_DSP_ON;
  }
  else {
    _display = TM1637_DSP_OFF;
  }
  
  _writeCmd(TM1637_DSP_CTRL_CMD, _display | _bright );  // Display control cmd, display on/off, brightness   
}

/** Write databyte to TM1637
  *  @param  int address display memory location to write byte
  *  @param  char data byte written at given address
  *  @return none
  */ 
void TM1637::writeData(char data, int address) {
  
  _start();

  _write(TM1637_ADDR_SET_CMD | (address & TM1637_ADDR_MSK)); // Set Address cmd     
  _write(data); // data 

  _stop();  
}

/** Write Display datablock to TM1637
  *  @param  DisplayData_t data Array of TM1637_DISPLAY_MEM (=16) bytes for displaydata
  *  @param  length number bytes to write (valid range 0..(TM1637_MAX_NR_GRIDS * TM1637_BYTES_PER_GRID) (=16), when starting at address 0)  
  *  @param  int address display memory location to write bytes (default = 0) 
  *  @return none
  */  
void TM1637::writeData(DisplayData_t data, int length, int address) {

  _start();

// sanity check
  address &= TM1637_ADDR_MSK;
  if (length < 0) {length = 0;}
  if ((length + address) > TM1637_DISPLAY_MEM) {length = (TM1637_DISPLAY_MEM - address);}
    
//  _write(TM1637_ADDR_SET_CMD | 0x00); // Set Address at 0
  _write(TM1637_ADDR_SET_CMD | address); // Set Address
  
  for (int idx=0; idx<length; idx++) {    
//    _write(data[idx]); // data 
    _write(data[address + idx]); // data 
  }
  
  _stop();  
}

/** Read keydata block from TM1637
  *  @param  *keydata Ptr to bytes for keydata
  *  @return bool keypress True when at least one key was pressed
  *
  */   

bool TM1637::getKeys(KeyData_t *keydata) {

  _start();

  // Enable Key Read mode
  _write(TM1637_DATA_SET_CMD | TM1637_KEY_RD | TM1637_ADDR_INC | TM1637_MODE_NORM); // Data set cmd, normal mode, auto incr, read data

  // Read keys
  // Bitpattern S0 S1 S2 K1 K2 1 1 1  
  *keydata = _read();
//  printf("Key = 0x%02x\r\n", *keydata);

  _stop();  
  
  // Restore Data Write mode
  _writeCmd(TM1637_DATA_SET_CMD, TM1637_DATA_WR | TM1637_ADDR_INC | TM1637_MODE_NORM); // Data set cmd, normal mode, auto incr, write data  
      
  return (*keydata != TM1637_SW_NONE);    
}

/** Generate Start condition for TM1637
  *  @param  none
  *  @return none
  */ 

void TM1637::_start() {

  _dio=0;
  wait_us(1);
  _clk=0;
  wait_us(1);
}
/** Generate Stop condition for TM1637
  *  @param  none
  *  @return none
  */ 

void TM1637::_stop() {

  _dio=0;
  wait_us(1);  
  _clk=1;
  wait_us(1);
  _dio=1;
  wait_us(1);
}

/** Send byte to TM1637
  *  @param  int data
  *  @return none
  */ 

void TM1637::_write(int data) {
 
  for (int bit=0; bit<8; bit++) {    
    //The TM1637 expects LSB first
    if (((data >> bit) & 0x01) == 0x01) {
      _dio=1;      
    }
    else {    
      _dio=0;      
    }  
    wait_us(1);
    _clk=1;
    wait_us(1);
    _clk=0;  
    wait_us(1);
  }  

  _dio=1;
  
  // Prepare DIO to read data
  _dio.input();
  wait_us(3);
      
  // dummy Ack
  _clk=1;
  wait_us(1);
//  _ack = _dio;  
  _clk=0;  
  wait_us(1); 
  
  // Return DIO to output mode
  _dio.output();  
  wait_us(3);  

  _dio=1; //idle  
}

/** Read byte from TM1637
  *  @param  int senddata
  *  @return read byte 
  */ 
char TM1637::_read() {
  char keycode = 0;

  // Prepare DIO to read data
  _dio.input();
  wait_us(3);
    
  for (int bit=0; bit<8; bit++) {    
   
    //The TM1637 sends bitpattern: S0 S1 S2 K1 K2 1 1 1
    //Data is shifted out by the TM1637 on the falling edge of CLK
    //Observe sufficient delay to allow the Open Drain DIO to rise to H levels
    // Prepare to read next bit, LSB (ie S0) first. 
    // The code below flips bits for easier matching with datasheet
    keycode = keycode << 1;  

    _clk=1;
    wait_us(1);
    
    // Read next bit
    if (_dio) { keycode |= 0x01; }        

    _clk=0;        
    wait_us(5); // Delay to allow for slow risetime
  }  
  
  // Return DIO to output mode
  _dio.output();
  wait_us(3);  

  // dummy Ack
  _dio=0; //Ack   
  wait_us(1);
  
  _clk=1;
  wait_us(1);
  _clk=0;  
  wait_us(1); 

  _dio=1; //idle

  return keycode;
}

/** Write command and parameter to TM1637
  *  @param  int cmd Command byte
  *  &Param  int data Parameters for command
  *  @return none
  */  
void TM1637::_writeCmd(int cmd, int data){
    
  _start();

  _write((cmd & TM1637_CMD_MSK) | (data & ~TM1637_CMD_MSK));   
 
  _stop();          
}  

void TM1637::writeInt(int input) {
    int temp = input;
    char result[TM1637_DISPLAY_MEM];
    int size = TM1637_DISPLAY_MEM;
    if (temp > 9999) temp = temp % 10000;

    for (int i = size - 1; i >= 0; i--) {
        int digit = temp % 10;
        short font = FONT_7S[digit];
        //File the array
        result[i] = font;
        //printf("result[%d] = 0x%02X\r\n", i, result[i]);
        // Move to the next digit
        temp /= 10;
    }
    bool nonZeroFound = false;
    for(int i = 0; i < size-1; i++) {
        if(result[i] == C7_0 && !nonZeroFound) {
            result[i] = 0;
        }
        else {
            nonZeroFound = true;
        }
    }
    writeData(result);
}

