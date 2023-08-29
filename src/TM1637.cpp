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

/** Constructor for class for driving TM1637 LED controller with Serial bus interface device. 
 *  @brief Supports 6 digits @ 8 segments. 
 *   
 *  @param  PinName mosi Serial bus MOSI pin
 *  @param  PinName miso Serial bus MISO pin 
 *  @param  PinName sclk Serial bus SCLK pin
*/
// TM1637::TM1637(PinName mosi, PinName miso, PinName sclk) : _mosi(mosi), _miso(miso), _sclk(sclk) {

//   _init();
// }
TM1637::TM1637(PinName dio, PinName clk) : _dio(dio), _clk(clk) {

  _init();
}

/** Init the Serial interface and the controller
  * @param  none
  * @return none
  */ 
// void TM1637::_init(){
  
// //TM1637 uses a Serial bus that looks like I2C, but really is not.
// //It has Start and Stop conditions like I2C and an Ack pulse, but instead of slaveaddresses and a RW bit it just transmits commands and data.
// //init Serial bus
//   _mosi=1;
//   _sclk=1;  

// //init controller  
//   _display = TM1637_DSP_ON;
//   _bright  = TM1637_BRT_DEF; 
//   _writeCmd(TM1637_DSP_CTRL_CMD, _display | _bright );                                 // Display control cmd, display on/off, brightness   
  
//   _writeCmd(TM1637_DATA_SET_CMD, TM1637_DATA_WR | TM1637_ADDR_INC | TM1637_MODE_NORM); // Data set cmd, normal mode, auto incr, write data  
// }   
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
// bool TM1637::getKeys(KeyData_t *keydata) {

//   _start();

//   // Enable Key Read mode
//   _write(TM1637_DATA_SET_CMD | TM1637_KEY_RD | TM1637_ADDR_INC | TM1637_MODE_NORM); // Data set cmd, normal mode, auto incr, read data

//   // Read keys
//   *keydata = _read(0xFF);

//   _stop();  
  
//   // Restore Data Write mode
//   _writeCmd(TM1637_DATA_SET_CMD, TM1637_DATA_WR | TM1637_ADDR_INC | TM1637_MODE_NORM); // Data set cmd, normal mode, auto incr, write data  
      
//   return (*keydata != TM1637_SW_NONE);    
// }
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
// void TM1637::_start() {

//   _mosi=0;
//   wait_us(1);
//   _sclk=0;
//   wait_us(1);
  
// }
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
// void TM1637::_stop() {

//   _mosi=0;
//   wait_us(1);  
//   _sclk=1;
//   wait_us(1);
//   _mosi=1;
//   wait_us(1);
// }
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
// void TM1637::_write(int data) {
 
//   for (int bit=0; bit<8; bit++) {    
//     //The TM1637 expects LSB first
//     if (((data >> bit) & 0x01) == 0x01) {
//       _mosi=1;      
//     }
//     else {    
//       _mosi=0;      
//     }  
//     wait_us(1);
//     _sclk=1;
//     wait_us(1);
//     _sclk=0;  
//     wait_us(1);
//   }  
  
//   // dummy Ack
//   _sclk=1;
//   wait_us(1);
//   _sclk=0;  
//   wait_us(1); 
// }

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

// TM1637::DisplayData_t intToChar(int input) {
//     // Find the number of digits in the input
//     int numDigits = 0;
//     int temp = input;
//     while (temp > 0) {
//         temp /= 10;
//         numDigits++;
//     }

//     // Allocate memory for the result char array
//     char* result = new char[numDigits * 7 + 1]; // 7 is the maximum size of FONT_7S

//     // Decompose the input integer into digits and look up in FONT_7S
//     temp = input;
//     int index = 0;
//     while (temp > 0) {
//         int digit = temp % 10;
//         short font = FONT_7S[digit];
//         printf("digit = %d, font = 0x%02x\r\n", digit, font);

//         // Copy the character array from FONT_7S to the result
//         // for (int i = 7; i > 0; i--) {
//         //     result[index++] = (font & (1 << (6 - i))) ? '1' : '0';   
//         // }
//         // result[index++] = ',';
//         sprintf(result + index, "0x%02x", font);
//         index += 4;
//         result[index++] = ',';
//         // Move to the next digit
//         temp /= 10;
//     }
//     //result[index - 1] = '\0';
//     // if (index >= 2) {
//     //     result[index - 2] = '\0';
//     // }

//     // Null-terminate the result char array
//     result[index-1] = '\0';

//     printf("result = %s\r\n", result);
//     // Reverse the result because we filled it in reverse order
//     // int start = 0;
//     // int end = index - 2;
//     // while (start < end) {
//     //     char temp = result[start];
//     //     result[start] = result[end];
//     //     result[end] = temp;
//     //     start++;
//     //     end--;
//     // }
//     result[index-1] = '\0';
//     printf("result = %s\r\n", result);
//     return result;
// }

#if (CATALEX_TEST == 1) 
// Derived class for TM1637 used in LED&KEY display unit
//

/** Constructor for class for driving TM1637 LED controller as used in CATALEX
  *
  *  @brief Supports 8 Digits of 7 Segments + DP.
  *   
  *  @param  PinName mosi Serial bus MOSI pin
  *  @param  PinName miso Serial bus MISO pin  
  *  @param  PinName sclk Serial bus SCLK pin
  */
// TM1637_CATALEX::TM1637_CATALEX(PinName mosi, PinName miso, PinName sclk) : TM1637(mosi, miso, sclk) {
//   _column  = 0;
//   _columns = CATALEX_NR_DIGITS;    
// }  
TM1637_CATALEX::TM1637_CATALEX(PinName dio, PinName clk) : TM1637(dio, clk) {
  _column  = 0;
  _columns = CATALEX_NR_DIGITS;    
}  

#if(0)
#if DOXYGEN_ONLY
    /** Write a character to the Display
     *
     * @param c The character to write to the display
     */
    int putc(int c);

    /** Write a formatted string to the Display
     *
     * @param format A printf-style format string, followed by the
     *               variables to use in formatting the string.
     */
    int printf(const char* format, ...);   
#endif
#endif

/** Locate cursor to a screen column
  *
  * @param column  The horizontal position from the left, indexed from 0
  */
void TM1637_CATALEX::locate(int column) {
  //sanity check
  if (column < 0) {column = 0;}
  if (column > (_columns - 1)) {column = _columns - 1;}  
  
  _column = column;       
}


/** Number of screen columns
  *
  * @param none
  * @return columns
  */
int TM1637_CATALEX::columns() {
    return _columns;
}

    
/** Clear the screen and locate to 0
  * @param bool clrAll Clear Icons also (default = false)
  */ 
void TM1637_CATALEX::cls(bool clrAll) {  

  if (clrAll) {
    //clear local buffer (inclumosig Icons)
    for (int idx=0; idx < CATALEX_NR_GRIDS; idx++) {
      _displaybuffer[idx] = 0x00;  
    }
  }  
  else {
    //clear local buffer (preserving Icons)
    for (int idx=0; idx < CATALEX_NR_GRIDS; idx++) {
      _displaybuffer[idx] = _displaybuffer[idx] & MASK_ICON_GRID[idx];  
    }  
  }

  writeData(_displaybuffer, (CATALEX_NR_GRIDS * TM1637_BYTES_PER_GRID));

  _column = 0;   
}     

/** Set Icon
  *
  * @param Icon icon Enums Icon has Grid position encoded in 8 MSBs, Icon pattern encoded in 16 LSBs
  * @return none
  */
void TM1637_CATALEX::setIcon(Icon icon) {
  int addr, icn;

   icn =        icon  & 0xFFFF;
  addr = (icon >> 24) & 0xFF; 
  addr = (addr - 1);
    
  //Save char...and set bits for icon to write
  _displaybuffer[addr] = _displaybuffer[addr] | LO(icn);      
//  writeData(_displaybuffer, (CATALEX_NR_GRIDS * TM1637_BYTES_PER_GRID));
  writeData(_displaybuffer, TM1637_BYTES_PER_GRID, addr);  
}

/** Clr Icon
  *
  * @param Icon icon Enums Icon has Grid position encoded in 8 MSBs, Icon pattern encoded in 16 LSBs
  * @return none
  */
void TM1637_CATALEX::clrIcon(Icon icon) {
  int addr, icn;

   icn =        icon  & 0xFFFF;
  addr = (icon >> 24) & 0xFF; 
  addr = (addr - 1);
    
  //Save char...and clr bits for icon to write
  _displaybuffer[addr] = _displaybuffer[addr] & ~LO(icn);      
//  writeData(_displaybuffer, (CATALEX_NR_GRIDS * TM1637_BYTES_PER_GRID));
  writeData(_displaybuffer, TM1637_BYTES_PER_GRID, addr);    
}


/** Set User Defined Characters (UDC)
  *
  * @param unsigned char udc_idx  The Index of the UDC (0..7)
  * @param int udc_data           The bitpattern for the UDC (8 bits)       
  */
void TM1637_CATALEX::setUDC(unsigned char udc_idx, int udc_data) {

  //Sanity check
  if (udc_idx > (CATALEX_NR_UDC-1)) {
    return;
  }
  // Mask out Icon bits?

  _UDC_7S[udc_idx] = LO(udc_data);
}


/** Write a single character (Stream implementation)
  */
int TM1637_CATALEX::_putc(int value) {
    int addr;
    bool validChar = false;
    char pattern   = 0x00;
    
    if ((value == '\n') || (value == '\r')) {
      //No character to write
      validChar = false;
      
      //Update Cursor      
      _column = 0;
    }
    else if ((value == '.') || (value == ',')) {
      //No character to write
      validChar = false;
      pattern = S7_DP; // placeholder for all DPs
      
      // Check to see that DP can be shown for current column
      if (_column > 0) {
        //Translate between _column and displaybuffer entries
        //Add DP to bitpattern of digit left of current column.
        addr = (_column - 1);
      
        //Save icons...and set bits for decimal point to write
        _displaybuffer[addr] = _displaybuffer[addr] | pattern;
//        writeData(_displaybuffer, (CATALEX_NR_GRIDS * TM1637_BYTES_PER_GRID));
        writeData(_displaybuffer, TM1637_BYTES_PER_GRID, addr); 
        
        //No Cursor Update
      }
    }
    else if ((value >= 0) && (value < CATALEX_NR_UDC)) {
      //Character to write
      validChar = true;
      pattern = _UDC_7S[value];
    }  
    
#if (SHOW_ASCII == 1)
    //display all ASCII characters
    else if ((value >= FONT_7S_START) && (value <= FONT_7S_END)) {   
      //Character to write
      validChar = true;
      pattern = FONT_7S[value - FONT_7S_START];
    } // else
#else    
    //display only digits and hex characters      
    else if (value == '-') {
      //Character to write
      validChar = true;
      pattern = C7_MIN;         
    }
    else if ((value >= (int)'0') && (value <= (int) '9')) {   
      //Character to write
      validChar = true;
      pattern = FONT_7S[value - (int) '0'];
    }
    else if ((value >= (int) 'A') && (value <= (int) 'F')) {   
      //Character to write
      validChar = true;
      pattern = FONT_7S[10 + value - (int) 'A'];
    }
    else if ((value >= (int) 'a') && (value <= (int) 'f')) {   
      //Character to write
      validChar = true;
      pattern = FONT_7S[10 + value - (int) 'a'];
    } //else
#endif

    if (validChar) {
      //Character to write
 
      //Translate between _column and displaybuffer entries
      addr = _column;

      //Save icons...and set bits for character to write
      _displaybuffer[addr] = (_displaybuffer[addr] & MASK_ICON_GRID[_column]) | pattern;

//      writeData(_displaybuffer, (CATALEX_NR_GRIDS * TM1637_BYTES_PER_GRID));
      writeData(_displaybuffer, TM1637_BYTES_PER_GRID, addr);        
                                
      //Update Cursor
      _column++;
      if (_column > (CATALEX_NR_DIGITS - 1)) {
        _column = 0;
      }

    } // if validChar           

    return value;
}


// get a single character (Stream implementation)
int TM1637_CATALEX::_getc() {
    return -1;
}

#endif
