// /* mbed TM1637 Library, for TM1637 LED controller
//  * Copyright (c) 2016, v01: WH, Initial version
//  *               2017, v02: WH, Added RobotDyn 6 Digit module,
//  *                          Added Eyewink 6 Digit + 6 Keys module,   
//  *                          Constructor adapted to 2 pins: dio, clk  
//  *
//  * Permission is hereby granted, free of charge, to any person obtaining a copy
//  * of this software and associated documentation files (the "Software"), to deal
//  * in the Software without restriction, inclumosig without limitation the rights
//  * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
//  * copies of the Software, and to permit persons to whom the Software is
//  * furnished to do so, subject to the following conditions:
//  *
//  * The above copyright notice and this permission notice shall be included in
//  * all copies or substantial portions of the Software.
//  *
//  * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//  * IMPLIED, INCLUmosiG BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//  * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//  * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//  * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//  * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
//  * THE SOFTWARE.
//  */

// #ifndef TM1637_H
// #define TM1637_H

// // Select one of the testboards for TM1637 LED controller
// #include "TM1637_Config.h"

// /** An interface for driving TM1637 LED controller
//  *
//  * @code
//  * #include "mbed.h"
//  * #include "TM1637.h" 
//  * 
//  * Serial pc(USBTX, USBRX);
//  *
//  * //DisplayData_t size is 6 bytes (6 grids @ 8 segments)
//  * TM1637::DisplayData_t all_str  = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};  
//  *
//  * // KeyData_t size is 1 bytes  
//  * TM1637::KeyData_t keydata; 
//  *
//  * // TM1637 declaration, Select the desired type in TM1637_Config.h
//  * //TM1637_CATALEX CATALEX(p5,p6,p7);     //LPC1768 DEPRECATED version 
//  * TM1637_CATALEX CATALEX(p6, p7);       //LPC1768
//  * //TM1637_CATALEX CATALEX(D9, D10);      //F401
//  *
//  * int main() {
//  *   CATALEX.cls(); 
//  *   CATALEX.writeData(all_str);
//  *   wait(1);
//  *   CATALEX.setBrightness(TM1637_BRT0);
//  *   wait(1);
//  *   CATALEX.setBrightness(TM1637_BRT3);
//  *
//  *   while (1) {
//  *     CATALEX.cls(); 
//  *     wait(0.5); 
//  *     CATALEX.writeData(all_str);
//  *     wait(1.0);
//  *     CATALEX.cls(); 
//  *     CATALEX.printf(" HI ");  
//  *     wait(1.0); 
//  *
//  *     // Check and read keydata
//  *     if (CATALEX.getKeys(&keydata)) {
//  *       pc.printf("Keydata = 0x%02x\r\n", keydata);
//  *
//  *       if (keydata == TM1637_SW9_BIT) { //sw9  
//  *         CATALEX.cls(); 
//  *         CATALEX.printf("--09"); 
//  *       }  
//  *     } // Check keydata
//  *   } // while 
//  * }
//  * @endcode
//  */


// //TM1637 Display data
// #define TM1637_MAX_NR_GRIDS    6
// #define TM1637_BYTES_PER_GRID  1

// //Significant bits Keymatrix data
// //#define TM1638_KEY_MSK      0xFF 

// //Memory size in bytes for Display and Keymatrix
// #define TM1637_DISPLAY_MEM  (TM1637_MAX_NR_GRIDS * TM1637_BYTES_PER_GRID)
// #define TM1637_KEY_MEM         2

// //Reserved bits for commands
// #define TM1637_CMD_MSK      0xC0

// //Data setting commands
// #define TM1637_DATA_SET_CMD 0x40
// #define TM1637_DATA_WR      0x00
// #define TM1637_KEY_RD       0x02
// #define TM1637_ADDR_INC     0x00
// #define TM1637_ADDR_FIXED   0x04
// #define TM1637_MODE_NORM    0x00
// #define TM1637_MODE_TEST    0x08

// //Address setting commands
// #define TM1637_ADDR_SET_CMD 0xC0
// #define TM1637_ADDR_MSK     0x07 //0..5

// //Display control commands
// #define TM1637_DSP_CTRL_CMD 0x80
// #define TM1637_BRT_MSK      0x07
// #define TM1637_BRT0         0x00 //Pulsewidth 1/16
// #define TM1637_BRT1         0x01
// #define TM1637_BRT2         0x02
// #define TM1637_BRT3         0x03
// #define TM1637_BRT4         0x04
// #define TM1637_BRT5         0x05
// #define TM1637_BRT6         0x06
// #define TM1637_BRT7         0x07 //Pulsewidth 14/16

// #define TM1637_BRT_DEF      TM1637_BRT3

// #define TM1637_DSP_OFF      0x00
// #define TM1637_DSP_ON       0x08


// //Access to 16 Switches
// //S0 S1 S2 K1 K2 1 1 1
// //K1,K2 = 0 1
// #define TM1637_SW1_BIT      0xEF
// #define TM1637_SW2_BIT      0x6F
// #define TM1637_SW3_BIT      0xAF
// #define TM1637_SW4_BIT      0x2F
// #define TM1637_SW5_BIT      0xCF
// #define TM1637_SW6_BIT      0x4F
// #define TM1637_SW7_BIT      0x8F
// #define TM1637_SW8_BIT      0x0F

// //K1,K2 = 1 0
// #define TM1637_SW9_BIT      0xF7
// #define TM1637_SW10_BIT     0x77
// #define TM1637_SW11_BIT     0xB7
// #define TM1637_SW12_BIT     0x37
// #define TM1637_SW13_BIT     0xD7
// #define TM1637_SW14_BIT     0x57
// #define TM1637_SW15_BIT     0x97
// #define TM1637_SW16_BIT     0x17

// #define TM1637_SW_NONE      0xFF

// /** A class for driving TM1637 LED controller
//  *
//  * @brief Supports 6 Grids @ 8 Segments and 16 Keys. 
//  *        Serial bus interface device. 
//  */
// class TM1637 {
//  public:

//   /** Datatype for displaydata */
//   typedef char DisplayData_t[TM1637_DISPLAY_MEM];

//   /** Datatypes for keymatrix data */
//   typedef char KeyData_t;

// #if(SPI==1)    
//  /** Constructor for class for driving TM1637 LED controller
//   *
//   * @brief Supports 6 Grids @ 8 segments and 16 Keys. 
//   *        Serial bus interface device.
//   *        DEPRECATED version 
//   *
//   *  @param  PinName mosi_nc  Serial bus NC pin
//   *  @param  PinName miso_dio Serial bus DIO pin  
//   *  @param  PinName sclk_clk Serial bus CLK pin 
//   */
//   TM1637(PinName mosi_nc, PinName miso_dio, PinName sclk_clk);
// #endif      

//  /** Constructor for class for driving TM1637 LED controller
//   *
//   * @brief Supports 6 Grids @ 8 segments and 16 Keys. 
//   *        Serial bus interface device. 
//   *
//   *  @param  PinName dio Serial bus DIO pin
//   *  @param  PinName sck Serial bus CLK pin 
//   */
//   TM1637(PinName dio, PinName clk);


//   /** Clear the screen and locate to 0
//    */ 
//   void cls();  

//   /** Write databyte to TM1637
//    *  @param  char data byte written at given address
//    *  @param  int address display memory location to write byte
//    *  @return none
//    */ 
//    void writeData(char data, int address); 

//    /** Write Display datablock to TM1637
//     *  @param  DisplayData_t data Array of TM1637_DISPLAY_MEM (=6) bytes for displaydata
//     *  @param  length number bytes to write (valid range 0..(TM1637_MAX_NR_GRIDS * TM1637_BYTES_PER_GRID) (=6), when starting at address 0)  
//     *  @param  int address display memory location to write bytes (default = 0) 
//     *  @return none
//     */ 
//     void writeData(DisplayData_t data, int length = (TM1637_MAX_NR_GRIDS * TM1637_BYTES_PER_GRID), int address = 0);

//   /** Read keydata block from TM1637
//    *  @param  *keydata Ptr to bytes for keydata
//    *  @return bool keypress True when at least one key was pressed
//    *
//    */   
//   bool getKeys(KeyData_t *keydata);

//   /** Set Brightness
//     *
//     * @param  char brightness (3 significant bits, valid range 0..7 (1/16 .. 14/16 dutycycle)  
//     * @return none
//     */
//   void setBrightness(char brightness = TM1637_BRT_DEF);
  
//   /** Set the Display mode On/off
//     *
//     * @param bool display mode
//     */
//   void setDisplay(bool on);
  
//  private:  
// #if(SPI==1)
//     DigitalIn    _mosi_nc;  //Dummy to avoid breaking deprecated constructor    
// #endif
//     DigitalInOut _dio;
//     DigitalOut   _clk;  

//     char _display;
//     char _bright; 
  
//   /** Init the Serial interface and the controller
//     * @param  none
//     * @return none
//     */ 
//     void _init();


//   /** Generate Start condition for TM1637
//     *  @param  none
//     *  @return none
//     */ 
//     void _start();
  
//   /** Generate Stop condition for TM1637
//     *  @param  none
//     *  @return none
//     */ 
//     void _stop();

//   /** Send byte to TM1637
//     *  @param  int data
//     *  @return none
//     */ 
//     void _write(int data);

//   /** Read byte from TM1637
//     *  @return read byte 
//     */ 
//     char _read();

//   /** Write command and parameter to TM1637
//     *  @param  int cmd Command byte
//     *  &Param  int data Parameters for command
//     *  @return none
//     */ 
//     void _writeCmd(int cmd, int data);  
// };

// #if (CATALEX_TEST == 1) 
// // Derived class for TM1637 used in CATALEX display unit with 4 Digits
// //

// #include "Font_7Seg.h"

// #define CATALEX_NR_GRIDS  4
// #define CATALEX_NR_DIGITS 4
// #define CATALEX_NR_UDC    8


// /** Constructor for class for driving TM1637 controller as used in CATALEX
//   *
//   *  @brief Supports 4 Digits of 7 Segments + DP (or Colon for Digit2 on some models). 
//   *         Also Supports up to 16 Keys. Serial bus interface device. 
//   *  
//   *  @param  PinName mosi Serial bus MOSI pin
//   *  @param  PinName miso Serial bus MISO pin  
//   *  @param  PinName sclk Serial bus SCLK pin 
//   */
// class TM1637_CATALEX : public TM1637, public Stream {
//  public:

//   /** Enums for Icons */
//   //  Grid encoded in 8 MSBs, Icon pattern encoded in 16 LSBs
//   enum Icon {
//     DP1   = ( 1<<24) | S7_DP1,  /**<  Digit 1 */
//     DP2   = ( 2<<24) | S7_DP2,  /**<  Digit 2 */
//     DP3   = ( 3<<24) | S7_DP3,  /**<  Digit 3 */
//     DP4   = ( 4<<24) | S7_DP4,  /**<  Digit 4 */

//     COL2  = ( 2<<24) | S7_DP2,  /**<  Column 2 */
//   };
  
//   typedef char UDCData_t[CATALEX_NR_UDC];

// #if (SPI==1)  
//  /** Constructor for class for driving TM1637 LED controller as used in CATALEX
//   *
//   * @brief Supports 4 Digits of 7 Segments + DP (or Colon for Digit2 on some models).
//   *        Also Supports up to 16 Keys. Serial bus interface device.      
//   *        DEPRECATED version 
//   *
//   *  @param  PinName mosi_nc  Serial bus NC pin
//   *  @param  PinName miso_dio Serial bus DIO pin  
//   *  @param  PinName sclk_clk Serial bus CLK pin 
//   */
//   TM1637_CATALEX(PinName mosi_nc, PinName miso_dio, PinName sclk_clk);
// #endif

//  /** Constructor for class for driving TM1637 LED controller
//   *
//   * @brief Supports 4 Digits of 7 Segments + DP (or Colon for Digit2 on some models).
//   *        Also Supports up to 16 Keys. Serial bus interface device. 
//   *
//   *  @param  PinName dio Serial bus DIO pin
//   *  @param  PinName sck Serial bus CLK pin 
//   */
//   TM1637_CATALEX(PinName dio, PinName clk);

  
// #if DOXYGEN_ONLY
//     /** Write a character to the Display
//      *
//      * @param c The character to write to the display
//      */
//     int putc(int c);

//     /** Write a formatted string to the Display
//      *
//      * @param format A printf-style format string, followed by the
//      *               variables to use in formatting the string.
//      */
//     int printf(const char* format, ...);   
// #endif

//      /** Locate cursor to a screen column
//      *
//      * @param column  The horizontal position from the left, indexed from 0
//      */
//     void locate(int column);
    
//     /** Clear the screen and locate to 0
//      * @param bool clrAll Clear Icons also (default = false)
//      */
//     void cls(bool clrAll = false);

//     /** Set Icon
//      *
//      * @param Icon icon Enums Icon has Grid position encoded in 8 MSBs, Icon pattern encoded in 16 LSBs
//      * @return none
//      */
//     void setIcon(Icon icon);

//     /** Clr Icon
//      *
//      * @param Icon icon Enums Icon has Grid position encoded in 8 MSBs, Icon pattern encoded in 16 LSBs
//      * @return none
//      */
//     void clrIcon(Icon icon);

//    /** Set User Defined Characters (UDC)
//      *
//      * @param unsigned char udc_idx   The Index of the UDC (0..7)
//      * @param int udc_data            The bitpattern for the UDC (16 bits)       
//      */
//     void setUDC(unsigned char udc_idx, int udc_data);


//    /** Number of screen columns
//     *
//     * @param none
//     * @return columns
//     */
//     int columns();   

//    /** Write databyte to TM1637
//      *  @param  char data byte written at given address
//      *  @param  int address display memory location to write byte
//      *  @return none
//      */ 
//     void writeData(char data, int address){
//       TM1637::writeData(data, address);
//     }        

//    /** Write Display datablock to TM1637
//     *  @param  DisplayData_t data Array of TM1637_DISPLAY_MEM (=4) bytes for displaydata
//     *  @param  length number bytes to write (valid range 0..(CATALEX_NR_GRIDS * TM1637_BYTES_PER_GRID) (=4), when starting at address 0)  
//     *  @param  int address display memory location to write bytes (default = 0)
//     *  @return none
//     */   
//     void writeData(DisplayData_t data, int length = (CATALEX_NR_GRIDS * TM1637_BYTES_PER_GRID), int address = 0) {
//       TM1637::writeData(data, length, address);
//     }  

// protected:  
//     // Stream implementation functions
//     virtual int _putc(int value);
//     virtual int _getc();

// private:
//     int _column;
//     int _columns;   
    
//     DisplayData_t _displaybuffer;
//     UDCData_t _UDC_7S; 
// };
// #endif

// #if (ROBOTDYN_TEST == 1) 
// // Derived class for TM1637 used in ROBOTDYN 6 Digit display unit
// //

// #include "Font_7Seg.h"

// #define ROBOTDYN_NR_GRIDS  6
// #define ROBOTDYN_NR_DIGITS 6
// #define ROBOTDYN_NR_UDC    8


// /** Constructor for class for driving TM1637 controller as used in ROBOTDYN
//   *
//   *  @brief Supports 6 Digits of 7 Segments + DP.
//   *  
//   *  @param  PinName dio Serial bus DIO pin
//   *  @param  PinName clk Serial bus CLK pin 
//   */
// class TM1637_ROBOTDYN : public TM1637, public Stream {
//  public:

//   /** Enums for Icons */
//   //  Grid encoded in 8 MSBs, Icon pattern encoded in 16 LSBs
//   //  Note that Digits 1,3 and 4,6 are swapped!
//   enum Icon {
//     DP1   = ( 3<<24) | S7_DP1,  /**<  Digit 1 */
//     DP2   = ( 2<<24) | S7_DP2,  /**<  Digit 2 */
//     DP3   = ( 1<<24) | S7_DP3,  /**<  Digit 3 */
//     DP4   = ( 6<<24) | S7_DP4,  /**<  Digit 4 */
//     DP5   = ( 5<<24) | S7_DP5,  /**<  Digit 5 */
//     DP6   = ( 4<<24) | S7_DP6,  /**<  Digit 6 */   
//   };
  
//   typedef char UDCData_t[ROBOTDYN_NR_UDC];

//  /** Constructor for class for driving TM1637 LED controller as used in ROBOTDYN
//    *
//    * @brief Supports 6 Digits of 7 Segments + DP. Also supports up to 16 Keys.
//    *  
//    *  @param  PinName dio Serial bus DIO pin
//    *  @param  PinName clk Serial bus CLK pin 
//   */
//   TM1637_ROBOTDYN(PinName dio, PinName clk);


// #if DOXYGEN_ONLY
//     /** Write a character to the Display
//      *
//      * @param c The character to write to the display
//      */
//     int putc(int c);

//     /** Write a formatted string to the Display
//      *
//      * @param format A printf-style format string, followed by the
//      *               variables to use in formatting the string.
//      */
//     int printf(const char* format, ...);   
// #endif

//      /** Locate cursor to a screen column
//      *
//      * @param column  The horizontal position from the left, indexed from 0
//      */
//     void locate(int column);
    
//     /** Clear the screen and locate to 0
//      * @param bool clrAll Clear Icons also (default = false)
//      */
//     void cls(bool clrAll = false);

//     /** Set Icon
//      *
//      * @param Icon icon Enums Icon has Grid position encoded in 8 MSBs, Icon pattern encoded in 16 LSBs
//      * @return none
//      */
//     void setIcon(Icon icon);

//     /** Clr Icon
//      *
//      * @param Icon icon Enums Icon has Grid position encoded in 8 MSBs, Icon pattern encoded in 16 LSBs
//      * @return none
//      */
//     void clrIcon(Icon icon);

//    /** Set User Defined Characters (UDC)
//      *
//      * @param unsigned char udc_idx   The Index of the UDC (0..7)
//      * @param int udc_data            The bitpattern for the UDC (16 bits)       
//      */
//     void setUDC(unsigned char udc_idx, int udc_data);


//    /** Number of screen columns
//     *
//     * @param none
//     * @return columns
//     */
//     int columns();   

//    /** Write databyte to TM1637
//      *  @param  char data byte written at given address
//      *  @param  int address display memory location to write byte
//      *  @return none
//      */ 
//     void writeData(char data, int address){
//       TM1637::writeData(data, address);
//     }        

//    /** Write Display datablock to TM1637
//     *  @param  DisplayData_t data Array of TM1637_DISPLAY_MEM (=6) bytes for displaydata
//     *  @param  length number bytes to write (valid range 0..(ROBOTDYN_NR_GRIDS * TM1637_BYTES_PER_GRID) (=6), when starting at address 0)  
//     *  @param  int address display memory location to write bytes (default = 0)
//     *  @return none
//     */   
//     void writeData(DisplayData_t data, int length = (ROBOTDYN_NR_GRIDS * TM1637_BYTES_PER_GRID), int address = 0) {
//       TM1637::writeData(data, length, address);
//     }  

// protected:  
//     // Stream implementation functions
//     virtual int _putc(int value);
//     virtual int _getc();

// private:
//     int _column;
//     int _columns;   
    
//     DisplayData_t _displaybuffer;
//     UDCData_t _UDC_7S; 
// };
// #endif


// #if (EYEWINK_TEST == 1) 
// // Derived class for TM1637 used in EYEWINK 6 Digit + 6 Keys display unit
// //

// #include "Font_7Seg.h"

// #define EYEWINK_NR_GRIDS  6
// #define EYEWINK_NR_DIGITS 6
// #define EYEWINK_NR_UDC    8


// /** Constructor for class for driving TM1637 controller as used in EYEWINK
//   *
//   *  @brief Supports 6 Digits of 7 Segments + DP and 6 Keys.
//   *  
//   *  @param  PinName dio Serial bus DIO pin
//   *  @param  PinName clk Serial bus CLK pin 
//   */
// class TM1637_EYEWINK : public TM1637, public Stream {
//  public:

//   /** Enums for Icons */
//   //  Grid encoded in 8 MSBs, Icon pattern encoded in 16 LSBs
//   enum Icon {
//     DP1   = ( 1<<24) | S7_DP1,  /**<  Digit 1 */
//     DP2   = ( 2<<24) | S7_DP2,  /**<  Digit 2 */
//     DP3   = ( 3<<24) | S7_DP3,  /**<  Digit 3 */
//     DP4   = ( 4<<24) | S7_DP4,  /**<  Digit 4 */
//     DP5   = ( 5<<24) | S7_DP5,  /**<  Digit 5 */
//     DP6   = ( 6<<24) | S7_DP6,  /**<  Digit 6 */   
//   };
  
//   typedef char UDCData_t[EYEWINK_NR_UDC];

//  /** Constructor for class for driving TM1637 LED controller as used in EYEWINK
//    *
//    * @brief Supports 6 Digits of 7 Segments + DP.
//    *  
//    *  @param  PinName dio Serial bus DIO pin
//    *  @param  PinName clk Serial bus CLK pin 
//   */
//   TM1637_EYEWINK(PinName dio, PinName clk);


// #if DOXYGEN_ONLY
//     /** Write a character to the Display
//      *
//      * @param c The character to write to the display
//      */
//     int putc(int c);

//     /** Write a formatted string to the Display
//      *
//      * @param format A printf-style format string, followed by the
//      *               variables to use in formatting the string.
//      */
//     int printf(const char* format, ...);   
// #endif

//      /** Locate cursor to a screen column
//      *
//      * @param column  The horizontal position from the left, indexed from 0
//      */
//     void locate(int column);
    
//     /** Clear the screen and locate to 0
//      * @param bool clrAll Clear Icons also (default = false)
//      */
//     void cls(bool clrAll = false);

//     /** Set Icon
//      *
//      * @param Icon icon Enums Icon has Grid position encoded in 8 MSBs, Icon pattern encoded in 16 LSBs
//      * @return none
//      */
//     void setIcon(Icon icon);

//     /** Clr Icon
//      *
//      * @param Icon icon Enums Icon has Grid position encoded in 8 MSBs, Icon pattern encoded in 16 LSBs
//      * @return none
//      */
//     void clrIcon(Icon icon);

//    /** Set User Defined Characters (UDC)
//      *
//      * @param unsigned char udc_idx   The Index of the UDC (0..7)
//      * @param int udc_data            The bitpattern for the UDC (16 bits)       
//      */
//     void setUDC(unsigned char udc_idx, int udc_data);


//    /** Number of screen columns
//     *
//     * @param none
//     * @return columns
//     */
//     int columns();   

//    /** Write databyte to TM1637
//      *  @param  char data byte written at given address
//      *  @param  int address display memory location to write byte
//      *  @return none
//      */ 
//     void writeData(char data, int address){
//       TM1637::writeData(data, address);
//     }        

//    /** Write Display datablock to TM1637
//     *  @param  DisplayData_t data Array of TM1637_DISPLAY_MEM (=6) bytes for displaydata
//     *  @param  length number bytes to write (valid range 0..(EYEWINK_NR_GRIDS * TM1637_BYTES_PER_GRID) (=6), when starting at address 0)  
//     *  @param  int address display memory location to write bytes (default = 0)
//     *  @return none
//     */   
//     void writeData(DisplayData_t data, int length = (EYEWINK_NR_GRIDS * TM1637_BYTES_PER_GRID), int address = 0) {
//       TM1637::writeData(data, length, address);
//     }  

// protected:  
//     // Stream implementation functions
//     virtual int _putc(int value);
//     virtual int _getc();

// private:
//     int _column;
//     int _columns;   
    
//     DisplayData_t _displaybuffer;
//     UDCData_t _UDC_7S; 
// };
// #endif

// #endif