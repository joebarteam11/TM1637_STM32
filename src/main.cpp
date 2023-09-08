/* mbed TM1637 Test program, for TM1637 LED controller
 * Copyright (c) 2016, v01: WH, Initial version
 *               2021,    : MT, Ported to mbed-os
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

#include "mbed.h"
#include "TM1637.h"
#include "TM1637_Config.h"

#if (TM1637_TEST == 1)
// screen TM1637 4 Digit display test

#include "Font_7Seg.h"

DigitalOut myled(LED1);  //NOTE: On F401 LED1 is Pin D13, which is SCK!

// DisplayData_t size is 4 bytes (4 Grids @ 7 Segments) 
TM1637::DisplayData_t all_str   = {0xFF, 0xFF, 0xFF, 0xFF};  
TM1637::DisplayData_t cls_str   = {0x00, 0x00, 0x00, 0x00};  
TM1637::DisplayData_t hello_str = {C7_H, C7_I, 0x00, 0x00};
TM1637::DisplayData_t bye_str   = {C7_B, C7_Y, C7_E, 0x00};

// KeyData_t size is 1 bytes  
TM1637::KeyData_t keydata; 

//New Constructor
TM1637 screen(D8, D7);    
TM1637::DisplayData_t data;

// Function to convert an integer to a fixed-size array of short int using FONT_7S containg the 7-segment font
void intTo7seg(int input, char* result, int size) {
    int temp = input;
    for (int i = size - 1; i >= 0; i--) {
        int digit = temp % 10;
        short font = FONT_7S[digit];
        //File the array
        result[i] = font;
        printf("result[%d] = 0x%02X\r\n", i, result[i]);
        // Move to the next digit
        temp /= 10;
    }
    bool nonZeroFound = false;
    for(int i = 0; i < size; i++) {
        if(result[i] == C7_0 && !nonZeroFound) {
            result[i] = 0;
        }
        else {
            nonZeroFound = true;
        }
    }
}


int main() {
    
    printf("Hello World\r\n"); //    
    screen.cls();        
    wait_us(1000000);
    screen.setBrightness(TM1637_BRT3);            
    wait_us(1000000);
    screen.cls(); 
    screen.writeData(hello_str); 
    wait_us(1000000);
    screen.writeData(bye_str);
    wait_us(1000000);
    screen.cls();

    for(int i = 0; i < 10000; i++) {
        //screen.cls();
        intTo7seg(i,data,sizeof(data));
        screen.writeData(data);
        wait_us(5000);
    }
}
#endif
