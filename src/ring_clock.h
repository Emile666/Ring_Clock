#ifndef RING_CLOCK_H
#define RING_CLOCK_H

#include <iostm8s103f3.h>
#include <intrinsics.h> 
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>

//-----------------------------------------------------------------------------------------------
//  Schematic of the connections to the MCU.
// 
//                                      STM8S103F3
//                                     ------------
//                                 PD4 | 1     20 | PD3/AIN4 
//                      UART1-TX   PD5 | 2     19 | PD2/AIN3 
//                      UART1-RX   PD6 | 3     18 | PD1      SWIM
//                                nRST | 4     17 | PC7      
//                                 PA1 | 5     16 | PC6      
//                                 PA2 | 6     15 | PC5      
//                                 GND | 7     14 | PC4      
//                                VCAP | 8     13 | PC3      DI_3V3
//                                 VCC | 9     12 | PB4      I2C-SCL
//                                 PA3 | 10    11 | PB5      I2C-SDA
//                                     ------------
//-----------------------------------------------------------------------------------------------
#define DI_3V3 (0x08)
#define TX     (0x20)
#define RX     (0x40)

//-----------------------------------------------------------------------------------------------
// https://wp.josh.com/2014/05/13/ws2812-neopixels-are-not-so-finicky-once-you-get-to-know-them/
//
// At 16 MHz, 1 NOP is approx. 62.5 nsec.
//
// Symbol Parameter	                Min	Typical	Max	Units Measured
// T0H	  0 code ,high voltage time	200	350	500	ns    360..400
// T1H	  1 code ,high voltage time	550	700	5.500	ns    760
// TLD	  data, low voltage time	450	600	5.000	ns    1120
// TLL	  latch, low voltage time	6.000			ns    3120 (max)
//
//-----------------------------------------------------------------------------------------------
#define wait_T0H  __asm("nop\n nop\n nop\n nop\n nop")
#define wait_T1H  wait_T0H; wait_T0H; __asm("nop")

#define ws2812b_send_1   PC_ODR |=  0x08; /* Turn PC3 on */  \
                         wait_T1H;                           \
                         PC_ODR &= ~0x08; /* Turn PC3 off */ 
#define ws2812b_send_0   PC_ODR |=  0x08; /* Turn PC3 on */  \
                         wait_T0H;                           \
                         PC_ODR &= ~0x08; /* Turn PC3 off */ 
                             
//-------------------------------------------------
// The Number of WS2812B devices present
// For the full-size clock, this is 3 x 60 = 180
// For one PCB segment, this is 36
// For the small test ring, this is 12
//-------------------------------------------------
#define NR_LEDS (180)                    
                         
void print_date_and_time(void);
void print_dow(uint8_t dow);
void execute_single_command(char *s);
void rs232_command_handler(void);

#endif
