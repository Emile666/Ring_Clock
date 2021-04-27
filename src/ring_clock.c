/*==================================================================
  File Name    : ring_clock.c
  Author       : Emile
  ------------------------------------------------------------------
  Purpose : This files contains all the functions for a hardware
            project called the Ring-Clock. It consists of 3 rings of
            60 WS2812 LEDs, making a total of 180 WS2812 LEDs.
            The inner ring is used for the seconds, the middle ring
            for the minutes and the outer ring for the hours.
  ------------------------------------------------------------------
  This is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
 
  This file is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
 
  You should have received a copy of the GNU General Public License
  along with this file.  If not, see <http://www.gnu.org/licenses/>.
  --------------------------------------------------------------------
  Revision 0.33  2021/03/30 Emile
  - Bugfix: time was reset at power-up: dt struct not initialized
  - Blanking begin and end the same now disables blanking
  ====================================================================*/ 
#include "ring_clock.h"
#include "delay.h"
#include "scheduler.h"
#include "i2c_bb.h"
#include "i2c_ds3231_bb.h"
#include "uart.h"
#include "eep.h"

char     rs232_inbuf[UART_BUFLEN]; // buffer for RS232 commands
uint8_t  rs232_ptr     = 0;        // index in RS232 buffer
extern uint32_t t2_millis;         // Updated in TMR2 interrupt

char     ring_clk_ver[] = "Ring Clock v0.33\n";
uint8_t led_r[NR_LEDS]; // Array with 8-bit red colour for all WS2812
uint8_t led_g[NR_LEDS]; // Array with 8-bit green colour for all WS2812
uint8_t led_b[NR_LEDS]; // Array with 8-bit blue colour for all WS2812
uint8_t slpos   = 0;    // position of sling
uint8_t enable_test_pattern = 0; // 1 = enable WS2812 test-pattern
uint8_t watchdog_test       = 0; // 1 = watchdog test modus
uint8_t led_intensity;           // Intensity of WS2812 LEDs
bool    dst_active          = false; // true = Daylight Saving Time active
Time    dt;             // Struct with time and date values, updated every sec.

uint8_t blank_begin_h  = 23;
uint8_t blank_begin_m  = 30;
uint8_t blank_end_h    =  8;
uint8_t blank_end_m    = 30;

/*-----------------------------------------------------------------------------
  Purpose  : This is the interrupt routine for the Timer 2 Overflow handler.
             It runs at 1 kHz and drives the scheduler and the multiplexer.
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
#pragma vector = TIM2_OVR_UIF_vector
__interrupt void TIM2_UPD_OVF_IRQHandler(void)
{
    scheduler_isr();  // Run scheduler interrupt function
    t2_millis++;      // update milliseconds timer
    TIM2_SR1_UIF = 0; // Reset the interrupt otherwise it will fire again straight away.
} // TIM2_UPD_OVF_IRQHandler()

/*-----------------------------------------------------------------------------
  Purpose  : This routine initialises the system clock to run at 16 MHz.
             It uses the internal HSI oscillator.
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
void initialise_system_clock(void)
{
    CLK_ICKR       = 0;           //  Reset the Internal Clock Register.
    CLK_ICKR_HSIEN = 1;           //  Enable the HSI.
    while (CLK_ICKR_HSIRDY == 0); //  Wait for the HSI to be ready for use.
    CLK_CKDIVR     = 0;           //  Ensure the clocks are running at full speed.
 
    // The datasheet lists that the max. ADC clock is equal to 6 MHz (4 MHz when on 3.3V).
    // Because fMASTER is now at 16 MHz, we need to set the ADC-prescaler to 4.
    ADC_CR1_SPSEL  = 0x02;        //  Set prescaler to 4, fADC = 4 MHz
    CLK_SWIMCCR    = 0;           //  Set SWIM to run at clock / 2.
    CLK_SWR        = 0xe1;        //  Use HSI as the clock source.
    CLK_SWCR       = 0;           //  Reset the clock switch control register.
    CLK_SWCR_SWEN  = 1;           //  Enable switching.
    while (CLK_SWCR_SWBSY != 0);  //  Pause while the clock switch is busy.
} // initialise_system_clock()

/*-----------------------------------------------------------------------------
  Purpose  : This routine initialises Timer 2 to generate a 1 kHz interrupt.
             16 MHz / (  16 *  1000) = 1000 Hz (1000 = 0x03E8)
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
void setup_timer2(void)
{
    TIM2_PSCR    = 0x04;  //  Prescaler = 16
    TIM2_ARRH    = 0x03;  //  High byte of 1000
    TIM2_ARRL    = 0xE8;  //  Low  byte of 1000
    TIM2_IER_UIE = 1;     //  Enable the update interrupts
    TIM2_CR1_CEN = 1;     //  Finally enable the timer
} // setup_timer2()

/*-----------------------------------------------------------------------------
  Purpose  : This routine initialises all the GPIO pins of the STM8 uC.
             See stc1000p.h for a detailed description of all pin-functions.
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
void setup_output_ports(void)
{
  PB_ODR     |=  (I2C_SCL | I2C_SDA); // Must be set here, or I2C will not work
  PB_DDR     |=  (I2C_SCL | I2C_SDA); // Set as outputs
  //PB_CR2     |=  (I2C_SCL | I2C_SDA); // Set speed to 10 MHz
  
  PC_DDR     |= DI_3V3;               // Set DI for WS2812B as output
  PC_CR1     |= DI_3V3;               // Set DI to Push-Pull
  PC_ODR     &= ~DI_3V3;              // Turn off DI
  
  PD_DDR     |= TX;                   // Set UART1-TX as output
  PD_CR1     |= TX;                   // Set UART1-TX to Push-Pull
  PD_ODR     |= TX;                   // Set TX high
  PD_DDR     &= ~RX;                  // Set UART1-RX as input
  PD_CR1     &= ~RX;                  // Set to floating
} // setup_output_ports()

/*-----------------------------------------------------------------------------
  Purpose  : This routine sends one byte to the WS2812B LED-string.
  Variables: bt: the byte to send
  Returns  : -
  ---------------------------------------------------------------------------*/
void ws2812b_send_byte(uint8_t bt)
{
    uint8_t i,x = 0x80; // Start with MSB first
    
    for (i = 0; i < 8; i++)
    {
        if (bt & x)
        {    // Send a 1   
             ws2812b_send_1;
        } // if
        else 
        {   // Send a 0
            ws2812b_send_0;
        } // else
        x >>= 1; // Next bit
    } // for i
} // ws2812b_send_byte()

/*-----------------------------------------------------------------------------
  Purpose  : This routine clears all WS2812B LEDs.
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
void clear_all_leds(void)
{
    uint8_t i;

    for (i = 0; i < NR_LEDS; i++)
    {
        led_g[i] = led_r[i] = led_b[i] = 0x00;
    } // for i
 } // clear_all_leds()

/*-----------------------------------------------------------------------------
  Purpose  : This routine sends a test pattern to all WS2812B LEDs.
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
void test_pattern(void)
{
    static uint8_t cntr_b = 0, tmr_b = 0;
    uint8_t i;

    if (++tmr_b >= 20)
    {
        tmr_b = 0;
        switch (cntr_b)
        {
            case 0: 
                for (i = 0; i < NR_LEDS; i++)
                {
                    led_b[i] = led_intensity;
                    led_g[i] = led_r[i] = 0x00;
                } // for
                cntr_b = 1;
                break;
            case 1: 
                for (i = 0; i < NR_LEDS; i++)
                {
                    led_g[i] = led_intensity;
                    led_b[i] = led_r[i] = 0x00;
                } // for
                cntr_b = 2;
                break;
            case 2: 
                for (i = 0; i < NR_LEDS; i++)
                {
                    led_r[i] = led_intensity;
                    led_b[i] = led_g[i] = 0x00;
                } // for
                cntr_b = 0;
                break;
        } // switch
    } // if
} // test_pattern() 

uint8_t get_bottom_led_nr(uint8_t x)
{
    uint8_t seg_nr = x / 12; // segment 0..4
    uint8_t led_nr = x - 12 * seg_nr + 24;
    led_nr += 36 * seg_nr;
    return led_nr;
} // get_bottom_led_nr()

/*-----------------------------------------------------------------------------
  Purpose  : This routine creates a pattern for the LEDs and stores it in
             the arrays led_r, led_g and led_b
  Variables: bt: the byte to send
  Returns  : -
  ---------------------------------------------------------------------------*/
void pattern_task(void)
{
    const uint8_t hr[12]    = {0,5,10,39,44,73,78,83,112,117,146,151};
    const uint8_t sl[7]     = {74,75,76,77,78,88,100};
    const uint8_t slidx[10] = {0,1,2,3,4,4,3,2,1,0};
    
    uint8_t lred; // AM: green, PM: yellow
    uint8_t i,h,bsec,esec,bmin,emin,bhr,ehr,sli;
    uint8_t wli = led_intensity >> 1;
    
    if (enable_test_pattern)
    {   // WS2812 test-pattern
        test_pattern();
    } // if
    else
    {
        for (i = 0; i < NR_LEDS; i++)
        {
            led_g[i] = led_r[i] = led_b[i] = 0;
        } // for i
        
        if (dt.sec == 0)
        {
            check_and_set_summertime(); // check summertime change every minute
            bsec = 0;
            esec = 59;
        } // if
        else
        {
            bsec = 1;
            esec = dt.sec;
        } // else
        if (blanking_active()) return;
        if (dt.min == 0)
        {
            bmin = 0;
            emin = 59;
        } // if
        else
        {
            bmin = 1;
            emin = dt.min;
        } // else
        for (i = bsec; i <= esec; i++)
        {
            led_b[get_bottom_led_nr(i)] = ((i % 5) ? led_intensity : (led_intensity << 3));
        } // for i
        for (i = bmin; i <= emin; i++)
        {
            led_g[get_bottom_led_nr(i)-12] = ((i % 5) ? led_intensity : (led_intensity << 3));
        } // for i
        if (dt.hour >= 12)
        {
             h    = dt.hour - 12;
             lred = wli; // yellow color for PM
             //lred = 0x00;
        } // if
        else 
        {
            h    = dt.hour;
            lred = 0x00; // only green for AM
        } // else
        if (h == 0)
        {
            bhr = 0;
            ehr = 11;
        } // if
        else
        {
            bhr = 1;
            ehr = h;
        } // else
        for (i = bhr; i <= ehr; i++)
        {
            led_r[hr[i]] = led_intensity;
            led_g[hr[i]] = lred;
        } // for i

        sli = slidx[slpos];
        if (++slpos > 9) slpos = 0;
        for (i = 0; i < 7; i++)
        {
            led_g[sl[i]+sli] |= wli;
            led_r[sl[i]+sli] |= wli;
            led_b[sl[i]+sli] |= wli;
        } // for i
    } // else
} // pattern_task()    
        
/*-----------------------------------------------------------------------------
  Purpose  : This routine sends the RGB-bytes for every LED to the WS2812B
             LED string.
  Variables: 
      led_g: the green byte array
      led_r: the red byte array
      led_b: the blue byte array
  Returns  : -
  ---------------------------------------------------------------------------*/
void ws2812_task(void)
{
    uint8_t i;

    __disable_interrupt(); // disable IRQ for time-sensitive LED-timing
    for (i = 0; i < NR_LEDS; i++)
    {
        ws2812b_send_byte(led_g[i]); // Send one byte of Green
        ws2812b_send_byte(led_r[i]); // Send one byte of Red
        ws2812b_send_byte(led_b[i]); // Send one byte of Blue
    } // for i
    __enable_interrupt(); // enable IRQ again
    if (!watchdog_test)   // only refresh when watchdog_test == 0 (X0 command)
        IWDG_KR = IWDG_KR_KEY_REFRESH;   // Refresh watchdog (reset after 500 msec.)
} // ws2812_task()

/*------------------------------------------------------------------------
Purpose  : This task is called every minute by pattern_task(). It checks 
           for a change from summer- to wintertime and vice-versa.
           To start DST: Find the last Sunday in March  : @2 AM advance clock to 3 AM.
           To stop DST : Find the last Sunday in October: @3 AM set clock back to 2 AM (only once!).
Variables: p: pointer to time-struct
Returns  : -
------------------------------------------------------------------------*/
void check_and_set_summertime(void)
{
    uint8_t        hr,day,lsun03,lsun10,dst_eep;
    static uint8_t advance_time = 0;
    static uint8_t revert_time  = 0;
#ifdef DEBUG_SENSORS
    char           s[20];
#endif
    
    if (dt.mon == 3)
    {
        day    = ds3231_calc_dow(31,3,dt.year); // Find day-of-week for March 31th
        lsun03 = 31 - (day % 7);                // Find last Sunday in March
#ifdef DEBUG_SENSORS
        sprintf(s,"lsun03=%d\n",lsun03); 
        uart_printf(s);
#endif
        switch (advance_time)
        {
        case 0: if ((dt.day == lsun03) && (dt.hour == 2) && (dt.min == 0))
                {   // At 2:00 AM advance time to 3 AM, check for one minute
                    advance_time = 1;
                } // if
                else if (dt.day < lsun03) dst_active = false;
                else if (dt.day > lsun03) dst_active = true;
                else if (dt.hour < 2)     dst_active = false;
                break;
        case 1: // Now advance time, do this only once
             ds3231_settime(3,0,dt.sec); // Set time to 3:00, leave secs the same
             advance_time = 2;
             dst_active   = true;
             eeprom_write_config(EEP_ADDR_DST_ACTIVE,0x01); // set DST in eeprom
             break;
        case 2: 
             if (dt.min > 0) advance_time = 0; // At 3:01:00 back to normal
             dst_active = true;
        break;
        } // switch
    } // if
    else if (dt.mon == 10)
    {
        day    = ds3231_calc_dow(31,10,dt.year); // Find day-of-week for October 31th
        lsun10 = 31 - (day % 7);                 // Find last Sunday in October
#ifdef DEBUG_SENSORS
        sprintf(s,"lsun10=%d\n",lsun10); 
        uart_printf(s);
#endif
        switch (revert_time)
        {
        case 0: if ((dt.day == lsun10) && (dt.hour == 3) && (dt.min == 0))
        {   // At 3:00 AM revert time back to 2 AM, check for one minute
            revert_time = 1;
        } // if
        else if (dt.day > lsun10) dst_active = false;
        else if (dt.day < lsun10) dst_active = true;
        else if (dt.hour < 3)     dst_active = true;
        break;
        case 1: // Now revert time, do this only once
            ds3231_settime(2,0,dt.sec); // Set time back to 2:00, leave secs the same
            revert_time = 2;
            dst_active  = false;
            eeprom_write_config(EEP_ADDR_DST_ACTIVE,0x00); // reset DST in eeprom
            break;
        case 2: // make sure we passed 3 AM in order to prevent multiple reverts
            if (dt.hour > 3) revert_time = 0; // at 4:00:00 back to normal
            dst_active = false;
            break;
        } // switch
    } // else if
    else if ((dt.mon < 3) || (dt.mon > 10)) dst_active = false;
    else                                    dst_active = true;

    //------------------------------------------------------------------------
    // If, for some reason, the clock was powered-off during the change to
    // summer- or winter-time, the eeprom value differs from the actual 
    // dst_active value. If so, set the actual sommer- and winter-time.
    //------------------------------------------------------------------------
    dst_eep = (uint8_t)eeprom_read_config(EEP_ADDR_DST_ACTIVE);
    if (dst_active && !dst_eep)
    {   // It is summer-time, but clock has not been advanced yet
        hr = (dt.hour >= 23) ? 0 : dt.hour + 1;
        ds3231_settime(hr,dt.min,dt.sec); // Set summer-time to 1 hour later
        eeprom_write_config(EEP_ADDR_DST_ACTIVE,0x01); // set DST in eeprom
    } // if
    else if (!dst_active && dst_eep)
    {   // It is winter-time, but clock has not been moved back yet
        hr = (dt.hour > 0) ? dt.hour - 1 : 23;
        ds3231_settime(hr,dt.min,dt.sec); // Set summer-time to 1 hour earlier
        eeprom_write_config(EEP_ADDR_DST_ACTIVE,0x00); // set DST in eeprom
    } // if
} // check_and_set_summertime()

/*-----------------------------------------------------------------------------
  Purpose  : This routine retrieves time and date info from the RTC. 
             It is called every second by the scheduler.
  Variables: 
    seconds: global variable [0..59]
    minutes: global variable [0..59]
      hours: global variable [0..23]
  Returns  : -
  ---------------------------------------------------------------------------*/
void clock_task(void)
{
    ds3231_gettime(&dt); // Update td struct every second
} // clock_task()

/*-----------------------------------------------------------------------------
  Purpose  : This routine prints the actual date and time to the UART.
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
void print_date_and_time(void)
{
    char s2[40]; // Used for printing to RS232 port
    uart_printf("DS3231: ");
    check_and_set_summertime();
    sprintf(s2," %d-%d-%d, %d:%d.%d",
               dt.day , dt.mon, dt.year,
               dt.hour, dt.min, dt.sec);
    uart_printf(s2);
    sprintf(s2," dow:%d, dst:%d, blanking:%d\n",
               dt.dow, dst_active, blanking_active());
    uart_printf(s2);
} // print_date_and_time()

/*------------------------------------------------------------------------
  Purpose  : This function converts hours and minutes to minutes.
  Variables: h  : hours of actual time
             min: minutes of actual time
  Returns  : time in minutes
  ------------------------------------------------------------------------*/
uint16_t cmin(uint8_t h, uint8_t m)
{
    return (uint16_t)h * 60 + m;
} // cmin()

/*------------------------------------------------------------------------
  Purpose  : This function decides if the current time falls between the
             blanking time for the LEDs.
  Variables: -
  Returns  : true: blanking is true, false: no blanking
  ------------------------------------------------------------------------*/
bool blanking_active(void)
{
	uint16_t x = cmin(dt.hour      , dt.min);
	uint16_t b = cmin(blank_begin_h, blank_begin_m);
	uint16_t e = cmin(blank_end_h  , blank_end_m);
	
	// (b > e): Example: 23:30 and 05:30, active if x>=b OR  x<=e
	// (b < e): Example: 02:30 and 05:30, active if x>=b AND x<=e
	return ((b > e) && ((x >= b) || (x <= e))) || 
               ((b < e) && ((x >= b) && (x < e))); 
} // blanking_active()

/*-----------------------------------------------------------------------------
  Purpose: interpret commands which are received via the USB serial terminal:
  Variables: 
          s: the string that contains the command from RS232 serial port 0
  Returns  : -
  ---------------------------------------------------------------------------*/
void execute_single_command(char *s)
{
   uint8_t  num  = atoi(&s[1]); // convert number in command (until space is found)
   char     s2[40]; // Used for printing to RS232 port
   char     *s1;
   uint8_t  d,m,h,sec;
   uint16_t i,y;
   int16_t  temp;
   const char sep[] = ":-.";
   
   switch (s[0])
   {
	case 'd': // 0 = Set Date, 1 = Get Date
		 switch (num)
		 {
                    case 0: // Set Date
			    s1 = strtok(&s[3],sep);
                            d  = atoi(s1);
                            s1 = strtok(NULL ,sep);
                            m  = atoi(s1);
                            s1 = strtok(NULL ,sep);
                            y  = atoi(s1);
                            sprintf(s2,"Date: %d-%d-%d\n",d,m,y);
                            uart_printf(s2);
                            ds3231_setdate(d,m,y); // write to DS3231 IC
                            break;
                    case 1: // Set Time
                            s1      = strtok(&s[3],sep);
                            h       = atoi(s1);
                            s1      = strtok(NULL ,sep);
                            m       = atoi(s1);
                            s1      = strtok(NULL ,sep);
                            sec     = atoi(s1);
                            sprintf(s2,"Time: %d:%d:%d\n",h,m,sec);
                            uart_printf(s2);
                            ds3231_settime(h,m,sec); // write to DS3231 IC
                            break;
                    case 2: print_date_and_time();
                            sprintf(s2,"Blanking: %d:%d - %d:%d\n",
                                       blank_begin_h, blank_begin_m,
                                       blank_end_h  , blank_end_m);
                            uart_printf(s2);
                            break;
                    case 3: // Get Temperature
                            temp = ds3231_gettemp();
                            sprintf(s2,"DS3231: %d.",temp>>2);
                            uart_printf(s2);
                            switch (temp & 0x03)
                            {
				case 0: uart_printf("00 °C\n"); break;
				case 1: uart_printf("25 °C\n"); break;
				case 2: uart_printf("50 °C\n"); break;
				case 3: uart_printf("75 °C\n"); break;
                            } // switch
                            break;
                    case 4: // Set Start-Time for blanking Nixies
                            s1 = strtok(&s[3],sep);
                            h  = atoi(s1);
                            s1 = strtok(NULL ,sep);
                            m  = atoi(s1);
                            if ((h < 24) && (m < 60))
                            {
                                blank_begin_h = h;
                                blank_begin_m = m;
                                eeprom_write_config(EEP_ADDR_BBEGIN_H,blank_begin_h);
                                eeprom_write_config(EEP_ADDR_BBEGIN_M,blank_begin_m);
                            } // if
                            break;
                                    
                    case 5: // Set End-Time for blanking Nixies
                            s1 = strtok(&s[3],sep);
                            h  = atoi(s1);
                            s1 = strtok(NULL ,sep);
                            m  = atoi(s1);
                            if ((h < 24) && (m < 60))
                            {
                                blank_end_h = h;
                                blank_end_m = m;
                                eeprom_write_config(EEP_ADDR_BEND_H,blank_end_h);
                                eeprom_write_config(EEP_ADDR_BEND_M,blank_end_m);
                            } // if
                            break;		 
							 
                   default: break;
                 } // switch
                 break;
  
	case 'i': // Set intensity of WS2812 LEDs between 1..255
		 if (num > 0)
                 {
                     led_intensity = num;
                     eeprom_write_config(EEP_ADDR_INTENSITY,led_intensity);
                 } // if
		 break;

        case 's': // System commands
		 switch (num)
		 {
                    case 0: // revision
                            uart_printf(ring_clk_ver);
                            break;
                    case 1: // List all tasks
                            list_all_tasks(); 
                            break;
                    case 2: // I2C-scan
			    uart_printf("I2C-scan: ");
			    for (i = 0x02; i < 0xff; i+=2)
			    {
				if (i2c_start_bb(i) == I2C_ACK)
				{
                                    sprintf(s2,"0x%x, ",i);
		  		    uart_printf(s2);
				} // if
				i2c_stop_bb();
			    } // for
			    uart_putc('\r');
                            break;
		   default: break;
                 } // switch
		 break;
                                        
	case 'w': // WS2812 test-pattern command
		 enable_test_pattern = num; // 1 = enable test-pattern
                 if (!num)
                 {  // clear all leds when finished with test-pattern
                    clear_all_leds();
                 } // if
		 break;
        default: break;
   } // switch
} // execute_single_command()

/*-----------------------------------------------------------------------------
  Purpose  : Non-blocking RS232 command-handler via the USB port
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
void rs232_command_handler(void)
{
  char    ch;
  static uint8_t cmd_rcvd = 0;
  
  if (!cmd_rcvd && uart_kbhit())
  { // A new character has been received
    ch = tolower(uart_getc()); // get character as lowercase
    switch (ch)
	{
            case 255 : uart_putc('.');
            case '\n': break;
            case '\r': cmd_rcvd  = 1;
		       rs232_inbuf[rs232_ptr] = '\0';
		       rs232_ptr = 0;
                       uart_putc('\n');
                       break;
            default  : if (rs232_ptr < UART_BUFLEN)
                       {   
                           rs232_inbuf[rs232_ptr++] = ch;
                           uart_putc(ch);
                       }
                       else rs232_ptr = 0;
                       break;
	} // switch
  } // if
  if (cmd_rcvd)
  {
	  cmd_rcvd = 0;
	  execute_single_command(rs232_inbuf);
  } // if
} // rs232_command_handler()

/*-----------------------------------------------------------------------------
  Purpose  : This functions initializes the independent watchdog (IWDG) and 
             sets the watchdog timeout to the maximum of T = 512 msec.
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
void init_watchdog(void)
{
	IWDG_KR  = IWDG_KR_KEY_ENABLE;  // start the IWDG
	IWDG_KR  = IWDG_KR_KEY_ACCESS;  // enable access to IWDG_PR and IWDG_RLR registers
	IWDG_PR  = 0x05;                // prescaler divider 128
	IWDG_RLR = 0xFF;	        // Reload register to maximum
	IWDG_KR  = IWDG_KR_KEY_REFRESH; // reset the IWDG
} // init_watchdog()

/*-----------------------------------------------------------------------------
  Purpose  : This is the main entry-point for the program
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
int main(void)
{
    uint8_t i2c_err;

    __disable_interrupt();
    initialise_system_clock(); // Set system-clock to 16 MHz
    setup_output_ports();      // Init. needed output-ports for LED and keys
    setup_timer2();            // Set Timer 2 to 1 kHz
    i2c_err = i2c_reset_bus(); // Init. I2C-peripheral
    uart_init();               // Init. UART-peripheral
    uart_printf(ring_clk_ver);
    uart_printf("i2c_reset_bus:");
    uart_putc(0x30+i2c_err);
    uart_putc('\n');
    led_intensity = (uint8_t)eeprom_read_config(EEP_ADDR_INTENSITY);
    if (!led_intensity)
    {   // First time power-up: eeprom value is 0x00
        led_intensity = LED_INTENSITY; // initial value
        eeprom_write_config(EEP_ADDR_INTENSITY,led_intensity);
        eeprom_write_config(EEP_ADDR_BBEGIN_H,blank_begin_h);
        eeprom_write_config(EEP_ADDR_BBEGIN_M,blank_begin_m);
        eeprom_write_config(EEP_ADDR_BEND_H  ,blank_end_h);
        eeprom_write_config(EEP_ADDR_BEND_M  ,blank_end_m);
    } // if
    blank_begin_h = (uint8_t)eeprom_read_config(EEP_ADDR_BBEGIN_H);
    blank_begin_m = (uint8_t)eeprom_read_config(EEP_ADDR_BBEGIN_M);
    blank_end_h   = (uint8_t)eeprom_read_config(EEP_ADDR_BEND_H);
    blank_end_m   = (uint8_t)eeprom_read_config(EEP_ADDR_BEND_M);
    ds3231_gettime(&dt);   // Read time from DS3231 RTC
    print_date_and_time(); // and output to UART   
    
    // Initialise all tasks for the scheduler
    scheduler_init();                          // clear task_list struct
    add_task(clock_task  , "CLK"   ,10, 1000); // every second
    add_task(pattern_task, "PTRN"  ,40,  100); // every 100 msec.
    add_task(ws2812_task , "WS2812",70,  100); // every 100 msec.
    init_watchdog();                           // init. the IWDG watchdog
    __enable_interrupt();

    while (1)
    {   // background-processes
        dispatch_tasks();        // Run task-scheduler()
        rs232_command_handler(); // run command handler continuously
    } // while
} // main()
