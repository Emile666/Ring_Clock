#include "ring_clock.h"
#include "delay.h"
#include "scheduler.h"
#include "i2c.h"
#include "uart.h"

extern char     rs232_inbuf[];  // RS232 input buffer
extern uint8_t  rs232_ptr;      // index in RS232 buffer
extern uint32_t t2_millis;      // Updated in TMR2 interrupt

uint8_t led_r[NR_LEDS]; // Array with 8-bit red colour for all WS2812
uint8_t led_g[NR_LEDS]; // Array with 8-bit green colour for all WS2812
uint8_t led_b[NR_LEDS]; // Array with 8-bit blue colour for all WS2812
uint8_t seconds = 0;
uint8_t minutes = 0;
uint8_t hours   = 0;
uint8_t slpos   = 0; // position of sling

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
                    led_b[i] = 0x10;
                    led_g[i] = 0x00;
                    led_r[i] = 0x00;
                } // for
                cntr_b = 1;
                break;
            case 1: 
                for (i = 0; i < NR_LEDS; i++)
                {
                    led_b[i] = 0x00;
                    led_g[i] = 0x10;
                    led_r[i] = 0x00;
                } // for
                cntr_b = 2;
                break;
            case 2: 
                for (i = 0; i < NR_LEDS; i++)
                {
                    led_b[i] = 0x00;
                    led_g[i] = 0x00;
                    led_r[i] = 0x10;
                } // for
                cntr_b = 0;
                break;
        } // switch
    } // if
    
} // test_pattern()

#define SEC_TO_LED_NR(x) (((x/12)*36)+x)

uint8_t get_bottom_led_nr(uint8_t x)
{
    uint8_t seg_nr = x / 12; // segment 0..4
    uint8_t led_nr = x - 12 * seg_nr + 24;
    led_nr += 36 * seg_nr;
    return led_nr;
} // get_top_led_nr()

/*-----------------------------------------------------------------------------
  Purpose  : This routine creates a pattern for the LEDs and stores it in
             the arrays led_r, led_g and led_b
  Variables: bt: the byte to send
  Returns  : -
  ---------------------------------------------------------------------------*/
void pattern_task(void)
{
    uint8_t hr[12]    = {0,5,10,39,44,73,78,83,112,117,146,151};
    uint8_t sl[7]     = {74,75,76,77,78,88,100};
    uint8_t slidx[10] = {0,1,2,3,4,4,3,2,1,0};
    
    uint8_t lred; // AM: green, PM: yellow
    uint8_t i,h,bsec,esec,bmin,emin,sli;
    
    //test_pattern();
    for (i = 0; i < NR_LEDS; i++)
    {
        led_g[i] = led_r[i] = led_b[i] = 0;
    } // for i
 
    if (seconds == 0)
    {
        bsec = 0;
        esec = 59;
    } // if
    else
    {
        bsec = 1;
        esec = seconds;
    } // else
    if (minutes == 0)
    {
        bmin = 0;
        emin = 59;
    } // if
    else
    {
        bmin = 1;
        emin = minutes;
    } // else
        
    for (i = bsec; i <= esec; i++)
    {
        led_r[get_bottom_led_nr(i)] = ((i % 5) ? 0x08 : 0x40);
    } // for i
    for (i = bmin; i <= emin; i++)
    {
        led_b[get_bottom_led_nr(i)-12] = ((i % 5) ? 0x08 : 0x40);
    } // for i
    if (hours > 12)
    {
         h    = hours - 12;
         lred = 0x10; // yellow color for PM
    } // if
    else 
    {
        h    = hours;
        lred = 0x00; // only green for AM
    } // else
    for (i = ((h == 12) ? 0 : 1); i <= h; i++)
    {
        led_g[hr[i]] = 0x10;
        led_r[hr[i]] = lred;
    } // for i

    sli = slidx[slpos];
    if (++slpos > 9) slpos = 0;
    for (i = 0; i < 7; i++)
    {
        led_g[sl[i]+sli] |= 0x08;
        led_r[sl[i]+sli] |= 0x08;
        led_b[sl[i]+sli] |= 0x08;
    } // for i
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
} // ws2812_task()

/*-----------------------------------------------------------------------------
  Purpose  : This routine calculates seconds, minutes and hours
  Variables: 
    seconds: global variable [0..59]
    minutes: global variable [0..59]
      hours: global variable [0..23]
  Returns  : -
  ---------------------------------------------------------------------------*/
void clock_task(void)
{
    Time p;

    ds3231_gettime(&p);
    __disable_interrupt();
    seconds = p.sec;
    minutes = p.min;
    hours   = p.hour;
    __enable_interrupt();
} // clock_task()

void print_date_and_time(void)
{
    char s2[40]; // Used for printing to RS232 port
    Time p;
    
    ds3231_gettime(&p);
    uart_printf("DS3231: ");
    sprintf(s2," %d-%d-%d, %d:%d.%d\n",p.date,p.mon,p.year,p.hour,p.min,p.sec);
    uart_printf(s2);
} // print_date_and_time()

void print_dow(uint8_t dow)
{
    char day[8][4] = {"???","Mon","Tue","Wed","Thu","Fri","Sat","Sun"};

    uart_printf(day[dow]);
} // print_dow()

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
   uint8_t  d,m;
   uint16_t i,y;
   int16_t  temp;
   
   switch (s[0])
   {
	case 'd': // 0 = Set Date, 1 = Get Date
		 switch (num)
		 {
                    case 0: // Set Date
			    s1 = strtok(&s[3],":-");
                            d  = atoi(s1);
                            s1 = strtok(NULL ,":-");
                            m  = atoi(s1);
                            s1 = strtok(NULL ,":-");
                            y  = atoi(s1);
                            uart_printf("Date: ");
                            print_dow(ds3231_calc_dow(d,m,y));
                            sprintf(s2,", %d-%d-%d\n",d,m,y);
                            uart_printf(s2);
                            ds3231_setdate(d,m,y); // write to DS3231 IC
                            break;
                    case 1: print_date_and_time();
                            break;
                   default: uart_printf("Usage: d0 = Set Date dd-mm-yyyy\n");
                            uart_printf("d1 = Get Date and Time\n");
                            break;
                 } // switch
                 break;
  
	case 's': // System commands
		 switch (num)
		 {
                    case 0: // revision
                            uart_printf("Ring-Clock V0.1\n");
                            break;
                    case 1: // List all tasks
                            list_all_tasks(); 
                            break;
                    case 2: // I2C-scan
			    uart_printf("I2C-scan: ");
			    for (i = 0x02; i < 0xff; i+=2)
			    {
				if (i2c_start(i) == I2C_ACK)
				{
					sprintf(s2,"0x%x, ",i);
		  		    	uart_printf(s2);
				} // if
				i2c_stop();
			    } // for
			    uart_putc('\r');
                            break;

		   default: uart_printf("s0 = Revision number\n");
                            uart_printf("s1 = List all Tasks\n");
                            uart_printf("s2 = I2C-scan\n");
                            break;
                 } // switch
		 break;
                                        
        case 't': // 0 = Set Time, 1 = Get Time, 2 = Get Temperature
		 switch (num)
		 {
                    case 0: // Set Time
                            s1      = strtok(&s[3],":-.");
                            hours   = atoi(s1);
                            s1      = strtok(NULL ,":-.");
                            minutes = atoi(s1);
                            s1      = strtok(NULL ,":-.");
                            seconds = atoi(s1);
                            sprintf(s2,"Time: %d:%d:%d\n",hours,minutes,seconds);
                            uart_printf(s2);
                            ds3231_settime(hours,minutes,seconds); // write to DS3231 IC
                            break;
                    case 1: print_date_and_time();
                            break;
                    case 2: // Get Temperature
                            temp = ds3231_gettemp();
                            sprintf(s2,"DS3231: %d.",temp>>2);
                            uart_printf(s2);
                            switch (temp & 0x03)
                            {
				case 0: uart_printf("00 �C\n"); break;
				case 1: uart_printf("25 �C\n"); break;
				case 2: uart_printf("50 �C\n"); break;
				case 3: uart_printf("75 �C\n"); break;
                            } // switch
                            break;
                   default: uart_printf("t0 = Set Time hh:mm:ss\n");
                            uart_printf("t1 = Get Date and Time\n");
                            uart_printf("t2 = Get Temperature\n");
                            break;
                 } // switch
                 break;
	   
        default: uart_printf("Possible commands: d0 dd-mm-yyyy, d1\n");
                 uart_printf("s0, s1, s2, t0 hh:mm:ss, t1, t2\n");
	         break;

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
  Purpose  : This is the main entry-point for the program
  Variables: -
  Returns  : -
  ---------------------------------------------------------------------------*/
int main(void)
{
    __disable_interrupt();
    initialise_system_clock(); // Set system-clock to 16 MHz
    setup_output_ports();      // Init. needed output-ports for LED and keys
    setup_timer2();            // Set Timer 2 to 1 kHz
    i2c_init(0);               // Init. I2C-peripheral
    uart_init();               // Init. UART-peripheral
    uart_printf("ring-clock v01\n");
    
    // Initialise all tasks for the scheduler
    scheduler_init();                      // clear task_list struct
    add_task(pattern_task, "PTRN"  , 0,  100); // every 100 msec.
    add_task(ws2812_task , "WS2812",50,  100); // every 100 msec.
    add_task(clock_task  , "CLK"   ,80, 1000); // every second
    __enable_interrupt();

    while (1)
    {   // background-processes
        dispatch_tasks();        // Run task-scheduler()
        rs232_command_handler(); // run command handler continuously
        __wait_for_interrupt();  // do nothing
    } // while
} // main()
