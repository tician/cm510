/**
 *  @file
 *  @author  Matthew Paulishen <ticiane1@uga.edu>
 *  @version v2012.02.17.1145
 *
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *  @section LICENSE
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *  Just to be safe:
 *   'Dynamixel' is property of Robotis, Inc.
 *      <http://www.robotis.com>
 *
 *
 *  Copyright (c) 2011, 2012 Matthew Paulishen. All rights reserved.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *  @section DESCRIPTION
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *  -A header file of questionable quality for use with the CM-510 from Robotis.
 *    A bit of a smorgasbord of code intended to keep user's code cleaner.  It
 *    is cm530.h backported to the CM-510.
 *
 *  -The Dynamixel and Zigbee libraries are based primarily on the examples
 *    from Robotis, but have been modified a bit (renamed/removed variables,
 *    switched to ring buffers, added more descriptive error output for
 *    dynamixel packets (16-bit maskable error variable), etc.).
 *
 *  -The PC UART library is partly based on the Robotis library, but variables
 *    were changed to achieve uniformity with Dxl and Zig libraries as well
 *    as switching to a ring buffer.  Other print/get functions were created
 *    to work around a problem with WinARM's eabi-arm libraries not linking (no
 *    stdio.h accessible).
 *
 *  FUNCTIONS:
 *    SetLED(LED_t led, uint8_t state);
 *        // Control LEDs of CM-530
 *        //   {MANAGE, PROGRAM, PLAY, TXD, RXD, AUX, POWER}
 *    ReadButton(Button_t button);
 *        // Read User Buttons and Microphone input of CM-530
 *        //   {UP, DOWN, LEFT, RIGHT, START, MIC}
 *    Buzzed(uint32_t mlength, uint32_t tone);
 *        // Control Buzzer of CM-530
 *        //   mlength is the length in [ms] to play the tone
 *        //   tone is the delay in [us] to produce a 50% duty cycle
 *        //     (Buzzer_ON -> uDelay(tone) -> Buzzer_OFF -> uDelay(tone)
 *    PlayNote(uint32_t mlength, buzzed_note_t note, uint8_t octave);
 *        // Play a note (12-TET. 12 notes from A flat to G sharp.)
 *        //   mlength is the length in [ms] to play the note
 *        //   note is the note to play
 *        //     (C, C#, Db, D, D#, Eb, E, F, F#, Gb, G, G#, Ab, A, A#, Bb, B)
 *        //   octave indicates the octave where the note is to be played
 *        //     To play Middle C (C_4) for 10 seconds:
 *        //       PlayNote(10000, NOTE_C, 4);
 *    ReadIR(EPortA_t port);
 *        // Control a Bioloid IR module easily (not the SHARP module used in chest)
 *    SetEPort(EPortD_t pin, uint8_t state);
 *        // Control Pins 1 and 5 of the 6 External Ports
 *    ReadAnalog(EPortA_t port);
 *        // Read Pin 3 (analog) of the 6 External Ports or Battery Voltage (VBUS)
 *    StartCountdown(uint32_t nTime);
 *        // Start a variable (glCountdownCounter) to countdown in 1 [ms]
 *        //   intervals without stopping code execution
 *    mDelay(uint32_t nTime);
 *        // Delay code execution for nTime in [ms]
 *    uDelay(uint32_t nTime);
 *        // Delay code execution for nTime in [us]
 *        //   The default interval is 10 [us] between SysTick() decrements, so
 *        //   will actually round to nearest 10 [us] (can be changed, but may
 *        //   introduce bugs)
 *
 *  KNOWN ISSUES:
 *    -There may still be a bug somewhere in the revised dynamixel library that
 *      had been causing a few problems with servos not responding immediately
 *      or not being detected during initial startup (simply not found in the 
 *      bus scan or having an improper Model number).
 *      It appears to be fixed since adding a function to clear the received
 *      packet array, adding a small delay between calls to dxl_rxpacket(),
 *      and adding a check for the DXL_RXSUCCESS status bit for all read based
 *      high-level functions.
 *  
 *  TODO:
 *    -Test the ZigBee library.
 *    -Verify the reboot to bootloader commands.
 *    -Add documentation using Doxygen?
 *
 *~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */


#include "cm510.h"

#ifdef USING_DYNAMIXEL
uint32_t Baudrate_DXL = 1000000;
#endif
#ifdef USING_ZIGBEE
uint32_t Baudrate_ZIG = 57600;
#endif
#ifdef USING_PC_UART
uint32_t Baudrate_PCU = 57600;
#endif

// If active, will cause AVR to reboot into the bootloader after receiving
//   'n' sequential '#' characters over either the Pcu or Zig UART (n==15).
// Know the command to cause a Watchdog Reset, but do not want to attempt
//   without confirmation from Robotis (if bootloader does not clear the
//   WDT timer, it may get stuck in an endless reset loop).
//#define USING_BREAK_TO_BOOTLOADER


// Select which time interval to call SysTick() interrupt
//   Using "USING_SYSTICK_1US" calls interrupt every 16 cycles, so increased
//   accuracy/resolution for Dxl/Pcu timeouts might not be worth the lost cycles
//   due to handling the interrupt.  i.e. 10 [us] interval works fine and will
//   round to nearest 10 [us].
//#define USING_SYSTICK_100US
#define USING_SYSTICK_10US
//#define USING_SYSTICK_1US

#define DEBUG_PRINT_VOLTAGE


//##############################################################################
//##############################################################################
// All CM-510 PORT and PIN definitions
//##############################################################################
//##############################################################################
// Button/Mic PORT and PIN definitions
#define PORT_SW_UP                      5//PORTE
#define PORT_SW_DOWN                    5//PORTE
#define PORT_SW_RIGHT                   5//PORTE
#define PORT_SW_LEFT                    5//PORTE
#define PORT_SW_START                   4//PORTD
#define PORT_MIC                        4//PORTD
#define PIN_SW_UP                       0x10
#define PIN_SW_DOWN                     0x20
#define PIN_SW_RIGHT                    0x80
#define PIN_SW_LEFT                     0x40
#define PIN_SW_START                    0x01
#define PIN_MIC                         0x02

// LED PORT and PIN definitions
#define PORT_LED_POWER                  3//PORTC
#define PORT_LED_MANAGE                 3//PORTC
#define PORT_LED_PROGRAM                3//PORTC
#define PORT_LED_PLAY                   3//PORTC
#define PORT_LED_TXD                    3//PORTC
#define PORT_LED_RXD                    3//PORTC
#define PORT_LED_AUX                    3//PORTC
#define PIN_LED_POWER                   0x01
#define PIN_LED_MANAGE                  0x10
#define PIN_LED_PROGRAM                 0x20
#define PIN_LED_PLAY                    0x40
#define PIN_LED_TXD                     0x02
#define PIN_LED_RXD                     0x04
#define PIN_LED_AUX                     0x08

// Buzzer PORT and PIN definitions
#define PORT_BUZZER                     2//PORTB
#define PIN_BUZZER                      0x20

// ADC select PORT and PIN definitions
//#define PORT_ADC_SELECT0                GPIOC
//#define PORT_ADC_SELECT1                GPIOC
//#define PIN_ADC_SELECT0                 GPIO_Pin_1
//#define PIN_ADC_SELECT1                 GPIO_Pin_2
//#define PIN_ADC0                        GPIO_Pin_0
//#define PIN_ADC1                        GPIO_Pin_5
//#define PIN_VDD_VOLT                    GPIO_Pin_3

// Ollo port PORT and PIN definitions
#define PORT_SIG_MOT1P                  1//PORTA
#define PORT_SIG_MOT1M                  0//NC
#define PORT_SIG_MOT2P                  1//PORTA
#define PORT_SIG_MOT2M                  0//NC
#define PORT_SIG_MOT3P                  1//PORTA
#define PORT_SIG_MOT3M                  0//NC
#define PORT_SIG_MOT4P                  1//PORTA
#define PORT_SIG_MOT4M                  0//NC
#define PORT_SIG_MOT5P                  1//PORTA
#define PORT_SIG_MOT5M                  0//NC
#define PORT_SIG_MOT6P                  1//PORTA
#define PORT_SIG_MOT6M                  0//NC
#define PIN_SIG_MOT1P                   0x80
#define PIN_SIG_MOT1M                   0
#define PIN_SIG_MOT2P                   0x40
#define PIN_SIG_MOT2M                   0
#define PIN_SIG_MOT3P                   0x20
#define PIN_SIG_MOT3M                   0
#define PIN_SIG_MOT4P                   0x10
#define PIN_SIG_MOT4M                   0
#define PIN_SIG_MOT5P                   0x08
#define PIN_SIG_MOT5M                   0
#define PIN_SIG_MOT6P                   0x04
#define PIN_SIG_MOT6M                   0

// Dynamixel
#define PORT_ENABLE_TXD                 5//PORTE
#define PORT_ENABLE_RXD                 5//PORTE
//#define PORT_DXL_TXD                    
//#define PORT_DXL_RXD                    
#define PIN_ENABLE_TXD                  0x04
#define PIN_ENABLE_RXD                  0x08
//#define PIN_DXL_TXD                     
//#define PIN_DXL_RXD                     

// Zigbee
//#define PORT_ZIGBEE_TXD                 
//#define PORT_ZIGBEE_RXD                 
//#define PORT_ZIGBEE_RESET               
//#define PIN_ZIGBEE_TXD                  
//#define PIN_ZIGBEE_RXD                  
//#define PIN_ZIGBEE_RESET                

// Serial/PC_UART
//#define PORT_PC_TXD                     
//#define PORT_PC_TXD                     
//#define PIN_PC_TXD                      
//#define PIN_PC_RXD                      



//##############################################################################
//##############################################################################
// CM-530 Helper functions - NaN
//##############################################################################
//##############################################################################
// Mic - input - (read(mic, mic) != SET) => heard
    // Pull-up?
    // Differential opamp?
// LED - out - reset(led, led) => ON
    // 5V -> LED -> Resistor -> Pin
// SW  - input - (read(sw, sw) != SET) => pressed
    // Pull-ups
    // 3.3V -> Resistor -> Pin -> GND

//##############################################################################
typedef struct EasyPort_s {
    uint8_t port;
    uint8_t pin;
} EasyPort_t;



EasyPort_t
    EasyButton[6] = {
        {PORT_SW_UP, PIN_SW_UP},
        {PORT_SW_DOWN, PIN_SW_DOWN},
        {PORT_SW_LEFT, PIN_SW_LEFT},
        {PORT_SW_RIGHT, PIN_SW_RIGHT},
        {PORT_SW_START, PIN_SW_START},
        {PORT_MIC, PIN_MIC}
    },
    EasyLED[7] = {
        {PORT_LED_POWER, PIN_LED_POWER},
        {PORT_LED_MANAGE, PIN_LED_MANAGE},
        {PORT_LED_PROGRAM, PIN_LED_PROGRAM},
        {PORT_LED_PLAY, PIN_LED_PLAY},
        {PORT_LED_TXD, PIN_LED_TXD},
        {PORT_LED_RXD, PIN_LED_RXD},
        {PORT_LED_AUX, PIN_LED_AUX}
    },
    EasyEPort[12] = {
        {PORT_SIG_MOT1P, PIN_SIG_MOT1P},
        {PORT_SIG_MOT1M, PIN_SIG_MOT1M},
        {PORT_SIG_MOT2P, PIN_SIG_MOT2P},
        {PORT_SIG_MOT2M, PIN_SIG_MOT2M},
        {PORT_SIG_MOT3P, PIN_SIG_MOT3P},
        {PORT_SIG_MOT3M, PIN_SIG_MOT3M},
        {PORT_SIG_MOT4P, PIN_SIG_MOT4P},
        {PORT_SIG_MOT4M, PIN_SIG_MOT4M},
        {PORT_SIG_MOT5P, PIN_SIG_MOT5P},
        {PORT_SIG_MOT5M, PIN_SIG_MOT5M},
        {PORT_SIG_MOT6P, PIN_SIG_MOT6P},
        {PORT_SIG_MOT6M, PIN_SIG_MOT6M}
    };

//##############################################################################
void SetLED(LED_t led, uint8_t state)
{
    if (state)
       	PORTC &= (~EasyLED[led].pin);
    else
       	PORTC |= EasyLED[led].pin;
}

//##############################################################################
uint8_t ReadButton(Button_t button)
{
    if (EasyButton[button].port==4)
    {
        if (!(PIND&EasyButton[button].pin))
            return 1;
    }
    else if (EasyButton[button].port==5)
    {
        if (!(PINE&EasyButton[button].pin))
            return 1;
    }
    return 0;
}

volatile uint32_t glBuzzerCounter;
void start_countdown_buzzer(uint32_t);
//##############################################################################
void Buzzed(uint32_t mlength, uint32_t tone)
{
    // Twelve-Tone Equal Temperment (12-TET)
    //   1 octave is a doubling of frequency and equal to 1200 cents
    //   1 octave => 12 equally distributed notes (12 intervals/semitones)
    //     so 100 cents per note
    // Tuned to A 440 (440Hz), so 100 cents per note relative to A_5 (440Hz)

    // n [cents] = 1200 log2(b/a)
    // b = a * 2^(n/1200)

    // tone = 1/(2*1e-6*f) = 1/(2*1e-6*440*2^(cents_relative/1200))
    //   using uDelay(), 50% duty cycle, cents relative to A_5

//#define FREQTOTONE(f)    (5000000/f)    // (1/(2*1e-6*f))

    start_countdown_buzzer(mlength);
    while (glBuzzerCounter>0)
    {
        PORTB &= (~PIN_BUZZER);
        uDelay(tone);
        PORTB |= PIN_BUZZER;
        uDelay(tone);
    }
}

//##############################################################################
void PlayNote(uint32_t mlength, buzzed_note_t note, uint8_t octave)
{
    Buzzed(mlength, (uint32_t) (note>>octave));
}

//##############################################################################
void SetEPort(EPortD_t pin, uint8_t state)
{
    if (EasyEPort[pin].port==0)
        return;

    if (state)
        PORTA &= (~EasyEPort[pin].pin);
    else
        PORTA |= EasyEPort[pin].pin;
}

#define ANALOG_RIGHT_BIT_SHIFT          0
//##############################################################################
uint16_t ReadAnalog(EPortA_t port)
{
   	// To which ADC pin is VBUS connected?
    if (port<=VBUS)
    {
        // Set ADC pin
        ADMUX = port+1;
        // Clear ADC Interrupt Flag
        ADCSRA |= (uint8_t) (1 << ADIF);
        // Begin ADC Conversion
        ADCSRA |= (uint8_t) (1 << ADSC);

        // Wait while ADC Interrupt Flag not set
        while(!(ADCSRA & (1 << ADIF)));

        return (uint16_t) ADC>>ANALOG_RIGHT_BIT_SHIFT;
    }
    return 0x8000;
}

//##############################################################################
uint16_t ReadIR(EPortA_t port)
{
    uint16_t temp;

    SetEPort((port*2), 1);
//    SetEPort((port*2)+1, 1);

    uDelay(25);
    temp = ReadAnalog(port);

    SetEPort((port*2), 0);
//    SetEPort((port*2)+1, 1);

    return temp;
}

//##############################################################################
void Battery_Monitor_Alarm(void)
{
    uint16_t volt = ReadAnalog(VBUS)>>4;
#ifdef DEBUG_PRINT_VOLTAGE
    PrintString("\nBattery Voltage: ");
    Printu32d(volt);
    PrintString("e-1 [Volts]\n");
#endif

    // ALARM!!!
    if (volt<VBUS_LOW_LIMIT)
    {
        Buzzed(500,100);
        Buzzed(500,5000);
        Buzzed(500,100);
        Buzzed(500,5000);
        PrintString("\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
        PrintString("Battery Voltage Critical");
        PrintString("\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
    }
    return;
}




//##############################################################################
//##############################################################################
// CM-530 Utility functions
//##############################################################################
//##############################################################################
volatile uint32_t glDelayCounter;
volatile uint32_t glCountdownCounter;
volatile uint8_t glDxlTimeoutCounter;
volatile uint8_t glPcuTimeoutCounter;
//volatile uint32_t glBatTimeoutCounter;
//volatile uint32_t glBatTimeoutSet;
volatile uint8_t gbCounterCount;

void start_timeout_dxl(uint32_t);
void start_timeout_pcu(uint32_t);

enum {SysTick_Counter_Disable = 0, SysTick_Counter_Enable = 1};
void SysTick_CounterCmd(uint8_t);
#ifdef USING_BREAK_TO_BOOTLOADER
void BreakToBootLoader(void);
#endif

//##############################################################################
void mDelay(uint32_t nTime)
{
#ifdef USING_UTIL_DELAY_H
    _delay_ms((double)nTime);
#else
    uDelay(nTime*1000);
#endif
}

//##############################################################################
void uDelay(uint32_t nTime)
{
#ifdef USING_UTIL_DELAY_H
    _delay_us((double)nTime);
#else
    if (glDelayCounter==0)
        gbCounterCount++;

    // Due to SysTick() interrupt, default is using 10 [us] intervals
#ifdef USING_SYSTICK_100US
    if (nTime>=100)
        glDelayCounter = (nTime/100);
    else
        glDelayCounter = 1;
#elif defined USING_SYSTICK_10US
    if (nTime>=10)
        glDelayCounter = (nTime/10);
    else
        glDelayCounter = 1;
#elif defined USING_SYSTICK_1US
    glDelayCounter = (nTime);
#endif

    if (gbCounterCount==1)
    {
        // Enable the SysTick Counter
        SysTick_CounterCmd(SysTick_Counter_Enable);
    }

    while (glDelayCounter!=0);
#endif
}

//##############################################################################
void StartCountdown(uint32_t StartTime)
{
    if (glCountdownCounter==0)
        gbCounterCount++;

    glCountdownCounter = StartTime;
/*
    // Want Timer counting in 1 [ms] intervals
#ifdef USING_SYSTICK_100US
    glCountdownCounter = (StartTime*10);
#elif defined USING_SYSTICK_10US
    glCountdownCounter = (StartTime*100);
#elif defined USING_SYSTICK_1US
    glCountdownCounter = (StartTime*1000);
#endif
*/
    if (gbCounterCount==1)
    {
        // Enable the SysTick Counter
        SysTick_CounterCmd(SysTick_Counter_Enable);
    }

    SetLED(AUX, 1);
}

//##############################################################################
void start_countdown_buzzer(uint32_t nTime)
{
    if (glBuzzerCounter==0)
        gbCounterCount++;

    glBuzzerCounter = nTime;
/*
    // Want Timer counting in 1 [ms] intervals
#ifdef USING_SYSTICK_100US
    glBuzzerCounter = (nTime*10);
#elif defined USING_SYSTICK_10US
    glBuzzerCounter = (nTime*100);
#elif defined USING_SYSTICK_1US
    glBuzzerCounter = (nTime*1000);
#endif
*/
    if (gbCounterCount==1)
    {
        // Enable the SysTick Counter
        SysTick_CounterCmd(SysTick_Counter_Enable);
    }
}


//##############################################################################
void start_timeout_counter(uint32_t nTime)
{
    OCR4AH = ((nTime-1)&0xFF00)>>8;
	OCR4AL = ((nTime-1)&0x00FF);
    TIMSK4 = (0<<OCIE4C) | (0<<OCIE4B) | (1<<OCIE4A) | (0<<TOIE4);
	sei();
}

//##############################################################################
void start_timeout_dxl(uint32_t nTime)
{
    glDxlTimeoutCounter=1;
    start_timeout_counter(nTime*2);
/*
    if (glDxlTimeoutCounter==0)
        gbCounterCount++;

    // Due to SysTick() interrupt, default is using 10 [us] intervals
#ifdef USING_SYSTICK_100US
    if (nTime>=100)
        glDxlTimeoutCounter = (nTime/100);
    else
        glDxlTimeoutCounter = 1;
#elif defined USING_SYSTICK_10US
    if (nTime>=10)
        glDxlTimeoutCounter = (nTime/10);
    else
        glDxlTimeoutCounter = 1;
#elif defined USING_SYSTICK_1US
    glDxlTimeoutCounter = (nTime);
#endif

    if (gbCounterCount==1)
    {
        // Enable the SysTick Counter
        SysTick_CounterCmd(SysTick_Counter_Enable);
    }
*/
}

//##############################################################################
void start_timeout_pcu(uint32_t nTime)
{
    glPcuTimeoutCounter=1;
    start_timeout_counter(nTime*2);
/*
    if (glPcuTimeoutCounter==0)
        gbCounterCount++;

    // Due to SysTick() interrupt, default is using 10 [us] intervals
#ifdef USING_SYSTICK_100US
    if (nTime>=100)
        glPcuTimeoutCounter = (nTime/100);
    else
        glPcuTimeoutCounter = 1;
#elif defined USING_SYSTICK_10US
    if (nTime>=10)
        glPcuTimeoutCounter = (nTime/10);
    else
        glPcuTimeoutCounter = 1;
#elif defined USING_SYSTICK_1US
    glPcuTimeoutCounter = (nTime);
#endif

    if (gbCounterCount==1)
    {
        // Enable the SysTick Counter
        SysTick_CounterCmd(SysTick_Counter_Enable);
    }
*/
}

//void start_timeout_bat(uint32_t nTime)
//{
//    if (glBatTimeoutCounter==0)
//        gbCounterCount++;
//
//    // Want Timer counting in 1 [s] intervals
//#ifdef USING_SYSTICK_100US
//    glBatTimeoutSet = (nTime*10000);
//#elif defined USING_SYSTICK_10US
//    glBatTimeoutSet = (nTime*100000);
//#elif defined USING_SYSTICK_1US
//    glBatTimeoutSet = (nTime*1000000);
//#endif
//
//    glBatTimeoutCounter = glBatTimeoutSet;
//
//    if (gbCounterCount==1)
//    {
//        // Enable the SysTick Counter
//        SysTick_CounterCmd(SysTick_Counter_Enable);
//    }
//}


#ifdef USING_BREAK_TO_BOOTLOADER
//##############################################################################
void BreakToBootLoader(void)
{
}
#endif



#ifdef USING_DYNAMIXEL
//##############################################################################
//##############################################################################
// Dynamixel SDK platform dependent source
//##############################################################################
//##############################################################################
#define DXL_BUFFER_LENGTH               256

static volatile uint16_t gbDxlWrite=0, gbDxlRead=0;
static volatile uint8_t gbpDxlBuffer[DXL_BUFFER_LENGTH];

uint8_t dxl_hal_open(uint32_t);
void dxl_hal_close(void);
void dxl_hal_clear(void);
uint8_t dxl_hal_tx(uint8_t*, uint8_t);
uint8_t dxl_hal_rx(uint8_t*, uint8_t);
void dxl_hal_set_timeout(uint8_t);
uint8_t dxl_hal_timeout(void);
void RxD_DXL_Interrupt(void);

//##############################################################################
uint8_t dxl_hal_open(uint32_t baudrate)
{
    // Double USART Transmission Speed
    // RXD Interupt enable, RXD enable, TXD enable
    // Asynchronous USART, No Parity, One Stop Bit, Eight Data Bits, Rising Edge
    // Baud rate setting
    UCSR0A = (0<<RXC0) | (0<<TXC0) | (0<<UDRE0) | (0<<FE0) | (0<<DOR0) | (0<<UPE0) | (1<<U2X0) | (0<<MPCM0);
    UCSR0B = (1<<RXCIE0) | (0<<TXCIE0) | (0<<UDRIE0) | (1<<RXEN0) | (1<<TXEN0) | (0<<UCSZ02) | (0<<RXB80) | (0<<TXB80);
    UCSR0C = (0<<UMSEL01) | (0<<UMSEL00) | (0<<UPM01) | (0<<UPM00) | (0<<USBS0) | (1<<UCSZ01) | (1<<UCSZ00) | (0<<UCPOL0);
    UBRR0H = ((F_CPU/(baudrate<<3))-1)>>8;
    UBRR0L = ((F_CPU/(baudrate<<3))-1);

	dxl_hal_clear();

    // TX Disable
    PORTE &= ~0x04;
    // RX Enable
    PORTE |= 0x08;

    return 1;
}

//##############################################################################
void dxl_hal_close(void)
{
	// RXD Interupt disable, RXD disable, TXD disable
	UCSR0B = (0<<RXCIE0) | (0<<TXCIE0) | (0<<UDRIE0) | (0<<RXEN0) | (0<<TXEN0) | (0<<UCSZ02) | (0<<RXB80) | (0<<TXB80);

    // TX Disable
    PORTE &= ~0x04;
    // RX Enable
    PORTE |= 0x08;
}

//##############################################################################
void dxl_hal_clear(void)
{
    // Clear communication buffer
    uint16_t i;
    for (i=0; i<DXL_BUFFER_LENGTH; i++)
        gbpDxlBuffer[i] = 0;
    gbDxlRead = 0;
    gbDxlWrite = 0;
}

//##############################################################################
uint8_t dxl_hal_tx(uint8_t *pPacket, uint8_t numPacket)
{
    uint8_t sreg = SREG;
    cli();

    // RX Disable
    PORTE &= ~0x08;
    // TX Enable
    PORTE |= 0x04;

    uint8_t i;
    for (i=0; i<numPacket; i++)
    {
        while ( !(UCSR0A&(1<<UDRE0)) );
        UDR0 = pPacket[i];
    }

    // TX Disable
    PORTE &= ~0x04;
    // RX Enable
    PORTE |= 0x08;

    SREG = sreg;

    return numPacket;
}

//##############################################################################
uint8_t dxl_hal_rx(uint8_t *pPacket, uint8_t numPacket)
{
//    uint8_t sreg = SREG;
//    cli();

    uint8_t i;
    for (i=0; i<numPacket; i++)
    {
        if (gbDxlRead!=gbDxlWrite)
        {
            pPacket[i] = gbpDxlBuffer[gbDxlRead++];
            if (gbDxlRead>(DXL_BUFFER_LENGTH-1))
                gbDxlRead = 0;
        }
        else
            return i;
    }

//    SREG = sreg;

    return numPacket;
}

//##############################################################################
void dxl_hal_set_timeout(uint8_t NumRcvByte)
{
    start_timeout_dxl(NumRcvByte*30);
}

//##############################################################################
uint8_t dxl_hal_timeout(void)
{
    if (glDxlTimeoutCounter==0)
        return 1;
    else
        return 0;
}

//##############################################################################
void RxD_DXL_Interrupt(void)
{
    uint8_t temp;
    temp = UDR0;

    if (gbDxlWrite<(DXL_BUFFER_LENGTH-1))
    {
        gbpDxlBuffer[gbDxlWrite++] = temp;
    }
    else
    {
        gbpDxlBuffer[gbDxlWrite] = temp;
        gbDxlWrite = 0;
    }

    if (gbDxlRead==gbDxlWrite)
        gbDxlRead++;
    if (gbDxlRead>(DXL_BUFFER_LENGTH-1))
        gbDxlRead=0;
}


//##############################################################################
//##############################################################################
// Dynamixel SDK platform independent source
//##############################################################################
//##############################################################################
#define DXL_MAXNUM_TXPARAM                  160
#define DXL_MAXNUM_RXPARAM                  80

static uint8_t gbInstructionPacket[DXL_MAXNUM_TXPARAM] = {0};
static uint8_t gbStatusPacket[DXL_MAXNUM_RXPARAM] = {0};
static uint8_t gbRxPacketLength = 0;
static uint8_t gbRxGetLength = 0;
static volatile uint16_t gbCommStatus = DXL_RXSUCCESS;
static volatile uint8_t giBusUsing = 0;

void dxl_tx_packet(void);
void dxl_rx_packet(void);
void dxl_clear_statpkt(void);

//##############################################################################
uint8_t dxl_initialize(uint32_t baudrate)
{
    if (dxl_hal_open(baudrate)==0)
        return 0;

    gbCommStatus = DXL_RXSUCCESS;
    giBusUsing = 0;

    return 1;
}

//##############################################################################
void dxl_terminate(void)
{
    dxl_hal_close();
}

//##############################################################################
void dxl_tx_packet(void)
{
    uint8_t i;
    uint8_t TxNumByte, RealTxNumByte;
    uint8_t checksum = 0;

    if (giBusUsing==1)
        return;

    giBusUsing = 1;

    gbCommStatus = 0;

    if (gbInstructionPacket[DXL_PKT_LEN]>(DXL_MAXNUM_TXPARAM+2))
    {
        gbCommStatus |= DXL_TXERROR;
        giBusUsing = 0;
        return;
    }

    if (   (gbInstructionPacket[DXL_PKT_INST] != INST_PING)
        && (gbInstructionPacket[DXL_PKT_INST] != INST_READ_DATA)
        && (gbInstructionPacket[DXL_PKT_INST] != INST_WRITE_DATA)
        && (gbInstructionPacket[DXL_PKT_INST] != INST_REG_WRITE)
        && (gbInstructionPacket[DXL_PKT_INST] != INST_ACTION)
        && (gbInstructionPacket[DXL_PKT_INST] != INST_RESET)
        && (gbInstructionPacket[DXL_PKT_INST] != INST_SYNC_WRITE)
        && (gbInstructionPacket[DXL_PKT_INST] != INST_CAP_REGION) )
    {
        gbCommStatus |= DXL_BAD_INST;
        giBusUsing = 0;
        return;
    }

    gbInstructionPacket[0] = 0xFF;
    gbInstructionPacket[1] = 0xFF;
    for (i=0; i<(gbInstructionPacket[DXL_PKT_LEN]+1); i++)
        checksum += gbInstructionPacket[i+2];
    gbInstructionPacket[gbInstructionPacket[DXL_PKT_LEN]+3] = ~checksum;

    if (gbCommStatus&(DXL_RXFAIL | DXL_RXTIMEOUT | DXL_RXCHECKSUM | DXL_RXLENGTH | DXL_BAD_INST | DXL_BAD_ID))
    {
        dxl_hal_clear();
    }

    TxNumByte = gbInstructionPacket[DXL_PKT_LEN] + 4;
    RealTxNumByte = dxl_hal_tx((uint8_t*)gbInstructionPacket, TxNumByte);

    if (TxNumByte!=RealTxNumByte)
    {
        gbCommStatus |= DXL_TXFAIL;
        giBusUsing = 0;
        return;
    }

    if (gbInstructionPacket[DXL_PKT_INST]==INST_READ_DATA)
        dxl_hal_set_timeout(gbInstructionPacket[DXL_PKT_PARA+1]+6);
    else
        dxl_hal_set_timeout(6);

    gbCommStatus = DXL_TXSUCCESS;
}

//##############################################################################
void dxl_rx_packet(void)
{
    uint8_t i, j, nRead;
    uint8_t checksum = 0;

    if (giBusUsing==0)
        return;

    giBusUsing = 1;

    if (gbInstructionPacket[DXL_PKT_ID]==BROADCAST_ID)
    {
        gbCommStatus = DXL_RXSUCCESS;
        giBusUsing = 0;
        return;
    }

    if (gbCommStatus&DXL_TXSUCCESS)
    {
        gbRxGetLength = 0;
        gbRxPacketLength = 6;
    }

    nRead = dxl_hal_rx((uint8_t*)&gbStatusPacket[gbRxGetLength], gbRxPacketLength-gbRxGetLength);

    gbRxGetLength += nRead;
    if (gbRxGetLength<gbRxPacketLength)
    {
        if (dxl_hal_timeout()==1)
        {
            if (gbRxGetLength==0)
                gbCommStatus = DXL_RXTIMEOUT;
            else
                gbCommStatus = DXL_RXLENGTH;
            giBusUsing = 0;
            return;
        }
    }

    // Find packet header
    for (i=0; i<(gbRxGetLength-1); i++)
    {
        if ( (gbStatusPacket[i]==0xFF) && (gbStatusPacket[i+1]==0xFF) )
        {
            break;
        }
        else if ( (i==gbRxGetLength-2) && (gbStatusPacket[gbRxGetLength-1]==0xFF) )
        {
            break;
        }
    }
    if (i>0)
    {
        for (j=0; j<(gbRxGetLength-i); j++)
            gbStatusPacket[j] = gbStatusPacket[j + i];

        gbRxGetLength -= i;
    }

    // Check if received full packet
    if (gbRxGetLength<gbRxPacketLength)
    {
        gbCommStatus = DXL_RXWAITING;
        return;
    }

    // Check id pairing
    if (gbInstructionPacket[DXL_PKT_ID]!=gbStatusPacket[DXL_PKT_ID])
    {
        gbCommStatus = DXL_BAD_ID | DXL_RXFAIL;
        giBusUsing = 0;
        return;
    }

    gbRxPacketLength = gbStatusPacket[DXL_PKT_LEN] + 4;
    if (gbRxGetLength<gbRxPacketLength)
    {
        nRead = dxl_hal_rx((uint8_t*)&gbStatusPacket[gbRxGetLength], gbRxPacketLength-gbRxGetLength);
        gbRxGetLength += nRead;
        if (gbRxGetLength<gbRxPacketLength)
        {
            gbCommStatus = DXL_RXWAITING;
            return;
        }
    }

    // Check checksum
    for (i=0; i<(gbStatusPacket[DXL_PKT_LEN]+1); i++)
        checksum += gbStatusPacket[i+2];
    checksum = ~checksum;

    if (gbStatusPacket[gbStatusPacket[DXL_PKT_LEN]+3]!=checksum)
    {
        gbCommStatus = DXL_RXCHECKSUM | DXL_RXFAIL;
        giBusUsing = 0;
        return;
    }

    gbCommStatus = DXL_RXSUCCESS;
    giBusUsing = 0;
}

//##############################################################################
void dxl_txrx_packet(void)
{
    dxl_tx_packet();

    if (!(gbCommStatus&DXL_TXSUCCESS))
        return;

    dxl_clear_statpkt();
    do {
        dxl_rx_packet();
        uDelay(50);
    } while (gbCommStatus&DXL_RXWAITING);
}

//##############################################################################
uint16_t dxl_get_result(void)
{
    return gbCommStatus;
}

//##############################################################################
void dxl_set_txpacket_id(uint8_t id)
{
    gbInstructionPacket[DXL_PKT_ID] = id;
}

//##############################################################################
void dxl_set_txpacket_instruction(uint8_t instruction)
{
    gbInstructionPacket[DXL_PKT_INST] = instruction;
}

//##############################################################################
void dxl_set_txpacket_parameter(uint8_t index, uint8_t value )
{
    gbInstructionPacket[DXL_PKT_PARA+index] = value;
}

//##############################################################################
void dxl_set_txpacket_length(uint8_t length)
{
    gbInstructionPacket[DXL_PKT_LEN] = length;
}

//##############################################################################
uint8_t dxl_get_rxpacket_error(uint8_t errbit)
{
    if ((gbCommStatus&DXL_RXFAIL))
        return 0x80;

    if (gbStatusPacket[DXL_PKT_ERR]&errbit)
        return 1;

    return 0;
}

//##############################################################################
uint8_t dxl_get_rxpacket_length(void)
{
    if ((gbCommStatus&DXL_RXFAIL))
        return 0;

    return gbStatusPacket[DXL_PKT_LEN];
}

//##############################################################################
uint8_t dxl_get_rxpacket_parameter(uint8_t index)
{
    if ((gbCommStatus&DXL_RXFAIL))
        return 0;

    return gbStatusPacket[DXL_PKT_PARA+index];
}

//##############################################################################
uint16_t dxl_makeword(uint8_t lowbyte, uint8_t highbyte)
{
    uint16_t word;

    word = highbyte;
    word = word<<8;
    word = word+lowbyte;
    return word;
}

//##############################################################################
uint8_t dxl_get_lowbyte(uint16_t word)
{
    uint16_t temp = (word&0x00FF);
    return (uint8_t) temp;
}

//##############################################################################
uint8_t dxl_get_highbyte(uint16_t word)
{
    uint16_t temp = ((word&0xFF00)>>8);
    return (uint8_t) temp;
}

//##############################################################################
void dxl_ping(uint8_t id)
{
    while (giBusUsing);

    gbInstructionPacket[DXL_PKT_ID] = id;
    gbInstructionPacket[DXL_PKT_INST] = INST_PING;
    gbInstructionPacket[DXL_PKT_LEN] = 2;

    dxl_txrx_packet();
}

//##############################################################################
uint8_t dxl_read_byte(uint8_t id, uint8_t address)
{
    while(giBusUsing);

    gbInstructionPacket[DXL_PKT_ID] = id;
    gbInstructionPacket[DXL_PKT_INST] = INST_READ_DATA;
    gbInstructionPacket[DXL_PKT_PARA] = address;
    gbInstructionPacket[DXL_PKT_PARA+1] = 1;
    gbInstructionPacket[DXL_PKT_LEN] = 4;

    dxl_txrx_packet();

    if ((gbCommStatus&DXL_RXFAIL))
        return 0;

    return gbStatusPacket[DXL_PKT_PARA];
}

//##############################################################################
void dxl_write_byte(uint8_t id, uint8_t address, uint8_t value)
{
    while(giBusUsing);

    gbInstructionPacket[DXL_PKT_ID] = id;
    gbInstructionPacket[DXL_PKT_INST] = INST_WRITE_DATA;
    gbInstructionPacket[DXL_PKT_PARA] = address;
    gbInstructionPacket[DXL_PKT_PARA+1] = value;
    gbInstructionPacket[DXL_PKT_LEN] = 4;

    dxl_txrx_packet();
}

//##############################################################################
uint16_t dxl_read_word(uint8_t id, uint8_t address)
{
    while (giBusUsing);

    gbInstructionPacket[DXL_PKT_ID] = id;
    gbInstructionPacket[DXL_PKT_INST] = INST_READ_DATA;
    gbInstructionPacket[DXL_PKT_PARA] = address;
    gbInstructionPacket[DXL_PKT_PARA+1] = 2;
    gbInstructionPacket[DXL_PKT_LEN] = 4;

    dxl_txrx_packet();

    if ((gbCommStatus&DXL_RXFAIL))
        return 0;

    return dxl_makeword(gbStatusPacket[DXL_PKT_PARA], gbStatusPacket[DXL_PKT_PARA+1]);
}

//##############################################################################
void dxl_write_word(uint8_t id, uint8_t address, uint16_t value)
{
    while (giBusUsing);

    gbInstructionPacket[DXL_PKT_ID] = id;
    gbInstructionPacket[DXL_PKT_INST] = INST_WRITE_DATA;
    gbInstructionPacket[DXL_PKT_PARA] = address;
    gbInstructionPacket[DXL_PKT_PARA+1] = dxl_get_lowbyte(value);
    gbInstructionPacket[DXL_PKT_PARA+2] = dxl_get_highbyte(value);
    gbInstructionPacket[DXL_PKT_LEN] = 5;

    dxl_txrx_packet();
}

//##############################################################################
void dxl_clear_statpkt(void)
{
    uint8_t i, max=gbStatusPacket[DXL_PKT_LEN];
    if ( (max>0) && (max<DXL_MAXNUM_RXPARAM) )
    {
        for (i=0; i<(max+4); i++)
            gbStatusPacket[i]=0;
    }
    else
    {
        for (i=0; i<6; i++)
            gbStatusPacket[i]=0;
    }
}

//##############################################################################
void dxl_capture(uint8_t id)
{
//    while(giBusUsing);

//    gbInstructionPacket[DXL_PKT_ID] = id;
//    gbInstructionPacket[DXL_PKT_INST] = INST_CAP_REGION;
//    gbInstructionPacket[DXL_PKT_LEN] = 2;

//    dxl_txrx_packet();

    dxl_write_byte(id, 0, 0);
}

//##############################################################################
uint8_t dxl_recover(uint8_t id, HaViMo2_Region_Buffer_t* hvm2rb)
{
    if (hvm2rb==0)//NULL)
        return 0xFF;

    while (giBusUsing);

    uint8_t i;
    for (i=0; i<15; i++)
    {
        gbInstructionPacket[DXL_PKT_ID] = id;
        gbInstructionPacket[DXL_PKT_INST] = INST_READ_DATA;
        gbInstructionPacket[DXL_PKT_PARA] = ((i+1)*16);
        gbInstructionPacket[DXL_PKT_PARA+1] = 16;
        gbInstructionPacket[DXL_PKT_LEN] = 4;

        dxl_txrx_packet();

        if (gbStatusPacket[DXL_PKT_LEN]==(16+2))
        {
        	hvm2rb->valid++;
        }
        else
        {
//            PrintCommStatus(gbCommStatus);
//            PrintErrorCode();
            return hvm2rb->valid;
//            break;
        }

        hvm2rb->rb[i].Index=gbStatusPacket[DXL_PKT_PARA];
        hvm2rb->rb[i].Color=gbStatusPacket[DXL_PKT_PARA+1];
        hvm2rb->rb[i].NumPix=(
                (uint16_t)gbStatusPacket[DXL_PKT_PARA+2]+
                ((uint16_t)gbStatusPacket[DXL_PKT_PARA+3]<<8));
        hvm2rb->rb[i].SumX=
                (
                    ((uint32_t)gbStatusPacket[DXL_PKT_PARA+4]+
                    ((uint32_t)gbStatusPacket[DXL_PKT_PARA+5]<<8)+
                    ((uint32_t)gbStatusPacket[DXL_PKT_PARA+6]<<16))
                );
        hvm2rb->rb[i].SumY=
                (
                    ((uint32_t)gbStatusPacket[DXL_PKT_PARA+8]+
                    ((uint32_t)gbStatusPacket[DXL_PKT_PARA+9]<<8)+
                    ((uint32_t)gbStatusPacket[DXL_PKT_PARA+10]<<16))
                );
        hvm2rb->rb[i].MaxX=gbStatusPacket[DXL_PKT_PARA+12];
        hvm2rb->rb[i].MinX=gbStatusPacket[DXL_PKT_PARA+13];
        hvm2rb->rb[i].MaxY=gbStatusPacket[DXL_PKT_PARA+14];
        hvm2rb->rb[i].MinY=gbStatusPacket[DXL_PKT_PARA+15];
    }
    return hvm2rb->valid;
}

#endif



#ifdef USING_PC_UART
//##############################################################################
//##############################################################################
// Serial/PC_UART platform dependent source
//##############################################################################
//##############################################################################
#define PC_UART_BUFFER_LENGTH           128

static volatile uint16_t gbPcuWrite, gbPcuRead;
static volatile uint8_t gbpPcuBuffer[PC_UART_BUFFER_LENGTH]={0};
static volatile uint8_t ReBootToBootLoader;
static volatile uint8_t bUsingUSART1=0;

uint8_t pcu_hal_open(uint32_t);
void pcu_hal_close(void);
void pcu_hal_set_timeout(uint8_t);
uint8_t pcu_hal_timeout(void);

void pcu_put_byte(uint8_t);
uint8_t pcu_get_queue(void);
uint8_t pcu_peek_queue(void);
//uint8_t pcu_get_qstate(void);
void pcu_clear_queue(void);
void pcu_put_queue(void);
void RxD_PCU_Interrupt(void);

#ifdef USING_STDIO_H
static FILE *PC_UART_Device;
int uart_putchar(char c, FILE *stream) {return std_putchar(c);}
int uart_getchar(FILE *stream) {return std_getchar();}
#endif

//##############################################################################
uint8_t pcu_hal_open(uint32_t baudrate)
{
    if (bUsingUSART1==0xFA)
        return 0;

    // Double USART Transmission Speed
    // RXD Interupt enable, RXD enable, TXD enable
    // Asynchronous USART, No Parity, One Stop Bit, Eight Data Bits, Rising Edge
    // Baud rate setting
    UCSR1A = (0<<RXC1) | (0<<TXC1) | (0<<UDRE1) | (0<<FE1) | (0<<DOR1) | (0<<UPE1) | (1<<U2X1) | (0<<MPCM1);
    UCSR1B = (1<<RXCIE1) | (0<<TXCIE1) | (0<<UDRIE1) | (1<<RXEN1) | (1<<TXEN1) | (0<<UCSZ12) | (0<<RXB81) | (0<<TXB81);
    UCSR1C = (0<<UMSEL11) | (0<<UMSEL10) | (0<<UPM11) | (0<<UPM10) | (0<<USBS1) | (1<<UCSZ11) | (1<<UCSZ10) | (0<<UCPOL1);
    UBRR1H = ((F_CPU/(baudrate<<3))-1)>>8;
    UBRR1L = ((F_CPU/(baudrate<<3))-1);

	pcu_clear_queue();

#ifdef USING_STDIO_H
    PC_UART_Device = fdevopen( uart_putchar, uart_getchar );
#endif
    bUsingUSART1 = 0x01;

    return 1;
}

//##############################################################################
void pcu_hal_close(void)
{
	// RXD Interupt disable, RXD disable, TXD disable
	UCSR1B = (0<<RXCIE1) | (0<<TXCIE1) | (0<<UDRIE1) | (0<<RXEN1) | (0<<TXEN1) | (0<<UCSZ12) | (0<<RXB81) | (0<<TXB81);
}

//##############################################################################
void pcu_hal_set_timeout(uint8_t NumRcvByte)
{
    // 200us; ~180 us to transmit one byte at 57600 bps
    start_timeout_pcu(NumRcvByte*200);
}

//##############################################################################
uint8_t pcu_hal_timeout(void)
{
    if (glPcuTimeoutCounter==0)
        return 1;
    else
        return 0;
}

//##############################################################################
void pcu_put_byte(uint8_t bTxdData)
{
    SetLED(TXD, 1);

    while ( !(UCSR1A&(1<<UDRE1)) );
    UDR1 = bTxdData;

    SetLED(TXD, 0);
}

//##############################################################################
uint8_t pcu_get_queue(void)
{
    if (gbPcuWrite==gbPcuRead)
        return 0xFF;

    uint8_t data = gbpPcuBuffer[gbPcuRead++];

    if (gbPcuRead>(PC_UART_BUFFER_LENGTH-1))
        gbPcuRead = 0;

    return data;
}

//##############################################################################
uint8_t pcu_peek_queue(void)
{
    if (gbPcuWrite==gbPcuRead)
        return 0xFF;

    uint8_t data = gbpPcuBuffer[gbPcuRead];

    return data;
}

//##############################################################################
void pcu_put_queue(void)
{
    uint8_t temp;
    temp = UDR1;

    if (temp=='#')
        ReBootToBootLoader++;
    else
        ReBootToBootLoader=0;

    if (ReBootToBootLoader>15)
    {
#ifdef USING_BREAK_TO_BOOTLOADER
        //BreakToBootLoader();
#endif
    }

    SetLED(RXD, 1);

    if (gbPcuWrite<(PC_UART_BUFFER_LENGTH-1))
    {
        gbpPcuBuffer[gbPcuWrite++] = temp;
    }
    else
    {
        gbpPcuBuffer[gbPcuWrite] = temp;
        gbPcuWrite = 0;
    }

    if (gbPcuRead==gbPcuWrite)
        gbPcuRead++;
    if (gbPcuRead>(PC_UART_BUFFER_LENGTH-1))
        gbPcuRead=0;

    SetLED(RXD, 0);
}

//##############################################################################
void pcu_clear_queue(void)
{
    gbPcuWrite = 0;
    gbPcuRead = 0;
}

//##############################################################################
uint8_t pcu_get_qstate(void)
{
    if (gbPcuWrite==gbPcuRead)
    {
        pcu_clear_queue();
        return 0;
    }
    else if (gbPcuRead<gbPcuWrite)
        return (uint8_t) (gbPcuWrite-gbPcuRead);
    else
        return (uint8_t) (PC_UART_BUFFER_LENGTH-(gbPcuRead-gbPcuWrite));
}

//##############################################################################
void RxD_PCU_Interrupt(void)
{
    pcu_put_queue();
}



//##############################################################################
//##############################################################################
// PC UART platform independent source
//##############################################################################
//##############################################################################

//##############################################################################
uint8_t pcu_initialize(uint32_t baudrate)
{
    if (pcu_hal_open(baudrate)==0)
        return 0;

    return 1;
}

//##############################################################################
void pcu_terminate(void)
{
    pcu_hal_close();
}

//##############################################################################
int std_putchar(char c)
{
    if (c=='\n')
    {
        pcu_put_byte((uint8_t) '\r'); //0x0D
        pcu_put_byte((uint8_t) '\n'); //0x0A
    }
    else
    {
        pcu_put_byte((uint8_t) c);
    }

    return c;
}

//##############################################################################
int std_puts(const char *str)
{
    int n=0;
    while (str[n])
        std_putchar(str[n++]);

    return n;
}

//##############################################################################
int std_getchar(void)
{
    char c;

    pcu_hal_set_timeout(10);
    while ( (pcu_hal_timeout()==0) && (pcu_get_qstate()==0) );
    if (pcu_get_qstate()==0)
        return 0xFF;

    c = pcu_get_queue();

    if (c=='\r')
        c = '\n';

    return c;
}

//##############################################################################
char* std_gets(char *str)
{
    uint8_t c, len=0;

    while (len<128)
    {
        pcu_hal_set_timeout(10);
        while ( (pcu_hal_timeout()==0) && (pcu_get_qstate()==0) );
        if (pcu_get_qstate()==0)
        {
            if (len==0)
            {
                return 0;//NULL;
            }
            else
            {
                str[len] = '\0';
                return str;
            }
        }

        c = pcu_get_queue();
        if ( (c=='\n') || (c=='\0') )
        {
            if (len==0)
            {
                return 0;//NULL;
            }
            else
            {
                str[len] = '\0';
                return str;
            }
        }
        else
            str[len++] = (int8_t) c;
    }

    return str;
}


//##############################################################################
void PrintCommStatus(uint16_t Status)
{
    if (Status&DXL_TXFAIL)
        std_puts("\nDXL_TXFAIL: Failed transmit instruction packet!\n");

    if (Status&DXL_RXFAIL)
        std_puts("\nDXL_RXFAIL: Failed get status packet from device!\n");

    if (Status&DXL_TXERROR)
        std_puts("\nDXL_TXERROR: Incorrect instruction packet!\n");

    if (Status&DXL_BAD_INST)
        std_puts("\nDXL_BAD_INST: Invalid Instruction byte\n");

    if (Status&DXL_BAD_ID)
        std_puts("\nDXL_BAD_ID: ID's not same for instruction and status packets\n");

    if (Status&DXL_RXWAITING)
        std_puts("\nDXL_RXWAITING: Now receiving status packet!\n");

    if (Status&DXL_RXTIMEOUT)
        std_puts("\nDXL_RXTIMEOUT: There is no status packet!\n");

    if (Status&DXL_RXCHECKSUM)
        std_puts("\nDXL_RXCHECKSUM: Incorrect status packet checksum!\n");

//    else
//        std_puts("\nThis is unknown error code!\n");
}

//##############################################################################
void PrintErrorCode(void)
{
    if(dxl_get_rxpacket_error(ERRBIT_VOLTAGE) == 1)
        std_puts("\nInput voltage error!\n");

    if(dxl_get_rxpacket_error(ERRBIT_ANGLE) == 1)
        std_puts("\nAngle limit error!\n");

    if(dxl_get_rxpacket_error(ERRBIT_OVERHEAT) == 1)
        std_puts("\nOverheat error!\n");

    if(dxl_get_rxpacket_error(ERRBIT_RANGE) == 1)
        std_puts("\nOut of range error!\n");

    if(dxl_get_rxpacket_error(ERRBIT_CHECKSUM) == 1)
        std_puts("\nChecksum error!\n");

    if(dxl_get_rxpacket_error(ERRBIT_OVERLOAD) == 1)
        std_puts("\nOverload error!\n");

    if(dxl_get_rxpacket_error(ERRBIT_INSTRUCTION) == 1)
        std_puts("\nInstruction code error!\n");
}

//##############################################################################
int PrintChar(char c){return std_putchar(c);}

//##############################################################################
int PrintString(const char* s){return std_puts(s);}

//##############################################################################
int GetChar(void){return std_getchar();}

//##############################################################################
char* GetString(char* s){return std_gets(s);}

//##############################################################################
void Printu32d(uint32_t lNum)
{
    uint32_t temp, div=1000000000;
    char out[11];
    uint8_t i, j;

    for (i=0; i<10; i++)
    {
        temp = (char) (lNum/div);
        lNum = (lNum%div);
//        lNum -= (uint32_t) (temp*div);
//        out[i] = (char) (temp&0x0000000F)+0x30;
        out[i] = (char) ((temp&0x0F)+0x30);
        div /= 10;
    }
    out[i] = '\0';

    for (i=0; i<10; i++)
    {
        if (out[0]=='0')
        {
            for (j=0; j<10; j++)
            {
                out[j] = out[j+1];
                if (out[j]=='\0')
                    break;
            }
        }
    }

    std_puts(out);
    return;
}

//##############################################################################
void Prints32d(int32_t lNumS)
{
    uint32_t temp, lNum, div=1000000000;
    char out[12];
    uint8_t i, j;

    if (lNum<0)
    {
        out[0] = '-';
        lNum = (uint32_t) ((~lNumS)+1);
    }
    else
    {
        out[0] = '+';
        lNum = (uint32_t) (lNumS);
    }

    for (i=1; i<11; i++)
    {
        temp = (lNum/div);
        lNum = (lNum%div);
//        lNum -= (uint32_t) (temp*div);
//        out[i] = (char) (temp&0x0000000F)+0x30;
        out[i] = (char) ((temp&0x0F)+0x30);
        div /= 10;
    }
    out[i] = '\0';

    for (i=0; i<11; i++)
    {
        if (out[0]=='0')
        {
            for (j=0; j<11; j++)
            {
                out[j] = out[j+1];
                if (out[j]=='\0')
                    break;
            }
        }
    }

    std_puts(out);
    return;
}

//##############################################################################
void Printu16h(uint16_t wNum)
{
    char out[7];
    out[0] = '0';
    out[1] = 'x';
    out[6] = '\0';

    out[2] = (char) ((wNum>>12)&0x0F)+0x30;
    if (out[2] > '9')
        out[2] += 7;

    out[3] = (char) ((wNum>>8)&0x0F)+0x30;
    if (out[3] > '9')
        out[3] += 7;

    out[4] = (char) ((wNum>>4)&0x0F)+0x30;
    if (out[4] > '9')
        out[4] += 7;

    out[5] = (char) (wNum&0x0F)+0x30;
    if (out[5] > '9')
        out[5] += 7;

    std_puts(out);
    return;
}

//##############################################################################
void Printu8h(uint8_t bNum)
{
    char out[5];
    out[0] = '0';
    out[1] = 'x';
    out[4] = '\0';

    out[2] = (char) ((bNum>>4)&0x0F)+0x30;
    if (out[2] > '9')
        out[2] += 7;

    out[3] = (char) (bNum&0x0F)+0x30;
    if (out[3] > '9')
        out[3] += 7;

    std_puts(out);
    return;
}

#endif





#ifdef USING_ZIGBEE
//##############################################################################
//##############################################################################
// Zigbee SDK platform dependent source
//##############################################################################
//##############################################################################
#define ZIGBEE_BUFFER_LENGTH            64

static volatile uint8_t gbZigWrite=0, gbZigRead=0;
static volatile uint8_t gbpZigBuffer[ZIGBEE_BUFFER_LENGTH];

uint8_t zgb_hal_open(uint32_t);
void zgb_hal_close(void);
uint8_t zgb_hal_tx(uint8_t*, uint8_t);
uint8_t zgb_hal_rx(uint8_t*, uint8_t);
void RxD_ZIG_Interrupt(void);

//##############################################################################
uint8_t zgb_hal_open(uint32_t baudrate)
{
    if (bUsingUSART1==0x01)
        return 0;
    // Double USART Transmission Speed
    // RXD Interupt enable, RXD enable, TXD enable
    // Asynchronous USART, No Parity, One Stop Bit, Eight Data Bits, Rising Edge
    // Baud rate setting
    UCSR1A = (0<<RXC1) | (0<<TXC1) | (0<<UDRE1) | (0<<FE1) | (0<<DOR1) | (0<<UPE1) | (1<<U2X1) | (0<<MPCM1);
    UCSR1B = (1<<RXCIE1) | (0<<TXCIE1) | (0<<UDRIE1) | (1<<RXEN1) | (1<<TXEN1) | (0<<UCSZ12) | (0<<RXB81) | (0<<TXB81);
    UCSR1C = (0<<UMSEL11) | (0<<UMSEL10) | (0<<UPM11) | (0<<UPM10) | (0<<USBS1) | (1<<UCSZ11) | (1<<UCSZ10) | (0<<UCPOL1);
    UBRR1H = ((F_CPU/(baudrate<<3))-1)>>8;
    UBRR1L = ((F_CPU/(baudrate<<3))-1);

    bUsingUSART1 = 0xFA;

    // Still not sure what these actually do (never changed to Output pins)
//    PORTD &= ~0x80;	//PORT_LINK_PLUGIN = 0;   // no pull up
//    PORTD &= ~0x20;	//PORT_ENABLE_RXD_LINK_PC = 0;
//    PORTD |= 0x40;	//PORT_ENABLE_RXD_LINK_ZIGBEE = 1;

    return 1;
}

//##############################################################################
void zgb_hal_close(void)
{
	// RXD Interupt disable, RXD disable, TXD disable
	UCSR1B = (0<<RXCIE1) | (0<<TXCIE1) | (0<<UDRIE1) | (0<<RXEN1) | (0<<TXEN1) | (0<<UCSZ12) | (0<<RXB81) | (0<<TXB81);
}

//##############################################################################
uint8_t zgb_hal_tx(uint8_t *pPacket, uint8_t numPacket)
{
    uint8_t i;
    for (i=0; i<numPacket; i++)
    {
        SetLED(TXD, 1);

        while ( !(UCSR1A&(1<<UDRE1)) );
        UDR0 = bTxdData;

        SetLED(TXD, 0);
    }

    return numPacket;
}

//##############################################################################
uint8_t zgb_hal_rx(uint8_t *pPacket, uint8_t numPacket)
{
    uint8_t i;
    for (i=0; i<numPacket; i++)
    {
        if (gbZigRead!=gbZigWrite)
        {
            pPacket[i] = gbpZigBuffer[gbZigRead++];
            if (gbZigRead>(ZIGBEE_BUFFER_LENGTH-1))
                gbZigRead = 0;
        }
        else
            return i;
    }

    return numPacket;
}

//##############################################################################
void RxD_ZIG_Interrupt(void)
{
    uint8_t temp;
    temp = UDR1;

    if (temp=='#')
        ReBootToBootLoader++;
    else
        ReBootToBootLoader=0;

    if (ReBootToBootLoader>15)
    {
#ifdef USING_BREAK_TO_BOOTLOADER
        //BreakToBootLoader();
#endif
    }

    SetLED(RXD, 1);

    if (gbZigWrite<(ZIGBEE_BUFFER_LENGTH-1))
    {
        gbpZigBuffer[gbZigWrite++] = temp;
    }
    else
    {
        gbpZigBuffer[gbZigWrite] = temp;
        gbZigWrite = 0;
    }

    if (gbZigRead==gbZigWrite)
        gbZigRead++;
    if (gbZigRead>(ZIGBEE_BUFFER_LENGTH-1))
        gbZigRead=0;

    SetLED(RXD, 0);
}



//##############################################################################
//##############################################################################
// Zigbee SDK platform independent source
//##############################################################################
//##############################################################################
#define PACKET_LENGTH                   6

static uint8_t gbRcvPacket[PACKET_LENGTH];
static uint8_t gbRcvPacketNum;
static uint16_t gwRcvData;
static volatile uint8_t gbRcvFlag;

//##############################################################################
uint8_t zgb_initialize(uint32_t baudrate)
{
    if (zgb_hal_open(baudrate)==0)
        return 0;

    gbRcvFlag = 0;
    gwRcvData = 0;
    gbRcvPacketNum = 0;
    return 1;
}

//##############################################################################
void zgb_terminate(void)
{
    zgb_hal_close();
}

//##############################################################################
uint8_t zgb_tx_data(uint16_t word)
{
    uint8_t SndPacket[6];
    uint8_t lowbyte = (uint8_t) (word&0xFF);
    uint8_t highbyte = (uint8_t) ((word>>8)&0xFF);

    SndPacket[0] = 0xFF;
    SndPacket[1] = 0x55;
    SndPacket[2] = lowbyte;
    SndPacket[3] = ~lowbyte;
    SndPacket[4] = highbyte;
    SndPacket[5] = ~highbyte;

    if (zgb_hal_tx(SndPacket, 6)!=6)
        return 0;

    return 1;
}

//##############################################################################
uint8_t zgb_rx_check(void)
{
    uint8_t RcvNum;
    uint8_t checksum;
    uint8_t i, j;

    if (gbRcvFlag==1)
        return 1;

    // Fill packet buffer
    if (gbRcvPacketNum<6)
    {
        RcvNum = zgb_hal_rx((uint8_t*)&gbRcvPacket[gbRcvPacketNum], (6-gbRcvPacketNum));
        if (RcvNum!=-1)
            gbRcvPacketNum += RcvNum;
    }

    // Find header
    if (gbRcvPacketNum>=2)
    {
        for (i=0; i<gbRcvPacketNum; i++)
        {
            if (gbRcvPacket[i]==0xFF)
            {
                if (i<=(gbRcvPacketNum-2))
                {
                    if (gbRcvPacket[i+1]==0x55)
                        break;
                }
            }
        }

        if (i>0)
        {
            if (i==gbRcvPacketNum)
            {
                // Cannot find header
                if (gbRcvPacket[i-1]==0xFF)
                    i--;
            }

            // Remove data before header
            for (j=i; j<gbRcvPacketNum; j++)
            {
                gbRcvPacket[j-i] = gbRcvPacket[j];
            }
            gbRcvPacketNum -= i;
        }
    }

    // Verify packet
    if (gbRcvPacketNum==6)
    {
        if ( (gbRcvPacket[0]==0xFF) && (gbRcvPacket[1]==0x55) )
        {
            checksum = ~gbRcvPacket[3];
            if (gbRcvPacket[2]==checksum)
            {
                checksum = ~gbRcvPacket[5];
                if (gbRcvPacket[4]==checksum)
                {
                    gwRcvData = (uint16_t) ((gbRcvPacket[4]<<8)&0xFF00);
                    gwRcvData += gbRcvPacket[2];
                    gbRcvFlag = 1;
                }
            }
        }
        gbRcvPacket[0] = 0x00;
        gbRcvPacketNum = 0;
    }

    return gbRcvFlag;
}

//##############################################################################
uint16_t zgb_rx_data(void)
{
    gbRcvFlag = 0;
    return gwRcvData;
}

#endif



//##############################################################################
//##############################################################################
// CM-530 Configuration functions
//##############################################################################
//##############################################################################
void ISR_Delay_Base(void);

void SysTick_Configuration(void);
void GPIO_Configuration(void);
void ADC_Configuration(void);

//##############################################################################
void SysInit(void)
{
    // Clear the WatchDog Early Wakeup interrupt flag
//    WWDG_ClearFlag();
    ReBootToBootLoader = 0;

    // GPIO configuration
    GPIO_Configuration();

    // System clock count configuration
    SysTick_Configuration();

    // Analog to Digital Converter Configuration
    ADC_Configuration();
	
//##############################################################################
    uint16_t error=0, tog=0;

#ifdef USING_PC_UART
    mDelay(100);
    if (!pcu_initialize(Baudrate_PCU))
        error|=(1<<0);
#endif
#ifdef USING_ZIGBEE
    mDelay(100);
    if (!zgb_initialize(Baudrate_ZIG))
        error|=(1<<1);
#endif
#ifdef USING_DYNAMIXEL
    mDelay(100);
    if (!dxl_initialize(Baudrate_DXL))
        error|=(1<<2);
#endif

    SetLED(PLAY, (error&(1<<0)));
    SetLED(PROGRAM, (error&(1<<1)));
    SetLED(MANAGE, (error&(1<<2)));

    while(error)
    {
        SetLED(POWER, tog);
        tog ^= 1;

        mDelay(500);
    }

    SetLED(POWER, 1);

    PrintString("\n\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");
    PrintString("CM-510 Experimental Example         ");
    PrintString(CM530_FIRMWARE_VERSION);

    PrintString("Battery Voltage: ");
    Printu32d((uint32_t)ReadAnalog(VBUS)>>2);
    PrintString("e-1 [Volts]\n");

    PrintString("PCU:");
#ifdef USING_PC_UART
    Printu32d(Baudrate_PCU);
    PrintString("(bps)\n");
#else
    PrintString("Not in use\n");
#endif

    PrintString("ZIG:");
#ifdef USING_ZIGBEE
    Printu32d(Baudrate_ZIG);
    PrintString("(bps)\n");
#else
    PrintString("Not in use\n");
#endif

    PrintString("DXL:");
#ifdef USING_DYNAMIXEL
    Printu32d(Baudrate_DXL);
    PrintString("(bps)\n");
#else
    PrintString("Not in use\n");
#endif

    Buzzed(150, 200);    // 2500 Hz ~ Ds_7/Eb_7

    uint8_t id, num=0;
    uint16_t wdata;
    for (id=1; id<(250); id++)
    {
        wdata = (dxl_read_byte(id, P_ID)&0x00FF);
        if (wdata==id)
        {
            wdata=0;
            num++;
            PrintString("{");
            Printu32d(id);
            PrintString(", ");

            wdata = dxl_read_word(id, P_MODEL_NUMBER_L);
            error = dxl_get_result();
            if (!(error&DXL_RXSUCCESS))
                PrintCommStatus(error);
            Printu32d(wdata);
            if (wdata==MODEL_AX12)
            {
                PrintString(" (AX-12)");
            }
            else if (wdata==MODEL_AX18)
            {
                PrintString(" (AX-18)");
            }
            else if (wdata==MODEL_AXS1)
            {
                PrintString(" (AX-S1)");
            }
            else if (wdata==MODEL_AXS20)
            {
                PrintString(" (AX-S20)");
            }
            else if (wdata==MODEL_JHFPS)
            {
                PrintString(" (JH-FPS)");
            }
            else if (wdata==MODEL_MX28)
            {
                PrintString(" (MX-28)");
            }
            else if (wdata==MODEL_HaViMo2)
            {
                PrintString(" (HaViMo2)");
            }

            PrintString(", ");
            Printu32d(dxl_read_byte(id, P_FIRMWARE_VERSION));
            PrintString("} \n");
        }
    }
    PrintString("\nDXL DEVICES:");
    Printu32d(num);
    PrintString("\n~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n");

    Buzzed(150, 2300);    // 217 Hz ~ A_4

    PrintString("Press START to begin User Program...\n\n");
    
// Wait for START button to be pressed (and toggle MANAGE LED)
    while (!ReadButton(START))
    {
        if (glCountdownCounter==0)
        {
            SetLED(MANAGE, tog);
            tog ^= 1;
            StartCountdown(500);
        }
    }
}

//##############################################################################
void ISR_Delay_Base(void)
{
#ifndef USING_UTIL_DELAY_H
    // User accessible delay counter
    if (glDelayCounter>1)
        glDelayCounter--;
    else if (glDelayCounter>0)
    {
        glDelayCounter--;
        gbCounterCount--;
    }
#endif
    // User accessible timeout/countdown counter
    if (glCountdownCounter>1)
    {
        glCountdownCounter--;
        SetLED(AUX, (glCountdownCounter&0x40));
/*
#ifdef USING_SYSTICK_100US
        if ( (glCountdownCounter&0x00000200) )
#elif defined USING_SYSTICK_10US
        if ( (glCountdownCounter&0x00001000) )
#elif defined USING_SYSTICK_1US
        if ( (glCountdownCounter&0x00010000) )
#endif
            SetLED(AUX, 1);
        else
            SetLED(AUX, 0);
*/
    }
    else if (glCountdownCounter>0)
    {
        SetLED(AUX, 0);
        glCountdownCounter--;
        gbCounterCount--;
    }

    // Buzzer countdown counter
    if (glBuzzerCounter>1)
        glBuzzerCounter--;
    else if (glBuzzerCounter>0)
    {
        glBuzzerCounter--;
        gbCounterCount--;
    }
#ifndef USING_UTIL_DELAY_H
    // Dynamixel timeout counter
    if (glDxlTimeoutCounter>1)
        glDxlTimeoutCounter--;
    else if (glDxlTimeoutCounter>0)
    {
        glDxlTimeoutCounter--;
        gbCounterCount--;
    }

    // PC UART timeout counter
    if (glPcuTimeoutCounter>1)
        glPcuTimeoutCounter--;
    else if (glPcuTimeoutCounter>0)
    {
        glPcuTimeoutCounter--;
        gbCounterCount--;
    }
#endif

    // Battery Monitor timeout counter
//    if (glBatTimeoutCounter>1)
//        glBatTimeoutCounter--;
//    else
//    {
//        Battery_Monitor_Alarm();
//        if (glBatTimeoutSet>100000)
//            glBatTimeoutCounter = glBatTimeoutSet;
//        else
//            glBatTimeoutCounter = 100000;
//    }

    // If no active counters, disable interrupt
    if (gbCounterCount==0)
    {
        SysTick_CounterCmd(SysTick_Counter_Disable);
    }
}

//##############################################################################
void SysTick_Configuration(void)
{
//#ifdef USING_SYSTICK_100US
//#define SYSTICK_INTERVAL_MICROS        100
//#elif defined USING_SYSTICK_10US
//#define SYSTICK_INTERVAL_MICROS        10
//#elif defined USING_SYSTICK_1US
//#define SYSTICK_INTERVAL_MICROS        1
//#endif

// General Countdown Timer
    // Timer5/Counter5 - 16-bit (no pins on 64-pin TQFP)
    // No Compare Match Output, CTC Mode (OCRnA==TOP)
    TCCR5A = (0<<COM5A1) | (0<<COM5A0) | (0<<COM5B1) | (0<<COM5B0) | (0<<WGM51) | (0<<WGM50);
    // CTC Mode (OCRnA==TOP), fclk_CTC = (fclk_IO/(2*N*(1+OCRnA))), N==64
    TCCR5B = (0<<ICNC5) | (0<<ICES5) | (0<<WGM53) | (1<<WGM52) | (0<<CS52) | (1<<CS51) | (1<<CS50);
    TCCR5C = (0<<FOC5A) | (0<<FOC5B);
    // 1/(nTime_ms/1000) = 16e6/(2*N*(1+OCRnA)), N==64
    // 1/(nTime_ms/1000) = (16e6/(2*N))/(1+OCRnA)
    // OCRnA = (((16e3/(2*N))*nTime_ms)-1);
    OCR5AH = 0;
//    OCR5AL = 249;
    OCR5AL = 124;
    TIMSK5 = (0<<OCIE5C) | (0<<OCIE5B) | (0<<OCIE5A) | (0<<TOIE5);


// Timeout Timer
    // Timer4/Counter4 - 16-bit (no pins on 64-pin TQFP)
    // No Compare Match Output, CTC Mode (OCRnA==TOP)
    TCCR4A = (0<<COM4A1) | (0<<COM4A0) | (0<<COM4B1) | (0<<COM4B0) | (0<<WGM41) | (0<<WGM40);
    // CTC Mode (OCRnA==TOP), fclk_CTC = (fclk_IO/(2*N*(1+OCRnA))), N==8
    TCCR4B = (0<<ICNC4) | (0<<ICES4) | (0<<WGM43) | (1<<WGM42) | (0<<CS42) | (1<<CS41) | (0<<CS40);
    TCCR4C = (0<<FOC4A) | (0<<FOC4B);
    // 1/(nTime_us/1000000) = 16e6/(2*N*(1+OCRnA)), N==8
    // 1/(nTime_us/1000000) = (16e6/(2*N))/(1+OCRnA)
    // OCRnA = (nTime_us-1);
    TIMSK4 = (0<<OCIE4C) | (0<<OCIE4B) | (0<<OCIE4A) | (0<<TOIE4);
}

//##############################################################################
void SysTick_CounterCmd(uint8_t state)
{
    if (state)
	{
        TIMSK5 |= (1<<OCIE5A);
        sei();
    }
    else
        TIMSK5 &= ~(1<<OCIE5A);
}

//##############################################################################
void GPIO_Configuration(void)
{
    // Enable LED's
    DDRC |= (PIN_LED_POWER | PIN_LED_MANAGE | PIN_LED_PROGRAM | PIN_LED_PLAY | PIN_LED_TXD | PIN_LED_RXD | PIN_LED_AUX);

    // Enable Buzzer
    DDRB |= (PIN_BUZZER);

    // Enable EPORT's
    DDRA |= (PIN_SIG_MOT1P | PIN_SIG_MOT2P | PIN_SIG_MOT3P | PIN_SIG_MOT4P | PIN_SIG_MOT5P | PIN_SIG_MOT6P);

    // Enable DXL Direction Buffer control pins
//    DDRE |= (0x04 | 0x08);

    // Enable ZIG/PCU Selection Buffer control pins?
//    DDRD |= (0x80 | 0x40 | 0x20 | 0x10);
}

//##############################################################################
void ADC_Configuration(void)
{
    ADCSRA = (uint8_t) (1<<ADEN) | (1<<ADPS2) | (1<<ADPS1);
}

//##############################################################################
ISR(TIMER4_COMPA_vect)
{
    TIMSK4 = (0<<OCIE4C) | (0<<OCIE4B) | (0<<OCIE4A) | (0<<TOIE4);
	OCR4AH = 0;
	OCR4AL = 19;
	glDxlTimeoutCounter=0;
	glPcuTimeoutCounter=0;
}

//##############################################################################
ISR(TIMER5_COMPA_vect)
{
    ISR_Delay_Base();
}

//##############################################################################
ISR(USART0_RX_vect)
{
#ifdef USING_DYNAMIXEL
	RxD_DXL_Interrupt();
#endif
}

//##############################################################################
ISR(USART1_RX_vect)
{
    if (bUsingUSART1==0xFA)
    {
#ifdef USING_ZIGBEE
		RxD_ZIG_Interrupt();
#endif
    }
    else if (bUsingUSART1==0x01)
    {
#ifdef USING_PC_UART
        RxD_PCU_Interrupt();
#endif
    }
}

