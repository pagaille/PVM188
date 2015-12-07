
#include <xc.h>


// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

// CONFIG
#pragma config FOSC = INTRCIO   // Oscillator Selection bits (INTOSC oscillator: I/O function on RA4/OSC2/CLKOUT pin, I/O function on RA5/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = ON      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF       // RA3/MCLR pin function select (RA3/MCLR pin function is MCLR)
#pragma config BOREN = ON       // Brown-out Detect Enable bit (BOD enabled)
#pragma config CP = OFF         // Code Protection bit (Program Memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)

#define JmpPulseSec RC5
#define JmpPulseMin RC4
#define JmpPulseHour RC3
#define JmpPauseSec RC0
#define JmpPauseMin RC1
#define JmpPauseHour RC2
#define JmpLoop RA5
#define JmpStartWithPause RA4
#define Relay RA2
#define ADCPulseChannel 1 // AD1
#define ADCPauseChannel 0 // AD0 

#define MIELE_VERSION
#define SLEEP() asm("sleep")

//Lookup table used to convert ADC [0..255] result to [1..60] result
const unsigned char ADCbase60[256] = {1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 3, 3, 3, 3, 
4, 4, 4, 4, 4, 5, 5, 5, 5, 6, 6, 6, 6, 7, 7, 7, 7, 8, 8, 8, 8, 8, 9, 9, 9, 9, 
10, 10, 10, 10, 11, 11, 11, 11, 12, 12, 12, 12, 12, 13, 13, 13, 13, 14, 14, 14, 
14, 15, 15, 15, 15, 16, 16, 16, 16, 16, 17, 17, 17, 17, 18, 18, 18, 18, 19, 19,
19, 19, 20, 20, 20, 20, 20, 21, 21, 21, 21, 22, 22, 22, 22, 23, 23, 23, 23, 24,
24, 24, 24, 24, 25, 25, 25, 25, 26, 26, 26, 26, 27, 27, 27, 27, 28, 28, 28, 28, 
28, 29, 29, 29, 29, 30, 30, 30, 30, 31, 31, 31, 31, 32, 32, 32, 32, 32, 33, 33,
33, 33, 34, 34, 34, 34, 35, 35, 35, 35, 36, 36, 36, 36, 36, 37, 37, 37, 37, 38,
38, 38, 38, 39, 39, 39, 39, 40, 40, 40, 40, 40, 41, 41, 41, 41, 42, 42, 42, 42,
43, 43, 43, 43, 44, 44, 44, 44, 44, 45, 45, 45, 45, 46, 46, 46, 46, 47, 47, 47,
47, 48, 48, 48, 48, 48, 49, 49, 49, 49, 50, 50, 50, 50, 51, 51, 51, 51, 52, 52,
52, 52, 52, 53, 53, 53, 53, 54, 54, 54, 54, 55, 55, 55, 55, 56, 56, 56, 56, 56,
57, 57, 57, 57, 58, 58, 58, 58, 59, 59, 59, 59, 60, 60, 60 };

volatile char Tick;

struct sHMS {
    unsigned hour:1;
    unsigned min:1;
    unsigned sec:1;
 };

struct sHMS PauseUnit;
struct sHMS PulseUnit;

// Interrupt Handler
void interrupt TMR (void)
{

  // Timer1 Interrupt - Freq = 2.00 Hz - Period = 0.500008 seconds
  if (TMR1IF == 1)  // timer 1 interrupt flag
  {
    Tick++ ;
    TMR1IF = 0;     // interrupt must be cleared by software
    TMR1IE = 1;     // reenable the interrupt
    TMR1H = 11;     // preset for timer1 MSB register
    TMR1L = 219;    // preset for timer1 LSB register
  }

}

void Init_PIC(void)
{   
    /* Initialize A/D */
    ADON = 1; /* Enable ADC */
    ADFM = 0; /* A/D Result Form Select bit  --> Right justified */
    VCFG = 0; /* Voltage Reference bit 1 = VREF pin 0 = VDD */
    CHS2 = 0; /* CHS<2:0>: Analog Channel Select bits : 000 = Channel 00 (AN0) */
    CHS1 = 0;
    CHS0 = 0;
    ADCS2 = 0; /* ADCS<2:0>: A/D Conversion Clock Select bits : x11 = FRC */
    ADCS1 = 0; /* (clock derived from a dedicated internal oscillator) */
    ADCS0 = 0;

    /* Initialize Ports */
    ANSEL  = 0x03;  /* set AN0 & AN1 as ADC inputs */
    TRISA2 = 0;     /* Set PORTA2 as output (default = input) */
    TRISA5 = 1;
    TRISA4 = 1;
    OPTION_REGbits.nRAPU = 0; // Enable the global RAPU bit
    WPUA0 = 0;      /* Weak Pull-up Register bits : */
    WPUA1 = 0;      /* 0 = Pull-up disabled (for A/D) */
    WPUA4 = 1;      /* 1 = Pull-up enabled for Loop & start with pause / pulse */
    WPUA5 = 1;

    //Timer1 Registers Prescaler= 8 - TMR1 Preset = 3035 - Freq = 2.00 Hz - Period = 0.500008 seconds
    T1CKPS1 = 1;   // bits 5-4  Prescaler Rate Select bits
    T1CKPS0 = 1;   // bit 4
    T1OSCEN = 0;   // bit 3 Timer1 Oscillator Enable Control bit 1 = off
    nT1SYNC = 1;    // bit 2 Timer1 External Clock Input Synchronization Control bit...1 = Do not synchronize external clock input
    TMR1CS = 0;    // bit 1 Timer1 Clock Source Select bit...0 = Internal clock (FOSC/4)
    TMR1H = 11;             // preset for timer1 MSB register
    TMR1L = 219;             // preset for timer1 LSB register


    // Interrupt Registers
    INTCON = 0;           // clear the interrupt control register
    TMR0IE = 0;        // bit5 TMR0 Overflow Interrupt Enable bit...0 = Disables the TMR0 interrupt
    TMR1IF = 0;            // clear timer1 interrupt flag TMR1IF
    TMR1IE  =   1;         // enable Timer1 interrupts
    TMR0IF = 0;        // bit2 clear timer 0 interrupt flag
    GIE = 1;           // bit7 global interrupt enable
    PEIE = 1;          // bit6 Peripheral Interrupt Enable bit...1 = Enables all unmasked peripheral interrupts

}

unsigned char ReadADC8(char ANx)
{
    ADCON0bits.CHS = ANx;               // channel must be defined as ADCON0bits.CHS
    ADCON0bits.GO = 1;              // start conversion
    while (ADCON0bits.GO);          // wait for conversion to finish
    return ADRESH;                  // returns only 8 MSBits (0..256)
}

unsigned char GetPause()
{
    return ADCbase60[ReadADC8(ADCPauseChannel)];  
}

unsigned char GetPulse()
{
                    
    return ADCbase60[ReadADC8(ADCPulseChannel)];  
}

void Delay1Sec(void)
{
    Tick = 0;
    TMR1ON = 1; 
    while (Tick != 2) {} // T = 0,5s
    TMR1ON = 0; 
}

void DelayHMS(char Timeout,  struct sHMS *Unit) {
    char j;
    char k;
    
    for (Timeout; Timeout != 0; Timeout--) {             // do "timeout" times...
        if (Unit->sec) {
            Delay1Sec();
        }                                                 
        
        if (Unit->min) {
            for (j=59; j!=0; j--) {
                Delay1Sec();
            }
        }                          
        
        if (Unit->hour) {
            for (j=59; j!=0; j--) {
                for (k=59; k!=0; k--) {
                    Delay1Sec();
                }
            }
        }                     
    }                                                                       
}

void GeneratePulse(char PauseDuration, char PulseDuration) {
    struct sHMS PauseUnit;
    struct sHMS PulseUnit;
    
    PulseUnit.hour=!JmpPulseHour;
    PulseUnit.min=!JmpPulseMin;
    PulseUnit.sec=!JmpPulseSec;
    
    PauseUnit.hour=!JmpPauseHour;
    PauseUnit.min=!JmpPauseMin;
    PauseUnit.sec=!JmpPauseSec;
    
    if (JmpStartWithPause == 0) {     // Start with a pause 
            DelayHMS(PauseDuration, &PauseUnit);
            Relay = 1;
            DelayHMS(PulseDuration, &PulseUnit);
            Relay = 0;
    }
    else {                             // Start with a pulse
            Relay = 1;
            DelayHMS(PulseDuration, &PulseUnit);
            Relay = 0;
            DelayHMS(PauseDuration, &PauseUnit);
    }
}

char main(void)
{
    Init_PIC();
    Relay=0;

    if (JmpLoop == 0)
        do{
            GeneratePulse(GetPulse(), GetPause());
        } while(1);
        else 
            GeneratePulse(GetPulse(), GetPause());
    SLEEP();
}
