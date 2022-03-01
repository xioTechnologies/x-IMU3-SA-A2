/**
 * @file main.c
 * @author Seb Madgwick
 * @brief Main file.
 *
 * Device:
 * PIC10F322
 *
 * Compiler:
 * XC8 v2.35
 */

//------------------------------------------------------------------------------
// Configuration bits

#pragma config FOSC = INTOSC    // Oscillator Selection bits (INTOSC oscillator: CLKIN function disabled)
#pragma config BOREN = ON       // Brown-out Reset Enable (Brown-out Reset enabled)
#pragma config WDTE = SWDTEN    // Watchdog Timer Enable (WDT controlled by the SWDTEN bit in the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = ON       // MCLR Pin Function Select bit (MCLR pin function is MCLR)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config LVP = ON         // Low-Voltage Programming Enable (Low-voltage programming enabled)
#pragma config LPBOR = ON       // Brown-out Reset Selection bits (BOR enabled)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (Vbor), low trip point selected.)
#pragma config WRT = OFF        // Flash Memory Self-Write Protection (Write protection off)

//------------------------------------------------------------------------------
// Includes

#include "Delay.h"
#include <stdbool.h>
#include "UartTX.h"
#include <xc.h>

//------------------------------------------------------------------------------
// Function declarations

static uint8_t GetAdcResult(void);
static void PrintVoltage(const uint8_t result);

//------------------------------------------------------------------------------
// Functions

void main(void) {

    // Select 16 MHz oscillator
    OSCCONbits.IRCF = 0b111;

    // Configure TX output
    ANSELAbits.ANSA1 = 0;
    TRISAbits.TRISA1 = 0;
    LATAbits.LATA1 = 1;

    // Enable ADC
    ADCONbits.ADCS = 0b101; // TAD = 1 us
    ADCONbits.ADON = 1;

    // Print firmware version
    Delay(DelayPeriod512ms);
    UartTXString("x-IMU3-SA-A2 v1.0\r\n");

    // Main program loop
    while (true) {

        // Begin channel 1 acquisition
        ADCONbits.CHS = 0b010; // AN2

        // Delay for sample period
        Delay(DelayPeriod8ms);

        // Channel 1 conversion
        uint8_t result = GetAdcResult();

        // Begin channel 2 acquisition
        ADCONbits.CHS = 0b000; // AN0

        // Print channel 1 voltage
        PrintVoltage(result);

        // Print comma
        UartTXByte(',');

        // Channel 2 conversion
        result = GetAdcResult();

        // Print channel 2 voltage
        PrintVoltage(result);

        // Print termination
        UartTXByte('\r');
        UartTXByte('\n');
    }
}

/**
 * @brief Performs ADC conversion and returns result.
 * @return ADC result.
 */
static uint8_t GetAdcResult(void) {
    ADCONbits.GO_nDONE = 1;
    while (ADCONbits.GO_nDONE == 0);
    return ADRES;
}

/**
 * @brief Prints ADC result as voltage.
 */
static void PrintVoltage(const uint8_t result) {
    uint16_t value = result * (28000 / UINT8_MAX); // 2.8 V reference
    UartTXByte('0' + (uint8_t) (value / 10000));
    value %= 10000;
    UartTXByte('.');
    UartTXByte('0' + (uint8_t) (value / 1000));
    value %= 1000;
    UartTXByte('0' + (uint8_t) (value / 100));
}

//------------------------------------------------------------------------------
// End of file
