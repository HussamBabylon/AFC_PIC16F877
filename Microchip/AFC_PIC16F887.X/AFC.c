/*
 * File:   AFC.c
 * Author: Hussam
 *
 * Created on March 23, 2023, 12:46 AM
 */


#include <xc.h>
#include "config.h"
#include "I2C_LCD.h"
#include <stdint.h>
#include <stdio.h>
#include <math.h>
//=================================
//-----[ Globals ]-----
int AN0RES = 0;
float Voltage;
//float Voltage2;
int Temperature;
int RTemperature;
//int Alarm;
int RAlarm;
//char* line1 = "Temp";
char* TempSTR[16];
int tempMin = 75;
int tempMax = 130;
int fanSpeed;
int fanLCD;
char* fanSTR[16];
float RT, VR, ln, T0, VRT;
int i, i2;
int c = 130;
int c2 = 130;
#define PWM            RC2// use the name OUT for RC2 pin.
#define Buzzer           RB1      // use the name OUT for RC2 pin.

#define RT0 5684  // 25°C Degree In 5684 ?  .
#define B 3965    // Beta sabiti
//--------------------------------------

#define VCC 5     //Supply voltage
#define R 5500  //R=5.4 K?



unsigned long CCPR = 0; // holds the value needed to be put in CCP's registers.
unsigned long current_period = 130; // holds the period that timer1 will use.
//unsigned long CCPR = 0; // holds the value needed to be put in CCP's registers.
//float current_period = 0; // holds the period that timer1 will use.
const unsigned long total_period = 2500; // 20ms for 50hz frequency.
//=================================

//=================================
//-----[ Functions Prototypes ]----
void ADC_Init();
uint16_t ADC_Read(uint8_t);
void PWM1_Set_Duty(uint16_t);
void interrupt ISR();
float map(float x, float in_min, float in_max, float out_min, float out_max);
int GetTemp();
int Alarms();
//=================================

void main(void) {

    I2C_Master_Init();
    LCD_Init(0x4E); // Initialize LCD module with I2C address = 0x4E
    LCD_Clear();
    ADC_Init();

    //====================================================================
    TRISCbits.TRISC2 = 0; // port c is output.
    TRISBbits.TRISB1 = 0; // port c is output.
    Buzzer = 0;
    PWM = 1; // port c = 0.

    //    TRISC = 0; // port c is output.
    //    PORTC = 1; // port c = 0.

    T1CON = 0x30; // timer1 uses prescaler value of 8 and it is off.
    TMR1H = 0; // timer1 registers have 0 (clear).
    TMR1L = 0;

    CCP1CON = 0x0b; // set CCP module to compare mode and trigger special event when interrupt happens.
    CCPR = 0; // load 0 in CCPR.
    CCP1IF = 0; // clear CCP1 interrupt flag.
    CCP1IE = 1; // enable CCP1 interrupt. 
    INTCON = 0xC0; // enable global and peripheral interrupt. 
    T1CON = 0x31; // start timer1 with the same settings like before.
    //====================================================================

    LCD_Clear();
    LCD_Set_Cursor(1, 1);
    LCD_Write_String("  BBBBBBBBBBB");
    LCD_Set_Cursor(2, 1);
    LCD_Write_String("  AAAAAAAAAAA");
    Buzzer = 1;
    __delay_ms(5000);
    Buzzer = 0;
    LCD_Clear();
    LCD_Set_Cursor(1, 1);
    LCD_Write_String("     Fan Test");
    for (i; i <= 150; i++) {
        current_period = c;
        c = c + 5;
        __delay_ms(10);
    }
    current_period = c;
    __delay_ms(5000);
    i = 0;
    c = 0;
    current_period = 130;

    LCD_Clear();
    LCD_Set_Cursor(1, 1);
    LCD_Write_String("     Fan Ok");
    __delay_ms(2000);
    LCD_Clear();

    while (1) {


        RAlarm = Alarms();
//        sprintf(TempSTR, "%3d C", RAlarm);
//        //    sprintf(fanSTR, "%3d", fanLCD);
//            LCD_Set_Cursor(1, 1);
//            LCD_Write_String("Temp Is : ");
//            LCD_Set_Cursor(1, 11);
//            LCD_Write_String(TempSTR);
//            LCD_Set_Cursor(2, 1);
        //        RTemperature = GetTemp();
        //        
        //        sprintf(TempSTR, "%3d C", RTemperature);
        //        sprintf(fanSTR, "%4d", RAlarm);
        //        LCD_Set_Cursor(1, 1);
        //        LCD_Write_String("Temp Is : ");
        //        LCD_Set_Cursor(1, 11);
        //        LCD_Write_String(TempSTR);
        //        LCD_Set_Cursor(2, 1);
        //        LCD_Write_String("Voltage:");
        //        LCD_Set_Cursor(2, 10);
        //        LCD_Write_String(fanSTR);
        //        LCD_Set_Cursor(2, 14);
        //        LCD_Write_String("%");
        //        __delay_ms(40);

        while (RAlarm >= 20 && RAlarm <= 700) {
            RTemperature = GetTemp();
            RAlarm = Alarms();
            if ((RTemperature >= tempMin) && (RTemperature <= tempMax)) {
                //            PWM = 1;
                fanSpeed = map(RTemperature, tempMin, tempMax, 712, 2375);
                fanLCD = map(RTemperature, tempMin, tempMax, 0, 100);
                current_period = fanSpeed;
                Buzzer = 0;
            }
            if (RTemperature > tempMax) {
                Buzzer = 0;
                current_period = 2375;
                fanLCD = 100;
            }
            if (RTemperature < tempMin) {
                Buzzer = 0;
                current_period = 130;
                fanLCD = 0;
            }
            sprintf(TempSTR, "%3d C", RTemperature);
            sprintf(fanSTR, "%3d", fanLCD);
            LCD_Set_Cursor(1, 1);
            LCD_Write_String("Temp Is : ");
            LCD_Set_Cursor(1, 11);
            LCD_Write_String(TempSTR);
            LCD_Set_Cursor(2, 1);
            LCD_Write_String("FanSpeed:");
            LCD_Set_Cursor(2, 10);
            LCD_Write_String(fanSTR);
            LCD_Set_Cursor(2, 14);
            LCD_Write_String("%");
            __delay_ms(250);

        }

        while (RAlarm >= 800) {
            RAlarm = Alarms();
            Buzzer = 1;
            for (i; i <= 449; i++) {
                current_period = c;
                c = c + 5;
                __delay_ms(30);
            }
            current_period = c;
            LCD_Clear();
            LCD_Set_Cursor(1, 1);
            LCD_Write_String("Wire  Cut !!");
            LCD_Set_Cursor(2, 1);
            LCD_Write_String("Wire  Cut !!");
            __delay_ms(10);
            LCD_Clear();
        }
//
//        while (RAlarm <= 20) {
//            RAlarm = Alarms();
//            Buzzer = 1;
//            for (i2; i2 <= 449; i2++) {
//                current_period = c2;
//                c2 = c2 + 5;
//                __delay_ms(30);
//            }
//            current_period = c2;
//            LCD_Clear();
//            LCD_Set_Cursor(1, 1);
//            LCD_Write_String("Sensor Error !!");
//            LCD_Set_Cursor(2, 1);
//            LCD_Write_String("Sensor Error !!");
//            __delay_ms(10);
//            LCD_Clear();
//        }

    }
  //  return;
}

//=================================
//--------[ ADC Routines ]---------

void ADC_Init() {
    ADCON0 = 0x41; // Turn ADC ON, Select AN0 Channel, ADC Clock = Fosc/8
    ADCON1 = 0x80; // All 8 Channels Are Analog, Result is "Right-Justified"
    // ADC Clock = Fosc/8
}

uint16_t ADC_Read(uint8_t ANC) {
    if (ANC < 0 || ANC > 7) // Check Channel Number Validity
    {
        return 0;
    }
    ADCON0 &= 0x11000101; // Clear The Channel Selection Bits
    ADCON0 |= ANC << 3; // Select The Required Channel (ANC)
    // Wait The Aquisition Time
    __delay_us(30); // The Minimum Tacq = 20us, So That should be enough
    GO_DONE = 1; // Start A/D Conversion
    while (ADCON0bits.GO_DONE); // Polling GO_DONE Bit
    // Provides Delay Until Conversion Is Complete
    return ((ADRESH << 8) + ADRESL); // Return The Right-Justified 10-Bit Result
}

void interrupt ISR() {

    // Declaration Interrupt Function 
    //=======================================================
    // Check The Flag Bit
    if (CCP1IF == 1) { // if CCP compare interrupt flag is set


        if ((current_period >= 0) && (current_period <= total_period)) { // if duty is > 0% AND < 100% then:

            if (PWM == 1) { // if the output was 1 -> was "on-time".         
                PWM = 0; // set output to 0 in order to achieve "off-time".
                CCPR = total_period - current_period; // make it time for "off-time", off-time = full time - on time.
            } else { // if the output was 0 -> was "off-time".
                PWM = 1; // set output to 1 in order to achieve "on-time"
                CCPR = current_period; // make it time for "on-time".
            }
        }
        //            else {
        //            if (current_period == total_period) {
        //                PWM = 1;
        //            } // if duty = 100%, then output 1 all the time.
        //            if (current_period == 0) {
        //                PWM = 0;
        //            } // if duty = 0%, then output 0 all the time.
        //        }


        // now set the value of CCPR into CCP module's registers:

        CCPR1H = CCPR >> 8; // right-shift CCPR by 8 then load it into CCPR1H register (load higher byte).
        CCPR1L = CCPR; // put the lower byte of CCPR in CCPR1L register.
        CCP1IF = 0; // reset CCP1 interrupt flag.
    }
}

// Adjestment of variable Range

float map(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

int GetTemp() {
    T0 = 25 + 273.15; //Temperature T0 from datasheet, conversion from Celsius to kelvin
    VRT = ADC_Read(0);
    VRT = (0.0048828) * VRT; //Conversion to voltage  0048828  4.7 v 0045898  4.3 0041992 
    VR = VCC - VRT;
    RT = VRT / (VR / R); //Resistance of RT
    ln = log(RT / RT0);
    Temperature = (1 / ((ln / B) + (1 / T0))); //Temperature from thermistor
    Temperature = Temperature - 273.15; //Conversion to Celsius
    //        __delay_ms(250);
    __delay_us(125);
    return Temperature;

}

int Alarms() {
    //         Read The ADC
    AN0RES = ADC_Read(0); // Read Analog Channel 0
    // Calculate The Temperature
    //        Voltage2 = AN0RES * 0.0048828; // 0032226
    //        Alarm = Voltage2 / 0.01;
    __delay_us(125);
    return AN0RES;
    // Convert The Temperature From Float To String
}
//========================================================= 