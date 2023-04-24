/* 
 * File:   Lab9.c
 * Device: PIC16F887
 * Author: Judah Pérez - 21536
 *Compiler: XC8 (v2.40)
 * 
 * Program: EEPROM
 * Hardware:
 * 
 * Created: April 24, 2023
 * Last updated:
 */

/*--------------------------------- LIBRARIES --------------------------------*/
#include <xc.h>
#include <PIC16F887.h>
#include <stdio.h>
#include <stdlib.h>

/*---------------------------- CONFIGURATION BITS ----------------------------*/
// CONFIG1
#pragma config FOSC = INTRC_NOCLKOUT// Oscillator Selection bits (INTOSCIO oscillator: I/O function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled and can be enabled by SWDTEN bit of the WDTCON register)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RE3/MCLR pin function select bit (RE3/MCLR pin function is digital input, MCLR internally tied to VDD)
#pragma config CP = OFF         // Code Protection bit (Program memory code protection is disabled)
#pragma config CPD = OFF        // Data Code Protection bit (Data memory code protection is disabled)
#pragma config BOREN = OFF      // Brown Out Reset Selection bits (BOR disabled)
#pragma config IESO = OFF       // Internal External Switchover bit (Internal/External Switchover mode is disabled)
#pragma config FCMEN = OFF      // Fail-Safe Clock Monitor Enabled bit (Fail-Safe Clock Monitor is disabled)
#pragma config LVP = OFF        // Low Voltage Programming Enable bit (RB3 pin has digital I/O, HV on MCLR must be used for programming)

// CONFIG2
#pragma config BOR4V = BOR40V   // Brown-out Reset Selection bit (Brown-out Reset set to 4.0V)
#pragma config WRT = OFF        // Flash Program Memory Self Write Enable bits (Write protection off)

/*----------------------- GLOBAL VARIABLES & CONSTANTS -----------------------*/
#define _XTAL_FREQ 8000000  //Constante para delay
        
/*-------------------------------- PROTOTYPES --------------------------------*/
void setup(void);
void config_ADC(void);

void ioc_portB(void);

void potRead(void);

/*------------------------------- RESET VECTOR -------------------------------*/

/*----------------------------- INTERRUPT VECTOR -----------------------------*/
void __interrupt() isr(void){
    if(ADIF){
        //ADC
        PORTC = ADRESH;
        ADIF = 0;   //Reset flag        
    }
    if(RBIF){
        ioc_portB();
        RBIF = 0;
    }
    return;
}

/*--------------------------- INTERRUPT SUBROUTINES --------------------------*/
void ioc_portB(void){
    if(!RB1){
        
    }        
    else if(!RB0)
        SLEEP();
}

/*---------------------------------- TABLES ----------------------------------*/

/*----------------------------------- SETUP ----------------------------------*/

/*----------------------------------- MAIN -----------------------------------*/
int main(void) {
    setup();
    config_ADC();
    while(1){
        //Loop
        potRead();
        PORTE++;
    }
}
/*-------------------------------- SUBROUTINES -------------------------------*/
void setup(void){
   //I/O CONFIG
    ANSEL = 0b1;
    ANSELH = 0; //RA0 as analog
    
    TRISA = 0b1; //RA0 input pot
    TRISB = 0b111; //RB0-RB2 input buttons
    
    TRISC = 0;  //Pot read output
    PORTC = 0;
    PORTA = 0;
    PORTB = 0;
    
    TRISE = 0;
    PORTE = 0;
    
    //PORTB weak pull-up
    nRBPU = 0;   //Enable PORTB pull-up
    WPUB = 0b111;//on RB0 - RB2

    //OSCILLATOR CONFIG
    OSCCONbits.IRCF = 0b111;  //Internal clock frequency 8MHz
    SCS = 1;
    
    //INTERRUPT CONFIG
    GIE  = 1;   //Global Interrupt Enable
    RBIE = 1;   //PORTB Change Interrupt Enable
    IOCB = 0b111;
    
    PEIE = 1;   //Peripheral Interrupt Enable
    PIE1bits.ADIE = 1;   //ADC Interrupt Enable
    
    return;
}

void config_ADC(void){
    //ADCON1
    ADCON1bits.ADFM  = 0;   //Left justified
    ADCON1bits.VCFG0 = 0;   //Vref+ = Vdd
    ADCON1bits.VCFG1 = 0;   //Vref- = Vss
    //ADCON0
    ADCON0bits.ADCS = 0b010;//Conversion clock = Fosc/32
    ADCON0bits.CHS   = 0;   //ADC input from AN0
    ADCON0bits.ADON  = 1;   //Enable ADC Module
    return;
}

void potRead(void){    
    if (ADCON0bits.GO == 0)
        ADCON0bits.GO = 1;
    __delay_us(50);
    //Value stored on interrupt
    return;
}