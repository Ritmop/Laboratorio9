/* 
 * File:   Lab8.c
 * Device: PIC16F887
 * Author: Judah Pérez - 21536
 *Compiler: XC8 (v2.40)
 * 
 * Program: UART Serial Com
 * Hardware:
 * 
 * Created: April 17, 2023
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

uint8_t ADval; //Analog-Digital converted value
uint8_t valid = 0;
        
/*-------------------------------- PROTOTYPES --------------------------------*/
void setup(void);
void config_ADC(void);
void initUART(void);

void potRead(void);
void outputVal(void);
void SerialPrint(char string[], int lenght);

/*------------------------------- RESET VECTOR -------------------------------*/

/*----------------------------- INTERRUPT VECTOR -----------------------------*/
void __interrupt() isr(void){
    if(ADIF){
        //ADC
        ADval = ADRESH;
        ADIF = 0;   //Reset flag        
    }
    return;
}

/*--------------------------- INTERRUPT SUBROUTINES --------------------------*/

/*---------------------------------- TABLES ----------------------------------*/

/*----------------------------------- SETUP ----------------------------------*/

/*----------------------------------- MAIN -----------------------------------*/
int main(void) {
    setup();
    config_ADC();
    initUART();
    RA1 = 0;
    while(1){
        //Loop
        valid = 0;       
        //Mostrar mensaje
        SerialPrint("\r1. Leer Pot \r2. Enviar ASCII\r",30);
        while(!valid){
            //Recolectar input
            if(BAUDCTLbits.RCIDL == 1){
                //Verificar input
                PORTD = RCREG;
                    if(PORTD == '1' || PORTD == '2')
                        valid = 1;
                    else
                        SerialPrint("El caracter ingresado no  es valido.\r",37);                
            }
        }
        
        if(PORTD == '1'){
            potRead(); //Capture pot value
            RA0 = 1; //Show pot value on display
            SerialPrint("El valor del potenciometro es: ",31);
        }
        if(PORTD == '2'){
            potRead(); //Capture pot value
            RA0 = 0; //Show pot value on PORTB
        }
        outputVal();  //Display value
    }
}
/*-------------------------------- SUBROUTINES -------------------------------*/
void setup(void){
   //I/O CONFIG
    ANSEL = 0b1;
    TRISAbits.TRISA0 = 1; //RA0 as analog input
    TRISAbits.TRISA1 = 0; //RA1 as digital output    
    ANSELH = 0;
    
    TRISD = 0;  //ASCII Output
    PORTD = 0;
    TRISB = 0;  //ASCII Output
    PORTB = 0;

    //OSCILLATOR CONFIG
    OSCCONbits.IRCF = 0b111;  //Internal clock frequency 8MHz
    SCS = 1;
    
    //INTERRUPT CONFIG
    GIE  = 1;   //Global Interrupt Enable
    PEIE = 1;
    PIE1bits.ADIE = 1;  //ADC Interrupt Enable
    
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

void initUART(void){
    //12.1.1.6 Asynchronous Transmission Set-up
    //12.1.2.8 Asynchronous Reception Set-up
    //Boud Rate = 9600 / %Error = 0.16%
    SPBRG = 12;
    SPBRGH = 0;
    BRGH = 0;   //TABLE 12-3: BAUD RATE FORMULAS
    BRG16 = 0;  //8bit boud rate generator
    //Asynchronous serial port
    SYNC = 0;
    SPEN = 1;
    //Enable transmission
    TXEN = 1;
    TXIF = 0;   //Clear flag
    //Enable reception
    CREN = 1;
    return;
}

void potRead(void){    
    if (ADCON0bits.GO == 0)
        ADCON0bits.GO = 1;
    __delay_us(5);
    //Value stored on interrupt
    return;
}

void outputVal(void){
    if(RA1 == 0)    //Display on PORTB
        PORTB = ADval;
    else{ //Display on terminal
        if(TXSTAbits.TRMT == 1){                    
            TXREG = ADval;
        }
    }       
    return;
}

void SerialPrint(char string[], int lenght){
    int i = 0;
    while(i <= lenght){
        __delay_ms(5);
        if(TXSTAbits.TRMT == 1){
            TXREG = string[i];
            i++;
        }
    }
    if(TXSTAbits.TRMT == 1)
        TXREG = '\n';   //Send null char to end string4
    return;
    __delay_ms(100);
}