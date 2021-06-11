/*
 * File:   main.c
 * Author: Quentin
 *
 * Created on May 18, 2021, 9:03 PM
 */


#define MANUAL
#ifdef MANUAL
#define _XTAL_FREQ 4000000
#endif
#include <xc.h>

// CONFIG
// CONFIG
#pragma config FOSC = INTOSCCLK // Oscillator Selection bits (INTOSC oscillator: CLKOUT function on RA6/OSC2/CLKOUT pin, I/O function on RA7/OSC1/CLKIN)
#pragma config WDTE = OFF       // Watchdog Timer Enable bit (WDT disabled)
#pragma config PWRTE = ON      // Power-up Timer Enable bit (PWRT disabled)
#pragma config MCLRE = OFF      // RA5/MCLR/VPP Pin Function Select bit (RA5/MCLR/VPP pin function is digital input, MCLR internally tied to VDD)
#pragma config BOREN = OFF      // Brown-out Detect Enable bit (BOD disabled)
#pragma config LVP = OFF        // Low-Voltage Programming Enable bit (RB4/PGM pin has digital I/O function, HV on MCLR must be used for programming)
#pragma config CPD = OFF        // Data EE Memory Code Protection bit (Data memory code protection off)
#pragma config CP = OFF         // Flash Program Memory Code Protection bit (Code protection off)

#define LED_LOCKED RA1
#define TRIS_LED_LOCKED TRISA1

#define LED_ARMED RA2
#define TRIS_LED_ARMED TRISA2

#define LED_LAUCHED_DETECTED RA3
#define TRIS_LAUCHED_DETECTED TRISA3

#define ARM_BUTTON RB4
#define LAUNCH_DETECT RB5
#define TRIS_LAUNCH_DETECT TRISB5

#define SERVO_MOTOR RB0
void deploy() {

    for (int j=0;j<20;++j) {
    SERVO_MOTOR = 1;
    const int val = 1060; // open position
    __delay_us(val);

    SERVO_MOTOR = 0;
    int i = 20000 - val;
    for (; i >= 1000; i -= 1000) {
        __delay_us(1000);
    }
    __delay_us(val % 1000);
    }

}

void lock() {
    for (int j=0;j<20;++j) {
        SERVO_MOTOR = 1;
        const int val = 2400; // locked position
        __delay_us(val);

        SERVO_MOTOR = 0;
        int i = 20000 - val;
        for (; i >= 1000; i -= 1000) {
            __delay_us(1000);
        }
        __delay_us(val % 1000);
    }
}

void notifyCommandAck() {
    TMR2ON = 1;
    CCPR1L = 60;
    __delay_ms(250);
    TMR2ON = 0;
}

void notifyStateChanged() {    
    __delay_ms(250);
    TMR2ON = 1;
    __delay_ms(250);
    TMR2ON = 0;
    
}

// 125 * 4 * 16 / ( 4 000 000)

enum State {
    Idle, Closed, Armed, ProgramSeconds
};

void gotoIdleState(enum State *state) {
    LED_LOCKED=0;
    LED_ARMED=0;
    LED_LAUCHED_DETECTED=0;
    *state = Idle;
}

void gotoLockedState(enum State *state) {
    notifyCommandAck();
    lock();                                                          
    *state = Closed;
    LED_LOCKED = 1;
    notifyStateChanged();   
}

int main() {
    enum State state = Idle;
    CMCONbits.CM = 0b111;
     
    //PCONbits.OSCF = 0;
    //#define _XTAL_FREQ 48000
    
    CCP1M3 = 1; // PWM mode
    CCP1M2 = 1; // PWM mode
    PR2 = 249; // maximal period
    CCPR1L = 0;
    CCP1X = 0;
    CCP1Y = 0;
    //Prescaler to 1:1
    T2CKPS1 = 0;
    T2CKPS0 = 0;
    TMR2ON = 1;
    TRISB3 = 0;
    
    int launchDetected = 0;
    
    nRBPU = 0; // Tie RB pins to a pull up
    
    TRISB0 = 0; //RB0 as Output PIN
    TRISB3 = 0; //RB0 as Output PIN
    
    TRIS_LED_LOCKED = 0; //RB6 as Output PIN
    TRIS_LED_ARMED = 0; //RB5 as Output PIN
    TRIS_LAUCHED_DETECTED = 0; //RB4 as Output PIN for motor
        
    TRISB4 = 1; // Push button
    TRIS_LAUNCH_DETECT = 1; // Launch detection trigger
    
    LED_LOCKED=0;
    LED_ARMED=0;
    LED_LAUCHED_DETECTED=0;

    int buttonPushed = 0;
    
    int buttonPressed = 0;
    int buttonPressedPeriod=0;
    int buttonWasPressed = 0;
    
    int buttonClickedPendingEvent = 0;
    int buttonClickedEvent = 0;
    
    int buttonLongClickedEvent = 0;
    
    int delay = 2000;
    int displayTime=0;
    
    while (1) {
        buttonPushed = !ARM_BUTTON;
                
        int buttonClicked = !buttonWasPressed && buttonPushed;
        int buttonReleased = buttonWasPressed && !buttonPushed;
        buttonClickedEvent = 0;
        buttonLongClickedEvent =0;
        
        if (buttonClicked) {
            buttonPressedPeriod=0;
            buttonClickedPendingEvent=1;
        }
        else if (buttonReleased) {
            if (buttonClickedPendingEvent) {
                buttonClickedEvent = 1;
                buttonClickedPendingEvent=0;
            }
        }
        else if (buttonPushed) {
            ++buttonPressedPeriod;
            if (buttonPressedPeriod > 50) {
                buttonLongClickedEvent = 1;
                buttonClickedPendingEvent=0;
                buttonPressedPeriod=0;
            }
        }
        buttonWasPressed = buttonPushed;
        if (state != Armed) {
            __delay_ms(20);
        }
        
        int closeButton = buttonClickedEvent;
        
        launchDetected = !LAUNCH_DETECT;
        switch (state) {
            case Idle: {
                if (closeButton) {
                    gotoLockedState(&state);
                }
                else if (buttonLongClickedEvent) {
                    LED_LAUCHED_DETECTED=1;
                    state = ProgramSeconds;
                    displayTime=0;
                }
                break;
            }
            case Closed: {
                if (closeButton) {
                    notifyCommandAck();
                    state = Armed;
                    LED_ARMED = 1;
                    notifyStateChanged();                   
                 }
                break;            
            }
            case Armed: {
                if (launchDetected) {
                    LED_LAUCHED_DETECTED=1;
                    __delay_ms(2600);
                    deploy();
                    gotoIdleState(&state);  
                }
                break;
            }
            case ProgramSeconds:{
                if (buttonLongClickedEvent) {
                    gotoIdleState(&state);
                }
                if (buttonClickedEvent) {
                    delay = (delay + 1000) % 6000;
                    displayTime = 0;
                }
                displayTime += 20;
                if (displayTime % 1000 < 750 )
                {
                    if (displayTime < delay) {
                        LED_LOCKED = 1;
                    }
                    else {
                        LED_LOCKED = 0;
                    }
                }
                else {
                    LED_LOCKED = 0;
                }
                displayTime = displayTime % 6000;
                break;
            }
            
        }
        
    }

    return 0;
}
