/*
 * TI_main.c
 * 
 * Author: Logan Culver and Garrett Brindle 
 */
 
#include "msp.h"
#include "uart.h"
#include "timerA.h"
#include "motors.h"
#include "servo.h"
#include "led.h"
#include "controlpins.h"
#include "adc.h"
#include "timer32.h"
#include "cortexm.h"
#include "switches.h"
#include "systicktimer.h"
#include "camera.h"
#include "oled.h"
#include "TI_Main.h"
#include "PID.h"
#include <stdio.h>



extern unsigned char OLED_clr_data[1024];
extern unsigned char OLED_TEXT_ARR[1024];
extern unsigned char OLED_GRAPH_ARR[1024];
//Set up OLED variables
unsigned char state1[16] = "State 1";
unsigned char state2[16] = "State 2";
unsigned char state3[16] = "State 3";
extern uint16_t line[128];
extern char str[STR_SIZE];
extern uint16_t SmoothData[128];
char temp[STR_SIZE];
float test = 0.0;
extern BOOLEAN center_flag;
BOOLEAN debug = 0;


BOOLEAN carpet_detection() {
    if (line[64] < 6000){
			return TRUE;
}
}

//END PID SETUP


void init() {
    uart2_init();
		uart2_put("Bluetooth on");
    P3->SEL0 &= ~BIT6;
    P3->SEL0 &= ~BIT7;
    P3->SEL1 &= ~BIT6;
    P3->SEL1 &= ~BIT7;
    P3->DIR |= BIT6;
    P3->DIR |= BIT7;
    P3->OUT |= BIT6;
    P3->OUT |= BIT7;
    Switch1_Init();
    Switch2_Init();
    TIMER_A2_PWM_Init((48000000/50/64), 0.1, 1);
    TIMER_A2_PWM_DutyCycle(0.075, 1); 
    INIT_Camera();
    OLED_Init();
    OLED_display_on();
    OLED_display_clear();
    OLED_display_on();
}



void forward(float left_motor, float right_motor) {
    TIMER_A0_PWM_Init(4800, 0.0, 1);
    TIMER_A0_PWM_Init(4800, left_motor, 2);
    TIMER_A0_PWM_Init(4800, 0.0, 3);
    TIMER_A0_PWM_Init(4800, right_motor, 4);
}

void stop_motors(void) {
    TIMER_A0_PWM_Init(4800, 0.0, 1);
    TIMER_A0_PWM_Init(4800, 0.0, 2);
    TIMER_A0_PWM_Init(4800, 0.0, 3);
    TIMER_A0_PWM_Init(4800, 0.0, 4);
}
void reverse_motors(float speed){
    TIMER_A0_PWM_Init(4800, speed, 1);
    TIMER_A0_PWM_Init(4800, 0.0, 2);
    TIMER_A0_PWM_Init(4800, speed, 3);
    TIMER_A0_PWM_Init(4800, 0.0, 4);
}


float turn(float amount){
    TIMER_A2_PWM_DutyCycle(amount, 1); 
		return amount;
}


int main(void) {

    init();
    delay(200);
	
    forward(0.25,0.25);
    while(1) {

			test = turn(PID());
			if(center_flag){
				forward(0.50,0.50);
			} else {
				if(test >= 0.8){
					forward(0.3, 0.45);
				} else if(test <= 0.7){
					forward(0.45, 0.3);
				} else {
					forward(0.50,0.50);
				}
				
			}
			
			if(debug){
				uart2_put("Center Val: ");
				sprintf(temp,"%i\n\r", find_center());
				uart2_put(temp);
			}
					if(debug){
					uart2_put("I EAT CARPET : ");
					}

        }
    }
