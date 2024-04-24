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
#include "PID.h"
#include "math.h"
#include <stdio.h>


extern unsigned char OLED_clr_data[1024];
extern unsigned char OLED_TEXT_ARR[1024];
extern unsigned char OLED_GRAPH_ARR[1024];
extern uint16_t line[128];
extern char str[STR_SIZE];
extern uint16_t SmoothData[128];

extern BOOLEAN center_flag;

BOOLEAN debug = 0;
int mode_control = 0;
float wheelbase_length = 8.0;
float wheelbase_width = 5.5;
char temp[STR_SIZE];
float test = 0.0;


BOOLEAN carpet_detection() {
    if (line[64] < 6000){
			return TRUE;
}
}

//END PID SETUP


void init() {
	// UART
    uart2_init();
		uart2_put("Bluetooth on");
	uart0_init();
	uart0_put("Init");
	// MOTORS
    P3->SEL0 &= ~BIT6;
    P3->SEL0 &= ~BIT7;
    P3->SEL1 &= ~BIT6;
    P3->SEL1 &= ~BIT7;
    P3->DIR |= BIT6;
    P3->DIR |= BIT7;
    P3->OUT |= BIT6;
    P3->OUT |= BIT7;
	// SWITCHES
    Switch1_Init();
    Switch2_Init();
	//LEDS
		LED1_Init();
		LED2_Init();
		LED1_Off();
		LED2_Off();
	// TIMERS
    TIMER_A2_PWM_Init((48000000/50/64), 0.1, 1);
    TIMER_A2_PWM_DutyCycle(0.075, 1); 
	// CAMERA
    INIT_Camera();
	// OLED
	/*
    OLED_Init();
    OLED_display_on();
    OLED_display_clear();
    OLED_display_on();
		*/
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

void differential_turning(void){
	
	float forward_speed = 0.60;
	
	double turn_angle = find_angle(PID());
	
	if (turn_angle == 0){
		turn_angle = 0.00001;
	}
	
	float turning_radius = (wheelbase_length / tan(turn_angle));
	
	if (turn_angle > 0){//riht_turn
		float left = 3.1415 * (2.0 * turning_radius + (wheelbase_width / 2.0));
		float right = 3.1415 * (2.0 * turning_radius - (wheelbase_width / 2.0));
		float proportion = left/right;
		
		forward(forward_speed/proportion, forward_speed);
		
	}else{//left_turn
		
		float left = 3.1415 * (2.0 * turning_radius + (wheelbase_width / 2.0));
		float right = 3.1415 * (2.0 * turning_radius - (wheelbase_width / 2.0));
		float proportion = right/left;
		
		forward(forward_speed, forward_speed/proportion);
	}
}


int main(void) {

		init();

	if(debug){
		OLED_DisplayCameraData(SmoothData);
	}
	while(!Switch1_Pressed()){
		if(Switch2_Pressed()){
			delay(4000000);
			if(mode_control == 0){
				P2->OUT &= ~BIT2;		//BLUE off
				P2->OUT |= BIT0;		//RED on
				//Motor control settings
				
				mode_control = 1;
			} else if (mode_control == 1){
				P2->OUT &= ~BIT0;		//RED off
				P2->OUT |= BIT1;		//GREEN on
				//Motor control settings
				
				mode_control = 2;
			} else if (mode_control == 2){
				P2->OUT &= ~BIT1;		//GREEN off
				P2->OUT |= BIT2;		//BLUE on
				//Motor control settings
				
				mode_control = 0;
			}
		}
	}
	
		uart0_put("Go");
    forward(0.60,0.60);
    while(1) {
			bin_enc();
			test = turn(PID());
			if(center_flag){
				forward(0.60,0.60);
			} else {
				forward(0.40,0.40);
				//differential_turning();
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
