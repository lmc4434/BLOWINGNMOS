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
extern BOOLEAN center_flag_hold;


//Value to easily turn on diff turning
BOOLEAN diff_turn = 0;

BOOLEAN debug = 0;
BOOLEAN debug2 = 0;
int mode_control = 0;
float wheelbase_length = 8.0;
float wheelbase_width = 5.5;
char temp[STR_SIZE];
float test = 0.0;
int stop_delay = 0;

float Kp = 0.6; //0.6
float Ki = 0.017; //0.15 //Large delay turn hard
float Kd = 0.35; //0.20 //Return to center and turn hard
float forward_speed = 0.50;  //Motor speed


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

void differential_turning(float amount){

	double turn_angle = find_angle(updateServoPosition(amount));

	if (turn_angle == 0){
		turn_angle = 0.00001;
	}
	
	
	float turning_radius = (wheelbase_length / tan(turn_angle));

	if (turn_angle < 0){//riht_turn
		float left = 3.1415 * (2.0 * turning_radius + (wheelbase_width / 2.0));
		float right = 3.1415 * (2.0 * turning_radius - (wheelbase_width / 2.0));
		float proportion = left/right;
		
		forward((forward_speed*0.8)/proportion, forward_speed*0.8);
		
	}else{//left_turn
		
		float left = 3.1415 * (2.0 * turning_radius + (wheelbase_width / 2.0));
		float right = 3.1415 * (2.0 * turning_radius - (wheelbase_width / 2.0));
		float proportion = right/left;
		
		forward(forward_speed*0.8, (forward_speed*0.8)/proportion);
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
			
			//DO NOT TOUCH, RUNS GREAT
			if(mode_control == 0){ 
				P2->OUT &= ~BIT2;		//BLUE off
				P2->OUT |= BIT0;		//RED on
				//Motor control settings
				stop_delay = 100000;
				forward_speed = 0.47;
				Kp = 0.6; 
				Ki = 0.017;
				Kd = 0.35;
				mode_control = 1;
				//END DO NOT TOUCH
				
				//Worked Faster and well
			} else if (mode_control == 1){
				P2->OUT &= ~BIT0;		//RED off
				P2->OUT |= BIT1;		//GREEN on
				//Motor control settings
				diff_turn = 0;
				stop_delay = 200000;
				forward_speed = 0.5;
				Kp = 0.6;
				Ki = 0.017; 
				Kd = 0.35;
				mode_control = 2;
				
				
			} else if (mode_control == 2){
				P2->OUT &= ~BIT1;		//GREEN off
				P2->OUT |= BIT2;		//BLUE on
				//Motor control settings
				diff_turn = 0;
				stop_delay = 350000;
				forward_speed = 0.55;
				Kp = 0.65;
				Ki = 0.017; 
				Kd = 0.35;
				mode_control = 3;
			}
		}
	}

    forward(forward_speed, forward_speed);
	
	
    while(1) {
			bin_enc();
			test = turn(PID());
			if(center_flag){
				forward(forward_speed, forward_speed);
			} else {
				if (center_flag_hold == 1){
					reverse_motors(-0.2);
					delay(stop_delay);
					center_flag_hold = 0;
				}
				if (diff_turn == 0){
					
				forward(forward_speed*0.8,forward_speed*0.8);
				}else{
					differential_turning(test);
				}}
				
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

		
