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
#include <stdio.h>

extern  unsigned char OLED_clr_data[1024];
extern unsigned char OLED_TEXT_ARR[1024];
extern unsigned char OLED_GRAPH_ARR[1024];
extern uint16_t line[128];
extern char str[STR_SIZE];
//global variables

//Low Index
int low_index;

//High Index
int high_index;

char ctr[STR_SIZE];
int switch_state = 0;
double slope1;
int tol;

void init(){
	//init display
	  uart0_init();
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
		TIMER_A2_PWM_DutyCycle(.075, 1); 
		INIT_Camera();
	  OLED_Init();
		OLED_display_on();
		OLED_display_clear();
		OLED_display_on();

	}

	
//Center of the graph should always be a slope of 0 if its on the right course
// so if the slope becomes + or -, have to turn the car
double generate_slope(int y2, int y1, int x2, int x1){
	double result = (y2 - y1)/ (x2 - x1);
	return result;
	
}


//Figure out if its a parabola - take index and value at a certain point and then the 
//exact point reflected over the axis of symmetry. If it is a slope of 0 +- a tol
// It is a parabola and it means its on carpet
int carpet_detection(int y2, int y1, int x2, int x1){
	double result = (y2 - y1)/ (x2 - x1);
	return result;

}

int main(void){
	
    init();

		 TIMER_A2_PWM_DutyCycle(.05, 1);
		 delay(200);
		while(1){
		camsequence();
		for (int i = 0; i < 128; i++){
					OLED_DisplayCameraData(line);
					
					if (slope1 <= generate_slope(line[high_index], line[low_index], high_index, low_index) - tol && slope1 <= generate_slope(line[high_index], line[low_index], high_index, low_index) + tol){
							//This Needs to Turn a direction
							TIMER_A2_PWM_Init((48000000/50/64), 0.1, 1);
							TIMER_A2_PWM_DutyCycle(.075, 1); //Centered
					}else if( slope1 >= generate_slope(line[high_index], line[low_index], high_index, low_index) - tol && slope1 >= generate_slope(line[high_index], line[low_index], high_index, low_index) + tol){
							//This Needs to Turn a direction
							TIMER_A2_PWM_Init((48000000/50/64), 0.1, 1);
							TIMER_A2_PWM_DutyCycle(.075, 1); //Centered
					}else{
							//This Needs to Turn a direction
							TIMER_A2_PWM_Init((48000000/50/64), 0.1, 1);
							TIMER_A2_PWM_DutyCycle(.075, 1); //Centered
						
						
					}if (slope1 <= carpet_detection(line[high_index], line[low_index], high_index, low_index) - tol && slope1 <= carpet_detection(line[high_index], line[low_index], high_index, low_index) + tol){
					}					
					sprintf(str, "%u\n\r", line[i]);
					uart0_put(str);

					
		}
	}
}
	 /*
	 
	 
	 
		TIMER_A2_PWM_DutyCycle(.075, 1); //Centered
	 
		TIMER_A0_PWM_Init(4800, 0.0, 1);
		TIMER_A0_PWM_Init(4800, 0.3, 2);
		TIMER_A0_PWM_Init(4800, 0.0, 3);
		TIMER_A0_PWM_Init(4800, 0.3, 4);

		delay(1000);*/

 
