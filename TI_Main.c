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
extern uint16_t BinaryData[128];
char temp[STR_SIZE];

// Global variables

// Low Index
int low_index = 50;

// High Index
int high_index = 78;

char ctr[STR_SIZE];
int switch_state = 0;
double slope1 = 0;
float tol = 0.01;

int switch1_state = 0;
int switch2_state = 0;

void init() {
    // Init display
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

void forward(float speed) {
    TIMER_A0_PWM_Init(4800, 0.0, 1);
    TIMER_A0_PWM_Init(4800, speed, 2);
    TIMER_A0_PWM_Init(4800, 0.0, 3);
    TIMER_A0_PWM_Init(4800, speed, 4);
}

void stop_motors(void) {
    TIMER_A0_PWM_Init(4800, 0.0, 1);
    TIMER_A0_PWM_Init(4800, 0.0, 2);
    TIMER_A0_PWM_Init(4800, 0.0, 3);
    TIMER_A0_PWM_Init(4800, 0.0, 4);
}


void turn_left(){
	      TIMER_A2_PWM_Init((48000000/50/64), 0.1, 1);
        TIMER_A2_PWM_DutyCycle(0.05, 1); // Centered

}
void turn_right(){
        TIMER_A2_PWM_Init((48000000/50/64), 0.1, 1);
        TIMER_A2_PWM_DutyCycle(0.1, 1); // Centered
}
void straight(){
	
	      TIMER_A2_PWM_Init((48000000/50/64), 0.1, 1);
        TIMER_A2_PWM_DutyCycle(0.075, 1); // Centered
}

// Center of the graph should always be a slope of 0 if it's on the right course
// so if the slope becomes + or -, have to turn the car
double generate_slope(int y2, int y1, int x2, int x1) {
    double result = (y2 - y1) / (x2 - x1);
    return result;
}

// Figure out if it's a parabola - take index and value at a certain point and then the 
// exact point reflected over the axis of symmetry. If it is a slope of 0 +- a tol
// It is a parabola and it means it's on carpet
BOOLEAN carpet_detection() {
    if (line[64] < 6000){
			return TRUE;
}
}
int main(void) {
    init();
    TIMER_A2_PWM_DutyCycle(0.05, 1);
    delay(200);
    uart0_put("INIT\n");
		/*while (switch2_state == 0){
			if (switch1_state == 0 && ){
				OLED_draw_line(1,1, state1);
				OLED_write_display(OLED_TEXT_ARR);
				delay(500);
		}
			if (Switch2_Pressed == TRUE){
				switch2_state += 1;
	}*/
    while(1) {
        bin_enc();
        OLED_DisplayCameraData(BinaryData);
				uart2_put("White Val: ");
				sprintf(temp,"%i\n\r", BinaryData[64]);
				uart2_put(temp);
				uart2_put("Black Val: ");
				sprintf(temp,"%i\n\r", BinaryData[3]);
				uart2_put(temp);
				
        
        if (slope1 <= generate_slope(line[high_index], line[low_index], high_index, low_index) - tol &&
            slope1 <= generate_slope(line[high_index], line[low_index], high_index, low_index) + tol) {
            // This Needs to Turn a direction
						uart2_put("Slope 1: ");
						sprintf(temp,"%i\n\r", (int)generate_slope(line[high_index], line[low_index], high_index, low_index));
						uart2_put(temp);
						forward(0.3);
            turn_left();
							
							
        } else if (slope1 >= generate_slope(line[high_index], line[low_index], high_index, low_index) - tol &&
                   slope1 >= generate_slope(line[high_index], line[low_index], high_index, low_index) + tol) {
            // This Needs to Turn a direction
						uart2_put("Slope 2: ");
						sprintf(temp,"%i\n\r", (int)generate_slope(line[high_index], line[low_index], high_index, low_index));
						uart2_put(temp);
						forward(0.3);
						turn_right();
										 
        } else {
            // This Needs to Turn a direction
						uart2_put("Slope 3: ");
						sprintf(temp,"%i\n\r", (int)generate_slope(line[high_index], line[low_index], high_index, low_index));
						uart2_put(temp);
            forward(0.3);
            straight();
        }
        
        if (carpet_detection() == TRUE) {
            // Additional logic for carpet detection
					stop_motors();
					
        }
    }
}

/*
 * 
 * TIMER_A0_PWM_Init(4800, 0.0, 1);
 * TIMER_A0_PWM_Init(4800, 0.3, 2);
 * TIMER_A0_PWM_Init(4800, 0.0, 3);
 * TIMER_A0_PWM_Init(4800, 0.3, 4);
 * 
 * delay(1000);
 */
