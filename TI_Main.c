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
extern uint16_t line[128];
extern char str[STR_SIZE];

// Global variables

// Low Index
int low_index = 50;

// High Index
int high_index = 78;

char ctr[STR_SIZE];
int switch_state = 0;
double slope1;
float tol = 0.01;

void init() {
    // Init display
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

// Center of the graph should always be a slope of 0 if it's on the right course
// so if the slope becomes + or -, have to turn the car
double generate_slope(int y2, int y1, int x2, int x1) {
    double result = (y2 - y1) / (x2 - x1);
    return result;
}

// Figure out if it's a parabola - take index and value at a certain point and then the 
// exact point reflected over the axis of symmetry. If it is a slope of 0 +- a tol
// It is a parabola and it means it's on carpet
int carpet_detection(int y2, int y1, int x2, int x1) {
    double result = (y2 - y1) / (x2 - x1);
    return result;
}

int main(void) {
    init();
    TIMER_A2_PWM_DutyCycle(0.05, 1);
    delay(200);
    uart0_put("INIT\n");
    while(1) {
        camsequence();
        OLED_DisplayCameraData(line);
        
        if (slope1 <= generate_slope(line[high_index], line[low_index], high_index, low_index) - tol &&
            slope1 <= generate_slope(line[high_index], line[low_index], high_index, low_index) + tol) {
            // This Needs to Turn a direction
						uart0_put("1\n");
						forward(0.3);
            TIMER_A2_PWM_Init((48000000/50/64), 0.1, 1);
            TIMER_A2_PWM_DutyCycle(0.05, 1); // Centered
							
							
        } else if (slope1 >= generate_slope(line[high_index], line[low_index], high_index, low_index) - tol &&
                   slope1 >= generate_slope(line[high_index], line[low_index], high_index, low_index) + tol) {
            // This Needs to Turn a direction
						uart0_put("2\n");
						forward(0.3);
            TIMER_A2_PWM_Init((48000000/50/64), 0.1, 1);
            TIMER_A2_PWM_DutyCycle(0.1, 1); // Centered
										 
										 
        } else {
            // This Needs to Turn a direction
						uart0_put("3\n");
            forward(0.3);
            
            TIMER_A2_PWM_Init((48000000/50/64), 0.1, 1);
            TIMER_A2_PWM_DutyCycle(0.075, 1); // Centered
        }
        
        if (slope1 <= carpet_detection(line[high_index], line[low_index], high_index, low_index) - tol &&
            slope1 <= carpet_detection(line[high_index], line[low_index], high_index, low_index) + tol) {
            // Additional logic for carpet detection
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
