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

BOOLEAN debug = 0;

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
float test = 0.0;

// Global variables


//PID SETUP
float servo_position = 0.0;


float integral = 0;
float prev_error = 0;

float Kp = 0.6; //0.6
float Ki = 0.017; //0.15 //Large delay turn hard
float Kd = 0.35; //0.20 //Return to center and turn hard
BOOLEAN center_flag = 0;

float updateServoPosition(float control_signal) {
    
    servo_position = control_signal;
    if (servo_position <= 0.05) {
        servo_position = 0.05;
    } else if (servo_position >= 0.1) {
        servo_position = 0.1;
    }
    // Return the updated servo position
		if(debug){
    uart2_put("Servo Position : ");
		sprintf(temp,"%i\n\r", (int)(servo_position*1000));
		uart2_put(temp);
		}
    return servo_position;
}

BOOLEAN carpet_detection() {
    if (line[64] < 6000){
			return TRUE;
}
}

//END PID SETUP

// Low Index
int low_index = 50;

// High Index
int high_index = 78;

char ctr[STR_SIZE];
int switch_state = 0;
double slope1 = 0;
float tol = 0.01;

float servo_current = 0.0;
float servo_previous = 0.075;
float s_err0 = 0.0;
float s_err1 = 0.0;
float s_err2 = 0.0;
float s_err3 = 0.0;
float s_err4 = 0.0;
float s_err5 = 0.0;

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

int find_center(void){
    BOOLEAN flag = FALSE;
    int left_val;
    int right_val;
    for (int i = 0; i < 127; i++){
        if (flag == FALSE){
            if (BinaryData[i] == 1){
                left_val = i;
                flag = TRUE;
                }
    }else if (flag == TRUE){
        if (BinaryData[i] == 0){
            right_val = i;
            flag = FALSE;
        }
    }
	}
  return (right_val + left_val)/2;
}

float PID(void){
	/*
	if (center > 63){
		s_err0 = -1*(0.075 - ((float)center / 128) * (0.1 - 0.05) + 0.025);
	} else {
		s_err0 = 0.075 - ((float)center / 128) * (0.1 - 0.05) + 0.025;
	}
	*/
	
		s_err0 = 0.075 - (((find_center()) / 128.00) * (0.1) + 0.025);
		if(debug){
		uart2_put("SERR0: ");
		sprintf(temp,"%f\n\r", s_err0*1000);
		uart2_put(temp);
		}
		//if(s_err0 <= -0.064 || 0.190 >= s_err0){
			servo_current = servo_previous + Kp*s_err0 + 
				Ki*((s_err0+s_err1+s_err2+s_err3+s_err4+s_err5)/6) +
				Kd*(s_err0 - s_err1);
		//if (carpet_detection() != TRUE){
				if(find_center() > 60 && find_center() < 68){
		
					servo_current = 0.075;
					s_err0 = 0.0;
					s_err1 = 0.0;
					s_err2 = 0.0;
					s_err3 = 0.0;
					s_err4 = 0.0;
					s_err5 = 0.0;
			//}
			center_flag = 1;
					
			servo_current = updateServoPosition(servo_current);
			servo_previous = servo_current;
			s_err5 = s_err4;
			s_err4 = s_err3;
			s_err3 = s_err2;
			s_err2 = s_err1;
			s_err1 = s_err0;
		} else {
			center_flag = 0;
		}
		//} 
		
				uart2_put("Servo Val: ");
				sprintf(temp,"%f\n\r", servo_current);
				uart2_put(temp);
		
		return servo_current;

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


// Center of the graph should always be a slope of 0 if it's on the right course
// so if the slope becomes + or -, have to turn the car
double generate_slope(int y2, int y1, int x2, int x1) {
    double result = (y2 - y1) / (x2 - x1);
    return result;
}

// Figure out if it's a parabola - take index and value at a certain point and then the 
// exact point reflected over the axis of symmetry. If it is a slope of 0 +- a tol
// It is a parabola and it means it's on carpet

int main(void) {

    init();
    delay(200);
		/*while (switch2_state == 0){
			if (switch1_state == 0 && ){
				OLED_draw_line(1,1, state1);
				OLED_write_display(OLED_TEXT_ARR);
				delay(500);
		}
			if (Switch2_Pressed == TRUE){
				switch2_state += 1;
	}*/
	
    forward(0.25,0.25);
    while(1) {
        bin_enc();
        //OLED_DisplayCameraData(SmoothData);
				// Check This
				//int square_wave_center = find_center();
				//float error = setpoint - ((float)square_wave_center / 128) * (0.1 - 0.05) + 0.05;
				
				//End
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
        //if (carpet_detection() != TRUE) {
            // This Needs to Turn a direction
        //}
				//else{
					if(debug){
					uart2_put("I EAT CARPET : ");
					}
					
				//}
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
