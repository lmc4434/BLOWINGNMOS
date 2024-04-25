/*
 * PID.c
 * 
 * Author: Logan Culver and Garrett Brindle 
 */

#include "PID.h"
#include "uart.h"


extern float Kp; //0.6
extern float Ki;
extern float Kd; //0.20 //Return to center and turn hard

//Servo to angle conversion
float servo_min = 0.05;
float servo_max = 0.1;
float angle_min = -60.0;
float angle_max = 60.0;
float servo_middle = 0.075;


extern BOOLEAN debug;
float servo_position = 0.0;
float servo_previous = 0.075;
float s_err0, s_err1, s_err2, s_err3, s_err4, s_err5 = 0.0;
extern uint16_t BinaryData[128];
BOOLEAN center_flag = 0;

BOOLEAN center_flag_hold = 0;

extern char temp[STR_SIZE];
float servo_current = 0.0;

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
			center_flag_hold = 1;
					
			servo_current = updateServoPosition(servo_current);
			servo_previous = servo_current;
			s_err5 = s_err4;
			s_err4 = s_err3;
			s_err3 = s_err2;
			s_err2 = s_err1;
			s_err1 = s_err0;
		} else {
			if (find_center() > 53 && find_center() < 75){
					center_flag = 1;
					center_flag_hold = 1;
				} else {
					center_flag = 0;
				}
		}
		//} 
		/*
				uart2_put("Servo Val in PID: ");
				sprintf(temp,"%f\n\r", servo_current);
				uart2_put(temp);
		*/
		
		return servo_current;

}
 
float find_angle(float servo_val){
	float slope = (angle_max - angle_min) / (servo_max - servo_min);
	float angle = (servo_val - servo_min) * slope + angle_min;
	return angle;
	
	
}
