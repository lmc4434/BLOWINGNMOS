#include "msp.h"
#include "led.h"  // you need to create this file with the function prototypes
#include "common.h"  // from Lab1 code

void LED1_Init(void)
{
	// configure PortPin for LED1 as port I/O 
	P1->SEL1 &= ~BIT0;
	P1->SEL0 &= ~BIT0;
	P1->DIR |= BIT0;
	P1->REN |= BIT0;

	// make built-in LED1 LED high drive strength
	P1->DS |= BIT0;

	// make built-in LED1 out
	P1->OUT |= BIT0;
	
	// turn off LED
	P1->OUT &= ~BIT0;
}

void LED2_Init(void)
{
	// configure PortPin for LED2 as port I/O 
	P2->SEL1 &= ~BIT0;
	P2->SEL0 &= ~BIT0;
	P2->DIR |= BIT0;
	P2->REN |= BIT0;
	
	P2->SEL1 &= ~BIT1;
	P2->SEL0 &= ~BIT1;
	P2->DIR |= BIT1;
	P2->REN |= BIT1;
	
	P2->SEL1 &= ~BIT2;
	P2->SEL0 &= ~BIT2;
	P2->DIR |= BIT2;
	P2->REN |= BIT2;

	// make built-in LED2 LEDs high drive strength
	P2->DS |= BIT0;
	P2->DS |= BIT1;
	P2->DS |= BIT2;

	// make built-in LED2 out	 
	P2->OUT |= BIT0;
	P2->OUT |= BIT1;
	P2->OUT |= BIT2;

	// turn off LED
	P2->OUT &= ~BIT0;
	P2->OUT &= ~BIT1;
	P2->OUT &= ~BIT2;
}

void LED1_Off(void){
	// turn off LED
	P1->OUT &= ~BIT0;
}

void LED1_On(void){
	// turn on LED
	P1->OUT |= BIT0;
}

void LED2_Off(void){
	P2->OUT &= ~BIT0;
	P2->OUT &= ~BIT1;
	P2->OUT &= ~BIT2;
}

void RED(void) {
	P2->OUT |= BIT0;
}

void GREEN(void) {
	P2->OUT |= BIT1;
}

void BLUE(void) {
	P2->OUT |= BIT2;
}

void CYAN(void) {
	P2->OUT |= BIT1;
	P2->OUT |= BIT2;
}

void MAGENTA(void) {
	P2->OUT |= BIT0;
	P2->OUT |= BIT2;
}

void YELLOW(void){
	P2->OUT |= BIT0;
	P2->OUT |= BIT1;
}

void WHITE(void){
	P2->OUT |= BIT0;
	P2->OUT |= BIT1;
	P2->OUT |= BIT2;
}

void LED2_On(int color){
	if(color == 0){
		RED();
	} 
	else if(color == 1){
		BLUE();
	}
	else if(color == 2){
		GREEN();
	}
	else if(color == 3){
		CYAN();
	}
	else if(color == 4){
		MAGENTA();
	}
	else if(color == 5){
		YELLOW();
	}
	else if(color == 6){
		WHITE();
	} else {
		LED2_Off();
	}
}
