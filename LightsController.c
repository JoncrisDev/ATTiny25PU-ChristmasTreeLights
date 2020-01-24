/*
* Multi-speed flashing Christmas Lights Controller.
* Hardware: ATTiny25-PU MCU
* Author: Moi, Date: 29/12/19
*/

#define __DELAY_BACKWARD_COMPATIBLE__

#include <util/delay.h>
#include <avr/io.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>

#include "LightsController.h"


// Setup the ports for the chipz
inline void Setup_Ports() {

	// Pin 1 and pin 3 as outputs.
	DDRB |= ((1 << DDB3) | (1 << DDB1) | (1 << DDB0));

	
	// pin 0 as an input with an internal pull-up
	//DDRB &= ~(1 << DDB0);
	//PORTB |= (1 << PB0);

	// pin 4 as an input with an internal pull-up
	DDRB &= ~(1 << DDB4);
	PORTB |= (1 << PB4);

	// Enable interrupts on PCINT4
	//GIMSK |= (1 << PCIE);
	//PCMSK |= (1 << PCINT4);


	// [DISABLE THE INTERRUPTS FOR A MOMENT]
	// Enable interrupt masks. on PB0
	//DDRB |= (1 << DDB0);
	//PORTB |= (1 << PB0);
	//GIMSK |= (1 << PCIE);
	//PCMSK |= (1 << PCINT0);
	//sei();
}

// Reset the current Mode and activate the next mode along in the modes Array
void SwapOutMode(struct LEDMode * mode) {
	if (mode->line_1.bEnabled == TRUE) {
		ResetLine(&mode->line_1);
	}
	if (mode->line_2.bEnabled == TRUE) {
		ResetLine(&mode->line_2);
	}
	// Increment Global Mode Index.
	CurrentModeID++;
	if (CurrentModeID > (NUM_MODES -1)) {
		CurrentModeID = 0;
	}
	
}

// Reset a Line to Initial State (Turn off active Pins)
void ResetLine(struct LEDLine * Line) {
	Line->ElapsedTimeInState = 0;
	Line->bIsHigh = FALSE;
	if (Line->bIsFade == TRUE) {
		Line->bFadingUp = TRUE;
		Line->bFadingDown = FALSE;

		// Totally revert the PWM registers that we changed.
		TIMSK &= ~((1 << OCIE0A) | (1 << OCIE1A));
		TCCR0A &= ~((1 << COM0A1) | (1 << WGM01) | (1 << WGM00));
		TCCR0B &= ~(1 << CS00);
	}
	LED_Off(Line->Pin);
}

// Run The Current Mode for one cycle
void RunMode(struct LEDMode * mode) {
	if(mode->line_1.bEnabled == TRUE) { RunLine(&mode->line_1); }
	if(mode->line_2.bEnabled == TRUE) { RunLine(&mode->line_2); }
}

// Run A Line Within the mode 
void RunLine(struct LEDLine * Line) {

	// Toggle	
	if ((Line->bFadingUp == FALSE) && (Line->bFadingDown == FALSE) ) {
		
		// flip the LED state.
		if ((Line->ElapsedTimeInState > Line->TimeIsOn) && (Line->bIsHigh == TRUE)) {
			if (Line->bIsFade == TRUE) {
				Line->bFadingDown = TRUE;
			} else {
				LED_Off(Line->Pin);
			}
			
			Line->bIsHigh = FALSE;
			Line->ElapsedTimeInState = 0;
		}

		if ((Line->ElapsedTimeInState > Line->TimeIsOff) && (Line->bIsHigh == FALSE)) {
			if (Line->bIsFade == TRUE) {
				Line->bFadingUp = TRUE;
			} else {
				LED_On(Line->Pin);
			}
			Line->bIsHigh = TRUE;
			Line->ElapsedTimeInState = 0;
		}
	}

	// Pulse-Width Modulation Fade
	if ( (Line->bIsFade == TRUE) && ((Line->bFadingUp == TRUE) || (Line->bFadingDown == TRUE)) ) {
		
		//PORTB &= ~(1 << PB0);

		// non-inverting 8 bit pwm
		TCCR0A |= (1 << COM0A1) | (1 << WGM01) | (1 << WGM00);
		//  Clock select (clk I/O / no scale)
		TCCR0B |= (1 << CS00);

		// Compare Match A
		TIMSK = (1 << OCIE0A) | (1 << OCIE1A);
		DDRB |= (1 << PB0) | (1 << PB5);

		if (Line->bFadingUp == TRUE) {

			float CurrentPhase = (float)Line->ElapsedTimeInState / Line->TimeFadeUp; 
			OCR0A = (uint8_t) (0xFF * CurrentPhase); 

			if (Line->ElapsedTimeInState >= Line->TimeFadeUp) {
				Line->bFadingUp = FALSE;
				Line->ElapsedTimeInState = 0;
				Line->bIsHigh = TRUE;
			}
		}

		// PWM Fade down
		if (Line->bFadingDown == TRUE) {

			float CurrentPhase = (float)Line->ElapsedTimeInState / Line->TimeFadeDown; 
			OCR0A = (uint8_t)(0xFF - (0xFF * CurrentPhase)); 

			if (Line->ElapsedTimeInState >= Line->TimeFadeDown) {
				Line->bFadingDown = FALSE;
				Line->ElapsedTimeInState = 0;
				Line->bIsHigh = FALSE;
			}

		}
		
	}
	
	Line->ElapsedTimeInState++;

}

int main (void)
{

	struct LEDMode modes[NUM_MODES];
	/* set board io port */
	Setup_Ports();

	// Mode 1
	modes[0].line_1.bEnabled = TRUE;
	modes[0].line_1.bIsFade = FALSE;
	modes[0].line_1.TimeIsOn = 500;
	modes[0].line_1.TimeIsOff = 500;
	//modes[0].line_1.bIsHigh = TRUE; // hmmm
	modes[0].line_1.Pin = LED0;
	modes[0].line_2.bEnabled = FALSE;

	// Mode 2
	modes[1].line_1.bEnabled = TRUE;
	modes[1].line_1.bIsFade = FALSE;
	modes[1].line_1.TimeIsOn = 500;
	modes[1].line_1.TimeIsOff = 500;
	modes[1].line_1.Pin = LED1;
	modes[1].line_2.bEnabled = TRUE;
	modes[1].line_2.bIsFade = FALSE;
	modes[1].line_2.TimeIsOn = 2000;
	modes[1].line_2.TimeIsOff = 600;
	modes[1].line_2.Pin = PB0;


	// Mode 3 PWM Fade
	modes[2].line_1.bEnabled = TRUE;
	modes[2].line_1.bIsFade = TRUE;
	modes[2].line_1.TimeIsOn = 4000;
	modes[2].line_1.TimeIsOff = 4000;
	modes[2].line_1.TimeFadeUp = 5000;
	modes[2].line_1.TimeFadeDown = 200;
	modes[2].line_1.bFadingUp = TRUE;

	
	// 8 Seconds
	const int ProgramCycleTime = 8000;
	int ButtonHoldTime = 0;

	while (1) {

		
		// DEBUG - Cycle the programs.
		//for (int cnt = 0; cnt < ProgramCycleTime; ++cnt) {
			RunMode(&modes[CurrentModeID]);
			_delay_ms(1);

			// Button control
			if(GetPinLevel(BUTTON0) == BUTTON_DOWN) {

				if (ButtonHoldTime > HOLD_TIME_POWER_DWON) {
					ButtonHoldTime = 0;
					MCU_PowerDown();
				}
				ButtonHoldTime++;
				ButtonPrevState = BUTTON_DOWN;

				
			} else if ((ButtonPrevState == BUTTON_DOWN) && (ButtonHoldTime > HOLD_TIME_PROGRAM_CHANGE)) {
				SwapOutMode(&modes[CurrentModeID]);
				ButtonPrevState = BUTTON_UP;
				ButtonHoldTime = 0;
			} else if (GetPinLevel(BUTTON0) == BUTTON_UP) {
				ButtonPrevState = BUTTON_UP;
			}

			 
		//}
		//SwapOutMode(&modes[CurrentModeID % NUM_MODES]);
	}

}

// Interrupt Vector
ISR(PCINT0_vect) {

	cli();
	sleep_disable();
	
	//CurrentModeID++;
	
	CurrentModeID = 0;
	PCMSK &= ~(0xFF);

	//cli();
	
	//PCMSK = 0x00;
}


// Power Down the controller on a long button press.
void MCU_PowerDown(void) {
	
	// Stop the thing turning back on again when we are trying to turn it off.
	LED_On(LED1);
	LED_On(LED0);
	_delay_ms(4000);
	LED_Off(LED1);
	LED_Off(LED0);
	
	// Enable interrupt masks.
	GIMSK |= (1 << PCIE);
	PCMSK |= (1 << PCINT4);
	
	// (Turn off) Enter power down state
	set_sleep_mode(SLEEP_MODE_PWR_DOWN);
	sleep_enable();
	
	// enable interrupts
	sei();
	sleep_cpu();
	
	// Resumption Instruction from wake-from-sleep (post Interrupt call.)
	//cli();
};





