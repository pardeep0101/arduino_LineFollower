/*
 * main.cpp
 *
 *  Created on: Nov 3, 2015
 *      Author: Pardeep Kumar
 *      Note: Everything should be made as simple as possible but not simpler
 */

#include <avr/io.h>
#include <avr/delay.h>
#include <avr/interrupt.h>

#if defined(__AVR_ATmega328P__)
// A mega328 is mostly like an mega168, in terms of peripherals
// This is defined here just to make sure we get access to all the ports and timer register
#define __AVR_ATmega168__ 1
#endif

// default direction is 0 (go straight), 1 for right and -1 for left
int direction = 0;

// motion control variable
bool runfor = false;
// sets the Output Compare register on port B - pin OCR1A -- and sets its value to 2000 less than ICR1
void goLeft() {
	OCR1A = ICR1 - 2000;

}
// sets the Output Compare register on port B - pin OCR1B -- and sets its value to 2000 less than ICR1
void goRight() {
	OCR1B = ICR1 - 2000;

}
// sets the Output Compare register on port A & B - pin OCR1A & OCR1A -- and sets its value to 2000 less than ICR1
void gostraight() {
	OCR1A = ICR1 - 2000;
	OCR1B = ICR1 - 2000;

}

// sets both register to zero
void car_motor_stop() {
	OCR1A = 0;
	OCR1B = 0;
}
// mini scheduler for moving the wheels according to the given direction
void car_motor(int direction) {
	//OCR1A = ICR1 - 100; //18000
	// counter for taking decision when it reaches 19000
	int cntl = 0;
	runfor = true;	// to make sure loop runs only once
	while (runfor) {
		cntl++;
		// starting the wave at 19000
		if (cntl == 19000) {
			//go straight
			if (direction == 0) {
				OCR1A = 0;
				OCR1B = 0;

				gostraight();

			}
			//go left
			else if (direction < 0) {
				// red LED connected along with the motor circuit to show its ON; Otherwise OFF
				OCR1B = 0;
				goLeft();

			}
			// go right
			else if (direction > 0) {
				// yellow LED connected along with the motor circuit to show its ON; Otherwise OFF
				OCR1A = 0;
				goRight();

			}
			cntl = 0;
			runfor = false;
		}
	}
}

// main logic for program
int main(void) {
	// store the last turn value
	int lastTurn = 0;

	// initial condition
	bool tookTurn = false;

	DDRB |= 0xFF; // setting all pins to listen for data on Port B
	DDRD &= ~(1 << PD2); // using port pin 2 on PORT D as input
	DDRD &= ~(1 << PD3); // using port pin 3 on PORT D as input
	DDRD &= ~(1 << PD4); //  using port pin 4 on PORT D as input

	/**
	 * Timer Logic for PWM wave generation
	 * TCCR1A -> WGM11 WGM12 & WGM 13 are set for Fast PWM wave generation,
	 * 		   along with COM1A1 and COM1B1 for inverted mode.
	 * TCCR1B -> CS10 is used for avoiding any system pre-scaler
	 * ICR1 -> setting up
	 */
	TCCR1A |= 1 << WGM11 | 1 << COM1A1 | 1 << COM1B1; //FOR NON INVERTED MODE
	TCCR1B |= 1 << WGM13 | 1 << WGM12 | 1 << CS10;
	ICR1 = F_CPU / 50; // FREQUENCY -- BLINKS FASTER IF USE HIGHER DIVISOR


	// infinite loop
	while (1) {

		/**
		 * Starting of the system wide scheduler and decision making based on the information
		 * being received by the sensors. Dynamic interaction of motor control logic with sensor input stream.
		 *
		 */


		/**
		 * Logic Explained: Sensor puts '1' on its OUTPUT stream whenever it is placed on white surface
		 * and put '0' on its output stream whenever it is placed on black-line.
		 * Therefore, we can create a logic using the output streams of all the sensors and then
		 * controlling the motor scheduler
		 */
		// OUTPUT pins for sensors on PORTD are 2 (Center) ,3 (Right) & 4 (Left)
		// if sensors are reading white board turn up the LED on pin 11 (Center), 12 (Right) & 8 (Left)
		if ((PIND & (1 << PD2)) && (PIND & (1 << PD3)) && (PIND & (1 << PD4))) {

			//turn on the LED on port B
			PORTB |= (1 << PB3);
			PORTB |= (1 << PB4);
			PORTB |= (1 << PB0);

			/**
			 * Little fault tolerance: Determine if we have taken any turn yet, if yes
			 * then repeat its opposite whenever all sensors of robot goes off the line
			 */

			//Check if we have made any turn before all the sensors went off the Line i.e. all
			// are on white board. If yes then make the opposite of the turn we took last time.
			if (tookTurn) {
				car_motor(lastTurn);
				_delay_ms(2000);
			}
			// if not just keep it stationary
			else {
				car_motor_stop();
			}

		}

		// if middle sensor (Center) is on the line
		if (!(PIND & (1 << PD2)) && (PIND & (1 << PD3)) && (PIND & (1 << PD4))) {
			// turn on the LED for middle sensor
			PORTB &= ~(1 << PB3);

			// first stop the robot
			car_motor_stop();
			_delay_ms(8000);
			// move it straight by passing '0' to the motor scheduler
			car_motor(0);
			_delay_ms(2000);

			// set the 'tookTurn' to true, so we can inform our Fault Tolerance module to set us back on the line
			tookTurn = true;

			// set the last turn
			lastTurn = 0;

		}

		// if right sensor is on black
		if ((PIND & (1 << PD2)) && !(PIND & (1 << PD3)) && (PIND & (1 << PD4))) {
			// turn on the LED for middle sensor
			PORTB &= ~(1 << PB4);
			car_motor_stop();
			_delay_ms(8000);

			// move it straight by passing '0' to the motor scheduler
			car_motor(1);
			_delay_ms(1000);

			// set the 'tookTurn' to true, so we can inform our Fault Tolerance module to set us back on the line
			tookTurn = true;

			// set the last turn to opposite of the turn we took, so the fault tolerance can reset us by going opp.
			lastTurn = -1;

		}

		/**
		 * Same comments for the loops below
		 */

		// if left sensor is on line
		if ((PIND & (1 << PD2)) && (PIND & (1 << PD3)) && !(PIND & (1 << PD4))) {
			PORTB &= ~(1 << PB0);
			car_motor_stop();
			_delay_ms(8000);
			car_motor(-1);
			_delay_ms(1000);
			tookTurn = true;
			lastTurn = 1;

		}
		// if middle and right sensor is on line
		if (!(PIND & (1 << PD2)) && !(PIND & (1 << PD3)) && (PIND & (1 << PD4))) {
			PORTB &= ~(1 << PB3);
			car_motor_stop();
			_delay_ms(8000);
			car_motor(0);
			_delay_ms(2000);
			tookTurn = true;
			lastTurn = 0;

		}
		// if middle and left sensor is on line
		if (!(PIND & (1 << PD2)) && (PIND & (1 << PD3)) && !(PIND & (1 << PD4))) {
			PORTB &= ~(1 << PB3);
			car_motor_stop();
			_delay_ms(8000);
			car_motor(0);
			_delay_ms(2000);
			tookTurn = true;
			lastTurn = 0;
		}
		// if middle, right and left sensor is on line
		if (!(PIND & (1 << PD2)) && !(PIND & (1 << PD3)) && !(PIND & (1 << PD4))) {
			PORTB &= ~(1 << PB3);
			car_motor_stop();
			_delay_ms(8000);
			car_motor(1);
			_delay_ms(2000);
			tookTurn = true;
			lastTurn = -1;
		}

		// stop the motors at last of each loop
		car_motor_stop();

	}
}


