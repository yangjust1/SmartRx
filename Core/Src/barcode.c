/*
 * barcode.c
 *
 *  Created on: Apr 10, 2023
 *      Author: ewbecker
 */

/* EXTERN VARIABLES & FUNCTION PROTOTYPES*/
extern int barcode[];
extern void dispense(uint8_t, uint8_t);
/********************/

/* PRIVATE VARIABLES */
static int compartments[4] = {1,2,3,4};
int free = 0;
int quantities[4] = {0};
int option = 0;
int id = 0;

void decode(void) {
	id += barcode[0];
	id += 10 * barcode[1];
	id += 100 * barcode[2];
	free = barcode[3];

	int temp_q = 0;

	for (int i = 4; i < 11; i+=2) {
		temp_q += barcode[i];
		temp_q += 10 * barcode[i+1];
		quantities[(i-4)/2] = temp_q;
		temp_q = 0;
	}

	for (int i = 0; i < 4; i++) {
		if (quantities[i] > 0) {
			dispense(compartments[i], quantities[i]);
		}
	}
}

