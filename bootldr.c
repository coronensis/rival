
#include <stdint.h>

/* Address of memory-mapped seven segment display */
volatile uint8_t *ss  = (volatile uint8_t*) 0x20000004;

const char *message = "Hello Rival!";

/* Delay by wasting CPU cycles. No HW timers are currently available */
void delay (uint32_t value)
{
	/* volatile keeps the compiler from "optimizing" this whole
	 * function away */
	volatile uint32_t i;
	for (i = 0; i < value; i++)
		;
}

int main(void)
{
	uint8_t i;

	for (i = 0; i < 12; i++) {
		*ss = message[i];
		delay(12500000);
	}

	return 0;
}
