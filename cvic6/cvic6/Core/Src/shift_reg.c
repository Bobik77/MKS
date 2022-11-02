#include <stdint.h>
#include <stm32f0xx.h>
#include "shift_reg.h"
static const uint32_t reg_values[3][10] = {
	{
	//PCDE--------GFAB @ DIS1
	0b0111000000000111 << 16,
	0b0100000000000001 << 16,
	0b0011000000001011 << 16,
	0b0110000000001011 << 16,
	0b0100000000001101 << 16,
	0b0110000000001110 << 16,
	0b0111000000001110 << 16,
	0b0100000000000011 << 16,
	0b0111000000001111 << 16,
	0b0110000000001111 << 16,
	},
	{
	//----PCDEGFAB---- @ DIS2
	0b0000011101110000 << 0,
	0b0000010000010000 << 0,
	0b0000001110110000 << 0,
	0b0000011010110000 << 0,
	0b0000010011010000 << 0,
	0b0000011011100000 << 0,
	0b0000011111100000 << 0,
	0b0000010000110000 << 0,
	0b0000011111110000 << 0,
	0b0000011011110000 << 0,
	},
	{
	//PCDE--------GFAB @ DIS3
	0b0111000000000111 << 0,
	0b0100000000000001 << 0,
	0b0011000000001011 << 0,
	0b0110000000001011 << 0,
	0b0100000000001101 << 0,
	0b0110000000001110 << 0,
	0b0111000000001110 << 0,
	0b0100000000000011 << 0,
	0b0111000000001111 << 0,
	0b0110000000001111 << 0,
	},
};
static const uint32_t dec_points_values[3] = { 1<<15, 1<<11, 1<<31};


//pin and port definitions
#define SR_PORT  GPIOB
#define SDI_PIN  (1<<4)
#define CLK_PIN  (1<<3)
#define NLA_PIN  (1<<5)
#define NOE_PIN  (1<<10)


// set/clear macros
#define sct_nla(x) do { if (x) SR_PORT->BSRR = NLA_PIN; else SR_PORT->BRR = NLA_PIN; } while (0)
#define sct_sdi(x) do { if (x) SR_PORT->BSRR = SDI_PIN; else SR_PORT->BRR = SDI_PIN; } while (0)
#define sct_clk(x) do { if (x) SR_PORT->BSRR = CLK_PIN; else SR_PORT->BRR = CLK_PIN; } while (0)
#define sct_noe(x) do { if (x) SR_PORT->BSRR = NOE_PIN; else SR_PORT->BRR = NOE_PIN; } while (0)

void sct_init(void) {
	// enable ports clocks
	RCC->AHBENR |=  RCC_AHBENR_GPIOBEN;
	// set as output
	SR_PORT->MODER |= GPIO_MODER_MODER4_0;
	SR_PORT->MODER |= GPIO_MODER_MODER3_0;
	SR_PORT->MODER |= GPIO_MODER_MODER5_0;
	SR_PORT->MODER |= GPIO_MODER_MODER10_0;
	//init
	sct_noe(0); // activate SR output
	sct_led(0);

}

void sct_led(uint32_t value) {
	for(uint8_t i = 0; i<32; i++){
		sct_sdi(value&1ul);
		sct_clk(1);
		sct_clk(0);
		value>>=1;
	}
	sct_nla(1);
	sct_nla(0);

}

void sct_value(uint16_t value) {
	uint32_t reg = 0;
	reg |= reg_values[0][(value / 100) % 10]; //hundreds
	reg |= reg_values[1][(value / 10) % 10]; //tens
	reg |= reg_values[2][value % 10]; //ones

	sct_led(reg);
}

void sct_value_point(uint16_t value, uint8_t point) {
	uint32_t reg = 0;

	reg |= reg_values[0][(value / 100) % 10]; //hundreds
	reg |= reg_values[1][(value / 10) % 10]; //tens
	reg |= reg_values[2][value % 10]; //ones

	if (point != 0) {
		for (uint8_t i = 0; i<3; i++) {
			if (( (point>>i) & 1) == 1) {
				reg |= dec_points_values[i];
			}
		}
	}

	sct_led(reg);
}
