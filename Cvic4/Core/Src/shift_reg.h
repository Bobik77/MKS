#ifndef SHIFT_REG
#define SHIFT_REG

// Init shift register
void sct_init(void);

// Roll in bit values into shift register
void sct_led(uint32_t value);

// Show value on 7 segment disp
void sct_value(uint16_t value,uint8_t led);


#endif /*SHIFT_REG*/
