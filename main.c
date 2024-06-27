#include "lcd.h"
#include "utils.h"
static int toggle = 0;

void initialize_tpm(TPM_Type *tpm, uint8_t prescaler) {
    tpm->SC = 0; // Disable TPM to configure
    tpm->MOD = PWM_FREQUENCY; // Set the period for the PWM
    tpm->SC = TPM_SC_PS(prescaler) | TPM_SC_CMOD(1); // Set prescaler and enable TPM
}

void initialize_interrupt(){

	PORTA->PCR[1] = PORT_PCR_MUX(1);
	PTA->PDDR &= ~(1u << 1); //PTA1 input
	PORTA->PCR[1] |= PORT_PCR_MUX(1) | PORT_PCR_PE_MASK | PORT_PCR_PS(1); //PTA1 has pull up resistor
	NVIC_EnableIRQ(PORTA_IRQn); //enable interrupt

}

void PORTA_IRQHandler(void){
	PORTA->ISFR = 0XFFFFFFFF;
	toggle ^= 1;
}

void initialize_pwm_channel(TPM_Type *tpm, uint8_t channel, uint32_t cnv) {
    tpm->CONTROLS[channel].CnSC = TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK; // Edge-aligned, high-true pulses
    tpm->CONTROLS[channel].CnV = cnv;
}

void configure_pin_for_tpm(PORT_Type *port, uint32_t pin, uint8_t mux_value) {
    port->PCR[pin] = PORT_PCR_MUX(mux_value);
}

void configure_pin_for_adc(PORT_Type *port, uint32_t pin, uint8_t mux_value) {
    port->PCR[pin] = PORT_PCR_MUX(mux_value);
}

void initialize_peripherals(){
    // Enable clocking
    SIM->SCGC5 |= SIM_SCGC5_PORTA_MASK | SIM_SCGC5_PORTE_MASK | SIM_SCGC5_PORTC_MASK; // Enable clock for PORTA and PORTE
    SIM->SCGC6 |= SIM_SCGC6_TPM1_MASK | SIM_SCGC6_TPM0_MASK;  // Enable clock for TPM1 and TPM0

    // Set clock source for TPM
    SIM->SOPT2 |= SIM_SOPT2_TPMSRC(1); // Use MCGFLLCLK 41.94MHz

    // Initialize TPM1
    initialize_tpm(TPM1, 5); // Prescaler 32

    // Configure PTA12 and PTA13 for TPM1 CH0 and CH1
    configure_pin_for_tpm(PORTA, 12, 3); // Set PTA12 to TPM1_CH0
    configure_pin_for_tpm(PORTA, 13, 3); // Set PTA13 to TPM1_CH1
    initialize_pwm_channel(TPM1, 0, 480); // Initial CnV value for TPM1 CH0
    initialize_pwm_channel(TPM1, 1, 384); // Initial CnV value for TPM1 CH1
	
    // Initialize TPM0
    initialize_tpm(TPM0, 6); // Prescaler 64

    // Configure PTA4 alternative PTC2 ALT4 and PTA5 alternative PTC3 ALT4 for TPM0 CH1 and CH2
    configure_pin_for_tpm(PORTC, 2, 4); // Set PTC2 to TPM0_CH1
    configure_pin_for_tpm(PORTC, 3, 4); // Set PTC3 to TPM0_CH2
    initialize_pwm_channel(TPM0, 1, 480); // Fixed CnV value for TPM0 CH1
    initialize_pwm_channel(TPM0, 2, 480); // Fixed CnV value for TPM0 CH2
		
		// Configure PTE20 for ADC SE0 - ALT0
		configure_pin_for_adc(PORTE, 20, 0);
		configure_pin_for_adc(PORTE, 21, 4)
		configure_pin_for_adc(PORTE, 22, 3);
		initialize_adc();
}

void initialize_adc(){
    SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK; // Enable ADC0 clock
    ADC0->CFG1 = ADC_CFG1_ADIV(3) | ADC_CFG1_MODE(1) | ADC_CFG1_ADLSMP_MASK; // 12-bit resolution, long sample time
    ADC0->SC2 = ADC_SC2_ADTRG(0); // Software trigger
}

void set_cnv(TPM_Type *tpm, int channel, uint32_t cnv){
    tpm->CONTROLS[channel].CnV = cnv;
}

void delay_500ms() {
    volatile unsigned int i;
    for (i = 0; i < 650000; i++) {
        __NOP(); // Perform no operation, safer than empty line
    }
}

void delay_5ms() {
    volatile unsigned int i;
    for (i = 0; i < 6500; i++) {
        __NOP(); // Perform no operation, safer than empty line
    }
}

uint16_t analog_read(uint8_t channel){
    ADC0->SC1[0] = channel & ADC_SC1_ADCH_MASK; // Start conversion on channel
    while(!(ADC0->SC1[0] & ADC_SC1_COCO_MASK)); // Wait for conversion to complete
    return (uint16_t) (ADC0->R[0]); // Return the ADC value
}

uint16_t smaRead(uint8_t channel) {
    static int readings[WINDOW_SIZE] = {0}; // Buffer to store the last n readings
    static int sum = 0;                     // Sum of the last n readings
    static int index = 0;                   // Current index in the buffer
    static int count = 0;                   // Number of values added to the buffer

    // Read new value from ADC
    uint16_t new_reading = analog_read(channel);

    // Subtract the oldest reading, add the new one
    sum -= readings[index];
    sum += new_reading;
    readings[index] = new_reading;

    // Move to the next buffer index, wrap around if necessary
    index = (index + 1) % WINDOW_SIZE;

    // Increment count until the buffer is filled
    if (count < WINDOW_SIZE) {
        count++;
    }

    // Calculate the average
    return (uint16_t)sum / count;
}

// Assumes a system clock of 48 MHz
void delay_ms(uint32_t ms) {
    volatile uint32_t i, j;
    for (i = 0; i < ms; i++) {
        for (j = 0; j < 6000; j++) {
            __NOP(); // Do nothing operation
        }
    }
}


uint16_t map_adc_value_to_cnv(uint16_t adc_val) {
    // Map from ADC range 0-4095 to PWM CnV range 330-2440
    return (adc_val * (2440 - 330) / 4095) + 330;
}

float interpolate_cnv_to_ms(uint16_t cnv){

		return (cnv * (3 - 0.5f) / 2440) + 0.5f;

}

int main (void) {
    initialize_peripherals();
		LCD_init();
		clear_lcd();
		delay_ms(200);
	
		uint16_t adc_result;
    uint32_t pwm_cnv1;
	  uint32_t pwm_cnv2;

		char str[10];
			
	
    while (1) {

			adc_result = analog_read(0);
			pwm_cnv1 = map_adc_value_to_cnv(adc_result);
			set_cnv(TPM1, 1, pwm_cnv1); //PTA13 pwm
			
		
			sprintf(str, "%d", pwm_cnv1);
			print_fnc((unsigned char *)str);
			
			delay_ms(10);
			
			
			print_fnc("      ");
			
			
			adc_result = analog_read(3);
			pwm_cnv2 = map_adc_value_to_cnv(adc_result);
			set_cnv(TPM1, 0, pwm_cnv2); //PTA12 pwm
			
			
			sprintf(str, "%d", pwm_cnv2);
			print_fnc((unsigned char *)str);
			
			delay_ms(50);
			clear_lcd();
			

		  
		}

	}