#ifndef UTILS_H
#define UTILS_H

#include <stdint.h>
#include <stdio.h>
#include <MKL25Z4.h> // Make sure this path is correct for your development environment

// Average count for adc smoothing
#define WINDOW_SIZE 10

// PWM frequency configuration
#define PWM_FREQUENCY 13106 // PWM frequency for timer overflows

// Neutral CnV positions for axis (NEEDS TRIAL)
#define NEUTRAL_UPPER_ARM 984 // Appropriate for angles 90 to 160 degrees
#define NEUTRAL_LOWER_ARM 984 // Free range from 0 to 180 degrees
#define NEUTRAL_BASE 984      // Free range from 0 to 180 degrees

// Claw CnV positions for open-closed states
#define CLAW_OPEN 750    // 90 degrees
#define CLAW_CLOSED 650  // 45 degrees

// ADC0 Channels used for all axis
#define ADC0_HEAD_CHANNEL 0 // PTE20
#define ADC0_UPPER_ARM_CHANNEL 3 // PTE22
#define ADC0_LOWER_ARM_CHANNEL 0 // PTE20
#define ADC0_BASE_CHANNEL 3      // PTE22

// PWM Channels used for all axis pta13, 
#define TPM1_UPPER_ARM_CHANNEL 1 // PTA13
#define TPM1_LOWER_ARM_CHANNEL 0 // PTA12
#define TPM0_BASE_CHANNEL 2      // PTA5
#define TPM2_CLAW_CHANNEL 1      // PTA2
#define TPM2_HEAD_CHANNEL 0 // PTB2

// TPM pins
#define PTA13_PIN 13
#define PTA12_PIN 12
#define PTA5_PIN 5
#define PTD0_PIN 0
#define PTD1_PIN 1
#define PTD2_PIN 2
#define PTD3_PIN 3

// ADC pins
#define PTE22_PIN 22
#define PTE21_PIN 21
#define PTE23_PIN 23

// Function prototypes

float interpolate_cnv_to_ms(uint16_t cnv);
void initialize_interrupt();
void delay_ms(uint32_t ms);
uint16_t smaRead(uint8_t channel);
uint16_t analog_read(uint8_t channel);
void initialize_adc(void);
uint16_t map_adc_value_to_cnv(uint16_t adc_val);
void configure_pin_for_adc(PORT_Type *port, uint32_t pin, uint8_t mux_value);
void initialize_tpm(TPM_Type *tpm, uint8_t prescaler);
void initialize_pwm_channel(TPM_Type *tpm, uint8_t channel, uint32_t cnv) ;
void configure_pin_for_tpm(PORT_Type *port, uint32_t pin, uint8_t mux_value);
void PORTA_IRQHandler(void);
void initialize_peripherals(void);
void initialize_adc(void);
void initialize_pwm(TPM_Type *tpm, int channel, uint32_t cnv);
uint16_t analog_read(uint8_t channel);
uint32_t map_angles(uint32_t adc_reading);
uint32_t map_cnv(uint32_t mapped_angle);
void servo_write(uint8_t adc_channel, uint8_t tpm_channel, TPM_Type *tpm);
void control_claw_tpm2(int tpm_channel);
void delay_5ms(void);
void delay_75ms(void);
void delay_500ms(void);
uint16_t readADCWithEMA(uint8_t channel);
void PORTD_IRQHandler(void);
void set_cnv(TPM_Type *tpm, int channel, uint32_t cnv);
#endif // UTILS_H
