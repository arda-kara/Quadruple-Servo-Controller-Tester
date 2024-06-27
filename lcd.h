#include <MKL25Z4.H>
#include <math.h>
#include <stdio.h>
#define RS 0x04     /* PTA2 mask */ 
#define RW 0x10     /* PTA4 mask */ 
#define EN 0x20     /* PTA5 mask */


void Delay(volatile unsigned int time_del);
void LCD_command(unsigned char command);
void LCD_data(unsigned char data);
void LCD_init(void);
void LCD_ready(void);
void print_fnc(unsigned char *data);
void clear_lcd(void);
void printDigits(int num);
extern char big;
extern char mid;
extern char low;
void initGPIO(void);



void initGPIO(void){
	SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK; // clock for portE
	SIM->SCGC6 |= (1ul << 27); //enable clock for ADC0
	PORTE -> PCR[21] = PORT_PCR_MUX(1); //pte21 gpio
	PORTE -> PCR[22] = PORT_PCR_MUX(1); //pte22 gpio
	PTE -> PDDR |= (1ul << 21);
	PTE -> PDDR |= (1ul << 22);
	// PTE-> PDDR &= ~(1ul << 20); //PTE20 input
	PORTE -> PCR[20] = PORT_PCR_MUX(0); //Analog input
	ADC0 -> SC2 &= ~(1ul << 6); //software triggered
	ADC0 -> CFG1 = 0x50; // clock div by 4, long sample time, 8 bit resolution, bus clock
}

void clear_lcd(void)
{	
	int i;
	LCD_command(0x80); //Start from the 1st line
	for(i = 16; i > 0; i--)
	{
			LCD_data(' '); //Clear the 1st line
	}
	LCD_command(0xC0); //Go to the 2nd line
	for(i = 16; i > 0; i--)
	{
			LCD_data(' '); //Clear the 2nd line
	}
	LCD_command(0x80);
}


void print_fnc(unsigned char *data)
{
	int i = 0 ;
	//Continue until a NULL char comes
	while(data[i] != 0x00)
	{
		LCD_data(data[i]);
		i++;
	}
}


   
void LCD_init(void)
{
    SIM->SCGC5 |= 0x1000;       /* enable clock to Port D */ 
    PORTD->PCR[0] = 0x100;      /* make PTD0 pin as GPIO */
    PORTD->PCR[1] = 0x100;      /* make PTD1 pin as GPIO */
    PORTD->PCR[2] = 0x100;      /* make PTD2 pin as GPIO */
    PORTD->PCR[3] = 0x100;      /* make PTD3 pin as GPIO */
    PORTD->PCR[4] = 0x100;      /* make PTD4 pin as GPIO */
    PORTD->PCR[5] = 0x100;      /* make PTD5 pin as GPIO */
    PORTD->PCR[6] = 0x100;      /* make PTD6 pin as GPIO */
    PORTD->PCR[7] = 0x100;      /* make PTD7 pin as GPIO */
    PTD->PDDR = 0xFF;           /* make PTD7-0 as output pins */
    
    SIM->SCGC5 |= 0x0200;       /* enable clock to Port A */ 
    PORTA->PCR[2] = 0x100;      /* make PTA2 pin as GPIO */
    PORTA->PCR[4] = 0x100;      /* make PTA4 pin as GPIO */
    PORTA->PCR[5] = 0x100;      /* make PTA5 pin as GPIO */
    PTA->PDDR |= 0x34;          /* make PTA5, 4, 2 as output pins */
    
    LCD_command(0x38);      /* set 8-bit data, 2-line, 5x7 font */
    LCD_command(0x01);      /* clear screen, move cursor to home */
    LCD_command(0x0F);      /* turn on display, cursor blinking */
}

/* This function waits until LCD controller is ready to
 * accept a new command/data before returns.
 */
void LCD_ready(void)
{
    uint32_t status;
    
    PTD->PDDR = 0x00;          /* PortD input */
    PTA->PCOR = RS;         /* RS = 0 for status */
    PTA->PSOR = RW;         /* R/W = 1, LCD output */
    
    do {    /* stay in the loop until it is not busy */
			  PTA->PCOR = EN;
			  Delay(500);
        PTA->PSOR = EN;     /* raise E */
        Delay(500);
        status = PTD->PDIR; /* read status register */
        PTA->PCOR = EN;
        Delay(500);			/* clear E */
    } while (status & 0x80UL);    /* check busy bit */
    
    PTA->PCOR = RW;         /* R/W = 0, LCD input */
    PTD->PDDR = 0xFF;       /* PortD output */
}

void LCD_command(unsigned char command)
{
    LCD_ready();			/* wait until LCD is ready */
    PTA->PCOR = RS | RW;    /* RS = 0, R/W = 0 */
    PTD->PDOR = command;
    PTA->PSOR = EN;         /* pulse E */
    Delay(500);
    PTA->PCOR = EN;
}

void LCD_data(unsigned char data)
{
    LCD_ready();			/* wait until LCD is ready */
    PTA->PSOR = RS;         /* RS = 1, R/W = 0 */
    PTA->PCOR = RW;
    PTD->PDOR = data;
    PTA->PSOR = EN;         /* pulse E */
    Delay(500);
    PTA->PCOR = EN;
}



void Delay(volatile unsigned int time_del) {
  while (time_del--) 
		{
  }
}
