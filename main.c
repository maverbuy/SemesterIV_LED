#include "stm32l476xx.h"

//*************************************  32L476GDISCOVERY ***************************************************************************
// STM32L4:  STM32L476VGT6 MCU = ARM Cortex-M4 + FPU + DSP, 
//           LQFP100, 1 MB of Flash, 128 KB of SRAM
//           Instruction cache = 32 lines of 4x64 bits (1KB)
//           Data cache = 8 lines of 4x64 bits (256 B)
//
// Joystick (MT-008A): 
//   Right = PA2        Up   = PA3         Center = PA0
//   Left  = PA1        Down = PA5
//
// User LEDs: 
//   LD4 Red   = PB2    LD5 Green = PE8
//   
// CS43L22 Audio DAC Stereo (I2C address 0x94):  
//   SAI1_MCK = PE2     SAI1_SD  = PE6    I2C1_SDA = PB7    Audio_RST = PE3    
//   SAI1_SCK = PE5     SAI1_FS  = PE4    I2C1_SCL = PB6                                           
//
// MP34DT01 Digital MEMS microphone 
//    Audio_CLK = PE9   Audio_DIN = PE7
//
// LSM303C eCompass (a 3D accelerometer and 3D magnetometer module): 
//   MEMS_SCK  = PD1    MAG_DRDY = PC2    XL_CS  = PE0             
//   MEMS_MOSI = PD4    MAG_CS  = PC0     XL_INT = PE1       
//                      MAG_INT = PC1 
//
// L3GD20 Gyro (three-axis digital output): 
//   MEMS_SCK  = PD1    GYRO_CS   = PD7
//   MEMS_MOSI = PD4    GYRO_INT1 = PD2
//   MEMS_MISO = PD3    GYRO_INT2 = PB8
//
// ST-Link V2 (Virtual com port, Mass Storage, Debug port): 
//   USART_TX = PD5     SWCLK = PA14      MFX_USART3_TX   MCO
//   USART_RX = PD6     SWDIO = PA13      MFX_USART3_RX   NRST
//   PB3 = 3V3_REG_ON   SWO = PB5      
//
// Quad SPI Flash Memory (128 Mbit)
//   QSPI_CS  = PE11    QSPI_D0 = PE12    QSPI_D2 = PE14
//   QSPI_CLK = PE10    QSPI_D1 = PE13    QSPI_D3 = PE15
//
// LCD (24 segments, 4 commons)
//   VLCD = PC3
//   COM0 = PA8     COM1  = PA9      COM2  = PA10    COM3  = PB9
//   SEG0 = PA7     SEG6  = PD11     SEG12 = PB5     SEG18 = PD8
//   SEG1 = PC5     SEG7  = PD13     SEG13 = PC8     SEG19 = PB14
//   SEG2 = PB1     SEG8  = PD15     SEG14 = PC6     SEG20 = PB12
//   SEG3 = PB13    SEG9  = PC7      SEG15 = PD14    SEG21 = PB0
//   SEG4 = PB15    SEG10 = PA15     SEG16 = PD12    SEG22 = PC4
//   SEG5 = PD9     SEG11 = PB4      SEG17 = PD10    SEG23 = PA6
// 
// USB OTG
//   OTG_FS_PowerSwitchOn = PC9    OTG_FS_VBUS = PC11    OTG_FS_DM = PA11  
//   OTG_FS_OverCurrent   = PC10   OTG_FS_ID   = PC12    OTG_FS_DP = PA12  
//
// PC14 = OSC32_IN      PC15 = OSC32_OUT
// PH0  = OSC_IN        PH1  = OSC_OUT 
// 
// PA4  = DAC1_OUT1 (NLMFX0 WAKEUP)   PA5 = DAC1_OUT2 (Joy Down)
// PA3  = OPAMP1_VOUT (Joy Up)        PB0 = OPAMP2_VOUT (LCD SEG21)
//
//****************************************************************************************************************
/*****************************************************************************************************************
main.c

Embedded Systems Software - Lab #1
Modified by: Mitch Verbuyst
						 Jan 26, 2018

The main.c file was provided as a basis for lab #1.  This file was then modified in order to turn off and on
the red and green LED's of the STM32L4 discovery board, using the on board joystick.

Functions are as follows:

Joystick Center: Toggle RED LED
Joystick Left: Toggle GREEN LED
Joystick Right: Toggle BOTH LED's
Joystick Up: Turn OFF both LED's
Joystick Down: Flash GREEN LED to spell "SOS" in morse code

PB2 -> RED LED (GPIO B)
PE8 -> GREEN LED (GPIO E)
GPIOA -> Joystick

Clock Speed: 16 MHz
******************************************************************************************************************/

//Prototypes
void dot(void);
void dash(void);

//Constants
#define LED_PIN 2
#define LED_GREEN 8
#define JS_MASK 0xFFFFF300
#define JS_PUP 0x8AA
int i = 0;

int main(void){
	
	
		
	// Enable High Speed Internal Clock (HSI = 16 MHz)
  RCC->CR |= ((uint32_t)RCC_CR_HSION);
	
  // wait until HSI is ready
  while ( (RCC->CR & (uint32_t) RCC_CR_HSIRDY) == 0 ) {;}
	
  // Select HSI as system clock source 
  RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
  RCC->CFGR |= (uint32_t)RCC_CFGR_SW_HSI;  //01: HSI16 oscillator used as system clock

  // Wait till HSI is used as system clock source 
  while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) == 0 ) {;}
  
		
	/***************************************RED LED*************************************/
  // Enable the clock to GPIO Port B	
  RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN; 
		
	// GPIO Mode: Input(00), Output(01), AlterFunc(10), Analog(11, reset)
	GPIOB->MODER &= ~(3UL<<(2*LED_PIN));  
	GPIOB->MODER |=   1UL<<(2*LED_PIN);      // Output(01)
	
	// GPIO Speed: Low speed (00), Medium speed (01), Fast speed (10), High speed (11)
	GPIOB->OSPEEDR &= ~(3<<(2*LED_PIN));
	GPIOB->OSPEEDR |=   2<<(2*LED_PIN);  // Fast speed
	
	// GPIO Output Type: Output push-pull (0, reset), Output open drain (1) 
	GPIOB->OTYPER &= ~(1<<LED_PIN);      // Push-pull

	// GPIO Push-Pull: No pull-up, pull-down (00), Pull-up (01), Pull-down (10), Reserved (11)
	GPIOB->PUPDR  &= ~(3<<(2*LED_PIN));  // No pull-up, no pull-down
	
  // Light up the LED	
	GPIOB->ODR |= 1 << LED_PIN;
	

	/*******************************************GREEN LED*******************************/
	//Enable the clock to GPIO Port E
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOEEN;
	
	// GPIO Mode: Input(00), Output(01), AlterFunc(10), Analog(11, reset)
	GPIOE->MODER &= ~(3UL<<(2*LED_GREEN));  
	GPIOE->MODER |=   1UL<<(2*LED_GREEN);      // Output(01)	

	// GPIO Speed: Low speed (00), Medium speed (01), Fast speed (10), High speed (11)
	GPIOE->OSPEEDR &= ~(3<<(2*LED_GREEN));
	GPIOE->OSPEEDR |=   2<<(2*LED_GREEN);  // Fast speed (10)
	
	// GPIO Output Type: Output push-pull (0, reset), Output open drain (1) 
	GPIOE->OTYPER &= ~(1<<LED_GREEN);      // Push-pull (0)
	
	// GPIO Push-Pull: No pull-up, pull-down (00), Pull-up (01), Pull-down (10), Reserved (11)
	GPIOE->PUPDR  &= ~(3<<(2*LED_GREEN));  // No pull-up, no pull-down (00)
	
	// Light up the LED	
	GPIOE->ODR |= 1 << LED_GREEN;
	
	
	/**********************************JOYSTICK***************************************/
	//Enable the clock to GPIO Port A
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN;
	
	//GPIO Mode: Input(00)
	GPIOA->MODER &= JS_MASK;
	
	//GPIOA Push-Pull: pull-down(10)
	GPIOA->PUPDR &= JS_MASK;
	GPIOA->PUPDR |= JS_PUP;
	
	while(1){
		//toggle RED LED when center button is pushed
		if((GPIOA->IDR & 0x1) == 0x1){
			GPIOB->ODR ^= 1 << LED_PIN;
			while ((GPIOA->IDR &0x1) != 0x00);
		}
		
		//toggle GREEN LED when left button is pushed
		if((GPIOA->IDR & 0x2) == 0x2){
			GPIOE->ODR ^= 1 << LED_GREEN;
			while ((GPIOA->IDR &0x2) != 0x00);
		}
		
		//toggle both LED's when right button is pushed
		if((GPIOA->IDR & 0x4) == 0x4){
			GPIOE->ODR ^= 1 << LED_GREEN;
			GPIOB->ODR ^= 1 << LED_PIN;
			while ((GPIOA->IDR &0x4) != 0x00);
		}
		
		//turn both LEDs off when up is pushed
		if((GPIOA->IDR & 0x8) == 0x8){
			GPIOE->ODR = 0;
			GPIOB->ODR = 0;
			while ((GPIOA->IDR &0x8) != 0x00);
		}
		
		//play morse code for SOS on green LED
		if((GPIOA->IDR & 0x20) == 0x20){
			
			GPIOE->ODR = 0;
			
			for (i = 0; i < 1000000; i++){} //start off 

			//the following sequence of dots and dashes spells SOS in morse code
			dot();
			dot();
			dot();
			dash();
			dash();
			dash();
			dot();
			dot();
			dot();
			dot();
		}
	}
}

//function for dot in morse code
//dot is roughly 0.25 secs, with 0.25 secs between flashes
void dot(void)
{
	GPIOE->ODR = 0;
	for (i = 0; i < 100000; i++){}
				
	GPIOE->ODR = 1 << LED_GREEN;
	for (i = 0; i < 200000; i++){}

	GPIOE->ODR = 0;
}

//function for dash in morse code
//dash is roughly 0.5 secs, with 0.25 secs between flashes
void dash(void)
{
	GPIOE->ODR = 0;
	for (i = 0; i < 100000; i++){}

	GPIOE->ODR = 1 << LED_GREEN;
	for (i = 0; i < 400000; i++){}

	GPIOE->ODR = 0;
}
