/*
* File:    main.c
* Title:   Concept test code to obtain position mesurement
*          using both potentiometer and encoder outputs.
*		   Outputs are emulated by software.
* Version: 17.02
* Created: 15/03/2022
* Author:  BKO
*
*/


/*
* Design Choices and Justifications
*
* The range of motion is only 0-80 degrees.
*
* For Potentiometer -
* As Vcc is 3.3v the maximum voltage is 2.93v.
* Using a 8 bit DAC the maximum bitwise value is 226.
* 
* For encoder - 
* 512 counts per 360 degrees
* 113 counts per 80  degrees
* 
* Transiencies - In moving average mode, the first 5 values 
* are incorrect, due to insufficient samples being gathered. 
*
*/


/*
* This program requires the following physical connections
* to be made.
* 
* PA4 (DAC Output) 		   to PA5 (OpAmp Input)
* PA2 (OpAmp Output)	   to PA1 (ADC Input)
* PC8 (Encoder CHA Output) to PB4 (Encoder CHA Input)
* PC9 (Encoder CHB Output) to PB2 (Encoder CHB Input)
*
*/

/*
* The input and output peripherals are listed below. 
* 
* B1 USER - PA0
* LD3     - PE9
* LD4     - PE8
* LD5     - PE10
* LD6     - PE15
* LD7     - PE11
* LD8     - PE14
* LD9     - PE12
* LD10    - PE13
*/

//Include required libaries and header files
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include "stm32f3xx.h" 

//Function Prototypes
void InitialiseOutputLEDs(void);
void InitialiseTriangleWaveInterrupt(void);
void InitialiseUserInterrupt(void);

//For Potentiometer
void InitialisePotDAC(void);
void InitialisePotOpAmp(void);
void InitialisePotADC(void);
void CalculatePotAverage(void);
void delay(int);

//For Encoder
void InitialiseEncoderOutput(void);
void EncoderOutputFSM(void);
void InitialiseEncoderInputInterrupt(void);
void EncoderInput(void);


//Initialise global variables
int  TriangleWave = 0;
int Mode = 0;
bool TriangleWaveFlag = false; //False = Ramping up, True = Ramping Down

//For Potentiometer
int PotCounter = 0;
int PotSamples[5] = {0,0,0,0,0};
int PotSamplesTotal = 0;
int PotSamplesPos = 0;
int PotAverage = 0;

//For Encoder
int PreviousState = 0;
int CurrentState = 0;
int EncoderCounter = 0; 
int EncoderState = 0; 
bool DivideByTwoFlag = true;

int main(void)
{
	//Initialisation
	InitialiseOutputLEDs();
	InitialiseTriangleWaveInterrupt();
	InitialiseUserInterrupt();
	InitialisePotDAC();
	InitialisePotOpAmp();
	InitialisePotADC();
	InitialiseEncoderOutput();
	InitialiseEncoderInputInterrupt();

	//Store initial state of CHA Encoder
	PreviousState = (GPIOB -> IDR & GPIO_IDR_4) >> 4; //Right shift 4 to make LSB to represent state
	
	
	while (1) {
		
		GPIOE->BSRRH |= 0xFFFF;					  	  //Turn all LEDs off
		switch (Mode) {
			
			//Potentiometer test mode
			case 0 :	
				GPIOE->BSRRL |= 0x100; 			      //Display Mode LED
				//Obtain value from ADC
				ADC1->CR |= ADC_CR_ADSTART; 	  	  //Start Conversion
				while (!(ADC1->ISR & ADC_ISR_EOC));   //Wait for end of converstion
				PotCounter = (ADC1->DR/8);		      //Scale down ADC 8 bit resolution to 5 bits
				GPIOE->BSRRL=(PotCounter << 11);	  //Display Potentiometer value
				break;
			
			//Encoder test mode
			case 1 :
				GPIOE->BSRRL |= 0x200; 			       //Display Mode LED
				//Display Encoder Count value
				GPIOE->BSRRL = EncoderCounter/2 << 11;//Scale down Encoder 6 bit resolution to 5 bits
				break;
			
			//Combined output test mode
			case 2 :
				GPIOE->BSRRL |= 0x400;  			  //Display Mode LED
				CalculatePotAverage();
				//Average and display value
				//Scale down Encoder and Potentiometer to 5 bits
				GPIOE->BSRRL = (((EncoderCounter/2)+PotAverage/8)/2)<< 11;
				break;	
		};
	};
};

void InitialiseOutputLEDs()
{
	//Direct clock to GPIOE
	RCC->AHBENR   |=  RCC_AHBENR_GPIOEEN;
	//Initialise All Output LEDS
	//General Purpose Output Mode	
	GPIOE->MODER  |=  GPIO_MODER_MODER8_0; 
	GPIOE->MODER  &= ~GPIO_MODER_MODER8_1;
	GPIOE->MODER  |=  GPIO_MODER_MODER9_0; 
	GPIOE->MODER  &= ~GPIO_MODER_MODER9_1;
	GPIOE->MODER  |=  GPIO_MODER_MODER10_0; 
	GPIOE->MODER  &= ~GPIO_MODER_MODER10_1;
	GPIOE->MODER  |=  GPIO_MODER_MODER11_0; 
	GPIOE->MODER  &= ~GPIO_MODER_MODER11_1;
	GPIOE->MODER  |=  GPIO_MODER_MODER12_0; 
	GPIOE->MODER  &= ~GPIO_MODER_MODER12_1;
	GPIOE->MODER  |=  GPIO_MODER_MODER13_0; 
	GPIOE->MODER  &= ~GPIO_MODER_MODER13_1;
	GPIOE->MODER  |=  GPIO_MODER_MODER14_0; 
	GPIOE->MODER  &= ~GPIO_MODER_MODER14_1;
	GPIOE->MODER  |=  GPIO_MODER_MODER15_0; 
	GPIOE->MODER  &= ~GPIO_MODER_MODER15_1;	
	//Push Pull
	GPIOE->OTYPER &= ~GPIO_OTYPER_OT_8; 
	GPIOE->OTYPER &= ~GPIO_OTYPER_OT_9; 
	GPIOE->OTYPER &= ~GPIO_OTYPER_OT_10; 	
	GPIOE->OTYPER &= ~GPIO_OTYPER_OT_11; 
	GPIOE->OTYPER &= ~GPIO_OTYPER_OT_12; 
	GPIOE->OTYPER &= ~GPIO_OTYPER_OT_13; 
	GPIOE->OTYPER &= ~GPIO_OTYPER_OT_14; 
	GPIOE->OTYPER &= ~GPIO_OTYPER_OT_15; 
	//No pull-up, pull-down
	GPIOE->PUPDR  &= ~GPIO_PUPDR_PUPDR8; 
	GPIOE->PUPDR  &= ~GPIO_PUPDR_PUPDR9; 
	GPIOE->PUPDR  &= ~GPIO_PUPDR_PUPDR10; 
	GPIOE->PUPDR  &= ~GPIO_PUPDR_PUPDR11; 
	GPIOE->PUPDR  &= ~GPIO_PUPDR_PUPDR12; 
	GPIOE->PUPDR  &= ~GPIO_PUPDR_PUPDR13; 
	GPIOE->PUPDR  &= ~GPIO_PUPDR_PUPDR14; 
	GPIOE->PUPDR  &= ~GPIO_PUPDR_PUPDR15; 
};

void InitialiseTriangleWaveInterrupt(){
	//Configure timer to run every 0.0022 seconds
	//Triangle wave has 452 samples
	//which gives a 1 sec triangle wave
	RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; //Direct clock pulses to timer
	TIM3->PSC = 999;					//Set pre-scaler register
	TIM3->ARR = 15;						//Set auto-reload register
	TIM3->CR1 |= TIM_CR1_CEN;           //Enable Timer
	TIM3->DIER |= TIM_DIER_UIE;			//Set DIER to watch out for update interrupt
	NVIC_EnableIRQ(TIM3_IRQn);			//Set interrupt request in NVIC
};


//IRQ Handler for Triangle Wave
void TIM3_IRQHandler()
{
	if ((TIM3->SR & TIM_SR_UIF) !=0)  //Check Interrupt source
	{
		//Ramp Up Triangle Wave
		if (TriangleWaveFlag == false) {TriangleWave++;}
		//Ramp Down Triangle Wave
		else if (TriangleWaveFlag == true) {TriangleWave--;}
		
		//Limit Triangle wave amplituite
		if (TriangleWave == 226) {TriangleWaveFlag = true;}
		else if (TriangleWave == 0) {TriangleWaveFlag = false;}

		//Send triangle wave to pot DAC
		DAC1->DHR12R1 = TriangleWave;
		
		//Increment Encoder Output Position
		//Divide Encoder increment count by two due to scaling
		if (DivideByTwoFlag == true) {
			EncoderOutputFSM();
			DivideByTwoFlag = false;
		}
		else {
			DivideByTwoFlag = true;
		}
	}
	TIM3->SR &= ~TIM_SR_UIF;        //Reset interrupt flag in SR register
};

//Encoder Output Controller
void EncoderOutputFSM()
{
	//Turn Encoder Output Off
	GPIOC->BSRRH |= 0x300;

	if (TriangleWaveFlag == false){
		//Anti-Clockwise
		switch(EncoderState)
			{
				case 10:
					GPIOC->BSRRL = 0x200; //Change output accoding to state
					EncoderState = 00;    //Change to next state
					break;
				
				case 00:
					EncoderState = 01;    //Change to next state
					break;
				
				case 01:
					GPIOC->BSRRL = 0x100; //Change output accoding to state
					EncoderState = 11;    //Change to next state
					break;	

				case 11:
					GPIOC->BSRRL = 0x300; //Change output accoding to state
					EncoderState = 10;    //Change to next state
					break;
			}
	}
	else{
		//Clockwise
		switch(EncoderState)
			{
				case 11:
					GPIOC->BSRRL = 0x300; //Change output accoding to state
					EncoderState = 01;    //Change to next state
					break;
				
				case 01:
					GPIOC->BSRRL = 0x100; //Change output accoding to state
					EncoderState = 00;    //Change to next state
					break;
				
				case 00:
					EncoderState = 10;    //Change to next state
					break;	

				case 10:
					GPIOC->BSRRL = 0x200; //Change output accoding to state
					EncoderState = 11;    //Change to next state
					break;
			}		
	}
};

//Configure User Button
void InitialiseUserInterrupt()
{
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;          //Set APB 2 peripheral clock
	EXTI->IMR |= EXTI_IMR_MR0; 					  //Remove mask from line 0
	EXTI->RTSR |= EXTI_RTSR_TR0; 				  //Set Rising edge trigger
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI0_PA; //Set PA0 to generate interrpt
	NVIC_EnableIRQ(EXTI0_IRQn);                   //Configure NVIC
	NVIC_SetPriority(EXTI0_IRQn, 0);              //Set lowest priority
};

//IRQ Handler for User Button
void EXTI0_IRQHandler()
{
	if (EXTI->PR & EXTI_PR_PR0)  //Check source
	{
	EXTI->PR |= EXTI_PR_PR0;     //Clear flag
		Mode++;					 //Change Mode
		if (Mode>2) Mode = 0;	 //Reset Mode
	}
};

//Configure DAC
void InitialisePotDAC()
{
	GPIOA->MODER |= GPIO_MODER_MODER4;   //Set PA4 to analogue mode
	RCC->APB1ENR |= RCC_APB1ENR_DAC1EN;  //Direct clock pulses to DAC1
	DAC1->CR |= DAC_CR_BOFF1;   		 //Disable Buffer function
	DAC1->CR |= DAC_CR_EN1;   			 //Enable DAC
};

//Configure Op Amp
void InitialisePotOpAmp()
{
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;//Connect Op-Amp to System Clock via APB2 Bus
	GPIOA->MODER |= GPIO_MODER_MODER5;   //Set analogue mode for PA5
	GPIOA->MODER |= GPIO_MODER_MODER2;   //Set analogue mode for PA2	
	OPAMP1->CSR |= OPAMP1_CSR_OPAMP1EN;  //Enable Op amp
	OPAMP1->CSR |= OPAMP1_CSR_VPSEL_0;   //Set PA5 as non-inverting input
	OPAMP1->CSR |= OPAMP1_CSR_VMSEL_1;   //Enable PGA mode
	OPAMP1->CSR |= OPAMP1_CSR_PGGAIN;    //Set PGA gain to 16
};

void InitialisePotADC()
{
	//Enable Voltage regulator
	ADC1->CR &= ~ADC_CR_ADVREGEN; //Set Intermediate state 
	ADC1->CR |= ADC_CR_ADVREGEN_0;//Enable regulator
	delay (100);				  //Allow regulator to start
	
	//Calibrate the ADC 
	ADC1->CR &= ~ADC_CR_ADEN ;      //Ensure ADC is disabled
	ADC1->CR &= ~ADC_CR_ADCALDIF;   //Set single ended input 
	ADC1->CR |= ADC_CR_ADCAL;       //Start Calibraiton
	while(ADC1->CR & ADC_CR_ADCAL); //Wait until Calibration is completed
	
	//Enable clock connection to ADC 
	RCC          ->CFGR2  |= RCC_CFGR2_ADCPRE12_DIV2;//Set Prescaler
	RCC          ->AHBENR |= RCC_AHBENR_ADC12EN;     //Enable Clock
	ADC1_2_COMMON->CCR    |= ADC12_CCR_CKMODE_0;     //Set to DIV/1

	//Configure GPIO
	RCC  ->AHBENR |= RCC_AHBENR_GPIOAEN;//Enable Clock
	GPIOA->MODER  |= GPIO_MODER_MODER1; //Set PA1 to analogue mode
	
	//Configure ADC 
	ADC1->CFGR &=  ADC_CFGR_RES; 
	ADC1->CFGR |=  ADC_CFGR_RES_1;	//Set to 8 bits resolution
	ADC1->CFGR &= ~ADC_CFGR_ALIGN;	//Set right hand data aligment
	ADC1->CFGR &= ~ADC_CFGR_CONT; 	//Set single Conversion mode
	
	//Configure the Multiplexer
	ADC1->SQR1 &= ~ADC_SQR1_SQ1; 
	ADC1->SQR1 |= ADC_SQR1_SQ1_1; 	//Set to channel 2	
	ADC1->SQR1 &= ~ADC_SQR1_L;
	ADC1->SQR1 |= ADC_SQR1_L_0; 	//Set length of sequence to 1

	//Configure Sample Time
	ADC1->SMPR1 &= ~ADC_SMPR1_SMP1;
	ADC1->SMPR1 |= ADC_SMPR1_SMP1_0;//Set to 7.5 ADC cycles
	ADC1->SMPR1 |= ADC_SMPR1_SMP1_1;
	
	ADC1->CR |= ADC_CR_ADEN;   		    //Enable the ADC
	while(!(ADC1->ISR & ADC_ISR_ADRD)); //Wait until ADC ready
};

//Delay function for ADC voltage regulator
void delay (int a)
{
    volatile int i,j;
    for (i=0 ; i < a ; i++){j++;} //Occupy Processor
    return;
}

//Configure Encoder Output
void InitialiseEncoderOutput()
{
	//Direct clock to GPIOC
	RCC->AHBENR   |=  RCC_AHBENR_GPIOCEN;
	//General Purpose Output Mode	
	GPIOC->MODER  |=  GPIO_MODER_MODER8_0; 
	GPIOC->MODER  &= ~GPIO_MODER_MODER8_1;
	GPIOC->MODER  |=  GPIO_MODER_MODER9_0; 
	GPIOC->MODER  &= ~GPIO_MODER_MODER9_1;
	//Push Pull
	GPIOC->OTYPER &= ~GPIO_OTYPER_OT_8; 
	GPIOC->OTYPER &= ~GPIO_OTYPER_OT_9; 
	//No pull-up, pull-down
	GPIOC->PUPDR  &= ~GPIO_PUPDR_PUPDR8; 
	GPIOC->PUPDR  &= ~GPIO_PUPDR_PUPDR9; 
};

//Configure Encoder Input Interrpts
void InitialiseEncoderInputInterrupt(){

	//Direct clock to GPIOB
	RCC->AHBENR  |=  RCC_AHBENR_GPIOBEN;
	//General Purpose Output Mode
	GPIOB->MODER &= ~GPIO_MODER_MODER4; 
	GPIOB->MODER &= ~GPIO_MODER_MODER2; 
	//Pull up Resistors
	GPIOB->PUPDR &= ~GPIO_PUPDR_PUPDR4; 
	GPIOB->PUPDR |= GPIO_PUPDR_PUPDR4_0; 
	GPIOB->PUPDR &= ~GPIO_PUPDR_PUPDR2;	
	GPIOB->PUPDR |=  GPIO_PUPDR_PUPDR2_0; 	
	
	//Configure Interrupts
	RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;		 //Direct Clock Pulses
	EXTI->IMR |= EXTI_IMR_MR4;            	   	 //Mask values
	EXTI->RTSR |= EXTI_RTSR_TR4;          		 //Set rising edge trigger
	EXTI->FTSR |= EXTI_FTSR_TR4;				 //Set faling edge trigger
	SYSCFG->EXTICR[1] |= SYSCFG_EXTICR2_EXTI4_PB;//Set IRQ port to PB4
	NVIC_SetPriority(EXTI4_IRQn,0);				 //Set low priority in NVIC
	NVIC_EnableIRQ(EXTI4_IRQn); 			     //Enable IRQ in NVIC
	
	EXTI->IMR |= EXTI_IMR_MR2;                   //Mask values
	EXTI->RTSR |= EXTI_RTSR_TR2;				 //Set rising edge trigger
	EXTI->FTSR |= EXTI_FTSR_TR2;				 //Set faling edge trigger
	SYSCFG->EXTICR[0] |= SYSCFG_EXTICR1_EXTI2_PB;//Set IRQ port to PB2
	NVIC_SetPriority(EXTI2_TSC_IRQn,1);			 //Set low priority in NVIC
	NVIC_EnableIRQ(EXTI2_TSC_IRQn);              //Enable IRQ in NVIC
};

//ISR for Encoder CHA
void EXTI4_IRQHandler()
{
	if (EXTI->PR & EXTI_PR_PR4)
	{
		//Update Position
		EncoderInput();
	}
	EXTI->PR |= EXTI_PR_PR4;
};

//ISR for Encoder CHB
void EXTI2_TSC_IRQHandler()
{
	if (EXTI->PR & EXTI_PR_PR2)
	{

		//UpdatePosition
		EncoderInput();
	}
	EXTI->PR |= EXTI_PR_PR2;
};

//Function to calculate position of encoder
void EncoderInput(){
	
	CurrentState = (GPIOB -> IDR & GPIO_IDR_4) >> 4; //Store Current State of CHA

	//Check if previous state of CHA is different from current state
	//Which indicates a pulse has elapsed
	if ( (PreviousState != CurrentState) && (CurrentState == 1) ) {
		//Check if CHA state is different to CHB to determine direction of rotation
		
		if (((GPIOB->IDR & GPIO_IDR_2)>>2) != (CurrentState) )
			{EncoderCounter+=2;} 		//Increment Counter (clockwise)
		else {EncoderCounter-=2;}      //Decrmenet Counter (anti-clockwise)
	}
	//Store processed state as last state
	PreviousState = CurrentState;
}

//Function to calculate 5 points moving average of potentiometer
void CalculatePotAverage(){
		ADC1->CR |= ADC_CR_ADSTART;				     //Start conversion
		while (!(ADC1->ISR & ADC_ISR_EOC));		     //Wait for the end of conversion
		PotCounter = (ADC1->DR);				     //Store ADC value in variable
		PotSamples[PotSamplesPos] = PotCounter;      //Store ADC value in array
		PotSamplesPos++;						     //Increment Array Position
	  	if (PotSamplesPos > 4) PotSamplesPos = 0;    //Reset Array Position
		for (int i=0; i < 4; i++) {PotSamplesTotal += PotSamples[i];} //Calculate array total
		PotAverage = PotSamplesTotal/5;              //Calculate sample average
		PotSamplesTotal = 0;                         //Reset total samples
};
