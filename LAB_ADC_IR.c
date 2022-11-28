/**
******************************************************************************
* @author  HeoDabin
******************************************************************************
*/

#include "stm32f411xe.h"
#include "ecGPIO.h"
#include "ecRCC.h"
#include "ecTIM.h"
#include "ecSysTick.h"
#include "ecUART.h"
#include "ecADC.h"
#include "ecPWM.h"

#define A 0
#define B 1

//IR parameter//
uint32_t IR1, IR2;
int flag = 0;
int seqCHn[16] = {8,9,};
//ultrasonic
uint32_t ovf_cnt = 0;
float distance_present = 0;
float distance_past;
float timeInterval = 0;
float time1 = 0;
float time2 = 0;
//dc motor
PWM_t dcPwm[2];
float duty = 0;

typedef struct {
	GPIO_TypeDef *port;
	int pin;
}_Pin;

_Pin dcPwmPin[2] = {
	//{GPIOA, 0}, // TIM2 Ch1
	//{GPIOA, 1}	// TIM2 Ch2
	{GPIOC, 8}, // TIM3 Ch3
	{GPIOC, 9}	// TIM3 Ch4
};

_Pin dcDirPin[2] = {
	{GPIOB, 8}, {GPIOC, 6}	
};

void setup(void);
	
int main(void) { 

	// Initialiization --------------------------------------------------------
	setup();
	// Inifinite Loop ----------------------------------------------------------
	while(1){
		distance_present = (float) timeInterval * 340.0 / 2.0 / 10.0; 	// [mm] -> [cm]
		
		if (distance_present - distance_past > 5000); // for prevent malfunction caused by ultrasonic sensors
		
		else if (distance_present < 15){							// collision prevention
			PWM_duty(&dcPwm[A], 1);
			PWM_duty(&dcPwm[B], 1);
			GPIO_write(dcDirPin[A].port, dcDirPin[A].pin, HIGH);
			GPIO_write(dcDirPin[B].port, dcDirPin[B].pin, HIGH);
		}
		else if (IR1 > 1000) {
			GPIO_write(dcDirPin[A].port, dcDirPin[A].pin, LOW);
			GPIO_write(dcDirPin[B].port, dcDirPin[B].pin, LOW);
			PWM_duty(&dcPwm[A], 0.2);
			PWM_duty(&dcPwm[B], 0.4);
		}
		else if (IR2 > 1000) {
			GPIO_write(dcDirPin[A].port, dcDirPin[A].pin, LOW);
			GPIO_write(dcDirPin[B].port, dcDirPin[B].pin, LOW);
			PWM_duty(&dcPwm[A], 0.37);
			PWM_duty(&dcPwm[B], 0.2);
		}
		else{
			GPIO_write(dcDirPin[A].port, dcDirPin[A].pin, LOW);
			GPIO_write(dcDirPin[B].port, dcDirPin[B].pin, LOW);
			PWM_duty(&dcPwm[A], 0.43);
			PWM_duty(&dcPwm[B], 0.4);
		}
		
		distance_past = distance_present;
		delay_ms(15);
	}
}

// Initialiization 
void setup(void)
{	
	RCC_PLL_init();                         // System Clock = 84MHz
	UART2_init();
	SysTick_init();
	
	// ADC setting
  ADC_init(GPIOB, 0, TRGO);		//channel 8
	ADC_init(GPIOB, 1, TRGO);		//channel 9

	// ADC channel sequence setting
	ADC_sequence(2, seqCHn);		//2 channel array
	
	// ADON, SW Trigger enable
	ADC_start();
	
	//PWM configuration ---------------------------------------------------------------------
	PWM_init(&dcPwm[A], dcPwmPin[A].port, dcPwmPin[A].pin);
	PWM_init(&dcPwm[B], dcPwmPin[B].port, dcPwmPin[B].pin);
	
	PWM_period_us(&dcPwm[A], 50000);
	PWM_period_us(&dcPwm[B], 50000);
	
	PWM_duty(&dcPwm[A], 0);
	PWM_duty(&dcPwm[B], 0);
	
	GPIO_write(dcDirPin[A].port, dcDirPin[A].pin, HIGH);
	GPIO_write(dcDirPin[B].port, dcDirPin[B].pin, HIGH);
	
	for (int i = 0; i < 2; i++){
		GPIO_init(dcDirPin[i].port, dcDirPin[i].pin, OUTPUT);
		GPIO_pupd(dcDirPin[i].port, dcDirPin[i].pin, EC_PD);
		GPIO_otype(dcDirPin[i].port, dcDirPin[i].pin, Push_Pull);
		GPIO_ospeed(dcDirPin[i].port, dcDirPin[i].pin, High);
	}
	
	
	// input capture
	// PWM configuration ---------------------------------------------------------------------	
	PWM_t trig;												// PWM1 for trig
	PWM_init(&trig, GPIOA, 6);			 		// PA_6: Ultrasonic trig pulse
	PWM_period_us(&trig, 50000);    	// PWM of 50ms period. Use period_us()
	PWM_pulsewidth_us(&trig, 10);   	// PWM pulse width of 10us
	
// Input Capture configuration -----------------------------------------------------------------------	
	IC_t echo;												// Input Capture for echo
	ICAP_init(&echo, GPIOB, 10);    		// PB10 as input caputre
 	ICAP_counter_us(&echo, 10);   		// ICAP counter step time as 10us
	ICAP_setup(&echo, 3, IC_RISE);   	// TIM2_CH3 as IC3 , rising edge detect
	ICAP_setup(&echo, 4, IC_FALL);   	// TIM2_CH3 as IC4 , falling edge detect
	
}


void ADC_IRQHandler(void){
	if((is_ADC_OVR())){
		clear_ADC_OVR();
	}
	
	if(is_ADC_EOC()){       //after finishing sequence
			if (flag==0){
				IR1 = ADC_read();
			}  
			else if (flag==1){
				IR2 = ADC_read();
			}
		flag =! flag;
	}
}

void TIM2_IRQHandler(void){
	if(is_UIF(TIM2)){                     	// Update interrupt
		ovf_cnt++;														// overflow count
		clear_UIF(TIM2);  							    	// clear update interrupt flag
	}
	if(is_CCIF(TIM2, 3)){ 									// TIM2_Ch3 (IC3) Capture Flag. Rising Edge Detect
		time1 = TIM2->CCR3;										// Capture TimeStart
		clear_CCIF(TIM2, 3);                 	// clear capture/compare interrupt flag 
	}								                      
	else if(is_CCIF(TIM2, 4)){ 							// TIM2_Ch3 (IC4) Capture Flag. Falling Edge Detect
		time2 = TIM2->CCR4;										// Capture TimeEnd
		//timeInterval = (time2- time1 + ovf_cnt * (TIM2->ARR +1)) *10.0 / 1000.0; 					// Total time of echo pulse (10us * counter pulses -> [msec] unit)
		timeInterval = (time2- time1 + ovf_cnt * (TIM2->ARR +1)) *10.0 / 1000.0; 					// Total time of echo pulse (10us * counter pulses -> [msec] unit)
		ovf_cnt = 0;                        	// overflow reset
		clear_CCIF(TIM2, 4);								  // clear capture/compare interrupt flag 
	}
}
