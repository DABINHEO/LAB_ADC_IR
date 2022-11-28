### LAB : ADC - IR sensor

**Date:** 2022.12.01

**Author/Partner:** Heo Dabin/ Hwang Seungeon 

**Github:** [Github code](https://github.com/DABINHEO/LAB_ADC_IR.git)

**Demo Video:** [Youtube Link](https://youtube.com/shorts/grO_D8HMQTM?feature=share)

##            



### Introduction

In this LAB, we control the movement of RC cars along the black line by using two IR reflective sensors. It will receive an analog signal through IR reflective sensors, which will be converted into digital signals using ADCs. The ADCs are triggered by a timer of given sampling rate. For the control of RC car, dc motor and ultra sonic sensor were additionally used. The speed of the dc motor can be controlled through the pwm signal, and a pin that can control the direction is also assigned separately. And ultrasonic distance sensor will be used in input capture mode to create a program to measure the distance between the sensor and an object from which ultrasonic waves collide. We used a pwm signal with a pulse width of 10 [usec], with a period of 50 [msec] as a trigger for the ultrasonic distance sensor, and take an echo pulse from the input capture pin to calculate the distance to the object. The trigger of the ultrasonic distance sensor and the pwm for controlling the speed of the DC motor use the same timer and are controlled through different channels.



### Requirement

#### Hardware

* MCU

  * NUCLEO-F401RE

* Sensor

  * IR Reflective Sensor (TCRT 5000) x2
  * HC-SR04

* Actuator

  * DC motor x2

  

#### Software

* Keil uVision, CMSIS, EC_HAL library

##          



### Problem1: Create HAL library

We wrote a header code for converting analog signals to digital. The detailed description of the code is as follows.

#### Description with Code

* ecADC.c  description

```c++
void ADC_init(GPIO_TypeDef *port, int pin, int trigmode){  //mode 0 : SW, 1 : TRGO
    // 0. Match Port and Pin for ADC channel	
    int CHn = ADC_pinmap(port, pin);			// ADC Channel <->Port/Pin mapping

    // GPIO configuration ---------------------------------------------------------------------	
    // 1. Initialize GPIO port and pin as ANALOG, no pull up / pull down
    GPIO_init(port, pin, ANALOG);  				// ANALOG = 3
    GPIO_pupd(port, pin, EC_NUD);  				// EC_NONE = 0

    // ADC configuration	---------------------------------------------------------------------			
    // 1. Total time of conversion setting
    // Enable ADC pheripheral clock
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; 		// Enable the clock of RCC_APB2ENR_ADC1EN

    // Configure ADC clock pre-scaler
    ADC->CCR &= ~ADC_CCR_ADCPRE;				// 0000: PCLK2 divided by 2	(42MHz)

    // Configure ADC resolution 
    ADC1->CR1 &= ~ADC_CR1_RES;     				// 00: 12-bit resolution (15cycle+)

    // Configure channel sampling time of conversion.	
    // Software is allowed to write these bits only when ADSTART=0 and JADSTART=0	!!
    // ADC clock cycles @42MHz = 2us
    if(CHn < 10) {
        ADC1->SMPR2 &= ~(7 << 3*CHn);			// clear bits
        ADC1->SMPR2 |= 4U << 3*CHn;				// sampling time conversion : 84  			
    }
    else{
        ADC1->SMPR1 &= ~(7 << (3* (CHn - 10)));
        ADC1->SMPR1 |= 4U << 3* (CHn - 10);
    }

    // 2. Regular / Injection Group 
    //Regular: SQRx, Injection: JSQx

    // 3. Repetition: Single or Continuous conversion
    ADC1->CR2 &= ~ADC_CR2_CONT;      			// default : Single conversion mode

    // 4. Single Channel or Scan mode	
    // Configure the sequence length
    ADC1->SQR1 &= ADC_SQR1_L; 					// 0000: 1 conversion in the regular channel conversion sequence

    // Configure the channel sequence 
    ADC1->SQR3 &= ~ADC_SQR3_SQ1;				// SQ1 clear bits
    ADC1->SQR3 |= (CHn & ADC_SQR3_SQ1); 		// Choose the channel to convert firstly

    // Single Channel: scan mode, right alignment
    ADC1->CR1 |= ADC_CR1_SCAN;					// 1: Scan mode enable 
    ADC1->CR2 &= ~ADC_CR2_ALIGN;   				// 0: Right alignment	

    // 5. Interrupt Enable
    // Enable EOC(conversion) interrupt. 
    ADC1->CR1 &= ~ADC_CR1_EOCIE;          		// Interrupt reset
    ADC1->CR1 |= ADC_CR1_EOCIE;           		// Interrupt enable

    // Enable ADC_IRQn 
    NVIC_SetPriority(ADC_IRQn, 2); 				// Set Priority to 2
    NVIC_EnableIRQ(ADC_IRQn);      				// Enable interrupt form ACD1 peripheral	


    /* -------------------------------------------------------------------------------------*/
    //					HW TRIGGER MODE
    /* -------------------------------------------------------------------------------------*/

    // TRGO Initialize : TIM3, 1msec, RISE edge
    if(trigmode == TRGO) ADC_TRGO(TIM3, 1, RISE);				

}

void ADC_TRGO(TIM_TypeDef* TIMx, int msec, int edge){
    // set timer
    int timer = 0;
    if(TIMx == TIM2) timer=2;
    else if(TIMx == TIM3) timer=3;	

    // Single conversion mode (disable continuous conversion)
    ADC1->CR2 &= ~ADC_CR2_CONT;     			// Discontinuous conversion mode
    ADC1->CR2 |= ADC_CR2_EOCS;  				// Enable EOCS


    // HW Trigger configuration -------------------------------------------------------------

    // 1. TIMx Trigger Output Config
    // Enable TIMx Clock
    TIM_init(TIMx, msec);
    TIMx->CR1 &= ~TIM_CR1_CEN; 					//counter disable

    // Set PSC, ARR
    // TIM_period_ms(TIMx, msec);

    // Master Mode Selection MMS[2:0]: Trigger output (TRGO)
    TIMx->CR2 &= ~TIM_CR2_MMS; 					// reset MMS
    TIMx->CR2 |= TIM_CR2_MMS_2;   				//100: Compare - OC1REF signal is used as trigger output (TRGO)

    // Output Compare Mode
    TIMx->CCMR1 &= ~TIM_CCMR1_OC1M;     		// OC1M : output compare 1 Mode 
    TIMx->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2;  // OC1M = 110 for compare 1 Mode ch1 

    // OC1 signal 
    TIMx->CCER |= TIM_CCER_CC1E;          		// CC1E Capture enabled
    TIMx->CCR1  = (TIMx->ARR)/2; 				// duty ratio 50%

    // Enable TIMx 
    TIMx->CR1 |= TIM_CR1_CEN; 					//counter enable

    // 2. ADC HW Trigger Config.
    // Select Trigger Source  			
    ADC1->CR2 &= ~ADC_CR2_EXTSEL; 				// reset EXTSEL
    ADC1->CR2 |= (timer*2 + 2) << 24; 			// TIMx TRGO event (ADC : TIM2, TIM3 TRGO)

    //Select Trigger Polarity
    ADC1->CR2 &= ~ADC_CR2_EXTEN;						// reset EXTEN, default
    if(edge==RISE) ADC1->CR2 |= ADC_CR2_EXTEN_0;		// trigger detection rising edge
    else if(edge==FALL) ADC1->CR2 |= ADC_CR2_EXTEN_1;	// trigger detection falling edge
    else if(edge==BOTH) ADC1->CR2 |= ADC_CR2_EXTEN_Msk;	// trigger detection both edge

}

void ADC_continue(int contmode){
    if(contmode == CONT){
        // Repetition: Continuous conversion
        ADC1->CR2 |= ADC_CR2_CONT;      		// Enable Continuous conversion mode	
        ADC1->CR1 &= ~ADC_CR1_SCAN;				// 0: Scan mode disable 
    }
    else { 										//if(contmode==SINGLE)
        // Repetition: Single conversion
        ADC1->CR2 &= ~ADC_CR2_CONT;      		// Disable Continuous conversion mode	
        ADC1->CR1 |= ADC_CR1_SCAN;				// 1: Scan mode enable
    }
} 

void ADC_sequence(int length, int *seq){

    ADC1->SQR1 &= ~ADC_SQR1_L; 					  // reset length of conversions in the regular channel 	
    ADC1->SQR1 |= (length - 1) << ADC_SQR1_L_Pos; // conversions in the regular channel conversion sequence

    for(int i = 0; i<length; i++){
        if (i<6){
            ADC1->SQR3 &= ~(0x1F << i*5);		  // SQn clear bits
            ADC1->SQR3 |= seq[i] << i*5;		  // Choose the channel to convert sequence
        }
        else if (i < 12){
            ADC1->SQR2 &= ~(0x1F << (i-6)*5);	  // SQn clear bits
            ADC1->SQR2 |= seq[i] << (i-6)*5;	  // Choose the channel to convert sequence
        }
        else{
            ADC1->SQR1 &= ~(0x1F << (i-12)*5);	  // SQn clear bits
            ADC1->SQR1 |= seq[i] << (i-12)*5;	  // Choose the channel to convert sequence
        }
    }
}

void ADC_start(void){
    // Enable ADON, SW Trigger-----------------------------------------------------------------------------
    ADC1->CR2 |= ADC_CR2_ADON;
    ADC1->CR2 |= ADC_CR2_SWSTART;
}
	// For read the signal(interrupt)
uint32_t is_ADC_EOC(void){
    return (ADC1->SR & ADC_SR_EOC) == ADC_SR_EOC;
}

uint32_t is_ADC_OVR(void){
    return (ADC1->SR & ADC_SR_OVR) == ADC_SR_OVR;
}

void clear_ADC_OVR(void){
    ADC1->SR &= ~ADC_SR_OVR;
}

uint32_t ADC_read(){
    return ADC1->DR;
}

uint32_t ADC_pinmap(GPIO_TypeDef *Port, int Pin){
    if(Port == GPIOA){
        if(Pin == 0) 			return 0;
        else if(Pin == 1) return 1;
        else if(Pin == 4) return 4;
        else if(Pin == 5) return 5;
        else if(Pin == 6) return 6;
        else if(Pin == 7) return 7;
        else 							while(1);
    }
    else if(Port == GPIOB){
        if(Pin == 0) 			return 8;
        else if(Pin == 1)	return 9;
        else 							while(1);
    }
    else if(Port == GPIOC){
        if(Pin == 0)			return 10;
        else if(Pin == 1)	return 11;
        else if(Pin == 2)	return 12;
        else if(Pin == 3)	return 13;
        else if(Pin == 4)	return 14;
        else if(Pin == 5)	return 15;
        else							while(1);
    }
}
```

##          



### Problem 2: IR Reflective Sensor (TCRT 5000)

It consists of an infrared LED and a phototransistor (that is sensitive to light). This sensor has a coating on it to filter out light that is not within the infrared spectrum to help reduce the chance of environmental interference - this is what gives the input side of the TCRT5000 its black color. 

The TCRT5000 itself works by transmitting infrared light from the LED and registering any reflected light on its phototransistor this alters the flow of current between its emitter and collector according to the level of light it receives.

It presents us with four pins. VCC, GND, D0 and A0. We supply a working voltage between 3.3v and 5v through the VCC and Ground pins. We receive our sensor data though either of the two remaining pins.

The analogue pin A0 provides a continuous reading in the form of varying voltage, the higher the voltage the more infrared light is being received. One major drawback of this sensor is it can be easily affected by environmental conditions. Any other source of infrared light such as sunlight or house lights are also detected by the sensor and can interfere with the readings.

![img](https://user-images.githubusercontent.com/91526930/200573269-26a4ffa1-c789-4545-9dc5-079294265d4d.png)

**The TCRT 5000 IR Reflective Sensor Specifications:**

- Detection Range: 1mm to 8 mm
- Focus Range: 2.5 mm
- Driving Ability: > 15 mA
- Working Voltage: 3.3 V to 5 V
- Output: digital switch outputs (0 and 1)
- PCB Size: 3.2 x 1.4 cm

#### Procedure



#### Configuration

![image](https://user-images.githubusercontent.com/113574284/204379497-7cd00eaa-5fe0-45a9-af1f-5cc746cd79f5.png)



#### Circuit Diagram

![image](https://user-images.githubusercontent.com/113574284/200164596-ca685f8d-cbe6-48fb-a89a-c5021c11de14.png)

#### Line Tracing

![image](https://user-images.githubusercontent.com/113574284/204384951-9d526938-1e45-4f81-acf7-5ed52bd7968d.png)

#### Discussion

1. How would you change the code if you need to use 3 Analog sensors?.

   I think it would be better to attach sensors in the middle of the front and on both sides of the car body. I think it is good to use a method of auxiliary steering by the sensors on both sides in case that the middle sensor does not recognize the black color. It is expected that this way of programming will be less burdensome for the system.

2. Which registers should be modified if you need to use Injection Groups instead of regular groups for 2 analog sensors?

   The regular group modifies the SQRx register, but the injected group modifies the JSQx register.




#### Description with Code

* LAB_Timer_InputCaputre_Ultrasonic.c  description

```c++
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
		printf("IR1 = %d \r\n",IR1);
		printf("IR2 = %d \r\n",IR2);
		printf("%f [cm]\r\n", distance_present);
		printf("\r\n");
		
		if (distance_present - distance_past > 5000);					// collision prevention
		
        //dcDirPin: HIGH, PWM_duty: 0 >> Full speed
        //dcDirPin: LOW, PWM_duty: 1 >> Full speed
		else if (distance_present < 15){
			printf("STOP\r\n");
			PWM_duty(&dcPwm[A], 1);
			PWM_duty(&dcPwm[B], 1);
			GPIO_write(dcDirPin[A].port, dcDirPin[A].pin, HIGH);
			GPIO_write(dcDirPin[B].port, dcDirPin[B].pin, HIGH);
		}
		else if (IR1 > 1000) {
			printf("GO LEFT\r\n");
			GPIO_write(dcDirPin[A].port, dcDirPin[A].pin, LOW);
			GPIO_write(dcDirPin[B].port, dcDirPin[B].pin, LOW);
			PWM_duty(&dcPwm[A], 0.2);
			PWM_duty(&dcPwm[B], 0.4);
		}
		else if (IR2 > 1000) {
			printf("GO RIGHT\r\n");
			GPIO_write(dcDirPin[A].port, dcDirPin[A].pin, LOW);
			GPIO_write(dcDirPin[B].port, dcDirPin[B].pin, LOW);
			PWM_duty(&dcPwm[A], 0.37);
			PWM_duty(&dcPwm[B], 0.2);
		}
		else{
			printf("GO Front\r\n");
			GPIO_write(dcDirPin[A].port, dcDirPin[A].pin, LOW);
			GPIO_write(dcDirPin[B].port, dcDirPin[B].pin, LOW);
			PWM_duty(&dcPwm[A], 0.43);
			PWM_duty(&dcPwm[B], 0.4);
		}
		
		printf("\r\n");
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
	
	GPIO_write(dcDirPin[A].port, dcDirPin[A].pin, LOW);
	GPIO_write(dcDirPin[B].port, dcDirPin[B].pin, LOW);
	
	for (int i = 0; i < 2; i++){
		GPIO_init(dcDirPin[i].port, dcDirPin[i].pin, OUTPUT);
		GPIO_pupd(dcDirPin[i].port, dcDirPin[i].pin, EC_PD);
		GPIO_otype(dcDirPin[i].port, dcDirPin[i].pin, Push_Pull);
		GPIO_ospeed(dcDirPin[i].port, dcDirPin[i].pin, High);
	}
	
	
	// input capture
	// PWM configuration ---------------------------------------------------------------------	
	PWM_t trig;							// PWM1 for trig
	PWM_init(&trig, GPIOA, 6);			// PA_6: Ultrasonic trig pulse
	PWM_period_us(&trig, 50000);    	// PWM of 50ms period. Use period_us()
	PWM_pulsewidth_us(&trig, 10);   	// PWM pulse width of 10us
	
// Input Capture configuration -----------------------------------------------------------------------	
	IC_t echo;							// Input Capture for echo
	ICAP_init(&echo, GPIOB, 10);    	// PB10 as input caputre
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
    if(is_UIF(TIM2)){               // Update interrupt
		ovf_cnt++;					// overflow count
		clear_UIF(TIM2);  			// clear update interrupt flag
	}
	if(is_CCIF(TIM2, 3)){ 			// TIM2_Ch3 (IC3) Capture Flag. Rising Edge Detect
		time1 = TIM2->CCR3;			// Capture TimeStart
		clear_CCIF(TIM2, 3);        // clear capture/compare interrupt flag 
	}								                      
	else if(is_CCIF(TIM2, 4)){ 		// TIM2_Ch3 (IC4) Capture Flag. Falling Edge Detect
		time2 = TIM2->CCR4;			// Capture TimeEnd
		timeInterval = (time2- time1 + ovf_cnt * (TIM2->ARR +1)) *10.0 / 1000.0; 					
        // Total time of echo pulse (10us * counter pulses -> [msec] unit)
		ovf_cnt = 0;                // overflow reset
		clear_CCIF(TIM2, 4);		// clear capture/compare interrupt flag 
	}
}
```



#### Results

![image](https://user-images.githubusercontent.com/113574284/204387000-f7b3595e-a346-4fad-9215-9b809dbbf691.jpg)

[Youtube Link](https://youtube.com/shorts/grO_D8HMQTM?feature=share)



##          



### Reference

[LAB: ADC - IR sensor](https://ykkim.gitbook.io/ec/course/lab/lab-adc-irsensor)

Class materials in Embedded Controller by Prof. Kim

RM0383 Reference manual

[TCRT5000 Infrared Reflective Sensor - How It Works and Example Circuit With Code](https://www.instructables.com/TCRT5000-Infrared-Reflective-Sensor-How-It-Works-a/)

##          



### Troubleshooting

It is thought that a lot of trial and errors will be required for complete control. Even if the same PWM value was given, there were many problems such as the body did not go straight or the amount of rotation to the left and right was different when giving rotation.
In addition, the range of the IR Reflective Sensor is so narrow that it seems necessary to significantly slow the speed of the vehicle body or thicken the black line on the floor to control the direction.

There were difficulties in the process of distributing limited timers. Initially, it tried to share the input capture of the ultrasonic sensor and the timer of the dcmotor. I thought it would be good to adjust the PSC and ARR values properly, but it was not easy. The existing body has a problem in that the motor does not operate properly when the period of PWM is long. After exchanging the body, I changed the cycle and operated it, and it was confirmed that it was working properly. Accordingly, the PWM signal of dcmotor and the trigger PWM signal of ultrasonic were appropriately distributed using different channels of the same timer.
