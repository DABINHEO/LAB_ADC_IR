/**
******************************************************************************
* @author  HeoDabin
******************************************************************************
*/


#include "stm32f4xx.h"
#include "stm32f411xe.h"
#include "ecGPIO.h"

void GPIO_init(GPIO_TypeDef *Port, int pin, int mode){     
	// mode  : Input(0), Output(1), AlterFunc(2), Analog(3)   
	if (Port == GPIOA)
		RCC_GPIOA_enable();
	if (Port == GPIOC)
		RCC_GPIOC_enable();
	
	if (Port == GPIOB)
		RCC_GPIOB_enable();

	GPIO_mode(Port, pin, mode);
}

// GPIO Write          : LOW(0) HIGH(1)
void GPIO_write(GPIO_TypeDef *Port, int pin, int Output){
	 Port->ODR &= ~(1UL<<pin);
	 Port->ODR |= Output<<pin;
}


// GPIO Mode          : Input(00), Output(01), AlterFunc(10), Analog(11)
void GPIO_mode(GPIO_TypeDef *Port, int pin, int mode){
   Port->MODER &= ~(3UL<<(2*pin));     
   Port->MODER |= mode<<(2*pin);    
}


// GPIO Speed          : Low speed (00), Medium speed (01), Fast speed (10), High speed (11)
void GPIO_ospeed(GPIO_TypeDef *Port, int pin, int speed){
	Port->OSPEEDR &= ~(3UL<<(2*pin));	
	Port->OSPEEDR |= speed<<(2*pin);
}

// GPIO Output Type: Output push-pull (0, reset), Output open drain (1)
void GPIO_otype(GPIO_TypeDef *Port, int pin, int type){
	Port->OTYPER &= ~(1UL<<pin);
	Port->OTYPER |= type<<pin;
}

// GPIO Push-Pull    : No pull-up, pull-down (00), Pull-up (01), Pull-down (10), Reserved (11)
void GPIO_pupd(GPIO_TypeDef *Port, int pin, int pupd){
	Port->PUPDR &= ~(3UL<<(2*pin));	
	Port->PUPDR |=	pupd<<(2*BUTTON_PIN);
}

int GPIO_read(GPIO_TypeDef *Port, int pin){
	int btVal;
	btVal=(Port->IDR) & (1UL<<pin);
	return btVal;
}


void LED_init(void){
	GPIO_init(GPIOA, 5, OUTPUT);
	GPIO_init(GPIOA, 6, OUTPUT);
	GPIO_init(GPIOA, 7, OUTPUT);
	GPIO_init(GPIOB, 6, OUTPUT);
	
	GPIO_pupd(GPIOA, 5, EC_PU);
	GPIO_pupd(GPIOA, 6, EC_PU);
	GPIO_pupd(GPIOA, 7, EC_PU);
	GPIO_pupd(GPIOB, 6, EC_PU);
	
	GPIO_ospeed(GPIOA, 5, Medium);
	GPIO_ospeed(GPIOA, 6, Medium);
	GPIO_ospeed(GPIOA, 7, Medium);
	GPIO_ospeed(GPIOB, 6, Medium);
	
	GPIO_otype(GPIOA, 5, 0);
	GPIO_otype(GPIOA, 6, 0);
	GPIO_otype(GPIOA, 7, 0);
	GPIO_otype(GPIOB, 6, 0);
}

void sevensegment_init(){
	GPIO_init(GPIOA, 5, OUTPUT);
	GPIO_init(GPIOA, 6, OUTPUT);
	GPIO_init(GPIOA, 7, OUTPUT);
	GPIO_init(GPIOB, 6, OUTPUT);
	GPIO_init(GPIOC, 7, OUTPUT);
	GPIO_init(GPIOA, 9, OUTPUT);
	GPIO_init(GPIOA, 8, OUTPUT);
	GPIO_init(GPIOB, 10, OUTPUT);
	
	GPIO_otype(GPIOA, 5, Push_Pull);
	GPIO_otype(GPIOA, 6, Push_Pull);
	GPIO_otype(GPIOA, 7, Push_Pull);
	GPIO_otype(GPIOB, 6, Push_Pull);
	GPIO_otype(GPIOC, 7, Push_Pull);
	GPIO_otype(GPIOA, 9, Push_Pull);
	GPIO_otype(GPIOA, 8, Push_Pull);
	GPIO_otype(GPIOB, 10, Push_Pull);
	
	GPIO_pupd(GPIOA, 5, EC_NUD);
	GPIO_pupd(GPIOA, 6, EC_NUD);
	GPIO_pupd(GPIOA, 7, EC_NUD);
	GPIO_pupd(GPIOB, 6, EC_NUD);
	GPIO_pupd(GPIOC, 7, EC_NUD);
	GPIO_pupd(GPIOA, 9, EC_NUD);
	GPIO_pupd(GPIOA, 8, EC_NUD);
	GPIO_pupd(GPIOB, 10, EC_NUD);
	
	GPIO_ospeed(GPIOA, 5, Medium);
	GPIO_ospeed(GPIOA, 6, Medium);
	GPIO_ospeed(GPIOA, 7, Medium);
	GPIO_ospeed(GPIOB, 6, Medium);	
	GPIO_ospeed(GPIOC, 7, Medium);
	GPIO_ospeed(GPIOA, 9, Medium);
	GPIO_ospeed(GPIOA, 8, Medium);
	GPIO_ospeed(GPIOB, 10, Medium);
}

void led_decode(int num){
	int number[4][4]={
		{1,0,0,0},          
		{0,1,0,0},          
		{0,0,1,0},          
		{0,0,0,1}
	};
	GPIO_write(GPIOA, 5, number[num][0]); 
	GPIO_write(GPIOA, 6, number[num][1]); 
	GPIO_write(GPIOA, 7, number[num][2]); 
	GPIO_write(GPIOB, 6, number[num][3]); 
}

void sevensegment_decode(int num){
	int number[11][8]={
		{0,0,0,1,1,0,0,0},          //zero
		{0,1,1,1,1,0,1,1},          //one
		{0,0,1,1,0,1,0,0},          //two
		{1,0,0,1,0,1,0,0},          //three
		{1,1,0,1,0,0,1,0},          //four
		{1,0,0,1,0,0,0,1},          //five
		{0,0,0,1,0,0,1,1},          //six
		{1,1,0,1,1,1,0,0},          //seven
		{0,0,0,1,0,0,0,0},          //eight
		{1,1,0,1,0,0,0,0},          //nine
		{1,1,1,0,1,1,1,1}          //dot
	};
	GPIO_write(GPIOA, 5, number[num][0]); //e
	GPIO_write(GPIOA, 6, number[num][1]); //d
	GPIO_write(GPIOA, 7, number[num][2]); //c
	GPIO_write(GPIOB, 6, number[num][3]); //DP
	GPIO_write(GPIOC, 7, number[num][4]); //g
	GPIO_write(GPIOA, 9, number[num][5]); //f
	GPIO_write(GPIOA, 8, number[num][6]); //a
	GPIO_write(GPIOB, 10, number[num][7]); //b
}

void LED_toggle(){
	
	GPIOA->ODR ^= 1 << LED_PIN;
}

void LED_toggle2(GPIO_TypeDef *Port, int pin){
	
	Port->ODR ^= 1 <<pin;
	
}

void LED_init_test(void){
	GPIO_init(GPIOA, 5, OUTPUT);
	GPIO_init(GPIOA, 0, OUTPUT);
	GPIO_init(GPIOA, 1, OUTPUT);
	GPIO_init(GPIOB, 0, OUTPUT);
	
	GPIO_pupd(GPIOA, 5, EC_NUD);
	GPIO_pupd(GPIOA, 0, EC_NUD);
	GPIO_pupd(GPIOA, 1, EC_NUD);
	GPIO_pupd(GPIOB, 0, EC_NUD);
	
	GPIO_ospeed(GPIOA, 5, Medium);
	GPIO_ospeed(GPIOA, 0, Medium);
	GPIO_ospeed(GPIOA, 1, Medium);
	GPIO_ospeed(GPIOB, 0, Medium);
	
	GPIO_otype(GPIOA, 5, 0);
	GPIO_otype(GPIOA, 0, 0);
	GPIO_otype(GPIOA, 1, 0);
	GPIO_otype(GPIOB, 0, 0);
}

void sevensegment_init_test(){
	GPIO_init(GPIOB, 9, OUTPUT);	
	GPIO_init(GPIOA, 6, OUTPUT);
	GPIO_init(GPIOA, 7, OUTPUT);
	GPIO_init(GPIOB, 6, OUTPUT);
	GPIO_init(GPIOC, 7, OUTPUT);
	GPIO_init(GPIOA, 9, OUTPUT);
	GPIO_init(GPIOA, 8, OUTPUT);
	GPIO_init(GPIOB, 10, OUTPUT);
	
	GPIO_otype(GPIOB, 9, Push_Pull);
	GPIO_otype(GPIOA, 6, Push_Pull);
	GPIO_otype(GPIOA, 7, Push_Pull);
	GPIO_otype(GPIOB, 6, Push_Pull);
	GPIO_otype(GPIOC, 7, Push_Pull);
	GPIO_otype(GPIOA, 9, Push_Pull);
	GPIO_otype(GPIOA, 8, Push_Pull);
	GPIO_otype(GPIOB, 10, Push_Pull);
	
	GPIO_pupd(GPIOB, 9, EC_NUD);
	GPIO_pupd(GPIOA, 6, EC_NUD);
	GPIO_pupd(GPIOA, 7, EC_NUD);
	GPIO_pupd(GPIOB, 6, EC_NUD);
	GPIO_pupd(GPIOC, 7, EC_NUD);
	GPIO_pupd(GPIOA, 9, EC_NUD);
	GPIO_pupd(GPIOA, 8, EC_NUD);
	GPIO_pupd(GPIOB, 10, EC_NUD);
	
	GPIO_ospeed(GPIOB, 9, Medium);
	GPIO_ospeed(GPIOA, 6, Medium);
	GPIO_ospeed(GPIOA, 7, Medium);
	GPIO_ospeed(GPIOB, 6, Medium);	
	GPIO_ospeed(GPIOC, 7, Medium);
	GPIO_ospeed(GPIOA, 9, Medium);
	GPIO_ospeed(GPIOA, 8, Medium);
	GPIO_ospeed(GPIOB, 10, Medium);
	
	GPIO_write(GPIOB, 9, 1); //e
	GPIO_write(GPIOA, 6, 1); //d
	GPIO_write(GPIOA, 7, 1); //c
	GPIO_write(GPIOB, 6, 1); //dp
	GPIO_write(GPIOC, 7, 1); //g
	GPIO_write(GPIOA, 9, 1); //f
	GPIO_write(GPIOA, 8, 1); //a
	GPIO_write(GPIOB, 10, 1); //b
}

void sevensegment_decode_test(int num){
	int number[11][8]={
		{0,0,0,1,1,0,0,0},          //zero
		{0,1,1,1,1,0,1,1},          //one
		{0,0,1,1,0,1,0,0},          //two
		{1,0,0,1,0,1,0,0},          //three
		{1,1,0,1,0,0,1,0},          //four
		{1,0,0,1,0,0,0,1},          //five
		{0,0,0,1,0,0,1,1},          //six
		{1,1,0,1,1,1,0,0},          //seven
		{0,0,0,1,0,0,0,0},          //eight
		{1,0,0,1,0,0,0,0},          //nine
		{1,1,1,0,1,1,1,1}          //dot
	};
	GPIO_write(GPIOB, 9, number[num][0]); //e
	GPIO_write(GPIOA, 6, number[num][1]); //d
	GPIO_write(GPIOA, 7, number[num][2]); //c
	GPIO_write(GPIOB, 6, number[num][3]); //dp
	GPIO_write(GPIOC, 7, number[num][4]); //g
	GPIO_write(GPIOA, 9, number[num][5]); //f
	GPIO_write(GPIOA, 8, number[num][6]); //a
	GPIO_write(GPIOB, 10, number[num][7]); //b
}

