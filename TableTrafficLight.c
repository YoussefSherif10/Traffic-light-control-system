// ***** 0. Documentation Section *****
// TableTrafficLight.c for Lab 10
// Runs on LM4F120/TM4C123
// Index implementation of a Moore finite state machine to operate a traffic light.  

// east/west red light connected to PB5
// east/west yellow light connected to PB4
// east/west green light connected to PB3
// north/south facing red light connected to PB2
// north/south facing yellow light connected to PB1
// north/south facing green light connected to PB0
// pedestrian detector connected to PE2 (1=pedestrian present)
// north/south car detector connected to PE1 (1=car present)
// east/west car detector connected to PE0 (1=car present)
// "walk" light connected to PF3 (built-in green LED)
// "don't walk" light connected to PF1 (built-in red LED)

// ***** 1. Pre-processor Directives Section *****
#include "TExaS.h"
#include "tm4c123gh6pm.h"

// ***** 2. Global Declarations Section *****
#define goS   0x00
#define waitS 0x01
#define goW   0x02
#define waitW 0x03
#define walk	0x04
#define low1 	0x05
#define high1 0x06
#define low2 	0x07
#define high2 0x08
#define low3  0x09

#define RED   1
#define GREEN	4
#define OFF		0

typedef struct state
{
	unsigned long outB;
	unsigned long outE;
	unsigned long time;
	unsigned long next[8];
}stype;

// FUNCTION PROTOTYPES: Each subroutine defined
void DisableInterrupts(void); // Disable interrupts
void EnableInterrupts(void);  // Enable interrupts

void portF_Init(void);
void portE_B_Init(void);
void SysTick_Init(void);
void SysTick_Wait(unsigned long);
void SysTick_Wait10ms(unsigned long delay);
// ***** 3. Subroutines Section *****




stype FSM[15] =
{
	//0	goS
	{0x21, RED, 100, {goS, waitS, goS, waitS, waitS, waitS, waitS, waitS}},
	//1 waitS
	{0x22, RED, 50, {goW, goW, goW, goW, walk, goW, walk, goW}},
	//2 goW
	{0x0C, RED, 100, {goW, goW, waitW, waitW, waitW, waitW, waitW, waitW}},
	//3 waitW
	{0x14, RED, 50, {goS, goS, goS, goS, walk, walk, goS, walk}},
	//4 walk
	{0x24, GREEN, 100, {walk, low1, low1, low1, walk, low1, low1, low1}},
	//5 low1
	{0x24, OFF, 10, {high1, high1, high1, high1, high1, high1, high1, high1}},
	//6	high1
	{0x24, RED, 10, {low2, low2, low2, low2, low2, low2, low2, low2}},
	//7	low2
	{0x24, OFF, 10,{high2, high2, high2, high2, high2, high2, high2, high2}},	
	//8 high2
	{0x24, RED, 10,{low3, low3, low3, low3, low3, low3, low3, low3}},
	//9 low3
  {0x24, OFF, 10,{goS, goW, goS, goS, walk, goW, goS, goS}}
};

unsigned long input;
unsigned long s;
int main(void)
{ 
  
	SysTick_Init();
	portF_Init();
	portE_B_Init();
  
  EnableInterrupts();
  s = goS;
	while(1)
	{
    GPIO_PORTB_DATA_R = FSM[s].outB;
		GPIO_PORTF_DATA_R = (FSM[s].outE << 1);
		SysTick_Wait10ms(FSM[s].time);
		input = GPIO_PORTE_DATA_R & 0x07;
		s = FSM[s].next[input];
  }
}


void portF_Init(void)
{ 
	volatile unsigned long delay;
  SYSCTL_RCGC2_R |= 0x00000020;     // 1) activate clock for Port F
  delay = SYSCTL_RCGC2_R;           // allow time for clock to start
  GPIO_PORTF_LOCK_R = 0x4C4F434B;   // 2) unlock GPIO Port F
  GPIO_PORTF_CR_R = 0x1F;           // allow changes to PF4-0
  // only PF0 needs to be unlocked, other bits can't be locked
  GPIO_PORTF_AMSEL_R = 0x00;        // 3) disable analog on PF
  GPIO_PORTF_PCTL_R = 0x00000000;   // 4) PCTL GPIO on PF4-0
  GPIO_PORTF_DIR_R = 0x0A;          // 5) PF3-1 input
  GPIO_PORTF_AFSEL_R = 0x00;        // 6) disable alt funct on PF7-0
  GPIO_PORTF_DEN_R = 0x0A;          // 7) enable digital I/O on PF4-0
}

void portE_B_Init(void)
{
	volatile unsigned long delay;
	SYSCTL_RCGC2_R |= 0x12;      // 1) B E
  delay = SYSCTL_RCGC2_R;      // 2) no need to unlock
  GPIO_PORTE_AMSEL_R &= ~0x03; // 3) disable analog function on PE1-0
  GPIO_PORTE_PCTL_R &= ~0x000000FF; // 4) enable regular GPIO
  GPIO_PORTE_DIR_R &= ~0x07;   // 5) inputs on PE1-0
  GPIO_PORTE_AFSEL_R &= ~0x07; // 6) regular function on PE1-0
  GPIO_PORTE_DEN_R |= 0x07;    // 7) enable digital on PE1-0
  GPIO_PORTB_AMSEL_R &= ~0x3F; // 3) disable analog function on PB5-0
  GPIO_PORTB_PCTL_R &= ~0x00FFFFFF; // 4) enable regular GPIO
  GPIO_PORTB_DIR_R |= 0x3F;    // 5) outputs on PB5-0
  GPIO_PORTB_AFSEL_R &= ~0x3F; // 6) regular function on PB5-0
  GPIO_PORTB_DEN_R |= 0x3F;    // 7) enable digital on PB5-0
}



void SysTick_Init(void){
  NVIC_ST_CTRL_R = 0;               // disable SysTick during setup
  NVIC_ST_CTRL_R = 0x00000005;      // enable SysTick with core clock
}
// The delay parameter is in units of the 80 MHz core clock. (12.5 ns)
void SysTick_Wait(unsigned long delay){
  NVIC_ST_RELOAD_R = delay-1;  // number of counts to wait
  NVIC_ST_CURRENT_R = 0;       // any value written to CURRENT clears
  while((NVIC_ST_CTRL_R&0x00010000)==0){ // wait for count flag
  }
}
// 10000us equals 10ms
void SysTick_Wait10ms(unsigned long delay){
  unsigned long i;
  for(i=0; i<delay; i++){
    SysTick_Wait(800000);  // wait 10ms
  }
}

