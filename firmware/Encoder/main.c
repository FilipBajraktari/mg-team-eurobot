#include <math.h>
#include "stm32f401xe.h"
#include "TIM_EncMode.h"
#include "Print.h"
#include "NucleoSetup.h"

#define delay() for(int i = 0; i < 1000000;i++)
#define PI 3.14159265

void TIM2_IRQHandler(void);
void TIM2Init(void);
double counts_to_radian(double count) {return (count * 2.0 * PI) / 8000.0;}

#define tim32bit TIM5
#define tim16bit TIM3

static const double sample_time = 0.01;
static const uint32_t sample_frequency = 100;

//Robot Constants
static const double Wheel_Distance = 30.0;
static const double Wheel_radius = 8.0;
static double RobotRot = 0.0;
static double RobotX = 0.0;
static double RobotY = 0.0;

// Odometry
static double A = 0.98393051;
static double B = 1.0;
static double C = -1.30162834;
static double D = 81.0;
double right_wheel_x = 0;
double right_wheel_omega = 0;
double left_wheel_x = 0;
double left_wheel_omega = 0;


int main()
{
	TIM2Init();
	
	nucleoLedInit();
	rccSetup();
	
	if (encoderInit32bit(tim32bit, true) == TIM_EncMode_OK &&
			encoderInit16bit(tim16bit, true) == TIM_EncMode_OK &&
			printInit(USART2, 115200) == Print_OK) GPIOA->ODR |= GPIO_ODR_OD5;
	
	while(1)
	{	
		delay();
		
	}
}


//*****************< INTERRUPT SETUP AND HANDLER >***********************

void TIM2Init(void)
{
	RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	
	TIM2->PSC = 84 - 1; // 84MHz / 84 = 1MHz = TIM2 CLOCK
	TIM2->ARR = 1000000 / sample_frequency - 1; 
	TIM2->CR1 = TIM_CR1_URS;
	TIM2->DIER = TIM_DIER_UIE;
	TIM2->EGR = TIM_EGR_UG; //TIMER 2 SETUP UPDATE INTERRUPT
	TIM2->CR1 = TIM_CR1_CEN; //TIMER 2 ENABLE
	
	NVIC_EnableIRQ(TIM2_IRQn); //TIMER 2 ENABLE INTERRUPT HANDLER
	
}

void TIM2_IRQHandler(void) //TIMER 2 INTERRUPT HANDLER
{
	TIM2->SR &= ~(TIM_SR_UIF); 
	
	double encoder_count, x, y;
	
	// Right Wheel
	encoder_count = counts_to_radian(encoderCount32bit(tim32bit));
	x = A*right_wheel_x + B*encoder_count;
	y = C*right_wheel_x + D*encoder_count;
	right_wheel_x = x;
	right_wheel_omega = y;
	
	// Left Wheel
	encoder_count = counts_to_radian(encoderCount16bit(tim16bit));
	x = A*left_wheel_x + B*encoder_count;
	y = C*left_wheel_x + D*encoder_count;
	left_wheel_x = x;
	left_wheel_omega = y;
	
	// Coordinates
	double vX = (left_wheel_omega+0)*Wheel_radius*0.5*cos(RobotRot);
	double vY = (left_wheel_omega+0)*Wheel_radius*0.5*sin(RobotRot);
	double vRot = (0-left_wheel_omega)*Wheel_radius/Wheel_Distance;
	RobotRot += vRot*sample_time;
	RobotX += vX*sample_time;
	RobotY += vY*sample_time;

	//print("%d\r\n", encoderCount32bit(tim32bit));
	//print("%f;%f;%f;%f\n", RobotX, RobotY, RobotRot,left_wheel_omega);
	print("%d    %d\n", encoderCount16bit(tim16bit), encoderCount32bit(tim32bit));
}

