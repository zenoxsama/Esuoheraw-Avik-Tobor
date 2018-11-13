/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart6;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

// Internal functions
uint8_t	GET_MOTOR_SPEED(uint8_t type);
double PROCESS_PID_CONTROL(signed int PID_error);
void TEST_KIVA_MOTION(void);
void CONTROL_KIVA_MOTION(uint8_t command, double PID_value);
void BACKWARD_KIVA_ROBOT(uint16_t duty_cycle_L, uint16_t duty_cycle_R, uint8_t time_Delay);
void FORWARD_KIVA_ROBOT(uint16_t duty_cycle_L, uint16_t duty_cycle_R);
void TURNLEFT_KIVA_ROBOT(uint16_t duty_cycle_L, uint16_t duty_cycle_R);
void TURNRIGHT_KIVA_ROBOT(uint16_t duty_cycle_L, uint16_t duty_cycle_R);
void STOP_KIVA_ROBOT(void);
void SET_ANGLE_KIVA_ROBOT(signed int angle);
void PROCESS_UART_PI3(void);
void GOAHEAD_KIVA_ROBOT(void);
void HAL_Delay_test_goahead(__IO uint32_t Delay);
void HAL_Delay_test_angle(__IO uint32_t Delay, signed int angle);

// Internal variables
//2250 pudges for 90*
char * command = "";
char * ACK = "OK";
uint8_t receive[3];

uint16_t MOTOR_STEP = 839;

//PID setting for following line
#define Kp 10
#define Kd 5
#define Ki 0.01
//PID setting for encoder control
#define Kpe 15
#define Kde 2.5 //2.2
#define Kie 0.05 //1

//PID setting for rotating angle
#define Kpa 10
#define Kda 2
#define Kia 0

#define MOTOR_SPEED_MAX 8400
#define MOTOR_SPEED_MIN 1
#define MOTOR_BASE_SPEED 2300
#define ROTAGE_BASE_ANGLE 2350
#define MOTOR_SPEED_NOR 2000

uint16_t PWM_DUTY_R1 = 0, PWM_DUTY_L1 = 0;
uint16_t PWM_DUTY_R2 = 0, PWM_DUTY_L2 = 0;
	
uint8_t MOTOR_SPEED_R = 0, MOTOR_SPEED_L = 0;
volatile uint16_t countR = 0, countL = 0;

double error=0, P=0, I=0, D=0; 
double speed_PID=0;
double previous_error=0, previous_I=0;

double error_Encoder=0, Pe=0, Ie=0, De=0; 
double previous_error_Encoder =0, previous_Ie=0;	

double error_Angle =0, Pa =0, Ia =0, Da =0; 
double previous_error_Angle =0, previous_Ia=0;

uint16_t tick =0;

enum KIVA_MOTION_STATE
{
	FORWARD = 0,
	BACKWARD,
	TURNLEFT,
	TURNRIGHT,
	STOP,
	ANGLE,
	GOAHEAD
};

int main(void)
{

  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_USART6_UART_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
	
		//Enable Tx UART1
  __HAL_UART_ENABLE_IT(&huart2,UART_IT_RXNE); // Enable TX
  __HAL_UART_ENABLE_IT(&huart2,UART_IT_TC);// Enable RX
	HAL_UART_Receive_IT(&huart2,(uint8_t *) receive,3);
	//Start PWM
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1); // R1 Right Motor
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2); // R2 Right Motor
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3); // L1 Left Motor
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4); // L2 Left Motor	
	//Enable Counter encoder
//	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
//	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	//SET_ANGLE_KIVA_ROBOT(90);

  while (1)
		{
//			countR = TIM2->CNT;
//			countL = TIM1->CNT;
//			if(receive[2] >= 128) error = (- (receive[2] - 128));
//			else error =  (127 - receive[2]);
//			
//			speed_PID = PROCESS_PID_CONTROL(error);
//			CONTROL_KIVA_MOTION(receive[0], 0);
			TIM1->CNT = 0;
			TIM2->CNT = 0;
			HAL_Delay_test_goahead(5000);
		//	HAL_TIM_Encoder_Stop(&htim1, TIM_CHANNEL_ALL);
		//	HAL_TIM_Encoder_Stop(&htim2, TIM_CHANNEL_ALL);
			
			STOP_KIVA_ROBOT();
			HAL_Delay(1000);
				TIM1->CNT = 0;
				TIM2->CNT = 0;
			//	SET_ANGLE_KIVA_ROBOT(50);
			
//			tick = HAL_GetTick();
			HAL_Delay_test_angle(2000, 60);
			STOP_KIVA_ROBOT();
			HAL_Delay(1000);			
	//		HAL_TIM_Encoder_Stop(&htim1, TIM_CHANNEL_ALL);
		//	HAL_TIM_Encoder_Stop(&htim2, TIM_CHANNEL_ALL);
		//	STOP_KIVA_ROBOT();

		}

}
//Handle RX Interrupt
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){ // day la chuong trinh ngat
	
  if(huart->Instance==huart2.Instance)
		{
			HAL_UART_Receive_IT(&huart2,(uint8_t *) receive,3);
			HAL_UART_Transmit_IT(&huart2,(uint8_t *)ACK,2);
		}	

}
//Convert error size from Pi 3
unsigned int CONVERT_ERROR(uint8_t error)
{
	if(error >= 128) return (- (error - 128));
	else return (127 - error);
}
void TEST_KIVA_MOTION(void)
{
		FORWARD_KIVA_ROBOT(MOTOR_SPEED_MAX-100, MOTOR_SPEED_MAX-100);
	
		//countR = TIM2->CNT;
		
		//countL = TIM1->CNT;
//		BACKWARD_KIVA_ROBOT((MOTOR_SPEED_MAX/1.5)-1, (MOTOR_SPEED_MAX/1.5)-1, 10);
//		
//		FORWARD_KIVA_ROBOT(MOTOR_SPEED_MAX-1, MOTOR_SPEED_MAX-1, 3);
//		
//		TURNRIGHT_KIVA_ROBOT((MOTOR_SPEED_MAX/1.5)-1,MOTOR_SPEED_MAX-1,5);
//		
//		FORWARD_KIVA_ROBOT(MOTOR_SPEED_MAX-1, MOTOR_SPEED_MAX-1, 3);
//		TURNLEFT_KIVA_ROBOT(MOTOR_SPEED_MAX-1,(MOTOR_SPEED_MAX/1.5)-1,5);
//		
//		FORWARD_KIVA_ROBOT(MOTOR_SPEED_MAX-1, MOTOR_SPEED_MAX-1, 5);
		STOP_KIVA_ROBOT();
		HAL_Delay(5000);
}

//delay
void HAL_Delay_test_goahead(__IO uint32_t Delay)
{
  uint32_t tickstart = HAL_GetTick();
  uint32_t wait = Delay;
  
  /* Add a period to guarantee minimum wait */
  if (wait < HAL_MAX_DELAY)
  {
     wait++;
  }
  
  while((HAL_GetTick() - tickstart) < wait)
  {
		GOAHEAD_KIVA_ROBOT();
  }
}
void HAL_Delay_test_angle(__IO uint32_t Delay, signed int angle)
{
  uint32_t tickstart = HAL_GetTick();
  uint32_t wait = Delay;
  
  /* Add a period to guarantee minimum wait */
  if (wait < HAL_MAX_DELAY)
  {
     wait++;
  }
  
  while((HAL_GetTick() - tickstart) < wait)
  {
		SET_ANGLE_KIVA_ROBOT(angle);
  }
}
//ANGLE
void SET_ANGLE_KIVA_ROBOT(signed int angle)
{
		HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
		HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	//Read encoder feedback
		countL = TIM1->CNT;
		countR = TIM2->CNT;
	//Set motor speed
	//Adjust encoder pulse since A rotates B 90* 
		if(angle > 5) // Turn Right
			{
				countR = 0xFFFF - countR;
			}
		else if(angle < - 5)// Turn Left
			{
				countL = 0xFFFF - countL;
			}
		else
			{		

			}//Forward
		error_Angle = countL - countR;
		Pa = error_Angle;
		Ia = Ia + error_Angle;
		Da = error_Angle - previous_error_Angle;		
		HAL_Delay(10);
		double adjust_PID = Kpa*Pa + Kia*Ia + Kda*Da;
		previous_error_Angle = error_Angle;	
		//Control motors and speed
		if(angle > 5) // Turn Right
			{
				if (1500 > error_Angle > 0)
				{
					PWM_DUTY_R1 = MOTOR_BASE_SPEED + adjust_PID;
					PWM_DUTY_L1 = MOTOR_BASE_SPEED - adjust_PID;
				}
				else if(-1500 < error_Angle < 0)
				{
					PWM_DUTY_R1 = MOTOR_BASE_SPEED - adjust_PID;
					PWM_DUTY_L1 = MOTOR_BASE_SPEED + adjust_PID;
				}
				else
				{
					PWM_DUTY_R1 = MOTOR_BASE_SPEED;
					PWM_DUTY_L1 = MOTOR_BASE_SPEED;
				}
				TURNRIGHT_KIVA_ROBOT(PWM_DUTY_L1, PWM_DUTY_R1);
			}
		else if(angle < - 5)// Turn Left
			{
				if (1500 > error_Angle > 0)
				{
					PWM_DUTY_R1 = MOTOR_BASE_SPEED + adjust_PID;
					PWM_DUTY_L1 = MOTOR_BASE_SPEED - adjust_PID;
				}
				else if(-1500 < error_Angle < 0)
				{
					PWM_DUTY_R1 = MOTOR_BASE_SPEED - adjust_PID;
					PWM_DUTY_L1 = MOTOR_BASE_SPEED + adjust_PID;
				}
				else
				{
					PWM_DUTY_R1 = MOTOR_BASE_SPEED;
					PWM_DUTY_L1 = MOTOR_BASE_SPEED;
				}
				TURNLEFT_KIVA_ROBOT(PWM_DUTY_L1, PWM_DUTY_R1);
			}
		else
			{
					PWM_DUTY_R1 = MOTOR_BASE_SPEED;
					PWM_DUTY_L1 = MOTOR_BASE_SPEED;
			}
			HAL_Delay(50);//Forward
//	if(angle > 0)
//		{
//			uint16_t count = (angle * ROTAGE_BASE_ANGLE)/90;
//			while(countL < count)
//				{
//					countL = TIM1->CNT;
//					countR = TIM2->CNT;
//					//TURNLEFT_KIVA_ROBOT(MOTOR_BASE_SPEED, MOTOR_BASE_SPEED, 0);
//					BACKWARD_KIVA_ROBOT(MOTOR_BASE_SPEED, MOTOR_BASE_SPEED,0);
//				}
//			HAL_TIM_Encoder_Stop(&htim1, TIM_CHANNEL_ALL);
//			HAL_TIM_Encoder_Stop(&htim2, TIM_CHANNEL_ALL);
//			STOP_KIVA_ROBOT();
//		}
//	else
//		{
//			
//		}
}
//PID processing
double PROCESS_PID_CONTROL (signed int PID_error)
{
		P = PID_error;
		I = I + PID_error;
    D = PID_error - previous_error;
    
    double PID_value = (Kp*P) + (Ki*I) + (Kd*D);
    
    previous_error = PID_error;
		return PID_value;
}
void CONTROL_KIVA_MOTION(uint8_t command, double PID_value)
{
	switch(command)
	{
		case FORWARD:
		case BACKWARD:
		case TURNRIGHT:
		case TURNLEFT:
			{
				PWM_DUTY_R1 = MOTOR_BASE_SPEED + PID_value;
				if(PWM_DUTY_R1 > (MOTOR_SPEED_MAX-1)) PWM_DUTY_R1 = MOTOR_SPEED_MAX-1;
				else if (PWM_DUTY_R1 < (MOTOR_SPEED_NOR)) PWM_DUTY_R1 = MOTOR_SPEED_NOR;
				
				PWM_DUTY_L1 = MOTOR_BASE_SPEED - PID_value;
				if(PWM_DUTY_L1 > (MOTOR_SPEED_MAX-1)) PWM_DUTY_L1 = MOTOR_SPEED_MAX-1;
				else if (PWM_DUTY_L1 < (MOTOR_SPEED_NOR)) PWM_DUTY_L1 = MOTOR_SPEED_NOR;
				
				FORWARD_KIVA_ROBOT(PWM_DUTY_R1, PWM_DUTY_L1);
				break;
			}
		case ANGLE:
			{
				
				break;
			}
		case GOAHEAD:
			{
				GOAHEAD_KIVA_ROBOT();
				//Stop encoder timer
				HAL_TIM_Encoder_Stop(&htim1, TIM_CHANNEL_ALL);
				HAL_TIM_Encoder_Stop(&htim2, TIM_CHANNEL_ALL);
				break;
			}
		case STOP:
			{
				STOP_KIVA_ROBOT();
				break;
			}
		default:
			{
				break;
			}
	}
			
}

void FORWARD_KIVA_ROBOT(uint16_t duty_cycle_L, uint16_t duty_cycle_R)
{
		if(duty_cycle_L > MOTOR_SPEED_MAX - 1) duty_cycle_L = MOTOR_SPEED_MAX - 1;
		if(duty_cycle_R > MOTOR_SPEED_MAX - 1) duty_cycle_R = MOTOR_SPEED_MAX - 1;
	
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, duty_cycle_L);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, MOTOR_SPEED_MIN - 1);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, duty_cycle_R);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, MOTOR_SPEED_MIN - 1);

}
void GOAHEAD_KIVA_ROBOT(void)
{
	//Enable encoder
		HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
		HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	//Read encoder feedback
		countL = TIM1->CNT;
		countR = TIM2->CNT;
	//PID control for encoder
		error_Encoder = countL - countR;
		
		Pe = error_Encoder;
		Ie = Ie + error_Encoder;
		De = error_Encoder - previous_error_Encoder;
	
		double adjust_PID = Kpe*Pe + Kie*Ie + Kde*De;
		previous_error_Encoder = error_Encoder;

		if(500 > error_Encoder > 0)
		{
			PWM_DUTY_L2 = MOTOR_BASE_SPEED  - adjust_PID;
			PWM_DUTY_R2 = MOTOR_BASE_SPEED + adjust_PID;
		}
		else if(-500 < error_Encoder < 0)
		{
			PWM_DUTY_L2 = PWM_DUTY_L2 + adjust_PID;
			PWM_DUTY_R2 = PWM_DUTY_R2 - adjust_PID;
		}
		else
		{
			//Keep going on
			PWM_DUTY_L2 = MOTOR_BASE_SPEED;
			PWM_DUTY_R2 = MOTOR_BASE_SPEED;
		}			
	//KIVA go ahead
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, PWM_DUTY_L2);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, MOTOR_SPEED_MIN - 1);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, PWM_DUTY_R2);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, MOTOR_SPEED_MIN - 1);
		HAL_Delay(100);
}
void BACKWARD_KIVA_ROBOT(uint16_t duty_cycle_L, uint16_t duty_cycle_R, uint8_t time_Delay)
{
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, MOTOR_SPEED_MIN - 1);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, duty_cycle_L);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, MOTOR_SPEED_MIN - 1);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, duty_cycle_R);
		
		HAL_Delay(time_Delay*1000);
}

void TURNLEFT_KIVA_ROBOT(uint16_t duty_cycle_L, uint16_t duty_cycle_R)
{
		if(duty_cycle_L > MOTOR_SPEED_MAX - 1) duty_cycle_L = MOTOR_SPEED_MAX - 1;
		if(duty_cycle_R > MOTOR_SPEED_MAX - 1) duty_cycle_R = MOTOR_SPEED_MAX - 1;
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, MOTOR_SPEED_MIN - 1);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, duty_cycle_L);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, duty_cycle_R);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, MOTOR_SPEED_MIN - 1);
}
void TURNRIGHT_KIVA_ROBOT(uint16_t duty_cycle_L, uint16_t duty_cycle_R)
{
		if(duty_cycle_L > MOTOR_SPEED_MAX - 1) duty_cycle_L = MOTOR_SPEED_MAX - 1;
		if(duty_cycle_R > MOTOR_SPEED_MAX - 1) duty_cycle_R = MOTOR_SPEED_MAX - 1;
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, duty_cycle_L);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, MOTOR_SPEED_MIN - 1);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, MOTOR_SPEED_MIN - 1);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, duty_cycle_R);
}

void STOP_KIVA_ROBOT(void)
{
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, MOTOR_SPEED_MIN - 1);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, MOTOR_SPEED_MIN - 1);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, MOTOR_SPEED_MIN - 1);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, MOTOR_SPEED_MIN - 1);
}
/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV4;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 0xFFFF;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFF;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 4200;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_SlaveConfigTypeDef sSlaveConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 10;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 8399;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_DISABLE;
  sSlaveConfig.InputTrigger = TIM_TS_ITR3;
  if (HAL_TIM_SlaveConfigSynchronization(&htim4, &sSlaveConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim4);

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART6 init function */
static void MX_USART6_UART_Init(void)
{

  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart6) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
