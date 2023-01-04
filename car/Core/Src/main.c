/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"
#include "oled.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
struct gesture   //测得角度，角速度，x加速度，y加速度，轮位移
{
	float angle;  //角度
	float distance;
	float v1;  //左轮速度
	float v2;  //右轮速度
	float x1;  //左轮位移
	float x2;  //右轮位移
	float x;   //总位移
};
struct pidstruct  //pid参数
{
	float kp,ki,kd;
	float p,i,d;
	float thisde,lastde;
};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
uint8_t rx_buf1[1] = {0};
uint8_t rx_buf2[1] = {0};
uint8_t rx_buf3[1] = {0};
int mode=3;  //运动模式
int k=0;
int flag=0;

struct gesture gest;//当前姿态
struct gesture tgest;//目标姿态
struct gesture globalgest;//初始姿态
struct pidstruct pidposition;//位置环
struct pidstruct pidspeed1;//速度环左轮
struct pidstruct pidspeed2;//速度环右轮
struct pidstruct pidangle;//角度修正
struct pidstruct piddistance;//距离修正

float x_right=5600.0;
float x_left=4900.0;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void DataGet1(uint8_t data)//蓝牙
{
	if(data=='1') mode=1;
	if(data=='0') mode=0;
	if(data=='2') mode=2;
	if(data=='3') mode=3;
//	printf("%f,%f,%f\r\n",gest.v1,gest.v2,gest.angle);
	//HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
}
void DataGet2(uint8_t data)//陀螺仪
{
	//HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
}
void DataGet3(uint8_t data)//上位机通讯
{
	//HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
}
void Wheel(int num,int pwm)//轮子调速
{
	int direct;
	
	if(pwm>1000)
		pwm=1000;
	if(pwm<-1000)
		pwm=-1000;
	if(pwm>0)
		direct=1;
	if(pwm<0)
		direct=0;
	
	switch(num){
		case 3:
			switch(direct){
				case 1:
					__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,1000-pwm);//3,1
				  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,1); 
				  break;
				case 0 :
					__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,-pwm);//3,1
				  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,0); 
				  break;
			}
			break;
			
		case 4:
			switch(direct){
				case 1:
					__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,pwm);//4,0
				  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,0); 
				  break;
				case 0 :
					__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,1000+pwm);//4,0
				  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,1); 
				  break;
			}
			break;	
		}			
}
struct pidstruct initPID(float kp,float ki,float kd)//pid初始化
{
	struct pidstruct e;
	e.kp=kp;
	e.ki=ki;
	e.kd=kd;
	e.p=0;
	e.i=0;
	e.d=0;
	e.lastde=0;
	e.thisde=0;
	return e;
}
float PID(struct pidstruct *e,float err,float outlow,float outhigh)//计算输出
{
	float out;
	e->thisde = err;
	e->p = e->kp * e->thisde;
	e->i = e->ki * (e->i/e->ki + e->thisde);
	e->d = e->kd * (e->thisde - e->lastde);
	e->lastde = e->thisde;
	out = e->p + e->i + e->d;
	if(e->i > outhigh) e->i=outhigh;
	if(e->i < outlow) e->i=outlow;
	if(out>outhigh) out=outhigh;
	if(out<outlow) out=outlow;
	return out;
}
void Move_v(float targetv)//速度为目标运动
{
	float dout=0,out3,out4,out;
	dout=-PID(&piddistance,0-gest.distance,-1000,1000);
	out3=PID(&pidspeed1,targetv-dout-gest.v1,300,900);
	out4=PID(&pidspeed2,targetv+dout-gest.v2,300,900);
	Wheel(3,out3);
	Wheel(4,out4);

}
void Move_stop()  //停车
{
	float out3,out4;
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,1000);//3,1
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_3,1); 
	
	__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_4,1000);//4,0
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_15,1);

	Wheel(4,out4);
	flag = 0;
}
void clear_x()   //清除记录路程
{
	gest.x=0;
	gest.x1=0;
	gest.x2=0;
}
void Wait_Stop(int time_stop)  //延时停车
{
	static int wait = 0;
	wait++;
	if(wait==time_stop)
	{
		Move_stop();
		clear_x();
		wait=0;
	}
}
void Waiting(int time_stop,int time_move)  //等停
{
	static int wait=0,time=0,flag=0;
	if(flag==0)
	{
		wait++;
		if(wait==time_stop)
	  {
		  Move_stop();
			wait=0;
			flag=1;
	  }
	}
	if(flag==1)
	{
		time++;
		if(time==time_move)
		{
		  mode=1;
			time=0;
		  flag=0;
		}
	}
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)  //串口中断
{
	static int count=0;
	static char a[10];
	if(huart == &huart1)
	{
		DataGet1(rx_buf1[0]);
		HAL_UART_AbortReceive_IT(&huart1);
		HAL_UART_Receive_IT(&huart1, rx_buf1, 1);
	}
	if(huart == &huart3)
	{
		if(rx_buf3[0]=='\n')  //接收树莓派数据
		{
			a[count]='\0';
			gest.distance=atof(a);
			count=0;
			a[0]='\0';
			k++;
		}
		else if(rx_buf3[0]=='\r')
		  mode = 0;
		else if(rx_buf3[0]=='\t')
			mode = 2;
		else if(rx_buf3[0]=='r')
		{
			mode = 1;
			clear_x();
			flag+=1;
			if(flag==3)  HAL_UART_Transmit(&huart3,(uint8_t *)'t',2,0xffff);
		}
		else if(rx_buf3[0]=='l')
		{
			mode = 4;
			clear_x();
			flag+=1;
			if(flag==3)  HAL_UART_Transmit(&huart3,(uint8_t *)'t',2,0xffff);
		}
		else
		{
			a[count]=rx_buf3[0];
			count++;
		}
		DataGet3(rx_buf3[0]);
		HAL_UART_AbortReceive_IT(&huart3);
		HAL_UART_Receive_IT(&huart3, rx_buf3, 1);
	}
}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)  //输入捕获中断
{
	static float catch3[3],catch4[3];
	int judge3,judge4;
	if(htim==&htim1&&htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)//轮三
	{
		//printf("1");
		judge3=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_14);
		catch3[1]=HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
		catch3[2]=catch3[1]-catch3[0];
		if(catch3[2]<0)  catch3[2]+=0xffff;
		catch3[0] = catch3[1];
		//printf("%f\t",catch3[2]);
		
		if(judge3==0)
		{
			gest.x1+=0.52f;
			gest.v1=523598.78/catch3[2];
		}
		if(judge3==1)
		{
			gest.x1-=0.52f;
			gest.v1=-523598.78/catch3[2];
		}
	}
	if(htim==&htim1&&htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4)//轮四
	{
		//printf("1");
		judge4=HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_15);
		catch4[1]=HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_4);
		catch4[2]=catch4[1]-catch4[0];
		if(catch4[2]<0)  catch4[2]+=0xffff;
		catch4[0] = catch4[1];
		//printf("%f\t",catch4[2]);
		
		if(judge4==1)
		{
			gest.x2+=0.52f;
			gest.v2=523598.78/catch4[2];
		}
		if(judge4==0)
		{
			gest.x2-=0.52f;
			gest.v2=-523598.78/catch4[2];
		}
	}
	gest.x=(gest.x1+gest.x2)/2;
}
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)//定时器中段
{
	static int wait=0;
	if(mode==0)  //完全停车模式
	{
		Wait_Stop(100);
	}
	if(mode==1)  //右循迹
	{
	  Move_v(600);
		if(gest.x>x_right)
		{
			HAL_UART_Transmit(&huart3,(uint8_t *)'o',2,0xffff);
			clear_x();
		}
	}
	if(mode==4)  //左循迹
	{
		Move_v(600);
		if(gest.x>x_left)
		{
			HAL_UART_Transmit(&huart3,(uint8_t *)'o',2,0xffff);
			clear_x();
		}
	}
	if(mode==2)  //等停模式
	{
		Waiting(1,5000);
//		Move_stop();
//	  time++;
//		if(time==5000)
//		{
//			mode=1;
//			time=0;
//		  flag=0;
//		}
	}
	if(mode==3)
	{
		Move_stop();
		clear_x();
		wait++;
		if(wait==2000)
		{
		  HAL_UART_Transmit(&huart3,(uint8_t *)'o',2,0xffff);
			wait=0;
		}
	}
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */
	HAL_UART_Receive_IT(&huart1,rx_buf1,1);
	HAL_UART_Receive_IT(&huart3,rx_buf3,1);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);
	HAL_TIM_Base_Start_IT(&htim4);
	HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_1);
	HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_2);
	
	HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_3);
	HAL_TIM_IC_Start_IT(&htim1,TIM_CHANNEL_4);
	
	pidspeed1=initPID(0.4,0.09,0.01);
	pidspeed2=initPID(0.4,0.09,0.01);
	piddistance=initPID(0.85,0,0.40);
	
	//OLED_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
//		printf("%f\t",gest.x);
		//printf("%f\t",gest.v2);
		//Wheel(3,600);
//	HAL_UART_Transmit(&huart3,(uint8_t *)'t',2,0xffff);
//////		Wheel(4,500);
//		HAL_Delay(1000);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
int fputc(int ch, FILE *f)
{
	HAL_UART_Transmit(&huart1,(uint8_t*)&ch,1,10);
	return ch;
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
