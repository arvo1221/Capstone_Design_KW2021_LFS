/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2021 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"

/* USER CODE BEGIN Includes */
#include "string.h"
#include "math.h"
#include "dataType.h"

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim9;

UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart6_rx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
#define RX_BUFFER_SIZE 18
#define TX_BUFFER_SIZE 12
#define dt1 0.0005
#define dt2 0.005
#define dt3 0.05
#define ppr 4000
#define pi 3.141592653589793
#define get_dma_data_length() huart6.hdmarx->Instance->NDTR
#define get_dma_total_size() huart6.RxXferSize
#define shooting 100
#define noshooting 0

int32_t encoder_cnt_R = 0;
int32_t encoder_cnt_pre_R = 0;
int32_t cnt_R = 0;
int32_t k = 0;
int16_t mode = 0;
volatile double angular_velocity_R = 0;
volatile double angular_velocity_pre_R = 0;
volatile double theta = 0;
volatile uint16_t Motor_CCR_R = 0;
volatile uint16_t shoot_degree = 0;
volatile uint32_t timer = 0;
volatile uint32_t shoot_timer = 0;

volatile double Kpc = 0.9;//45.5672;
volatile double Kic = 720;//8685.17;

volatile double Kac = 0;
volatile double Kps = 0.7;//{20, 0};
volatile double Kis = 5;//{30.24, 0};

volatile double Kas = 0;
volatile double Es = 0;
volatile double Eis = 0;
volatile double V_control = 0;//speed
volatile double rep_r = 0;
volatile double rep_current = 0;
volatile double Ecurrent = 0;
volatile double EScurrent = 0;
volatile double C_control = 0;
volatile double Motor_PWM = 0;
volatile double current_saturation = 1.1;

volatile double Kpp = 8.0;//8.0
volatile double Kip = 0;
volatile double Kdp = 0.24;//0.248
volatile double Kap = 0;
volatile double Eposition = 0;
volatile double ESposition = 0;
volatile double Edposition = 0;
volatile double rep_position = 0;
volatile double rep_degree = 0;
volatile double P_control;
volatile double velocity_saturation = 2.0;

volatile uint32_t adcVal[2];
volatile uint32_t adcVal_pre;
volatile double current_offset = 3121.5;
volatile double current;
volatile double current_pre;
volatile double current_mv[10];
uint16_t c_cnt = 0;
volatile double tau = 0.00051;
volatile double current_sum = 0;

int16_t watch_current;
int16_t watch_repc;
int16_t watch_velocity;
int16_t watch_repv;
int16_t watch_repp;
int16_t watch_degree;

uint8_t rx_dma_buffer[RX_BUFFER_SIZE];
uint8_t tx_buffer[TX_BUFFER_SIZE];
volatile uint8_t rx_buffer[RX_BUFFER_SIZE];
volatile uint32_t rx_size = 0;
volatile uint8_t tx_len;
volatile uint16_t old_pos = 0;
volatile uint16_t pos;
volatile unsigned char g_tail_pos = 255;
volatile unsigned char g_head_pos;
volatile unsigned char g_PacketMode;
volatile unsigned char g_checkSize = 0;
volatile unsigned char g_checksum;
volatile unsigned char g_ID = 2;
volatile Packet_t g_PacketBuffer;
volatile Packet_t g_Tx_Packet;
volatile uint8_t g_SendFlag;

volatile uint8_t shoot = 0;
volatile uint16_t s = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM9_Init(void);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

void Uart_rx_dma_handler() {
  uint8_t cnt;
  uint8_t i;
  
  rx_size = 0;
  g_head_pos = get_dma_total_size() - get_dma_data_length();
  while(g_tail_pos != g_head_pos) {
    switch(g_PacketMode) {
      case 0:
        if(rx_dma_buffer[g_tail_pos] == 0xFE) {//Header check
          g_checkSize++;
          if(g_checkSize == 4) g_PacketMode = 1; // header 4개가 다 들어왔으면 1번 모드로 변경 
        }
        else g_checkSize = 0;//초기화
      break;
      
      case 1:
        g_PacketBuffer.buffer[g_checkSize++] = rx_dma_buffer[g_tail_pos]; //checkSize 8될때 까지 g_PacketBuffer.buffer에 받아온 값 담기(size,mode,g_checksum)
        if (g_checkSize == 8) g_PacketMode = 2;
      break;
      
      case 2:
        g_PacketBuffer.buffer[g_checkSize++] = rx_dma_buffer[g_tail_pos];
        g_checksum += rx_dma_buffer[g_tail_pos];
         
        if (g_checkSize == g_PacketBuffer.data.size){					//data size만큼 버퍼읽음
          if (g_checksum == g_PacketBuffer.data.check){					// checksum check                 
            switch(g_PacketBuffer.data.mode){				// 모든게 올바르면, Get target position, Velocity limit Current Limit
              case 2:
                rep_position = -g_PacketBuffer.data.pos_P / 1000.;
                velocity_saturation = g_PacketBuffer.data.vel_P / 1000.;
                shoot = g_PacketBuffer.data.shoot;
                break;
            }              
           }
           //초기화
           g_checksum = 0;						
           g_PacketMode = 0;
           g_checkSize = 0;
        }
        
        else if(g_checkSize > g_PacketBuffer.data.size || g_checkSize > sizeof(Packet_t)){		//오류시 초기화
         g_checksum = 0;
         g_PacketMode = 0;
         g_checkSize = 0;
       }
      
    
    }//switch
  
    if(g_SendFlag >= 2) {			
      g_SendFlag = 0;
      
      g_SendFlag = 0;
      g_Tx_Packet.data.header[0] = g_Tx_Packet.data.header[1] = g_Tx_Packet.data.header[2] = g_Tx_Packet.data.header[3] = 0xFE;
      g_Tx_Packet.data.id = g_ID;							//motor id 다만 이번과제에선 사용하지 않음
      g_Tx_Packet.data.size = sizeof(Packet_data_t);
      g_Tx_Packet.data.mode = 3;//mode 3
      g_Tx_Packet.data.check = 0;
       
      g_Tx_Packet.data.pos_P = -theta * 1000.;
      g_Tx_Packet.data.vel_P = angular_velocity_R * 1000.;
      g_Tx_Packet.data.shoot = shoot;
      
      for(int i = 8; i< sizeof(Packet_t);i++)		// checksum 제작
      g_Tx_Packet.data.check += g_Tx_Packet.buffer[i];
      
      for(int i =0; i<g_Tx_Packet.data.size;i++){		//송신
        tx_buffer[i] = g_Tx_Packet.buffer[i];
      }
      HAL_UART_Transmit(&huart6, tx_buffer, g_Tx_Packet.data.size,10);
      s++;
    }
    g_tail_pos++;
  }//while
        
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if(htim == &htim4) {
    timer++;
    //HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);

    HAL_ADC_Start_DMA(&hadc1,(uint32_t*)adcVal,1);
    adcVal[0] = (double)(0.00005*adcVal[0] + tau*adcVal_pre)/(tau+0.00005);
    adcVal_pre = adcVal[0];
    current_sum -= current_mv[c_cnt];
    current_mv[c_cnt] = adcVal[0];
    current_sum += current_mv[c_cnt];
    c_cnt = (c_cnt+1)%10;
    current = (double)(current_sum/10.-current_offset)/4096*33.;
    if(mode == 1) {
      k++;
      if(k > 8000) {
        current_offset = current_sum/10.;
        Ecurrent = 0;
        EScurrent = 0;
        Es = 0;
        Eis = 0;
        Motor_CCR_R = 0;
        timer = 1;   
        C_control = 0;
        mode = 2;
      }
    }
    else if(mode ==2) {
      if(timer%1000 == 0) {
        
        rep_position = (double)rep_degree*pi/180;
        /*
        if(theta < -1 * pi) { 
          theta += 2.0*pi;
        }
        else if(theta > pi) {
          theta -= 2.0*pi;
        }
        if(rep_position < -1 * pi) { 
          rep_position += 2.0*pi;
        }
        else if(rep_position > pi) {
          rep_position -= 2.0*pi;
        }*/
        Edposition = Eposition;
        Eposition = (double)rep_position - theta;
        ESposition += (double)Eposition;
        //Kap = 1./(3*Kpp);
        P_control = (double)(Kpp * Eposition +/*Kip*ESposition*dt3 +*/ Kdp *(Eposition - Edposition)/dt3);
        
        if(P_control >= velocity_saturation) {
          ESposition -= (double)(P_control - velocity_saturation)*Kap;
          P_control = velocity_saturation;
        }
        else if(P_control <= -velocity_saturation) {
          ESposition -= (double)(P_control + velocity_saturation)*Kap;
          P_control = -velocity_saturation;
        }
      
        g_SendFlag++;
        timer = 0;
      }
      
      if(timer%100 == 0) {
        encoder_cnt_pre_R = encoder_cnt_R;
        encoder_cnt_R = TIM3->CNT;
        cnt_R = encoder_cnt_R - encoder_cnt_pre_R;
        rep_r = P_control;
        
        if(encoder_cnt_R < 500 && encoder_cnt_pre_R > 64500 ) cnt_R += 65535;
        else if(encoder_cnt_pre_R < 500 && encoder_cnt_R > 64500) cnt_R -= 65535;
        
        theta += (double)(2*pi*cnt_R/ppr);
        angular_velocity_R = (double)(2*pi*cnt_R/ppr)/dt2;
        
        Es = (double)(rep_r - angular_velocity_R);
        Eis += (double)Es;
        
        V_control = (double)(Kps * Es + Kis * Eis*dt2);
          
        if(V_control >= current_saturation) {
          Eis -= (double)(V_control - current_saturation)*Kas;
          V_control = current_saturation;
        }
        else if(V_control <= -current_saturation) {
          Eis -= (double)(V_control + current_saturation)*Kas;
          V_control = -current_saturation;
        }
        if(shoot == 1) {
        shoot_timer++;
        //shoot_degree =  shooting;
        if(shoot_timer%100 == 0) {
          //shoot_degree = noshooting;
          shoot = 0;
          shoot_timer = 0;
          }
        }
      }
      
      if(timer%10 == 0) {
        //current[0] = (double)(0.005*current_n[0] + tau*current_pre[0])/(tau+0.005);
        //current_pre[0] = current[0];
        rep_current = V_control;
        Ecurrent = (double)(rep_current-current);
        C_control = (double)(Kpc * Ecurrent + Kic * EScurrent * dt1);
        //C_control += 0.005*angular_velocity_R;
        EScurrent += (double)Ecurrent;
        
        if(C_control >= 12) {
          EScurrent -= (C_control - 12)*Kac;
          C_control = 12;
        }
        else if(C_control <= -12) {
          EScurrent -= (C_control + 12)*Kac;
          C_control = -12;
        }
        
        if(C_control < 0 ) {
          HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_SET);
          C_control = -C_control;
        }
        else {
          HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);
        }
      }
      
      Motor_PWM = (double)C_control*374.9166666667;
      Motor_CCR_R = (uint16_t)Motor_PWM;
      //HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET);
    }
    TIM9->CCR1 = Motor_CCR_R;
    //TIM1->CCR1 = shoot_degree;
    /*
    servomotor 20ms prescale 60 count 60000
    */
    watch_current = 1000*current;
    watch_repc = 1000*rep_current ;
    watch_velocity = 1000*angular_velocity_R;
    watch_repv = 1000 * rep_r;
    watch_repp = 1000 *rep_degree;
    watch_degree = 1000 *theta/pi*180.;
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  *
  * @retval None
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
      Motor_CCR_R = 0;
      mode=1;
  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_DMA_Init();
  MX_TIM4_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_USART6_UART_Init();
  MX_ADC1_Init();
  MX_TIM9_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
  Kac = 1./(2.1*Kpc);
  Kas = 1./(1.0*Kps);
  Kap = 1./(3*Kpp);
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_UART_Receive_DMA(&huart6, rx_dma_buffer, RX_BUFFER_SIZE);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim9, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim9, TIM_CHANNEL_1);
  //HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  //HAL_TIMEx_PWMN_Start(&htim4, TIM_CHANNEL_2);
  //HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
  //HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
  //HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_5,GPIO_PIN_RESET);//Set CCW //RESET CW
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
  Uart_rx_dma_handler();
  }
  /* USER CODE END 3 */

}

/**
  * @brief System Clock Configuration
  * @retval None
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Activate the Over-Drive mode 
    */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
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
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

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

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
    */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
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
  htim2.Init.Period = 60000;
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

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
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
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 1-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 4500-1;
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

  HAL_TIM_MspPostInit(&htim4);

}

/* TIM9 init function */
static void MX_TIM9_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 60-1;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 60000-1;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim9);

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

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

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

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA4 PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  file: The file name as string.
  * @param  line: The line in file as a number.
  * @retval None
  */
void _Error_Handler(char *file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
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
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
