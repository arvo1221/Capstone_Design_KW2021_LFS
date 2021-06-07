/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "dataType.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define RX_BUFFER_SIZE 256
#define TX_BUFFER_SIZE 256
#define M_PI 3.141592

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_usart6_rx;

/* USER CODE BEGIN PV */

//Serial 
uint8_t rx_dma_buffer[RX_BUFFER_SIZE];
uint8_t rx_buffer[RX_BUFFER_SIZE];
uint32_t rx_size = 0;

uint8_t tx_buffer[TX_BUFFER_SIZE];
uint8_t tx_len;

volatile unsigned char g_tail_pos = 255;
volatile unsigned char g_head_pos;

volatile Packet_t g_PacketBuffer;
volatile unsigned char g_PacketMode;
volatile unsigned char g_ID=1;
volatile unsigned char checkSize;
volatile unsigned char g_checksum;


volatile Packet_t g_Tx_Packet;
volatile unsigned char g_Tx_checksum;
volatile int g_SendFlag;

// Motor L
volatile int16_t Motor_CCR_Y = 0;
//position
volatile double g_Pdes_Y = 0;
volatile double g_Pcur_Y, g_Ppre_Y;
volatile double g_Perr_Y, g_Pperr_Y;
volatile double g_Pderr_Y;

volatile double Kp_p_Y = 9.85;
volatile double Kd_p_Y = 0.26;//0.12;

//speed
volatile double g_Vcur_Y, g_Vpre_Y;
volatile double g_Vdes_Y;
volatile double g_Verr_Y;
volatile double g_Vlimit_Y = 10.;
volatile double g_Verr_sum_Y;

volatile double Kp_s_Y = 0.5953;
volatile double Ki_s_Y = 83.3;     //153.3651;

	
volatile int g_Pdes_Y_display; 
volatile int g_Vlimit_Y_display;
volatile int g_Climit_display;



volatile double g_vel_control_Y;
volatile double g_pos_control_Y;

//Motor2
volatile int16_t Motor_CCR_P = 0;
//position
volatile double g_Pdes_P = 0., g_Ppre_P;
volatile double g_Pcur_P, g_Pvcur_P;
volatile double g_Perr_P;
volatile double g_Pderr_P;

volatile double Kp_p_P = 7.5;
volatile double Kd_p_P = 0.35;

//speed
volatile double g_Vcur_P, g_Vpre_P;
volatile double g_Vdes_P = 0;
volatile double g_Verr_P;
volatile double g_Vlimit_P = 10.;
volatile double g_Verr_sum_P;

volatile double Kp_s_P = 0.49053;
volatile double Ki_s_P = 33.3;//46.660104;

//curreent
volatile double g_Ccur_P, g_Cdes_P;
double g_Cerr_P, g_Cerr_sum_P;
volatile double g_Climit_P = 0.1;
volatile double Kp_c_P = 0.2353;
volatile double Ki_c_P = 46.3;//46.660104;


volatile double g_vel_control_P;
volatile double g_pos_control_P;
volatile double g_cur_control_P;

volatile int g_Pdes_P_display; 
volatile int g_Vlimit_P_display;
volatile int g_Climit_display;

//TImer4 관련
volatile uint16_t g_TimerCnt;
volatile uint16_t cnt;
volatile uint16_t timer;

//encoder
volatile uint16_t   encoder_cnt_P;
volatile int32_t   encoder_cnt_Y;
volatile int32_t   pre_encoder_cnt_Y;
volatile uint16_t   pre_encoder_cnt_P;
volatile int32_t P_cnt_error;

//ADC
volatile uint32_t g_ADC1[2]; 
volatile uint32_t g_PreADC1;
volatile uint32_t g_ADC1_filter;
int adc_dir;

//for J-link
int J_Vdes_Y;
int J_Vcur_Y;
int J_Pdes_Y;
int J_Pcur_Y;

int J_Vdes_P;
int J_Vcur_P;
int J_Pdes_P;
int J_Pcur_P;
int J_Cdes_P;
int J_Ccur_P;



double fake_v_des;


double fake_cnt;
int flag;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void Uart_rx_dma_handler() {
  uint8_t cnt;
  uint8_t i;

  rx_size = 0;
  g_head_pos = huart6.RxXferSize - huart6.hdmarx->Instance->NDTR -1;
  while(g_tail_pos != g_head_pos) {
//    sprintf(tx_buffer,"%c\n",rx_dma_buffer[g_tail_pos]);
//    HAL_UART_Transmit(&huart6,tx_buffer,sizeof(tx_buffer), 10);
//    
    switch (g_PacketMode) {    			//mode0 : 패킷의 들어오는지 확인
      case 0:
            if (rx_dma_buffer[g_tail_pos] == 0xFE) {			//Header check
                    checkSize++;
                    if (checkSize == 4) { // header 4개가 다 들어왔으면 1번 모드로 변경
                        g_PacketMode = 1;
                    }
            }
            else {
                    checkSize = 0;	//초기화 
            }
      break;

      case 1:
        
        g_PacketBuffer.buffer[checkSize++] = rx_dma_buffer[g_tail_pos]; //checkSize 8될때 까지 g_PacketBuffer.buffer에 받아온 값 담기(size,mode,g_checksum)
        if (checkSize == 8){
               g_PacketMode = 2;
        }
      break;
     
     case 2:
       g_PacketBuffer.buffer[checkSize++] = rx_dma_buffer[g_tail_pos];
       g_checksum += rx_dma_buffer[g_tail_pos];
       
       if (checkSize == g_PacketBuffer.data.size){					//data size만큼 버퍼읽음
               if (g_checksum == g_PacketBuffer.data.check){					// checksum check
                 
                    switch(g_PacketBuffer.data.mode){				// 모든게 올바르면, Get target position, Velocity limit Current Limit
                                 case 2:
                                   g_Pdes_Y_display = g_PacketBuffer.data.pos_Y ;
                                   g_Vlimit_Y_display = g_PacketBuffer.data.velo_Y ;
                                   
                                 g_Pdes_Y = g_PacketBuffer.data.pos_Y / 1000.;
                                 g_Vlimit_Y = g_PacketBuffer.data.velo_Y / 1000.;
                                 g_Pdes_P = g_PacketBuffer.data.pos_P / 1000.;
                                 g_Vlimit_P = g_PacketBuffer.data.velo_P / 1000.;
                                 break;
                         }
                   
                       
               }
               //초기화
               g_checksum = 0;						
               g_PacketMode = 0;
               checkSize = 0;
               
       }
       else if(checkSize > g_PacketBuffer.data.size || checkSize > sizeof(Packet_t)){		//오류시 초기화
               g_checksum = 0;
               g_PacketMode = 0;
               checkSize = 0;
       }
    }
    
     if(g_SendFlag >500) {			
            g_SendFlag = 0;
            g_Tx_Packet.data.id	= g_ID;							//motor id 다만 이번과제에선 사용하지 않음
            g_Tx_Packet.data.size = sizeof(Packet_data_t);
            g_Tx_Packet.data.mode = 3;						//mode 3
            g_Tx_Packet.data.check = 0;
            
            g_Tx_Packet.data.pos_Y = g_Pcur_Y*180./M_PI*1000;
            g_Tx_Packet.data.velo_Y = g_Vcur_Y* 1000;
            g_Tx_Packet.data.pos_P= g_Pcur_P*180./M_PI * 1000;
            g_Tx_Packet.data.velo_P= g_Vcur_P * 1000;
            
            for(int i = 8; i< sizeof(Packet_t);i++)		// checksum 제작
            g_Tx_Packet.data.check += g_Tx_Packet.buffer[i];
            
            for(int i =0; i<g_Tx_Packet.data.size;i++){		//송신
              tx_buffer[i] = g_Tx_Packet.buffer[i];
            }
            HAL_UART_Transmit(&huart6, tx_buffer, g_Tx_Packet.data.size,10);
    }
    
    
    g_tail_pos++;
  }
  
 //     HAL_UART_Transmit(&huart6,"a",2, 10);

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if(htim == &htim4) {                                                          //20Khz
   //     HAL_GPIO_TogglePin(GPIOA,GPIO_PIN_8);             //motor1 REF

       
       
       
    if((g_TimerCnt % 100) == 0){		// position control     200Hz(5ms)

                      g_TimerCnt = 0;
                      
                      //Yaw관련 제어기
                //      if(g_Pdes_Y < 0) g_Pdes_Y + 2*M_PI;	// 음수 target시 0~360으로 표현
                      // 효율적인 도는 방향 정하기
                      
                      if(g_Pdes_Y > 90) g_Pdes_Y = 90;
                      else if(g_Pdes_Y<-90) g_Pdes_Y = -90;
                      
                      g_Perr_Y = ((double)g_Pdes_Y*M_PI/180.) - g_Pcur_Y;		//position error
                      if(g_Perr_Y > 360) {
                              g_Perr_Y = g_Pperr_Y;
                      }
                      else if(g_Perr_Y < -360) {
                              g_Perr_Y = g_Pperr_Y;
                      }
                      g_Pderr_Y = g_Perr_Y - g_Pperr_Y;		//position error_dot
                      
                      // 효율적인 도는 방향 정하기
                      if(g_Perr_Y > M_PI) {
                              g_Perr_Y -= 2*M_PI;
                      }
                      else if(g_Perr_Y < -M_PI) {
                              g_Perr_Y += 2*M_PI;
                      }
                      
                      g_pos_control_Y = (double) g_Perr_Y * Kp_p_Y +  g_Pderr_Y* Kd_p_Y;		//pd controller

                      //gear가 적용된 motor의 최대 속도는 161rpm (16.8599 rad/sec)
                      // turret의 초대 속도는 4.25rpm
                      if(g_pos_control_Y > 4.0150){
                              g_pos_control_Y = 4.2150;
                      }
                      else if(g_pos_control_Y < -4.0150){
                              g_pos_control_Y = -4.2150;
                      }
                      
                      //pitch관련 제어기 
                      if(g_Pdes_P >30 || g_Pcur_P > 30) g_Pdes_P = 30;                   //joint limit
                      if(g_Pdes_P<-90 || g_Pcur_P < -90) g_Pdes_P = -90;                  //joint limit
                      g_Perr_P = ((double)g_Pdes_P*M_PI/180.) - (g_Pcur_P);		//position error
                      g_Pderr_P = g_Perr_P - g_Ppre_P;		//position error_dot
                      
                      // 효율적인 도는 방향 정하기
                      if(g_Perr_P > M_PI) {
                              g_Perr_P -= 2*M_PI;
                      }
                      else if(g_Perr_P < -M_PI) {
                              g_Perr_P += 2*M_PI;
                      }
                      
                      g_pos_control_P = (double) g_Perr_P * Kp_p_P +  g_Pderr_P* Kd_p_P;		//pd controller

                      //gear가 적용된 motor의 최대 속도는 161rpm (16.8599 rad/sec)
                      if(g_pos_control_P > 2.5){
                              g_pos_control_P = 2.5;
                      }
                      else if(g_pos_control_P < -2.5){
                              g_pos_control_P = -2.5;
                      }
                        
                      g_Pperr_Y = g_Perr_Y;
                      g_Ppre_P = g_Perr_P;
                      
              }
    if((g_TimerCnt%20) == 0) {                          // speed control     1000Hz 1ms   0.001초
             //speed limit -Vlimit ~ +Vlimit
                if(g_pos_control_P > g_Vlimit_P){	
                        g_pos_control_P = g_Vlimit_P;
                }
                else if(g_pos_control_P < -g_Vlimit_P){
                        g_pos_control_P = -g_Vlimit_P;
                }
                
                 encoder_cnt_P = TIM3->CNT;
                P_cnt_error = encoder_cnt_P - pre_encoder_cnt_P;
                if(encoder_cnt_P>65000 && pre_encoder_cnt_P < 500){
                      P_cnt_error -= 65535 ;
                }
                else if(encoder_cnt_P< 100 && pre_encoder_cnt_P > 65500){
                      P_cnt_error += 65535 ;
                }
                g_Pcur_P += (P_cnt_error / (4000. * 49.) ) * 2 * M_PI;   //  encoder counter -> radian
                 
                pre_encoder_cnt_P = encoder_cnt_P;
                
                
                g_Vdes_P = g_pos_control_P; // rad/sec
               // g_Vdes_P = fake_v_des; // rad/sec
                
                
               g_Vcur_P = (g_Pcur_P - g_Pvcur_P) / 0.001;	// (현재 엔코더값 - 예전 엔코더값)/0.0005초 -> 현재속도 [rad/sec]
              //  g_Vcur_P = (P_cnt_error)*2*M_PI / 12.;
                g_Pvcur_P = g_Pcur_P;						// 에전 엔코더값 저장
                
                g_Verr_P = g_Vdes_P - g_Vcur_P;  //speed error
                 
                g_vel_control_P = g_Verr_P * Kp_s_P + g_Verr_sum_P * Ki_s_P * 0.001;		// PI제어기
                
                
                g_Verr_sum_P += g_Verr_P;	// spped error sum
                
                
                J_Vdes_P = g_Vdes_P*100;
                J_Vcur_P = g_Vcur_P*100;
                J_Pdes_P = g_Pdes_P;
                J_Pcur_P = g_Pcur_P*180/M_PI;
                
                
                //최대 허용 전류 saturation & anti-windup
                if(g_vel_control_P > 1.8){								
                        g_Verr_sum_P -= (g_vel_control_P - 1.8) * 1. / (Kp_s_P*3);	//  anti windup gain은 1/Kps
                        g_vel_control_P = 1.8;
                }
                else if(g_vel_control_P < -1.8){
                        g_Verr_sum_P -= (g_vel_control_P + 1.8) * 1. / (Kp_s_P*3); //  anti windup gain은 1/Kps
                        g_vel_control_P = -1.8;
                }
                
                Motor_CCR_P = g_vel_control_P * (2250. / 2.0) - 1;                
                  if(Motor_CCR_P>=0) {
                    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_RESET);             //motor1 DIR
                  TIM4->CCR2 =  Motor_CCR_P;
                  }
                  else{
                    HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_SET);             //motor1 DIR
                    TIM4->CCR2 = -Motor_CCR_P;
                  }
    }   
      
      if((g_TimerCnt % 10) == 0){			// speed control     2000Hz 0.5ms   0.0005초
                //speed limit -Vlimit ~ +Vlimit
                if(g_pos_control_Y > g_Vlimit_Y){	
                        g_pos_control_Y = g_Vlimit_Y;
                }
                else if(g_pos_control_Y < -g_Vlimit_Y){
                        g_pos_control_Y = -g_Vlimit_Y;
                }
                
                encoder_cnt_Y = TIM2->CNT-214748360;
               
                
                
                g_Pcur_Y = ((encoder_cnt_Y ) / (4096. * 26. * 4.)) * 2 * M_PI;   //  encoder counter -> radian
                

                
                
                g_Vdes_Y = g_pos_control_Y; // rad/sec
                //g_Vdes_Y = fake_v_des; // rad/sec
                
                
                
                g_Vcur_Y = (g_Pcur_Y - g_Ppre_Y) / 0.0005;	// (현재 엔코더값 - 예전 엔코더값)/0.0005초 -> 현재속도 [rad/sec]
                g_Ppre_Y = g_Pcur_Y;						// 에전 엔코더값 저장
                
                g_Verr_Y = g_Vdes_Y - g_Vcur_Y;  //speed error
                 
                g_vel_control_Y = g_Verr_Y * Kp_s_Y + g_Verr_sum_Y * Ki_s_Y * 0.0005;		// PI제어기
                
                
                g_Verr_sum_Y += g_Verr_Y;	// spped error sum
                
                J_Vdes_Y = g_Vdes_Y*100;
                J_Vcur_Y = g_Vcur_Y*100;
               J_Pdes_Y = g_Pdes_Y;
                J_Pcur_Y = g_Pcur_Y*180/M_PI;
                                
                //최대 허용 전류 saturation & anti-windup
                if(g_vel_control_Y > 1.6){								
                        g_Verr_sum_Y -= (g_vel_control_Y - 1.6) * 1. / (3*Kp_s_Y);	//  anti windup gain은 1/Kps
                        g_vel_control_Y = 1.6;
                }
                else if(g_vel_control_Y < -1.6){
                        g_Verr_sum_Y -= (g_vel_control_Y + 1.6) * 1. / (3*Kp_s_Y); //  anti windup gain은 1/Kps
                        g_vel_control_Y = -1.6;
                }
                
               Motor_CCR_Y = g_vel_control_Y * (2250. / 1.6);
                if(Motor_CCR_Y>=0) {
                  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_SET);             //motor1 DIR
                  TIM4->CCR1 = Motor_CCR_Y;
                }
                else{
                  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_RESET);             //motor1 DIR
                  TIM4->CCR1 = -Motor_CCR_Y;
                }
                
               
              }
                
           // Curret control
    
              
            HAL_ADC_Start_DMA(&hadc1,(uint32_t *)g_ADC1,1);
            
             g_ADC1_filter = (0.04)*g_ADC1[0] + (0.96)*g_PreADC1;
            
             
             g_PreADC1 = g_ADC1_filter;
          
            g_Ccur_P = ((g_ADC1_filter* 3.3 / 4096.  ) -  2.5548) * 10. ;
            /*
            
            J_Ccur_P = g_Ccur_P*100;
            J_Cdes_P = g_Cdes_P*100;
            
           // g_Cdes_P = -g_vel_control_P;
             g_Cdes_P = -0.1;
             
            if(g_Cerr_P > g_Climit_P)
                  g_Cerr_P = g_Climit_P;
            else if(g_Cerr_P < -g_Climit_P)
                g_Cerr_P = -g_Climit_P;
            
            
            g_Cerr_P = g_Cdes_P - g_Ccur_P;
            
            g_cur_control_P = g_Cerr_P * Kp_c_P + g_Cerr_sum_P * Ki_c_P*0.00005;
     //       g_cur_control_P += g_Vcur_P * 
            g_Cerr_sum_P += g_Cerr_P;
            
          if(g_cur_control_P >= 24){									//최대 출력 24V
            g_Cerr_sum_P -= (g_cur_control_P - 24.) * 1. /  (Kp_c_P*3.);		// anti windup 계수 1/3kp
            g_cur_control_P = 24;
          }
          else if(g_cur_control_P <= -24){
                  g_Cerr_sum_P -= (g_cur_control_P + 24.) * 1. / (Kp_c_P*3.);		// anti windup 계수 1/3kp
                  g_cur_control_P = -24;
          }
            */
            
          // Motor_CCR_P = g_cur_control_P * (2250. / 24.) - 1;                
          
            
          g_SendFlag++;
          g_TimerCnt++;      
   //       HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_RESET);             //motor1 REF


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
        flag = 1;
        Motor_CCR_P = 0;

  /* USER CODE END 1 */
  

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  g_Tx_Packet.data.header[0] = g_Tx_Packet.data.header[1] = g_Tx_Packet.data.header[2] = g_Tx_Packet.data.header[3] = 0xfe;
  
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART6_UART_Init();
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
    TIM2->CNT = 214748360;

  HAL_UART_Receive_DMA(&huart6, rx_dma_buffer, RX_BUFFER_SIZE);
  HAL_TIM_Base_Start_IT(&htim4);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(&htim4, TIM_CHANNEL_1);                          //motor1 pwm
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
  
  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_8,GPIO_PIN_RESET);             //motor1 REF
  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_SET);             //motor1 DIR            set일때 CW
  
  
  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_10,GPIO_PIN_SET);           // motor2  reset
  HAL_GPIO_WritePin(GPIOA,GPIO_PIN_11,GPIO_PIN_SET);            // 
  
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_8,GPIO_PIN_RESET);             //motor2 DIR
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_9,GPIO_PIN_RESET);             //motor2 MODE
  
  HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10,GPIO_PIN_RESET);           // motor2 COAST
//  g_Pcur_Y = 0;
  

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {    
        //fake_cnt = -360%360;
    Uart_rx_dma_handler();
                         //  fake_cnt=   ((2147083640-2147483647)/(4096. * 26. * 4.)) * 2 * M_PI;
/*
    if (fake_cnt < 0 ){
       HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_RESET);             //motor1 DIR
    }
    else{
       HAL_GPIO_WritePin(GPIOA,GPIO_PIN_9,GPIO_PIN_SET);             //motor1 DIR    
    }
    TIM4->CCR1 = Motor_CCR_Y;
       TIM4->CCR2 = Motor_CCR_P;*/
             
  //  HAL_Delay(10);`
    
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

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode 
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion) 
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
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time. 
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_144CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 429496720;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
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
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 2249;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
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
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

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

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LD2_Pin|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10 
                          |GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin PA8 PA9 PA10 
                           PA11 */
  GPIO_InitStruct.Pin = LD2_Pin|GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10 
                          |GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
