/**
  ******************************************************************************
  * @file    main.c
  * @author  Rodrigo Rodrigues 1180659
  * @author  Luís Rocha 	   1180755
  * @version V69
  * @date    09/02/2022
  * @brief   FreeRTOS Final SOTER Project.
  ******************************************************************************
**/

/* Standard includes. */
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include <lcd.h>
#include "lcd.h"

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"

//Cores Utilizadas no programa e respetivos codigos hexadecimais
#define RED     0xF800
#define GREEN 	0x07E0
#define YELLOW  0xFFE0
#define WHITE   0xFFFF
#define BLACK 	0x0000
#define BLUE	0x001F
#define MINT    0x07EF

/* Task priorities. */
#define LEDControlPriority	( tskIDLE_PRIORITY + 1)
#define MenuPriority	( tskIDLE_PRIORITY + 1)

#define LCDPriority	( tskIDLE_PRIORITY + 3)
#define Receber_ValoresPriority	( tskIDLE_PRIORITY + 3)
#define RGBControlPriority	( tskIDLE_PRIORITY + 3)
#define CreditosPriority	( tskIDLE_PRIORITY + 3)

#define startRoutinePriority	( tskIDLE_PRIORITY + 4)
#define PrintMenuPriority	( tskIDLE_PRIORITY + 4)

/* The rate at which the flash task toggles the LED. */
#define LED_DELAY			( ( TickType_t ) 1 / portTICK_RATE_MS )


/******** Setup Functions *****/
/* Configure RCC clocks */
static void prvSetupRCC( void );

/* Configure GPIO. */
static void prvSetupGPIO( void );

/* Configure RGB IO. */
static void prvrgb_init( void );

/* Configure GPIO. */
static void prvi2c_init( void );

/* Configure GPIO. */
static void prvi2c_start( void );

/********** Useful functions **********/
/*Funções de comunicação com o Accel por I2C */
int receber_valores(char addr_msb, char addr_lsb);
char I2C_receive(char adress);

/* RGB Control Function */
void SetRGBControl(uint16_t red, uint16_t green, uint16_t blue);

/* software delays */
static void delay_ms(uint16_t delay_ms);
static void delay_us(uint32_t delay_us);

/***************************************/

/***** Program Tasks ******/
/* Start Animation */
static void prvStart_routine( void  *pvParameters);

/* LED toggle task. */
static void prvLEDControl( void *pvParameters );

/* Menu Printer Task */
static void prvPrint_Menu( void *pvParameters );

/*Main Menu Task */
static void prvMenu( void *pvParameters );

/*Mostrar Valores no LCD task */
static void prvMostrarLCD( void *pvParameters );
static void prvReceber_valores( void *pvParameters);

/*Controlo RGB task */
static void prvControlRGB( void *pvParameters );

/*Creditos task */
static void prvCreditos( void *pvParameters );


/**************************************/

/* Start Routine handle variable. */
TaskHandle_t Start_RoutineHandle;

/* Main Menu handle variable. */
TaskHandle_t Menu;

/* Menu printer handle variable. */
TaskHandle_t Print_Menu;

/* LEDControl handle variable. */
TaskHandle_t LEDControl;

/* LCD handle variable. */
TaskHandle_t LCDHandle;

/* Receber Valores handle variable. */
TaskHandle_t Receber_ValoresHandle;

/* RGB Control handle variable. */
TaskHandle_t RGBControl;

/* Creditos handle variable. */
TaskHandle_t Creditos;


QueueHandle_t Dados_Queue;
SemaphoreHandle_t PrintMenu_Sem;
SemaphoreHandle_t MenuLED_Sem;
SemaphoreHandle_t LCD_Sem;
SemaphoreHandle_t Receive_Sem;
SemaphoreHandle_t RGB_Sem;
SemaphoreHandle_t Cred_Sem;
SemaphoreHandle_t LED_Mutex;

char menustart = 0;
char menu = 0;
char pos = 1; //flag para saber posição no menu

typedef struct{
        	int16_t x;
    		int16_t y;
    		int16_t z;
        }reading;

int main( void )
{

	/*Setup the hardware, RCC, GPIO, etc...*/
    prvSetupRCC();
    prvSetupGPIO();
    prvrgb_init();
    prvi2c_init();
    prvi2c_start();
    lcd_init();

	/* Create the tasks */

    xTaskCreate( prvLEDControl, "LED", 24 , NULL, LEDControlPriority, &LEDControl );
    xTaskCreate( prvStart_routine, "Start", 56 , NULL, startRoutinePriority, &Start_RoutineHandle );
    xTaskCreate( prvPrint_Menu, "MenuPrint", configMINIMAL_STACK_SIZE, NULL, PrintMenuPriority, &Print_Menu );
    xTaskCreate( prvMenu, "Menu", configMINIMAL_STACK_SIZE, NULL, MenuPriority, &Menu );
    xTaskCreate( prvMostrarLCD, "MostrarLCD", configMINIMAL_STACK_SIZE, NULL, LCDPriority, &LCDHandle );
    xTaskCreate( prvReceber_valores	, "ReceberVals", configMINIMAL_STACK_SIZE, NULL, Receber_ValoresPriority, &Receber_ValoresHandle );
    xTaskCreate( prvControlRGB, "RGBControl", configMINIMAL_STACK_SIZE, NULL, RGBControlPriority, &RGBControl );
    xTaskCreate( prvCreditos, "Creditos", configMINIMAL_STACK_SIZE, NULL, CreditosPriority, &Creditos );

    //Queues
    Dados_Queue = xQueueCreate( 10, sizeof( reading ) );

    //Semaphores

    MenuLED_Sem = xSemaphoreCreateBinary();
    LCD_Sem = xSemaphoreCreateBinary();
    RGB_Sem = xSemaphoreCreateBinary();
    Cred_Sem = xSemaphoreCreateBinary();
    PrintMenu_Sem = xSemaphoreCreateBinary();
    Receive_Sem = xSemaphoreCreateBinary();
    LED_Mutex = xSemaphoreCreateMutex();

    vTaskSuspend(Menu);
    vTaskSuspend(Receber_ValoresHandle);

	/* Start the scheduler. */
	vTaskStartScheduler();

	/* Will only get here if there was not enough heap space to create the idle task. */
	return 0;

}


/***********************************************************/


static void prvMenu( void *pvParameters ){

		//seta
		lcd_draw_line(3,pos*10,12,pos*10, MINT);
		lcd_draw_line(9, pos*10-2,12,pos*10, MINT);
		lcd_draw_line(9, pos*10+2,12,pos*10, MINT);
		xSemaphoreGive(MenuLED_Sem);

		while (menu == 1){

			delay_ms(50);

			if(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_10) == 0){
				if(pos > 1){
					pos --;
					//seta
					lcd_draw_fillrect(0, 0 , 19, LCD_HEIGHT, BLACK);
					lcd_draw_line(3,pos*10+3,12,pos*10+3, MINT);
					lcd_draw_line(9, pos*10+1,12,pos*10+3, MINT);
					lcd_draw_line(9, pos*10+5,12,pos*10+3, MINT);
					xSemaphoreGive(MenuLED_Sem);
				}
				else{

				}
			}//sw2

			else if(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_12) == 0 ){
				if(pos < 3){
					pos++;
					//seta
					lcd_draw_fillrect(0, 0 , 19, LCD_HEIGHT, BLACK);
					lcd_draw_line(3,pos*10+3,12,pos*10+3, MINT);
					lcd_draw_line(9, pos*10+1,12,pos*10+3, MINT);
					lcd_draw_line(9, pos*10+5,12,pos*10+3, MINT);
					xSemaphoreGive(MenuLED_Sem);
				}
				else{

				}
			}//sw4

			else if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1) == 0){ //Se carregar em SW5
				menu = 0;

				switch(pos){ //Selecionar opção dependendo da flag posição(opção) correspondente
				case 1:
					xSemaphoreGive(LCD_Sem);
					xSemaphoreGive(PrintMenu_Sem);
					xSemaphoreGive(MenuLED_Sem);
					break;

				case 2:
					xSemaphoreGive(RGB_Sem);
					xSemaphoreGive(PrintMenu_Sem);
					xSemaphoreGive(MenuLED_Sem);
					break;

				case 3:
					xSemaphoreGive(Cred_Sem);
					xSemaphoreGive(PrintMenu_Sem);
					xSemaphoreGive(MenuLED_Sem);
					break;

				}//switch(pos)
				lcd_draw_fillrect(0, 0 , 19, LCD_HEIGHT, BLACK);
				lcd_draw_line(3,pos*10+3,12,pos*10+3, MINT);
				lcd_draw_line(9, pos*10+1,12,pos*10+3, MINT);
				lcd_draw_line(9, pos*10+5,12,pos*10+3, MINT);

			}//sw5

		}//while(1)
}

/*-----------------------------------------------------------*/
static void prvMostrarLCD( void *pvParameters ){


		char buffer_lcd[32];
		reading dados;

		for (;;){
			xSemaphoreTake( LCD_Sem, ( TickType_t)  portMAX_DELAY);
			vTaskResume(Receber_ValoresHandle);
			//Ciclo de limpeza do ecrã
			for(int i = 0; i<=160; i++){
				for (int j = 0; j<=128; j++){
					lcd_draw_pixel(j, i , BLACK);
				}
			}

			sprintf(buffer_lcd, "Valores Accel.");
			lcd_draw_string(10,10, (unsigned char *) buffer_lcd, MINT , 1 );
			sprintf(buffer_lcd, "SW1 Para sair");
			lcd_draw_string(10,150, (unsigned char *) buffer_lcd, RED , 1 );


			while(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13) != 0){
			xSemaphoreGive(Receive_Sem);
			xQueueReceive( Dados_Queue, &dados, ( TickType_t ) portMAX_DELAY );


			sprintf(buffer_lcd, "X : %d ", dados.x);
			lcd_draw_string(10 ,30, (unsigned char *) buffer_lcd, MINT , 1 );
			sprintf(buffer_lcd, "Y : %d ", dados.y);
			lcd_draw_string(10 ,40, (unsigned char *) buffer_lcd, MINT , 1 );
			sprintf(buffer_lcd, "Z : %d ", dados.z);
			lcd_draw_string(10 ,50, (unsigned char *) buffer_lcd, MINT , 1 );
			}

			vTaskSuspend(Receber_ValoresHandle);
			}

}

/*-----------------------------------------------------------*/
static void prvControlRGB( void *pvParameters )
{

	for( ;; ){
		xSemaphoreTake( RGB_Sem , ( TickType_t)  portMAX_DELAY);
		vTaskResume(Receber_ValoresHandle);
		char buffer_lcd[32];
		char widthx, widthy;
		reading dados;

		//Ciclo de limpeza do ecrã
		for(int i = 0; i<=160; i++){
			for (int j = 0; j<=128; j++){
				lcd_draw_pixel(j, i , BLACK);
			}
		}

		sprintf(buffer_lcd, "RGB Control");
		lcd_draw_string(10,10, (unsigned char *) buffer_lcd, MINT , 1 );

		sprintf(buffer_lcd, "SW1 Para sair");
		lcd_draw_string(10,150, (unsigned char *) buffer_lcd, RED , 1 );

		sprintf(buffer_lcd, "RED");
		lcd_draw_string(10,30, (unsigned char *) buffer_lcd, RED , 1 );
		lcd_draw_rect(10, 45, 100 , 10, RED);

		sprintf(buffer_lcd, "GREEN");
		lcd_draw_string(10,80, (unsigned char *) buffer_lcd, GREEN , 1 );
		lcd_draw_rect(10, 95, 100 , 10, GREEN);



		while(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13) != 0){

			xSemaphoreGive(Receive_Sem);
			xQueueReceive( Dados_Queue, &dados, ( TickType_t ) portMAX_DELAY );

			//Conversão dos valores de escala -1048 +1048  para 0-100
			//dados.x = receber_valores(0x01, 0x02);
			dados.x += 1048;
			dados.x = (dados.x*100)/2096;
			//dados.y = receber_valores(0x03, 0x04);
			dados.y += 1048;
			dados.y = (dados.y*100)/2096;
			delay_us(10);

			if (dados.x>100){ dados.x= 100;}
			if (dados.y>100){ dados.y= 100;}
			if (dados.x<0){ dados.x= 0;}
			if (dados.y<0){ dados.y= 0;}

			widthx = 100 -dados.x;
			widthy = 100 -dados.y;

			//Apaga na barra o que não se pinta
			lcd_draw_fillrect(dados.x+10, 46, widthx, 9, BLACK);
			lcd_draw_fillrect(dados.y+10, 96, widthy, 9, BLACK);

			lcd_draw_fillrect(10, 45, dados.x, 10, RED);
			lcd_draw_fillrect(10, 95, dados.y, 10, GREEN);

			SetRGBControl(dados.x, dados.y, 0);

		}//sw1

		//Apagar LED RGB
		SetRGBControl(0,0,0);
	}
}
/*-----------------------------------------------------------*/

static void prvCreditos( void *pvParameters ) {
	for (;;){

		xSemaphoreTake( Cred_Sem, ( TickType_t)  portMAX_DELAY);
		char buffer_lcd[32];

		//Ciclo de limpeza do ecrã
		for(int i = 0; i<=160; i++){
			for (int j = 0; j<=128; j++){
				lcd_draw_pixel(j, i , BLACK);
			}
		}

		sprintf(buffer_lcd, "Creditos");
		lcd_draw_string(20,10, (unsigned char *) buffer_lcd, YELLOW , 2 );
		sprintf(buffer_lcd, "Realizado por:");
		lcd_draw_string(20,40, (unsigned char *) buffer_lcd, WHITE , 1 );
		sprintf(buffer_lcd, "Rodrigo R.");
		lcd_draw_string(20,60, (unsigned char *) buffer_lcd, WHITE , 1 );
		sprintf(buffer_lcd, "1180659");
		lcd_draw_string(20,70, (unsigned char *) buffer_lcd, WHITE , 1 );
		sprintf(buffer_lcd, "Luis R.");
		lcd_draw_string(20,90, (unsigned char *) buffer_lcd, WHITE , 1 );
		sprintf(buffer_lcd, "1180755");
		lcd_draw_string(20,100, (unsigned char *) buffer_lcd, WHITE , 1 );

		sprintf(buffer_lcd, "SW1 Para sair");
		lcd_draw_string(10,150, (unsigned char *) buffer_lcd, RED , 1 );

		while(GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13) != 0){


		}





	}
}


/*-----------------------------------------------------------*/
static void prvLEDControl( void  *pvParameters)
{
	TickType_t xLastExecutionTime;

	xLastExecutionTime = xTaskGetTickCount();

    for( ;; )
	{
    	xSemaphoreTake( MenuLED_Sem, ( TickType_t)  portMAX_DELAY);
    	if (menu == 1){

    		switch ( pos ){
    		case 1:
    			xSemaphoreTake(LED_Mutex, ( TickType_t)  portMAX_DELAY);
    			GPIO_WriteBit(GPIOB, GPIO_Pin_0, Bit_RESET);
				GPIO_WriteBit(GPIOB, GPIO_Pin_1, Bit_RESET);
				GPIO_WriteBit(GPIOB, GPIO_Pin_2, Bit_SET);
				vTaskDelayUntil( &xLastExecutionTime, LED_DELAY );
				xSemaphoreGive(LED_Mutex);
				break;

    		case 2:
    			xSemaphoreTake(LED_Mutex, ( TickType_t)  portMAX_DELAY);
				GPIO_WriteBit(GPIOB, GPIO_Pin_0, Bit_RESET);
				GPIO_WriteBit(GPIOB, GPIO_Pin_1, Bit_SET);
				GPIO_WriteBit(GPIOB, GPIO_Pin_2, Bit_RESET);
				vTaskDelayUntil( &xLastExecutionTime, LED_DELAY );
				xSemaphoreGive(LED_Mutex);
				break;

    		case 3:
    			xSemaphoreTake(LED_Mutex, ( TickType_t)  portMAX_DELAY);
    			GPIO_WriteBit(GPIOB, GPIO_Pin_0, Bit_SET);
				GPIO_WriteBit(GPIOB, GPIO_Pin_1, Bit_RESET);
				GPIO_WriteBit(GPIOB, GPIO_Pin_2, Bit_RESET);
				vTaskDelayUntil( &xLastExecutionTime, LED_DELAY );
				xSemaphoreGive(LED_Mutex);
			    break;
    	}
    	}
    	else if( menu == 0){
    			xSemaphoreTake(LED_Mutex, ( TickType_t)  portMAX_DELAY);
				GPIO_WriteBit(GPIOB, GPIO_Pin_0, Bit_RESET);
				GPIO_WriteBit(GPIOB, GPIO_Pin_1, Bit_RESET);
				GPIO_WriteBit(GPIOB, GPIO_Pin_2, Bit_RESET);
				vTaskDelayUntil( &xLastExecutionTime, LED_DELAY );
				xSemaphoreGive(LED_Mutex);
    		}
    	}
}

/*---------------------------------------------*/
static void prvPrint_Menu( void *pvParameters ){


	for ( ;; ){
	xSemaphoreTake(PrintMenu_Sem, ( TickType_t)  portMAX_DELAY);
	menu = 1;
	//Ciclo de limpeza do ecrã
	for(int i = 128; i>=0; i--){
		for (int j = 160; j>=0; j--){
			lcd_draw_pixel(i, j , BLACK);
		}
	}

	char buffer_lcd[32];
	sprintf(buffer_lcd, "1 - Valores LCD");
	lcd_draw_string(20,10, (unsigned char *) buffer_lcd, WHITE , 1 );
	sprintf(buffer_lcd, "2 - Controlo RGB");
	lcd_draw_string(20,20, (unsigned char *) buffer_lcd, WHITE , 1 );
	sprintf(buffer_lcd, "3 - Creditos");
	lcd_draw_string(20,30, (unsigned char *) buffer_lcd, WHITE , 1 );
	sprintf(buffer_lcd, "MENU");
	lcd_draw_string(20,100, (unsigned char *) buffer_lcd, MINT , 4 );

	if(menustart == 0){
		vTaskResume(Menu);
		menustart = 1;
	}
	}
}


/*********************************************/
char I2C_receive(char adress){
	    char valor;

	    //Início da comunicação com o Slave
	    I2C_GenerateSTART(I2C2, ENABLE);
	    //Esperar ACK
	    while(!(I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT)));
	    //Transmite o endreço 0x3A ( Função de Write do sensor )
	    I2C_Send7bitAddress(I2C2,0x3A, I2C_Direction_Transmitter);
	    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));
	    //Enviar o endereço do registo que se pretende ler
	    I2C_SendData(I2C2,adress);
	    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	    //Inicia nova rotina de comunicação para recepção dos dados requeridos
	    I2C_GenerateSTART(I2C2, ENABLE);
	    //Esperar ACK
	    while(!(I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT)));
	    //Função de leitura do registo pedido anteriormente
	    I2C_Send7bitAddress(I2C2, 0x3B, I2C_Direction_Receiver);

	     // Espera recepção
	    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));

	    while (!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_RECEIVED));
	    // Recebe e guarda, finalmente, o valor pretendido
	    valor = I2C_ReceiveData(I2C2);
	    I2C_AcknowledgeConfig(I2C2, DISABLE);
	    //Termina comunicação
	    I2C_GenerateSTOP(I2C2, ENABLE);
	    while(I2C_GetFlagStatus(I2C2, I2C_FLAG_STOPF));

	    return valor;
	}


/*************************************************************/
int receber_valores(char addr_msb, char addr_lsb){

		char valor_msb; //Most significant bits
		char valor_lsb; //Least significant bits

		int16_t valor;

		valor_msb = I2C_receive(addr_msb); //recebe MSB
		valor_lsb = I2C_receive(addr_lsb); //recebe LSB
		valor = (valor_msb<<8)|(valor_lsb); //guarda MSB e LSB realizando o shift de 8 bits do MSB necessário
		valor = (valor>>4); //Shift de 4 bits de forma a organizar a palavra de 12bits (na variavel de 16)

		if(addr_msb == 0x05){ //se for Z inverte para ficar de acordo com a forma padrão de colocar a placa
			valor = valor*(-1);
		}
		return valor;
}

/*************************************************************/
static void prvReceber_valores( void *pvParameters ){

		reading dados;

		for(;;){

		xSemaphoreTake( Receive_Sem, ( TickType_t)  portMAX_DELAY);
		dados.x = receber_valores(0x01, 0x02);
		dados.y = receber_valores(0x03, 0x04);
		dados.z = receber_valores(0x05, 0x06);

		if( Dados_Queue != 0 ){
			if( xQueueSendToBack( Dados_Queue, ( void * ) &dados, ( TickType_t ) 10 ) != pdPASS ){
			/* Failed to post the message within 10 ticks. The message queue is full. */ }
		}
		}
}
/*************************************************************/

static void prvSetupRCC( void )
{
    /* RCC configuration - 72 MHz */
    ErrorStatus HSEStartUpStatus;

    RCC_DeInit();
    /*Enable the HSE*/
    RCC_HSEConfig(RCC_HSE_ON);
    /* Wait untill HSE is ready or time out */
    HSEStartUpStatus = RCC_WaitForHSEStartUp();
    if(HSEStartUpStatus == SUCCESS)
    {
        /* Enable The Prefetch Buffer */
        FLASH_PrefetchBufferCmd(FLASH_PrefetchBuffer_Enable);
        /* 72 MHZ - 2 wait states */
        FLASH_SetLatency(FLASH_Latency_2);

        /* No division HCLK = SYSCLK */
        RCC_HCLKConfig(RCC_SYSCLK_Div1);
        /* PCLK1 = HCLK/2 (36MHz) */
        RCC_PCLK1Config(RCC_HCLK_Div2);
        /* PCLK2 = HCLK (72MHz)*/
        RCC_PCLK2Config(RCC_HCLK_Div1);

        /* Use PLL with HSE=12MHz */
        RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_6);
        /* Enable the PLL *///olá
        RCC_PLLCmd(ENABLE);
        /* Wait for PLL ready */
        while (RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET );

        /* Select the PLL as system clock source */
        RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);
        /* Wait until PLL is used as system clock */
        while( RCC_GetSYSCLKSource() != 0x08 );
    }
    else
    {
        while(1);
    }
}
/*-----------------------------------------------------------*/



void prvSetupGPIO( void )
{
    /* GPIO configuration - MINT LED*/
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
   	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
   	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);


	/* SW2 GPIOC10 configuration: Input 50Mhz  */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* SW4 GPIOC12 configuration: Input 50Mhz  */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* SW1 GPIOC13 configuration: Input 50Mhz  */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	/* SW5 GPIOA1 configuration: Input 50Mhz  */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	//LEDS
	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_1;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin =  GPIO_Pin_2;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);



}
/*-----------------------------------------------------------*/


void prvrgb_init( void ){

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);

	//Configuração LED RGB
	/* GPIOB6 configuration: Output AF_PP 50 MHz */
	GPIO_InitTypeDef GPIOB6_InitStructure;
	GPIOB6_InitStructure.GPIO_Pin = GPIO_Pin_6;
	GPIOB6_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIOB6_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIOB6_InitStructure);

	/* GPIOB7 configuration: Output AF_PP 50 MHz */
	GPIO_InitTypeDef GPIOB7_InitStructure;
	GPIOB7_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIOB7_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIOB7_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIOB7_InitStructure);

	/* GPIOB8 configuration: Output AF_PP 50 MHz */
	GPIO_InitTypeDef GPIOB8_InitStructure;
	GPIOB8_InitStructure.GPIO_Pin = GPIO_Pin_8;
	GPIOB8_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIOB8_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(GPIOB, &GPIOB8_InitStructure);

	TIM_DeInit(TIM4);
	/*TIM4 Config for RGB LED*/
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TimeBaseStructure.TIM_Period = 100;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_Prescaler = 712;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);

	/*TIM4 GPIOB6 Rgb LED Config*/
	TIM_OCInitTypeDef TIM_OCInitStructure;
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OC1Init(TIM4, &TIM_OCInitStructure);

	/*TIM4 GPIOB7 rGb LED Config*/
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OC2Init(TIM4, &TIM_OCInitStructure);

	/*TIM4 GPIOB8 rgB LED Config*/
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = 0;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
	TIM_OC3Init(TIM4, &TIM_OCInitStructure);
	/* Enables the TIM4 counter */
	TIM_Cmd(TIM4, ENABLE);

}

void prvi2c_init( void ){


	RCC_AHBPeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C2, ENABLE);

	// I2C2 SCL and SDA configuration
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);


	I2C_InitTypeDef  I2C_InitStructure;
	I2C_DeInit(I2C2);
	I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
	I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
	I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
	I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
	I2C_InitStructure.I2C_ClockSpeed = 200000;
	I2C_InitStructure.I2C_OwnAddress1 = 0;
	I2C_Init(I2C2, &I2C_InitStructure);
	I2C_Cmd(I2C2, ENABLE);

}

//Inicializar comunicação I2C
void prvi2c_start( void )
{

    GPIO_WriteBit(GPIOB, GPIO_Pin_12, Bit_SET);

    I2C_GenerateSTART(I2C2, ENABLE);
    while(!(I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_MODE_SELECT)));

    I2C_Send7bitAddress(I2C2,0x3A, I2C_Direction_Transmitter);
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

    I2C_SendData(I2C2,0x2A);
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    I2C_SendData(I2C2,0x01);
    while(!I2C_CheckEvent(I2C2, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

    I2C_GenerateSTOP(I2C2, ENABLE);
    while(I2C_GetFlagStatus(I2C2, I2C_FLAG_STOPF));

}

/*-----------------------------------------------------------*/

static void prvStart_routine( void  *pvParameters){
	char buffer_lcd[32];

	for(int i = 0; i<=160; i++){
		for (int j = 0; j<=128; j++){
			lcd_draw_pixel(j, i , WHITE);
		}
	}

	sprintf(buffer_lcd, "R");
	lcd_draw_string(12, 30, (unsigned char *)buffer_lcd , RED, 4);
	delay_ms(400);
	sprintf(buffer_lcd, "G");
	lcd_draw_string(53, 30, (unsigned char *)buffer_lcd , GREEN, 4);
	delay_ms(400);
	sprintf(buffer_lcd, "B");
	lcd_draw_string(95, 30, (unsigned char *)buffer_lcd , BLUE, 4);
	delay_ms(200);
	sprintf(buffer_lcd, "CONTROL");
	lcd_draw_string(5, 63, (unsigned char *)buffer_lcd , WHITE, 3);
	delay_ms(300);

	lcd_draw_fillrect(2, 138 , 55, 11, BLACK);
	sprintf(buffer_lcd, "Accel - I2C");
	lcd_draw_string(5, 140, (unsigned char *)buffer_lcd , RED, 1);
	lcd_draw_fillrect(2, 149 , 77, 11, BLACK);
	sprintf(buffer_lcd, "FREERTOS");
	lcd_draw_string(5, 150, (unsigned char *)buffer_lcd , RED, 1);
	delay_ms(300);

	lcd_draw_fillrect(8, 98 , 110, 12, BLACK);
	sprintf(buffer_lcd, "SW5 Para continuar");
	lcd_draw_string(10, 100, (unsigned char *)buffer_lcd , RED, 1);



	while(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_1) != 0){

		// Esperar que o utilizador pressione SW5
	}

	xSemaphoreGive(PrintMenu_Sem);
	vTaskSuspend(NULL);
}

/*************************************************************/

void SetRGBControl(uint16_t red, uint16_t green, uint16_t blue){

	TIM_SetCompare1(TIM4, red); //alteração do valor Capture compare do OC1 TIM4
	TIM_SetCompare2(TIM4, green); //alteração do valor Capture compare do OC2 TIM4
	TIM_SetCompare3(TIM4, blue); //alteração do valor Capture compare do OC3 TIM4
}


void delay_us(uint32_t delay_us)
{
  volatile unsigned int num;
  volatile unsigned int t;


  for (num = 0; num < delay_us; num++)
  {
    t = 11;
    while (t != 0)
    {
      t--;
    }
  }
}

void delay_ms(uint16_t delay_ms)
{
  volatile unsigned int num;
  for (num = 0; num < delay_ms; num++)
  {
    delay_us(1000);
  }
}
































































































//1000
