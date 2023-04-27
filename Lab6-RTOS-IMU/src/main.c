#include <asf.h>
#include "conf_board.h"

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"
#include "mcu6050.h"

#include "Fusion.h"

// Import para utilizar funções sqrt e pow
#include <math.h>

/* Botao da placa */
#define BUT_PIO     PIOA
#define BUT_PIO_ID  ID_PIOA
#define BUT_PIO_PIN 11
#define BUT_PIO_PIN_MASK (1 << BUT_PIO_PIN)

/* LED da placa */
#define LED_PIO     PIOC
#define LED_PIO_ID  ID_PIOC
#define LED_PIO_IDX 8
#define LED_PIO_IDX_MASK (1 << LED_PIO_IDX)

/* LED's do OLED */
// LED 1
#define LED1_PIO			PIOA
#define LED1_PIO_ID			ID_PIOA
#define LED1_PIO_IDX		0
#define LED1_PIO_IDX_MASK (1u << LED1_PIO_IDX)

// LED 2
#define LED2_PIO           PIOC                 
#define LED2_PIO_ID        ID_PIOC              
#define LED2_PIO_IDX       30                    
#define LED2_PIO_IDX_MASK  (1u << LED2_PIO_IDX) 

// LED3
#define LED3_PIO		PIOB
#define LED3_PIO_ID		ID_PIOB
#define LED3_PIO_IDX	2
#define LED3_PIO_IDX_MASK	(1u << LED3_PIO_IDX)

/** RTOS  */
#define TASK_OLED_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_OLED_STACK_PRIORITY            (tskIDLE_PRIORITY)

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,  signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

int8_t mcu6050_i2c_bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt);
int8_t mcu6050_i2c_bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt);
static void mcu6050_i2c_bus_init(void);

/** prototypes */
void but_callback(void);
static void BUT_init(void);

enum orientacao {ESQUERDA, FRENTE, DIREITA};

/** Semaforo a ser usado pela task house down */
SemaphoreHandle_t xSemaphoreHouseDown;

/* Fila para task de orientação */
QueueHandle_t xQueueOrientation;

/************************************************************************/
/* RTOS application funcs                                               */
/************************************************************************/

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName) {
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	for (;;) {	}
}

extern void vApplicationIdleHook(void) { }

extern void vApplicationTickHook(void) { }

extern void vApplicationMallocFailedHook(void) {
	configASSERT( ( volatile void * ) NULL );
}

/************************************************************************/
/* handlers / callbacks                                                 */
/************************************************************************/

void but_callback(void) {
}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

static void task_orientacao(void *pvParameters) {
	int orientacao;

	for (;;)  {
		if (xQueueReceive(xQueueOrientation, &orientacao, 50)) {

			if (orientacao == ESQUERDA) {
			
				// Pisca LED 1
				pio_clear(LED1_PIO, LED1_PIO_IDX_MASK);
				vTaskDelay(10);
				pio_set(LED1_PIO, LED1_PIO_IDX_MASK);
			
			} else if (orientacao == FRENTE) {
			
				// Pisca LED 2
				pio_clear(LED2_PIO, LED2_PIO_IDX_MASK);
				vTaskDelay(10);
				pio_set(LED2_PIO, LED2_PIO_IDX_MASK);
			
			} else if (orientacao == DIREITA) {
			
				// Pisca LED 3
				pio_clear(LED3_PIO, LED3_PIO_IDX_MASK);
				vTaskDelay(10);
				pio_set(LED3_PIO, LED3_PIO_IDX_MASK);
			
			} else {
			
				// Apaga todos os LEDs
				pio_set(LED1_PIO, LED1_PIO_IDX_MASK);
				pio_set(LED2_PIO, LED2_PIO_IDX_MASK);
				pio_set(LED3_PIO, LED3_PIO_IDX_MASK);
			
			}

		}
	}
}

static void task_house_down(void *pvParameters) {

	for (;;)  {
		
		if (xSemaphoreTake(xSemaphoreHouseDown, 1000)) {

			for (int i = 0; i < 10; i++) {
				// Pisca o led da placa
				pio_clear(LED_PIO, LED_PIO_IDX_MASK);
				vTaskDelay(100);
				pio_set(LED_PIO, LED_PIO_IDX_MASK);
				vTaskDelay(100);
			}
		
		}
	}
}

static void task_imu(void *pvParameters) {
	int16_t  raw_acc_x, raw_acc_y, raw_acc_z;
	volatile uint8_t  raw_acc_xHigh, raw_acc_yHigh, raw_acc_zHigh;
	volatile uint8_t  raw_acc_xLow,  raw_acc_yLow,  raw_acc_zLow;
	float proc_acc_x, proc_acc_y, proc_acc_z;

	int16_t  raw_gyr_x, raw_gyr_y, raw_gyr_z;
	volatile uint8_t  raw_gyr_xHigh, raw_gyr_yHigh, raw_gyr_zHigh;
	volatile uint8_t  raw_gyr_xLow,  raw_gyr_yLow,  raw_gyr_zLow;
	float proc_gyr_x, proc_gyr_y, proc_gyr_z;
	
	mcu6050_i2c_bus_init();

	/* buffer para recebimento de dados */
	uint8_t bufferRX[10];
	uint8_t bufferTX[10];

	/* resultado da função */
	uint8_t rtn;

	// Lê registrador WHO AM I
	rtn = mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_WHO_AM_I, bufferRX, 1);
	if(rtn != TWIHS_SUCCESS){
		printf("[ERRO] [i2c] [read] \n");
	} else {
		printf("[DADO] [i2c] %x:%x", MPU6050_RA_WHO_AM_I, bufferRX[0]);
	}

	if (bufferRX[0] == 0x68) {
		printf(" - MPU6050 encontrado\n");
	} else {
		printf(" - MPU6050 nao encontrado\n");
	}

	// Set Clock source
	bufferTX[0] = MPU6050_CLOCK_PLL_XGYRO;
	rtn = mcu6050_i2c_bus_write(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, bufferTX, 1);
	if(rtn != TWIHS_SUCCESS)
		printf("[ERRO] [i2c] [write] \n");

	// Aceletromtro em 2G
	bufferTX[0] = MPU6050_ACCEL_FS_2 << MPU6050_ACONFIG_AFS_SEL_BIT; 
	rtn = mcu6050_i2c_bus_write(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_CONFIG, bufferTX, 1);
	if(rtn != TWIHS_SUCCESS)
		printf("[ERRO] [i2c] [write] \n");

	// Configura range giroscopio para operar com 250 °/s
	bufferTX[0] = 0x00; // 250 °/s
	rtn = mcu6050_i2c_bus_write(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_CONFIG, bufferTX, 1);
	if(rtn != TWIHS_SUCCESS)
		printf("[ERRO] [i2c] [write] \n");

	/* Inicializa Função de fusão */
	FusionAhrs ahrs;
	FusionAhrsInitialise(&ahrs); 

	while(1) {

		const FusionVector gyroscope = {proc_gyr_x, proc_gyr_y, proc_gyr_z}; 
		const FusionVector accelerometer = {proc_acc_x, proc_acc_y, proc_acc_z};    

		// dados em pitch roll e yaw
		const FusionEuler euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));

		// Le valor do acc X High e Low
		mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_XOUT_H, &raw_acc_xHigh, 1);
		mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_XOUT_L, &raw_acc_xLow,  1);

		// Le valor do acc y High e  Low
		mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_YOUT_H, &raw_acc_yHigh, 1);
		mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_ZOUT_L, &raw_acc_yLow,  1);

		// Le valor do acc z HIGH e Low
		mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_ZOUT_H, &raw_acc_zHigh, 1);
		mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_ACCEL_ZOUT_L, &raw_acc_zLow,  1);

		// Dados são do tipo complemento de dois
		raw_acc_x = (raw_acc_xHigh << 8) | (raw_acc_xLow << 0);
		raw_acc_y = (raw_acc_yHigh << 8) | (raw_acc_yLow << 0);
		raw_acc_z = (raw_acc_zHigh << 8) | (raw_acc_zLow << 0);

		// Le valor do gyr X High e Low
		mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_XOUT_H, &raw_gyr_xHigh, 1);
		mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_XOUT_L, &raw_gyr_xLow,  1);

		// Le valor do gyr y High e  Low
		mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_YOUT_H, &raw_gyr_yHigh, 1);
		mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_ZOUT_L, &raw_gyr_yLow,  1);

		// Le valor do gyr z HIGH e Low
		mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_ZOUT_H, &raw_gyr_zHigh, 1);
		mcu6050_i2c_bus_read(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_GYRO_ZOUT_L, &raw_gyr_zLow,  1);

		// Dados são do tipo complemento de dois
		raw_gyr_x = (raw_gyr_xHigh << 8) | (raw_gyr_xLow << 0);
		raw_gyr_y = (raw_gyr_yHigh << 8) | (raw_gyr_yLow << 0);
		raw_gyr_z = (raw_gyr_zHigh << 8) | (raw_gyr_zLow << 0);

		// Dados em escala real
		proc_acc_x = (float)raw_acc_x/16384;
		proc_acc_y = (float)raw_acc_y/16384;
		proc_acc_z = (float)raw_acc_z/16384;

		proc_gyr_x = (float)raw_gyr_x/131;
		proc_gyr_y = (float)raw_gyr_y/131;
		proc_gyr_z = (float)raw_gyr_z/131;

		// Tempo entre amostras
		float dT = 0.1;

		float modulo = sqrt(pow(proc_acc_x,2) + pow(proc_acc_y,2) + pow(proc_acc_z,2));
		if (modulo < (float)0.5) {
			xSemaphoreGive(xSemaphoreHouseDown);
		}

		// aplica o algoritmo
		FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, dT);
		
		printf("Roll %0.1f, Pitch %0.1f, Yaw %0.1f, Acc %0.1f\n", euler.angle.roll, euler.angle.pitch, euler.angle.yaw, modulo); 

		int orientacao;
		BaseType_t xHigherPriorityTaskWoken = pdTRUE;
        
		if (euler.angle.pitch > 70) {
            // FRENTE
			orientacao = FRENTE;
			xQueueSend(xQueueOrientation, &orientacao, &xHigherPriorityTaskWoken);       
		}

		else if (euler.angle.roll < -80 && (euler.angle.pitch > -10 && euler.angle.pitch < 10)) {
			// ESQUERDA
			orientacao = ESQUERDA;
			xQueueSend(xQueueOrientation, &orientacao, &xHigherPriorityTaskWoken);
		}

		else if (euler.angle.roll > 60 && (euler.angle.pitch > -10 && euler.angle.pitch < 10)) {
			// DIREITA
			orientacao = DIREITA;
			xQueueSend(xQueueOrientation, &orientacao, &xHigherPriorityTaskWoken);
		}

		// uma amostra a cada 1ms
		vTaskDelay(50);
	}

}

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/
int8_t mcu6050_i2c_bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt)
{
    int32_t ierror = 0x00;

    twihs_packet_t p_packet;
    p_packet.chip         = dev_addr;
    p_packet.addr[0]      = reg_addr;
    p_packet.addr_length  = 1;
    p_packet.buffer       = reg_data;
    p_packet.length       = cnt;

    ierror = twihs_master_write(TWIHS2, &p_packet);

    return (int8_t)ierror;
}

int8_t mcu6050_i2c_bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt)
{
    int32_t ierror = 0x00;

    twihs_packet_t p_packet;
    p_packet.chip         = dev_addr;
    p_packet.addr[0]      = reg_addr;
    p_packet.addr_length  = 1;
    p_packet.buffer       = reg_data;
    p_packet.length       = cnt;

// TODO: Algum problema no SPI faz com que devemos ler duas vezes o registrador para
//       conseguirmos pegar o valor correto.
    ierror = twihs_master_read(TWIHS2, &p_packet);

    return (int8_t)ierror;
}

void mcu6050_i2c_bus_init(void)
{
    twihs_options_t mcu6050_option;
    pmc_enable_periph_clk(ID_TWIHS2);

    /* Configure the options of TWI driver */
    mcu6050_option.master_clk = sysclk_get_cpu_hz();
    mcu6050_option.speed      = 40000;
    twihs_master_init(TWIHS2, &mcu6050_option);

	/** Enable TWIHS port to control PIO pins */
	pmc_enable_periph_clk(ID_PIOD);
	pio_set_peripheral(PIOD, PIO_TYPE_PIO_PERIPH_C, 1 << 28);
	pio_set_peripheral(PIOD, PIO_TYPE_PIO_PERIPH_C, 1 << 27);

	/** Enable TWIHS port to control PIO pins */
	pmc_enable_periph_clk(ID_PIOD);
	pio_set_peripheral(PIOD, PIO_TYPE_PIO_PERIPH_C, 1 << 28);
	pio_set_peripheral(PIOD, PIO_TYPE_PIO_PERIPH_C, 1 << 27);
}

static void configure_console(void) {
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		.charlength = CONF_UART_CHAR_LENGTH,
		.paritytype = CONF_UART_PARITY,
		.stopbits = CONF_UART_STOP_BITS,
	};

	/* Configure console UART. */
	stdio_serial_init(CONF_UART, &uart_serial_options);

	/* Specify that stdout should not be buffered. */
	setbuf(stdout, NULL);
}

static void BUT_init(void) {
	/* configura prioridae */
	NVIC_EnableIRQ(BUT_PIO_ID);
	NVIC_SetPriority(BUT_PIO_ID, 4);		
	
	// Ativa o PIO na qual o LED foi conectado
	// para que possamos controlar o LED.
	pmc_enable_periph_clk(LED1_PIO_ID);
	pmc_enable_periph_clk(LED2_PIO_ID);
	pmc_enable_periph_clk(LED3_PIO_ID);

	/* conf bot�o como entrada */
	pio_configure(BUT_PIO, PIO_INPUT, BUT_PIO_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT_PIO, BUT_PIO_PIN_MASK, 60);
	pio_enable_interrupt(BUT_PIO, BUT_PIO_PIN_MASK);
	pio_handler_set(BUT_PIO, BUT_PIO_ID, BUT_PIO_PIN_MASK, PIO_IT_FALL_EDGE , but_callback);
	
	pio_configure(LED1_PIO, PIO_OUTPUT_1, LED1_PIO_IDX_MASK, PIO_DEFAULT);
	pio_configure(LED2_PIO, PIO_OUTPUT_1, LED2_PIO_IDX_MASK, PIO_DEFAULT);
	pio_configure(LED3_PIO, PIO_OUTPUT_1, LED3_PIO_IDX_MASK, PIO_DEFAULT);
}

/************************************************************************/
/* main                                                                 */
/************************************************************************/


int main(void) {
	/* Initialize the SAM system */
	sysclk_init();
	board_init();
	BUT_init();

	/* Initialize the console uart */
	configure_console();

	// Clear OLED
	gfx_mono_ssd1306_init();
	gfx_mono_draw_filled_rect(0, 0, 128, 32, GFX_PIXEL_CLR);
	gfx_mono_draw_string("Lab 6 - RTOS - imu", 0, 5, &sysfont);

	/* Iniciar FILA */
	xQueueOrientation = xQueueCreate(100, sizeof(float));

	/* Attempt to create a semaphore. */
	xSemaphoreHouseDown = xSemaphoreCreateBinary();
	if (xSemaphoreHouseDown == NULL)
		printf("falha em criar o semaforo \n");

	/* Create task to control oled */
	if (xTaskCreate(task_house_down, "house_down", TASK_OLED_STACK_SIZE, NULL, TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
	  printf("Failed to create oled task\r\n");
	}

	if (xTaskCreate(task_imu, "imu", TASK_OLED_STACK_SIZE, NULL, TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
	  printf("Failed to create imu task\r\n");
	}

	if (xTaskCreate(task_orientacao, "orientacao", TASK_OLED_STACK_SIZE, NULL, TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
	  printf("Failed to create orientacao task\r\n");
	}

	/* Start the scheduler. */
	vTaskStartScheduler();

  /* RTOS n�o deve chegar aqui !! */
	while(1){}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
