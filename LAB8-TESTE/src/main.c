#include <asf.h>
#include "conf_board.h"

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

// Configurando LEDs 1, 2 e 3
// --- LED P ---
#define LED_P            PIOC
#define LED_P_ID         ID_PIOC
#define LED_P_IDX        8
#define LED_P_IDX_MASK   (1 << LED_P_IDX)
// --- LED 1 ---
#define LED_1            PIOA
#define LED_1_ID         ID_PIOA
#define LED_1_IDX        0
#define LED_1_IDX_MASK   (1 << LED_1_IDX)
// --- LED 2 ---
#define LED_2            PIOC
#define LED_2_ID         ID_PIOC
#define LED_2_IDX        30
#define LED_2_IDX_MASK   (1 << LED_2_IDX)
// --- LED 3 ---
#define LED_3            PIOB
#define LED_3_ID         ID_PIOB
#define LED_3_IDX        2
#define LED_3_IDX_MASK   (1 << LED_3_IDX)
// --- BUT 3 ---
#define BUT_3            PIOA
#define BUT_3_ID         ID_PIOA
#define BUT_3_IDX        19
#define BUT_3_IDX_MASK   (1u << BUT_3_IDX)

/** RTOS  */
#define TASK_OLED_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_OLED_STACK_PRIORITY            (tskIDLE_PRIORITY)

#define TASK_PRINTCONSOLE_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_PRINTCONSOLE_STACK_PRIORITY            (tskIDLE_PRIORITY)

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,  signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

/** prototypes */
void but_callback(void);
static void BUT_init(void);
void pin_toggle(Pio *pio, uint32_t mask);

void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq); // 
void TC1_Handler(void);                                     // TC
void TC4_Handler(void);                                     //

static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource); // RTT

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

static void task_oled(void *pvParameters) {
	BUT_init();
    
	gfx_mono_ssd1306_init();
    gfx_mono_draw_string("Exemplo RTOS", 0, 0, &sysfont);
    gfx_mono_draw_string("oii", 0, 20, &sysfont);

	/** Configura timer TC1, canal 1, com interrupcao a 5Hz */
	TC_init(TC1, ID_TC4, 1, 5);
  	tc_start(TC1, 1);
  	/** Configura timer TC0, canal 1, com interrupcao a 4Hz */
	TC_init(TC0, ID_TC1, 1, 4);
  	tc_start(TC0, 1);

	/** Inicia contagem do RTT */
	RTT_init(4, 16, RTT_MR_ALMIEN);


	uint32_t cont=0;
	for (;;)
	{
		char buf[3];
		
		cont++;
		
		sprintf(buf,"%03d",cont);
		gfx_mono_draw_string(buf, 0, 20, &sysfont);
				
		vTaskDelay(1000);
	}
}


static void task_printConsole(void *pvParameters) {
	
	uint32_t cont=0;
	for (;;)
	{		
		cont++;
		
		printf("%03d\n",cont);
		
		vTaskDelay(1000);
	}
}

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

void pin_toggle(Pio *pio, uint32_t mask) {
  if(pio_get_output_data_status(pio, mask))
    pio_clear(pio, mask);
  else
    pio_set(pio,mask);
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
	// -- LED P --
	pmc_enable_periph_clk(LED_P_ID);
	pio_configure(LED_P, PIO_OUTPUT_1, LED_P_IDX_MASK, PIO_DEFAULT);

	// -- LED 1 --
	pmc_enable_periph_clk(LED_1_ID);
	pio_configure(LED_1, PIO_OUTPUT_1, LED_1_IDX_MASK, PIO_DEFAULT);

	// -- LED 2 --
	pmc_enable_periph_clk(LED_2_ID);
	pio_configure(LED_2, PIO_OUTPUT_1, LED_2_IDX_MASK, PIO_DEFAULT);

	// -- LED 3 --
	pmc_enable_periph_clk(LED_3_ID);
	pio_configure(LED_3, PIO_OUTPUT_1, LED_3_IDX_MASK, PIO_DEFAULT);
	
}

/************************************************************************/
/* TC                                                                   */
/************************************************************************/

void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq){
	uint32_t ul_div;
	uint32_t ul_tcclks;
	uint32_t ul_sysclk = sysclk_get_cpu_hz();

	/* Configura o PMC */
	pmc_enable_periph_clk(ID_TC);

	/** Configura o TC para operar em  freq hz e interrupçcão no RC compare */
	tc_find_mck_divisor(freq, ul_sysclk, &ul_div, &ul_tcclks, ul_sysclk);
	
	/** ATIVA clock canal 0 TC */
	if(ul_tcclks == 0 )
		pmc_enable_pck(PMC_PCK_6);
	
	tc_init(TC, TC_CHANNEL, ul_tcclks | TC_CMR_CPCTRG);
	tc_write_rc(TC, TC_CHANNEL, (ul_sysclk / ul_div) / freq);

	/* Configura NVIC*/
  	NVIC_SetPriority(ID_TC, 4);
	NVIC_EnableIRQ((IRQn_Type) ID_TC);
	tc_enable_interrupt(TC, TC_CHANNEL, TC_IER_CPCS);
}

void TC1_Handler(void) {
	/**
	* Devemos indicar ao TC que a interrupção foi satisfeita.
	* Isso é realizado pela leitura do status do periférico
	**/
	volatile uint32_t status = tc_get_status(TC0, 1);

	// Pisca o LED 1
	pin_toggle(LED_1, LED_1_IDX_MASK);

}

void TC4_Handler(void) {
	/**
	* Devemos indicar ao TC que a interrupção foi satisfeita.
	* Isso é realizado pela leitura do status do periférico
	**/
	volatile uint32_t status = tc_get_status(TC1, 1);

	// Pisca o LED P
	pin_toggle(LED_P, LED_P_IDX_MASK);
}

/************************************************************************/
/* RTT                                                                  */
/************************************************************************/

static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource) {

  uint16_t pllPreScale = (int) (((float) 32768) / freqPrescale);
	
  rtt_sel_source(RTT, false);
  rtt_init(RTT, pllPreScale);
  
  if (rttIRQSource & RTT_MR_ALMIEN) {
	uint32_t ul_previous_time;
  	ul_previous_time = rtt_read_timer_value(RTT);
  	while (ul_previous_time == rtt_read_timer_value(RTT));
  	rtt_write_alarm_time(RTT, IrqNPulses+ul_previous_time);
  }

  /* config NVIC */
  NVIC_DisableIRQ(RTT_IRQn);
  NVIC_ClearPendingIRQ(RTT_IRQn);
  NVIC_SetPriority(RTT_IRQn, 4);
  NVIC_EnableIRQ(RTT_IRQn);

  /* Enable RTT interrupt */
  if (rttIRQSource & (RTT_MR_RTTINCIEN | RTT_MR_ALMIEN))
	rtt_enable_interrupt(RTT, rttIRQSource);
  else
	rtt_disable_interrupt(RTT, RTT_MR_RTTINCIEN | RTT_MR_ALMIEN);
		  
}

void RTT_Handler(void) {
	uint32_t ul_status;
	ul_status = rtt_get_status(RTT);

	/* IRQ due to Alarm */
	if ((ul_status & RTT_SR_ALMS) == RTT_SR_ALMS) {
		// Pisca LED 2
		pin_toggle(LED_2, LED_2_IDX_MASK);
		RTT_init(4, 16, RTT_MR_ALMIEN);
	}  
}

/************************************************************************/
/* main                                                                 */
/************************************************************************/


int main(void) {
	/* Initialize the SAM system */
	sysclk_init();
	board_init();

	/* Initialize the console uart */
	configure_console();

	/* Create task to control oled */
	if (xTaskCreate(task_oled, "oled", TASK_OLED_STACK_SIZE, NULL, TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
	  printf("Failed to create oled task\r\n");
	}
	
	if (xTaskCreate(task_printConsole, "task_printConsole", TASK_PRINTCONSOLE_STACK_SIZE, NULL, TASK_PRINTCONSOLE_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create printConsole task\r\n");
	}

	/* Start the scheduler. */
	vTaskStartScheduler();

  /* RTOS não deve chegar aqui !! */
	while(1){
		pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
	}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
