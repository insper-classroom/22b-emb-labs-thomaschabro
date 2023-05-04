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

typedef struct  {
  uint32_t year;
  uint32_t month;
  uint32_t day;
  uint32_t week;
  uint32_t hour;
  uint32_t minute;
  uint32_t seccond;
} calendar;

xSemaphoreHandle xSemaphoreClock;

/************************************************************************/
/* RTOS                                                                 */
/************************************************************************/
#define TASK_OLED_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_OLED_STACK_PRIORITY            (tskIDLE_PRIORITY)

#define TASK_PRINTCONSOLE_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_PRINTCONSOLE_STACK_PRIORITY            (tskIDLE_PRIORITY)

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,  signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

/************************************************************************/
/* prototypes                                                           */
/************************************************************************/
/* Leitura do valor atual do RTC */           
uint32_t current_hour, current_min, current_sec;
uint32_t current_year, current_month, current_day, current_week;


void but_callback(void);
static void BUT_init(void);
void pin_toggle(Pio *pio, uint32_t mask);

void TC_init(Tc * TC, int ID_TC, int TC_CHANNEL, int freq); // 
void TC1_Handler(void);                                     // TC
void TC4_Handler(void);                                     //

static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource); // RTT


void RTC_init(Rtc *rtc, uint32_t id_rtc, calendar t, uint32_t irq_type); // RTC

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

	/* configura alarme do RTC para daqui 20 segundos */                                                                   
    rtc_set_date_alarm(RTC, 1, current_month, 1, current_day);                              
    rtc_set_time_alarm(RTC, 1, current_hour, 1, current_min, 1, current_sec + 5);

}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

static void task_oled(void *pvParameters) {
	BUT_init();
    
	gfx_mono_ssd1306_init();

  	/** Configura timer TC1, canal 1, com interrupcao a 4Hz */
	TC_init(TC0, ID_TC1, 1, 4);
  	tc_start(TC0, 1);
	/** Configura timer TC1, canal 1, com interrupcao a 5Hz */
	TC_init(TC1, ID_TC4, 1, 5);
  	tc_start(TC1, 1);

	/** Inicia contagem do RTT */
	RTT_init(4, 16, RTT_MR_ALMIEN);

	/** Configura RTC */
	calendar rtc_initial = {2018, 3, 19, 12, 15, 45 ,1};
	RTC_init(RTC, ID_RTC, rtc_initial, RTC_IER_ALREN | RTC_IER_SECEN);

	for (;;)
	{
		char buf[30];
		
		if(xSemaphoreTake(xSemaphoreClock, 10)){
			rtc_get_date(RTC, &current_year, &current_month, &current_day, &current_week);
			rtc_get_time(RTC, &current_hour, &current_min, &current_sec);
			gfx_mono_draw_filled_rect(0, 0, 128, 32, GFX_PIXEL_CLR);
			sprintf(buf,"%d:%d:%02d",current_hour, current_min, current_sec);
			gfx_mono_draw_string(buf, 10, 10, &sysfont);
		}
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
	
	// Iniciando botões

	// -- BUT 3 --
	pmc_enable_periph_clk(BUT_3_ID);
	pio_configure(BUT_3, PIO_INPUT, BUT_3_IDX_MASK, PIO_PULLUP);
	pio_handler_set(BUT_3, BUT_3_ID, BUT_3_IDX_MASK, PIO_IT_FALL_EDGE , but_callback);

	pio_enable_interrupt(BUT_3, BUT_3_IDX_MASK);
	pio_get_interrupt_status(BUT_3);

	NVIC_EnableIRQ(BUT_3_ID);
	NVIC_SetPriority(BUT_3_ID, 4);	

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
/* RTC                                                                  */
/************************************************************************/

void RTC_init(Rtc *rtc, uint32_t id_rtc, calendar t, uint32_t irq_type) {
	/* Configura o PMC */
	pmc_enable_periph_clk(ID_RTC);

	/* Default RTC configuration, 24-hour mode */
	rtc_set_hour_mode(rtc, 0);

	/* Configura data e hora manualmente */
	rtc_set_date(rtc, t.year, t.month, t.day, t.week);
	rtc_set_time(rtc, t.hour, t.minute, t.seccond);

	/* Configure RTC interrupts */
	NVIC_DisableIRQ(id_rtc);
	NVIC_ClearPendingIRQ(id_rtc);
	NVIC_SetPriority(id_rtc, 4);
	NVIC_EnableIRQ(id_rtc);

	/* Ativa interrupcao via alarme */
	rtc_enable_interrupt(rtc,  irq_type);
}

void RTC_Handler(void) {
    uint32_t ul_status = rtc_get_status(RTC);
		
	/* seccond tick */
    if ((ul_status & RTC_SR_SEC) == RTC_SR_SEC) {	
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		xSemaphoreGiveFromISR(xSemaphoreClock, &xHigherPriorityTaskWoken);
		rtc_clear_status(RTC, RTC_SCCR_SECCLR);
    }
	

    /* Time or date alarm */
    if ((ul_status & RTC_SR_ALARM) == RTC_SR_ALARM) {

		pin_toggle(LED_3, LED_3_IDX_MASK);
		delay_ms(100);
		pin_toggle(LED_3, LED_3_IDX_MASK);

    }

    rtc_clear_status(RTC, RTC_SCCR_SECCLR);
    rtc_clear_status(RTC, RTC_SCCR_ALRCLR);
    rtc_clear_status(RTC, RTC_SCCR_ACKCLR);
    rtc_clear_status(RTC, RTC_SCCR_TIMCLR);
    rtc_clear_status(RTC, RTC_SCCR_CALCLR);
    rtc_clear_status(RTC, RTC_SCCR_TDERRCLR);
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

	xSemaphoreClock = xSemaphoreCreateBinary();
	if (xSemaphoreClock == NULL){
		printf("falha em criar o semaforo \n");
	}

	/* Create task to control oled */
	if (xTaskCreate(task_oled, "oled", TASK_OLED_STACK_SIZE, NULL, TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
	  printf("Failed to create oled task\r\n");
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
