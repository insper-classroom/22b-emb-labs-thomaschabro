#include <asf.h>
#include <stdio.h>

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"
#include "conf_board.h"

/***********************************************************************/
/*               defines                                               */
/***********************************************************************/

#define USART_COM_ID ID_USART1
#define USART_COM USART1

// Configurando LEDs 1, 2 e 3
// --- LED 1 ---
#define LED_1            PIOA
#define LED_1_ID         ID_PIOA
#define LED_1_IDX        0
#define LED_1_IDX_MASK   (1 << LED_1_IDX)

// Configurando botoes 1, 2 e 3
// --- BUT 1 ---
#define BUT_1            PIOD
#define BUT_1_ID         ID_PIOD
#define BUT_1_IDX        28
#define BUT_1_IDX_MASK   (1u << BUT_1_IDX)

#define TRIG             PIOD
#define TRIG_ID          ID_PIOD
#define TRIG_IDX         30
#define TRIG_IDX_MASK    (1u << TRIG_IDX)

#define ECHO             PIOA
#define ECHO_ID          ID_PIOA
#define ECHO_IDX         6
#define ECHO_IDX_MASK    (1u << ECHO_IDX)

/***********************************************************************/
/*              Variaveis globais                                      */
/***********************************************************************/
int freq = 32768;
int delta_t;
int contagem;

static void USART1_init(void);
static void configure_console(void);

char texto[25];
volatile char but_flag;
volatile char echo_flag;
static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource);

// Array utilizado para plotar o gráfico
int distancias[9] = {1, 1, 1, 1, 1, 1, 1, 1, 1};

/***********************************************************************/
/*           functions                                                 */
/***********************************************************************/
/**
 * \brief Configure the console UART.
 */
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

void but_callback(void)
{
	but_flag = 1;
}

void callback_echo(void)
{
	echo_flag = !echo_flag;
}

int altura_graf(int distancia) {
	int altura = distancia * 7 / 300;
	int resultado = 7 - altura;
	return resultado;
}

// Funcao para colocar o valor como o ultimo valor do vetor, e deslocar os outros valores
void atualiza_graf(int distancia) {
	int i;
	for (i = 0; i < 8; i++) {
		distancias[i] = distancias[i+1];
	}
	distancias[8] = distancia;
}

int io_init(void) {	
	// Iniciando botoes
	// -- BUT 1 --
	pmc_enable_periph_clk(BUT_1_ID);
	pio_configure(BUT_1, PIO_INPUT, BUT_1_IDX_MASK, PIO_PULLUP);
	pio_handler_set(BUT_1, BUT_1_ID, BUT_1_IDX_MASK, PIO_IT_FALL_EDGE, but_callback);
	
	pio_enable_interrupt(BUT_1, BUT_1_IDX_MASK);
	pio_get_interrupt_status(BUT_1);
	
	NVIC_EnableIRQ(BUT_1_ID);
	NVIC_SetPriority(BUT_1_ID, 4);
	// -- FIM --
	
	// Iniciando LED's
	// -- LED 1 --
	pmc_enable_periph_clk(LED_1_ID);
	pio_configure(LED_1, PIO_OUTPUT_1, LED_1_IDX_MASK, PIO_DEFAULT);

	// Configurar TRIG como output
	pmc_enable_periph_clk(TRIG_ID);
	pio_configure(TRIG, PIO_OUTPUT_0, TRIG_IDX_MASK, PIO_DEFAULT);

	// Configurar ECHO como inputs
	pmc_enable_periph_clk(ECHO_ID);
	pio_configure(ECHO, PIO_INPUT, ECHO_IDX_MASK, PIO_DEFAULT);
	pio_handler_set(ECHO, ECHO_ID, ECHO_IDX_MASK, PIO_IT_EDGE, callback_echo);

	pio_enable_interrupt(ECHO, ECHO_IDX_MASK);
	pio_get_interrupt_status(ECHO);

	NVIC_EnableIRQ(ECHO_ID);
	NVIC_SetPriority(ECHO_ID, 4);

	
}

/**
 *  \brief FreeRTOS Real Time Kernel example entry point.
 *
 *  \return Unused (ANSI-C compatibility).
 */
int main (void) {
	board_init();
	sysclk_init();
	delay_init();
  	configure_console();

    // Init OLED
	gfx_mono_ssd1306_init();
	
	// Inicializando
	io_init();
  
  /* Insert application code here, after the board has been initialized. */
	while(1) {
		
		// Gerando pulso de 10us no pino
		pio_set(TRIG, TRIG_IDX_MASK);
		delay_us(10);
		pio_clear(TRIG, TRIG_IDX_MASK);

		// Esperando o pino ECHO subir
		while (echo_flag == 0);

		// Iniciar RTT sem alarmes
		rtt_init(RTT, 1);

		// Esperando o pino ECHO descer
		while (echo_flag == 1);

		// Faz contagem e calcula distancia quando echo_flag desce
		contagem = rtt_read_timer_value(RTT);
		delta_t = contagem * 10e5 / freq;
		int distancia = (delta_t * 340 / 2 / 10000);

		

		// ================== DESCOMENTAR PARA VIZUALIZAR APENAS A DISTANCIA ==================
		// // Trabalhando com erros
		// if (distancia > 400) {
		// 	sprintf(texto, "fora de alcance", distancia);
		// } else if (distancia > 0 && distancia < 400) {
		// 	sprintf(texto, "%d cm", distancia);
		// } else if (distancia < 0) {
		// 	sprintf(texto, "invalido", texto);
		// }

		// // Limpando o OLED e exibindo o valor
		// // |-> limpar o oled protege contra bugs de escrita e visualização
		// gfx_mono_draw_filled_rect(0, 0, 128, 32, GFX_PIXEL_CLR);
		// gfx_mono_draw_string(texto, 0, 0, &sysfont);
		// delay_ms(80);


		// ================== DESCOMENTAR PARA VIZUALIZAR APENAS O GRAFICO ==================
		// Nomeando os eixos do grafico no OLED
		gfx_mono_draw_string("c", 0, 0, &sysfont);
		gfx_mono_draw_string("m", 0, 12, &sysfont);
		gfx_mono_draw_string("tempo", 40, 20, &sysfont);
		
		// Definindo maximos e minimos dos valores para plotar no grafico
		if (distancia > 400) {
			distancia = 400;
		} else if (distancia < 0) {
			distancia = 0;
		}
		
		// Adicionando distancia no array
		atualiza_graf(distancia);
		
		int n = altura_graf(distancias[8]);
		gfx_mono_draw_filled_rect(8, 0, 128, 10, GFX_PIXEL_CLR);

		// Desenhando o grafico
		for (int i = 8; i > -1; i--) {
			int n = altura_graf(distancias[i]);
			gfx_mono_draw_string(".", 25 + i*10, n, &sysfont);
		}
		delay_ms(100);
	}
}