/**
 * 5 semestre - Eng. da Computação - Insper
 * Rafael Corsi - rafael.corsi@insper.edu.br
 *
 * Projeto 0 para a placa SAME70-XPLD
 *
 * Objetivo :
 *  - Introduzir ASF e HAL
 *  - Configuracao de clock
 *  - Configuracao pino In/Out
 *
 * Material :
 *  - Kit: ATMEL SAME70-XPLD - ARM CORTEX M7
 */

/************************************************************************/
/* includes                                                             */
/************************************************************************/

#include "asf.h"

/************************************************************************/
/* defines                                                              */
/************************************************************************/

//#define LED_PIO           PIOC                 // periferico que controla o LED
// # (1)
//#define LED_PIO_ID        ID_PIOC              // ID do periférico PIOC (controla LED)
//#define LED_PIO_IDX       8                    // ID do LED no PIO
//#define LED_PIO_IDX_MASK  (1 << LED_PIO_IDX)   // Mascara para CONTROLARMOS o LED

// Configuracoes do botao
//#define BUT_PIO          PIOA
//#define BUT_PIO_ID	   ID_PIOA
//#define BUT_PIO_IDX      11
//#define BUT_PIO_IDX_MASK (1u << BUT_PIO_IDX) // esse já está pronto.

// Configurando botões 1, 2 e 3
// --- BUT 1 ---
#define BUT_1            PIOD
#define BUT_1_ID         ID_PIOD
#define BUT_1_IDX        28
#define BUT_1_IDX_MASK   (1u << BUT_1_IDX)          
// --- BUT 2 ---
#define BUT_2            PIOC
#define BUT_2_ID         ID_PIOC
#define BUT_2_IDX        31
#define BUT_2_IDX_MASK   (1u << BUT_2_IDX)
// --- BUT 3 ---
#define BUT_3            PIOA
#define BUT_3_ID         ID_PIOA
#define BUT_3_IDX        19
#define BUT_3_IDX_MASK   (1u << BUT_3_IDX)


// Configurando LEDs 1, 2 e 3
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

/************************************************************************/
/* constants                                                            */
/************************************************************************/

/************************************************************************/
/* variaveis globais                                                    */
/************************************************************************/

/************************************************************************/
/* prototypes                                                           */
/************************************************************************/

void init(void);

/************************************************************************/
/* interrupcoes                                                         */
/************************************************************************/

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

// Função de inicialização do uC
void init(void)
{
	// Initialize the board clock
	sysclk_init();
	
	// Desativa WatchDog Timer
	WDT->WDT_CR = WDT_MR_WDDIS;
	
	// Ativa o PIO na qual o LED foi conectado
	// para que possamos controlar o LED.
	//pmc_enable_periph_clk(LED_PIO_ID);
	
	// Inicializa o PIO do botão
	//pmc_enable_periph_clk(BUT_PIO_ID);
	
	// Inicializa o pino PC8 como saída
	//pio_set_output(LED_PIO, LED_PIO_IDX_MASK, 1, 0, 0);
	
	// Configura o pino ligado ao botão como entrada com um pull-up.
	//pio_set_input(BUT_PIO, BUT_PIO_IDX_MASK, PIO_DEFAULT);
	//pio_pull_up(BUT_PIO, BUT_PIO_IDX_MASK, 1);
	
	// --- EXERCICIO ENTEGAVEL ---
	
	// Iniciando botões
	// -- BUT 1 --
	pmc_enable_periph_clk(BUT_1_ID);
	pio_set_input(BUT_1, BUT_1_IDX_MASK, PIO_DEFAULT);
	pio_pull_up(BUT_1, BUT_1_IDX_MASK, 1);
	// -- BUT 2 --
	pmc_enable_periph_clk(BUT_2_ID);
	pio_set_input(BUT_2, BUT_2_IDX_MASK, PIO_DEFAULT);
	pio_pull_up(BUT_2, BUT_2_IDX_MASK, 1);
	// -- BUT 3 --
	pmc_enable_periph_clk(BUT_3_ID);
	pio_set_input(BUT_3, BUT_3_IDX_MASK, PIO_DEFAULT);
	pio_pull_up(BUT_3, BUT_3_IDX_MASK, 1);
	
	// Iniciando LED's
	// -- LED 1 --
	pmc_enable_periph_clk(LED_1_ID);
	pio_set_output(LED_1, LED_1_IDX_MASK, 1, 0, 0);
	// -- LED 2 --
	pmc_enable_periph_clk(LED_2_ID);
	pio_set_output(LED_2, LED_2_IDX_MASK, 1, 0, 0);
	// -- LED 3 --
	pmc_enable_periph_clk(LED_3_ID);
	pio_set_output(LED_3, LED_3_IDX_MASK, 1, 0, 0);
}

/************************************************************************/
/* Main                                                                 */
/************************************************************************/

// Funcao principal chamada na inicalizacao do uC.
int main(void) {
  // Inicializa sistema e IOs
  init();

  // super loop
  // aplicacoes embarcadas não devem sair do while(1).
  while (1)
  {	
	// Verifica valor lido pelo input
	//if(pio_get(BUT_PIO, PIO_INPUT, BUT_PIO_IDX_MASK) == 0) {
	//	for (int i =0; i < 5; i++) {
	//		pio_clear(LED_PIO, LED_PIO_IDX_MASK);
	//		delay_ms(100);
	//		pio_set(LED_PIO, LED_PIO_IDX_MASK);
	//		delay_ms(100);
	//	}
	
	// Ligando LED 1
	//pio_clear(LED)
	//}
	if (pio_get(BUT_1, PIO_INPUT, BUT_1_IDX_MASK) == 0) {
		for (int i = 0; i < 5; i++) {
			pio_clear(LED_1, LED_1_IDX_MASK);
			delay_ms(100);
			pio_set(LED_1, LED_1_IDX_MASK);
			delay_ms(100);
		}
	}	
	if (pio_get(BUT_2, PIO_INPUT, BUT_2_IDX_MASK) == 0) {
		for (int i = 0; i < 5; i++) {
			pio_clear(LED_2, LED_2_IDX_MASK);
			delay_ms(100);
			pio_set(LED_2, LED_2_IDX_MASK);
			delay_ms(100);
		}
	}
	if (pio_get(BUT_3, PIO_INPUT, BUT_3_IDX_MASK) == 0) {
		for (int i = 0; i < 5; i++) {
			pio_clear(LED_3, LED_3_IDX_MASK);
			delay_ms(100);
			pio_set(LED_3, LED_3_IDX_MASK);
			delay_ms(100);
		}
	}

  }
  
  return 0;
}
