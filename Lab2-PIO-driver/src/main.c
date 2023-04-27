/**
 * ==== LAB 2 ====
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
#define LED_3            PIOC
#define LED_3_ID         ID_PIOC
#define LED_3_IDX        8
#define LED_3_IDX_MASK   (1 << LED_3_IDX)

/*  Default pin configuration (no attribute). */
#define _PIO_DEFAULT             (0u << 0)
/*  The internal pin pull-up is active. */
#define _PIO_PULLUP              (1u << 0)
/*  The internal glitch filter is active. */
#define _PIO_DEGLITCH            (1u << 1)
/*  The internal debouncing filter is active. */
#define _PIO_DEBOUNCE            (1u << 3)

/**
 * \brief Set a high output level on all the PIOs defined in ul_mask.
 * This has no immediate effects on PIOs that are not output, but the PIO
 * controller will save the value if they are changed to outputs.
 *
 * \param p_pio Pointer to a PIO instance.
 * \param ul_mask Bitmask of one or more pin(s) to configure.
 */
void _pio_set(Pio *p_pio, const uint32_t ul_mask)
{
	p_pio->PIO_SODR = ul_mask;
}

/**
 * \brief Set a low output level on all the PIOs defined in ul_mask.
 * This has no immediate effects on PIOs that are not output, but the PIO
 * controller will save the value if they are changed to outputs.
 *
 * \param p_pio Pointer to a PIO instance.
 * \param ul_mask Bitmask of one or more pin(s) to configure.
 */
void _pio_clear(Pio *p_pio, const uint32_t ul_mask)
{
	p_pio->PIO_CODR = ul_mask;
}

/**
 * \brief Configure PIO internal pull-up.
 *
 * \param p_pio Pointer to a PIO instance.
 * \param ul_mask Bitmask of one or more pin(s) to configure.
 * \param ul_pull_up_enable Indicates if the pin(s) internal pull-up shall be
 * configured.
 */
void _pio_pull_up(Pio *p_pio, const uint32_t ul_mask, const uint32_t ul_pull_up_enable)
{
	if (ul_pull_up_enable) {
		p_pio->PIO_PUER = ul_mask;
	} else {
		p_pio->PIO_PUDR = ul_mask;
	}
}

/**
 * \brief Configure one or more pin(s) or a PIO controller as inputs.
 * Optionally, the corresponding internal pull-up(s) and glitch filter(s) can
 * be enabled.
 *
 * \param p_pio Pointer to a PIO instance.
 * \param ul_mask Bitmask indicating which pin(s) to configure as input(s).
 * \param ul_attribute PIO attribute(s).
 */
void _pio_set_input(Pio *p_pio, const uint32_t ul_mask,
const uint32_t ul_attribute)
{
	pio_disable_interrupt(p_pio, ul_mask);
	pio_pull_up(p_pio, ul_mask, ul_attribute & PIO_PULLUP);
	
	if (ul_attribute & (PIO_DEGLITCH | PIO_DEBOUNCE)) {
		p_pio->PIO_IFER = ul_mask;
		} else {
		p_pio->PIO_IFDR = ul_mask;
	}
	
	p_pio->PIO_ODR = ul_mask;
	p_pio->PIO_PER = ul_mask;

}

/**
 * \brief Configure one or more pin(s) of a PIO controller as outputs, with
 * the given default value. Optionally, the multi-drive feature can be enabled
 * on the pin(s).
 *
 * \param p_pio Pointer to a PIO instance.
 * \param ul_mask Bitmask indicating which pin(s) to configure.
 * \param ul_default_level Default level on the pin(s).
 * \param ul_multidrive_enable Indicates if the pin(s) shall be configured as
 * open-drain.
 * \param ul_pull_up_enable Indicates if the pin shall have its pull-up
 * activated.
 */
void _pio_set_output(Pio *p_pio, const uint32_t ul_mask,
        const uint32_t ul_default_level,
        const uint32_t ul_multidrive_enable,
        const uint32_t ul_pull_up_enable)
{
	// Configurar o PIO para controlar o pino 
	p_pio->PIO_PER = ul_mask;
	p_pio->PIO_OER = ul_mask;
	
	// Definir saída inicial do pino
	_pio_pull_up(p_pio, ul_mask, ul_pull_up_enable);
	
	// Ativar ou não o multidrive
	if (ul_multidrive_enable) {
		p_pio->PIO_MDER = ul_mask;
	} else {
		p_pio->PIO_MDDR = ul_mask;
	}
	
	if (ul_default_level) {
		p_pio->PIO_SODR = ul_mask;
		} else {
		p_pio->PIO_CODR = ul_mask;
	}
}

/**
 * \brief Return 1 if one or more PIOs of the given Pin instance currently have
 * a high level; otherwise returns 0. This method returns the actual value that
 * is being read on the pin. To return the supposed output value of a pin, use
 * pio_get_output_data_status() instead.
 *
 * \param p_pio Pointer to a PIO instance.
 * \param ul_type PIO type.
 * \param ul_mask Bitmask of one or more pin(s) to configure.
 *
 * \retval 1 at least one PIO currently has a high level.
 * \retval 0 all PIOs have a low level.
 */
uint32_t _pio_get(Pio *p_pio, const pio_type_t ul_type,
        const uint32_t ul_mask)
{
	uint32_t x;

	if ((ul_type == PIO_OUTPUT_0) || (ul_type == PIO_OUTPUT_1)) {
		x = p_pio->PIO_ODSR;
		} else {
		x = p_pio->PIO_PDSR;
	}

	if ((x & ul_mask) == 0) {
		return 0;
		} else {
		return 1;
	}
}


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
	_pio_set_input(BUT_1, BUT_1_IDX_MASK, _PIO_PULLUP | _PIO_DEBOUNCE);
	// -- BUT 2 --
	pmc_enable_periph_clk(BUT_2_ID);
	_pio_set_input(BUT_2, BUT_2_IDX_MASK, _PIO_PULLUP | _PIO_DEBOUNCE);
	// -- BUT 3 --
	pmc_enable_periph_clk(BUT_3_ID);
	_pio_set_input(BUT_3, BUT_3_IDX_MASK, _PIO_PULLUP | _PIO_DEBOUNCE);
	
	// Iniciando LED's
	// -- LED 1 --
	pmc_enable_periph_clk(LED_1_ID);
	_pio_set_output(LED_1, LED_1_IDX_MASK, 1, 0, 0);
	// -- LED 2 --
	pmc_enable_periph_clk(LED_2_ID);
	_pio_set_output(LED_2, LED_2_IDX_MASK, 1, 0, 0);
	// -- LED 3 --
	pmc_enable_periph_clk(LED_3_ID);
	_pio_set_output(LED_3, LED_3_IDX_MASK, 1, 0, 0);
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
	if (_pio_get(BUT_1, PIO_INPUT, BUT_1_IDX_MASK) == 0) {
		for (int i = 0; i < 5; i++) {
			_pio_clear(LED_1, LED_1_IDX_MASK);
			delay_ms(100);
			_pio_set(LED_1, LED_1_IDX_MASK);
			delay_ms(100);
		}
	}	
	if (_pio_get(BUT_2, PIO_INPUT, BUT_2_IDX_MASK) == 0) {
		for (int i = 0; i < 5; i++) {
			_pio_clear(LED_2, LED_2_IDX_MASK);
			delay_ms(100);
			_pio_set(LED_2, LED_2_IDX_MASK);
			delay_ms(100);
		}
	}
	if (_pio_get(BUT_3, PIO_INPUT, BUT_3_IDX_MASK) == 0) {
		for (int i = 0; i < 5; i++) {
			_pio_clear(LED_3, LED_3_IDX_MASK);
			delay_ms(100);
			_pio_set(LED_3, LED_3_IDX_MASK);
			delay_ms(100);
		}
	}

  }
  
  return 0;
}
