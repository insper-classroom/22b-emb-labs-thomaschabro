#include <asf.h>
#include <stdio.h>

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

/***********************************************************************/
/*               defines                                               */
/***********************************************************************/

// Configurando LEDs 1, 2 e 3
// --- LED 1 ---
#define LED_1            PIOA
#define LED_1_ID         ID_PIOA
#define LED_1_IDX        0
#define LED_1_IDX_MASK   (1 << LED_1_IDX)
// --- LED 2 ---
#define LED_2            PIOC
#define LED_2_ID         ID_PIOC
#define LED_2_IDX        8
#define LED_2_IDX_MASK   (1 << LED_2_IDX)
// --- LED 3 ---
#define LED_3            PIOB
#define LED_3_ID         ID_PIOB
#define LED_3_IDX        2
#define LED_3_IDX_MASK   (1 << LED_3_IDX)

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

/***********************************************************************/
/*              Variáveis globais                                      */
/***********************************************************************/
int n = 30;
int tempo = 200;

char texto[25];
volatile char but_flag;
volatile char but_flag2;
volatile char but_flag3;


/***********************************************************************/
/*           functions                                                 */
/***********************************************************************/

void pisca_led(int n, int t){	
	int j = 70;
	for (int i=0;i<n;i++){
		for (int x = j; x  < (j + 10);x++) {
			gfx_mono_draw_rect(x - 20, 5, 2, 10, GFX_PIXEL_SET);
		}
		pio_clear(LED_2, LED_2_IDX_MASK);
		delay_ms(t);
		pio_set(LED_2, LED_2_IDX_MASK);
		delay_ms(t);
		
		if (but_flag2) {
			break;
		}
		j+=2;
	}
}


void but_callback(void)
{
	but_flag = 1;
}

void but_callback2(void) {
	but_flag2 = 1;
}

void but_callback3(void) {
	but_flag3 = 1;
}

int io_init(void) {	
	// Iniciando botões
	// -- BUT 1 --
	pmc_enable_periph_clk(BUT_1_ID);
	pio_configure(BUT_1, PIO_INPUT, BUT_1_IDX_MASK, PIO_PULLUP);
	pio_handler_set(BUT_1, BUT_1_ID, BUT_1_IDX_MASK, PIO_IT_FALL_EDGE, but_callback);
	
	pio_enable_interrupt(BUT_1, BUT_1_IDX_MASK);
	pio_get_interrupt_status(BUT_1);
	
	NVIC_EnableIRQ(BUT_1_ID);
	NVIC_SetPriority(BUT_1_ID, 4);
	// -- BUT 2 --
	pmc_enable_periph_clk(BUT_2_ID);
	pio_configure(BUT_2, PIO_INPUT, BUT_2_IDX_MASK, PIO_PULLUP);
	pio_handler_set(BUT_2, BUT_2_ID, BUT_2_IDX_MASK, PIO_IT_FALL_EDGE, but_callback2);
		
	pio_enable_interrupt(BUT_2, BUT_2_IDX_MASK);
	pio_get_interrupt_status(BUT_2);
		
	NVIC_EnableIRQ(BUT_2_ID);
	NVIC_SetPriority(BUT_2_ID, 4);
	// -- BUT 3 --
	pmc_enable_periph_clk(BUT_3_ID);
	pio_configure(BUT_3, PIO_INPUT, BUT_3_IDX_MASK, PIO_PULLUP);
	pio_handler_set(BUT_3, BUT_3_ID, BUT_3_IDX_MASK, PIO_IT_FALL_EDGE, but_callback3);
	
	pio_enable_interrupt(BUT_3, BUT_3_IDX_MASK);
	pio_get_interrupt_status(BUT_3);
	
	NVIC_EnableIRQ(BUT_3_ID);
	NVIC_SetPriority(BUT_3_ID, 4);	

	// -- FIM --
	
	// Iniciando LED's
	// -- LED 1 --
	pmc_enable_periph_clk(LED_1_ID);
	pio_configure(LED_1, PIO_OUTPUT_1, LED_1_IDX_MASK, PIO_DEFAULT);

}

int main (void) {
	board_init();
	sysclk_init();
	delay_init();

    // Init OLED
	gfx_mono_ssd1306_init();
	
	// Inicializando
	io_init();
  
  
	gfx_mono_draw_filled_circle(20, 16, 16, GFX_PIXEL_SET, GFX_WHOLE);
   
 

  /* Insert application code here, after the board has been initialized. */
	while(1) {
			sprintf(texto, "%d ms", tempo);
			gfx_mono_draw_string(texto, 50,16, &sysfont);
						
			for(int i=120;i>=50;i-=2){

				gfx_mono_draw_rect(i, 5, 2, 10, GFX_PIXEL_CLR);
				delay_ms(10);

			}
						
			// Handling buttons
			
			if (but_flag) {	
				delay_ms(1000);
				if (pio_get(BUT_1, PIO_INPUT, BUT_1_IDX_MASK)) {
					tempo += 100;
					} else {
					if (tempo == 100) {
						tempo == 100;
						} else {
						tempo -= 100;
					}
				}
				pisca_led(n, tempo);
				but_flag = 0;
			}
			
			if (but_flag2) {
				but_flag2 = 0;
				delay_ms(200);
			}
			
			if (but_flag3) {
				tempo += 100;
				pisca_led(n, tempo);
				but_flag3 = 0;
			}
			
			
			
	}
}