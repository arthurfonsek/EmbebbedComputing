#include <asf.h>

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

/************************************************************************/
/* defines                                                              */
/************************************************************************/

//LEDS

// # (1)

#define LED_PIO			PIOA
#define LED_PIO_ID		ID_PIOA
#define LED_PIO_IDX		0
#define LED_PIO_IDX_MASK (1 << LED_PIO_IDX)

// # (2)
#define LED2_PIO           PIOC                 // periferico que controla o LED
#define LED2_PIO_ID        ID_PIOC                 // ID do perif�rico PIOC (controla LED)
#define LED2_PIO_IDX       30                    // ID do LED no PIO
#define LED2_PIO_IDX_MASK  (1 << LED2_PIO_IDX)   // Mascara para CONTROLARMOS o LED

// # (3)

#define LED3_PIO		PIOB
#define LED3_PIO_ID		ID_PIOB
#define LED3_PIO_IDX	2
#define LED3_PIO_IDX_MASK	(1 << LED3_PIO_IDX)

#define LED4_PIO		PIOC
#define LED4_PIO_ID		ID_PIOC
#define LED4_PIO_IDX	8
#define LED4_PIO_IDX_MASK	(1 << LED4_PIO_IDX)

//BOTOES

// # (1)
#define BTN1_PIO	PIOD
#define BTN1_PIO_ID		ID_PIOD
#define BTN1_PIO_IDX	28
#define BTN1_PIO_IDX_MASK (1u << BTN1_PIO_IDX)

// # (2)
#define BTN2_PIO	PIOC
#define BTN2_PIO_ID	ID_PIOC
#define BTN2_PIO_IDX	31
#define BTN2_PIO_IDX_MASK (1u << BTN2_PIO_IDX)

// # (3)
#define BTN3_PIO	PIOA
#define BTN3_PIO_ID	ID_PIOA
#define BTN3_PIO_IDX	19
#define BTN3_PIO_IDX_MASK (1u << BTN3_PIO_IDX)

#define BTN4_PIO	PIOA
#define BTN4_PIO_ID	ID_PIOA
#define BTN4_PIO_IDX	11
#define BTN4_PIO_IDX_MASK (1u << BTN4_PIO_IDX)

// # DEFINE (PIO_SET_INPUT)

/*  Default pin configuration (no attribute). */
#define _PIO_DEFAULT             (0u << 0)
/*  The internal pin pull-up is active. */
#define _PIO_PULLUP              (1u << 0)
/*  The internal glitch filter is active. */
#define _PIO_DEGLITCH            (1u << 1)
/*  The internal debouncing filter is active. */
#define _PIO_DEBOUNCE            (1u << 3)

/************************************************************************/
/* constants                                                            */
/************************************************************************/

/************************************************************************/
/* variaveis globais                                                    */
/************************************************************************/
volatile char but_flag;
volatile char but_flag2;
volatile char but_flag3;
int tempo = 0;
char txt[20];

/************************************************************************/
/* prototype                                                            */
/************************************************************************/
void io_init(void);
void pisca_led(int n, int t);

/************************************************************************/
/* handler / callbacks                                                  */
/************************************************************************/

/*
 * Exemplo de callback para o botao, sempre que acontecer
 * ira piscar o led por 5 vezes
 *
 * !! Isso � um exemplo ruim, nao deve ser feito na pratica, !!
 * !! pois nao se deve usar delays dentro de interrupcoes    !!
 */
void but_callback(void)
{
  but_flag = 1;
}

void but_callback2(void){
	but_flag2 = 1;
}

void but_callback3(void){
	but_flag3 = 1;
}

/************************************************************************/
/* fun��es                                                              */
/************************************************************************/

void io_init(void)
{

// Configura led
pmc_enable_periph_clk(LED_PIO_ID);
pio_configure(LED_PIO, PIO_OUTPUT_0, LED_PIO_IDX_MASK, PIO_DEFAULT);

// Inicializa clock do periférico PIO responsavel pelo botao
pmc_enable_periph_clk(BTN1_PIO_ID);
pmc_enable_periph_clk(BTN2_PIO_ID);
pmc_enable_periph_clk(BTN3_PIO_ID);
pio_set(LED_PIO, LED_PIO_IDX_MASK );

// Configura PIO para lidar com o pino do botão como entrada
// com pull-up
pio_configure(BTN1_PIO, PIO_INPUT, BTN1_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
pio_configure(BTN2_PIO, PIO_INPUT, BTN2_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);
pio_configure(BTN3_PIO, PIO_INPUT, BTN3_PIO_IDX_MASK, PIO_PULLUP | PIO_DEBOUNCE);

// Configura interrupção no pino referente ao botao e associa
// função de callback caso uma interrupção for gerada
// a função de callback é a: but_callback()
pio_handler_set(BTN1_PIO,
BTN1_PIO_ID,
BTN1_PIO_IDX_MASK,
PIO_IT_FALL_EDGE,
but_callback);

pio_handler_set(BTN2_PIO,
BTN2_PIO_ID,
BTN2_PIO_IDX_MASK,
PIO_IT_FALL_EDGE,
but_callback2);

pio_handler_set(BTN2_PIO,
BTN3_PIO_ID,
BTN3_PIO_IDX_MASK,
PIO_IT_FALL_EDGE,
but_callback3);

// Ativa interrupção e limpa primeira IRQ gerada na ativacao
pio_enable_interrupt(BTN1_PIO, BTN1_PIO_IDX_MASK);
pio_enable_interrupt(BTN2_PIO, BTN2_PIO_IDX_MASK);
pio_enable_interrupt(BTN3_PIO, BTN3_PIO_IDX_MASK);
pio_get_interrupt_status(BTN1_PIO);
pio_get_interrupt_status(BTN2_PIO);
pio_get_interrupt_status(BTN3_PIO);

// Configura NVIC para receber interrupcoes do PIO do botao
// com prioridade 4 (quanto mais próximo de 0 maior)
NVIC_EnableIRQ(BTN1_PIO_ID);
NVIC_EnableIRQ(BTN2_PIO_ID);
NVIC_EnableIRQ(BTN3_PIO_ID);

NVIC_SetPriority(BTN1_PIO_ID, 4);
NVIC_SetPriority(BTN2_PIO_ID, 4);
NVIC_SetPriority(BTN3_PIO_ID, 4); // Prioridade 4
}


// pisca led N vez no periodo T
void pisca_led(int n, int t){
  for (int i=0;i<n;i++){
    pio_clear(LED_PIO, LED_PIO_IDX_MASK);
    delay_ms(t);
    pio_set(LED_PIO, LED_PIO_IDX_MASK);
    delay_ms(t);
	if (but_flag2){
		but_flag2 = 0;
		break;
	}
  }
}


int main (void)
{
	io_init();
	board_init();
	sysclk_init();
	delay_init();

	tempo = 200;

  // Init OLED
 	gfx_mono_ssd1306_init();
	sprintf(txt, "%.2f hertz", (1000.0/(tempo+tempo)));

  /* Insert application code here, after the board has been initialized. */
	while(1) {
			if (but_flag){
				delay_ms(300);
				if (pio_get(BTN1_PIO, PIO_INPUT, BTN1_PIO_IDX_MASK)){
					if (tempo > 100){
						tempo -=100;
					}
					else{
						tempo = 200;
					}
				}
				else{
					tempo += 100;
				}
				
				sprintf(txt, "%.2f hertz", (1000.0/(tempo+tempo)));
				gfx_mono_draw_string(txt, 20, 16, &sysfont);
				pisca_led(8, tempo);
				but_flag = 0;
				pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
			}
			if (but_flag3){
				sprintf(txt, "%.2f hertz", (1000.0/(tempo+tempo)));
				gfx_mono_draw_string(txt, 20, 16, &sysfont);
				tempo+=100;
				pisca_led(8, tempo);
				but_flag3 = 0;
			}
				

			// Escreve na tela um circulo e um texto
			
			for(int i=70;i<=120;i+=2){
				
				gfx_mono_draw_rect(i, 5, 2, 10, GFX_PIXEL_SET);
				delay_ms(10);
				
			}
			
			for(int i=120;i>=70;i-=2){
				
				gfx_mono_draw_rect(i, 5, 2, 10, GFX_PIXEL_CLR);
				delay_ms(10);
				
			}
			
			
	}
}
