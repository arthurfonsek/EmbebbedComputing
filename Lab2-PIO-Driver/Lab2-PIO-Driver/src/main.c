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

//LEDS

// # (1)

#define LED_PIO			PIOA
#define LED_PIO_ID		ID_PIOA
#define LED_PIO_IDX		0
#define LED_PIO_IDX_MASK (1 << LED_PIO_IDX)

// # (2)
#define LED2_PIO           PIOC                 // periferico que controla o LED
#define LED2_PIO_ID        ID_PIOC                 // ID do periférico PIOC (controla LED)
#define LED2_PIO_IDX       30                    // ID do LED no PIO
#define LED2_PIO_IDX_MASK  (1 << LED2_PIO_IDX)   // Mascara para CONTROLARMOS o LED

// # (3)

#define LED3_PIO		PIOB
#define LED3_PIO_ID		ID_PIOB
#define LED3_PIO_IDX	2
#define LED3_PIO_IDX_MASK	(1 << LED3_PIO_IDX)

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

void _pio_set_input(Pio *p_pio, const uint32_t ul_mask,	const uint32_t ul_attribute)
{
	_pio_pull_up(p_pio, ul_mask, ul_attribute);
	if (ul_attribute == PIO_DEBOUNCE){
		p_pio -> PIO_IFSCER = ul_mask;
	}
	if (ul_attribute == PIO_DEGLITCH){
		p_pio -> PIO_IFSCDR = ul_mask;
	}
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

void _pio_set_output(Pio *p_pio, const uint32_t ul_mask, const uint32_t ul_default_level, const uint32_t ul_multidrive_enable, const uint32_t ul_pull_up_enable)
{
	p_pio ->PIO_PER;
	p_pio ->PIO_OER;
	
	if(ul_default_level){
		_pio_set(p_pio, ul_mask);
	}
	else{
		_pio_clear(p_pio, ul_mask);
	}
	
	if (ul_multidrive_enable){
		p_pio -> PIO_MDER = ul_mask;
	}
	else{
		p_pio ->PIO_MDDR = ul_mask;
	}
	
	_pio_pull_up(p_pio, ul_mask, ul_pull_up_enable);
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
uint32_t _pio_get(Pio *p_pio, const pio_type_t ul_type, const uint32_t ul_mask)
{
	uint32_t ul_status;
	
	if(ul_type == PIO_OUTPUT_0){
		ul_status = p_pio -> PIO_ODSR;
	}
	else{
		ul_status = p_pio -> PIO_PDSR;
	}
	
	//BITMASK PARA VER SE TEM UM 0
	if(ul_status & ul_mask){
		return 1;
	}
	else{
		return 0;
	}
}

void _delay_ms(uint32_t milisegundo){
	int loop_unico = 138000;
	int loops = loop_unico*milisegundo;
	while (loops > 0){
		loops -=1;
		asm("nop");
	}
}


// Função de inicialização do uC
void init(void)
{
	//Initialize the board cloack
	sysclk_init();
	
	//desativa watchdog timer
	WDT->WDT_MR = WDT_MR_WDDIS;
	
	// Ativa o PIO na qual o LED foi conectado
	// para que possamos controlar o LED.
	pmc_enable_periph_clk(LED_PIO_ID);
	pmc_enable_periph_clk(LED2_PIO_ID);
	pmc_enable_periph_clk(LED3_PIO_ID);
	
	//Inicializa PC8 como saída
	pio_set_output(LED_PIO, LED_PIO_IDX_MASK, 1, 0, 0);
	pio_set_output(LED2_PIO, LED2_PIO_IDX_MASK, 1, 0, 0);
	pio_set_output(LED3_PIO, LED3_PIO_IDX_MASK, 1, 0, 0);
	
	pmc_enable_periph_clk(BTN1_PIO_ID);
	pmc_enable_periph_clk(BTN2_PIO_ID);
	pmc_enable_periph_clk(BTN3_PIO_ID);
	
	_pio_set_input(BTN1_PIO, BTN1_PIO_IDX_MASK, _PIO_PULLUP | _PIO_DEBOUNCE);
	_pio_set_input(BTN2_PIO, BTN2_PIO_IDX_MASK, _PIO_PULLUP | _PIO_DEBOUNCE);
	_pio_set_input(BTN3_PIO, BTN3_PIO_IDX_MASK, _PIO_PULLUP | _PIO_DEBOUNCE);
}

  
/************************************************************************/
/* Main                                                                 */
/************************************************************************/

// Funcao principal chamada na inicalizacao do uC.
int main(void)
{
  init();

  // super loop
  // aplicacoes embarcadas não devem sair do while(1).
   while (1)
   {
		int i = 0;
		if (_pio_get(BTN1_PIO, PIO_INPUT, BTN1_PIO_IDX_MASK) == 0){
			while (i < 5){
				_pio_clear(LED_PIO, LED_PIO_IDX_MASK);
				_delay_ms(800);
				_pio_set(LED_PIO, LED_PIO_IDX_MASK);
				_delay_ms(300);
				i++;
			}
		}
		if (_pio_get(BTN2_PIO, PIO_INPUT, BTN2_PIO_IDX_MASK) == 0){
			while (i < 5){
				_pio_clear(LED2_PIO, LED2_PIO_IDX_MASK);
				delay_ms(800);
				_pio_set(LED2_PIO, LED2_PIO_IDX_MASK);
				delay_ms(300);
				i++;
			}
		}
		if (_pio_get(BTN3_PIO, PIO_INPUT, BTN3_PIO_IDX_MASK) == 0){
			while (i < 5){
				_pio_clear(LED3_PIO, LED3_PIO_IDX_MASK);
				delay_ms(800);
				_pio_set(LED3_PIO, LED3_PIO_IDX_MASK);
				delay_ms(300);
				i++;
			}
		}
	}
	return 0;
}
