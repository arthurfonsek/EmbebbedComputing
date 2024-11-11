#include <asf.h>
#include "conf_board.h"

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"
#include "sensor.h"

/* Botao da placa */
#define LED_1_PIO PIOA
#define LED_1_PIO_ID ID_PIOA
#define LED_1_IDX 0
#define LED_1_IDX_MASK (1 << LED_1_IDX)

#define LED_2_PIO PIOC
#define LED_2_PIO_ID ID_PIOC
#define LED_2_IDX 30
#define LED_2_IDX_MASK (1 << LED_2_IDX)

#define LED_3_PIO PIOB
#define LED_3_PIO_ID ID_PIOB
#define LED_3_IDX 2
#define LED_3_IDX_MASK (1 << LED_3_IDX)

#define BUT_1_PIO PIOD
#define BUT_1_PIO_ID ID_PIOD
#define BUT_1_IDX 28
#define BUT_1_IDX_MASK (1u << BUT_1_IDX)

#define BUT_2_PIO PIOA
#define BUT_2_PIO_ID ID_PIOA
#define BUT_2_IDX 19
#define BUT_2_IDX_MASK (1u << BUT_2_IDX)

#define BUT_3_PIO PIOC
#define BUT_3_PIO_ID ID_PIOC
#define BUT_3_IDX 31
#define BUT_3_IDX_MASK (1u << BUT_3_IDX)

#define VEL_PIO PIOA
#define VEL_PIO_ID ID_PIOA
#define VEL_IDX 21
#define VEL_IDX_MASK (1u << VEL_IDX)

/** RTOS  */
#define TASK_MAIN_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_MAIN_STACK_PRIORITY            (tskIDLE_PRIORITY)

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,  signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);



QueueHandle_t xQueuePot;
QueueHandle_t xQueue2;

volatile int potencia = 0;

/** prototypes */
void io_init(void);

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
static void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource);
/************************************************************************/
/* handlers / callbacks                                                 */
/************************************************************************/

void but1_callback(void) {
  potencia++;
  if (potencia == 4) potencia = 3; 
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xQueueSendFromISR(xQueuePot, &potencia, xHigherPriorityTaskWoken);
}

void but2_callback(void) { 

}

void but3_callback(void) { 
  potencia--;
  if (potencia < 0) potencia = 0;
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  xQueueSendFromISR(xQueuePot, &potencia, xHigherPriorityTaskWoken);

}

void vel_callback(void) { 
    uint32_t ticks=rtt_read_timer_value(RTT);
    RTT_init(100,0,0);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(xQueue2, &ticks, xHigherPriorityTaskWoken);
}

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

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

static void task_main(void *pvParameters) {
	gfx_mono_ssd1306_init();

	init_sensor();
	io_init();
	int potencia=0;
	uint32_t dt=0;
	pio_set(LED_1_PIO, LED_1_IDX_MASK);
	pio_set(LED_2_PIO, LED_2_IDX_MASK);
	pio_set(LED_3_PIO, LED_3_IDX_MASK);
	RTT_init(100, 0, 0);


	for (;;)  {
    if (xQueueReceive(xQueuePot, &potencia, 100)) {
      patinete_power(potencia);
      if (potencia == 0) {
        gfx_mono_draw_filled_rect(0, 0, 128, 32, GFX_PIXEL_CLR);
        gfx_mono_draw_string("Potencia 0", 35, 0, &sysfont);
        pio_set(LED_1_PIO, LED_1_IDX_MASK);
		
      } else if (potencia == 1) {
        gfx_mono_draw_filled_rect(0, 0, 128, 32, GFX_PIXEL_CLR);
        gfx_mono_draw_string("Potencia 1", 35, 0, &sysfont);
        pio_clear(LED_1_PIO, LED_1_IDX_MASK);
        pio_set(LED_2_PIO, LED_2_IDX_MASK);
        pio_set(LED_3_PIO, LED_3_IDX_MASK);

      } else if (potencia == 2) {
        gfx_mono_draw_filled_rect(0, 0, 128, 32, GFX_PIXEL_CLR);
        gfx_mono_draw_string("Potencia 2", 35, 0, &sysfont);
        pio_clear(LED_1_PIO, LED_1_IDX_MASK);
        pio_clear(LED_2_PIO, LED_2_IDX_MASK);
        pio_set(LED_3_PIO, LED_3_IDX_MASK);
		
      } else if (potencia == 3) {
        gfx_mono_draw_filled_rect(0, 0, 128, 32, GFX_PIXEL_CLR);
        gfx_mono_draw_string("Potencia 3", 35, 0, &sysfont);
        pio_clear(LED_1_PIO, LED_1_IDX_MASK);
        pio_clear(LED_2_PIO, LED_2_IDX_MASK);
        pio_clear(LED_3_PIO, LED_3_IDX_MASK);
      }
    }
    if(xQueueReceive(xQueue2, &dt, 100)){
      float T=dt*0.01;
      float w=2*3.1415/T;
      float v=0.2*w*3.6;
      gfx_mono_draw_filled_rect(0, 16, 128, 16, GFX_PIXEL_CLR);
      char str[128];
      sprintf(str, "%.2f",v);
      gfx_mono_draw_string(str, 35, 16, &sysfont);
      gfx_mono_draw_string("Km/h", 90, 16, &sysfont);

    }
	}
}

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/



void io_init(void) {
  pmc_enable_periph_clk(LED_1_PIO_ID);
  pmc_enable_periph_clk(LED_2_PIO_ID);
  pmc_enable_periph_clk(LED_3_PIO_ID);
  pmc_enable_periph_clk(BUT_1_PIO_ID);
  pmc_enable_periph_clk(BUT_2_PIO_ID);
  pmc_enable_periph_clk(BUT_3_PIO_ID);
  pmc_enable_periph_clk(VEL_PIO_ID);

  pio_configure(LED_1_PIO, PIO_OUTPUT_0, LED_1_IDX_MASK, PIO_DEFAULT);
  pio_configure(LED_2_PIO, PIO_OUTPUT_0, LED_2_IDX_MASK, PIO_DEFAULT);
  pio_configure(LED_3_PIO, PIO_OUTPUT_0, LED_3_IDX_MASK, PIO_DEFAULT);

  pio_configure(BUT_1_PIO, PIO_INPUT, BUT_1_IDX_MASK, PIO_PULLUP| PIO_DEBOUNCE);
  pio_configure(BUT_2_PIO, PIO_INPUT, BUT_2_IDX_MASK, PIO_PULLUP| PIO_DEBOUNCE);
  pio_configure(BUT_3_PIO, PIO_INPUT, BUT_3_IDX_MASK, PIO_PULLUP| PIO_DEBOUNCE);

  pio_handler_set(BUT_1_PIO, BUT_1_PIO_ID, BUT_1_IDX_MASK, PIO_IT_FALL_EDGE,
  but1_callback);
  pio_handler_set(BUT_2_PIO, BUT_2_PIO_ID, BUT_2_IDX_MASK, PIO_IT_FALL_EDGE,
  but3_callback);
  pio_handler_set(BUT_3_PIO, BUT_3_PIO_ID, BUT_3_IDX_MASK, PIO_IT_FALL_EDGE,
  but2_callback);

  pio_enable_interrupt(BUT_1_PIO, BUT_1_IDX_MASK);
  pio_enable_interrupt(BUT_2_PIO, BUT_2_IDX_MASK);
  pio_enable_interrupt(BUT_3_PIO, BUT_3_IDX_MASK);

  pio_get_interrupt_status(BUT_1_PIO);
  pio_get_interrupt_status(BUT_2_PIO);
  pio_get_interrupt_status(BUT_3_PIO);

  NVIC_EnableIRQ(BUT_1_PIO_ID);
  NVIC_SetPriority(BUT_1_PIO_ID, 4);

  NVIC_EnableIRQ(BUT_2_PIO_ID);
  NVIC_SetPriority(BUT_2_PIO_ID, 4);

  NVIC_EnableIRQ(BUT_3_PIO_ID);
  NVIC_SetPriority(BUT_3_PIO_ID, 4);


  pio_configure(VEL_PIO, PIO_INPUT, VEL_IDX_MASK, PIO_DEFAULT);
  pio_enable_interrupt(VEL_PIO, VEL_IDX_MASK);
  pio_handler_set(VEL_PIO, VEL_PIO_ID, VEL_IDX_MASK, PIO_IT_RISE_EDGE,
                  vel_callback);
  pio_get_interrupt_status(VEL_PIO);
  NVIC_EnableIRQ(VEL_PIO_ID);
  NVIC_SetPriority(VEL_PIO_ID, 4);
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

/************************************************************************/
/* main                                                                 */
/************************************************************************/


int main(void) {
	/* Initialize the SAM system */
	sysclk_init();
	board_init();
	io_init();
	xQueuePot=xQueueCreate(32, sizeof(int));
	xQueue2=xQueueCreate(32, sizeof(int));
	/* Initialize the console uart */
	configure_console();

	/* Create task to control oled */
	if (xTaskCreate(task_main, "main", TASK_MAIN_STACK_SIZE, NULL, TASK_MAIN_STACK_PRIORITY, NULL) != pdPASS) {
	  printf("Failed to create main task\r\n");
	}

	/* Start the scheduler. */
	vTaskStartScheduler();

  /* RTOS n�o deve chegar aqui !! */
	while(1){}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
