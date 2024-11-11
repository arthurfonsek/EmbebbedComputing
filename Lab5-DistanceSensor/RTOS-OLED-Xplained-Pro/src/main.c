#include <asf.h>
#include "conf_board.h"

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

/* Botao da placa */
#define BUT_PIO     PIOA
#define BUT_PIO_ID  ID_PIOA
#define BUT_PIO_PIN 11
#define BUT_PIO_PIN_MASK (1 << BUT_PIO_PIN)

//trigger
#define PIO_TRIGGER     PIOA
#define PIO_TRIGGER_ID  ID_PIOA
#define PIO_TRIGGER_IDX 4
#define PIO_TRIGGER_IDX_MASK (1 << PIO_TRIGGER_IDX)

//echo
#define PIO_ECHO     PIOB
#define PIO_ECHO_ID  ID_PIOB
#define PIO_ECHO_IDX 4
#define PIO_ECHO_IDX_MASK (1 << PIO_ECHO_IDX)

//led
#define OLED_WIDTH 128
#define OLED_HEIGHT 32

/** RTOS  */
#define TASK_OLED_STACK_SIZE                (1024*6/sizeof(portSTACK_TYPE))
#define TASK_OLED_STACK_PRIORITY            (tskIDLE_PRIORITY)

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,  signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

/** prototypes */
void but_callback(void);
static void BUT_init(void);
void echo_callback(void);

/*QUEUES, TASKS & SEMAPHORES*/
QueueHandle_t xQueueEcho;

/*FLAGS*/
volatile char echo_flag;

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
void echo_callback(void){
	int tempo = rtt_read_timer_value(RTT);
	int dist = (tempo*1000000)/32768;
	dist = dist *170/10000;
	BaseType_t xHigherPriorityTaskWoken = pdTRUE;
	xQueueSendFromISR(xQueueEcho, &dist, &xHigherPriorityTaskWoken);
}

void but_callback(void) {
	pio_set(PIO_TRIGGER, PIO_TRIGGER_IDX_MASK);
	delay_us(10);
	pio_clear(PIO_TRIGGER, PIO_TRIGGER_IDX_MASK);
	rtt_init(RTT,1);
}
/************************************************************************/
/* TASKS                                                                */
/************************************************************************/
static void task_oled(void *pvParameters) {
	char tempo_str[20];
	gfx_mono_ssd1306_init();
  	gfx_mono_draw_string("inicio", 0, 0, &sysfont);
	int tempo;
	
	while(1)  { 
		if (xQueueReceive(xQueueEcho, &(tempo), 100)){
			gfx_mono_draw_filled_rect(0,0, OLED_WIDTH, OLED_HEIGHT,GFX_PIXEL_CLR);
			sprintf(tempo_str, "   %d cm    ", tempo);
			gfx_mono_draw_string(tempo_str, 0,16, &sysfont);
		}
	}
}

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/

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

static void init(void) {
	//desativa WatchDog
	WDT ->WDT_MR = WDT_MR_WDDIS;
	
	
	/* conf bot�o como entrada */
	pmc_enable_periph_clk(BUT_PIO_ID);
	pio_configure(BUT_PIO, PIO_INPUT, BUT_PIO_PIN_MASK, PIO_PULLUP | PIO_DEBOUNCE);
	pio_set_debounce_filter(BUT_PIO, BUT_PIO_PIN_MASK, 60);
	pio_enable_interrupt(BUT_PIO, BUT_PIO_PIN_MASK);
	pio_get_interrupt_status(BUT_PIO_ID);
	pio_handler_set(BUT_PIO, BUT_PIO_ID, 
					BUT_PIO_PIN_MASK, 
					PIO_IT_FALL_EDGE , 
					but_callback);

	/* configura prioridade */
	NVIC_EnableIRQ(BUT_PIO_ID);
	NVIC_SetPriority(BUT_PIO_ID, 4);

	//ECHO CONFIGURE
	pmc_enable_periph_clk(PIO_ECHO_ID);
	pio_configure(PIO_ECHO, PIO_INPUT, PIO_ECHO_IDX_MASK, PIO_DEFAULT);
	pio_set_debounce_filter(PIO_ECHO, PIO_ECHO_IDX_MASK, 60);
	pio_enable_interrupt(PIO_ECHO, PIO_ECHO_IDX_MASK);
	pio_get_interrupt_status(PIO_ECHO_ID);
	pio_handler_set(PIO_ECHO, 
					PIO_ECHO_ID, 
					PIO_ECHO_IDX_MASK, 
					PIO_IT_FALL_EDGE , 
					echo_callback);
	
	/* configura prioridade */
	NVIC_EnableIRQ(PIO_ECHO_ID);
	NVIC_SetPriority(PIO_ECHO_ID, 4);

	/*TRIGGER CONFIGURE*/
	pio_configure(PIO_TRIGGER, PIO_OUTPUT_0, PIO_TRIGGER_IDX_MASK, PIO_DEFAULT);
	pio_set_debounce_filter(PIO_TRIGGER, PIO_TRIGGER_IDX_MASK, 60);

}

/************************************************************************/
/* main                                                                 */
/************************************************************************/
int main(void) {
	/* Initialize the SAM system */
	sysclk_init();
	board_init();
	configure_console();
	init();
	
	xQueueEcho = xQueueCreate(100, sizeof(int));
	if (xQueueEcho == NULL)
		printf("falha em criar a queue xQueueADC \n");
	/* Create task to control oled */
	
	if (xTaskCreate(task_oled, "oled", TASK_OLED_STACK_SIZE, NULL, TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
	   printf("Failed to create oled task\r\n");
	}
	/* Start the scheduler. */
	vTaskStartScheduler();

	while(1){
	}

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
