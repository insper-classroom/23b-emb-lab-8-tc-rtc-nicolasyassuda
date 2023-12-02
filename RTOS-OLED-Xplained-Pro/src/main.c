#include <asf.h>
#include "conf_board.h"

#include "gfx_mono_ug_2832hsweg04.h"
#include "gfx_mono_text.h"
#include "sysfont.h"

#define BUT_PIO           PIOD               
#define BUT_PIO_ID        ID_PIOD           
#define BUT_PIO_IDX       28                 
#define BUT_PIO_PIN_MASK (1u << BUT_PIO_IDX) 

#define LED1_PIO           PIOA                 
#define LED1_PIO_ID        ID_PIOA              
#define LED1_PIO_IDX       0                    
#define LED1_PIO_IDX_MASK  (1 << LED1_PIO_IDX)  

#define LED2_PIO           PIOC                
#define LED2_PIO_ID        ID_PIOC              
#define LED2_PIO_IDX       30                    
#define LED2_PIO_IDX_MASK  (1 << LED2_PIO_IDX)   

#define LED3_PIO           PIOB                
#define LED3_PIO_ID        ID_PIOB              
#define LED3_PIO_IDX       2                    
#define LED3_PIO_IDX_MASK  (1 << LED3_PIO_IDX)   

/** RTOS  */
#define TASK_OLED_STACK_SIZE (1024 * 6 / sizeof(portSTACK_TYPE))
#define TASK_OLED_STACK_PRIORITY (tskIDLE_PRIORITY)

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
                                          signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

/** prototypes */
void but_callback(void);
static void BUT_init(void);
typedef struct  {
	uint32_t year;
	uint32_t month;
	uint32_t day;
	uint32_t week;
	uint32_t hour;
	uint32_t minute;
	uint32_t second;
} calendar;
/************************************************************************/
/* RTOS application funcs                                               */
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
void RTC_init(Rtc *rtc, uint32_t id_rtc, calendar t, uint32_t irq_type) {
	/* Configura o PMC */
	pmc_enable_periph_clk(ID_RTC);

	/* Default RTC configuration, 24-hour mode */
	rtc_set_hour_mode(rtc, 0);

	/* Configura data e hora manualmente */
	rtc_set_date(rtc, t.year, t.month, t.day, t.week);
	rtc_set_time(rtc, t.hour, t.minute, t.second);

	/* Configure RTC interrupts */
	NVIC_DisableIRQ(id_rtc);
	NVIC_ClearPendingIRQ(id_rtc);
	NVIC_SetPriority(id_rtc, 4);
	NVIC_EnableIRQ(id_rtc);

	/* Ativa interrupcao via alarme */
	rtc_enable_interrupt(rtc,  irq_type);
}
extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
                                          signed char *pcTaskName) {
  printf("stack overflow \n");
  for (;;) {
  }
}

extern void vApplicationIdleHook(void) {}

extern void vApplicationTickHook(void) {}

extern void vApplicationMallocFailedHook(void) {
  configASSERT((volatile void *)NULL);
}

/************************************************************************/
/* handlers / callbacks                                                 */
/************************************************************************/

void but_callback(void) {
	uint32_t current_hour, current_min, current_sec;
	uint32_t current_year, current_month, current_day, current_week;
	rtc_get_time(RTC, &current_hour, &current_min, &current_sec);
	rtc_get_date(RTC, &current_year, &current_month, &current_day, &current_week);
	rtc_set_date_alarm(RTC, 1, current_month, 1, current_day);
	rtc_set_time_alarm(RTC, 1, current_hour, 1, current_min, 1, current_sec + 20);

}

/************************************************************************/
/* TASKS                                                                */
/************************************************************************/

static void task_oled(void *pvParameters) {
  gfx_mono_ssd1306_init();
	char hora[10] = "";
	uint32_t current_hour, current_min, current_sec;

  for (;;) {
	  	rtc_get_time(RTC, &current_hour, &current_min, &current_sec);
	  	sprintf(hora,"%u:%u:%u",current_hour,current_min,current_sec);
	  	gfx_mono_draw_string(hora, 0, 20, &sysfont);
	  	vTaskDelay(100);

  }
}

/************************************************************************/
/* funcoes                                                              */
/************************************************************************/
void pisca_led () {

	pio_clear(LED1_PIO, LED1_PIO_IDX_MASK);
	delay_ms(100);
	pio_set(LED1_PIO, LED1_PIO_IDX_MASK);
	
}
void pisca_led2 () {

	pio_clear(LED2_PIO, LED2_PIO_IDX_MASK);
	delay_ms(100);
	pio_set(LED2_PIO, LED2_PIO_IDX_MASK);
	
}

void TC1_Handler(void) {
	/**
	* Devemos indicar ao TC que a interrupção foi satisfeita.
	* Isso é realizado pela leitura do status do periférico
	**/
	tc_get_status(TC0, 1);

	/** Muda o estado do LED (pisca) **/
  pisca_led();
		
}

void RTC_Handler(void) {
	uint32_t ul_status = rtc_get_status(RTC);
	
	/* seccond tick */
	if ((ul_status & RTC_SR_SEC) == RTC_SR_SEC) {
		// o código para irq de segundo vem aqui
	}
	
	/* Time or date alarm */
	if ((ul_status & RTC_SR_ALARM) == RTC_SR_ALARM) {
		// o código para irq de alame vem aqui
		pisca_led2();
		pmc_sleep(SAM_PM_SMODE_SLEEP_WFI);
	}

	rtc_clear_status(RTC, RTC_SCCR_SECCLR);
	rtc_clear_status(RTC, RTC_SCCR_ALRCLR);
	rtc_clear_status(RTC, RTC_SCCR_ACKCLR);
	rtc_clear_status(RTC, RTC_SCCR_TIMCLR);
	rtc_clear_status(RTC, RTC_SCCR_CALCLR);
	rtc_clear_status(RTC, RTC_SCCR_TDERRCLR);
}
void RTT_init(float freqPrescale, uint32_t IrqNPulses, uint32_t rttIRQSource) {
	/** 
	* Configura RTT
	*
	* arg0 pllPreScale  : Frequência na qual o contador irá incrementar
	* arg1 IrqNPulses   : Valor do alarme 
	* arg2 rttIRQSource : Pode ser uma 
	*     - 0: 
	*     - RTT_MR_RTTINCIEN: Interrupção por incremento (pllPreScale)
	*     - RTT_MR_ALMIEN : Interrupção por alarme
	*/
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
void RTT_Handler(void) { // RTT interrupt handler (ISR) - this function will be called every 1 second
	rtt_get_status(RTT);
	uint32_t IrqNPulses = 20 * 4;
	RTT_init(20, IrqNPulses, RTT_MR_ALMIEN);

	if (pio_get(LED3_PIO, PIO_INPUT, LED3_PIO_IDX_MASK)) {
		pio_clear(LED3_PIO, LED3_PIO_IDX_MASK);
		} else {
		pio_set(LED3_PIO, LED3_PIO_IDX_MASK);
	}
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
  /* configura prioridae */
  NVIC_EnableIRQ(BUT_PIO_ID);
  NVIC_SetPriority(BUT_PIO_ID, 4);

  /* conf botão como entrada */
  pio_configure(BUT_PIO, PIO_INPUT, BUT_PIO_PIN_MASK,
                PIO_PULLUP | PIO_DEBOUNCE);
  pio_set_debounce_filter(BUT_PIO, BUT_PIO_PIN_MASK, 60);
  pio_enable_interrupt(BUT_PIO, BUT_PIO_PIN_MASK);
  pio_handler_set(BUT_PIO, BUT_PIO_ID, BUT_PIO_PIN_MASK, PIO_IT_FALL_EDGE,
                  but_callback);
}
static void IO_init(void){
		pio_set_output(LED1_PIO, LED1_PIO_IDX_MASK, 1, 0, 0);
		pio_set_output(LED2_PIO, LED2_PIO_IDX_MASK, 1, 0, 0);
		pio_set_output(LED3_PIO, LED3_PIO_IDX_MASK, 1, 0, 0);
}
/************************************************************************/
/* main                                                                 */
/************************************************************************/

int main(void) {
  /* Initialize the SAM system */
  sysclk_init();
  board_init();
  IO_init();
  BUT_init();
  /* Initialize the console uart */
  configure_console();
  TC_init(TC0, ID_TC1, 1, 4);
  tc_start(TC0, 1);
  calendar rtc_initial = {2023, 3, 19, 12, 15, 45 ,1};
  RTC_init(RTC, ID_RTC, rtc_initial, RTC_IER_ALREN);
  uint32_t IrqNPulses = 20 * 4;
  RTT_init(20, IrqNPulses, RTT_MR_ALMIEN);
  /* Create task to control oled */
  if (xTaskCreate(task_oled, "oled", TASK_OLED_STACK_SIZE, NULL,
                  TASK_OLED_STACK_PRIORITY, NULL) != pdPASS) {
    printf("Failed to create oled task\r\n");
  }

  /* Start the scheduler. */
  vTaskStartScheduler();

  /* RTOS não deve chegar aqui !! */
  while (1) {
  }

  /* Will only get here if there was insufficient memory to create the idle
   * task. */
  return 0;
}
