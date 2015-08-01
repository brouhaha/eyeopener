/*
Vector generator test firmware for EFM32WG microcontroller
RetroChallenge 2015/07 project
Copyright 2015 Eric Smith <spacewar@gmail.com>

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License version 3 as
published by the Free Software Foundation.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

"System Libraries" as defined in section 1 of the GPL is construed
to include the source and/or object code of the Silicon Labs EFM32 SDK,
which is available for download by Silicon Labs "Simplicity Studio":
  http://www.silabs.com/products/mcu/Pages/simplicity-studio.aspx

For use with an updated version of the vector generator by
Steve Ciarcia and Hal Chamberlin:

  "Make Your Next Peripheral a Real Eye Opener"
  Steve Ciarcia, BYTE magazine, November 1976

  "A Graphics Display for the 8008" Part 1-3
  Hal Chamberlin, The Computer Hobbyist Volume 1 Numbers 1-3, 1974-1975

Updated vector generator schematic:
  http://www.brouhaha.com/~eric/retrocomputing/retrochallenge-2015.07/eyeopener2.pdf

This code presently drives a single channel of the
vector generator for test purposes. However, driving
two (or even three) channels requires only minor
modifications.

Uses USART1 to control Microchip MCP4822 DACs.
Uses LEUART0 serial port at 9600 bps for control.
Uses GPIO pins for DAC chip selects and to control analog
switches for vector generation.

The "0" through "9" commands (digits) are used to set endpoint
coordinates, scaled from 0 to 4095.

The "v" (vector) command draws a vector between the last two
endpoints entered.
*/

#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "em_chip.h"
#include "em_device.h"
#include "em_cmu.h"
#include "em_emu.h"
#include "em_leuart.h"
#include "em_gpio.h"
#include "em_int.h"
#include "em_usart.h"
#include "em_timer.h"


uint32_t INT_LockCnt;


uint32_t core_clk_per_us;

void delay_init(void)
{
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
	DWT->CTRL |= 1;
	core_clk_per_us = CMU_ClockFreqGet(cmuClock_CORE) / 1000000;
}

void delay_us(uint32_t us)
{
	uint32_t cnt = us * core_clk_per_us;
	DWT->CYCCNT = 0;
	while (DWT->CYCCNT < cnt)
		;
}


typedef struct
{
	GPIO_Port_TypeDef port;
	unsigned int      pin;
	GPIO_Mode_TypeDef mode;
	unsigned int      out;  // initial value of output register bit
} port_init_t;


void port_init(const port_init_t *table, unsigned int count)
{
	while(count--)
	{
		GPIO_PinModeSet(table->port, table->pin, table->mode, table->out);
		table++;
	}
}


typedef struct
{
  uint32_t size;
  uint32_t count;
  uint32_t write_idx;
  uint32_t read_idx;
  uint8_t data[0];
} buf_t;

buf_t *init_buf(uint8_t *raw_buf, uint32_t size)
{
	buf_t *buf = (buf_t *) raw_buf;
	memset(buf, 0, size);
	buf->size = size - sizeof(buf_t);
	buf->count = 0;
	buf->write_idx = 0;
	buf->read_idx = 0;
	return buf;
}

bool buf_write_byte(buf_t *buf, const uint8_t data)
{
	if (buf->count >= buf->size)
		return false;  // no room
	buf->data[buf->write_idx++] = data;
	if (buf->write_idx >= buf->size)
		buf->write_idx = 0;
	INT_Disable();
	buf->count++;
	INT_Enable();
	return true;
}


bool buf_read_byte(buf_t *buf, uint8_t *data)
{
	if (buf->count == 0)
		return false;  // no data
	*data = buf->data[buf->read_idx++];
	if (buf->read_idx >= buf->size)
		buf->read_idx = 0;
	INT_Disable();
	buf->count--;
	INT_Enable();
	return true;
}


bool buf_write_char(buf_t *buf, const char c)
{
  return buf_write_byte(buf, (uint8_t) c);
}


bool buf_read_char(buf_t *buf, char *c)
{
	return buf_read_byte(buf, (uint8_t *) c);
}


buf_t *leu_tx_buf;
buf_t *leu_rx_buf;

void LEUART0_IRQHandler(void)
{
  if (LEUART0->IF & LEUART_IF_RXDATAV)
  {
	  uint8_t data = LEUART0->RXDATA;
	  buf_write_byte(leu_rx_buf, data);
	  if (leu_rx_buf->count == 0)
		  LEUART0->IEN &= ~ LEUART_IEN_RXDATAV;  // disable rx interrupt
  }

  if (LEUART0->IF & LEUART_IF_TXBL)
  {
	  uint8_t data;
	  if (buf_read_byte(leu_tx_buf, & data))
	  {
		  while (LEUART0->SYNCBUSY)
			  ;
		  LEUART0->TXDATA = data;
	  }
	  else
		  LEUART0->IEN &= ~ LEUART_IEN_TXBL;  // disable tx interrupt
  }

  // XXX do we need to explicitly clear interrupts?
  //LEUART_IntClear(LEUART0, ???);
}


void leuart_blocking_write_char(char c)
{
	while (! buf_write_char(leu_tx_buf, c))
	{
	    //EMU_EnterEM2(true);  // wait for interrupt
	}
	LEUART0->IEN |= LEUART_IEN_TXBL;  // enable tx if not already going
}


void leuart_blocking_write_str(const char *p)
{
	while (*p)
		leuart_blocking_write_char(*p++);
}


char leuart_blocking_read_char(void)
{
	char c;
	while (! buf_read_char(leu_rx_buf, & c))
	{
	    //EMU_EnterEM2(true);  // wait for interrupt
	}
	return c;
}

int RETARGET_WriteChar(char c)
{
	leuart_blocking_write_char(c);
	return 1;
}

int RETARGET_ReadChar(void)
{
	return -1;
}


char *mfgets(char *s, int size, FILE *stream)
{
	char *p = s;
	int n = size;
	while (n)
	{
		int c = leuart_blocking_read_char();
		if (c < 0)
			break;
		*p++ = c;
		n--;
		if ((c == '\r') || (c == '\n'))
		{
			fputs("\r\n", stdout);
			break;
		}
		putchar(c);
		fflush(stdout);
	}
	if (n == 0)
		p--;
	*p = '\0';
	return s;
}


LEUART_Init_TypeDef leuart0Init =
{
  .enable   = leuartEnableRx | leuartEnableTx,
  .refFreq  = 0,
  .baudrate = 9600,
  .databits = leuartDatabits8,
  .parity   = leuartNoParity,
  .stopbits = leuartStopbits1,
};

void initLeuart(void)
{
  static uint8_t leu_raw_tx_buf[300];
  static uint8_t leu_raw_rx_buf[300];

  leu_tx_buf = init_buf(leu_raw_tx_buf, sizeof(leu_raw_tx_buf));
  leu_rx_buf = init_buf(leu_raw_rx_buf, sizeof(leu_raw_rx_buf));

  CMU_ClockEnable(cmuClock_CORELE, true);
  CMU_ClockEnable(cmuClock_GPIO, true);
  CMU_ClockEnable(cmuClock_LEUART0, true);

  LEUART_Reset(LEUART0);
  LEUART_Init(LEUART0, &leuart0Init);
  LEUART0->ROUTE = LEUART_ROUTE_RXPEN | LEUART_ROUTE_TXPEN | LEUART_ROUTE_LOCATION_LOC0;

  /* Enable GPIO for LEUART0. RX is on D5 */
  GPIO_PinModeSet(gpioPortD,            /* Port */
                  5,                    /* Port number */
                  gpioModeInputPull,    /* Pin mode is set to input only, with pull direction given bellow */
                  1);                   /* Pull direction is set to pull-up */

  /* Enable GPIO for LEUART0. TX is on D4 */
  GPIO_PinModeSet(gpioPortD,                /* GPIO port */
                  4,                        /* GPIO port number */
                  gpioModePushPull,         /* Pin mode is set to push pull */
                  1);                       /* High idle state */

  LEUART0->IEN = LEUART_IF_RXDATAV;  // enable RX interrupt
  NVIC_EnableIRQ(LEUART0_IRQn);
}


#define SPI_USART     USART1
#define SPI_USART_LOC 1
#define SPI_USART_IRQ USART1_IRQn

USART_InitSync_TypeDef spi_init =
{
	.enable      = usartEnable,
	.refFreq     = 0,
	.baudrate    = 6000000,
	.databits    = usartDatabits16,
	.master      = true,
	.msbf        = true,
	.clockMode   = usartClockMode0,
	.prsRxEnable = false,
	.prsRxCh     = usartPrsRxCh0,
	.autoTx      = false,
};

typedef enum
{
	SPI_CS1,
	SPI_CS2,
	SPI_LDAC,
	SPI_CLK,
	SPI_MOSI,
} spi_pin_t;

const port_init_t spi_pins[] =
{
	{ gpioPortD, 3, gpioModePushPull, 1 }, // *CS1
	{ gpioPortC, 4, gpioModePushPull, 1 }, // *CS2
	{ gpioPortC, 3, gpioModePushPull, 1 }, // *LDAC
	{ gpioPortD, 2, gpioModePushPull, 0 }, // CLK
	{ gpioPortD, 0, gpioModePushPull, 0 }, // MOSI
};

void initSpi(void)
{
	CMU_ClockEnable(cmuClock_GPIO, true);
	CMU_ClockEnable(cmuClock_USART1, true);

	port_init(spi_pins, sizeof(spi_pins)/sizeof(port_init_t));

	USART_InitSync(SPI_USART, & spi_init);
	SPI_USART->ROUTE = USART_ROUTE_TXPEN | USART_ROUTE_CLKPEN | (SPI_USART_LOC << _USART_ROUTE_LOCATION_SHIFT);

	//SPI_USART->CTRL |= USART_CTRL_AUTOCS;
	// NOTE: we can't use USART_ROUTE_CSPEN in the USART ROUTE register
	// and AUTO_CS in the USART CTRL register for two reasons:
	// 1. we have chip selects for two DACs
	// 2. our polled transmit using USART_SpiTransfer() doesn't keep
	//    the tx buffer full, so CS would deassert between bytes

    //NVIC_EnableIRQ(SPI_USART_IRQ);
}


uint16_t spi_xfer_16(uint16_t data)
{
	while (!(SPI_USART->STATUS & USART_STATUS_TXBL))
		;
	SPI_USART->TXDOUBLE = data;
	while (!(SPI_USART->STATUS & USART_STATUS_TXC))
	    ;
	return (uint16_t) (SPI_USART->RXDOUBLE);
}


#define DAC_GAIN_1  0x2000
#define DAC_ACTIVE  0x1000

void setDAC(int index, bool enable, uint16_t value)
{
	uint16_t d = ((index & 1) << 15) | DAC_GAIN_1 | (enable ? DAC_ACTIVE : 0) | (value & 0x0fff);

	GPIO_PinOutClear(spi_pins[index>>1].port, spi_pins[index>>1].pin);
	(void) spi_xfer_16(d);
	GPIO_PinOutSet(spi_pins[index>>1].port, spi_pins[index>>1].pin);

	GPIO_PinOutClear(spi_pins[SPI_LDAC].port, spi_pins[SPI_LDAC].pin);
	// With nothing explicitly done here for a delay, the
	// LDAC pulse width is just over a microsecond with a 48 MHz
	// core clock.
	GPIO_PinOutSet(spi_pins[SPI_LDAC].port, spi_pins[SPI_LDAC].pin);
	delay_us(10);
}


static inline int min(int a, int b)
{
	return (a < b) ? a : b;
}

static inline int max(int a, int b)
{
	return (a > b) ? a : b;
}


typedef enum
{
	T1,
	T2,
	ANALOG_SWITCH_MAX,
} analog_switch_t;

const port_init_t analog_switches[ANALOG_SWITCH_MAX] =
{
	[T1] = { gpioPortD, 6, gpioModePushPull, 0 },  // initially enabled
	[T2] = { gpioPortD, 7, gpioModePushPull, 1 }, // initially disabled
};


#define T2_TIMER     TIMER1
#define T2_TIMER_LOC 4
#define T2_TIMER_CC  1

const TIMER_Init_TypeDef t2_timer_init =
{
	.enable     = false,
	.debugRun   = false,
	.prescale   = timerPrescale1,
	.clkSel     = timerClkSelHFPerClk,
	.mode       = timerModeUp,
	.oneShot    = true,
	.sync       = false,
};

const TIMER_InitCC_TypeDef t2_timer_cc_init =
{
    .mode       = timerCCModeCompare,
	.cufoa      = timerOutputActionNone,   // underflow
    .cmoa       = timerOutputActionClear,  // match
    .cofoa      = timerOutputActionClear,  // overflow
    .coist      = true,
    .outInvert  = false,
};

#define ROUND_F_TO_I(v) (int)((v)+0.5f)

#define T1_TO_T2_DELAY 5.0e-6f   /* 5 microseconds */
#define T2_PULSE_TIME 100.0e-6f  /* 100 microseconds */

static void init_analog_switches(void)
{
  CMU_ClockEnable(cmuClock_GPIO, true);
  CMU_ClockEnable(cmuClock_TIMER1, true);

  port_init(analog_switches, sizeof(analog_switches)/sizeof(port_init_t));

  TIMER_Init(T2_TIMER, & t2_timer_init);
  TIMER_InitCC(T2_TIMER, T2_TIMER_CC, & t2_timer_cc_init);
  T2_TIMER->ROUTE = TIMER_ROUTE_CC1PEN | (T2_TIMER_LOC << _TIMER_ROUTE_LOCATION_SHIFT);
  T2_TIMER->CTRL |= TIMER_CTRL_RSSCOIST;

  uint16_t t1_to_t2_delay = ROUND_F_TO_I(T1_TO_T2_DELAY * CMU_ClockFreqGet(cmuClock_TIMER1));
  uint16_t timer_max_count = ROUND_F_TO_I((T1_TO_T2_DELAY + T2_PULSE_TIME) * CMU_ClockFreqGet(cmuClock_TIMER1));
#if 0
  printf("t1_to_t2_delay:  %" PRIu16 "\r\n", t1_to_t2_delay);
  printf("timer_max_count: %" PRIu16 "\r\n", timer_max_count);
#endif
  TIMER_CompareSet(T2_TIMER, T2_TIMER_CC, t1_to_t2_delay);
  TIMER_TopSet(T2_TIMER, timer_max_count);
}


static inline void set_analog_switch(analog_switch_t as, bool value)
{
	if (value)
		GPIO_PinOutClear(analog_switches[as].port, analog_switches[as].pin);
	else
		GPIO_PinOutSet(analog_switches[as].port, analog_switches[as].pin);
}


int main(void)
{
  uint16_t vector_start = 0;
  uint16_t vector_end = 0;

  /* Initialize chip */
  CHIP_Init();

  // start HFXO
  CMU_ClockSelectSet(cmuClock_HF, cmuSelect_HFXO);

  /* Start LFXO, and use LFXO for low-energy modules */
  CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFXO);
  CMU_ClockSelectSet(cmuClock_LFB, cmuSelect_LFXO);

  CMU_ClockEnable(cmuClock_GPIO, true);

  delay_init();

  initLeuart();

  initSpi();

#if 0
  printf("cmuClock_HF:     %" PRIu32 "\r\n", CMU_ClockFreqGet(cmuClock_HF));
  printf("cmuClock_CORE:   %" PRIu32 "\r\n", CMU_ClockFreqGet(cmuClock_CORE));
  printf("cmuClock_HFPER:  %" PRIu32 "\r\n", CMU_ClockFreqGet(cmuClock_HFPER));
  printf("cmuClock_TIMER1: %" PRIu32 "\r\n", CMU_ClockFreqGet(cmuClock_TIMER1));
#endif

  init_analog_switches();

  printf("\r\n* vector test *\r\n");

  while (1)
  {
    char c;

    c = leuart_blocking_read_char();
    switch(c)
    {
    case '0':
    case '1':
    case '2':
    case '3':
    case '4':
    case '5':
    case '6':
    case '7':
    case '8':
    case '9':
    	vector_start = vector_end;
    	vector_end = 4095 * (c - '0') / 9;
    	break;

    case 'v':
    	printf("vector from %d to %d\r\n", vector_start, vector_end);
    	setDAC(0, true, vector_start);
    	set_analog_switch(T1, false);
    	setDAC(0, true, vector_end);
    	TIMER_CounterSet(T2_TIMER, 0);
    	TIMER_Enable(T2_TIMER, true);
    	while(T2_TIMER->STATUS & TIMER_STATUS_RUNNING)
    		;
      	set_analog_switch(T1, true);
    	break;

    default:
    	break;
    }
  }
}
