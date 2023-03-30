/***************************************************************************//**
 * @file
 * @brief Top level application functions
 *******************************************************************************
 * # License
 * <b>Copyright 2020 Silicon Laboratories Inc. www.silabs.com</b>
 *******************************************************************************
 *
 * The licensor of this software is Silicon Laboratories Inc. Your use of this
 * software is governed by the terms of Silicon Labs Master Software License
 * Agreement (MSLA) available at
 * www.silabs.com/about-us/legal/master-software-license-agreement. This
 * software is distributed to you in Source Code format and is governed by the
 * sections of the MSLA applicable to Source Code.
 *
 ******************************************************************************/
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "em_cmu.h"
#include "em_gpio.h"
#include "em_ldma.h"
#include "em_timer.h"

#define DATA_PIN (0)
#define DATA_PORT (gpioPortB)
#define BUTTON0_PIN (2)
#define BUTTON0_PORT (gpioPortB)

#define MATRIX_FREQ_HERTZ (833333ul)
#define MATRIX_RESET_SIGNAL_USEC (50u)
#define MATRIX_RESET_SIGNAL_EXTRA_CYCLES (5u)
#define MATRIX_RESET_SIGNAL_SIZE (((MATRIX_RESET_SIGNAL_USEC*MATRIX_FREQ_HERTZ)/1000000) + MATRIX_RESET_SIGNAL_EXTRA_CYCLES)
#define MATRIX_COLOR_BITS (24u)
#define MATRIX_ROW_SIZE (16u)
#define MATRIX_COLUMN_SIZE (16u)
#define MATRIX_SIZE (MATRIX_ROW_SIZE * MATRIX_COLUMN_SIZE)

#define MATRIX_DUTY_CYCLE_PERCENTAGE_BIT0 (36u)
#define MATRIX_DUTY_CYCLE_PERCENTAGE_BIT1 (68u)

static uint32_t buffer[(MATRIX_SIZE * MATRIX_COLOR_BITS) + (MATRIX_ROW_SIZE * MATRIX_COLOR_BITS)] = {0};

static LDMA_Descriptor_t matrix_ldma_desc[3] = {0}; /* the descriptor must have static lifetime */
static LDMA_TransferCfg_t matrix_transfer_config = {0};

const uint8_t gamma_lut[256] = {
     0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,   0,
     0,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   1,   2,   2,   2,   2,
     2,   2,   2,   3,   3,   3,   3,   3,   4,   4,   4,   4,   4,   5,   5,   5,
     5,   6,   6,   6,   7,   7,   7,   7,   8,   8,   8,   9,   9,  10,  10,  10,
    11,  11,  11,  12,  12,  13,  13,  13,  14,  14,  15,  15,  16,  16,  17,  17,
    18,  18,  19,  19,  20,  20,  21,  21,  22,  23,  23,  24,  24,  25,  26,  26,
    27,  28,  28,  29,  30,  30,  31,  32,  32,  33,  34,  35,  35,  36,  37,  38,
    38,  39,  40,  41,  42,  42,  43,  44,  45,  46,  47,  48,  49,  49,  50,  51,
    52,  53,  54,  55,  56,  57,  58,  59,  60,  61,  62,  63,  64,  65,  66,  67,
    69,  70,  71,  72,  73,  74,  75,  76,  78,  79,  80,  81,  82,  84,  85,  86,
    87,  89,  90,  91,  92,  94,  95,  96,  98,  99, 100, 102, 103, 104, 106, 107,
   109, 110, 112, 113, 114, 116, 117, 119, 120, 122, 123, 125, 126, 128, 130, 131,
   133, 134, 136, 138, 139, 141, 143, 144, 146, 148, 149, 151, 153, 154, 156, 158,
   160, 161, 163, 165, 167, 169, 170, 172, 174, 176, 178, 180, 182, 183, 185, 187,
   189, 191, 193, 195, 197, 199, 201, 203, 205, 207, 209, 211, 213, 215, 218, 220,
   222, 224, 226, 228, 230, 233, 235, 237, 239, 241, 244, 246, 248, 250, 253, 255,
};

static void gpio_init(void)
{
  CMU_ClockEnable(cmuClock_GPIO, true);
  GPIO_PinModeSet(DATA_PORT, DATA_PIN, gpioModePushPull, 0);
  GPIO_SlewrateSet(DATA_PORT, 7, 7);

  GPIO_PinModeSet(BUTTON0_PORT, BUTTON0_PIN, gpioModeInputPullFilter, 1);
}

static void timer_init(void)
{
  CMU_ClockEnable(cmuClock_TIMER0, true);

  TIMER_Init_TypeDef timer_init = TIMER_INIT_DEFAULT;
  timer_init.enable = false;
  TIMER_Init(TIMER0, &timer_init);

  // Route CC0 output to DATA_PIN
  GPIO->TIMERROUTE[0].ROUTEEN  = GPIO_TIMER_ROUTEEN_CC0PEN;
  GPIO->TIMERROUTE[0].CC0ROUTE = (DATA_PORT << _GPIO_TIMER_CC0ROUTE_PORT_SHIFT)
                    | (DATA_PIN << _GPIO_TIMER_CC0ROUTE_PIN_SHIFT);

  TIMER_InitCC_TypeDef cc_init = TIMER_INITCC_DEFAULT;
  cc_init.mode = timerCCModePWM;
  TIMER_InitCC(TIMER0, 0, &cc_init);

  uint32_t timer_freq = CMU_ClockFreqGet(cmuClock_TIMER0) / (timer_init.prescale + 1);
  uint32_t top_value = (timer_freq / MATRIX_FREQ_HERTZ);
  TIMER_TopSet(TIMER0, top_value);
  TIMER_CompareSet(TIMER0, 0, 0);

  TIMER_Enable(TIMER0, true);
}

static void ldma_init(void)
{
  LDMA_Init_t init = LDMA_INIT_DEFAULT;
  LDMA_Init(&init);

  matrix_transfer_config = (LDMA_TransferCfg_t)
    LDMA_TRANSFER_CFG_PERIPHERAL_LOOP(ldmaPeripheralSignal_TIMER0_CC0, MATRIX_ROW_SIZE-2);

  matrix_ldma_desc[0] = (LDMA_Descriptor_t) LDMA_DESCRIPTOR_LINKREL_M2P_BYTE(
    buffer,
    &TIMER0->CC[0].OC,
    MATRIX_ROW_SIZE * MATRIX_COLOR_BITS,
    1
  );

  matrix_ldma_desc[1] = (LDMA_Descriptor_t) LDMA_DESCRIPTOR_LINKREL_M2P_BYTE(
    0,
    &TIMER0->CC[0].OC,
    MATRIX_ROW_SIZE * MATRIX_COLOR_BITS,
    0
  );

  matrix_ldma_desc[2] = (LDMA_Descriptor_t) LDMA_DESCRIPTOR_SINGLE_M2P_BYTE(
    0,
    &TIMER0->CC[0].OC,
    MATRIX_RESET_SIGNAL_SIZE
  );

  matrix_ldma_desc[0].xfer.size = ldmaCtrlSizeWord;
  matrix_ldma_desc[0].xfer.srcAddrMode = ldmaCtrlSrcAddrModeAbs;

  matrix_ldma_desc[1].xfer.size = ldmaCtrlSizeWord;
  matrix_ldma_desc[1].xfer.srcAddrMode = ldmaCtrlSrcAddrModeRel;
  matrix_ldma_desc[1].xfer.decLoopCnt = 1;

  matrix_ldma_desc[2].xfer.size = ldmaCtrlSizeWord;
  matrix_ldma_desc[2].xfer.srcAddrMode = ldmaCtrlSrcAddrModeRel;

  // Start transfer, LDMA will trigger after first compare event
  TIMER_CounterSet(TIMER0, 0);
  TIMER_CompareSet(TIMER0, 0, 0);
  TIMER_CompareBufSet(TIMER0, 0, 0);
  LDMA_StartTransfer(0, (void*)&matrix_transfer_config, (void*)&matrix_ldma_desc);
}

static uint32_t matrix_duty_cycle(bool bit_value) {
  uint32_t top_value = TIMER_TopGet(TIMER0);
  uint32_t duty_cycle = (bit_value) ? MATRIX_DUTY_CYCLE_PERCENTAGE_BIT1 : MATRIX_DUTY_CYCLE_PERCENTAGE_BIT0;
  return (duty_cycle * top_value) / 100u;
}

static void matrix_set_pixel(uint32_t pixel, uint32_t r, uint32_t g, uint32_t b)
{
  r = gamma_lut[r];
  g = gamma_lut[g];
  b = gamma_lut[b];

  uint32_t color = ((g << 16) | (r << 8) | (b));

  for(uint32_t bit = 0; bit < MATRIX_COLOR_BITS; ++bit) {
    bool color_bit = color & (1 << (23 - bit));
    buffer[pixel * MATRIX_COLOR_BITS + bit] = matrix_duty_cycle(color_bit);
  }
}

static void matrix_color(uint32_t r, uint32_t g, uint32_t b, uint32_t start, uint32_t end)
{
  for(uint32_t pixel = start; pixel < end; ++pixel) {
    matrix_set_pixel(pixel, r, g, b);
  }
}

static void matrix_fill(uint32_t r, uint32_t g, uint32_t b)
{
  matrix_color(r, g, b, 0, MATRIX_SIZE);
}

static void matrix_scanline(uint32_t r, uint32_t g, uint32_t b)
{
  static uint32_t current_pixel = 0;
  for(uint32_t pixel = 0; pixel < MATRIX_SIZE; ++pixel) {
    if(pixel == current_pixel) {
      matrix_set_pixel(pixel, r, g, b);
    }
    else {
      matrix_set_pixel(pixel, 0, 0, 0);
    }
  }
  current_pixel = (current_pixel + 1) % MATRIX_SIZE;
}

static void matrix_rainbow()
{
  matrix_color(0xff, 0x00, 0x00, 0, (MATRIX_ROW_SIZE * 2));
  matrix_color(0xfc, 0x44, 0x44, (MATRIX_ROW_SIZE * 2), (MATRIX_ROW_SIZE * 4));
  matrix_color(0xfc, 0x64, 0x04, (MATRIX_ROW_SIZE * 4), (MATRIX_ROW_SIZE * 6));
  matrix_color(0xfc, 0xd4, 0x44, (MATRIX_ROW_SIZE * 6), (MATRIX_ROW_SIZE * 8));
  matrix_color(0x8c, 0xc4, 0x3c, (MATRIX_ROW_SIZE * 8), (MATRIX_ROW_SIZE * 10));
  matrix_color(0x02, 0x96, 0x58, (MATRIX_ROW_SIZE * 10), (MATRIX_ROW_SIZE * 12));
  matrix_color(0x1a, 0xbc, 0x9c, (MATRIX_ROW_SIZE * 12), (MATRIX_ROW_SIZE * 14));
  matrix_color(0x64, 0x54, 0xac, (MATRIX_ROW_SIZE * 14), (MATRIX_ROW_SIZE * 16));
}

void LDMA_IRQHandler(void)
{
  uint32_t flags;

  flags = LDMA_IntGet();

  LDMA_IntClear(flags);

  if (flags & LDMA_IF_ERROR) {
    while (1);
  }

  LDMA->SWREQ |= 0;
}
/***************************************************************************//**
 * Initialize application.
 ******************************************************************************/
void app_init(void)
{
  gpio_init();
  timer_init();
  matrix_fill(255, 255, 255);
  ldma_init();
}

/***************************************************************************//**
 * App ticking function.
 ******************************************************************************/
void app_process_action(void)
{
  if(!GPIO_PinInGet(BUTTON0_PORT, BUTTON0_PIN)) {
    if(LDMA_TransferDone(0)) {
        matrix_scanline(0x1F, 0, 0x1F);
        LDMA_StartTransfer(0, (void*)&matrix_transfer_config, (void*)&matrix_ldma_desc);
    }
  }
}
