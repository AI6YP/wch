// WCH CH32V307EVT-R1 board
// PC11 (J4.41) ~~ LED1 (J3.19)
// PC12 (J4.40) ~~ LED2 (J3.21)

#include <stdint.h>
#include "ch32v305-drom.h"

const int LED1 = 11;
const int LED2 = 12;

void _start () {

  // Enable GPIOs
  RCC->APB2PCENR |= RCC_APB2Periph_GPIOC;

  // GPIO C11 Push-Pull
  GPIOC->CFGHR &= ~(0xf << (4 * ((LED1) - 8)));
  GPIOC->CFGHR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP) << (4 * ((LED1) - 8));
  // GPIO C12 Push-Pull
  GPIOC->CFGHR &= ~(0xf << (4 * ((LED2) - 8)));
  GPIOC->CFGHR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP) << (4 * ((LED2) - 8));

  int count = 0;

  while (1) {
    GPIOC->OUTDR = (
      (((count >> 0) & 1) << (LED1)) |
      (((count >> 1) & 1) << (LED2))
    );

    for (uint32_t i = 0; i < (250 * 1300); i++) {
      __asm__( "nop" );
    }
    count += 1;
  }
}
