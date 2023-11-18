// WCH CH32Vx03C-R0-1v0 board
// PB9 (P2.5) ~~ LED1 (P2.1)

#include <stdint.h>
#include "drom.h"

void _start () {

  // Enable GPIOs
  RCC->APB2PCENR |= RCC_APB2Periph_GPIOB;

  // GPIO B5 Push-Pull
  GPIOB->CFGLR &= ~(0xf << (4 * 5));
  GPIOB->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP) << (4 * 5);

  while (1) {
    GPIOB->BSHR = 1 << 5; // on
    for (uint32_t i = 0; i < (250 * 1300); i++) {
      __asm__( "nop" );
    }
    GPIOB->BSHR = 1 << (16 + 5); // off
    for (uint32_t i = 0; i < (250 * 1300); i++) {
      __asm__( "nop" );
    }
  }
}
