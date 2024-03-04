// WCH CH32V003F4P6-R0-1v1 board
// PC1 (P2.14) ~~ LED2 (P4.2)
// PC2 (P2.13) ~~ LED1 (P4.1)

#include <stdint.h>
#include "ch32v003-drom.h"

void _start () {

  // Enable GPIOs
  RCC->APB2PCENR |= RCC_APB2Periph_GPIOC;

  // GPIO C1 Push-Pull
  GPIOC->CFGLR &= ~(0xf << (4 * (1)));
  GPIOC->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP) << (4 * (1));

  while (1) {
    GPIOC->BSHR = 1 << (1); // on
    for (uint32_t i = 0; i < (250 * 1300); i++) {
      __asm__( "nop" );
    }
    GPIOC->BSHR = 1 << (16 + (1)); // off
    for (uint32_t i = 0; i < (250 * 1300); i++) {
      __asm__( "nop" );
    }
  }
}
