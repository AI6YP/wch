// WCH CH32Vx03C-R0-1v0 board
// PB9 (P2.5) ~~ LED1 (P2.1)

#include <stdint.h>
#include "ch32v20x.h"

void delay_ms (int time) {
  for (int i = 0; i < (time * 13000); i++) {
    __asm__( "nop" );
  }
}

int main () {

  GPIO_InitTypeDef GPIO_InitStructure={0};

  // Enable GPIOs
  // RCC->APB2PCENR |= RCC_APB2Periph_GPIOB;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE );


  // GPIO B5 Push-Pull
  // GPIOB->CFGLR &= ~(0xf << (4 * 5));
  // GPIOB->CFGLR |= (GPIO_Speed_10MHz | GPIO_CNF_OUT_PP) << (4 * 5);

  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);


  while (1) {
    GPIOB->BSHR = 1 << 5; // on
    delay_ms(500);
    GPIOB->BSHR = 1 << (16 + 5); // off
    delay_ms(500);
  }
  return 0;
}
