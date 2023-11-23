#ifndef __CH32V20x_DROM_H
#define __CH32V20x_DROM_H

#include <stdint.h>



/* Analog to Digital Converter */
typedef struct
{
    volatile uint32_t STATR;
    volatile uint32_t CTLR1;
    volatile uint32_t CTLR2;
    volatile uint32_t SAMPTR1;
    volatile uint32_t SAMPTR2;
    volatile uint32_t IOFR1;
    volatile uint32_t IOFR2;
    volatile uint32_t IOFR3;
    volatile uint32_t IOFR4;
    volatile uint32_t WDHTR;
    volatile uint32_t WDLTR;
    volatile uint32_t RSQR1;
    volatile uint32_t RSQR2;
    volatile uint32_t RSQR3;
    volatile uint32_t ISQR;
    volatile uint32_t IDATAR1;
    volatile uint32_t IDATAR2;
    volatile uint32_t IDATAR3;
    volatile uint32_t IDATAR4;
    volatile uint32_t RDATAR;
} ADC_TypeDef;

/* Backup Registers */
typedef struct
{
    uint32_t      RESERVED0;
    volatile uint16_t DATAR1;
    uint16_t      RESERVED1;
    volatile uint16_t DATAR2;
    uint16_t      RESERVED2;
    volatile uint16_t DATAR3;
    uint16_t      RESERVED3;
    volatile uint16_t DATAR4;
    uint16_t      RESERVED4;
    volatile uint16_t DATAR5;
    uint16_t      RESERVED5;
    volatile uint16_t DATAR6;
    uint16_t      RESERVED6;
    volatile uint16_t DATAR7;
    uint16_t      RESERVED7;
    volatile uint16_t DATAR8;
    uint16_t      RESERVED8;
    volatile uint16_t DATAR9;
    uint16_t      RESERVED9;
    volatile uint16_t DATAR10;
    uint16_t      RESERVED10;
    volatile uint16_t OCTLR;
    uint16_t      RESERVED11;
    volatile uint16_t TPCTLR;
    uint16_t      RESERVED12;
    volatile uint16_t TPCSR;
    uint16_t      RESERVED13[5];
    volatile uint16_t DATAR11;
    uint16_t      RESERVED14;
    volatile uint16_t DATAR12;
    uint16_t      RESERVED15;
    volatile uint16_t DATAR13;
    uint16_t      RESERVED16;
    volatile uint16_t DATAR14;
    uint16_t      RESERVED17;
    volatile uint16_t DATAR15;
    uint16_t      RESERVED18;
    volatile uint16_t DATAR16;
    uint16_t      RESERVED19;
    volatile uint16_t DATAR17;
    uint16_t      RESERVED20;
    volatile uint16_t DATAR18;
    uint16_t      RESERVED21;
    volatile uint16_t DATAR19;
    uint16_t      RESERVED22;
    volatile uint16_t DATAR20;
    uint16_t      RESERVED23;
    volatile uint16_t DATAR21;
    uint16_t      RESERVED24;
    volatile uint16_t DATAR22;
    uint16_t      RESERVED25;
    volatile uint16_t DATAR23;
    uint16_t      RESERVED26;
    volatile uint16_t DATAR24;
    uint16_t      RESERVED27;
    volatile uint16_t DATAR25;
    uint16_t      RESERVED28;
    volatile uint16_t DATAR26;
    uint16_t      RESERVED29;
    volatile uint16_t DATAR27;
    uint16_t      RESERVED30;
    volatile uint16_t DATAR28;
    uint16_t      RESERVED31;
    volatile uint16_t DATAR29;
    uint16_t      RESERVED32;
    volatile uint16_t DATAR30;
    uint16_t      RESERVED33;
    volatile uint16_t DATAR31;
    uint16_t      RESERVED34;
    volatile uint16_t DATAR32;
    uint16_t      RESERVED35;
    volatile uint16_t DATAR33;
    uint16_t      RESERVED36;
    volatile uint16_t DATAR34;
    uint16_t      RESERVED37;
    volatile uint16_t DATAR35;
    uint16_t      RESERVED38;
    volatile uint16_t DATAR36;
    uint16_t      RESERVED39;
    volatile uint16_t DATAR37;
    uint16_t      RESERVED40;
    volatile uint16_t DATAR38;
    uint16_t      RESERVED41;
    volatile uint16_t DATAR39;
    uint16_t      RESERVED42;
    volatile uint16_t DATAR40;
    uint16_t      RESERVED43;
    volatile uint16_t DATAR41;
    uint16_t      RESERVED44;
    volatile uint16_t DATAR42;
    uint16_t      RESERVED45;
} BKP_TypeDef;

/* Controller Area Network TxMailBox */
typedef struct
{
    volatile uint32_t TXMIR;
    volatile uint32_t TXMDTR;
    volatile uint32_t TXMDLR;
    volatile uint32_t TXMDHR;
} CAN_TxMailBox_TypeDef;

/* Controller Area Network FIFOMailBox */
typedef struct
{
    volatile uint32_t RXMIR;
    volatile uint32_t RXMDTR;
    volatile uint32_t RXMDLR;
    volatile uint32_t RXMDHR;
} CAN_FIFOMailBox_TypeDef;

/* Controller Area Network FilterRegister */
typedef struct
{
    volatile uint32_t FR1;
    volatile uint32_t FR2;
} CAN_FilterRegister_TypeDef;

/* Controller Area Network */
typedef struct
{
    volatile uint32_t              CTLR;
    volatile uint32_t              STATR;
    volatile uint32_t              TSTATR;
    volatile uint32_t              RFIFO0;
    volatile uint32_t              RFIFO1;
    volatile uint32_t              INTENR;
    volatile uint32_t              ERRSR;
    volatile uint32_t              BTIMR;
    uint32_t                   RESERVED0[88];
    CAN_TxMailBox_TypeDef      sTxMailBox[3];
    CAN_FIFOMailBox_TypeDef    sFIFOMailBox[2];
    uint32_t                   RESERVED1[12];
    volatile uint32_t              FCTLR;
    volatile uint32_t              FMCFGR;
    uint32_t                   RESERVED2;
    volatile uint32_t              FSCFGR;
    uint32_t                   RESERVED3;
    volatile uint32_t              FAFIFOR;
    uint32_t                   RESERVED4;
    volatile uint32_t              FWR;
    uint32_t                   RESERVED5[8];
    CAN_FilterRegister_TypeDef sFilterRegister[28];
} CAN_TypeDef;

/* CRC Calculation Unit */
typedef struct
{
    volatile uint32_t DATAR;
    volatile uint8_t  IDATAR;
    uint8_t       RESERVED0;
    uint16_t      RESERVED1;
    volatile uint32_t CTLR;
} CRC_TypeDef;

/* DMA Channel Controller */
typedef struct
{
    volatile uint32_t CFGR;
    volatile uint32_t CNTR;
    volatile uint32_t PADDR;
    volatile uint32_t MADDR;
} DMA_Channel_TypeDef;

/* DMA Controller */
typedef struct
{
    volatile uint32_t INTFR;
    volatile uint32_t INTFCR;
} DMA_TypeDef;

/* External Interrupt/Event Controller */
typedef struct
{
    volatile uint32_t INTENR;
    volatile uint32_t EVENR;
    volatile uint32_t RTENR;
    volatile uint32_t FTENR;
    volatile uint32_t SWIEVR;
    volatile uint32_t INTFR;
} EXTI_TypeDef;

/* FLASH Registers */
typedef struct
{
    volatile uint32_t ACTLR;
    volatile uint32_t KEYR;
    volatile uint32_t OBKEYR;
    volatile uint32_t STATR;
    volatile uint32_t CTLR;
    volatile uint32_t ADDR;
    volatile uint32_t RESERVED;
    volatile uint32_t OBR;
    volatile uint32_t WPR;
    volatile uint32_t MODEKEYR;
} FLASH_TypeDef;

/* Option Bytes Registers */
typedef struct
{
    volatile uint16_t RDPR;
    volatile uint16_t USER;
    volatile uint16_t Data0;
    volatile uint16_t Data1;
    volatile uint16_t WRPR0;
    volatile uint16_t WRPR1;
    volatile uint16_t WRPR2;
    volatile uint16_t WRPR3;
} OB_TypeDef;

/* General Purpose I/O */
typedef struct
{
    volatile uint32_t CFGLR;
    volatile uint32_t CFGHR;
    volatile uint32_t INDR;
    volatile uint32_t OUTDR;
    volatile uint32_t BSHR;
    volatile uint32_t BCR;
    volatile uint32_t LCKR;
} GPIO_TypeDef;

/* Alternate Function I/O */
typedef struct
{
    volatile uint32_t ECR;
    volatile uint32_t PCFR1;
    volatile uint32_t EXTICR[4];
    uint32_t      RESERVED0;
    volatile uint32_t PCFR2;
} AFIO_TypeDef;

/* Inter Integrated Circuit Interface */
typedef struct
{
    volatile uint16_t CTLR1;
    uint16_t      RESERVED0;
    volatile uint16_t CTLR2;
    uint16_t      RESERVED1;
    volatile uint16_t OADDR1;
    uint16_t      RESERVED2;
    volatile uint16_t OADDR2;
    uint16_t      RESERVED3;
    volatile uint16_t DATAR;
    uint16_t      RESERVED4;
    volatile uint16_t STAR1;
    uint16_t      RESERVED5;
    volatile uint16_t STAR2;
    uint16_t      RESERVED6;
    volatile uint16_t CKCFGR;
    uint16_t      RESERVED7;
    volatile uint16_t RTR;
    uint16_t      RESERVED8;
} I2C_TypeDef;

/* Independent WatchDog */
typedef struct
{
    volatile uint32_t CTLR;
    volatile uint32_t PSCR;
    volatile uint32_t RLDR;
    volatile uint32_t STATR;
} IWDG_TypeDef;

/* Power Control */
typedef struct
{
    volatile uint32_t CTLR;
    volatile uint32_t CSR;
} PWR_TypeDef;

/* Reset and Clock Control */
typedef struct
{
    volatile uint32_t CTLR;
    volatile uint32_t CFGR0;
    volatile uint32_t INTR;
    volatile uint32_t APB2PRSTR;
    volatile uint32_t APB1PRSTR;
    volatile uint32_t AHBPCENR;
    volatile uint32_t APB2PCENR;
    volatile uint32_t APB1PCENR;
    volatile uint32_t BDCTLR;
    volatile uint32_t RSTSCKR;

    volatile uint32_t AHBRSTR;
    volatile uint32_t CFGR2;
} RCC_TypeDef;

/* Real-Time Clock */
typedef struct
{
    volatile uint16_t CTLRH;
    uint16_t      RESERVED0;
    volatile uint16_t CTLRL;
    uint16_t      RESERVED1;
    volatile uint16_t PSCRH;
    uint16_t      RESERVED2;
    volatile uint16_t PSCRL;
    uint16_t      RESERVED3;
    volatile uint16_t DIVH;
    uint16_t      RESERVED4;
    volatile uint16_t DIVL;
    uint16_t      RESERVED5;
    volatile uint16_t CNTH;
    uint16_t      RESERVED6;
    volatile uint16_t CNTL;
    uint16_t      RESERVED7;
    volatile uint16_t ALRMH;
    uint16_t      RESERVED8;
    volatile uint16_t ALRML;
    uint16_t      RESERVED9;
} RTC_TypeDef;

/* Serial Peripheral Interface */
typedef struct
{
    volatile uint16_t CTLR1;
    uint16_t      RESERVED0;
    volatile uint16_t CTLR2;
    uint16_t      RESERVED1;
    volatile uint16_t STATR;
    uint16_t      RESERVED2;
    volatile uint16_t DATAR;
    uint16_t      RESERVED3;
    volatile uint16_t CRCR;
    uint16_t      RESERVED4;
    volatile uint16_t RCRCR;
    uint16_t      RESERVED5;
    volatile uint16_t TCRCR;
    uint16_t      RESERVED6;
    volatile uint16_t I2SCFGR;
    uint16_t      RESERVED7;
    volatile uint16_t I2SPR;
    uint16_t      RESERVED8;
    volatile uint16_t HSCR;
    uint16_t      RESERVED9;
} SPI_TypeDef;

/* TIM */
typedef struct
{
    volatile uint16_t CTLR1;
    uint16_t      RESERVED0;
    volatile uint16_t CTLR2;
    uint16_t      RESERVED1;
    volatile uint16_t SMCFGR;
    uint16_t      RESERVED2;
    volatile uint16_t DMAINTENR;
    uint16_t      RESERVED3;
    volatile uint16_t INTFR;
    uint16_t      RESERVED4;
    volatile uint16_t SWEVGR;
    uint16_t      RESERVED5;
    volatile uint16_t CHCTLR1;
    uint16_t      RESERVED6;
    volatile uint16_t CHCTLR2;
    uint16_t      RESERVED7;
    volatile uint16_t CCER;
    uint16_t      RESERVED8;
    volatile uint16_t CNT;
    uint16_t      RESERVED9;
    volatile uint16_t PSC;
    uint16_t      RESERVED10;
    volatile uint16_t ATRLR;
    uint16_t      RESERVED11;
    volatile uint16_t RPTCR;
    uint16_t      RESERVED12;
    volatile uint16_t CH1CVR;
    uint16_t      RESERVED13;
    volatile uint16_t CH2CVR;
    uint16_t      RESERVED14;
    volatile uint16_t CH3CVR;
    uint16_t      RESERVED15;
    volatile uint16_t CH4CVR;
    uint16_t      RESERVED16;
    volatile uint16_t BDTR;
    uint16_t      RESERVED17;
    volatile uint16_t DMACFGR;
    uint16_t      RESERVED18;
    volatile uint16_t DMAADR;
    uint16_t      RESERVED19;
} TIM_TypeDef;

/* Universal Synchronous Asynchronous Receiver Transmitter */
typedef struct
{
    volatile uint16_t STATR;
    uint16_t      RESERVED0;
    volatile uint16_t DATAR;
    uint16_t      RESERVED1;
    volatile uint16_t BRR;
    uint16_t      RESERVED2;
    volatile uint16_t CTLR1;
    uint16_t      RESERVED3;
    volatile uint16_t CTLR2;
    uint16_t      RESERVED4;
    volatile uint16_t CTLR3;
    uint16_t      RESERVED5;
    volatile uint16_t GPR;
    uint16_t      RESERVED6;
} USART_TypeDef;

/* Window WatchDog */
typedef struct
{
    volatile uint32_t CTLR;
    volatile uint32_t CFGR;
    volatile uint32_t STATR;
} WWDG_TypeDef;

/* Enhanced Registers */
typedef struct
{
    volatile uint32_t EXTEN_CTR;
} EXTEN_TypeDef;

/* OPA Registers */
typedef struct
{
    volatile uint32_t CR;
} OPA_TypeDef;

/* USBFS Registers */
typedef struct
{
    volatile uint8_t  BASE_CTRL;
    volatile uint8_t  UDEV_CTRL;
    volatile uint8_t  INT_EN;
    volatile uint8_t  DEV_ADDR;
    volatile uint8_t  Reserve0;
    volatile uint8_t  MIS_ST;
    volatile uint8_t  INT_FG;
    volatile uint8_t  INT_ST;
    volatile uint32_t RX_LEN;
    volatile uint8_t  UEP4_1_MOD;
    volatile uint8_t  UEP2_3_MOD;
    volatile uint8_t  UEP5_6_MOD;
    volatile uint8_t  UEP7_MOD;
    volatile uint32_t UEP0_DMA;
    volatile uint32_t UEP1_DMA;
    volatile uint32_t UEP2_DMA;
    volatile uint32_t UEP3_DMA;
    volatile uint32_t UEP4_DMA;
    volatile uint32_t UEP5_DMA;
    volatile uint32_t UEP6_DMA;
    volatile uint32_t UEP7_DMA;
    volatile uint16_t UEP0_TX_LEN;
    volatile uint8_t  UEP0_TX_CTRL;
    volatile uint8_t  UEP0_RX_CTRL;
    volatile uint16_t UEP1_TX_LEN;
    volatile uint8_t  UEP1_TX_CTRL;
    volatile uint8_t  UEP1_RX_CTRL;
    volatile uint16_t UEP2_TX_LEN;
    volatile uint8_t  UEP2_TX_CTRL;
    volatile uint8_t  UEP2_RX_CTRL;
    volatile uint16_t UEP3_TX_LEN;
    volatile uint8_t  UEP3_TX_CTRL;
    volatile uint8_t  UEP3_RX_CTRL;
    volatile uint16_t UEP4_TX_LEN;
    volatile uint8_t  UEP4_TX_CTRL;
    volatile uint8_t  UEP4_RX_CTRL;
    volatile uint16_t UEP5_TX_LEN;
    volatile uint8_t  UEP5_TX_CTRL;
    volatile uint8_t  UEP5_RX_CTRL;
    volatile uint16_t UEP6_TX_LEN;
    volatile uint8_t  UEP6_TX_CTRL;
    volatile uint8_t  UEP6_RX_CTRL;
    volatile uint16_t UEP7_TX_LEN;
    volatile uint8_t  UEP7_TX_CTRL;
    volatile uint8_t  UEP7_RX_CTRL;
    volatile uint32_t Reserve1;
    volatile uint32_t OTG_CR;
    volatile uint32_t OTG_SR;
} USBOTG_FS_TypeDef;

typedef struct
{
    volatile uint8_t   BASE_CTRL;
    volatile uint8_t   HOST_CTRL;
    volatile uint8_t   INT_EN;
    volatile uint8_t   DEV_ADDR;
    volatile uint8_t   Reserve0;
    volatile uint8_t   MIS_ST;
    volatile uint8_t   INT_FG;
    volatile uint8_t   INT_ST;
    volatile uint16_t  RX_LEN;
    volatile uint16_t  Reserve1;
    volatile uint8_t   Reserve2;
    volatile uint8_t   HOST_EP_MOD;
    volatile uint16_t  Reserve3;
    volatile uint32_t  Reserve4;
    volatile uint32_t  Reserve5;
    volatile uint32_t  HOST_RX_DMA;
    volatile uint32_t  HOST_TX_DMA;
    volatile uint32_t  Reserve6;
    volatile uint32_t  Reserve7;
    volatile uint32_t  Reserve8;
    volatile uint32_t  Reserve9;
    volatile uint32_t  Reserve10;
    volatile uint16_t  Reserve11;
    volatile uint16_t  HOST_SETUP;
    volatile uint8_t   HOST_EP_PID;
    volatile uint8_t   Reserve12;
    volatile uint8_t   Reserve13;
    volatile uint8_t   HOST_RX_CTRL;
    volatile uint16_t  HOST_TX_LEN;
    volatile uint8_t   HOST_TX_CTRL;
    volatile uint8_t   Reserve14;
    volatile uint32_t  Reserve15;
    volatile uint32_t  Reserve16;
    volatile uint32_t  Reserve17;
    volatile uint32_t  Reserve18;
    volatile uint32_t  Reserve19;
    volatile uint32_t  OTG_CR;
    volatile uint32_t  OTG_SR;
} USBOTG_FS_HOST_TypeDef;

/* Peripheral memory map */
#define FLASH_BASE                              ((uint32_t)0x08000000) /* FLASH base address in the alias region */
#define SRAM_BASE                               ((uint32_t)0x20000000) /* SRAM base address in the alias region */
#define PERIPH_BASE                             ((uint32_t)0x40000000) /* Peripheral base address in the alias region */

#define APB1PERIPH_BASE                         (PERIPH_BASE)
#define APB2PERIPH_BASE                         (PERIPH_BASE + 0x10000)
#define AHBPERIPH_BASE                          (PERIPH_BASE + 0x20000)

#define TIM2_BASE                               (APB1PERIPH_BASE + 0x0000)
#define TIM3_BASE                               (APB1PERIPH_BASE + 0x0400)
#define TIM4_BASE                               (APB1PERIPH_BASE + 0x0800)
#define TIM5_BASE                               (APB1PERIPH_BASE + 0x0C00)
#define RTC_BASE                                (APB1PERIPH_BASE + 0x2800)
#define WWDG_BASE                               (APB1PERIPH_BASE + 0x2C00)
#define IWDG_BASE                               (APB1PERIPH_BASE + 0x3000)
#define SPI2_BASE                               (APB1PERIPH_BASE + 0x3800)
#define USART2_BASE                             (APB1PERIPH_BASE + 0x4400)
#define USART3_BASE                             (APB1PERIPH_BASE + 0x4800)
#define UART4_BASE                              (APB1PERIPH_BASE + 0x4C00)
#define I2C1_BASE                               (APB1PERIPH_BASE + 0x5400)
#define I2C2_BASE                               (APB1PERIPH_BASE + 0x5800)
#define CAN1_BASE                               (APB1PERIPH_BASE + 0x6400)
#define BKP_BASE                                (APB1PERIPH_BASE + 0x6C00)
#define PWR_BASE                                (APB1PERIPH_BASE + 0x7000)

#define AFIO_BASE                               (APB2PERIPH_BASE + 0x0000)
#define EXTI_BASE                               (APB2PERIPH_BASE + 0x0400)
#define GPIOA_BASE                              (APB2PERIPH_BASE + 0x0800)
#define GPIOB_BASE                              (APB2PERIPH_BASE + 0x0C00)
#define GPIOC_BASE                              (APB2PERIPH_BASE + 0x1000)
#define GPIOD_BASE                              (APB2PERIPH_BASE + 0x1400)
#define GPIOE_BASE                              (APB2PERIPH_BASE + 0x1800)
#define GPIOF_BASE                              (APB2PERIPH_BASE + 0x1C00)
#define GPIOG_BASE                              (APB2PERIPH_BASE + 0x2000)
#define ADC1_BASE                               (APB2PERIPH_BASE + 0x2400)
#define ADC2_BASE                               (APB2PERIPH_BASE + 0x2800)
#define TIM1_BASE                               (APB2PERIPH_BASE + 0x2C00)
#define SPI1_BASE                               (APB2PERIPH_BASE + 0x3000)
#define USART1_BASE                             (APB2PERIPH_BASE + 0x3800)

#define DMA1_BASE                               (AHBPERIPH_BASE + 0x0000)
#define DMA1_Channel1_BASE                      (AHBPERIPH_BASE + 0x0008)
#define DMA1_Channel2_BASE                      (AHBPERIPH_BASE + 0x001C)
#define DMA1_Channel3_BASE                      (AHBPERIPH_BASE + 0x0030)
#define DMA1_Channel4_BASE                      (AHBPERIPH_BASE + 0x0044)
#define DMA1_Channel5_BASE                      (AHBPERIPH_BASE + 0x0058)
#define DMA1_Channel6_BASE                      (AHBPERIPH_BASE + 0x006C)
#define DMA1_Channel7_BASE                      (AHBPERIPH_BASE + 0x0080)
#define DMA1_Channel8_BASE                      (AHBPERIPH_BASE + 0x0094)
#define RCC_BASE                                (AHBPERIPH_BASE + 0x1000)
#define FLASH_R_BASE                            (AHBPERIPH_BASE + 0x2000)
#define CRC_BASE                                (AHBPERIPH_BASE + 0x3000)
#define EXTEN_BASE                              (AHBPERIPH_BASE + 0x3800)
#define OPA_BASE                                (AHBPERIPH_BASE + 0x3804)
#define ETH10M_BASE                             (AHBPERIPH_BASE + 0x8000)

#define USBFS_BASE                              ((uint32_t)0x50000000)

#define OB_BASE                                 ((uint32_t)0x1FFFF800)

/* Peripheral declaration */
/* Peripheral declaration */
#define TIM2                                    ((TIM_TypeDef *)TIM2_BASE)
#define TIM3                                    ((TIM_TypeDef *)TIM3_BASE)
#define TIM4                                    ((TIM_TypeDef *)TIM4_BASE)
#define TIM5                                    ((TIM_TypeDef *)TIM5_BASE)
#define RTC                                     ((RTC_TypeDef *)RTC_BASE)
#define WWDG                                    ((WWDG_TypeDef *)WWDG_BASE)
#define IWDG                                    ((IWDG_TypeDef *)IWDG_BASE)
#define SPI2                                    ((SPI_TypeDef *)SPI2_BASE)
#define USART2                                  ((USART_TypeDef *)USART2_BASE)
#define USART3                                  ((USART_TypeDef *)USART3_BASE)
#define UART4                                   ((USART_TypeDef *)UART4_BASE)
#define I2C1                                    ((I2C_TypeDef *)I2C1_BASE)
#define I2C2                                    ((I2C_TypeDef *)I2C2_BASE)
#define CAN1                                    ((CAN_TypeDef *)CAN1_BASE)
#define BKP                                     ((BKP_TypeDef *)BKP_BASE)
#define PWR                                     ((PWR_TypeDef *)PWR_BASE)

#define AFIO                                    ((AFIO_TypeDef *)AFIO_BASE)
#define EXTI                                    ((EXTI_TypeDef *)EXTI_BASE)
#define GPIOA                                   ((GPIO_TypeDef *)GPIOA_BASE)
#define GPIOB                                   ((GPIO_TypeDef *)GPIOB_BASE)
#define GPIOC                                   ((GPIO_TypeDef *)GPIOC_BASE)
#define GPIOD                                   ((GPIO_TypeDef *)GPIOD_BASE)
#define GPIOE                                   ((GPIO_TypeDef *)GPIOE_BASE)
#define GPIOF                                   ((GPIO_TypeDef *)GPIOF_BASE)
#define GPIOG                                   ((GPIO_TypeDef *)GPIOG_BASE)
#define ADC1                                    ((ADC_TypeDef *)ADC1_BASE)
#define ADC2                                    ((ADC_TypeDef *)ADC2_BASE)
#define TKey1                                   ((ADC_TypeDef *)ADC1_BASE)
#define TKey2                                   ((ADC_TypeDef *)ADC2_BASE)
#define TIM1                                    ((TIM_TypeDef *)TIM1_BASE)
#define SPI1                                    ((SPI_TypeDef *)SPI1_BASE)
#define USART1                                  ((USART_TypeDef *)USART1_BASE)

#define DMA1                                    ((DMA_TypeDef *)DMA1_BASE)
#define DMA1_Channel1                           ((DMA_Channel_TypeDef *)DMA1_Channel1_BASE)
#define DMA1_Channel2                           ((DMA_Channel_TypeDef *)DMA1_Channel2_BASE)
#define DMA1_Channel3                           ((DMA_Channel_TypeDef *)DMA1_Channel3_BASE)
#define DMA1_Channel4                           ((DMA_Channel_TypeDef *)DMA1_Channel4_BASE)
#define DMA1_Channel5                           ((DMA_Channel_TypeDef *)DMA1_Channel5_BASE)
#define DMA1_Channel6                           ((DMA_Channel_TypeDef *)DMA1_Channel6_BASE)
#define DMA1_Channel7                           ((DMA_Channel_TypeDef *)DMA1_Channel7_BASE)
#define DMA1_Channel8                           ((DMA_Channel_TypeDef *)DMA1_Channel8_BASE)
#define RCC                                     ((RCC_TypeDef *)RCC_BASE)
#define FLASH                                   ((FLASH_TypeDef *)FLASH_R_BASE)
#define CRC                                     ((CRC_TypeDef *)CRC_BASE)
#define USBOTG_FS                               ((USBOTG_FS_TypeDef *)USBFS_BASE)
#define USBOTG_H_FS                             ((USBOTG_FS_HOST_TypeDef *)USBFS_BASE)
#define EXTEN                                   ((EXTEN_TypeDef *)EXTEN_BASE)
#define OPA                                     ((OPA_TypeDef *)OPA_BASE)
#define ETH10M                                  ((ETH10M_TypeDef *)ETH10M_BASE)

#define OB                                      ((OB_TypeDef *)OB_BASE)

// RCC

/* RCC_Exported_Types */
typedef struct
{
    uint32_t SYSCLK_Frequency; /* returns SYSCLK clock frequency expressed in Hz */
    uint32_t HCLK_Frequency;   /* returns HCLK clock frequency expressed in Hz */
    uint32_t PCLK1_Frequency;  /* returns PCLK1 clock frequency expressed in Hz */
    uint32_t PCLK2_Frequency;  /* returns PCLK2 clock frequency expressed in Hz */
    uint32_t ADCCLK_Frequency; /* returns ADCCLK clock frequency expressed in Hz */
} RCC_ClocksTypeDef;

/* HSE_configuration */
#define RCC_HSE_OFF                     ((uint32_t)0x00000000)
#define RCC_HSE_ON                      ((uint32_t)0x00010000)
#define RCC_HSE_Bypass                  ((uint32_t)0x00040000)

/* PLL_entry_clock_source */
#define RCC_PLLSource_HSI_Div2          ((uint32_t)0x00000000)
#define RCC_PLLSource_HSE_Div1          ((uint32_t)0x00010000)
#define RCC_PLLSource_HSE_Div2          ((uint32_t)0x00030000)

/* PLL_multiplication_factor for other CH32V20x  */
#define RCC_PLLMul_2                    ((uint32_t)0x00000000)
#define RCC_PLLMul_3                    ((uint32_t)0x00040000)
#define RCC_PLLMul_4                    ((uint32_t)0x00080000)
#define RCC_PLLMul_5                    ((uint32_t)0x000C0000)
#define RCC_PLLMul_6                    ((uint32_t)0x00100000)
#define RCC_PLLMul_7                    ((uint32_t)0x00140000)
#define RCC_PLLMul_8                    ((uint32_t)0x00180000)
#define RCC_PLLMul_9                    ((uint32_t)0x001C0000)
#define RCC_PLLMul_10                   ((uint32_t)0x00200000)
#define RCC_PLLMul_11                   ((uint32_t)0x00240000)
#define RCC_PLLMul_12                   ((uint32_t)0x00280000)
#define RCC_PLLMul_13                   ((uint32_t)0x002C0000)
#define RCC_PLLMul_14                   ((uint32_t)0x00300000)
#define RCC_PLLMul_15                   ((uint32_t)0x00340000)
#define RCC_PLLMul_16                   ((uint32_t)0x00380000)
#define RCC_PLLMul_18                   ((uint32_t)0x003C0000)

/* System_clock_source */
#define RCC_SYSCLKSource_HSI            ((uint32_t)0x00000000)
#define RCC_SYSCLKSource_HSE            ((uint32_t)0x00000001)
#define RCC_SYSCLKSource_PLLCLK         ((uint32_t)0x00000002)

/* AHB_clock_source */
#define RCC_SYSCLK_Div1                 ((uint32_t)0x00000000)
#define RCC_SYSCLK_Div2                 ((uint32_t)0x00000080)
#define RCC_SYSCLK_Div4                 ((uint32_t)0x00000090)
#define RCC_SYSCLK_Div8                 ((uint32_t)0x000000A0)
#define RCC_SYSCLK_Div16                ((uint32_t)0x000000B0)
#define RCC_SYSCLK_Div64                ((uint32_t)0x000000C0)
#define RCC_SYSCLK_Div128               ((uint32_t)0x000000D0)
#define RCC_SYSCLK_Div256               ((uint32_t)0x000000E0)
#define RCC_SYSCLK_Div512               ((uint32_t)0x000000F0)

/* APB1_APB2_clock_source */
#define RCC_HCLK_Div1                   ((uint32_t)0x00000000)
#define RCC_HCLK_Div2                   ((uint32_t)0x00000400)
#define RCC_HCLK_Div4                   ((uint32_t)0x00000500)
#define RCC_HCLK_Div8                   ((uint32_t)0x00000600)
#define RCC_HCLK_Div16                  ((uint32_t)0x00000700)

/* RCC_Interrupt_source */
#define RCC_IT_LSIRDY                   ((uint8_t)0x01)
#define RCC_IT_LSERDY                   ((uint8_t)0x02)
#define RCC_IT_HSIRDY                   ((uint8_t)0x04)
#define RCC_IT_HSERDY                   ((uint8_t)0x08)
#define RCC_IT_PLLRDY                   ((uint8_t)0x10)
#define RCC_IT_CSS                      ((uint8_t)0x80)

/* USB_Device_clock_source */
#define RCC_USBCLKSource_PLLCLK_Div1    ((uint8_t)0x00)
#define RCC_USBCLKSource_PLLCLK_Div2    ((uint8_t)0x01)
#define RCC_USBCLKSource_PLLCLK_Div3    ((uint8_t)0x02)

#ifdef CH32V20x_D8W
  #define RCC_USBCLKSource_PLLCLK_Div5    ((uint8_t)0x03)

#endif

/* ADC_clock_source */
#define RCC_PCLK2_Div2                 ((uint32_t)0x00000000)
#define RCC_PCLK2_Div4                 ((uint32_t)0x00004000)
#define RCC_PCLK2_Div6                 ((uint32_t)0x00008000)
#define RCC_PCLK2_Div8                 ((uint32_t)0x0000C000)

/* LSE_configuration */
#define RCC_LSE_OFF                    ((uint8_t)0x00)
#define RCC_LSE_ON                     ((uint8_t)0x01)
#define RCC_LSE_Bypass                 ((uint8_t)0x04)

/* RTC_clock_source */
#define RCC_RTCCLKSource_LSE           ((uint32_t)0x00000100)
#define RCC_RTCCLKSource_LSI           ((uint32_t)0x00000200)
#define RCC_RTCCLKSource_HSE_Div128    ((uint32_t)0x00000300)

/* AHB_peripheral */
#define RCC_AHBPeriph_DMA1             ((uint32_t)0x00000001)
#define RCC_AHBPeriph_DMA2             ((uint32_t)0x00000002)
#define RCC_AHBPeriph_SRAM             ((uint32_t)0x00000004)
#define RCC_AHBPeriph_CRC              ((uint32_t)0x00000040)
#define RCC_AHBPeriph_FSMC             ((uint32_t)0x00000100)
#define RCC_AHBPeriph_RNG              ((uint32_t)0x00000200)
#define RCC_AHBPeriph_SDIO             ((uint32_t)0x00000400)
#define RCC_AHBPeriph_USBHS            ((uint32_t)0x00000800)
#define RCC_AHBPeriph_OTG_FS           ((uint32_t)0x00001000)

#ifdef CH32V20x_D8W
#define RCC_AHBPeriph_BLE_CRC          ((uint32_t)0x00030040)
#endif

/* APB2_peripheral */
#define RCC_APB2Periph_AFIO            ((uint32_t)0x00000001)
#define RCC_APB2Periph_GPIOA           ((uint32_t)0x00000004)
#define RCC_APB2Periph_GPIOB           ((uint32_t)0x00000008)
#define RCC_APB2Periph_GPIOC           ((uint32_t)0x00000010)
#define RCC_APB2Periph_GPIOD           ((uint32_t)0x00000020)
#define RCC_APB2Periph_GPIOE           ((uint32_t)0x00000040)
#define RCC_APB2Periph_ADC1            ((uint32_t)0x00000200)
#define RCC_APB2Periph_ADC2            ((uint32_t)0x00000400)
#define RCC_APB2Periph_TIM1            ((uint32_t)0x00000800)
#define RCC_APB2Periph_SPI1            ((uint32_t)0x00001000)
#define RCC_APB2Periph_TIM8            ((uint32_t)0x00002000)
#define RCC_APB2Periph_USART1          ((uint32_t)0x00004000)
#define RCC_APB2Periph_TIM9            ((uint32_t)0x00080000)
#define RCC_APB2Periph_TIM10           ((uint32_t)0x00100000)

/* APB1_peripheral */
#define RCC_APB1Periph_TIM2            ((uint32_t)0x00000001)
#define RCC_APB1Periph_TIM3            ((uint32_t)0x00000002)
#define RCC_APB1Periph_TIM4            ((uint32_t)0x00000004)
#define RCC_APB1Periph_TIM5            ((uint32_t)0x00000008)
#define RCC_APB1Periph_TIM6            ((uint32_t)0x00000010)
#define RCC_APB1Periph_TIM7            ((uint32_t)0x00000020)
#define RCC_APB1Periph_UART6           ((uint32_t)0x00000040)
#define RCC_APB1Periph_UART7           ((uint32_t)0x00000080)
#define RCC_APB1Periph_UART8           ((uint32_t)0x00000100)
#define RCC_APB1Periph_WWDG            ((uint32_t)0x00000800)
#define RCC_APB1Periph_SPI2            ((uint32_t)0x00004000)
#define RCC_APB1Periph_SPI3            ((uint32_t)0x00008000)
#define RCC_APB1Periph_USART2          ((uint32_t)0x00020000)
#define RCC_APB1Periph_USART3          ((uint32_t)0x00040000)
#define RCC_APB1Periph_UART4           ((uint32_t)0x00080000)
#define RCC_APB1Periph_UART5           ((uint32_t)0x00100000)
#define RCC_APB1Periph_I2C1            ((uint32_t)0x00200000)
#define RCC_APB1Periph_I2C2            ((uint32_t)0x00400000)
#define RCC_APB1Periph_USB             ((uint32_t)0x00800000)
#define RCC_APB1Periph_CAN1            ((uint32_t)0x02000000)
#define RCC_APB1Periph_CAN2            ((uint32_t)0x04000000)
#define RCC_APB1Periph_BKP             ((uint32_t)0x08000000)
#define RCC_APB1Periph_PWR             ((uint32_t)0x10000000)
#define RCC_APB1Periph_DAC             ((uint32_t)0x20000000)

/* Clock_source_to_output_on_MCO_pin */
#define RCC_MCO_NoClock                ((uint8_t)0x00)
#define RCC_MCO_SYSCLK                 ((uint8_t)0x04)
#define RCC_MCO_HSI                    ((uint8_t)0x05)
#define RCC_MCO_HSE                    ((uint8_t)0x06)
#define RCC_MCO_PLLCLK_Div2            ((uint8_t)0x07)

/* RCC_Flag */
#define RCC_FLAG_HSIRDY                ((uint8_t)0x21)
#define RCC_FLAG_HSERDY                ((uint8_t)0x31)
#define RCC_FLAG_PLLRDY                ((uint8_t)0x39)
#define RCC_FLAG_LSERDY                ((uint8_t)0x41)
#define RCC_FLAG_LSIRDY                ((uint8_t)0x61)
#define RCC_FLAG_PINRST                ((uint8_t)0x7A)
#define RCC_FLAG_PORRST                ((uint8_t)0x7B)
#define RCC_FLAG_SFTRST                ((uint8_t)0x7C)
#define RCC_FLAG_IWDGRST               ((uint8_t)0x7D)
#define RCC_FLAG_WWDGRST               ((uint8_t)0x7E)
#define RCC_FLAG_LPWRRST               ((uint8_t)0x7F)

/* SysTick_clock_source */
#define SysTick_CLKSource_HCLK_Div8    ((uint32_t)0xFFFFFFFB)
#define SysTick_CLKSource_HCLK         ((uint32_t)0x00000004)

/* USBFS_clock_source */
#define RCC_USBPLL_Div1                ((uint32_t)0x00)
#define RCC_USBPLL_Div2                ((uint32_t)0x01)
#define RCC_USBPLL_Div3                ((uint32_t)0x02)
#define RCC_USBPLL_Div4                ((uint32_t)0x03)
#define RCC_USBPLL_Div5                ((uint32_t)0x04)
#define RCC_USBPLL_Div6                ((uint32_t)0x05)
#define RCC_USBPLL_Div7                ((uint32_t)0x06)
#define RCC_USBPLL_Div8                ((uint32_t)0x07)

// GPIO

/* Output Maximum frequency selection */
typedef enum
{
    GPIO_Speed_10MHz = 1,
    GPIO_Speed_2MHz,
    GPIO_Speed_50MHz
} GPIOSpeed_TypeDef;

#define GPIO_CNF_IN_ANALOG   0
#define GPIO_CNF_IN_FLOATING 4
#define GPIO_CNF_IN_PUPD     8
#define GPIO_CNF_OUT_PP      0
#define GPIO_CNF_OUT_OD      4
#define GPIO_CNF_OUT_PP_AF   8
#define GPIO_CNF_OUT_OD_AF   12


/* Bit_SET and Bit_RESET enumeration */
typedef enum
{
    Bit_RESET = 0,
    Bit_SET
} BitAction;

/* GPIO_pins_define */
#define GPIO_Pin_0                      ((uint16_t)0x0001) /* Pin 0 selected */
#define GPIO_Pin_1                      ((uint16_t)0x0002) /* Pin 1 selected */
#define GPIO_Pin_2                      ((uint16_t)0x0004) /* Pin 2 selected */
#define GPIO_Pin_3                      ((uint16_t)0x0008) /* Pin 3 selected */
#define GPIO_Pin_4                      ((uint16_t)0x0010) /* Pin 4 selected */
#define GPIO_Pin_5                      ((uint16_t)0x0020) /* Pin 5 selected */
#define GPIO_Pin_6                      ((uint16_t)0x0040) /* Pin 6 selected */
#define GPIO_Pin_7                      ((uint16_t)0x0080) /* Pin 7 selected */
#define GPIO_Pin_8                      ((uint16_t)0x0100) /* Pin 8 selected */
#define GPIO_Pin_9                      ((uint16_t)0x0200) /* Pin 9 selected */
#define GPIO_Pin_10                     ((uint16_t)0x0400) /* Pin 10 selected */
#define GPIO_Pin_11                     ((uint16_t)0x0800) /* Pin 11 selected */
#define GPIO_Pin_12                     ((uint16_t)0x1000) /* Pin 12 selected */
#define GPIO_Pin_13                     ((uint16_t)0x2000) /* Pin 13 selected */
#define GPIO_Pin_14                     ((uint16_t)0x4000) /* Pin 14 selected */
#define GPIO_Pin_15                     ((uint16_t)0x8000) /* Pin 15 selected */
#define GPIO_Pin_All                    ((uint16_t)0xFFFF) /* All pins selected */

/* GPIO_Remap_define */
/* PCFR1 */
#define GPIO_Remap_SPI1                 ((uint32_t)0x00000001) /* SPI1 Alternate Function mapping */
#define GPIO_Remap_I2C1                 ((uint32_t)0x00000002) /* I2C1 Alternate Function mapping */
#define GPIO_Remap_USART1               ((uint32_t)0x00000004) /* USART1 Alternate Function mapping low bit */
#define GPIO_Remap_USART2               ((uint32_t)0x00000008) /* USART2 Alternate Function mapping */
#define GPIO_PartialRemap_USART3        ((uint32_t)0x00140010) /* USART3 Partial Alternate Function mapping */
#define GPIO_FullRemap_USART3           ((uint32_t)0x00140030) /* USART3 Full Alternate Function mapping */
#define GPIO_PartialRemap_TIM1          ((uint32_t)0x00160040) /* TIM1 Partial Alternate Function mapping */
#define GPIO_FullRemap_TIM1             ((uint32_t)0x001600C0) /* TIM1 Full Alternate Function mapping */
#define GPIO_PartialRemap1_TIM2         ((uint32_t)0x00180100) /* TIM2 Partial1 Alternate Function mapping */
#define GPIO_PartialRemap2_TIM2         ((uint32_t)0x00180200) /* TIM2 Partial2 Alternate Function mapping */
#define GPIO_FullRemap_TIM2             ((uint32_t)0x00180300) /* TIM2 Full Alternate Function mapping */
#define GPIO_PartialRemap_TIM3          ((uint32_t)0x001A0800) /* TIM3 Partial Alternate Function mapping */
#define GPIO_FullRemap_TIM3             ((uint32_t)0x001A0C00) /* TIM3 Full Alternate Function mapping */
#define GPIO_Remap_TIM4                 ((uint32_t)0x00001000) /* TIM4 Alternate Function mapping */
#define GPIO_Remap1_CAN1                ((uint32_t)0x001D4000) /* CAN1 Alternate Function mapping */
#define GPIO_Remap2_CAN1                ((uint32_t)0x001D6000) /* CAN1 Alternate Function mapping */
#define GPIO_Remap_PD01                 ((uint32_t)0x00008000) /* PD01 Alternate Function mapping */
#define GPIO_Remap_TIM5CH4_LSI          ((uint32_t)0x00200001) /* LSI connected to TIM5 Channel4 input capture for calibration */
#define GPIO_Remap_ADC1_ETRGINJ         ((uint32_t)0x00200002) /* ADC1 External Trigger Injected Conversion remapping */
#define GPIO_Remap_ADC1_ETRGREG         ((uint32_t)0x00200004) /* ADC1 External Trigger Regular Conversion remapping */
#define GPIO_Remap_ADC2_ETRGINJ         ((uint32_t)0x00200008) /* ADC2 External Trigger Injected Conversion remapping */
#define GPIO_Remap_ADC2_ETRGREG         ((uint32_t)0x00200010) /* ADC2 External Trigger Regular Conversion remapping */
#define GPIO_Remap_ETH                  ((uint32_t)0x00200020) /* Ethernet remapping (only for Connectivity line devices) */
#define GPIO_Remap_CAN2                 ((uint32_t)0x00200040) /* CAN2 remapping (only for Connectivity line devices) */
#define GPIO_Remap_MII_RMII_SEL         ((uint32_t)0x00200080) /* MII or RMII selection */
#define GPIO_Remap_SWJ_NoJTRST          ((uint32_t)0x00300100) /* Full SWJ Enabled (JTAG-DP + SW-DP) but without JTRST */
#define GPIO_Remap_SWJ_JTAGDisable      ((uint32_t)0x00300200) /* JTAG-DP Disabled and SW-DP Enabled */
#define GPIO_Remap_SWJ_Disable          ((uint32_t)0x00300400) /* Full SWJ Disabled (JTAG-DP + SW-DP) */
#define GPIO_Remap_SPI3                 ((uint32_t)0x00201000) /* SPI3/I2S3 Alternate Function mapping (only for Connectivity line devices) */
#define GPIO_Remap_TIM2ITR1_PTP_SOF     ((uint32_t)0x00202000) /* Ethernet PTP output or USB OTG SOF (Start of Frame) connected \
                                                                  to TIM2 Internal Trigger 1 for calibration                    \
                                                                  (only for Connectivity line devices) */
#define GPIO_Remap_PTP_PPS              ((uint32_t)0x00204000) /* Ethernet MAC PPS_PTS output on PB05 (only for Connectivity line devices) */

/* PCFR2 */
#define GPIO_Remap_TIM8                 ((uint32_t)0x80000004) /* TIM8 Alternate Function mapping */
#define GPIO_PartialRemap_TIM9          ((uint32_t)0x80130008) /* TIM9 Partial Alternate Function mapping */
#define GPIO_FullRemap_TIM9             ((uint32_t)0x80130010) /* TIM9 Full Alternate Function mapping */
#define GPIO_PartialRemap_TIM10         ((uint32_t)0x80150020) /* TIM10 Partial Alternate Function mapping */
#define GPIO_FullRemap_TIM10            ((uint32_t)0x80150040) /* TIM10 Full Alternate Function mapping */
#define GPIO_Remap_FSMC_NADV            ((uint32_t)0x80000400) /* FSMC_NADV Alternate Function mapping */
#define GPIO_PartialRemap_USART4        ((uint32_t)0x80300001) /* USART4 Partial Alternate Function mapping */
#define GPIO_FullRemap_USART4           ((uint32_t)0x80300002) /* USART4 Full Alternate Function mapping */
#define GPIO_PartialRemap_USART5        ((uint32_t)0x80320004) /* USART5 Partial Alternate Function mapping */
#define GPIO_FullRemap_USART5           ((uint32_t)0x80320008) /* USART5 Full Alternate Function mapping */
#define GPIO_PartialRemap_USART6        ((uint32_t)0x80340010) /* USART6 Partial Alternate Function mapping */
#define GPIO_FullRemap_USART6           ((uint32_t)0x80340020) /* USART6 Full Alternate Function mapping */
#define GPIO_PartialRemap_USART7        ((uint32_t)0x80360040) /* USART7 Partial Alternate Function mapping */
#define GPIO_FullRemap_USART7           ((uint32_t)0x80360080) /* USART7 Full Alternate Function mapping */
#define GPIO_PartialRemap_USART8        ((uint32_t)0x80380100) /* USART8 Partial Alternate Function mapping */
#define GPIO_FullRemap_USART8           ((uint32_t)0x80380200) /* USART8 Full Alternate Function mapping */
#define GPIO_Remap_USART1_HighBit       ((uint32_t)0x80200400) /* USART1 Alternate Function mapping high bit */

/* GPIO_Port_Sources */
#define GPIO_PortSourceGPIOA            ((uint8_t)0x00)
#define GPIO_PortSourceGPIOB            ((uint8_t)0x01)
#define GPIO_PortSourceGPIOC            ((uint8_t)0x02)
#define GPIO_PortSourceGPIOD            ((uint8_t)0x03)
#define GPIO_PortSourceGPIOE            ((uint8_t)0x04)
#define GPIO_PortSourceGPIOF            ((uint8_t)0x05)
#define GPIO_PortSourceGPIOG            ((uint8_t)0x06)

/* GPIO_Pin_sources */
#define GPIO_PinSource0                 ((uint8_t)0x00)
#define GPIO_PinSource1                 ((uint8_t)0x01)
#define GPIO_PinSource2                 ((uint8_t)0x02)
#define GPIO_PinSource3                 ((uint8_t)0x03)
#define GPIO_PinSource4                 ((uint8_t)0x04)
#define GPIO_PinSource5                 ((uint8_t)0x05)
#define GPIO_PinSource6                 ((uint8_t)0x06)
#define GPIO_PinSource7                 ((uint8_t)0x07)
#define GPIO_PinSource8                 ((uint8_t)0x08)
#define GPIO_PinSource9                 ((uint8_t)0x09)
#define GPIO_PinSource10                ((uint8_t)0x0A)
#define GPIO_PinSource11                ((uint8_t)0x0B)
#define GPIO_PinSource12                ((uint8_t)0x0C)
#define GPIO_PinSource13                ((uint8_t)0x0D)
#define GPIO_PinSource14                ((uint8_t)0x0E)
#define GPIO_PinSource15                ((uint8_t)0x0F)

/* Ethernet_Media_Interface */
#define GPIO_ETH_MediaInterface_MII     ((u32)0x00000000)
#define GPIO_ETH_MediaInterface_RMII    ((u32)0x00000001)


// void delay_ms(uint32_t d) {
//   for (uint32_t i = 0; i < (d * 1300); i++) {
//     __asm__( "nop; nop; nop" );
//   }
// }

#endif
