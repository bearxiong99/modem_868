#include "sx1276_cfg.h"

#include "stm32f4xx.h"

#define SX1276_CFG_SPI_SCK                  GPIO_Pin_10
#define SX1276_CFG_SPI_MISO                 GPIO_Pin_11
#define SX1276_CFG_SPI_M0SI                 GPIO_Pin_12
#define SX1276_CFG_SPI_LINE                 GPIOC
#define SX1276_CFG_GPIO_AF                  GPIO_AF_SPI3
#define SX1276_CFG_GPIO_AF_SCK              GPIO_PinSource10
#define SX1276_CFG_GPIO_AF_MISO             GPIO_PinSource11
#define SX1276_CFG_GPIO_AF_M0SI             GPIO_PinSource12
#define SX1276_CFG_SPI_SEL_LINE             GPIOD
#define SX1276_CFG_SPI_SEL_PIN              GPIO_Pin_7
#define SX1276_CFG_EXTI_GPIO_DIO_0          GPIOD
#define SX1276_CFG_EXTI_PIN_DIO_0           GPIO_Pin_15
#define SX1276_CFG_EXTI_GPIO_DIO_1          GPIOB
#define SX1276_CFG_EXTI_PIN_DIO_1           GPIO_Pin_7
#define SX1276_CFG_SYSCFG_PORT_DIO_0        EXTI_PortSourceGPIOD
#define SX1276_CFG_SYSCFG_PIN_DIO_0         EXTI_PinSource15
#define SX1276_CFG_SYSCFG_PORT_DIO_1        EXTI_PortSourceGPIOB
#define	SX1276_CFG_SYSCFG_PIN_DIO_1         EXTI_PinSource7
#define SX1276_CFG_EXTI_DIO_0               EXTI15_10_IRQn
#define SX1276_CFG_EXTI_DIO_1               EXTI9_5_IRQn
#define SX1276_CFG_SPI_INTERFACE            SPI3

static void gpio_init()
{
    GPIO_InitTypeDef gpio;
    
    GPIO_StructInit(&gpio);
    
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_DOWN;
    gpio.GPIO_Pin = SX1276_CFG_SPI_SCK |
                   SX1276_CFG_SPI_MISO | SX1276_CFG_SPI_M0SI;
                   
    GPIO_Init(SX1276_CFG_SPI_LINE, &gpio);
    
    GPIO_PinAFConfig(SX1276_CFG_SPI_LINE, 
                     SX1276_CFG_GPIO_AF_SCK, 
                     SX1276_CFG_GPIO_AF);
    GPIO_PinAFConfig(SX1276_CFG_SPI_LINE, 
                     SX1276_CFG_GPIO_AF_MISO, 
                     SX1276_CFG_GPIO_AF);
    GPIO_PinAFConfig(SX1276_CFG_SPI_LINE, 
                     SX1276_CFG_GPIO_AF_M0SI, 
                     SX1276_CFG_GPIO_AF);
    
    gpio.GPIO_Mode = GPIO_Mode_OUT;
    gpio.GPIO_OType = GPIO_OType_PP;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    gpio.GPIO_Pin = SX1276_CFG_SPI_SEL_PIN;
    
    GPIO_Init(SX1276_CFG_SPI_SEL_LINE, &gpio);
    
    gpio.GPIO_Mode = GPIO_Mode_IN;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_DOWN;
    gpio.GPIO_Pin = SX1276_CFG_EXTI_PIN_DIO_0;

    GPIO_Init(SX1276_CFG_EXTI_GPIO_DIO_0, &gpio);
    
    gpio.GPIO_Mode = GPIO_Mode_IN;
    gpio.GPIO_Speed = GPIO_Speed_50MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_DOWN;
    gpio.GPIO_Pin = SX1276_CFG_EXTI_PIN_DIO_1;

    GPIO_Init(SX1276_CFG_EXTI_GPIO_DIO_1, &gpio);
    
    SYSCFG_EXTILineConfig(SX1276_CFG_SYSCFG_PORT_DIO_0,
                          SX1276_CFG_SYSCFG_PIN_DIO_0);
    SYSCFG_EXTILineConfig(SX1276_CFG_SYSCFG_PORT_DIO_1,
                          SX1276_CFG_SYSCFG_PIN_DIO_1);
    
    GPIO_ResetBits(SX1276_CFG_SPI_LINE, SX1276_CFG_SPI_SCK |
                                        SX1276_CFG_SPI_MISO |
                                        SX1276_CFG_SPI_M0SI);
}

static void spi_init()
{
    SPI_InitTypeDef spi;

    SPI_StructInit(&spi);

    spi.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_8;
    spi.SPI_CPHA = SPI_CPHA_1Edge;
    spi.SPI_CPOL = SPI_CPOL_Low;
    spi.SPI_CRCPolynomial = 7;
    spi.SPI_DataSize = SPI_DataSize_8b;
    spi.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    spi.SPI_FirstBit = SPI_FirstBit_MSB;
    spi.SPI_Mode = SPI_Mode_Master;
    spi.SPI_NSS = SPI_NSS_Soft;
    
    SPI_Init(SX1276_CFG_SPI_INTERFACE, &spi);

    SPI_Cmd(SX1276_CFG_SPI_INTERFACE, ENABLE);
}

static void nvic_init()
{
    NVIC_InitTypeDef nvic;

    nvic.NVIC_IRQChannel = SX1276_CFG_EXTI_DIO_0;
    nvic.NVIC_IRQChannelPreemptionPriority = 6;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;

    NVIC_Init(&nvic);
    
    nvic.NVIC_IRQChannel = SX1276_CFG_EXTI_DIO_1;
    nvic.NVIC_IRQChannelPreemptionPriority = 6;
    nvic.NVIC_IRQChannelSubPriority = 0;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    
    NVIC_Init(&nvic);
}

static void exti_set(uint8_t status)
{
    EXTI_InitTypeDef exti;
    
    exti.EXTI_Line = SX1276_CFG_EXTI_LINE_DIO_0 | SX1276_CFG_EXTI_LINE_DIO_1;
    exti.EXTI_Mode = EXTI_Mode_Interrupt;
    exti.EXTI_Trigger = EXTI_Trigger_Rising;
    exti.EXTI_LineCmd = status;
    
    EXTI_Init(&exti);
}

static uint8_t spi_send(uint8_t d)
{
    SPI_I2S_SendData(SX1276_CFG_SPI_INTERFACE, d);
    while(SPI_I2S_GetFlagStatus(SX1276_CFG_SPI_INTERFACE, SPI_I2S_FLAG_RXNE) == RESET);

    return SPI_I2S_ReceiveData(SX1276_CFG_SPI_INTERFACE);
}

inline static void spi_select(uint8_t status)
{
    GPIO_WriteBit(SX1276_CFG_SPI_SEL_LINE, SX1276_CFG_SPI_SEL_PIN, !status);
}

void sx1276_cfg_init()
{
    gpio_init();
    spi_init();
    exti_set(ENABLE);
    nvic_init();
}

void sx1276_cfg_write_data(sx1276_cfg_reg_t r, uint8_t *d, uint8_t s)
{
    spi_select(ENABLE);

    spi_send(r | 0x80);

    while(s)
    {
        spi_send(*d);
        d++; s--;
    }

    spi_select(DISABLE);
}

void sx1276_cfg_write(sx1276_cfg_reg_t r, uint8_t d)
{
    sx1276_cfg_write_data(r, &d, 1);
}

void sx1276_cfg_read_data(sx1276_cfg_reg_t r, uint8_t *d, uint8_t s)
{
    spi_select(ENABLE);

    spi_send(r & 0x7F);

    while(s)
    {
        *d = spi_send(0x00);
        d++; s--;
    }
    
    spi_select(DISABLE);
}

uint8_t sx1276_cfg_read(sx1276_cfg_reg_t r)
{
    uint8_t d = 0;

    sx1276_cfg_read_data(r, &d, 1);

    return d;
}

void sx1276_cfg_write_fifo(uint8_t *d, uint8_t s)
{
    sx1276_cfg_write_data(sx1276_cfg_reg_fifo, d, s);
}

void sx1276_cfg_read_fifo(uint8_t *d, uint8_t s)
{
    sx1276_cfg_read_data(sx1276_cfg_reg_fifo, d, s);
}