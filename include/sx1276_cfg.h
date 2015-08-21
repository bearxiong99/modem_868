#ifndef SX1276_CFG_H
#define SX1276_CFG_H

#include <stdint.h>

#define SX1276_CFG_EXTI_LINE_DIO_0        EXTI_Line15
#define SX1276_CFG_EXTI_LINE_DIO_1        EXTI_Line7

typedef enum
{
    sx1276_cfg_reg_fifo         = 0x00,
    sx1276_cfg_reg_mode         = 0x01,
    sx1276_cfg_reg_bitrate      = 0x02,
    sx1276_cfg_reg_rf           = 0x06,
    sx1276_cfg_reg_pa           = 0x09,
    sx1276_cfg_reg_pa_ramp      = 0x0A,
    sx1276_cfg_reg_addr_ptr     = 0x0D,
    sx1276_cfg_reg_tx_addr      = 0x0E,
    sx1276_cfg_reg_rx_addr      = 0x0F,
    sx1276_cfg_reg_irq_fmask    = 0x11,
    sx1276_cfg_reg_irq_flags    = 0x12,
    sx1276_cfg_reg_rx_number    = 0x13,
    sx1276_cfg_reg_snr          = 0x19,
    sx1276_cfg_reg_pkt_rssi     = 0x1A,
    sx1276_cfg_reg_rssi         = 0x1B,
    sx1276_cfg_reg_conf         = 0x1D,
    sx1276_cfg_reg_conf_2       = 0x1E,
    sx1276_cfg_reg_timeout      = 0x1F,
    sx1276_cfg_reg_preamb       = 0x20,
    sx1276_cfg_reg_payload      = 0x22,
    sx1276_cfg_reg_hop_period   = 0x24,
    sx1276_cfg_reg_fifo_rx      = 0x25,
    sx1276_cfg_reg_conf_3       = 0x26,
    sx1276_cfg_reg_pkt_conf_1   = 0x30,
    sx1276_cfg_reg_pkt_conf_2   = 0x31,
    sx1276_cfg_reg_seq_conf_2   = 0x37,
    sx1276_cfg_reg_calibr       = 0x3B,
    sx1276_cfg_reg_dio_map_1    = 0x40,
    sx1276_cfg_reg_dio_map_2    = 0x41,
    sx1276_cfg_reg_version      = 0x42,
    sx1276_cfg_reg_pa_dac       = 0x4D
} sx1276_cfg_reg_t;

void sx1276_cfg_init();

void sx1276_cfg_write_data(sx1276_cfg_reg_t r, uint8_t *d, uint8_t s);

void sx1276_cfg_write(sx1276_cfg_reg_t r, uint8_t d);

void sx1276_cfg_read_data(sx1276_cfg_reg_t r, uint8_t *d, uint8_t s);

uint8_t sx1276_cfg_read(sx1276_cfg_reg_t reg);

void sx1276_cfg_write_fifo(uint8_t *d, uint8_t s);

void sx1276_cfg_read_fifo(uint8_t *d, uint8_t s);

#endif