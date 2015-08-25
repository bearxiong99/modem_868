#ifndef SX1276_MISC_H
#define SX1276_MISC_H

#include "sx1276_cfg.h"

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    sx1276_mode_sleep        = 0,
    sx1276_mode_standby      = 1,
    sx1276_mode_fstx         = 2,
    sx1276_mode_tx           = 3,
    sx1276_mode_fsrx         = 4,
    sx1276_mode_rxc          = 5,
    sx1276_mode_rx           = 6,
    sx1276_mode_cad          = 7
} sx1276_mode_t;

typedef enum
{
    sx1276_bw_7_8            = 0,
    sx1276_bw_10_4           = 1,
    sx1276_bw_15_6           = 2,
    sx1276_bw_20_8           = 3,
    sx1276_bw_31_25          = 4,
    sx1276_bw_41_7           = 5,
    sx1276_bw_62_5           = 6,
    sx1276_bw_125            = 7,
    sx1276_bw_250            = 8,
    sx1276_bw_500            = 9
} sx1276_bw_t;

typedef enum
{
    sx1276_sf_64             = 6,
    sx1276_sf_128            = 7,
    sx1276_sf_256            = 8,
    sx1276_sf_512            = 9,
    sx1276_sf_1024           = 10,
    sx1276_sf_2048           = 11,
    sx1276_sf_4096           = 12
} sx1276_sf_t;

typedef enum
{
    sx1276_rate_4_5          = 1,
    sx1276_rate_4_6          = 2,
    sx1276_rate_4_7          = 3,
    sx1276_rate_4_8          = 4
} sx1276_rate_t;

void sx1276_set_modem();

uint8_t sx1276_get_reg(uint8_t d);

uint32_t sx1276_get_frequency();

void sx1276_set_frequency(uint32_t d);

void sx1276_set_power(uint8_t d);

uint8_t sx1276_get_bandwidth();

void sx1276_set_bandwidth(uint8_t d);

uint8_t sx1276_get_sf();

void sx1276_set_sf(sx1276_sf_t d);

uint8_t sx1276_get_coding_rate();

void sx1276_set_coding_rate(sx1276_rate_t d);

bool sx1276_get_crc();

void sx1276_set_crc(bool status);

uint16_t sx1276_get_preamble();

void sx1276_set_preamble(uint16_t d);

bool sx1276_get_header_mode();

void sx1276_set_header_mode(bool status);

uint8_t sx1276_get_hop_period();

void sx1276_set_hop_period(uint8_t d);

uint8_t sx1276_get_payload();

void sx1276_set_payload(uint8_t d);

uint8_t sx1276_get_rx_number();

void sx1276_set_full_buf_tx();

void sx1276_set_full_buf_rx();

bool sx1276_get_pa_boost();

void sx1276_set_pa_boost(bool status);

bool sx1276_get_low_data_rate_optimize();

void sx1276_set_low_data_rate_optimize(bool status);

uint8_t sx1276_get_mode();

void sx1276_set_mode(sx1276_mode_t d);

int32_t sx1276_get_pkt_rssi();

int32_t sx1276_get_rssi();

void sx1276_set_timeout(uint16_t d);

void sx1276_detect_opt();

#endif