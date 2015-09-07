#include "sx1276_misc.h"
#include "delay.h"

#define SX1276_FREQUENCY_STEP       (61.03515625)
#define SX1276_FREQUENCY_MIN        (868000000)
#define SX1276_FREQUENCY_MAX        (915000000)

#define CHECK(VALUE, VALUE_MIN, VALUE_MAX)          { if(VALUE > VALUE_MAX) { VALUE = VALUE_MAX; } else if(VALUE < VALUE_MIN) { VALUE = VALUE_MIN; } }
#define IS_BOUNDS(VALUE)                            CHECK(VALUE, SX1276_FREQUENCY_MIN, SX1276_FREQUENCY_MAX)

void sx1276_set_modem()
{
    sx1276_set_mode(sx1276_mode_sleep);
    
    sx1276_cfg_write(sx1276_cfg_reg_mode, 0x80);

    sx1276_cfg_write(sx1276_cfg_reg_dio_map_1, 0x00);
    sx1276_cfg_write(sx1276_cfg_reg_dio_map_2, 0x00);
    
    sx1276_set_mode(sx1276_mode_standby);
}

uint8_t sx1276_get_reg(uint8_t d)
{
    return sx1276_cfg_read(d);
}

uint32_t sx1276_get_frequency()
{
    uint8_t d[3];
    uint32_t freq = 0;

    sx1276_cfg_read_data(sx1276_cfg_reg_rf, d, 3);

    freq = ((uint32_t)d[0] << 16) |
            ((uint32_t)d[1] << 8) |
            ((uint32_t)d[2]);
    freq = (uint32_t)((float)freq * (float)SX1276_FREQUENCY_STEP);

    return freq;
}

void sx1276_set_frequency(uint32_t d)
{
    uint8_t data[3];
    
    IS_BOUNDS(d);

    d = (unsigned int)((float)d / (float)SX1276_FREQUENCY_STEP);

    data[0] = (unsigned int)((d >> 16) & 0xFF);
    data[3] = (unsigned int)((d >> 8) & 0xFF);
    data[2] = (unsigned int)(d & 0xFF);

    sx1276_cfg_write_data(sx1276_cfg_reg_rf, data, 3);
}

void sx1276_set_power(uint8_t d)
{
    uint8_t pa = sx1276_cfg_read(sx1276_cfg_reg_pa);
    uint8_t pa_dac = sx1276_cfg_read(sx1276_cfg_reg_pa_dac);

    pa = (pa & 0x0F) | 0xF0;

    if(d > 17)
    {
        pa_dac = 0x87;
        CHECK(d, 5, 20);
        d -= 5;
    }
    else
    {
        pa_dac = 0x84;
        CHECK(d, 2, 17);
        d -= 2;
    }

    pa = (pa & 0xF0) | (d & 0x0F);
    
    sx1276_cfg_write(sx1276_cfg_reg_pa, pa);
    sx1276_cfg_write(sx1276_cfg_reg_pa_dac, pa_dac);
}

uint8_t sx1276_get_bandwidth()
{
    uint8_t d = sx1276_cfg_read(sx1276_cfg_reg_conf);
    
    return ((d & 0xF0) >> 4);
}

void sx1276_set_bandwidth(sx1276_bw_t d)
{
    uint8_t reg = sx1276_cfg_read(sx1276_cfg_reg_conf);

    reg = (reg & 0x0F) | (d << 4);

    sx1276_cfg_write(sx1276_cfg_reg_conf, reg);
}

uint8_t sx1276_get_sf()
{
    uint8_t sp = sx1276_cfg_read(sx1276_cfg_reg_conf_2);

    return ((sp & 0xF0) >> 4);
}

void sx1276_set_sf(sx1276_sf_t d)
{
    uint8_t reg = sx1276_cfg_read(sx1276_cfg_reg_conf_2);

    reg = (reg & 0x0F) | (d << 4);

    sx1276_cfg_write(sx1276_cfg_reg_conf_2, reg);
}

uint8_t sx1276_get_coding_rate()
{
    uint8_t d = sx1276_cfg_read(sx1276_cfg_reg_conf);

    return ((d & 0x0E) >> 1);
}

void sx1276_set_coding_rate(sx1276_rate_t d)
{
    uint8_t reg = sx1276_cfg_read(sx1276_cfg_reg_conf);

    reg = (reg & 0xF1) | (d << 1);

    sx1276_cfg_write(sx1276_cfg_reg_conf, reg);
}

bool sx1276_get_crc()
{
    uint8_t crc = sx1276_cfg_read(sx1276_cfg_reg_conf_2);

    return (((crc & 0x04) == 0x04) ? true : false);
}

void sx1276_set_crc(bool status)
{
    uint8_t reg = sx1276_cfg_read(sx1276_cfg_reg_conf_2);

    reg = (reg & 0xFB) | (status << 2);

    sx1276_cfg_write(sx1276_cfg_reg_conf_2, reg);
}

uint16_t sx1276_get_preamble()
{
    uint8_t data[2];
	
    sx1276_cfg_read_data(sx1276_cfg_reg_preamb, data, 2);
	
    return (((uint16_t)data[0] & 0x00FF) << 8) | data[1];
}

void sx1276_set_preamble(uint16_t d)
{
    uint8_t data[2];
    
    CHECK(d, 0x04, 0xFFFF);
	
    data[0] = (d >> 8) & 0xFF;
    data[1] = d & 0xFF;
	
    sx1276_cfg_write_data(sx1276_cfg_reg_preamb, data, 2);
}

bool sx1276_get_header_mode()
{
    uint8_t mode = sx1276_cfg_read(sx1276_cfg_reg_conf);
	
    return (((mode & 0x01) == 0x01) ? true : false);
}

void sx1276_set_header_mode(bool status)
{
    uint8_t data = sx1276_cfg_read(sx1276_cfg_reg_conf);
	
    data = (data & 0xFE) | (uint8_t)status;
	
    sx1276_cfg_write(sx1276_cfg_reg_conf, data);
}

uint8_t sx1276_get_hop_period()
{
    return sx1276_cfg_read(sx1276_cfg_reg_hop_period);
}

void sx1276_set_hop_period(uint8_t d)
{
    sx1276_cfg_write(sx1276_cfg_reg_hop_period, d);
}

uint8_t sx1276_get_payload()
{
    return sx1276_cfg_read(sx1276_cfg_reg_payload);
}

void sx1276_set_payload(uint8_t d)
{
    CHECK(d, 0, 255);
	
    sx1276_cfg_write(sx1276_cfg_reg_payload, d);
}

uint8_t sx1276_get_rx_number()
{
    return sx1276_cfg_read(sx1276_cfg_reg_rx_number);
}

void sx1276_set_full_buf_tx()
{
    sx1276_cfg_write(sx1276_cfg_reg_tx_addr, 0);
    sx1276_cfg_write(sx1276_cfg_reg_addr_ptr, 0);
}

void sx1276_set_full_buf_rx()
{
    sx1276_cfg_write(sx1276_cfg_reg_rx_addr, 0);
    sx1276_cfg_write(sx1276_cfg_reg_addr_ptr, 0);
}

bool sx1276_get_pa_boost()
{
    uint8_t data = sx1276_cfg_read(sx1276_cfg_reg_pa_dac);

    return (((data & 0x07) == 0x07) ? true : false);
}

void sx1276_set_pa_boost(bool status)
{
    uint8_t pa = sx1276_cfg_read(sx1276_cfg_reg_pa_dac);
    uint8_t pa_dac = sx1276_cfg_read(sx1276_cfg_reg_pa_dac);
	
    if((pa & 0x80) == 0x80)
    {
        if(status)
        {
            pa_dac = 0x87;
	}
    }
    else
    {
        pa_dac = 0x84;
    }

    sx1276_cfg_write(sx1276_cfg_reg_pa_dac, pa_dac);
}

bool sx1276_get_low_data_rate_optimize()
{
    uint8_t data = sx1276_cfg_read(sx1276_cfg_reg_conf_3);
	
    return (((data & 0x08) == 0x08) ? true : false);
}

void sx1276_set_low_data_rate_optimize(bool status)
{
    uint8_t data = sx1276_cfg_read(sx1276_cfg_reg_conf_3);
	
    data = (data & 0xF7) | (status << 3);
	
    sx1276_cfg_write(sx1276_cfg_reg_conf_3, data);
}

uint8_t sx1276_get_mode()
{
    uint8_t data = sx1276_cfg_read(sx1276_cfg_reg_mode);
	
    return (data & 0x07);
}

void sx1276_set_mode(sx1276_mode_t d)
{
    uint8_t data = sx1276_cfg_read(sx1276_cfg_reg_mode);
	
    data = (data & 0xF8) | (d & 0x07);
	
    sx1276_cfg_write(sx1276_cfg_reg_mode, data);
}

int32_t sx1276_get_pkt_rssi()
{
    return (-157 + sx1276_cfg_read(sx1276_cfg_reg_pkt_rssi));
}

int32_t sx1276_get_rssi()
{
    return (-157 + sx1276_cfg_read(sx1276_cfg_reg_rssi));
}

void sx1276_set_timeout(uint16_t d)
{
    uint8_t reg = sx1276_cfg_read(sx1276_cfg_reg_conf_2);
    reg = (reg & 0xFC) | ((d >> 8) & 0x03);
	
    sx1276_cfg_write(sx1276_cfg_reg_conf_2, reg);
    sx1276_cfg_write(sx1276_cfg_reg_timeout, d & 0xFF);
}

void sx1276_detect_opt()
{
    uint8_t d = sx1276_cfg_read(sx1276_cfg_reg_pkt_conf_2) & 0xF8;
	
    /*sx1276_cfg_write(sx1276_cfg_reg_pkt_conf_2, d | 0x05);
    sx1276_cfg_write(sx1276_cfg_reg_seq_conf_2, 0x0C); for SP = 6*/

    sx1276_cfg_write(sx1276_cfg_reg_pkt_conf_2, d | 0x03);
    sx1276_cfg_write(sx1276_cfg_reg_seq_conf_2, 0x0A);
}
