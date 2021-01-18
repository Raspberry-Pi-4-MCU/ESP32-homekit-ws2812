#include "sht20.h"
#define GPIO_PULLUP_ENABLE 1
#define I2C_NUM I2C_NUM_0
#define RH_HOLD 0xE5
#define TEMP_HOLD 0xE3

void sht20_initial(int scl_pin, int sda_pin){
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = sda_pin,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = scl_pin,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 1000000,
    };
    i2c_param_config(I2C_NUM, &conf);
    i2c_driver_install(I2C_NUM, I2C_MODE_MASTER, 0 ,0, 0);
}

float readhum(void){
    uint8_t hum[3] = {0};
    uint8_t *ptr_hum = hum;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SHT20_ADDRESS << 1) | 0x00, 1);
    i2c_master_write_byte(cmd, TEMP_HOLD, 1);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SHT20_ADDRESS << 1) | 0x01, 1);
    i2c_master_read_byte(cmd, ptr_hum, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, ptr_hum + 1, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, ptr_hum + 2, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ((hum[0] << 8) | hum[1]) / (1 << 16) * 125.0 - 6.0;
}

float readtemp(void){
    uint8_t temp[3] = {0};
    uint8_t *ptr_temp = temp;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SHT20_ADDRESS << 1) | 0x00, 1);
    i2c_master_write_byte(cmd, TEMP_HOLD, 1);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SHT20_ADDRESS << 1) | 0x01, 1);
    i2c_master_read_byte(cmd, ptr_temp, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, ptr_temp + 1, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, ptr_temp + 2, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_NUM, cmd, 1000 / portTICK_RATE_MS);
    i2c_cmd_link_delete(cmd);
    return ((temp[0] << 8) | temp[1]) / (1 << 16) * 175.72 - 46.85;
}

temphum readtemphum(void){
    temphum temphumobj;
    temphumobj.hum = readhum();
    temphumobj.temp = readtemp();
    return temphumobj;
}