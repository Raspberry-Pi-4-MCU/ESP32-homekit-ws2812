#include "wificonfig.h"

wifi_config_t wificonfig_read(void)
{
    nvs_handle_t handle;
    wifi_config_t wifi_config_stored;
    ESP_ERROR_CHECK(nvs_open(NVS_CUSTOMER, NVS_READWRITE, &handle));
    memset(&wifi_config_stored, 0x0, sizeof(wifi_config_stored));
    uint32_t len = sizeof(wifi_config_stored);
    ESP_ERROR_CHECK(nvs_get_blob(handle, NVS_REGION, &wifi_config_stored, &len));
    nvs_close(handle);
    return wifi_config_stored;
}

void wificonfig_write(wifi_config_t wifi_config)
{
    nvs_handle_t handle;
    ESP_ERROR_CHECK(nvs_open(NVS_CUSTOMER, NVS_READWRITE, &handle));
    ESP_ERROR_CHECK(nvs_set_blob(handle, NVS_REGION, &wifi_config, sizeof(wifi_config)));
    nvs_close(handle);
}

void wificonfig_initial(void)
{
    netwrok_connect_signal = xQueueCreate(10, sizeof(int));
}
