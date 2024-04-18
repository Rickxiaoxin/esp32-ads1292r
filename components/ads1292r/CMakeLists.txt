
idf_component_register(SRCS "ads1292r.c"
                       INCLUDE_DIRS "include"
                       REQUIRES "driver" "wifiudp" "esp_event"
                       PRIV_REQUIRES "driver" "lwip" "esp_wifi" "nvs_flash" "esp-dsp")
