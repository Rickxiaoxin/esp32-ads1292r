
idf_component_register(SRCS "wifiudp.c"
                       INCLUDE_DIRS "include"
                       PRIV_REQUIRES "esp_wifi" "lwip" "nvs_flash")
