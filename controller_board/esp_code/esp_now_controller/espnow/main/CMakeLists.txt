idf_component_register(SRCS "controller_modified_with_display.c"
                    PRIV_REQUIRES nvs_flash esp_event esp_netif esp_wifi esp_timer esp_adc
                    INCLUDE_DIRS ".")
