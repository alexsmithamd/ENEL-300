idf_component_register(SRCS "espnow_example_main_ultrasonic.c"
                    PRIV_REQUIRES nvs_flash esp_event esp_netif esp_wifi esp_driver_gpio esp_driver_ledc esp_driver_i2c esp_timer
                    INCLUDE_DIRS ".")
