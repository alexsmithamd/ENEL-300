idf_component_register(SRCS "slave.c" 
                    PRIV_REQUIRES esp_driver_gpio esp_timer esp_adc
                       INCLUDE_DIRS ".")
