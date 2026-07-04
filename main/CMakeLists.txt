idf_component_register(SRCS "perfusorESP32.c"
                       PRIV_REQUIRES spi_flash
                                     driver
                                     esp_driver_gpio
                                     esp_driver_gptimer
                                     esp_driver_touch_sens
                                     esp_lcd
                                     esp_timer
                                     nvs_flash
                       INCLUDE_DIRS "")
