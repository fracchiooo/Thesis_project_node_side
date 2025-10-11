idf_component_register(
    SRCS "wifi_wrapper.cpp"
    INCLUDE_DIRS "."
    REQUIRES driver esp_event esp_wifi esp_timer nvs_flash
)