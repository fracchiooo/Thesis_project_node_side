idf_component_register(
    SRCS "pwmController.cpp"
    INCLUDE_DIRS "."
    REQUIRES driver esp_timer
)