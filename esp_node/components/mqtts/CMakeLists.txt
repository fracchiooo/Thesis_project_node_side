idf_component_register(
    SRCS "mqtt_wrapper.cpp"
    INCLUDE_DIRS "."
    REQUIRES esp_event mqtt esp_hw_support pwmController
    EMBED_TXTFILES "certs/emqxsl-ca.crt"
)