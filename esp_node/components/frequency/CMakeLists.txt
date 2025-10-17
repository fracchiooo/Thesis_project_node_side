idf_component_register(
    SRCS "fft_wrapper.cpp"
    INCLUDE_DIRS "."
    REQUIRES driver esp-dsp esp_adc
)