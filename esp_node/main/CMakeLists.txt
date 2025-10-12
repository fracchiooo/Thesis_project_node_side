idf_component_register(SRCS "esp_node.cpp"
                    INCLUDE_DIRS "."
                    REQUIRES driver nvs_flash temperature pwmController wifi mqtts json esp_netif
                    )

target_compile_features(${COMPONENT_LIB} PUBLIC cxx_std_17)