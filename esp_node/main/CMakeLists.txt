idf_component_register(SRCS "esp_node.c"
                    INCLUDE_DIRS "."
                    REQUIRES driver nvs_flash temperature pwmController
                    )

target_compile_features(${COMPONENT_LIB} PUBLIC cxx_std_17)