idf_component_register(
    SRCS "sensor.cpp" "actuator.cpp"
    INCLUDE_DIRS "."
    REQUIRES onewire_bus driver pwmController
)