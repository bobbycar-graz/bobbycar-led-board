file(
    GLOB FILES
    *.h
    */*.h
    *.cpp
    */*.cpp
)

set(dependencies
    freertos nvs_flash esp_http_server esp_system driver
    arduino-esp32 ArduinoJson cpputils cxx-ring-buffer date
    espchrono espcpputils espconfiglib esphttpdutils
    espwifistack expected fmt
)

idf_component_register(
    SRCS ${FILES}
    INCLUDE_DIRS .
    REQUIRES ${dependencies}
)

target_compile_options(${COMPONENT_TARGET}
    PRIVATE
        -fstack-reuse=all
        -fstack-protector-all
        -fdiagnostics-color=always
        -Wno-unused-function
        -Wno-deprecated-declarations
        -Wno-missing-field-initializers
        -Wno-parentheses
)
