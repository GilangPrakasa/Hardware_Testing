set(COMPONENT_SRCDIRS
    "SerialFlash"
)

set(COMPONENT_ADD_INCLUDEDIRS
    "SerialFlash"
)

set(COMPONENT_REQUIRES
    "arduino-esp32"
)

register_component()

target_compile_options(${COMPONENT_LIB} PRIVATE 
    -DARDUINO=10800 -DESP32=1 -DARDUINO_ARCH_ESP32=1 -DARDUINOJSON_USE_LONG_LONG=1
)


