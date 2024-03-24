set(COMPONENT_SRCS
    "main.cpp"
    "Flash_AVR/ElegantOTA.cpp"
    "Flash_AVR/ImageAVR.cpp" 
    "Flash_AVR/Adafruit_AVRProg.cpp"
    "Peripheral/motor_controller.c"
    "Peripheral/beeper.c"
    "Peripheral/led_status.c"
    "Peripheral/runbtn.c"
    "Peripheral/LiquidCrystal_I2C.cpp"
    "Peripheral/lcd.c"
    "Peripheral/eventbus.c"
    "Peripheral/ext_wdt.c"
)

set(COMPONENT_ADD_INCLUDEDIRS
    "."
    "Flash_AVR"
    "peripheral"
)

register_component()

target_compile_options(${COMPONENT_LIB} PRIVATE 
)
