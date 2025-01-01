file(GLOB SEN66_SOURCES "sen66/*.c")

idf_component_register(
    SRCS 
    ${SEN66_SOURCES}
    "sen66_esp32.c"
    "esp_zb_sen66.c"
    INCLUDE_DIRS "."
)
