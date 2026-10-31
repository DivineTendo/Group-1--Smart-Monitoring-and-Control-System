idf_component_register(
    SRCS "main.c"
    REQUIRES driver esp_adc nvs_flash fatfs sdmmc mbedtls esp_timer esp_rom esp_system
)

# IDF builds with -Werror; keep warnings visible but do not let them stop the build
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error)