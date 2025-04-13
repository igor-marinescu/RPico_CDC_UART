cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)

project(cdc_template)
pico_sdk_init()

# Export compilation commands "compile_commands.json" to be used by PC-lint
# Comment the following line if PC-Lint is not required
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

add_definitions(-DPROJECT_NAME="${PROJECT_NAME}")

# Debug/Logging: modify the following definitions for include header, printf 
# and dump function of the Debug module
add_compile_definitions(DEBUG_INCLUDE="uart_ascii.h")       # include        
add_compile_definitions(DEBUG_PRINTF=uart_ascii_printf0)    # function(__VA_ARGS__)
add_compile_definitions(DEBUG_DUMP=uart_ascii_dump)         # function(buff, len, addr)

add_compile_definitions(MAIN_DEBUG)
add_compile_definitions(UART_DRV_DEBUG)

add_executable(cdc_template)
target_sources(cdc_template PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/main.c
    ${CMAKE_CURRENT_LIST_DIR}/usb_descriptors.c
    ${CMAKE_CURRENT_LIST_DIR}/usb_main.c
    ${CMAKE_CURRENT_LIST_DIR}/gpio_drv.c
    ${CMAKE_CURRENT_LIST_DIR}/uart_drv.c
    ${CMAKE_CURRENT_LIST_DIR}/uart_ascii.c
    ${CMAKE_CURRENT_LIST_DIR}/cli.c
    ${CMAKE_CURRENT_LIST_DIR}/utils.c
)

# Make sure TinyUSB can find tusb_config.h
target_include_directories(cdc_template PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)

# In addition to pico_stdlib required for common PicoSDK functionality, add dependency on tinyusb_device
# for TinyUSB device support and tinyusb_board for the additional board support library used by the example
target_link_libraries(cdc_template PUBLIC 
    pico_stdlib 
    tinyusb_device 
    tinyusb_board
    pico_multicore
    hardware_uart 
)

pico_add_extra_outputs(cdc_template)

#-------------------------------------------------------------------------------
# Deploy
#-------------------------------------------------------------------------------
add_custom_target(
    deploy
    COMMAND ${CMAKE_COMMAND} -E echo "Deploying to pico..."
    COMMAND cp cdc_template.uf2 /media/igor/RPI-RP2/
    DEPENDS cdc_template
    COMMENT "Deploying the firmware to pico"
)

#-------------------------------------------------------------------------------
# PC-Lint
#-------------------------------------------------------------------------------
set(PCLINT_CONFIG "/home/igor/pclp/config/pclp_config.py")
set(PCLINT_GCC "/usr/bin/gcc")
add_custom_target(
    pclint
    COMMAND python3 ${PCLINT_CONFIG} --compiler=gcc --compiler-bin=${PCLINT_GCC} --config-output-lnt-file=co-gcc.lnt --config-output-header-file=co-gcc.h --generate-compiler-config
    COMMAND python3 ${PCLINT_CONFIG} --compiler=gcc --compilation-db=compile_commands.json --config-output-lnt-file=${PROJECT_NAME}_pclint.lnt --generate-project-config
    COMMAND pclp64_linux co-gcc.lnt ${PROJECT_NAME}_pclint.lnt > ${PROJECT_NAME}_pclint.out
    DEPENDS cdc_template
)
