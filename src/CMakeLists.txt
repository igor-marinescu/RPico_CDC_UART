cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)

project(rpico_cdc_uart)
pico_sdk_init()

# Export compilation commands "compile_commands.json" to be used by PC-lint
# Comment the following line if PC-Lint is not required
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Project 
add_definitions(-DPROJECT_NAME="${PROJECT_NAME}")

# Debug/Logging: include and printf/dump functions
add_compile_definitions(DEBUG_INCLUDE="uart_ascii.h")       # include        
add_compile_definitions(DEBUG_PRINTF=uart_ascii_printf0)    # function(__VA_ARGS__)
add_compile_definitions(DEBUG_DUMP=uart_ascii_dump)         # function(buff, len, addr)

# Debug/Logging: uncomment if a module has to debug
add_compile_definitions(MAIN_DEBUG)
add_compile_definitions(UART_DRV_DEBUG)
add_compile_definitions(PUART_DRV_DEBUG)

# Use PIO-UART instead of UART
add_compile_definitions(USE_PIO_UART)
#add_compile_definitions(PIO_TX_BIT_CNT=10)

# TX-ACTIVE Output Signal
add_compile_definitions(TX_ACTIVE_ENABLED)

# Don't allow USB to send small packets for each received UART byte.
# Send UART->USB when no further data is received from UART for more than 
# <UART_RX_BYTES_TOUT> bytes time (depending on baud rate) 
# or when >= 64 bytes have been received. 
add_compile_definitions(UART_RX_BYTES_TOUT=2)

add_executable(rpico_cdc_uart)
target_sources(rpico_cdc_uart PUBLIC
    main.c
    usb_descriptors.c
    usb_main.c
    gpio_drv.c
    uart_drv.c
    uart_ascii.c
    cli.c
    utils.c
    puart_drv.c
)

# Make sure TinyUSB can find tusb_config.h
target_include_directories(rpico_cdc_uart PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}
)

# In addition to pico_stdlib required for common PicoSDK functionality, add dependency on tinyusb_device
# for TinyUSB device support and tinyusb_board for the additional board support library used by the example
target_link_libraries(rpico_cdc_uart PUBLIC 
    pico_stdlib 
    tinyusb_device 
    tinyusb_board
    pico_multicore
    hardware_uart
    pico_stdlib 
    hardware_pio
)

# by default the header is generated into the build dir
pico_generate_pio_header(rpico_cdc_uart ${CMAKE_CURRENT_LIST_DIR}/puart_tx.pio)

pico_add_extra_outputs(rpico_cdc_uart)

#-------------------------------------------------------------------------------
# Deploy
#-------------------------------------------------------------------------------
set(RPICO_MOUNT_POINT "/media/$ENV{USER}/RPI-RP2/")
add_custom_target(
    deploy
    COMMAND ${CMAKE_COMMAND} -E echo "Deploying to pico..."
    #COMMAND cp "${PROJECT_NAME}.uf2" /media/igor/RPI-RP2/
    COMMAND python3 ../scripts/firmware_update.py -m "${RPICO_MOUNT_POINT}" "${PROJECT_NAME}.uf2"
    DEPENDS rpico_cdc_uart
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
