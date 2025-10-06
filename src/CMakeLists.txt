cmake_minimum_required(VERSION 3.13)
include(pico_sdk_import.cmake)

project(rpico_cdc_uart)
pico_sdk_init()

# Export compilation commands "compile_commands.json" to be used by PC-lint
# Comment the following line if PC-Lint is not required
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Project 
add_compile_definitions(PROJECT_NAME="${PROJECT_NAME}")

# Debug/Logging: include and printf/dump functions
add_compile_definitions(DEBUG_INCLUDE="uart_ascii.h")       # include        
add_compile_definitions(DEBUG_PRINTF=uart_ascii_printf0)    # function(__VA_ARGS__)
add_compile_definitions(DEBUG_DUMP=uart_ascii_dump)         # function(buff, len, addr)

# Debug/Logging: uncomment if a module has to debug
add_compile_definitions(MAIN_DEBUG)
add_compile_definitions(UART_DRV_DEBUG)
add_compile_definitions(PUART_DRV_DEBUG)

# Default UART configuration
# If CONFIG_UART_DATA_BIT > 8 the PIO-UART is used instead of UART
# If CONFIG_UART_DATA_HBLB defined - data transfer for PIO-UART is big-endian
add_compile_definitions(CONFIG_UART_BAUDRATE=250000)
add_compile_definitions(CONFIG_UART_DATA_BIT=9)
add_compile_definitions(CONFIG_UART_DATA_HBLB)

# TX-ACTIVE Output Signal
add_compile_definitions(TX_ACTIVE_SIGNAL=28)
#add_compile_definitions(TX_ACTIVE_SIGNAL_INVERTED)

# RX/TX Activity LED
add_compile_definitions(UART_RX_ACT_LED=16)
add_compile_definitions(UART_TX_ACT_LED=17)

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
    ustime.c
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
pico_generate_pio_header(rpico_cdc_uart ${CMAKE_CURRENT_LIST_DIR}/puart_rx.pio)

pico_add_extra_outputs(rpico_cdc_uart)

#-------------------------------------------------------------------------------
# Deploy (/dev/ttyACM0)
#-------------------------------------------------------------------------------
#set(RPICO_MOUNT_POINT "/media/$ENV{USER}/RPI-RP2/")
add_custom_target(
    deploy
    COMMAND ${CMAKE_COMMAND} -E echo "Deploying to pico..."
    #COMMAND cp "${PROJECT_NAME}.uf2" /media/igor/RPI-RP2/
    #COMMAND python3 ../scripts/firmware_update.py -m "${RPICO_MOUNT_POINT}" "${PROJECT_NAME}.uf2"
    COMMAND ../scripts/firmware_update.sh "${PROJECT_NAME}.uf2"
    DEPENDS rpico_cdc_uart
    COMMENT "Deploying the firmware to pico"
)

#-------------------------------------------------------------------------------
# Deploy (/dev/ttyACM1)
#-------------------------------------------------------------------------------
add_custom_target(
    deploy1
    COMMAND ${CMAKE_COMMAND} -E echo "Deploying to pico ttyACM1..."
    COMMAND ../scripts/firmware_update.sh "${PROJECT_NAME}.uf2" "/dev/ttyACM1"
    DEPENDS rpico_cdc_uart
    COMMENT "Deploying the firmware to pico (/dev/ttyACM1)"
)

#-------------------------------------------------------------------------------
# Deploy all (/dev/ttyACM0 and /dev/ttyACM1)
#-------------------------------------------------------------------------------
add_custom_target(
    deployall
    COMMAND ${CMAKE_COMMAND} -E echo "Deploying to pico ttyACM0..."
    COMMAND ../scripts/firmware_update.sh "${PROJECT_NAME}.uf2" "/dev/ttyACM0"
    COMMAND ${CMAKE_COMMAND} -E echo "Deploying to pico ttyACM1..."
    COMMAND ../scripts/firmware_update.sh "${PROJECT_NAME}.uf2" "/dev/ttyACM1"
    DEPENDS rpico_cdc_uart
    COMMENT "Deploying the firmware to picos (/dev/ttyACM0 and /dev/ttyACM1)"
)

#-------------------------------------------------------------------------------
# Deploy using SWD interface
#-------------------------------------------------------------------------------
add_custom_target(
    deployswd
    COMMAND ${CMAKE_COMMAND} -E echo "Deploying to pico using SWD..."
    COMMAND openocd -f interface/raspberrypi-swd.cfg -f target/rp2040.cfg -c "program ${PROJECT_NAME}.elf verify reset exit"
    DEPENDS rpico_cdc_uart
    COMMENT "Deploying the firmware to pico using SWD interface"
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
