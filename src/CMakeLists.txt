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

#-------------------------------------------------------------------------------
# UART configuration - modify the following variables to configure the build:
#   use_pio_uart: 0 - UART driver must be used - only standard baudrates available (the rest of use_pio_... configs are ignored)
#                 1 - PIO_UART driver is used, the baudrate is hardcoded (use_pio_baudrate or use_pio_clkdiv)
#   use_pio_baudrate: the baudrate to be used in case PIO_DRIVER is used (use_pio_uart=1)
#   use_pio_data_bit: specifies the frame format: 3–16 bits/frame (only when PIO_DRIVER is used)
#   use_pio_data_hblb: when the frame format exceeds 8 bits, each frame is sent to USB as two bytes
#                      0 - bytes are sent in low-byte/high-byte order
#                      1 - bytes are sent in high-byte/low-byte order 
#   use_pio_clkdiv : if not 0, use_pio_baudrate is ignored and the baud rate is calculated as:  
#                       use_pio_baudrate = 125MHz / (use_pio_clkdiv * 8)
#-------------------------------------------------------------------------------
set(use_pio_uart 1)
set(use_pio_baudrate 62500)
set(use_pio_data_bit 9)
set(use_pio_data_hblb 1)
set(use_pio_clkdiv 0)

#-------------------------------------------------------------------------------

if(use_pio_uart)
    add_compile_definitions(USE_PIO_UART)
endif()

add_compile_definitions(CONFIG_UART_DATA_BIT=${use_pio_data_bit})
add_compile_definitions(CONFIG_UART_DATA_HBLB=${use_pio_data_hblb})
add_compile_definitions(CONFIG_PIO_CLKDIV=${use_pio_clkdiv})

if(use_pio_clkdiv)
    math(EXPR calc_baudrate "(125000000 / ${use_pio_clkdiv}) / 8")
else()
    set(calc_baudrate ${use_pio_baudrate})
endif()
add_compile_definitions(CONFIG_UART_BAUDRATE=${calc_baudrate})

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

# Set output file name
if(use_pio_uart)
    # If PIO_UART driver used, set filename: rpico_cdc_<baudrate>bps_<bit>b
    set(file_name_sufix "${calc_baudrate}bps_${use_pio_data_bit}b")
else()
    # If UART driver used, set filename: rpico_cdc_uart
    set(file_name_sufix "uart")
endif()
set_target_properties(
    rpico_cdc_uart
    PROPERTIES 
        OUTPUT_NAME "rpico_cdc_${file_name_sufix}"
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
