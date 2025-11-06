# RPico CDC UART

!["RPico-CDC-UART_Cover"](docs/cover.png "RPico CDC UART Cover")

An implementation of a USB–UART bridge for the Raspberry Pi Pico (RPico). The Pico device (powered via USB and recognized by the operating system as a CDC device) forwards all received USB data to its UART and all data received from the UART back to USB, acting as a USB–UART bridge.

## Main Features

- **PIO-UART** – a fully configurable UART implemented using RP2040 PIO.  
  Supports a wide range of standard and non-standard baud rates (from 238 bit/s up to 3.1 Mbit/s) and frame formats (3–16 bits/frame).  
  Interrupt-based (non-blocking mode), maintaining the responsiveness of the main core.

- **Peripheral-UART** – a fully configurable UART implemented using the RP2040 UART peripheral.  
  Supports only standard baud rates and frame formats (5–8 bits/frame).  
  Interrupt-based (non-blocking mode), maintaining the responsiveness of the main core.

- **USB CDC** uses the TinyUSB library running on a separate core, leaving the main core free for UART–USB bridge processing.

- **Logging UART** – a separate, fully configurable, interrupt-based UART module implemented using the RP2040 UART peripheral.  
  Operates in ASCII mode and can be used as a logging and/or command-line interface (profiling, debugging, etc.).

- **Full software control** over data transfer.

- **Configurable GPIOs** – a set of GPIO pins can be used for custom purposes, such as triggering on specific received frames or sending messages when an input level is detected.

## Terms

- **Asynchronous Serial Communication**
A form of serial communication in which communicating endpoints are not continuously synchronized by a common clock signal.  
Instead, the data stream includes start and stop bits for synchronization.

- **UART (Universal Asynchronous Receiver–Transmitter)**  
A peripheral device for asynchronous serial communication.  
It transmits data bit by bit (least significant first), framed by start and stop bits.  
Electrical signaling levels (RS-232, RS-485, or TTL) are managed by external driver circuits.

- **USB CDC (USB Communications Device Class)**  
A USB class used for emulating serial communication ports.  
It allows modern systems to communicate with legacy equipment such as RS-232-based devices while maintaining software compatibility.

- **USB CDC ACM (Abstract Control Model)**  
A vendor-independent protocol that enables emulation of serial ports over USB.

- **RPico** – abbreviation for *Raspberry Pi Pico*.  

- **RP2040** – a 32-bit dual-core ARM Cortex-M0+ microcontroller used in the Raspberry Pi Pico board.  

- **PIO (Programmable I/O)** – a subsystem on the RP2040 enabling custom hardware interfaces via programmable state machines.  

- **PIO-UART** – a UART interface implemented on RP2040 using the PIO subsystem.

## Configuration

The USB–UART bridge can be configured by editing the `src/CMakeLists.txt` file.

### USE_PIO_UART

The UART interface can operate in one of two modes:

- **PIO-UART** – implemented in PIO; supports a wide range of non-standard baud rates and frame formats.  
  UART settings are hardcoded in the firmware and cannot be changed at runtime.
- **Peripheral-UART** – uses the RP2040 UART peripheral; supports only standard baud rates and frame formats.  
  UART settings can be changed at runtime (when connected via USB).

When `USE_PIO_UART` is defined, the UART operates in PIO-UART mode.  

```
add_compile_definitions(USE_PIO_UART)
```

When it is not defined, Peripheral-UART mode is used.

### PIO-UART Settings

When `USE_PIO_UART` is defined, configure the following parameters:

```
set(use_uart_baudrate 9600)
set(use_uart_data_bit 3)
set(use_uart_data_hblb 1)
set(use_pio_clkdiv 0)
```

- `use_uart_baudrate` – specifies the baud rate used by the PIO-UART.  
- `use_uart_data_bit` – specifies the frame format (3–16 bits/frame).  
- `use_uart_data_hblb` – when the frame format exceeds 8 bits, each frame is sent to USB as two bytes.  
  If set to `1`, bytes are sent in high-byte/low-byte order.  
- `use_pio_clkdiv` – if not `0`, `use_uart_baudrate` is ignored and the baud rate is calculated as:  
  `use_uart_baudrate = 125MHz / (use_pio_clkdiv * 8)`

### TX_ACTIVE_SIGNAL

This signal allows the UART to be used together with an RS-485 transceiver—switching the transceiver between receive and transmit modes.

```
add_compile_definitions(TX_ACTIVE_SIGNAL=28)
add_compile_definitions(TX_ACTIVE_SIGNAL_INVERTED)
```

- `TX_ACTIVE_SIGNAL` – specifies the GPIO pin used as TX_ACTIVE; it is active while UART sends data.  
- `TX_ACTIVE_SIGNAL_INVERTED` – when defined, the TX_ACTIVE signal is active low.

### RX/TX Activity LEDs

Two GPIO outputs can visually indicate receive or transmit activity (e.g., using LEDs):

```
add_compile_definitions(UART_RX_ACT_LED=16)
add_compile_definitions(UART_TX_ACT_LED=17)
```

- `UART_RX_ACT_LED` – GPIO pin toggled when the UART is receiving data.  
- `UART_TX_ACT_LED` – GPIO pin toggled when the UART is transmitting data.

### UART_RX_BYTES_TOUT

This setting prevents USB from sending small packets for every received UART byte.  
Instead, UART→USB data is sent when no further UART data is received for longer than `UART_RX_BYTES_TOUT` byte times (depending on baud rate) or after 64 bytes have been received.

```
add_compile_definitions(UART_RX_BYTES_TOUT=2)
```

## Build

Install Raspberry Pi Pico SDK:
https://www.raspberrypi.com/documentation/microcontrollers/c_sdk.html

Clone code to `~/pico_dev/`.

Export `pico-sdk` path, example:

```bash
export PICO_SDK_PATH=~/pico/pico-sdk
```
    
Create build directory in `~/pico_dev/RPico_CDC_UART/build`:

```bash
cd ~/pico_dev/RPico_CDC_UART
mkdir build
cd build
```

Build project:

```bash
cmake ../src/
make
```

## Deploy

### Prerequisites: picotool

The following steps describe the installation process of Picotool – a tool for interacting with RPico devices when they are in BOOTSEL mode (for example: firmware update).

__Getting picotool:__

```bash
cd ~/pico/
git clone https://github.com/raspberrypi/picotool.git --branch master
cd picotool
```

Install required libusb library:

```bash
sudo apt install libusb-1.0-0-dev
```

__Building picotool:__

```bash
mkdir build
cd build
export PICO_SDK_PATH=~/pico/pico-sdk
cmake ../
make
```

__Install picotool:__

To install `picotool` only for the current user (in `/home/$USER/.local/bin/picotool`):

```bash
cmake -DCMAKE_INSTALL_PREFIX=~/.local ..
make install
```

To install `picotool` for all users (in `/usr/bin/picotool`):

```bash
cmake -DCMAKE_INSTALL_PREFIX=/usr ..
sudo make install
```

### Prerequisites: usbtool

`firmware_update.sh` script uses the `lsusb` command to identify the RPico connected to the USB Port. If `lsusb` is not available, install it using the following command:

```bash
sudo apt-get install usbutils
```

### Flash Firmware to RPico

If this is the first time flashing the firmware (`rpico_cdc_uart.uf2`):

1. Unplug RPico.  
2. Hold **BOOTSEL** while reconnecting the USB cable.  
3. Release **BOOTSEL**.  
4. Copy the firmware file to the mounted partition.

If the RPico already runs the firmware, manual boot mode entry is not required.  
The script `scripts/firmware_update.py` checks for a connected TinyUSB device and sends a dummy byte at `/dev/ttyACM0` using 1200 bps to reboot the device into boot mode.

Use `sudo make deploy` command to flash the firmware:

```bash
$ sudo make deploy
[  1%] Built target bs2_default
[  4%] Built target bs2_default_library
[  5%] Built target rpico_cdc_uart_puart_tx_pio_h
[  6%] Built target rpico_cdc_uart_puart_rx_pio_h
[ 98%] Built target rpico_cdc_uart
[100%] Deploying the firmware to pico
Deploying to pico...
TinyUSB Device found, assuming this is /dev/ttyACM0. Try to jump to Boot.
Bus 001 Device 107: ID 2e8a:0003 Raspberry Pi RP2 Boot
Raspberry Pi RP2 Boot found.
Update device with firmware: rpico_cdc_uart.uf2
Loading into Flash:   [==============================]  100%
Success, reboot device.
[100%] Built target deploy
```

If you need to update two RPico devices simultaneously (e.g., for PIO-UART testing): `sudo make deployall` command ca be used.

## Test

### Prerequisites 

Python `pyserial` library is required to run the tests:

```bash
sudo apt-get install python3-serial
```

### Test Setups

Depending on which UART mode is being tested (Peripheral-UART or PIO-UART), use one of the following setups:

__Peripheral-UART mode: RPico + USB Serial Adapter__

!["Pico-to-USB"](docs/test_setup_pico_usb_serial.png "RPico + USB Serial Adapter")

Connect RXD↔TXD and TXD↔RXD between RPico and a USB Serial Adapter (e.g., FT232).
Both devices are connected to the PC via USB.
RPico appears as `/dev/ttyACMx` and the adapter as `/dev/ttyUSBx`.

__PIO-UART mode: 2 x RPico__

!["Pico-to-Pico"](docs/test_setup_pico_pico.png "2 x RPico")

When testing PIO-UART mode, two RPico boards are used.
Standard USB–Serial adapters cannot recognize non-standard frame formats or baud rates.
Connect RXD↔TXD between the two devices; both will appear as `/dev/ttyACMx`.

### Test using sertest.py

The `test/sertest.py` script performs data transfer tests between two devices.
It sends data packets of specified length and content from one device while receiving data on the other.
If the received data matches the sent data, the test passes.

The script can:

- Execute multiple tests with varying packet lengths (defined by `FROM_VALUE` and `TO_VALUE`).

- Change transfer direction automatically (direct, reverse, alternating, or random).

For Peripheral-UART mode, sertest.py automatically configures the UART settings.
For PIO-UART mode, settings must be preconfigured in `CMakeLists.txt` and firmware rebuilt.

The script accepts the following arguments:

```bash
$ python3 sertest.py
usage: sertest.py [-h] [-b BAUDRATE] [-c BIT_CNT] [-f FROM_VALUE] [-t TO_VALUE] [-n NO_CHECK] [-m TEST_MODE] dev1 dev2
```

Where:

- `-b BAUDRATE` – baud rate (e.g., 115200)
- `-c BIT_CNT` – frame format (e.g., 3 for 3 bits/frame)
- `-f FROM_VALUE`, `-t TO_VALUE` – start and end packet lengths
- `-n NO_CHECK` – skip received/sent data verification
- `-m TEST_MODE` – test mode (0–6, see below)
    - 0 - data packets sent from `dev1` to `dev2`, every new test increments the data packet length
    - 1 - data packets sent from `dev2` to `dev1`, every new test increments the data packet length
    - 2 - data packets sent repeatedly `dev1`->`dev2`, `dev2`->`dev1`, every new test increments the data packet length
    - 3 - data packets sent randomly `dev1`->`dev2` or `dev2`->`dev1`, every new test increments the data packet length
    - 4 - data packets sent from `dev1` to `dev2`, data packets have a random length
    - 5 - data packets sent from `dev2` to `dev1`, data packets have a random length
    - 6 - data packets sent randomly `dev1`->`dev2` or `dev2`->`dev1`, data packets have a random length
- `dev1` - first device (`/dev/ttyACMx` for RPico or `/dev/ttyUSBx` for USB Serial Adapter)
- `dev2` - second device (`/dev/ttyACMx` for RPico or `/dev/ttyUSBx` for USB Serial Adapter)

Example testing PIO-UART, 9600 bits/sec, 11 bits/frame, data packet length from 1 to 100 bytes, test mode 2:

```
 $ python3 sertest.py -b 9600 -c 11 -f 1 -t 100 -m 2 /dev/ttyACM0 /dev/ttyACM1
+---------------+---------------+-----------+--------+---------+------------+------+------+----------+-----------+
| Device1       | Device2       | Baud-Rate | CLKDIV | Bit-Cnt | Big-Endian | From | To   | No-Check | Test-Mode |
| /dev/ttyACM0  | /dev/ttyACM1  | 9600      | 0      | 11      | 1          | 1    | 100  | 0        | 2         |
+---------------+---------------+-----------+--------+---------+------------+------+------+----------+-----------+
End of test, Result: True ( 100 )##############]   100 /dev/ttyACM0->/dev/ttyACM1 (pack.size: 100)
```

### Test using test_master.py

`test/test_master.py` runs multiple test scenarios (baud rates, packet lengths, test modes, frame formats) in one command.
Tests are defined in a Python structure (`test_def`) and expanded into individual test cases.

```
test_def = (
    'dev1',
    'dev2',
    [
    |<---0-->|<----1---->|<-2->|<-3->|<-----4----->|<--------5-------->|
    (baudrate, pio_clkdiv, from,   to, [test_modes], [data_bits_counts]),
    (baudrate, pio_clkdiv, from,   to, [test_modes], [data_bits_counts]),
    (baudrate, pio_clkdiv, from,   to, [test_modes], [data_bits_counts]),
        ...
    ]
)
```

Example:

```python
test_def1 = (
    '/dev/ttyACM0',
    '/dev/ttyACM1',
    [
    #     0  |   1  |  2 | 3 |     4     |       5
    #        | pio- | length |           |
    #   baud |clkdiv| from-to| test-mode | data bits count
    (     238, 65535, 1,   50, [0, 1, 6], [3, 4, 5]),
    (    2400,     0, 1,   50, [0, 1, 6], [3, 6, 8]),
    (    4800,     0, 1,   50, [0, 1, 6], [3, 8, 16]),
    (    9600,     0, 1,  300, [0, 1, 6], [3, 7, 8]),
    #    ...    
    ]
)
```

The script breaks down each row of the table into a series of individual test cases:

```
test_case_list = [
    |<-0->|<-1->|<---2-->|<----3---->|<-4->|<-5->|<---6-->|<----7--->|<----8--->|
    (dev1, dev2, baudrate, pio_clkdiv, from,   to, bit_cnt, test_mode, build_req)
    ...
]
```

In example with `test_def1` above, from the first row `(238, 65535, 1, 50, [0, 1, 6], [3, 4, 5]),` the following set of test cases is created:

```python
test_case_list = [
    #|<--  dev1  -->|<--  dev2  -->|<-baudr.->|<-clkdiv->|<-from->|<-to->|<-bit_cnt->|<-test_mode->|<-build_req->|
    ('/dev/ttyACM0', '/dev/ttyACM1',       238,     65535,       1,    50,          3,            0,        True),
    ('/dev/ttyACM0', '/dev/ttyACM1',       238,     65535,       1,    50,          3,            1,       False),
    ('/dev/ttyACM0', '/dev/ttyACM1',       238,     65535,       1,    50,          3,            6,       False),
    ('/dev/ttyACM0', '/dev/ttyACM1',       238,     65535,       1,    50,          4,            0,        True),
    ('/dev/ttyACM0', '/dev/ttyACM1',       238,     65535,       1,    50,          4,            1,       False),
    ('/dev/ttyACM0', '/dev/ttyACM1',       238,     65535,       1,    50,          4,            6,       False),
    ('/dev/ttyACM0', '/dev/ttyACM1',       238,     65535,       1,    50,          5,            0,        True),
    ('/dev/ttyACM0', '/dev/ttyACM1',       238,     65535,       1,    50,          5,            1,       False),
    ('/dev/ttyACM0', '/dev/ttyACM1',       238,     65535,       1,    50,          5,            6,       False),
]
```

For every test case in the resulting `test_case_list` table the `sertest.py` is invoked. 

If the next test case has a different baud rate or frame format (`bit_cnt`) than the previous one - `build_req` flag is set, `CMakeLists.txt` is automatically modified using new PIO-UART settings, cmake is invoked to generate a new firmware, both devices are updated using the new firmware, and `sertest.py` is invoked.

This continues until all test cases are successfully executed or it stops as soon as one test case fails.

To invoke the `test_master.py` script:

```bash
$ export PICO_SDK_PATH=~/pico/pico-sdk
$ cd ~/pico/pico-dev/RPico_CDC_UART/test/
$ sudo python3 test_master.py
```

## Known Issues

**UART Configuration Delay**

When opening the USB CDC port, the device needs ~5 ms to apply UART settings.
If data is sent immediately, the first byte may be lost.
Tests include a short delay after opening ports.

```
...
ser_tx = serial.Serial(DEVICE_NAME_TX, BAUDRATE)
ser_rx = serial.Serial(DEVICE_NAME_RX, BAUDRATE, timeout=0)

# Give time (~10ms) to pico to configure the UART interface
time.sleep(0.01)
```

**UART TX Interrupt not triggered for first byte**

As per the RP2040 datasheet (4.2 UART -> 4.2.6 Interrupts -> 4.2.6.3. UARTTXINTR):

> _The transmit interrupt is based on a transition through a level, rather than on the level itself. When the interrupt and the UART is enabled before any data is written to the transmit FIFO the interrupt is not set. The interrupt is only set, after written data leaves the single location of the transmit FIFO and it becomes empty._

The Tx interrupt is generated after a byte is sent. We can't send the first byte directly from the interrupt—the interrupt isn't called. The workaround is to send a dummy byte (0) after initialization to enable the TX interrupt.

**Detecting when COM Port is open**

The software should only collect UART RX data when the COM port is open.

Linux:
The DTR signal is asserted by default when a COM port is opened.
Detection is done via `tud_cdc_line_state_cb()`.
When `dtr=true`, the port is open; when false, it is closed.

Windows:
The COM port driver does not automatically assert DTR.
The application must explicitly do so via `SetCommState()` or `EscapeCommFunction(..., SETDTR)`.

## Notes

### PIO Frequency Limits

System Clock: `clk_sys = 125MHz`

PIO Frequency: `clk_pio = clk_sys / (CLKDIV_INT + CLKDIV_FRAC/256)`

Max PIO Frequency: `clk_pio_max = clk_sys / (1 + 0/256) = 125MHz`

Min PIO Frequency: `clk_pio_min = clk_sys / (65536 + 0/256) = 1907,349Hz` 

One bit is sent/received in 8 PIO clocks.

Max PIO-UART baud rate: `pio_max_baud = clk_pio_max / 8 = 15,635MHz`

Min PIO-UART baud rate: `pio_min_baud = clk_pio_min / 8 = 238,42Hz` 

### Measured Bit-Length

All tests executed in 9 bit frame format.

| Baudrate | Measured Bit (Baudr) | CLKDIV_INT | CLKDIV_FRAC |  PIO Freq.  | element_req_ustime |
| -------- | -------------------- | ---------- | ----------- | ----------- | ------------------ |
| 238      | 4,20ms  (238)        | 65535      | 0           |     1907,37 | 50420us            |
| 9600     | 104us   (9615)       | 1627       | 155         |    76799,93 | 1250us             |
| 14400    | 69,4us  (14409)      | 1085       | 18          |   115199,90 | 833us              |
| 19200    | 52,1us  (19193)      | 813        | 205         |   153600,24 | 625us              |
| 38400    | 26us    (38461)      | 406        | 231         |   307199,01 | 312us              |
| 57600    | 17,36us (57603)      | 271        | 68          |   460802,94 | 208us              |
| 115200   | 8,68us  (115207)     | 135        | 162         |   921605,89 | 104us              |
| 230400   | 4.34us  (230302)     | 67         | 209         |  1843211,79 | 52us               |
| 1562500  | 640ns   (1562500)    | 10         | 0           | 12500000,00 | 7us                |
| 3125000* | 320ns   (3125000)    | 5          | 0           | 25000000,00 | 3us                |

3125000 Bits/second is not stable, sometimes the software cannot read on time the received data from PIO and the data is lost. Maybe can be fixed by implementing DMA.

### Transfer Bottleneck

USB (full speed 12 Mbit/s) is faster than UART, so the UART is the limiting factor.

__USB->UART__

!["USB-to-UART"](docs/USB-to-UART.png "USB to UART datatransfer")

When continuously sending data from USB to UART, the UART TX buffer can fill up.

Solutions:
- Increase UART TX buffer size.
- Implement flow control.

__UART->USB__

!["UART-to-USB"](docs/UART-to-USB.png "UART to USB datatransfer")

If the main loop is too slow or buffers are too small, data may be lost.

Solutions:
- Increase UART RX buffer size.
- Optimize main loop timing.
- Implement flow control.

### UART/USB devices in Linux

`/dev/ttyUSB0` – USB–UART converter (e.g., FTDI).
`/dev/ttyACM0` – RPico with TinyUSB implementing CDC ACM.

### TX_ACTIVE Signal Implementation

Use a GPIO output to indicate when data is being transmitted (active high or low).
This allows UART operation with RS-485 transceivers (e.g., THVD1429).

Set the TX_ACTIVE before writing the first data to the shift register and reset it after UART has finished sending the last data (together with the stop-bit).

How to detect when UART has finished sending the stop-bit of the last data:

- For Peripheral-UART: monitor `UARTFR` register, bit BUSY (3).
- For PIO-UART: monitor SMx_EXECCTRL (Execution/behavioural settings for state machine x) bit EXEC_STALLED (31). EXEC_STALLED (RO) - If 1, an instruction written to SMx_INSTR is stalled, and latched by the state machine. Will clear to 0 once this instruction completes.

## TODO

- Investigate using FIFO or DMA.
- Try to fix Known Issue #2 without sending a dummy byte.
- Reinitialize internal variables and buffers when UART settings change.
- Implement flow control to monitor buffer usage.
- Minimize dependencies on the Pico UART libraries by using a custom driver.
