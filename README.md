# RPico CDC UART

An implementation of a USB-CDC-UART bridge for Raspberry Pi Pico. The Pico device (which is powered by USB and recognized by the operating system as a CDC device) forwards all received USB data to its UART and data received from the UART back to USB, acting as a USB-UART bridge. 

Main features:
- USB CDC uses the TinyUSB library running on a separate core, leaving the main core for bridge processing.
- UART is fully configurable and based on interrupts (non-blocking mode), maintaining the responsiveness of the main core.
- A separate UART module (also fully configurable and interrupt-based/non-blocking) operating in ASCII mode and used as a logging and/or command line interface (profiling, debugging, etc.)
- Full Software control over data transfer.

## Terms

**Asynchronous Serial Communication** is a form of serial communication in which the communicating endpoints' interfaces are not continuously synchronized by a common clock signal. Instead of a common synchronization signal, the data stream contains synchronization information in form of start and stop signals, before and after each unit of transmission, respectively. The start signal prepares the receiver for arrival of data and the stop signal resets its state to enable triggering of a new sequence.

**UART** (Universal Asynchronous Receiver-Transmitter) is a peripheral device for asynchronous serial communication in which the data format and transmission speeds are configurable. It sends data one by one, from the least significant to the most significant, framed by start and stop bits so that precise timing is handled by the communication channel. The electric signaling levels are handled by a driver circuit external to the UART. Common signal levels are RS-232, RS-485, and raw TTL.

**USB CDC** (USB Communications Device Class) is a composite Universal Serial Bus device class. This class can be used for industrial equipment such as CNC machinery to allow upgrading from older RS-232 serial controllers and robotics, since they can keep software compatibility. The device attaches to an RS-232 communications line and the operating system on the USB side makes the USB device appear as a traditional RS-232 port.

USB CDC **ACM** (Abstract Control Model) is a vendor-independent publicly documented protocol that can be used for emulating serial ports over USB.

## Transfer bottleneck

USB (full speed 12 Mbps) is faster than UART. The bottleneck in USB-UART transmission is always UART. UART cannot send data as fast as USB receives.

### USB -> UART

!["USB-to-UART"](docs/USB-to-UART.png "USB to UART datatransfer")

If we continuously send data USB->UART, we will reach a point where the UART transmit buffer is full and there is no more space to store any more data received via USB (to be sent via UART).

Solutions:
1. Make the UART transmit buffer as big as possible.
2. Implement flow control: continuously monitor how full the UART transmit buffer is and control corresponding flow control signals on the USB side.

### UART -> USB

!["UART-to-USB"](docs/UART-to-USB.png "UART to USB datatransfer")

Theoretically, there shouldn't be any problems with UART->USB data transfer. However, if the main program cycle is long (takes a lot of time) and the UART receive buffer is small, the software may not be able to transfer the received UART data to USB in a timely manner. The UART receive buffer becomes full, and the newly received data is lost.

Solution:
1. Make UART receive buffer bigger.
2. Monitor program main cycle.
3. Implement flow control: continuously monitor how full the UART receive buffer is and control the corresponding flow control signals on the UART side.

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

The following steps describe the installation process of Picotool – a tool for interacting with Raspberry Pi Pico devices when they are in BOOTSEL mode (for example: firmware update).

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

`firmware_update.sh` script uses the `lsusb` command to identify the Raspberry Pi Pico connected to the USB Port. If `lsusb` is not available, install it using the following command:

```bash
sudo apt-get install usbutils
```

### Deploy firmware to Raspberry Pi Pico

If the Raspberry Pi Pico has not yet been updated with `rpico_cdc_uart.uf2` (this is the first time the device is updated with this firmware), manually put the Raspberry Pi Pico into boot mode: unplug from USB, press BOOTSEL, plug in the USB, release BOOTSEL.

If the Raspberry Pi Pico has already been updated with `rpico_cdc_uart.uf2`, the above step is not required. The `scripts/firmware_update.py` script checks if there is "TinyUSB" device connected, and sends a dummy byte at /dev/ttyACM0 uisng 1200 bps as baudrate. This reboots the device into boot mode.

Use `make deploy` command to load the firmware into the Raspberry Pi Pico:

```bash
$ make deploy
[  1%] Built target bs2_default
[  4%] Built target bs2_default_library
[  5%] Performing build step for 'pioasmBuild'
[100%] Built target pioasm
[  5%] Performing install step for 'pioasmBuild'
[100%] Built target pioasm
Install the project...
-- Install configuration: "Release"
[  6%] Completed 'pioasmBuild'
[ 11%] Built target pioasmBuild
[ 12%] Built target rpico_cdc_uart_puart_tx_pio_h
[ 98%] Built target rpico_cdc_uart
[100%] Deploying the firmware to pico
Deploying to pico...
TinyUSB Device found. Try to jump to Boot.
Bus 001 Device 012: ID 2e8a:0003 Raspberry Pi RP2 Boot
Raspberry Pi RP2 Boot found.
Update device with firmware: rpico_cdc_uart.uf2
Loading into Flash:   [==============================]  100%
Success, reboot device.
The device was rebooted into application mode.
[100%] Built target deploy
```

## Test

### Prerequisites 

Python `pyserial` library is required to run the tests:

```bash
sudo apt-get install python3-serial
```

### How to test

Use the python script `test/test.py` to test the data transfer:

```bash
python3 test/test.py --help
usage: test.py [-h] [-b BAUDRATE] [-f FROM_VALUE] [-t TO_VALUE] [-n NO_CHECK] [-m TEST_MODE] dev1 dev2

Test RPico_CDC_UART firmware.

positional arguments:
  dev1                  Serial Device 1 (Ex: /dev/ttyUSB0 for USB-UART converter, /dev/ttyACM0 for TinyUSB)
  dev2                  Serial Device 2 (Ex: /dev/ttyUSB0 for USB-UART converter, /dev/ttyACM0 for TinyUSB)

options:
  -h, --help            show this help message and exit
  -b BAUDRATE, --baudrate BAUDRATE
                        Baudrate (default: 115200)
  -f FROM_VALUE, --from FROM_VALUE
                        Test range from (default: 1)
  -t TO_VALUE, --to TO_VALUE
                        Test range to (default: 2050)
  -n NO_CHECK, --no-check NO_CHECK
                        Do not check the result (default: 0)
  -m TEST_MODE, --test-mode TEST_MODE
                        Test mode: 0=D1->D2, 1=D1/D2, 2=rand(D1/D2), 3=rand(pack_len), 4=rand(D1/D2,pack_len) (default: 0)
```

### Test Setup: Raspberry Pi Pico + USB Serial Adapter

!["Pico-to-USB"](docs/test_setup_pico_usb_serial.png "Raspberry Pi Pico + USB Serial Adapter")

### Test Setup: 2 x Raspberry Pi Pico

!["Pico-to-Pico"](docs/test_setup_pico_pico.png "2 x Raspberry Pi Pico")

### Test Examples

Example USB->Pico, 1000 bytes (from 1 to 1000), baudrate 115200 bps:

```bash
python3 test/test.py --to 1000 -b 115200 /dev/ttyUSB0 /dev/ttyACM0
```

Example Pico->USB, 200 bytes (from 100 to 300), baudrate 9600 bps:

```bash
python3 test/test.py --from 100 --to 300 -b 9600 /dev/ttyACM0 /dev/ttyUSB0
```

Example Pico1<->Pico2, random directions (Pico1->Pico2, Pico2->Pico1) for every packet:

```bash
python3 test/test.py --to 1000 -m 2 /dev/ttyACM0 /dev/ttyACM1
```

Example Pico1->Pico2, random packet length:

```bash
python3 test/test.py --to 1000 -m 3 /dev/ttyACM0 /dev/ttyACM1
```

## Supported Baudrates

### UART implemnted as PIO

System Clock Frequency: `clk_sys = 125MHz`

PIO Frequency: `clk_pio = clk_sys / (CLKDIV_INT + CLKDIV_FRAC/256)`

Maximal PIO Frequency: `clk_pio_max = clk_sys / (1 + 0/256) = 125MHz`

Minimal PIO Frequency: `clk_pio_min = clk_sys / (65536 + 0/256) = 1907,349Hz` 

One bit is sent/received in 8 PIO clocks.

Maximal UART-PIO Baudrate: `pio_max_baud = clk_pio_max / 8 = 15,635MHz`

Minimal UART-PIO Baudrate: `pio_min_baud = clk_pio_min / 8 = 238,42Hz` 

__Tests Standart Baudrates/Frame-Formats PIO-UART<->FTDI232__

Data transfers (both directions, different packet sizes) between /dev/ttyACM (PIO-UART) and /dev/ttyUSB (FTDI232):

| Baudrate | Frame Formats |
| -------- | ------------- |
| 9600     | 7N1, 8N1      |
| 14400    | 7N1, 8N1      |
| 19200    | 7N1, 8N1      |
| 38400    | 7N1, 8N1      |
| 57600    | 7N1, 8N1      |
| 115200   | 7N1, 8N1      |
| 230400   | 7N1, 8N1      |

__Test PIO-UART<->FTDI232__

Test #30 Fails:
```
+---------------+---------------+-----------+--------+---------+------------+------+------+----------+-----------+
| Device1       | Device2       | Baud-Rate | CLKDIV | Bit-Cnt | Big-Endian | From | To   | No-Check | Test-Mode |
| /dev/ttyACM0  | /dev/ttyACM1  | 9600      | 0      | 9       | 1          | 1    | 300  | 0        | 0         |
+---------------+---------------+-----------+--------+---------+------------+------+------+----------+-----------+
  85% [##################################______]   257 /dev/ttyACM0->/dev/ttyACM1 (pack.size: 257) <-- Failed

```

All tests executed in format: 9N1

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
| 781250   |                      | 20         | 0           |             |                    |
| 1562500  | 640ns   (1562500)    | 10         | 0           | 12500000,00 | 7us                |
| 3125000* | 320ns   (3125000)    | 5          | 0           | 25000000,00 | 3us                |

(*) 3125000 Bits/second is not stable, sometimes the software cannot read on time the received data from PIO and the data is lost. Maybe can be fixed by implementing DMA.

## Known Issues

**1. Time to configure UART**
When opening the USB CDC port, the device needs some time (~5 ms) to apply the same settings (baud rate and line coding) to the UART interface. If the data is sent immediately (without delay) after opening the USB CDC port, the first byte may be lost. To work around this, there is a delay in the tests after the ports are opened:

```
...
ser_tx = serial.Serial(DEVICE_NAME_TX, BAUDRATE)
ser_rx = serial.Serial(DEVICE_NAME_RX, BAUDRATE, timeout=0)

# Give time (~10ms) to pico to configure the UART interface
time.sleep(0.01)
...
```

**2. UART TX Interrupt not called for the first byte to send**

According to RP2040 Datasheet, 4.2 UART -> 4.2.6 Interrupts -> 4.2.6.3. UARTTXINTR:

> _The transmit interrupt is based on a transition through a level, rather than on the level itself. When the interrupt and the UART is enabled before any data is written to the transmit FIFO the interrupt is not set. The interrupt is only set, after written data leaves the single location of the transmit FIFO and it becomes empty._

The Tx interrupt is generated after a byte is sent. We can't send the first byte directly from the interrupt—the interrupt isn't called. To work around this, the UART driver sends a dummy byte (0) after initialization to enable the TX interrupt.

**3. Detect when COM-Port opened**

The software has to detect when the COM-Port is opened, and collect UART RX data (to be sent over USB) only when COM-Port is opened. There is no need to collect UART RX data if the COM-Port is closed - it leads to data inconsistency.

__Linux__

On Linux, when COM-Port is opened, for example using `open("/dev/ttyACM0", …)`,  the terminal driver asserts DTR by default. The detection could be done by analyzing `dtr` argument in `tud_cdc_line_state_cb()` callback. When `dtr=true`, Linux opened the COM-Port. When `dtr=false`, Linux closed the COM-Port. 

__Windows__

On Windows, the COM port driver does not automatically assert DTR when COM-Port is opened in applications. The application must explicitly request it (e.g. via SetCommState() or EscapeCommFunction(..., SETDTR)). The solution with DTR does not work.

## Notes

__UART/USB devices in Linux__

`/dev/ttyUSB0` - USB-UART converter, a separate IC (usually FTDI).
`/dev/ttyACM0` - Raspberry Pi Pico with TinyUSB library that implements CDC ACM (USB Communications Device Class).

__Implement TX_ACTIVE Signal__

Set a GPIO as Output and set it (to active High or Low) while transmitting data.
This allows using UART together with RS-485 transceivers (for example with THVD1429).

Set the TX_ACTIVE before writing the first data to the shift register and reset it after UART has finishing sending the last data (together with the stop-bit).

How to detect when UART has finished sending the stop-bit of the last data:

- In case of the UART: check UARTFR (Flag Register) bit BUSY (3).
- In case of PIO-UART: check SMx_EXECCTRL (Execution/behavioural settings for state machine x) bit EXEC_STALLED (31). EXEC_STALLED (RO) - If 1, an instruction written to SMx_INSTR is stalled, and latched by the state machine. Will clear to 0 once this instruction completes.

## TODO

- Try to fix Known-Issue #2 without sending the dummy byte at init.
- When UART settings changed, reinit all intern variables including ring buffers and pointers.
- Implement flow control: Continuously monitor the fullness of the receive/send buffer.
- Check if we can use FIFO or DMA.
- Get rid of pico UART libraries and implement as much as possible in driver.
