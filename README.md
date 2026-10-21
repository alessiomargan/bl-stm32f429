# bl-stm32f429

EtherCAT bootloader for the STM32F429I-DISC1 (STM32F429ZI) and ET1100.
The repository follows the `bl-stm32f407` layout, using CMake for command-line,
VS Code, and STM32CubeIDE builds.

## Repository layout

```text
Core/                         Board initialization and interrupt handlers
Drivers/                      STM32F4 HAL and CMSIS
external/soes/                SOES EtherCAT stack (main_advr)
external/bl-stm32f4xx-ecat/    Shared bootloader, F429 startup and linker scripts (main)
cmake/stm32cubemx/            Board and HAL source configuration
cmake/soes.cmake              SOES and shared bootloader integration
cmake/gcc-arm-none-eabi.cmake  GNU Arm toolchain
cmake/build_info.cmake        Build metadata generator
.vscode/                     Build and ST-LINK debug configuration
build/<preset>/              Generated firmware and metadata (ignored by Git)
```

## Clone

```sh
git clone --recurse-submodules https://github.com/alessiomargan/bl-stm32f429.git
cd bl-stm32f429
```

For an existing checkout:

```sh
git submodule update --init --recursive
```

Both external repositories use HTTPS submodules. The parent repository pins
exact commits; their configured tracking branches are `main_advr` for SOES
and `main` for the shared bootloader.

## Build

Install CMake 3.21 or newer, Ninja, and the GNU Arm Embedded toolchain
(`arm-none-eabi-gcc`, GCC 11 or newer for the linker script). Put the tools
on `PATH`, then run:

```sh
cmake --preset Debug
cmake --build --preset Debug
```

Presets `Release`, `RelWithDebInfo`, and `MinSizeRel` are also available.
For example:

```sh
cmake --preset Release
cmake --build --preset Release
```

Outputs are `build/Debug/bl-stm32f429.elf`, `.hex`, `.bin`, and `.map`.
Build time and Git metadata are generated in `build/<preset>/generated/include`;
no additional repository or generated header inside `external/` is required.

Makefile shortcuts:

```sh
make                 # Configure and perform a clean Debug build
make configure       # Configure Debug
make build           # Configure and perform a clean Debug build
make bin             # Build and check the BIN exists
make size            # Build and print ELF section sizes
make flash           # Build and flash with st-flash
make flash-openocd   # Build and flash with OpenOCD
make clean           # Remove build/
```

## VS Code

Open this repository root and install the recommended CMake Tools, C/C++, and
Cortex-Debug extensions. Select the `Debug` configure and build presets.
`Ctrl+Shift+B` builds Debug. The **Debug Microcontroller - ST-Link** launch
configuration builds and loads `build/Debug/bl-stm32f429.elf`.
An ST-LINK probe and ST-LINK GDB server must be available; configure the
Cortex-Debug server/toolchain paths if they are not detected automatically.

## STM32CubeIDE

Import the repository with **File > Import > General > Existing Projects into
Workspace**, without copying it. Select Debug or Release, configure CMake,
then build. Both configurations use Ninja, `cmake/gcc-arm-none-eabi.cmake`,
and their matching CMake presets under `build/`.

The checked-in launch configuration loads `build/Debug/bl-stm32f429.elf`.
The `.ioc` and all Core/Drivers files retain the existing F429 board, clock, and peripheral settings.
The CMake source list under `cmake/stm32cubemx/` is maintained alongside it;
review that list after regenerating CubeMX code.

## Flash layout

The F429 linker script comes from the bootloader submodule:
`external/bl-stm32f4xx-ecat/stm32f429/STM32F429ZITX_FLASH.ld`.

| Region | Start | Size |
| --- | --- | --- |
| Bootloader | `0x08000000` | 48 KiB |
| Parameters | `0x0800C000` | 16 KiB |
| Calibration | `0x08010000` | 64 KiB |
| Application | `0x08020000` | 128 KiB minus 4-byte CRC |
| Sectors 6–11 | `0x08040000` | 768 KiB |

Program this bootloader at `0x08000000`:

```sh
st-flash write build/Debug/bl-stm32f429.bin 0x08000000
```

## Update external repositories

```sh
git submodule update --remote --recursive
git diff --submodule
```

Build and validate the update before committing the changed submodule pointers.
Changes inside an external repository must be committed in that repository first.

## F429 board configuration

The original `bl-stm32f429.ioc`, `Core/`, and `Drivers/` are preserved.
The STM32F429ZI runs at 180 MHz with APB1 at 45 MHz.

| Function | Pins / peripheral |
| --- | --- |
| ET1100 SPI | SPI4: PE2 SCK, PE5 MISO, PE6 MOSI |
| EtherCAT CS / IRQ | PE4 / PE3 |
| EEPROM loaded input | PC13 |
| Serial console | USART1: PA9 TX, PA10 RX |
| LEDs | PG13 / PG14 |
| Pushbutton | PA0 |
| Debug outputs | PB3 / PB4 |

The F429 linker provides 64 KiB CCMRAM and 192 KiB RAM.
Its flash partitions cover the first 1 MiB of the MCU's 2 MiB flash;
this bootloader occupies the 48 KiB region starting at `0x08000000`.
The startup assembly and linker script come from the bootloader submodule's
`stm32f429/` directory. The SOES source list selects the ET1100 driver.
