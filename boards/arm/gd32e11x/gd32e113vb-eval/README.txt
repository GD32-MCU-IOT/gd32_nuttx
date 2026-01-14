GD32E113VB-EVAL Board Support for NuttX
========================================

## Board Information

- **Board**: GD32E113VB-EVAL
- **MCU**: GD32E113VBT6
- **Core**: ARM Cortex-M4 with FPU and MPU
- **Flash**: 128KB
- **SRAM**: 32KB
- **Package**: LQFP100

## Features

### On-board Resources

- **LEDs**: 4 user LEDs
  - LED1: PC0
  - LED2: PC2
  - LED3: PE0
  - LED4: PE1

- **Buttons**: 3 user buttons
  - WAKEUP: PA0
  - TAMPER: PC13
  - USER: PB14

- **Crystal Oscillator**:
  - HXTAL: 8MHz
  - LXTAL: 32.768kHz (optional)

### Peripherals

- **USART**: 3 (USART0-2)
- **UART**: 2 (UART3-4)
- **SPI**: 3 (SPI0-2, SPI1/2 support I2S)
- **I2C**: 2 (I2C0-1)
- **Timers**:
  - 1 Advanced timer (TIMER0)
  - 1 Advanced timer (TIMER7)
  - 10 General timers (TIMER1-4, TIMER8-13)
  - 2 Basic timers (TIMER5-6)
  - 2 Watchdogs
- **ADC**: 2 units, up to 16 channels
- **DAC**: 1 unit, 2 channels
- **USB**: 1 USB Full Speed Device
- **EXMC**: 1 External Memory Controller
- **DMA**: 2 controllers (DMA0: 7 channels, DMA1: 5 channels)

## Supported Configurations

### nsh

This configuration provides a NuttShell (NSH) with minimal system configuration.

**Features**:
- NuttShell command line interface
- Serial console on USART0 (PA9/PA10)
- System clock: 120MHz
- LED support (auto LED or user LED)
- Button support
- procfs file system

**Build and Flash**:

```bash
cd nuttx
./tools/configure.sh gd32e113vb-eval:nsh
make
```

**Serial Console**:
- USART0: 115200 baud, 8N1
- TX: PA9
- RX: PA10

## Clock Configuration

The board supports multiple system clock frequencies:

- **120MHz** (default): Maximum performance
- **108MHz**: Balanced performance
- **96MHz**: USB-compatible
- **72MHz**: Lower power consumption

PLL Configuration:
```
HXTAL = 8MHz
PREDV0 = /2 = 4MHz
PLLMF = x30 (for 120MHz)
SYSCLK = 120MHz
HCLK = 120MHz (AHB /1)
PCLK2 = 60MHz (APB2 /2)
PCLK1 = 30MHz (APB1 /4)
```

## Pin Mapping

### Console (USART0)
| Function | Pin | Description |
|----------|-----|-------------|
| TX | PA9 | USART0 transmit |
| RX | PA10 | USART0 receive |

### LEDs
| LED | Pin | Active Level |
|-----|-----|--------------|
| LED1 | PC0 | High |
| LED2 | PC2 | High |
| LED3 | PE0 | High |
| LED4 | PE1 | High |

### Buttons
| Button | Pin | Active Level |
|--------|-----|--------------|
| WAKEUP | PA0 | Low (pressed) |
| TAMPER | PC13 | Low (pressed) |
| USER | PB14 | Low (pressed) |

## Memory Map

### Flash Memory
- **Start**: 0x08000000
- **Size**: 128KB (0x20000)
- **Sectors**: Organized in pages

### SRAM
- **Start**: 0x20000000
- **Size**: 32KB (0x8000)

### Peripheral Memory
- **APB1**: 0x40000000 - 0x40007FFF
- **APB2**: 0x40010000 - 0x40015FFF
- **AHB1**: 0x40020000 - 0x4007FFFF
- **AHB3**: 0xA0000000 - 0xA0000FFF (EXMC)

## Programming and Debugging

### Using OpenOCD

```bash
openocd -f interface/stlink.cfg -f target/gd32e1x.cfg
```

### Using GDB

```bash
arm-none-eabi-gdb nuttx
(gdb) target remote localhost:3333
(gdb) monitor reset halt
(gdb) load
(gdb) continue
```

### Using ST-Link

The board supports ST-Link v2 for programming and debugging via SWD interface:
- SWDIO: PA13
- SWCLK: PA14
- GND and VCC

## Development Notes

### Differences from GD32F4

1. **Memory**: 128KB Flash / 32KB SRAM (vs F4's 3MB / 256KB)
2. **Timers**: 8 timers total (TIMER0-7) (vs F4's 14 timers)
3. **ADC**: 2 ADC units (vs F4's 3 ADC units)
4. **DMA**: DMA0 has 7 channels, DMA1 has 5 channels (vs F4's 8+8)
5. **Clock**: Max 120MHz (vs F4's 240MHz)

### Driver Status

- [x] GPIO
- [x] USART
- [x] Interrupt controller
- [x] Systick timer
- [ ] SPI
- [ ] I2C
- [ ] ADC
- [ ] DAC
- [ ] Timers
- [ ] USB
- [ ] DMA
- [ ] EXMC

## References

- GD32E113 User Manual
- GD32E113 Datasheet
- [GigaDevice Official Website](http://www.gigadevice.com/)
- [NuttX Documentation](https://nuttx.apache.org/docs/latest/)

## License

This board support package is licensed under Apache License 2.0.

## Maintainer

Created: December 30, 2024
