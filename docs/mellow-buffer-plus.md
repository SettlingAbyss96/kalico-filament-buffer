# Mellow Fly LLL Buffer Plus as a Kalico MCU

How the buffer was converted from Mellow's standalone firmware to a Kalico MCU, and everything
learned about its hardware. Done and verified on a Voron 2.4 on 2026-10-03.

## Hardware facts

| Item | Value |
|---|---|
| MCU | STM32F072 (Mellow's build files say F072C8 / 64 KiB, but the ROM DFU reports **128 KiB** of flash: 64 × 2 KiB pages) |
| Clock | 8 MHz crystal |
| USB | PA11/PA12 (native, no remap) |
| Power | 12–24 V on VIN. USB is used for data. **Don't hot-plug USB into the Pi while the printer runs**: plugging this 24 V-powered board in once dropped the printer's other USB devices (mainboard and CAN adapter) and needed a power cycle. A data-only cable (5 V not connected) is recommended |
| Stock firmware | open source, PlatformIO/Arduino: https://github.com/FLY3DTeam/Buffer |

## Pin map (from the stock source, verified on hardware)

| Function | Pin | Polarity / notes |
|---|---|---|
| Motor STEP | PC13 | stock firmware never used STEP/DIR (it ran the TMC in VACTUAL velocity mode) |
| Motor DIR | PA7 | positive move = feed toward the toolhead |
| Motor EN | PA6 | active low |
| Motor driver | PB1 | **TMC2208** (TMC222x family, `IOIN` version 0x20) on single-wire UART, 0.11 Ω sense resistor. Configure it as `[tmc2208 ...]` |
| Hall pos1 | PB4 | high = blocked. First along the travel; the slider rests here with no filament |
| Hall pos2 | PB3 | high = blocked. Middle: the target |
| Hall pos3 | PB2 | high = blocked. End of travel, most spring compression |
| Inlet filament switch | PB7 | **low = filament present** |
| Feed / retract buttons | PB12 / PB13 | low = pressed |
| LEDs | PA8 (blue) / PA15 (red) | |
| Runout output | PB15 | stock firmware's signal to a mainboard; unused under Kalico |

**Configure the driver as a TMC2208.** Configured as a TMC2209 it still runs, but `IOIN` decodes
wrongly (`enn` appears stuck at 0 and `diag` toggles with PA6). As `[tmc2208 ...]` Kalico decodes it as
`IOIN@TMC222x` and `enn` follows the enable pin.

**Stock-firmware bug noted:** `DIR_PIN` is defined twice in `buffer.h` (PA7, then PB11). The second
wins, which is harmless in stock (it never uses DIR) but a reason to trust the header's labels only
after testing.

## Conversion procedure

1. **Back up the stock firmware first.** Put the board in ROM DFU: hold BOOT, tap RESET, release BOOT.
   ```
   sudo dfu-util -d 0483:df11 -a 0 -s 0x08000000:131072 -U buffer-stock-flash-128k.bin
   sudo dfu-util -d 0483:df11 -a 1 -s 0x1FFFF800:16 -U buffer-stock-optionbytes.bin
   ```
   Check the option bytes: first byte `0xAA` = RDP level 0 (not read-protected), so the image is
   exact. Ours: flash sha256 `6fbaadea…6190`, ~46 KB of code plus settings in the last page.
2. **Flash Katapult** (config: `docs/firmware/buffer-f072-katapult.config`: 8 KiB application offset,
   8 MHz, USB PA11/PA12, status LED PA8, double-reset entry). Mass-erase so no stock code is left
   where Katapult expects the application:
   ```
   sudo dfu-util -d 0483:df11 -a 0 -s 0x08000000:mass-erase:force:leave -D katapult.bin
   ```
   **Then tap RESET once.** The ROM `leave` jumps into the new code without a real reset, so the
   ROM's clock setup is still active and USB fails with `error -71`. After a clean reset it appears
   as `usb-katapult_stm32f072xb_<id>`.
3. **Flash Kalico through Katapult** (config: `docs/firmware/buffer-f072-usb.config`: STM32F072,
   8 KiB bootloader, 8 MHz, USB; builds to ~42 KB):
   ```
   make KCONFIG_CONFIG=buffer-f072-usb.config OUT=$HOME/firmware-builds/buffer-f072-usb/
   python3 ~/katapult/scripts/flashtool.py -d /dev/serial/by-id/usb-katapult_stm32f072xb_<id>-if00 \
       -f ~/firmware-builds/buffer-f072-usb/klipper.bin
   ```
   It reboots as `usb-Klipper_stm32f072xb_<id>`. Future updates need no buttons:
   `flashtool.py -d <Klipper serial> -r`, then the same flash command.
4. **Configure:** `config/mellow-buffer-plus.cfg`, then run the hardware tests in
   `config/buffer-test.cfg` (`BUFFER_TEST_HELP`).

## Restoring the stock firmware

Enter ROM DFU (BOOT + RESET), then:
```
sudo dfu-util -d 0483:df11 -a 0 -s 0x08000000:mass-erase:force:leave -D buffer-stock-flash-128k.bin
```
and tap RESET. Alternatively, build the stock firmware from https://github.com/FLY3DTeam/Buffer.
