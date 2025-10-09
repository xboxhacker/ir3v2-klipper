# IR3V2 Klipper Eddy Current Probe

A compact, eddy-current probing setup for the IdeaFormer IR3V2 conveyor-belt printer using the BDsensor and Klipper. This project adapts the BDsensor to act as a virtual endstop for homing and probing along the Y axis.

---

## Table of Contents

- [What you get](#what-you-get)
- [Hardware](#hardware)
- [Firmware and host setup](#firmware-and-host-setup)
- [Wiring](#wiring)
- [Klipper configuration](#klipper-configuration)
  - [[mcu] for the F103 board](#mcu-for-the-f103-board)
  - [[BDsensor] section](#bdsensor-section)
  - [[stepper_y] homing via virtual endstop](#stepper_y-homing-via-virtual-endstop)
- [Bring-up and calibration](#bring-up-and-calibration)
- [Daily use](#daily-use)
- [Troubleshooting](#troubleshooting)
- [Notes](#notes)
- [Credits](#credits)
- [License](#license)

---

## What you get

- A tiny eddy-current probe mounted in the IR3V2 fan duct
- Secondary MCU (STM32F103) running Klipper MCU to provide two GPIO pins for the BDsensor’s I2C-like interface
- Klipper setup that:
  - Reads live distance from the BDsensor
  - Uses a virtual endstop for Y homing: `endstop_pin: probe:y_virtual_endstop`
  - Probes along the Y axis (no belt-angle math yet)

---

## Hardware

- [BDSensor](https://github.com/markniu/Bed_Distance_sensor) module (Mark Niu’s Bed Distance sensor)
- IR3V2 belt printer (CoreXY + belt Z)
- STM32F103 development board (e.g., FK103M2) as a USB-connected, secondary Klipper MCU
- Wiring:
  - 3.3V (or sensor-appropriate VCC) and GND to the BDsensor
  - Two GPIOs from the F103 for SDA and SCL/CLK to the BDsensor
  - Pull-ups (typically 4.7k–10k to 3.3V) on SDA/SCL if your board/sensor doesn’t provide them
- Mount: BDsensor fixed in the fan duct with a gap target of roughly 0.4–2.4 mm when “at zero” (fine-tune during calibration)

---

## Firmware and host setup

1. Flash Klipper MCU firmware to the STM32F103 board (as a secondary MCU).
2. Connect the F103 to your printer host via USB.
3. Note the serial path (e.g., `/dev/serial/by-id/...`).
4. Add an `[mcu]` (or named `[mcu bds]`) entry in `printer.cfg` for the F103 board.

---

## Wiring

- Keep SDA/SCL short and away from stepper motor wires.
- Ensure a solid common ground between the F103 board and the BDsensor.
- If you see unstable readings or `10.24` “connection error,” increase the `delay` in the `[BDsensor]` section (try `8–10`) and confirm pull-ups.

Example pin choice on the F103:
- SDA → `PB7`
- SCL → `PB6`

You can pick any free GPIOs on the F103 as long as you reference the same pins in `[BDsensor]`.

---

## Klipper configuration

Below are minimal snippets you can adapt into your `printer.cfg`. Adjust pins and ranges to your machine.

### [mcu] for the F103 board

If you dedicate the F103 to the BDsensor, you can name it for clarity:

```ini
[mcu bds]
serial: /dev/serial/by-id/usb-Klipper_STM32F103_FK103M2-if00
# restart_method: command
```

### [BDsensor] section

The BDsensor module bit-bangs the interface on any two GPIOs. Start with a moderate `delay` (e.g., `8`) for robust comms.

```ini
[BDsensor]
# Use the F103 pins (or your main MCU pins if not using a secondary MCU)
sda_pin: bds:PB7
scl_pin: bds:PB6
# Alternatively, you can use 'clk_pin:' if preferred by your firmware fork

delay: 8                 # 5–10; higher can help if wiring is noisy/long

# Sensor logic parameters
position_endstop: 0.7    # trigger threshold in mm (tune after calibration)
z_adjust: 0.0            # keep 0.0 while tuning
z_offset: 0.0            # keep 0.0 while tuning
speed: 3.0               # internal sample timing in the module

# Optional behavior toggles (uncomment/tune as needed)
# deactivate_on_each_sample: True
# no_stop_probe: 1
# collision_homing: 0
# collision_calibrate: 0
# rt_sample_time: 0
# rt_max_range: 0
# QGL_Tilt_Probe: 1
# SWITCH_MODE_SAMPLE_TIME: 0.006
```

### [stepper_y] homing via virtual endstop

Homing uses the BDsensor’s virtual endstop on Y. Do not set `position_endstop` in `[stepper_y]`; it’s invalid there and belongs in `[BDsensor]`.

```ini
[stepper_y]
# Your real motor pins:
step_pin: <your_mcu>:PA8
dir_pin:  <your_mcu>:PC9
enable_pin: !<your_mcu>:PB6

rotation_distance: 40
microsteps: 32
full_steps_per_rotation: 200

# Virtual endstop provided by BDsensor:
endstop_pin: probe:y_virtual_endstop

# Direction you home:
# false = toward Y-min, true = toward Y-max
homing_positive_dir: false

# Travel limits:
position_min: -6.1
position_max: 500

# Homing behavior:
homing_speed: 10
second_homing_speed: 3
homing_retract_dist: 5
```

Tip: If homing moves the wrong direction, flip `homing_positive_dir` (don’t flip `dir_pin` unless normal motion is also reversed).

---

## Bring-up and calibration

1. Enable force moves (only if you cannot home yet):
   ```ini
   [force_move]
   enable_force_move: true
   ```
2. Verify communications:
   - Raw data (twice): `M102 S-7`
     - Expect a number like `Raw data: 220` when close-ish to the belt.
   - Distance: `M102 S-2` (or `BDSENSOR_DISTANCE`)
     - After calibration, expect a value like `2.20 mm`. If it shows `0.00 mm`, calibrate.
3. Calibrate the BDsensor:
   - Position the sensor roughly 0.4–2.4 mm from the belt (don’t crash).
   - Run: `BDSENSOR_CALIBRATE` (or `M102 S-6`)
   - Wait for “Calibrate Finished!”
4. Set trigger threshold so it’s OPEN when away from the belt:
   - Move to your “start” position (away).
   - Read distance: `M102 S-2` → call it `r` (e.g., `2.20 mm`).
   - Set threshold slightly below that so it’s open when away but triggers as you approach:
     - `BDSENSOR_SET POSITION_ENDSTOP={r - 0.20}` → example: `2.00`
   - `SAVE_CONFIG` and restart Klipper.
   - `QUERY_PROBE` should now show `open` when away, and flip to `TRIGGERED` when close.
5. Home Y:
   - `G28 Y`
   - If it travels the wrong way, flip `homing_positive_dir` and retry.

---

## Daily use

- Check probe:
  - `QUERY_PROBE` → `open` when away, `TRIGGERED` when close
  - `M102 S-2` → shows live distance in mm
- Optional commands:
  - Version (may show garbage on some units, not critical): `M102 S-1`
  - Raw data: `M102 S-7`
  - Reboot sensor: `M102 S-8`
  - Force switch mode: `M102 S-9`

---

## Troubleshooting

- Probe is always TRIGGERED:
  - Distance (`M102 S-2`) is `0.00 mm` → calibrate (`M102 S-6`).
  - Or your `position_endstop` is set too high; set it below the “away” reading (e.g., `r - 0.20`) and `SAVE_CONFIG`.
- `10.24` or “connection error”:
  - Raise `delay` (try `8–10`), confirm pull-ups, shorten/route SDA/SCL away from motor wiring, check grounds.
  - Double-check SDA/SCL aren’t swapped; try swapping if stuck.
- Homing doesn’t move:
  - `QUERY_PROBE` reads TRIGGERED at start → adjust `position_endstop` so it’s `open` when away.
  - Ensure `homing_retract_dist` is non-zero (e.g., `5`).
- Homing wrong direction:
  - Flip `homing_positive_dir` in `[stepper_y]`.

---

## Notes

- This setup probes along the Y axis only and does not yet account for the belt’s 45° kinematics. It’s sufficient for homing and simple probing.
- If you later add belt-angle compensation or bed-mesh logic, integrate it on top of these basics.

---

## Credits

- BDsensor by Mark Niu: [repository](https://github.com/markniu/Bed_Distance_sensor) • [issues](https://github.com/markniu/Bed_Distance_sensor/issues)
- Klipper: [https://www.klipper3d.org/](https://www.klipper3d.org/)
- IR3V2 by IdeaFormer

---

## License

This repository’s documentation is provided under the MIT License unless otherwise noted. The BDsensor firmware, hardware, and associated code are subject to their respective upstream licenses.
