# Hall Effect Driver for ZMK

This directory contains a custom hall effect driver for the Serene65 keyboard in ZMK.

## Features

- Hall effect sensor support for analog key switches
- Multiplexer control for reading multiple sensors
- Calibration for noise floor and ceiling
- Multiple actuation modes:
  - Normal mode with configurable actuation and release points
  - Rapid trigger mode for gaming
- Configuration storage in settings

## Usage

The hall effect driver is automatically enabled for the Serene65 board. No additional configuration is needed.

## Calibration

To calibrate the hall effect sensors:

1. Boot the keyboard
2. The driver will automatically calibrate the noise floor on first boot
3. To calibrate the noise ceiling, press all keys down and run the calibration command (to be implemented)

## Configuration

The hall effect driver can be configured through the following Kconfig options:

- `CONFIG_ZMK_HALL_EFFECT`: Enable hall effect sensor support
- `CONFIG_ZMK_HALL_EFFECT_CALIBRATION`: Enable calibration for hall effect sensors
- `CONFIG_ZMK_HALL_EFFECT_RAPID_TRIGGER`: Enable rapid trigger mode for hall effect sensors

## Implementation Details

The hall effect driver consists of the following components:

- `hall_effect.h` and `hall_effect.c`: Core hall effect driver implementation
- `matrix_hall_effect.c`: Matrix driver that uses the hall effect driver
- `Kconfig`: Configuration options for the hall effect driver

The driver uses the ADC to read analog values from hall effect sensors through multiplexers. It then processes these values to determine key states based on configurable thresholds. 