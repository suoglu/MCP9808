# MCP9808 Temperature Sensor

## Contents of Readme

1. About
2. Modules
3. Interface Description
4. Test
5. Status Information
6. Possible Improvements

[![Repo on GitLab](https://img.shields.io/badge/repo-GitLab-6C488A.svg)](https://gitlab.com/suoglu/mcp9808)
[![Repo on GitHub](https://img.shields.io/badge/repo-GitHub-3D76C2.svg)](https://github.com/suoglu/MCP9808)

---

## About

[MCP9808](https://ww1.microchip.com/downloads/en/DeviceDoc/25095A.pdf) is a digital temperature sensor with I²C interface.

## Modules

Version 1 of the module `mcp9808` is a basic interface module for [MCP9808](https://ww1.microchip.com/downloads/en/DeviceDoc/25095A.pdf) temperature sensor. Version 1 supports reading ambient temperature register, writing to boundary registers, changing measurement resolution and taking sensor to shutdown mode.

## Interface Description

|   Port   | Type | Width |  Description |
| :------: | :----: | :----: |  ------  |
| `clk` | I | 1 | System Clock |
| `rst` | I | 1 | System Reset |
| `clkI2Cx2` | I | 1 | Double I²C Clock (800 kHz) |
| `addressPins` | I | 3 | Address pins of MCP9808 |
| `SCL`* | IO | 1 | I²C Clock |
| `SDA`* | IO | 1 | I²C Data |
| `tempVal` | O | 12 | Ambient Temperature Absolute Value (2^7...2^-4) |
| `tempSign` | O | 1 | Ambient Temperature Sign |
| `tempComp` | O | 3 | Boundary Comparison Results (Crit,Upper,Lower) |
| `tempInput` | I | 11 | Boundary Temperature Input (sign,2^7...2^-2) |
| `tempWrite` | I | 2 | Write Boundary Temperature (see below) |
| `res_i` | I | 2 | Measurement Resolution (see below) |
| `shutdown` | I | 1 | Take module to shutdown mode |
| `update` | I | 1 | Update temp registers data |
| `ready` | O | 1 | Module is ready for a new operation |

I: Input  O: Output

\* contain pins \_i, \_o and \_t

| `tempWrite` | Register to be written |
| :------: | :----: |
| 2'b00 | No Operation |
| 2'b01 | Lower Boundary |
| 2'b10 | Upper Boundary |
| 2'b11 | Critical Boundary |

| ``res_i`` | Resolution | t<sub>conv</sub> | Default |
| :------: | :----: | :----: | :----: |
| 2'b00 | 0.5 °C | 30 ms | |
| 2'b01 | 0.25 °C | 65 ms | |
| 2'b10 | 0.125 °C | 130 ms | |
| 2'b11 | 0.625 °C | 250 ms | x |

## Test

### Test 1 (20 April 2021)

Module tested on [testboard.v](Test/testboard.v). Right button used to read a new measurement result, and left button used to initiate a new boundry write. Left most switch is used for shutdown mode, following two switches used to select which boundary register to write, following two switches used to change resolution and remaining switches used as temperature input. Read temperature measurement displayed on seven segment display. I²C pins connected to JB header. During testing, I²C signals monitored via [Digilent Digital Discovery](https://reference.digilentinc.com/reference/instrumentation/digital-discovery/start). Reading temperature measurement result, writing to boundary registers, shutdown mode and changing measurement resolution tested.

## Status Information

**Last Test:** 20 April 2021, on [Digilent Basys 3](https://reference.digilentinc.com/reference/programmable-logic/basys-3/reference-manual).

## Possible Improvements

- Implement more functionality, e.g. alert configurations
