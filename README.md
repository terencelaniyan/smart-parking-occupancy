# Smart Parking Occupancy Detection System
Purdue University Indianapolis — ECE 49200 Senior Design, Spring 2026

## Overview
5-space embedded parking detection system using IR sensors, RGB LEDs, 
ILI9341 TFT LCD, and solar-assisted power on the Purdue Proton (ARM Cortex-M33).

## Hardware
- Purdue Proton board (RP2350B / ARM Cortex-M33)
- IR obstacle sensor modules (×5)
- 5mm RGB LEDs with current-limiting resistors
- ILI9341 TFT LCD via SPI at 62.5 MHz
- Waveshare Solar Power Manager + 18650 Li-ion cell

## Features
- Real-time per-space occupancy detection
- Color-coded LED status (green/red/blue/yellow)
- Handicap and maintenance mode indication
- Manual operator override with debounced hold-press logic
- Occupancy analytics + space recommendation via serial console
- Solar-assisted rechargeable power path

## Tech Stack
C, Pico SDK, KiCad 9.0, SPI, GPIO, ADC

## Repo Structure
```
smart-parking-occupancy/
├── firmware/
│   └── main.c          # Full embedded C firmware
├── hardware/
│   └── Smart_parking_project_.kicad_sch
├── docs/
│   └── Final_Report_SD-6.pdf
└── README.md
```

## Team
Fahd Laniyan · Essa Alansari · Nathan Bradshaw · Maitham Sulais
