# Repository with raspberry pico projects

[Proximity Faucet Controller](#proximity-faucet-controller)

## proximity-faucet-controller

A proximity faucet (also known as a touchless or sensor faucet) for almost any faucet. It was created for kitchen sink for elderly person to eliminate running water after forgetting to turn off the tap

### Components:
```
- Infrared Proximity Sensor E18-D80NK
- Power Supply Adapter - 24 Volt - 1 Ampere -DC 5,5/2,1mm male connector - MW Power ER24W24V
- SONGLE 5V, RELAY, Pinout – SRD-05VDC-SL-C module - connected as normally open
- DC Jack 5.5 x 2.1mm DC Power Female Jack Connector
- some AWG22 cables
- Raspberry Pi Pico - RP2040 ARM Cortex M0+ - with micropython installed
- Raspberry Pi Pico power adapter for USB supply
- 4-channel logic level converter - 5V - 3.3V - bidirectional
- 4x 6.3mm Female Spade Connector 
- 4x Slip On Cover for Female 6.3mm Spade Terminals
- 2x WAGO 221-413
- 2x 24V 6,3W Serie R ED 100% normally closed solenoid valve f.e. RPE R Mini 4115BC
```

### Schematic:
```
Raspberry Pi Pico (USB Powered)
│
├── Proximity Sensor:
│   ├── VCC → VBUS (5V)
│   ├── GND → Any Pico GND
│   ├── OUT → Logic Converter → GPIO14 (with pull-up)
│
│
├── Logic Converter:
│   ├── VCC -> HV (5V)
│   ├── 3V3(OUT) -> LV (3.3V)
│
│
└── Relay Control:
    ├── GPIO16 → Relay IN
    ├── Relay VCC → VBUS (5V)
    ├── Relay GND → Pico GND
    │
    └── Relay Output:
        ├── COM → 24V+ 
        ├── NO → Solenoid Valves+   (Through Wago 221-413 connector)
        └── Solenoid Valves- → 24V- (Through Wago 221-413 connector)
```