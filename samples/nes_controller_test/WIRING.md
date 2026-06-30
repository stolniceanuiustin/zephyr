# NES Controller Wiring — ZedBoard JA PMOD

## NES Controller Plug (looking at plug end, left to right)

```
┌─────────────────────────┐
│ 1  2  3  4  5  6  7    │
└─────────────────────────┘
  │  │  │  │  │  │  │
 VCC NC NC GND DAT CLK LAT
```

| NES Pin | Signal | 
|---------|--------|
| 1       | VCC    |
| 2       | N/C    |
| 3       | N/C    |
| 4       | GND    |
| 5       | DATA (serial out from controller) |
| 6       | CLOCK  |
| 7       | LATCH  |

## ZedBoard JA PMOD Header (top row, left to right)

```
┌──────────────────────────────────┐
│ JA1  JA2  JA3  JA4  GND   VCC  │
└──────────────────────────────────┘
```

## Connection Table

| NES Pin | Signal | Wire to  |
|---------|--------|----------|
| 1       | VCC    | JA VCC   |
| 4       | GND    | JA GND   |
| 7       | LATCH  | JA1      |
| 6       | CLOCK  | JA2      |
| 5       | DATA   | JA3      |

## Vivado XDC Constraints

```tcl
set_property PACKAGE_PIN Y11  [get_ports {GPIO_0_0_tri_io[0]}]  ;# JA1 = LATCH
set_property PACKAGE_PIN AA11 [get_ports {GPIO_0_0_tri_io[1]}]  ;# JA2 = CLOCK
set_property PACKAGE_PIN Y10  [get_ports {GPIO_0_0_tri_io[2]}]  ;# JA3 = DATA
set_property IOSTANDARD LVCMOS33 [get_ports {GPIO_0_0_tri_io[*]}]
```

## Zephyr Device Tree (app.overlay)

EMIO GPIO starts at psgpio_bank2, pin 0.

```
EMIO[0] = JA1 = LATCH  -> psgpio_bank2 pin 0
EMIO[1] = JA2 = CLOCK  -> psgpio_bank2 pin 1
EMIO[2] = JA3 = DATA   -> psgpio_bank2 pin 2
```
