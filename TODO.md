# SUFST BMS Firmware — TODO List

## 1. Regenerative Braking (Regen) Support & Protection
- [x] Split over-current fault detection into asymmetric **max discharge current** (`> +I_max_discharge`) and **max regen/charge current** (`< -I_max_regen`).
- [x] Track `min_temp_deci_c` in `struct bms_ctx` to block charging/regen below minimum cell charge temperature (e.g. $0^\circ\text{C}$).
- [x] Calculate dynamic `regen_allowed` / `max_regen_current_ma` in `struct bms_outputs` (derating when `max_cell_mv` nears upper voltage limit or temps approach limits) to broadcast over CAN to the VCU/inverter.
- [x] Set `BMS_CURRENT_MAX_REGEN_MA` from the cell datasheet (140s3p Molicel P45B: 3 x 13.5 A = 40.5 A).
- [x] Set `BMS_CURRENT_MAX_DISCHARGE_MA` to 200 A (inverter maximum; above the P45B 3 x 45 A = 135 A continuous rating, so sustained current is limited by cell temperature faults).
- [ ] Broadcast `regen_allowed` / `max_regen_current_ma` over CAN once `fdcan1` telemetry exists.

## 2. State Machine & Safety Logic
- [x] Implement fault evaluation & debounce timers (cell OV/UV, over/under-temp, over-current, open-wire, AFE comm timeout).
- [ ] Implement persistent fault storage in internal Flash (`storage_partition`) using Zephyr NVS / Settings so latched faults survive power cycles.

## 3. Hardware & Drivers (Pending PCB Completion)
- [ ] Update [`sufst_bms.dts`](boards/sufst/sufst_bms/sufst_bms.dts) pin assignments to match final schematic (resolve `PA5` status LED vs. `SPI1_SCK` conflict, add GPIO nodes for `AIR-`, `AIR+`, `Precharge`, and `SDC_FAULT`).
- [ ] Wire `apply_hardware_outputs()` in [`src/bms_sm.c`](src/bms_sm.c) to DeviceTree GPIO specs.
- [ ] Implement AFE driver (`spi1` / isoSPI) for cell voltage and temperature acquisition.
- [x] IVT-S current sensor receive on `fdcan2` (PB12/PB13, 500 kbit/s, default IDs 0x521..0x528): feeds `pack_current_ma`, timeout / sensor-error fault, shown in `bms status`.
- [ ] Verify IVT-S on the bench: current sign (`BMS_CURRENT_SENSOR_SIGN`), that the sensor powers up in run mode, and bus termination.
- [ ] Decide what IVT-S U1..U3 are wired to and cross-check them against the AMC3330 pack / DC-link readings.
- [ ] Implement VCU CAN telemetry and commands (TS request, regen limit, faults); `fdcan3` (PA8/PA15) is enabled in the devicetree but unused.
- [x] USB CDC-ACM virtual COM port with Zephyr shell: `bms status`, `bms clear_faults` (manual fault clear from PC).
- [ ] Confirm USB works on real hardware (HSI48 + CRS clock recovery in [`src/bms_usb.c`](src/bms_usb.c), D-/D+ on PA11/PA12).
- [ ] Replace the Zephyr test USB VID/PID (`0x2fe3:0x0004`) if the board is ever distributed outside the team.
