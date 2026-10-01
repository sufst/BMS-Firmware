# SUFST BMS Firmware — TODO List

## 1. Regenerative Braking (Regen) Support & Protection
- [ ] Split over-current fault detection into asymmetric **max discharge current** (`> +I_max_discharge`) and **max regen/charge current** (`< -I_max_regen`).
- [ ] Track `min_temp_deci_c` in `struct bms_ctx` to block charging/regen below minimum cell charge temperature (e.g. $0^\circ\text{C}$).
- [ ] Calculate dynamic `regen_allowed` / `max_regen_current_ma` in `struct bms_outputs` (derating when `max_cell_mv` nears upper voltage limit or temps approach limits) to broadcast over CAN to the VCU/inverter.

## 2. State Machine & Safety Logic
- [x] Implement fault evaluation & debounce timers (cell OV/UV, over/under-temp, over-current, open-wire, AFE comm timeout).
- [ ] Implement persistent fault storage in internal Flash (`storage_partition`) using Zephyr NVS / Settings so latched faults survive power cycles.

## 3. Hardware & Drivers (Pending PCB Completion)
- [ ] Update [`sufst_bms.dts`](boards/sufst/sufst_bms/sufst_bms.dts) pin assignments to match final schematic (resolve `PA5` status LED vs. `SPI1_SCK` conflict, add GPIO nodes for `AIR-`, `AIR+`, `Precharge`, and `SDC_FAULT`).
- [ ] Wire `apply_hardware_outputs()` in [`src/bms_sm.c`](src/bms_sm.c) to DeviceTree GPIO specs.
- [ ] Implement AFE driver (`spi1` / isoSPI) for cell voltage and temperature acquisition.
- [ ] Implement CAN bus (`fdcan1`) telemetry and PC application command handler (including manual fault clear).
