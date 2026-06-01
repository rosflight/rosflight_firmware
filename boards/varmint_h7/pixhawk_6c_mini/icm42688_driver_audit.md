# ICM-42688-P Driver Audit

## Scope

This audit covers the common ICM-42688-P driver in
`boards/varmint_h7/common/drivers/Icm42688.cpp` as configured by the Pixhawk 6C
Mini board in `Core/Src/Pixhawk6CMiniBoard.cpp`.

## Precision Mode

The driver currently implements maximum register precision mode, not FIFO
high-resolution mode. FIFO high-resolution mode is intentionally unsupported in
this driver revision because the current ROSflight board interface consumes one
IMU sample at a time and the board bring-up path already uses the data-ready
register path.

Register precision policy:

- Gyro ODR: 1 kHz.
- Accel ODR: 1 kHz.
- Gyro full-scale range: +/-500 dps.
- Accel full-scale range: +/-8 g.
- Gyro scale: `pi / 180 / 65.5` rad/s per LSB.
- Accel scale: `9.80665 / 4096` m/s^2 per LSB.
- Power mode: gyro and accel low-noise mode.

This changes gyro quantization from about `0.001064 rad/s` per LSB at +/-2000 dps
to about `0.000266 rad/s` per LSB at +/-500 dps.

## Static Findings

- SPI transaction format is correct for register mode: bit 7 set for reads,
  cleared for writes, followed by sequential data bytes.
- Sensor register samples are parsed as big-endian signed 16-bit two's
  complement values.
- Register-mode temperature conversion uses `TEMP_DATA / 132.48 + 25 C`.
- Data-ready is mapped to PE6 and starts a SPI1 DMA burst read.
- The emitted sample timestamp is captured at the DRDY edge; completion time is
  captured after the DMA transfer finishes.
- `INT_CONFIG1.INT_ASYNC_RESET` is cleared during initialization and read back.
- Initialization reads back precision-critical registers and fails the IMU init
  if any configured field does not match.

## Deliberate Non-Support

- FIFO high-resolution 20-byte packets are not implemented yet.
- FIFO packet parsing, FIFO lost-packet counters, and FIFO buffer sizing are not
  applicable until FIFO mode is added.
- Banked nonzero-register configuration for AAF/notch filters is not implemented
  in this revision.
- Self-test is not implemented in this revision.

## Remaining Hardware Validation

- Confirm PE6 data-ready frequency is 1 kHz within oscillator tolerance.
- Confirm no saturation for the intended vehicle rates at +/-500 dps.
- Confirm static accel magnitude is approximately 1 g after board rotation.
- Confirm gyro noise histogram no longer bins at the previous +/-2000 dps LSB
  size.
