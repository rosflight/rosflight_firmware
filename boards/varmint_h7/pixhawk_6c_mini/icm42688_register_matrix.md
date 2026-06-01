# ICM-42688-P Register Matrix

Expected board-local initialization values for the Pixhawk 6C Mini driver.

| Register | Address | Expected | Purpose |
|---|---:|---:|---|
| `DEVICE_CONFIG` | `0x11` | `0x01` | Soft reset before configuration |
| `REG_BANK_SEL` | `0x76` | `0x00` | Use bank 0 for runtime register path |
| `WHO_AM_I` | `0x75` | `0x47` | ICM-42688-P identity |
| `PWR_MGMT0` | `0x4E` | low nibble `0x0F` | Gyro and accel low-noise mode |
| `GYRO_CONFIG0` | `0x4F` | `0x46` | +/-500 dps, 1 kHz ODR |
| `ACCEL_CONFIG0` | `0x50` | `0x26` | +/-8 g, 1 kHz ODR |
| `SIGNAL_PATH_RESET` | `0x4B` | `0x0A` | Reset timestamp and signal paths |
| `INT_CONFIG1` | `0x64` | bit 4 clear | Disable async reset for INT operation |
| `INT_CONFIG` | `0x14` | `0x1B` | Interrupt pin electrical configuration |
| `INT_SOURCE0` | `0x65` | `0x08` | Map data-ready interrupt to INT1 |

The driver reads back `GYRO_CONFIG0`, `ACCEL_CONFIG0`, `PWR_MGMT0`,
`INT_CONFIG1`, `INT_CONFIG`, and `INT_SOURCE0` before marking the IMU initialized.
