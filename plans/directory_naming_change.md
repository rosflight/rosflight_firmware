# Directory Naming Change

**Date:** 2026-05-06  
**Change:** Renamed `common/drivers/` to `common/sensor_drivers/`

---

## Reason for Change

To avoid confusion and potential conflicts with the upcoming `common/Drivers/` directory (STM32 HAL drivers) on case-insensitive filesystems (Windows, macOS default).

## Directory Structure

```
boards/varmint_h7/common/
├── sensor_drivers/          # Sensor driver implementations (Bmi088, Mpu, etc.)
│   ├── Adc.cpp
│   ├── Adis165xx.cpp
│   ├── Bmi088.cpp
│   ├── Mpu.cpp
│   └── ... (sensor drivers)
│
└── Drivers/                 # STM32 HAL drivers (to be added in Phase 1)
    ├── STM32H7xx_HAL_Driver/
    └── CMSIS/
```

## Files Updated

- [`boards/varmint_h7/common/CMakeLists.txt`](../boards/varmint_h7/common/CMakeLists.txt)
  - Changed `../common/drivers/*.cpp` → `../common/sensor_drivers/*.cpp`
  - Changed include path `../common/drivers` → `../common/sensor_drivers`

## Naming Convention

- **`sensor_drivers/`** (lowercase) - Custom sensor driver implementations
- **`Drivers/`** (capitalized) - STM32 HAL/CMSIS drivers (STM32CubeMX convention)

This follows the STM32CubeMX naming convention where generated HAL drivers are placed in `Drivers/` with a capital D.

---

**Impact:** None on functionality, only directory naming for clarity.
