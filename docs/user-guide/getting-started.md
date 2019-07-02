# Getting Started

Reading through the pages in this user guide in order should provide you with the information you need to get a vehicle flying with ROSflight. The following is a summary of the steps you'll need to follow to get your vehicle set up, with links to the corresponding documentation pages:

  1. [Set up your hardware](hardware-setup.md) (fixed-wing or multirotor platform, flight controller, and companion computer)
  2. [Flash your flight controller with the latest ROSflight firmware](flight-controller-setup.md)
  3. [Set up your RC transmitter](rc-configuration.md)
  4. [Set up ROS on your companion computer](ros-setup.md)
  5. [Configure the flight controller for your setup](parameter-configuration.md): the configuration checklists below should help guide you through this process
  6. [Run through your preflight checks](preflight-checks.md)
  7. [Tune the attitude controller gains](performance.md) (multirotors only)
  8. [Set up autonomous flight via offboard control](autonomous-flight.md) (optional)

## Configuration Checklist

The following checklists should help you get a new vehicle set up for the first time. This checklist assumes that your [hardware is already set up correctly](hardware-setup.md).

### General Setup

  1. Set the `FIXED_WING` parameter (`1` if a fixed-wing, `0` if a multirotor)
  2. Set the `RC_TYPE` parameter (`0` if PPM, `1` if SBUS)
  3. Set the `MIXER` parameter to the appropriate value described in the [Hardware Setup](hardware-setup.md) page
  4. Set the `MOTOR_PWM_UPDATE` parameter (typically `490` for SimonK ESCs, `50` for standard servos)
  5. Make sure your [RC transmitter is set up correctly](rc-configuration.md)
  6. Set up your RC switches
      * If you want to arm/disarm using a switch, set the `ARM_CHANNEL` parameter to the appropriate channel (0-indexed)
      * If you want to use a switch to enable RC override, set the `RC_ATT_OVRD_CHN` and `RC_THR_OVRD_CHN` parameters to the appropriate channel(s) (0-indexed). If you want complete control (attitude and throttle) when you flip the switch, set both these parameters to the same channel.
  7. Calibrate your IMU: start `rosflight_io`, then run `rosservice call /calibrate_imu`
  8. Complete the multirotor-specific or fixed-wing-specific checklist below
  9. Save the parameters (`rosservice call /param_write`)
  10. You'll probably want to save a backup of your parameters (call `rosservice call /param_save_to_file /path/to/file.yml`)
  11. Make sure you run through the [Preflight Checklist](preflight-checks.md) before flying

### Multirotor-specific Setup

!!! danger
    **IMPORTANT:** Remove all props from the vehicle when calibrating ESCs!!!

  1. Calibrate ESCs
      1. Make sure `MOTOR_MIN_PWM` and `MOTOR_MAX_PWM` are correct (usually `1000` and `2000`)
      1. Set `MIXER` param to `0` (ESC calibration mixer)
      1. Set `ARM_SPIN_MOTORS` to `0`
      1. Perform ESC calibration. For standard ESCs:

          1. With power disconnected from the motors, arm the flight controller
          1. Set throttle to maximum
          1. Connect power to the motors
          1. Drop the throttle to minimum

      1. Set the `MIXER` parameter back to the appropriate value for your vehicle (see the [Hardware Setup](hardware-setup.md#motor-layouts) page)
      1. Set `ARM_SPIN_MOTORS` back to `1`

  1. The `ARM_SPIN_MOTORS` parameter should be set to `1` so the motors spin slowly when armed. The idle throttle setting can be adjusted with the `MOTOR_IDLE_THR` parameter.
  1. You'll most likely want to set the `CAL_GYRO_ARM` param to `1` to enable calibrating gyros before arming
  1. Set the `RC_ATT_MODE` parameter to set RC control mode (`0` for rate mode, `1` for angle mode [default])
  2. Set torque offsets as described in the [RC trim calculation](performance.md#rc-trim) section of the Improving Performance page
  3. Tune the controller gains as described in the [Multirotor gain tuning](performance.md#gain-tuning) section of the Improving Performance page

### Fixed-Wing-Specific Setup

  1. Reverse servo directions if necessary using the `AIL_REV`, `ELEVATOR_REV`, and `RUDDER_REV` parameters (`1` to reverse, `0` to keep current direction)
  1. You'll most likely want to set the `ARM_SPIN_MOTORS` parameter to `0` so that the prop doesn't spin at a minimum throttle setting when you arm, especially if you'll be doing hand launching
