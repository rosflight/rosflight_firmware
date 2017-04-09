# Pre-Flight Checklist

This is an example of a ROSflight pre-flight checklist.  You will likely need to augment this with checks specific to your hardware and the code running on the onboard computer.

### Before powering up motors
- ROS is running on onboard computer, communicating with base station
- Sensors are calibrated and publishing
    + IMU (re-calibrate every flight)
    + Magnetometer
    + Barometer
    + Sonar (if attached)
    + Airspeed (if attached)
- Estimated attitude is being published and looks accurate
- RC communication
- Arming and disarming
- Published outputs look reasonable
- Failsafe behavior
- RC override behavior
- RC range test
- Wire-wiggle test (wiggle all wires to look for bad connections)

### After Powering Up Motors (MAKE SURE THE FLIGHT CONTROLLER IS DISARMED!!!)
- Arm/Disarm test
- Propeller spin test (check directions and response to stick inputs)
- Control surface test (Fixed Wing)
- Response to offboard controls (Fixed Wing)

# Fixed Wing Parameter Configuration

Because ROSflight ships with default parameters for multirotors, you will probably want to change the following parameters if you want to fly a fixed wing aircraft.


| Parameter | Description | Type | Fixed Wing Value
|-----------|-------------|------|---------------|
| MOTOR_PWM_UPDATE | Refresh rate of motor commands to motors and servos (Hz) - See motor documentation | int |  50 |
| ARM_SPIN_MOTORS | Enforce MOTOR_IDLE_PWM | int |  false |
| MOTOR_IDLE_THR | Idle PWM sent to motors at zero throttle (Set above 1100 to spin when armed) | float |  0.0 |
| ARM_CHANNEL | RC switch channel mapped to arming [0 indexed, -1 to disable] | int |  4 |
| FIXED_WING | switches on passthrough commands for fixedwing operation | int |  true |
| MIXER | Which mixer to choose - See Mixer documentation | int | 4  |
| ELEVATOR_REV | reverses elevator servo output | int |  0/1 |
| AIL_REV | reverses aileron servo output | int |  0/1 |
| RUDDER_REV | reverses rudder servo output | int |  0/1 |


