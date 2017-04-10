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
- Parameter Check (if using a fixed wing, there are about 8 parameter you will need to change from default  )

### After Powering Up Motors (MAKE SURE THE FLIGHT CONTROLLER IS DISARMED!!!)
- Arm/Disarm test
- Propeller spin test (check directions and response to stick inputs)
- Control surface test (Fixed Wing)
- Response to offboard controls (Fixed Wing)


