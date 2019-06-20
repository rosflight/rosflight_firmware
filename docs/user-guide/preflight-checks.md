# Pre-Flight Checklist

This is an example of a ROSflight pre-flight checklist. You will likely need to augment this with checks specific to both (a) your hardware and (b) the code running on your onboard computer.

## Generic Checklist

### Before powering up motors
- ROS is running on the onboard computer, communicating with the base station
- Sensors are calibrated and publishing
    + IMU (re-calibrate every flight): `rosservice call /calibrate_imu`
    + Barometer: `rosservice call /calibrate_baro`
    + Sonar (if attached)
    + Airspeed (if attached)
- Estimated attitude is being published and looks accurate
- Published outputs look reasonable
- Parameter Check (if using a fixed-wing, there are about 8 parameters you will need to change from default)
- RC communication
- Failsafe behavior
- Arming and disarming
- RC override behavior
- RC range test
- Wire-wiggle test (wiggle all wires to look for bad connections)

### After Powering Up Motors

!!! warning
    Be sure the flight controller is disarmed before powering up motors!!!
- Arm/Disarm test
- Propeller spin test (check directions and response to stick inputs)
- Control surface test (fixed-wing)
- Response to offboard controls


## ROSplane Checklist

### Before Leaving for the Flying Field
- ROS is running and communicating on both the base station and onboard computer
- Set fixed-wing parameters
- Wireless network setup is working
- `rosflight_io` reports no errors
- RC communication and override check
- Proper directions on all control surfaces, from both RC and onboard computer
- Failsafe behavior is working
- Logging is set up and working (automatic rosbag recording)
- Blow into pitot tube and watch airspeed estimate
- Move aircraft up and down and watch altitude estimate (remember NED, so higher altitude = negative z)
- Check for GPS fix (go outside)
- Wire-wiggle test (look for sensor or RC dropout and brownouts when wiggling all wires - watch output from `rosflight_io` for potential errors)
- Walking estimator test (walk around outside with GPS and airspeed and watch estimator)

### After getting to flying field
- Set up Network
- Check RC communication
- Publish Commands to aircraft, check response
- Check RC switch setup
- Arm/Disarm Test
- GPS fix
- Check automatic logging
- Start external video recording
- Fly!
