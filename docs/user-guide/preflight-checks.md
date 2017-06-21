# Pre-Flight Checklist

This is an example of a ROSflight pre-flight checklist.  You will likely need to augment this with checks specific to your hardware and the code running on the onboard computer.

## Generic Checklist

### Before powering up motors
- ROS is running on onboard computer, communicating with base station
- Sensors are calibrated and publishing
    + IMU (re-calibrate every flight): `rosservice call /calibrate_imu`
    + Barometer: `rosservice call /calibrate_baro`
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


## ROSplane Checklist

### Before Leaving for the Flying Field
- ROS is running and communicating on both the base station and onboard computer
- Set fixed wing parameters
- Wireless network setup is working
- `rosflight_io` reports no errors
- RC communication and override check
- Proper directions on all control surfaces, from both RC and onboard computer
- Failsafe behavior is working
- logging is set up and working (automatic rosbag recording)
- blow into pitot tube and watch airspeed estimate
- move aircraft up and down and watch altitutde estimate (remember NED, so higher altitude = negative z)
- check for GPS fix (go outside)
- wire-wiggle test (look for sensor dropout and brownouts when wiggling all wires)
- walking estimator test (walk around outside with GPS and airspeed and watch estimator)

### After getting to flying field
- Set up Network
- Check RC communication
- Publish Commands to aircraft, check response
- Check RC switch setup
- Arm/Disarm Test
- GPS fix
- Check automatic logging
- Start external video recording
- Go fly!
