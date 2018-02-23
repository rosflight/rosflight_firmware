This page provides an overview of the basic operation of the ROSflight firmware and the concepts behind the behaviors.

# Companion computer

ROSflight is primarily intended to be used with an onboard companion computer running ROS.
The ROS interface is provided by the [rosflight_io](http://wiki.ros.org/rosflight) node.
All configuration of the flight controller is done through the ROS service API provided by `rosflight_io` (see the [parameter configuration](/user-guide/parameter-configuration) documentation page).
Sensor data such as IMU measurements are streamed from the flight controller to the companion computer and published as ROS topics.
Control setpoints can also be sent to the flight controller by publishing to the appropriate ROS topic (see the [autonomous flight](/user-guide/autonomous-flight) documentation page).

_A quick note on terminology:_ We commonly refer to the companion computer as the "onboard" computer, since it is carried onboard the vehicle.
However, we also refer to the control setpoints passed from that computer to the flight controller as "offboard" control.
The control is "offboard" from the perspective of the flight controller, even though the computer providing those commands is "onboard" the vehicle.
Perhaps not the greatest terminology; but hopefully this helps clear up potential confusion.

# RC safety pilot

ROSflight is designed for use with offboard control from experimental and research code.
As such, it provides several mechanisms for an RC safety pilot to intervene if something goes wrong with the control setpoints coming from the onboard computer:

  - **RC override switch:** The safety pilot can flip a switch on the transmitter to take back RC control. Attitude and throttle override can be mapped independently, meaning you can choose one or the other, put them on separate switches, or put them both on the same switch. Details on these switches are provided on the [RC configuration](/user-guide/rc-configuration) page.
  - **Stick deviations:** If a stick is deviated from its center position, then that channel is overridden by RC control. This allows the safety pilot to take control without flipping a switch, or to provide a momentary correction on a single axis. The fraction of stick travel needed to active RC override is controlled by the `RC_OVRD_DEV` parameter. The amount of time that the override remains active after the sticks returned to center is controlled by the `OVRD_LAG_TIME` parameter.
  - **Minimum throttle:** By default, the flight controller takes the minimum of the two throttle commands from RC and offboard control setpoints. This allows the safety pilot to drop the throttle quickly if needed. This behavior can be turned on or off with the `MIN_THROTTLE` parameter.

# Arming, errors & failsafe

The flight controller can only be armed and disarmed via RC control.
Two mechanisms are provided: sticks (left stick down and right to arm, down and left to disarm) and switch.
Only one of these options can be active at a time.
Details on configuration are given on the [RC configuration](/user-guide/rc-configuration) page.

The firmware runs a number of error checks before allowing the flight controller to arm.
Completing the configuration checklist on the [Getting Started](/user-guide/getting-started) page should avoid these errors.
In addition to a few internal health checks, the following conditions are checked:

  - **Mixer:** Valid mixer must have been selected (see the [hardware setup](/user-guide/hardware-setup) documentation page)
  - **IMU calibration:** The IMU must have been calibrated since firmware was flashed (it is recommended that you recalibrate often)
  - **RC:** There must be an active RC connection

In addition to the error checking before arming, the flight controller enters a failsafe mode if the RC connection is lost during flight while armed.
While in failsafe mode the flight controller commands level flight with the throttle value defined by the `FAILSAFE_THR` parameter.

The following is a simplified version of the finite state machine that defines logic used for the arming, error checks, and failsafe operations:

![Arming FSM](images/arming-fsm-simplified.svg)

# LEDs

The meaning of the various LEDs is summarized in the following tables. The colors of the LEDs may change depending on your specific board:

| LED           | On            | Off              | Slow Blink       | Fast Blink       |
|---------------|---------------|------------------|------------------|------------------|
| Power (Blue)  | Board powered | -                | -                | -                |
| Info (Green)  | RC control    | Offboard control | -                | -                |
| Warning (Red) | Armed         | Disarmed         | Error (disarmed) | Failsafe (armed) |
