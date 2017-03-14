| Parameter | Description | Default Value | Min | Max |
|-----------|-------------|---------------|-----|-----|
| BOARD_REV | Major board revision of naze32/flip32 |  2 | 1 | 6 |
| BAUD_RATE | Baud rate of MAVlink communication with onboard computer |  921600 | 9600 | 921600 |
| SYS_ID | Mavlink System ID |  1 | 1 | 255 |
| STRM_HRTBT | Rate of heartbeat streaming (Hz) |  1 | 0 | 1000 |
| STRM_ATTITUDE | Rate of attitude stream (Hz) |  100 | 0 | 1000 |
| STRM_IMU | Rate of IMU stream (Hz) |  500 | 0 | 1000 |
| STRM_MAG | Rate of magnetometer stream (Hz) |  75 | 0 | 75 |
| STRM_BARO | Rate of barometer stream (Hz) |  100 | 0 | 100 |
| STRM_AIRSPEED | Rate of airspeed stream (Hz) |  20 | 0 | 50 |
| STRM_GPS | Rate of GPS stream (Hz) |  0 | 0 | 1 |
| STRM_SONAR | Rate of sonar stream (Hz) |  40 | 0 | 40 |
| STRM_SERVO | Rate of raw output stream |  50 | 0 | 490 |
| STRM_RC | Rate of raw RC input stream |  50 | 0 | 50 |
| PARAM_MAX_CMD | saturation point for PID controller output |  1000 | 0 | 1000 |
| PID_ROLL_RATE_P | Roll Rate Proportional Gain |  0.070f | 0.0 | 1000.0 |
| PID_ROLL_RATE_I | Roll Rate Integral Gain |  0.000f | 0.0 | 1000.0 |
| PID_ROLL_RATE_D | Rall Rate Derivative Gain |  0.000f | 0.0 | 1000.0 |
| ROLL_RATE_TRIM | Roll Rate Trim - See RC calibration |  0.0f | -1000.0 | 1000.0 |
| MAX_ROLL_RATE | Maximum Roll Rate command accepted into PID controllers |  3.14159f | 0.0 | 1000.0 |
| PID_PITCH_RATE_P | Pitch Rate Proporitional Gain |  0.070f | 0.0 | 1000.0 |
| PID_PITCH_RATE_I | Pitch Rate Integral Gain |  0.0000f | 0.0 | 1000.0 |
| PID_PITCH_RATE_D | Pitch Rate Derivative Gain |  0.0000f | 0.0 | 1000.0 |
| PITCH_RATE_TRIM | Pitch Rate Trim - See RC calibration |  0.0f | -1000.0 | 1000.0 |
| MAX_PITCH_RATE | Maximum Pitch Rate command accepted into PID controllers |  3.14159f | 0.0 | 1000.0 |
| PID_YAW_RATE_P | Yaw Rate Proporitional Gain |  0.25f | 0.0 | 1000.0 |
| PID_YAW_RATE_I | Yaw Rate Integral Gain |  0.0f | 0.0 | 1000.0 |
| PID_YAW_RATE_D | Yaw Rate Derivative Gain |  0.0f | 0.0 | 1000.0 |
| YAW_RATE_TRIM | Yaw Rate Trim - See RC calibration |  0.0f | -1000.0 | 1000.0 |
| MAX_YAW_RATE | Maximum Yaw Rate command accepted into PID controllers |  1.507f | 0.0 | 1000.0 |
| PID_ROLL_ANG_P | Roll Angle Proporitional Gain |  0.15f | 0.0 | 1000.0 |
| PID_ROLL_ANG_I | Roll Angle Integral Gain |  0.0f | 0.0 | 1000.0 |
| PID_ROLL_ANG_D | Roll Angle Derivative Gain |  0.07f | 0.0 | 1000.0 |
| ROLL_TRIM | Roll Angle Trim - See RC calibration |  0.0f | -1000.0 | 1000.0 |
| MAX_ROLL_ANG | Maximum Roll Angle command accepted into PID controllers |  0.786f | 0.0 | 1000.0 |
| PID_PITCH_ANG_P | Pitch Angle Proporitional Gain |  0.15f | 0.0 | 1000.0 |
| PID_PITCH_ANG_I | Pitch Angle Integral Gain |  0.0f | 0.0 | 1000.0 |
| PID_PITCH_ANG_D | Pitch Angle Derivative Gain |  0.07f | 0.0 | 1000.0 |
| PITCH_TRIM | Pitch Angle Trim - See RC calibration |  0.0f | -1000.0 | 1000.0 |
| MAX_PITCH_ANG | Maximum Pitch Angle command accepted into PID controllers |  0.786 | 0.0 | 1000.0 |
| PID_ALT_P | Altitude Proporitional Gain |  0.0f | 0.0 | 1000.0 |
| PID_ALT_I | Altitude Integral Gain |  0.0f | 0.0 | 1000.0 |
| PID_ALT_D | Altitude Derivative Gain |  0.0f | 0.0 | 1000.0 |
| HOVER_THR | Hover Throttle - See RC calibration |  0.5 | 0.0 | 1.0 |
| PID_TAU | Dirty Derivative time constant - See controller documentation |  0.05f | 0.0 | 1.0 |
| MOTOR_PWM_UPDATE | Refresh rate of motor commands to motors - See motor documentation |  490 | 0 | 1000 |
| MOTOR_IDLE_THR | Idle PWM sent to motors at zero throttle (Set above 1100 to spin when armed) |  0.1 | 1000 | 2000 |
| MOTOR_MIN_PWM | Idle PWM sent to motors at zero throttle (Set above 1100 to spin when armed) |  1000 | 1000 | 2000 |
| MOTOR_MAX_PWM | Idle PWM sent to motors at zero throttle (Set above 1100 to spin when armed) |  2000 | 1000 | 2000 |
| ARM_SPIN_MOTORS | Enforce MOTOR_IDLE_PWM |  true | 0 | 1 |
| FILTER_INIT_T | Time in ms to initialize estimator |  3000 | 0 | 100000 |
| FILTER_KP | estimator proportional gain - See estimator documentation |  1.0f | 0 | 10.0 |
| FILTER_KI | estimator integral gain - See estimator documentation |  0.1f | 0 | 1.0 |
| GYRO_LPF_ALPHA | Low-pass filter constant - See estimator documentation |  0.888f | 0 | 1.0 |
| ACC_LPF_ALPHA | Low-pass filter constant - See estimator documentation |  0.888f | 0 | 1.0 |
| ACCEL_SCALE | Scale factor to apply to IMU measurements - Read-Only |  1.0f | 0.5 | 2.0 |
| GYRO_X_BIAS | Constant x-bias of gyroscope readings |  0.0f | -1.0 | 1.0 |
| GYRO_Y_BIAS | Constant y-bias of gyroscope readings |  0.0f | -1.0 | 1.0 |
| GYRO_Z_BIAS | Constant z-bias of gyroscope readings |  0.0f | -1.0 | 1.0 |
| ACC_X_BIAS | Constant x-bias of accelerometer readings |  0.0f | -2.0 | 2.0 |
| ACC_Y_BIAS | Constant y-bias of accelerometer readings |  0.0f | -2.0 | 2.0 |
| ACC_Z_BIAS | Constant z-bias of accelerometer readings |  0.0f | -2.0 | 2.0 |
| ACC_X_TEMP_COMP | Linear x-axis temperature compensation constant |  0.0f | -2.0 | 2.0 |
| ACC_Y_TEMP_COMP | Linear y-axis temperature compensation constant |  0.0f | -2.0 | 2.0 |
| ACC_Z_TEMP_COMP | Linear z-axis temperature compensation constant |  0.0f | -2.0 | 2.0 |
| MAG_A11_COMP | Soft iron compensation constant |  1.0f | -999.0 | 999.0 |
| MAG_A12_COMP | Soft iron compensation constant |  0.0f | -999.0 | 999.0 |
| MAG_A13_COMP | Soft iron compensation constant |  0.0f | -999.0 | 999.0 |
| MAG_A21_COMP | Soft iron compensation constant |  0.0f | -999.0 | 999.0 |
| MAG_A22_COMP | Soft iron compensation constant |  1.0f | -999.0 | 999.0 |
| MAG_A23_COMP | Soft iron compensation constant |  0.0f | -999.0 | 999.0 |
| MAG_A31_COMP | Soft iron compensation constant |  0.0f | -999.0 | 999.0 |
| MAG_A32_COMP | Soft iron compensation constant |  0.0f | -999.0 | 999.0 |
| MAG_A33_COMP | Soft iron compensation constant |  1.0f | -999.0 | 999.0 |
| MAG_X_BIAS | Hard iron compensation constant |  0.0f | -999.0 | 999.0 |
| MAG_Y_BIAS | Hard iron compensation constant |  0.0f | -999.0 | 999.0 |
| MAG_Z_BIAS | Hard iron compensation constant |  0.0f | -999.0 | 999.0 |
| RC_TYPE | Type of RC input 0 - Parallel PWM (PWM), 1 - Pulse-Position Modulation (PPM) |  1 | 0 | 1 |
| RC_TYPE | Type of RC input 0 - Parallel PWM (PWM), 1 - Pulse-Position Modulation (PPM) | 1 | 0 | 1 |
| RC_X_CHN | RC input channel mapped to x-axis commands [0 - indexed] |  0 | 0 | 3 |
| RC_Y_CHN | RC input channel mapped to y-axis commands [0 - indexed] |  1 | 0 | 3 |
| RC_Z_CHN | RC input channel mapped to z-axis commands [0 - indexed] |  3 | 0 | 3 |
| RC_F_CHN | RC input channel mapped to F-axis commands [0 - indexed] |  2 | 0 | 3 |
| RC_ATT_OVRD_CHN | RC switch mapped to attitude override [0 -indexed] |  4 | 4 | 7 |
| RC_THR_OVRD_CHN | RC switch hannel mapped to throttle override [0 -indexed] |  4 | 4 | 7 |
| RC_ATT_CTRL_CHN | RC switch channel mapped to attitude control type [0 -indexed] |  5 | 4 | 7 |
| RC_F_CTRL_CHN | RC switch channel mapped to throttle control type override [0 -indexed] |    7 | 4 | 7 |
| RC_NUM_CHN | number of RC input channels |  6 | 1 | 8 |
| RC_X_CENTER | RC calibration x-axis center (us) |  1500 | 1000 | 2000 |
| RC_Y_CENTER | RC calibration y-axis center (us) |  1500 | 1000 | 2000 |
| RC_Z_CENTER | RC calibration z-axis center (us) |  1500 | 1000 | 2000 |
| RC_F_BOTTOM | RC calibration F-axis center (us) |  1000 | 1000 | 2000 |
| RC_X_RANGE | RC calibration x-axis range (us) |  1000 | 500 | 2500 |
| RC_Y_RANGE | RC calibration y-axis range (us) |  1000 | 500 | 2500 |
| RC_Z_RANGE | RC calibration z-axis range (us) |  1000 | 500 | 2500 |
| RC_F_RANGE | RC calibration F-axis range (us) |  1000 | 500 | 2500 |
| SWITCH_5_DIR | RC switch 5 toggle direction |  1 | 0 | 1 |
| SWITCH_6_DIR | RC switch 6 toggle direction |  1 | 0 | 1 |
| SWITCH_7_DIR | RC switch 7 toggle direction |  1 | 0 | 1 |
| SWITCH_8_DIR | RC switch 8 toggle direction |  1 | 0 | 1 |
| RC_OVRD_DEV | RC stick deviation from center for overrride (us) |  100 | 0 | 1000 |
| OVRD_LAG_TIME | RC stick deviation lag time before returning control (ms) |  1000 | 0 | 100000 |
| MIN_THROTTLE | Take minimum throttle between RC and computer at all times |  false | 0 | 1 |
| RC_MAX_ROLL | Maximum roll angle command sent by full deflection of RC sticks |  0.786f | 0.0 | 3.14159 |
| RC_MAX_PITCH | Maximum pitch angle command sent by full stick deflection of RC sticks |  0.786f | 0.0 | 3.14159 |
| RC_MAX_ROLLRATE | Maximum roll rate command sent by full stick deflection of RC sticks |  3.14159f | 0.0 | 9.42477796077 |
| RC_MAX_PITCHRATE | Maximum pitch command sent by full stick deflection of RC sticks |  3.14159f | 0.0 | 3.14159 |
| RC_MAX_YAWRATE | Maximum pitch command sent by full stick deflection of RC sticks |  0.786f | 0.0 | 3.14159 |
| MIXER | Which mixer to choose - See Mixer documentation |  QUADCOPTER_X | 0 | 5 |
| FIXED_WING | switches on passthrough commands for fixedwing operation |  false | 0 | 1 |
| ELEVATOR_REV | reverses elevator servo output |  0 | 0 | 1 |
| AIL_REV | reverses aileron servo output |  0 | 0 | 1 |
| RUDDER_REV | reverses rudder servo output |  0 | 0 | 1 |
| ARM_STICKS | use RC sticks to arm vehicle (disables arm RC switch if enabled) |  true | 0 | 1 |
| ARM_CHANNEL | RC switch mapped to arm/disarm [0 -indexed] |  5 | 4 | 7 |
| ARM_THRESHOLD | RC deviation from max/min in yaw and throttle for arming and disarming check (us) |  150 | 0 | 500 |
