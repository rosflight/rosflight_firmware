# Welcome to ROSflight

ROSflight is an autopilot system that is designed from the ground up for integration with [ROS](http://www.ros.org/). It's purpose is to accomplish two main tasks:

  1. Stream high-rate sensor data from the embedded flight controller to ROS
  2. Provide a simple API for sending control setpoints to the embedded flight controller

A ROSflight setup consists of two main components:

  1. **The embedded flight controller:** Typically a Naze32 or one of its variants, running the ROSflight [firmware](https://github.com/rosflight/firmware)
  2. **The onboard computer:** Any computer that runs ROS, located on the vehicle with a physical serial connection to the embedded flight controller

## Why ROSflight?

There are a lot of excellent autopilots out there, with a lot of great firmware options. Why did we feel like the world needed ROSflight? Because in our experience none of the other available options satisfied our research needs. Specifically, we needed an autopilot that could stream sensor data at high rates, easily accept control setpoints from an onboard computer, and accomplish all of this with a lean, easy to understand code base.

The other options that we tried were limited in bandwidth for streaming sensor data, and the APIs for sending control setpoints were confusing and difficult to implement. Perhaps most importantly, the code was sometimes so complex (feature-rich, but complicated) that it was difficult to figure out what the autopilot was actually doing. In talking to other researchers and industry members, we found that many people shared similar frustrations. So we decided to create the autopilot that we wanted and share it in the hopes that it will be useful to other people as well.

## Our Vision

Perhaps more important than what we're trying to accomplish with ROSflight is what we're _not_ trying to accomplish. This is not intended to be, out-of-the-box, a fully-featured autopilot for high-level tasks such as autonomous GPS waypoint following. There are many good autopilots out there that already do this. Instead, ROSflight is intended to provide the minimal functionality required to keep a multirotor or fixed-wing vehicle in the air, and to serve as a building block for writing new code to perform these higher-level tasks (the [ROSplane](https://github.com/byu-magicc/ros_plane) and [ROScopter](https://github.com/byu-magicc/ros_copter) projects are excellent examples of what can be accomplished by building on top of the ROSflight architecture).

Therefore, one of the primary objectives of the ROSflight autopilot is to avoid feature creep and remain _lean_. We hope that others will extend our code and build on top of it, and would love to hear about your successes. But for the most part, we will not be incorporating these new features back into the main project. Instead, we hope that ROSflight will remain a lean, core code base that will continue to serve as a launch pad for exciting new projects and applications.
