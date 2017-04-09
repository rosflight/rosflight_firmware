# Multirotor gain tuning

Becuase there are a wide variety of multirotors out there, no one set of PID controller gains will work for all vehicles.  The default set of gains is relatively conservative for most platforms, and should be somewhat airworthy in most cases, however, depending on the inertia to torque ratio for your MAV, you may have to change these gains considerably.  There are some great tutorials online on multirotor gain tuning, however since this is the ROSflight documentation, I'll put my tuning method here for you.

If you are unfamiliar with PIDs, then you should probably go read about them before trying to tune a multirotor.  Getting an understanding for what is going on will definitely guide your decision making process as you try to figure out better gains.

Some people like to build test stands to tune your multirotor safely.  This is probably a good idea.  However, I've never used one of these stands and I've tuned multirotors that weigh several times as much as I do without one.  Just be careful.




# RC trim calculation
