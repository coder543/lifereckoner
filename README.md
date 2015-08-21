# lifereckoner
This is designed to be a system that will keep track of your location via dead reckoning alone. Sensor fusion with other devices such as a GPS module might be an interesting extension, but is outside the scope of this project.

Dead reckoning is a well established technique for estimating position based off of data from accelerometers, magnetometers, and gyroscopes. As far as I've been able to tell, this technique has never been applied to MEMS devices in a well-documented manner. The theory behind applying dead reckoning here is sound, but whether MEMS devices will hold high enough precision for this to work is to be seen. For a First Responder, the location data only needs to remain relatively accurate (say, less than a meter of inaccuracy) over a relatively short time period (say 20 minutes). I believe that this should be feasible with data from a compact MEMS device such as the LSM9DS0.

This code is part of the work I'm doing for Google Summer of Code 2015.

`ao_quaternion.h` was written by Keith Packard as part of the AltOS project. It is under the GPLv2, as my project now is, and its copyright notice is intact at the top of the file.

### status
The summer is coming to a close, so I'm prepping this project for submission. I plan to continue working on it after the summer as time permits, but this is where things stand for now.

- [x] Find a suitable platform for this project, namely the STM32 Nucleo + mbed.
- [x] Write an LSM9DS0 driver, so we can read real-time sensor data for instantaneous rate of rotatation, acceleration, and the magnetic field strength currently being sensed, all of which are three dimensional data points for a total of 9 axes.
- [x] Learn about quaternions, as they are the most effective way to represent angular state without gimbal lock, wrist flip, or other problems introduced by the more common methods.
- [x] Find or implement a suitable quaternion library, so as to be able to use them in code. I chose the path of finding an existing one, since implementing my own did not seem like it would benefit anyone else and it would just occupy more time.
- [x] Utilize quaternions to maintain angular state as data points are recorded. There seem to be some issues here, but comparing notes with what I've done against venerable references on the web indicates that my code is correct, and I haven't figured out what is causing my angular state to not be reflecting reality very well. The raw angular data that's coming in is certainly pretty reasonable. I have one theory that I'm testing soon, but nothing is for sure yet there.
- [x] Figure out how to rotate my three axis acceleration input into the frame of reference (FoR) (aka. initial frame, in this case) so that it could be integrated into velocity and position using the current quaternion state.
- [x] Integrate the FoR acceleration, which means keeping up with a value for velocity between each data point, and of course, the global positional state as well.
- [ ] [In Progress] Implement filtering to reduce noise from the sensors and improve the quality of the output data. I have a Kalman filter halfway implemented, along with some support infrastructure in the `kalman` branch of this code.
- [ ] [Wishlist] Combine `lifereckoner` with a Nordic Semi NRF2401 Shockburst transceiver setup to wirelessly transmit real-time positional data over significant distances -- inexpensive modules from eBay are rated at ranges of over 100 meters, with at least 30 meters of real world range based on [some experiments one person did.](https://hallard.me/nrf24l01-real-life-range-test/)

### conclusion

Overall, I would call this an incredibly educational and productive summer.

**The goal of this project was dual in nature: to create an open source purely dead reckoning system and determine the feasibility of a low-cost dead reckoning system for first responders by use of that open source system.** Perhaps this was too ambitious of a goal for a single summer, but I believe I have made phenomenal progress towards that goal. **This repository contains the software**, and anyone who has access to an STM32-F334R8 and an LSM9DS0 can quickly be up and running, ready to determine the feasibility for themselves or to modfiy and extend my code to test out new techniques, for research or personal gain, whatever the case may be. **For now, my data shows that it is *not* feasible**, but as noted above, my angular state information results are questionable, beyond what would be expected from the noise in the angular rate sensors. If I solve that issue, my determination may very well change altogether, and I'm still working to solve it.
