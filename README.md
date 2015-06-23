# lifereckoner
This is designed to be a system that will keep track of your location via dead reckoning alone. Sensor fusion with other devices such as a GPS module might be an interesting extension, but is outside the scope of this project.

Dead reckoning is a well established technique for estimating position based off of data from accelerometers, magnetometers, and gyroscopes. As far as I've been able to tell, this technique has never been applied to MEMS devices in a well-documented manner. The theory behind applying dead reckoning here is sound, but whether MEMS devices will hold high enough precision for this to work is to be seen. For a First Responder, the location data only needs to remain relatively accurate (say, less than a meter of inaccuracy) over a relatively short time period (say 20 minutes). I believe that this should be feasible with data from a compact MEMS device such as the LSM9DS0.

This code is part of the work I'm doing for Google Summer of Code 2015.
