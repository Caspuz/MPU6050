# MPU6050
Arduino project for the MPU6050

Goal was to make a very fast logger of the acceleration (which will indirect show the forces acting
on a piece of robotics when dropped from certain heights).

Should give about 1100Hz on an Arduino Uno, but the data saved requires some processing before it
is readable.

Used libraries:
Wire
SPI
MPU6050
SdFat
I2Cdev

Calibrations of the gyros and accelerometers are crude, and the calibration of the accelerometers
are only specific for the individual MPU used during the experiment.

Quick solution on the risk of the SD-card loosing connection on the impact, was to create, and
close several files in succession during the drop, that increased the possibility of some data
being usable.
