# This message represents the tactile information from a BioTac sensor with a header
# providing timestamp information as well as the frame of reference of the sensor.
# The BioTac sensor produces the following signals [1]:
#
# DC Pressure (Pdc): or static pressure can be read from the sensor and increases linearly
# when the fluid pressure increases.
#
# AC Pressure (Pac): or dynamic pressure is generated from a high-pass filtered version
# of DC Pressure with additional gain allowing for high resolution of vibrations.
#
# DC Temperature (Tdc): can be read from the sensor and decreases as the device warms up.
#
# AC Temperature (Tac): is a high-pass filtered version of DC Temperature that permits
# for higher resolution of thermal fluxes. This decreases as the device is cooled.
#
# Electrode Voltages (electrodes): 19 electrodes are measured across a voltage divider
# with reference to  a load resistor. When pressing down over an electrode the
# measured voltage will decrease.
#
# [1] http://wiki.ros.org/BioTac

std_msgs/Header header
int16 pac0
int16 pac1
int16 pdc
int16 tac
int16 tdc
int16[] electrodes
