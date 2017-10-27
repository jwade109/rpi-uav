# rpi-uav
_a bootleg uav framework for raspberry pi_

# modules

## <uav/math>
* vector
* quaternion
* matrix
* freebody
* angle
* angular
* coordinate
* related functions (angle2quat, quat2matrix, etc)

## <uav/logging>
* logstream
* serial translation functions

## <uav/hardware>
* arduino
* bmp085
* gps
* pwm
* i2c
* sensor bus

## <uav/filters>
* low pass filter
* high pass filter
* running average
* moving average
* running variance
* moving variance
* derivative
* integral
* range accumulator
* optional

## <uav/algorithm>
* gps + barometer altitude filter
* gps position filter
* motor director

## <uav/control>
* flight controller
* vehicle state
* parameters
* raw data
