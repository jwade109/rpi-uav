.PHONY: I2C

current_dir = $(shell pwd)

bmp: BMP280.h BMP280.cpp Adafruit_Sensor.h test.cpp
	g++ BMP280.cpp test.cpp -o bmp -lwiringPi -lm -I$(current_dir) -O2
