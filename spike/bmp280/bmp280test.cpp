#include <Wire.h>
#include <stdio.h>
#include "Adafruit_Sensor.h"
#include "Adafruit_BMP280.h"

Adafruit_BMP280 bmp;

int main() {
  
  if (!bmp.begin()) {  
    printf("Couldn't find BMP280\n");
    return 1;
  }

  printf("FOUNDED IT\n");

}
