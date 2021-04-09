/*
    YDLIDAR SYSTEM
    YDLIDAR Arduino

    Copyright 2015 - 2018 EAI TEAM
    http://www.eaibot.com

*/

#include <YDLidar.h>

// You need to create an driver instance
YDLidar lidar;

bool isScanning = false;



void setup() {

  lidar.begin(Serial, YDLidar::TX8);
}

void loop() {
  if (lidar.waitScanDot() == RESULT_OK)
  {

  }
  else
  {
    Serial.println(" YDLIDAR get Scandata failed!!");
  }
}
