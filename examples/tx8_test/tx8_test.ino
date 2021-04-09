#include <YDLidar.h>

YDLidar lidar;

#define RX_PIN 19
#define TX_PIN 18

void setup() {
  Serial1.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  Serial.println("Starting TX8 Demo");
  lidar.begin(Serial1, YDLidar::TX8);
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
