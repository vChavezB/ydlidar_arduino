#include <YDLidar.h>

YDLidar lidar;


#define RX_PIN 19
#define TX_PIN 18

#define LIDAR_ENABLE_PIN 12
#define MAX_SCAN_POINTS 360 //360° Range
uint16_t scan_points[MAX_SCAN_POINTS];
uint16_t origin_angle;
bool first_scan = true;
//True if lidar has done at least one 360° sweep
bool sweep_complete = false;
bool last_scan_has_sweep = false;

void reset_lidar()
{
  digitalWrite(LIDAR_ENABLE_PIN, LOW);
  delay(500);
  digitalWrite(LIDAR_ENABLE_PIN, HIGH);
}

void setup() {
  Serial1.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  Serial.begin(500000);
  Serial.println("Starting TX8 Demo");
  ydlidar_result_et result = lidar.begin(Serial1, YDLidar::TX8);
  pinMode(LIDAR_ENABLE_PIN, OUTPUT);
  pinMode(13, OUTPUT);
  pinMode(13, LOW);
  reset_lidar();
}

void copy_scan()
{
  for (uint16_t sample = 0; sample < lidar.scan_data.samples; sample++)
  {
    uint16_t angle_i = (uint16_t)lidar.scan_data.points[sample].angle;
    if (angle_i < MAX_SCAN_POINTS)
    {
      //filter out sample points that measure 0 mm
      if (lidar.scan_data.points[sample].distance != 0)
      {
        scan_points[angle_i] = lidar.scan_data.points[sample].distance;
      }
    }
  }
}

bool is_sweep_completed()
{
  bool sweep_completed = false;
  //Sometimes lidar scans only one point, which makes next scan
  //report that sweep is completed
  if (!last_scan_has_sweep)
  {
    for (uint16_t sample = 0; sample < lidar.scan_data.samples; sample++)
    {
      uint16_t angle_i = (uint16_t)lidar.scan_data.points[sample].angle;
      //angle between origin_angle and origin_angle+1
      if ( angle_i >= origin_angle && angle_i <= (origin_angle + 1))
      {
        sweep_completed = true;
        last_scan_has_sweep = true;
        break;
      }
    }
  }
  else
  {
    last_scan_has_sweep = false;
  }
  return sweep_completed;
}
void loop() {
  ydlidar_result_et scan_result = lidar.waitScanDot();
  if (scan_result == RESULT_OK && lidar.scan_data.samples > 0)
  {
    if (first_scan)
    {
      first_scan = false;
      origin_angle = (uint16_t)lidar.scan_data.start_angle;
    }
    else
    {

      sweep_complete = is_sweep_completed();
    }
    copy_scan();
  }
  else
  {
    Serial.print(" YDLIDAR get Scandata failed: ");
    Serial.print("Error Code:");
    Serial.println(scan_result, HEX);
  }

  if (sweep_complete)
  {
    sweep_complete = false;
    Serial.print("S");
    for (int i = 0; i < MAX_SCAN_POINTS; i++)
    {
      Serial.print(scan_points[i]);
      if (i != MAX_SCAN_POINTS - 1)
      {
        Serial.print(",");
      }

    }
    Serial.print("\n");
  }
}
