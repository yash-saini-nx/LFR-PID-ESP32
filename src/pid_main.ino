#include <Wire.h>
#include <Adafruit_VL53L1X.h>

Adafruit_VL53L1X vl53 = Adafruit_VL53L1X();

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!vl53.begin(0x29, &Wire)) {
    Serial.println("Failed to start VL53L1X");
    while (1);
  }

  vl53.startRanging();
}

void loop() {
  if (vl53.dataReady()) {
    int distance = vl53.distance();
    vl53.clearInterrupt();

    Serial.print("Distance (mm): ");
    Serial.println(distance);
  }

  delay(20);
}