#include <HCSR04.h>

UltraSonicDistanceSensor distanceSensor(13, 12);  // Initialize sensor that uses digital pins 13 and 12.
int count;
void setup () {
    count = 0;
    Serial.begin(9600);
}

void loop () {
    if (count < 50) {
      // Print reading to serial bus as fast as possible. Measured in cm.
      Serial.println(distanceSensor.measureDistanceCm());
      delay(100);
      count++;
    }
}
