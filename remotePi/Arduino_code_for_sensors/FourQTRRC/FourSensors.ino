#include <QTRSensors.h>
#define HARDCODE_CALIBRATION true
#define NUM_SENSORS   8     // number of sensors used
#define NUM_BOARDS 4        // number of boards
#define TIMEOUT       2500  // waits for 2500 microseconds for sensor outputs to go low
#define EMITTER_PIN   2     // emitter is controlled by digital pin 2

// sensors 0 through 7 are connected to digital pins 3 through 10, respectively
QTRSensorsRC qtrrcFront((unsigned char[]) {3, 4, 5, 6, 7, 8, 9, 10},
  NUM_SENSORS, TIMEOUT, EMITTER_PIN);
QTRSensorsRC qtrrcRight((unsigned char[]) {0, 0, 0, 0, 0, 0, 0, 0},
  NUM_SENSORS, TIMEOUT, EMITTER_PIN);
QTRSensorsRC qtrrcBack((unsigned char[]) {0, 0, 0, 0, 0, 0, 0, 0},
  NUM_SENSORS, TIMEOUT, EMITTER_PIN);
QTRSensorsRC qtrrcLeft((unsigned char[]) {0, 0, 0, 0, 0, 0, 0, 0},
  NUM_SENSORS, TIMEOUT, EMITTER_PIN);
QTRSensorsRC qtrrcs[4];
unsigned int sensorValues[NUM_SENSORS];

void setup()
{
  delay(500);
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);    // turn on Arduino's LED to indicate we are in calibration mode
  Serial.begin(9600);

  qtrrcs[0] = qtrrcFront;
  qtrrcs[1] = qtrrcRight;
  qtrrcs[2] = qtrrcBack;
  qtrrcs[3] = qtrrcLeft;

  if (!HARDCODE_CALIBRATION) {
    //run a few hundred times
    for (int i = 0 ; i < NUM_BOARDS ; i++) {
      Serial.print("Start calibrate ");
      Serial.println(i);
      for (int j = 0 ; j < 250 ; j ++)
        qtrrcs[i].calibrate();
      Serial.print("End calibrate ");
      Serial.println(i);
    }

    //print the calibration minimum values measured when emitters were on
    for (int i = 0 ; i < NUM_BOARDS ; i++) {
      Serial.print(i);
      Serial.print(": ");
      for (int j = 0; j < NUM_SENSORS; j++)
      {
        Serial.print(qtrrcs[i].calibratedMinimumOn[j]);
        Serial.print(' ');
      }
      Serial.println();
    }
    
    //print the calibration maximum values measured when emitters were on
    for (int i = 0; i < NUM_BOARDS; i++) {
      Serial.print(i);
      Serial.print(": ");
      for (int j = 0 ; j < NUM_SENSORS ; j++)
      {
        Serial.print(qtrrcs[i].calibratedMaximumOn[j]);
        Serial.print(' ');
      }
      Serial.println();
    }
  } else {
    for (int i = 0 ; i < NUM_BOARDS ; i++) {
      qtrrcs[i].calibrate(); // to initialise calibration arrays
      // INSERT HARDCODED VALUES NOW
    }
  }
  digitalWrite(13, LOW);     // turn off Arduino's LED to indicate we are through with calibration
}


void loop()
{
  for (int i = 0 ; i < NUM_BOARDS ; i++) {
    unsigned int position = qtrrcs[i].readLine(sensorValues);
    Serial.print(position); // comment this line out if you are using raw values
  }
  Serial.println();

  delayMicroseconds(7500);
}
