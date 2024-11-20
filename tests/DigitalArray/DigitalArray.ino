#define SENSOR 2

#define sensorCountBW 3
unsigned short sensorPinsBW[sensorCountBW] = {15, 2, 4};
unsigned int sensorValuesBW[sensorCountBW];

unsigned int* RCRead() {
    for (int i=0; i<sensorCountBW; i++)
      {
        sensorValuesBW[i] = 1023;
        // make sensor line an output (drives low briefly, but doesn't matter)
        pinMode(sensorPinsBW[i], OUTPUT);
        // drive sensor line high
        digitalWrite(sensorPinsBW[i], HIGH);
      }

      unsigned long startDelayTime = micros();
      // delay(10us)
      while (micros() - startDelayTime < 10) {}

      {
        // record start time before the first sensor is switched to input
        // (similarly, time is checked before the first sensor is read in the
        // loop below)
        unsigned long startTime = micros();
        unsigned int time = 0;

        for (int i=0; i<sensorCountBW; i++)
        {
          // make sensor line an input (should also ensure pull-up is disabled)
          pinMode(sensorPinsBW[i], INPUT);
        }

        while (time <= 1023)
        {
          time = micros() - startTime;
          for (int i=0; i<sensorCountBW; i++)
          {
            if ((digitalRead(sensorPinsBW[i]) == LOW) && (time < sensorValuesBW[i]))
            {
              // record the first time the line reads low
              sensorValuesBW[i] = time;
            }
          }
        }
      }

      return sensorValuesBW;
}

void setup() {
    Serial.begin(115200);
}

void loop() {
    RCRead();
    for (int i=0; i<sensorCountBW; i++) {
      Serial.print(sensorValuesBW[i]);
      Serial.print(" ");
    }
    Serial.println();

    delay(1000);
}