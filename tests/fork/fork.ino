#define FORK 11

volatile bool isfork = false;

void FORKISR() {
    isfork = (GPIO2_DR & (1 << 2)) ? true : false;
}

void setup() {
  Serial.begin(9600);
  attachInterrupt(digitalPinToInterrupt(FORK), FORKISR, CHANGE);
}

void loop() {
  Serial.println(isfork);
  delay(500);
}
