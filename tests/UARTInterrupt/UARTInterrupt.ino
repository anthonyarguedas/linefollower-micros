// Serial2
#define TX 8
#define RX 7

bool rxAvailable = false;

void UARTRXISR() {
    if (Serial2.available() >= 2) {rxAvailable = true;}
}

void setup() {
    Serial.begin(115200);
    Serial2.begin(115200);
    attachInterrupt(digitalPinToInterrupt(RX), UARTRXISR, RISING);
}

void loop() {
    if (rxAvailable) {
        unsigned short byte1 = Serial2.read();
        Serial.println(byte);
        unsigned short byte2 = Serial2.read();
        Serial.println(byte2);
        Serial.println();

        rxAvailable = false;
    }
}