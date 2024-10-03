#define BRAKE 2
#define REVERSE 3
#define CHANGE_SPEED 4

uint8_t green_action = BRAKE;
uint8_t blue_action = REVERSE;
uint8_t red_action = CHANGE_SPEED;

uint8_t green_encounters = 0;
uint8_t blue_encounters = 0;
uint8_t red_encounters = 0;

bool running = false;

// Buffer para guardar los bytes entrantes por UART
uint8_t rxBuffer[3];

void setup() {
    // UART para debugging
    Serial.begin(115200);

    // UART para comunicarse con la Raspbery Pi
    // RX = pin 0, TX = pin 1
    Serial1.begin(115200);
}

void loop() {
    if (Serial1.available() >= 3) {
        for (int i=0;i<3;i++) {
            rxBuffer[i] = Serial1.read();
        }

        if (rxBuffer[0] == rxBuffer[1] == rxBuffer[2] == 0) {
            running = true;
            Serial.println("Inicio");
        } else if (rxBuffer[0] == rxBuffer[1] == rxBuffer[2] == 1) {
            running = false;
            Serial.println("Fin");
        } else {
            green_action = rxBuffer[0];
            blue_action = rxBuffer[1];
            red_action = rxBuffer[2];
            Serial.println("Acciones asignadas (verde, azul, rojo):");
            
            for (int i=0;i<3;i++) {
                switch (rxBuffer[i]) {
                    case BRAKE:
                        Serial.print("Frenar, ");
                        break;
                    case REVERSE:
                        Serial.print("Retroceder, ");
                        break;
                    case CHANGE_SPEED:
                        Serial.print("Cambiar velocidad, ");
                }
            }

            Serial.println();
        }
    }

    if (running) {
        Serial1.print(green_encounters);
        Serial1.print(",");
        Serial1.print(blue_encounters);
        Serial1.print(",");
        Serial1.println(red_encounters);

        green_encounters++;
        blue_encounters++;
        red_encounters++;
    }

    delay(1500);
}