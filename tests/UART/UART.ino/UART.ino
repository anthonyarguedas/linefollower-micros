void setup() {
  Serial.begin(9600);  // Serial para el monitor serie
  Serial2.begin(9600); // UART para comunicación con la Raspberry Pi
  Serial.println("Teensy lista para comunicación UART...");
}

int colorCounter = 0; // Contador de colores

void loop() {
  // Leer datos desde la Raspberry Pi (10 bits enviados en 2 bytes)
  if (Serial2.available() >= 2) {
    byte data[2];
    data[0] = Serial2.read(); // Leer primer byte
    data[1] = Serial2.read(); // Leer segundo byte

    // Reconstruir los 10 bits desde los 2 bytes
    unsigned int receivedData = (data[0] << 8) | data[1];

    // Mostrar los bits recibidos
    Serial.print("Bits recibidos: ");
    for (int i = 9; i >= 0; i--) {
      Serial.print((receivedData >> i) & 0x01);
    }
    Serial.println();

    // Incrementar el contador de colores
    colorCounter++;
    if (colorCounter > 255) colorCounter = 0; // Reiniciar si supera 255

    // Enviar el contador de colores a la Raspberry Pi
    Serial2.write(colorCounter);
    Serial.print("Contador enviado: ");
    Serial.println(colorCounter);
  }
}
