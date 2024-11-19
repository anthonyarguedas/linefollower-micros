#include <Arduino.h>

#define SIGNAL 9

// Declaración de contadores
byte contador_rojo = 0;
byte contador_verde = 0;
byte contador_azul = 0;
unsigned long previousMillis = 0; // Tiempo anterior
const unsigned long interval = 1000; // Intervalo de 1 segundo

bool rxAvailable() = false;


void UARTRXISR() {
    rxAvailable = true;
}


void setup() {
  Serial.begin(9600);   // Monitor serial
  Serial2.begin(115200);  // UART en Serial2

  pinMode(SIGNAL, INPUT);
  attachInterrupt(digitalPinToInterrupt(SIGNAL), UARTRXISR, RISING);
}

void loop() {
  //if (Serial2.available() >= 7) {
  if (rxAvailable == true) {
    rxAvailable = false;
    // Leer los 7 bytes recibidos
    byte byte1 = Serial2.read();
    uint16_t kp_raw = (Serial2.read() << 8) | Serial2.read(); // Bytes 2 y 3
    uint16_t ki_raw = (Serial2.read() << 8) | Serial2.read(); // Bytes 4 y 5
    uint16_t kd_raw = (Serial2.read() << 8) | Serial2.read(); // Bytes 6 y 7

    // Convertir a valores decimales (dividir por 100)
    float kp = kp_raw / 100.0;
    float ki = ki_raw / 100.0;
    float kd = kd_raw / 100.0;

    // Extraer las secciones del byte1
    bool carro_activado = (byte1 & 0b10000000) >> 7;  // Bit 1
    bool direccion = (byte1 & 0b01000000) >> 6;       // Bit 2
    byte accion_rojo = (byte1 & 0b00110000) >> 4;     // Bits 3 y 4
    byte accion_verde = (byte1 & 0b00001100) >> 2;    // Bits 5 y 6
    byte accion_azul = (byte1 & 0b00000011);          // Bits 7 y 8

    // Imprimir datos en el monitor serial
    Serial.println("Datos recibidos:");
    Serial.print("Carro activado: ");
    Serial.println(carro_activado ? "Sí" : "No");

    Serial.print("Dirección: ");
    Serial.println(direccion ? "Derecha" : "Izquierda");

    Serial.print("Acción rojo: ");
    switch (accion_rojo) {
      case 0: Serial.println("Frenar"); break;
      case 1: Serial.println("Cambiar"); break;
      case 2: Serial.println("Retroceder"); break;
      default: Serial.println("Desconocido"); break;
    }

    Serial.print("Acción verde: ");
    switch (accion_verde) {
      case 0: Serial.println("Frenar"); break;
      case 1: Serial.println("Cambiar"); break;
      case 2: Serial.println("Retroceder"); break;
      default: Serial.println("Desconocido"); break;
    }

    Serial.print("Acción azul: ");
    switch (accion_azul) {
      case 0: Serial.println("Frenar"); break;
      case 1: Serial.println("Cambiar"); break;
      case 2: Serial.println("Retroceder"); break;
      default: Serial.println("Desconocido"); break;
    }

    Serial.print("Kp: ");
    Serial.println(kp, 2); // Mostrar 2 decimales

    Serial.print("Ki: ");
    Serial.println(ki, 2); // Mostrar 2 decimales

    Serial.print("Kd: ");
    Serial.println(kd, 2); // Mostrar 2 decimales

    Serial.println("-------------------");
  }

  // Enviar contadores cada segundo
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;

    // Incrementar los contadores
    contador_rojo++;
    contador_verde++;
    contador_azul++;

    // Enviar los contadores
    Serial2.write(contador_rojo);
    Serial2.write(contador_verde);
    Serial2.write(contador_azul);
  }
}