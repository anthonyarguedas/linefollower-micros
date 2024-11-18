import serial
import time

# Configuración del puerto serie
ser = serial.Serial('/dev/serial0', 9600, timeout=1)

print("Raspberry Pi lista para comunicación UART...")

def send_data(activation, speed, action_red, action_green, action_blue, direction):
    # Construir el mensaje de 10 bits
    data = 0
    data |= (activation & 0x01) << 9  # Bit 10: activación
    data |= (speed & 0x03) << 7       # Bits 9-8: velocidad
    data |= (action_red & 0x03) << 5  # Bits 7-6: acción rojo
    data |= (action_green & 0x03) << 3 # Bits 5-4: acción verde
    data |= (action_blue & 0x03) << 1 # Bits 3-2: acción azul
    data |= (direction & 0x01)        # Bit 1: dirección intersección

    # Dividir los 10 bits en 2 bytes
    byte1 = (data >> 8) & 0xFF  # Byte alto
    byte2 = data & 0xFF         # Byte bajo

    # Enviar los 2 bytes a la Teensy
    ser.write(bytearray([byte1, byte2]))
    return data

def receive_counter():
    if ser.in_waiting > 0:  # Si hay datos disponibles
        counter = ser.read(1)  # Leer un byte
        return int.from_bytes(counter, byteorder='little')
    return None

try:
    while True:
        # Enviar datos a la Teensy
        sentData = send_data(
            activation=1,        # Activar
            speed=2,             # Velocidad 2
            action_red=1,        # Cambio de velocidad para rojo
            action_green=0,      # Frenado para verde
            action_blue=2,       # Retroceder para azul
            direction=1          # Izquierda en intersecciones
        )
        print(f"Bits enviados: {bin(sentData)[2:].zfill(10)}")

        # Leer el contador de colores desde la Teensy
        counter = receive_counter()
        if counter is not None:
            print(f"Contador recibido: {counter}")

        time.sleep(1)  # Pausa de 1 segundo
except KeyboardInterrupt:
    print("\nFinalizando...")
    ser.close()
