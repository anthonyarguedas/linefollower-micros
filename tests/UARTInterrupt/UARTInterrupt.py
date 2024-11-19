import serial
import time

# TX: 14, RX: 15
ser = serial.Serial('/dev/serial0', baudrate=115200, timeout=1)

if ser.is_open:
    print("Serial port is open")

while True:
    try:
        ser.write(bytearray([0x8, 0x9]))
        time.sleep(1)
    except KeyboardInterrupt:
        ser.close()
