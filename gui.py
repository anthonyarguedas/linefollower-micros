import serial
import struct
import tkinter as tk
from tkinter import ttk
import RPi.GPIO as GPIO

# Configurar el puerto UART
uart = serial.Serial(
    port='/dev/ttyAMA0',  # Asegúrate de usar el puerto correcto
    baudrate=115200,       # Velocidad de transmisión
    timeout=1            # Tiempo de espera
)

GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.OUT)
GPIO.output(18, GPIO.LOW)

car_activated = False


def create_data(byte1, kp, speed, kd):
    """
    Crear el array de 7 bytes.
    byte1: Byte de funcionamiento.
    kp, speed, kd: Valores de intensidad (0 a 1000).
    """
    kp_bytes = struct.pack('>H', kp)  # >H: Unsigned short big-endian
    speed_bytes = struct.pack('>H', speed)
    kd_bytes = struct.pack('>H', kd)

    # Crear el paquete
    data = bytearray()
    data.append(byte1)       # Byte de funcionamiento
    data.extend(kp_bytes)    # Bytes kp
    data.extend(speed_bytes) # Bytes speed
    data.extend(kd_bytes)    # Bytes kd

    return data

def update_controls_state():
    """
    Actualiza el estado de los controles en función de si el carro está activo o no.
    """
    global car_activated

    if car_activate.get() == "1" and not car_activated:  # Carro activado
        # Deshabilitar todos los controles
        direction_radio_izq.config(state="disabled")
        direction_radio_der.config(state="disabled")
        red_radio_frenar.config(state="disabled")
        red_radio_cambiar.config(state="disabled")
        red_radio_retroceder.config(state="disabled")
        green_radio_frenar.config(state="disabled")
        green_radio_cambiar.config(state="disabled")
        green_radio_retroceder.config(state="disabled")
        blue_radio_frenar.config(state="disabled")
        blue_radio_cambiar.config(state="disabled")
        blue_radio_retroceder.config(state="disabled")
        speed_entry.config(state="disabled")
        kp_entry.config(state="disabled")
        kd_entry.config(state="disabled")

        car_activated = True
    elif car_activate.get() == "0" and car_activated:  # Carro desactivado
        # Habilitar todos los controles
        direction_radio_izq.config(state="normal")
        direction_radio_der.config(state="normal")
        red_radio_frenar.config(state="normal")
        red_radio_cambiar.config(state="normal")
        red_radio_retroceder.config(state="normal")
        green_radio_frenar.config(state="normal")
        green_radio_cambiar.config(state="normal")
        green_radio_retroceder.config(state="normal")
        blue_radio_frenar.config(state="normal")
        blue_radio_cambiar.config(state="normal")
        blue_radio_retroceder.config(state="normal")
        speed_entry.config(state="normal")
        kp_entry.config(state="normal")
        kd_entry.config(state="normal")

        car_activated = False

def send_data():
    """
    Obtener valores de la GUI y enviarlos a través de UART.
    """
    global car_activated

    if car_activate.get() == "1" and car_activated:
        # Mostrar mensaje de error en la terminal
        print("Error: El carro está activo. No se pueden cambiar parámetros.")
        return

    # Construir el byte1 con los bits configurados
    byte1 = (
        (int(car_activate.get()) << 7) |
        (int(direction.get()) << 6) |
        (int(red_action.get()) << 4) |
        (int(green_action.get()) << 2) |
        int(blue_action.get())
    )
    kp = int(kp_var.get())
    speed = int(speed_var.get())
    kd = int(kd_var.get())

    # Crear y enviar el paquete
    data_to_send = create_data(byte1, kp, speed, kd)
    uart.write(data_to_send)

    print("Datos enviados:", data_to_send)

    update_controls_state()

    GPIO.output(18, GPIO.HIGH)
    root.after(10, lambda: GPIO.output(18, GPIO.LOW))

def receive_data():
    """
    Leer datos desde UART.
    """
    #if uart.in_waiting >= 3:  # Esperar a tener al menos 3 bytes
    # TODO: Remove
    if uart.in_waiting >= 8:
        # Leer contadores
        contador_rojo = int.from_bytes(uart.read(1), byteorder='big')
        contador_verde = int.from_bytes(uart.read(1), byteorder='big')
        contador_azul = int.from_bytes(uart.read(1), byteorder='big')

        # Actualizar etiquetas en la GUI
        rojo_counter_label.config(text=f"Contador Rojo: {contador_rojo}")
        verde_counter_label.config(text=f"Contador Verde: {contador_verde}")
        azul_counter_label.config(text=f"Contador Azul: {contador_azul}")

        # TODO: Remove
        estado = int.from_bytes(uart.read(1), byteorder='big')
        estado_calibracion = int.from_bytes(uart.read(1), byteorder='big')
        color_actual = int.from_bytes(uart.read(1), byteorder='big')

        estados = {
            0: "PAUSED",
            1: "FORWARD",
            2: "FAST",
            3: "BACKWARD",
            4: "BRAKE"
        }

        estados_calibracion = {
            0: "UNCALIBRATED",
            1: "CALIBRATING",
            2: "CALIBRATED"
        }

        colores = {
            0: "RED",
            1: "GREEN",
            2: "BLUE",
            3: "OTHER_COLOR"
        }

        print(f"Estado: {estados[estado]}")
        print(f"Calibración: {estados_calibracion[estado_calibracion]}")
        print(f"Color: {colores[color_actual]}")

        posicion = int.from_bytes(uart.read(2), byteorder='big', signed=True)
        print(f"Posición: {posicion}\n")

    # Llamar nuevamente después de 10 ms
    root.after(10, receive_data)

# Crear la ventana principal
root = tk.Tk()
root.title("Control UART")

# Variables para los controles
car_activate = tk.StringVar(value="0")
direction = tk.StringVar(value="0")
red_action = tk.StringVar(value="0")
green_action = tk.StringVar(value="0")
blue_action = tk.StringVar(value="0")

kp_var = tk.StringVar(value="250")
speed_var = tk.StringVar(value="200")
kd_var = tk.StringVar(value="0")

# Controles de la GUI
frame1 = tk.LabelFrame(root, text="Control del Byte 1", padx=10, pady=10)
frame1.pack(padx=10, pady=10, fill="x")

tk.Label(frame1, text="Carro activado:").grid(row=0, column=0, sticky="w")
car_radio_off = tk.Radiobutton(frame1, text="Desactivado", variable=car_activate, value="0")
car_radio_on = tk.Radiobutton(frame1, text="Activado", variable=car_activate, value="1")
car_radio_off.grid(row=0, column=1)
car_radio_on.grid(row=0, column=2)

tk.Label(frame1, text="Dirección:").grid(row=1, column=0, sticky="w")
direction_radio_izq = tk.Radiobutton(frame1, text="Izquierda", variable=direction, value="0")
direction_radio_der = tk.Radiobutton(frame1, text="Derecha", variable=direction, value="1")
direction_radio_izq.grid(row=1, column=1)
direction_radio_der.grid(row=1, column=2)

tk.Label(frame1, text="Acción rojo:").grid(row=2, column=0, sticky="w")
red_radio_frenar = tk.Radiobutton(frame1, text="Frenar", variable=red_action, value="0")
red_radio_cambiar = tk.Radiobutton(frame1, text="Cambiar", variable=red_action, value="1")
red_radio_retroceder = tk.Radiobutton(frame1, text="Retroceder", variable=red_action, value="2")
red_radio_frenar.grid(row=2, column=1)
red_radio_cambiar.grid(row=2, column=2)
red_radio_retroceder.grid(row=2, column=3)

tk.Label(frame1, text="Acción verde:").grid(row=3, column=0, sticky="w")
green_radio_frenar = tk.Radiobutton(frame1, text="Frenar", variable=green_action, value="0")
green_radio_cambiar = tk.Radiobutton(frame1, text="Cambiar", variable=green_action, value="1")
green_radio_retroceder = tk.Radiobutton(frame1, text="Retroceder", variable=green_action, value="2")
green_radio_frenar.grid(row=3, column=1)
green_radio_cambiar.grid(row=3, column=2)
green_radio_retroceder.grid(row=3, column=3)

tk.Label(frame1, text="Acción azul:").grid(row=4, column=0, sticky="w")
blue_radio_frenar = tk.Radiobutton(frame1, text="Frenar", variable=blue_action, value="0")
blue_radio_cambiar = tk.Radiobutton(frame1, text="Cambiar", variable=blue_action, value="1")
blue_radio_retroceder = tk.Radiobutton(frame1, text="Retroceder", variable=blue_action, value="2")
blue_radio_frenar.grid(row=4, column=1)
blue_radio_cambiar.grid(row=4, column=2)
blue_radio_retroceder.grid(row=4, column=3)

frame2 = tk.LabelFrame(root, text="Parametros PID", padx=10, pady=10)
frame2.pack(padx=10, pady=10, fill="x")

tk.Label(frame2, text="Speed: (0-255)").grid(row=0, column=0, sticky="w")
speed_entry = tk.Entry(frame2, textvariable=speed_var, width=10)
speed_entry.grid(row=0, column=1, sticky="we")

tk.Label(frame2, text="Kp: (x1000)").grid(row=1, column=0, sticky="w")
kp_entry = tk.Entry(frame2, textvariable=kp_var, width=10)
kp_entry.grid(row=1, column=1, sticky="we")

tk.Label(frame2, text="Kd: (x1000)").grid(row=2, column=0, sticky="w")
kd_entry = tk.Entry(frame2, textvariable=kd_var, width=10)
kd_entry.grid(row=2, column=1, sticky="we")

# Frame para mostrar los contadores
frame3 = tk.LabelFrame(root, text="Contadores de Color", padx=10, pady=10)
frame3.pack(padx=10, pady=10, fill="x")

rojo_counter_label = tk.Label(frame3, text="Contador Rojo: 0")
rojo_counter_label.pack(anchor="w")

verde_counter_label = tk.Label(frame3, text="Contador Verde: 0")
verde_counter_label.pack(anchor="w")

azul_counter_label = tk.Label(frame3, text="Contador Azul: 0")
azul_counter_label.pack(anchor="w")

# Botón para enviar datos
send_button = tk.Button(root, text="Enviar", command=send_data)
send_button.pack(pady=10)

# Iniciar recepción de datos
receive_data()

# Iniciar el bucle principal
root.mainloop()
GPIO.cleanup()
