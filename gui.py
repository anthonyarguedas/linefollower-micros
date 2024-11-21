import serial
import struct
import tkinter as tk
from tkinter import ttk
import RPi.GPIO as GPIO
import threading
from queue import Queue
from collections import deque
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import matplotlib.pyplot as plt

# Define dictionaries for states and colors
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

# Configure UART
uart = serial.Serial(
    port='/dev/ttyAMA0',
    baudrate=921600,
    timeout=0
)

GPIO.setmode(GPIO.BCM)
GPIO.setup(18, GPIO.OUT)
GPIO.output(18, GPIO.LOW)

car_activated = False
position_values = deque(maxlen=300)
data_queue = Queue()

def create_data(byte1, kp, speed, kd):
    kp_bytes = struct.pack('>H', kp)
    speed_bytes = struct.pack('>H', speed)
    kd_bytes = struct.pack('>H', kd)

    data = bytearray()
    data.append(byte1)
    data.extend(kp_bytes)
    data.extend(speed_bytes)
    data.extend(kd_bytes)

    return data

def update_controls_state():
    global car_activated
    if car_activate.get() == "1" and not car_activated:
        car_activated = True
    elif car_activate.get() == "0" and car_activated:
        car_activated = False

def send_data():
    global car_activated
    if car_activate.get() == "1" and car_activated:
        print("Error: El carro está activo. No se pueden cambiar parámetros.")
        return

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

    data_to_send = create_data(byte1, kp, speed, kd)
    uart.write(data_to_send)

    print("Datos enviados:", data_to_send)

    update_controls_state()
    GPIO.output(18, GPIO.HIGH)
    root.after(10, lambda: GPIO.output(18, GPIO.LOW))

def uart_polling_thread():
    while True:
        if uart.in_waiting >= 10:
            contador_rojo = int.from_bytes(uart.read(1), byteorder='big')
            contador_verde = int.from_bytes(uart.read(1), byteorder='big')
            contador_azul = int.from_bytes(uart.read(1), byteorder='big')
            estado = int.from_bytes(uart.read(1), byteorder='big')
            estado_calibracion = int.from_bytes(uart.read(1), byteorder='big')
            color_actual = int.from_bytes(uart.read(1), byteorder='big')
            posicion = int.from_bytes(uart.read(2), byteorder='big', signed=True)
            out_of_bounds = bool(int.from_bytes(uart.read(1), byteorder='big'))
            is_fork = bool(int.from_bytes(uart.read(1), byteorder='big'))

            # Append data to thread-safe queue
            data_queue.put({
                "contador_rojo": contador_rojo,
                "contador_verde": contador_verde,
                "contador_azul": contador_azul,
                "estado": estado,
                "estado_calibracion": estado_calibracion,
                "color_actual": color_actual,
                "posicion": posicion,
                "out_of_bounds": out_of_bounds,
                "is_fork": is_fork
            })

def process_queue():
    while not data_queue.empty():
        data = data_queue.get()
        position_values.append(data["posicion"])

        rojo_counter_label.config(text=f"Contador Rojo: {data['contador_rojo']}")
        verde_counter_label.config(text=f"Contador Verde: {data['contador_verde']}")
        azul_counter_label.config(text=f"Contador Azul: {data['contador_azul']}")
        estado_label.config(text=f"Estado: {estados.get(data['estado'], 'UNKNOWN')}")
        calibracion_label.config(text=f"Calibración: {estados_calibracion.get(data['estado_calibracion'], 'UNKNOWN')}")
        color_label.config(text=f"Color: {colores.get(data['color_actual'], 'UNKNOWN')}")
        posicion_label.config(text=f"Posición: {data['posicion']}")
        out_of_bounds_label.config(text=f"Out of Bounds: {data['out_of_bounds']}")
        is_fork_label.config(text=f"Is Fork: {data['is_fork']}")

    root.after(5, process_queue)

def setup_plot(parent):
    global ax, line, fig, canvas
    fig, ax = plt.subplots()
    ax.set_xlim(0, 300)
    ax.set_ylim(0, 7000)
    ax.set_title("Posición")
    ax.set_ylabel("Posición")
    ax.set_xlabel("Time")
    line, = ax.plot([], [], label="Position")
    ax.legend()

    canvas = FigureCanvasTkAgg(fig, master=parent)
    canvas_widget = canvas.get_tk_widget()
    canvas_widget.pack(fill="both", expand=True)

def update_plot():
    if position_values:
        line.set_data(range(len(position_values)), list(position_values))
        ax.set_xlim(0, max(len(position_values), 300))
        ax.relim()
        ax.autoscale_view()
        canvas.draw()
    root.after(50, update_plot)

root = tk.Tk()
root.title("Control UART")

car_activate = tk.StringVar(value="0")
direction = tk.StringVar(value="0")
red_action = tk.StringVar(value="0")
green_action = tk.StringVar(value="0")
blue_action = tk.StringVar(value="0")

kp_var = tk.StringVar(value="4000")
speed_var = tk.StringVar(value="210")
kd_var = tk.StringVar(value="2000")

frame1 = tk.LabelFrame(root, text="Control del Byte 1", padx=10, pady=10)
frame1.pack(padx=10, pady=10, fill="x")

tk.Label(frame1, text="Carro activado:").grid(row=0, column=0, sticky="w")
car_radio_off = tk.Radiobutton(frame1, text="Desactivado", variable=car_activate, value="0")
car_radio_on = tk.Radiobutton(frame1, text="Activado", variable=car_activate, value="1")
car_radio_off.grid(row=0, column=1)
car_radio_on.grid(row=0, column=2)

frame2 = tk.LabelFrame(root, text="Parametros PID", padx=10, pady=10)
frame2.pack(padx=10, pady=10, fill="x")

tk.Label(frame2, text="Speed: (0-255)").grid(row=0, column=0, sticky="w")
speed_entry = tk.Entry(frame2, textvariable=speed_var, width=10)
speed_entry.grid(row=0, column=1, sticky="we")

frame3 = tk.LabelFrame(root, text="Contadores de Color", padx=10, pady=10)
frame3.pack(padx=10, pady=10, fill="x")

rojo_counter_label = tk.Label(frame3, text="Contador Rojo: 0")
rojo_counter_label.pack(anchor="w")

frame4 = tk.LabelFrame(root, text="Estado del Sistema", padx=10, pady=10)
frame4.pack(padx=10, pady=10, fill="x")

estado_label = tk.Label(frame4, text="Estado: PAUSED")
estado_label.pack(anchor="w")

frame5 = tk.LabelFrame(root, text="Gráfico de Posición", padx=10, pady=10)
frame5.pack(padx=10, pady=10, fill="both", expand=True)

setup_plot(frame5)

tk.Button(root, text="Enviar", command=send_data).pack(pady=10)

if uart.in_waiting > 0:
    uart.read_all()

# Start UART polling in a separate thread
threading.Thread(target=uart_polling_thread, daemon=True).start()

# Process the queue and update the GUI
root.after(5, process_queue)
root.after(50, update_plot)

root.mainloop()
GPIO.cleanup()