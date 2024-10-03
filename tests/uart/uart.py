import serial
import threading

def sender(teensy):
    # Bandera que determina si ya se envió un comando de "inicio"
    started = False

    while True:
        print("a - Inicio\nb - Fin\nc - Asignar acciones")
        option = input("Seleccione una opción: ")

        if option not in ("a", "b", "c"):
            print("Opción inválida\n")
            continue

        match option:
            case "a":
                if not started:
                    teensy.write(b'\x01\x01\x01')
                    started = True
                else:
                    print("El carrito ya inició\n")
            case "b":
                if started:
                    teensy.write(b'\x00\x00\x00')
                    started = False
                else:
                    print("El carrito ya finalizó\n")
            case "c":
                if not started:
                    command = b''
                    valid_command = True

                    print("a - Frenar\nb - Retroceder\nc - Cambio de velocidad")

                    for color in ("verde", "azul", "rojo"):
                        action = input(f"Seleccione una opción para el color {color}, no puede repetir: ")

                        if action not in ("a", "b", "c"):
                            print("Opción inválida\n")
                            valid_command = False
                            break

                        match action:
                            case "a":
                                if b"\x02" not in command:
                                    command += b"\x02"
                                else:
                                    print("No puede repetir acciones\n")
                                    valid_command = False
                                    break
                            case "b":
                                if b"\x03" not in command:
                                    command += b"\x03"
                                else:
                                    print("No puede repetir acciones\n")
                                    valid_command = False
                                    break
                            case "c":
                                if b"\x04" not in command:
                                    command += b"\x04"
                                else:
                                    print("No puede repetir acciones\n")
                                    valid_command = False
                                    break

                    if valid_command:
                        teensy.write(command)
                else:
                    print("No se pueden asignar acciones mientras el carrito está en movimiento\n")

def receiver(teensy):
    while True:
        # Esperar todo el tiempo necesario a que se reciba una línea
        line = teensy.readline()

        # Dividir en 3 valores separados por comas
        encounters = line.decode().strip().split(",")

        if len(encounters) == 3:
            print(f"Segmentos verdes encontrados: {encounters[0]}")
            print(f"Segmentos azules encontrados: {encounters[1]}")
            print(f"Segmentos rojos encontrados: {encounters[2]}\n")


# Ajustar el puerto COM y el baudrate
teensy = serial.Serial("COMX", 115200)

# Crear threads para el emisor y el receptor
sender_thread = threading.Thread(target=sender, args=(teensy,))
receiver_thread = threading.Thread(target=receiver, args=(teensy,))

sender_thread.start()
receiver_thread.start()

# Esperar a que se completen los threads
sender_thread.join()
receiver_thread.join()