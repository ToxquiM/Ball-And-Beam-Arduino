import serial
import threading
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import tkinter as tk
import time

# === CONFIGURA TU PUERTO SERIAL ===
puerto = 'COM8'  # Cambia al puerto correcto (ej. 'COM4' en Windows o '/dev/ttyUSB0' en Linux)
baudrate = 9600

# === Variables globales ===
posiciones = []
errores = []
tiempos = []
tiempo_actual = 0
setpoint = 5.0
estado_conexion = False

# === Abrir conexión serial ===
try:
    ser = serial.Serial(puerto, baudrate, timeout=1)
    estado_conexion = True
except:
    estado_conexion = False
    print("⚠️ No se pudo abrir el puerto serial.")

# === Función de lectura serial en hilo ===
def leer_serial():
    global tiempo_actual, estado_conexion
    while True:
        try:
            if ser.in_waiting:
                linea = ser.readline().decode('utf-8').strip()
                if linea.startswith("Posición"):
                    partes = linea.split("\t")
                    pos = float(partes[0].split(":")[1].strip().split()[0])
                    posiciones.append(pos)
                    tiempos.append(tiempo_actual)
                    tiempo_actual += 0.01

                    # Calcular error y guardar
                    err = setpoint - pos
                    errores.append(err)

                    actualizar_estado("🟢 Conectado", "green")
                    actualizar_posicion(pos)
                    actualizar_error(err)
            else:
                actualizar_estado("🔴 Desconectado", "red")
            time.sleep(0.1)
        except:
            actualizar_estado("🔴 Desconectado", "red")
            continue

# === Función para enviar el setpoint ===
def enviar_setpoint():
    global setpoint
    try:
        nuevo = float(entry.get())
        setpoint = nuevo
        comando = f"S{nuevo:.2f}\n"
        ser.write(comando.encode('utf-8'))
        print(f"Setpoint enviado: {nuevo} cm")
    except:
        print("⚠️ Error al enviar el setpoint.")

# === Funciones de actualización visual ===
def actualizar_estado(texto, color):
    estado_label.config(text=texto, fg=color)

def actualizar_posicion(valor):
    posicion_label.config(text=f"Posición actual: {valor:.2f} cm")

def actualizar_error(valor):
    error_label.config(text=f"Error actual: {valor:.2f} cm")

# === Gráfica en tiempo real ===
def animar(i):
    plt.cla()
    plt.plot(tiempos, posiciones, label='Posición (cm)', color='blue')
    plt.axhline(setpoint, color='red', linestyle='--', label='Setpoint')
    plt.xlabel('Tiempo (s)')
    plt.ylabel('Posición (cm)')
    plt.title('Respuesta del sistema Ball and Beam')
    plt.grid(True)
    plt.legend()

# === Interfaz gráfica con Tkinter ===
ventana = tk.Tk()
ventana.title("Interfaz PID - Ball and Beam")

tk.Label(ventana, text="Nuevo Setpoint (cm):").pack()

entry = tk.Entry(ventana)
entry.insert(0, "5.0")
entry.pack()

boton = tk.Button(ventana, text="Enviar", command=enviar_setpoint)
boton.pack(pady=5)

estado_label = tk.Label(ventana, text="🔴 Desconectado", fg="red", font=("Arial", 10, "bold"))
estado_label.pack()

# NUEVOS CAMPOS
posicion_label = tk.Label(ventana, text="Posición actual: --- cm", font=("Arial", 10))
posicion_label.pack()

error_label = tk.Label(ventana, text="Error actual: --- cm", font=("Arial", 10))
error_label.pack(pady=(0,10))

# === Iniciar lectura serial en segundo plano ===
hilo_serial = threading.Thread(target=leer_serial)
hilo_serial.daemon = True
hilo_serial.start()

# === Mostrar la gráfica y ventana ===
fig = plt.figure()
ani = FuncAnimation(fig, animar, interval=100)

plt.ion()
plt.show()
ventana.mainloop()
