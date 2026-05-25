#!/usr/bin/env python3
"""
plot_plancha.py — Graficador en tiempo real para PlanchaVapor
Lee datos CSV del puerto serial del ESP32 y muestra 3 gráficas en vivo:
  1. Temperatura vs Tiempo (con setpoint)
  2. Duty cycle vs Tiempo
  3. Términos PID (P, I, D) vs Tiempo

Uso:
  python3 plot_plancha.py                    # autodetecta puerto
  python3 plot_plancha.py /dev/cu.usbserial-0001   # puerto explícito
  python3 plot_plancha.py --baud 115200      # cambiar baudrate

Requisitos:
  pip3 install pyserial matplotlib

Formato esperado del ESP32 (una línea por lectura):
  DATA,timestamp_ms,temp,setpoint,duty,estado,p_term,i_term,d_term
"""

import sys
import time
import glob
import argparse
import csv
from collections import deque
from datetime import datetime

import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from matplotlib.patches import Patch

# ─── Configuración ───────────────────────────────────────────────
MAX_POINTS = 600          # Puntos en pantalla (~10 min a 1 lectura/s)
SAVE_CSV = True           # Guardar todos los datos a archivo CSV
INTERVAL_MS = 250         # Intervalo de actualización de gráfica (ms)

# Colores para los estados
STATE_COLORS = {
    "Reposo":       "#888888",
    "Precalent.":   "#FFA500",
    "Vapor alta":   "#FF0000",
    "Vapor media":  "#FF6600",
    "Vapor baja":   "#FFCC00",
    "Enfriando":    "#00BFFF",
    "Fin ciclo":    "#00CC00",
    "ERROR":        "#FF00FF",
}

# ─── Autodetección de puerto serial en macOS ─────────────────────
def find_serial_port():
    """Busca puertos USB-serial comunes en macOS."""
    patterns = [
        "/dev/cu.usbserial-*",
        "/dev/cu.SLAB_USBtoUART*",
        "/dev/cu.wchusbserial*",
        "/dev/cu.usbmodem*",
        "/dev/ttyUSB*",         # Linux
        "/dev/ttyACM*",         # Linux
    ]
    for pat in patterns:
        ports = glob.glob(pat)
        if ports:
            return ports[0]
    return None


# ─── Buffers de datos ────────────────────────────────────────────
t_sec    = deque(maxlen=MAX_POINTS)   # tiempo en segundos desde inicio
temp     = deque(maxlen=MAX_POINTS)   # temperatura medida
setpoint = deque(maxlen=MAX_POINTS)   # setpoint actual
duty     = deque(maxlen=MAX_POINTS)   # duty cycle (0-100%)
p_term   = deque(maxlen=MAX_POINTS)   # término proporcional
i_term   = deque(maxlen=MAX_POINTS)   # término integral
d_term   = deque(maxlen=MAX_POINTS)   # término derivativo
states   = deque(maxlen=MAX_POINTS)   # nombre del estado

t0_ms = None  # primer timestamp para calcular tiempo relativo


def parse_line(line_str):
    """Parsea una línea CSV del ESP32. Retorna dict o None."""
    global t0_ms
    line_str = line_str.strip()
    if not line_str.startswith("DATA,"):
        return None
    parts = line_str.split(",")
    if len(parts) < 9:
        return None
    try:
        ts_ms   = int(parts[1])
        t_c     = float(parts[2])
        sp      = float(parts[3])
        d       = float(parts[4])
        state   = parts[5].strip()
        p       = float(parts[6])
        i       = float(parts[7])
        d_val   = float(parts[8])
    except (ValueError, IndexError):
        return None

    if t0_ms is None:
        t0_ms = ts_ms

    return {
        "t_sec":    (ts_ms - t0_ms) / 1000.0,
        "temp":     t_c,
        "setpoint": sp,
        "duty":     d * 100.0,  # convertir 0-1 a porcentaje
        "state":    state,
        "p_term":   p,
        "i_term":   i,
        "d_term":   d_val,
    }


# ─── Archivo CSV de log ──────────────────────────────────────────
csv_file = None
csv_writer = None

def init_csv_log():
    global csv_file, csv_writer
    if not SAVE_CSV:
        return
    fname = datetime.now().strftime("plancha_log_%Y%m%d_%H%M%S.csv")
    csv_file = open(fname, "w", newline="")
    csv_writer = csv.writer(csv_file)
    csv_writer.writerow([
        "t_seg", "temp_C", "setpoint_C", "duty_%",
        "estado", "P", "I", "D"
    ])
    print(f"  Guardando datos en: {fname}")


# ─── Setup de gráficas ──────────────────────────────────────────
def setup_plots():
    fig, axes = plt.subplots(3, 1, figsize=(12, 8), sharex=True)
    fig.suptitle("PlanchaVapor — Monitor en Tiempo Real", fontsize=14, fontweight="bold")
    fig.patch.set_facecolor("#1e1e1e")

    for ax in axes:
        ax.set_facecolor("#2d2d2d")
        ax.tick_params(colors="#cccccc")
        ax.xaxis.label.set_color("#cccccc")
        ax.yaxis.label.set_color("#cccccc")
        ax.title.set_color("#ffffff")
        for spine in ax.spines.values():
            spine.set_color("#555555")
        ax.grid(True, alpha=0.3, color="#666666")

    # Gráfica 1: Temperatura
    ax1 = axes[0]
    ax1.set_title("Temperatura vs Tiempo")
    ax1.set_ylabel("°C")
    line_temp, = ax1.plot([], [], color="#FF4444", linewidth=1.5, label="Medida")
    line_sp,   = ax1.plot([], [], color="#44FF44", linewidth=1.2, linestyle="--", label="Setpoint")
    ax1.axhline(y=95, color="#FF00FF", linewidth=0.8, linestyle=":", alpha=0.7, label="Corte seguridad (95°C)")
    ax1.axhline(y=40, color="#00BFFF", linewidth=0.8, linestyle=":", alpha=0.5, label="Seguro abrir (40°C)")
    ax1.legend(loc="upper left", fontsize=8, facecolor="#333333", edgecolor="#555555", labelcolor="#cccccc")
    ax1.set_ylim(15, 100)

    # Gráfica 2: Duty cycle
    ax2 = axes[1]
    ax2.set_title("Duty Cycle del SSR")
    ax2.set_ylabel("%")
    line_duty, = ax2.plot([], [], color="#FFAA00", linewidth=1.5, label="Duty")
    ax2.axhline(y=90, color="#FF6666", linewidth=0.8, linestyle=":", alpha=0.7, label="Máx 90%")
    ax2.legend(loc="upper left", fontsize=8, facecolor="#333333", edgecolor="#555555", labelcolor="#cccccc")
    ax2.set_ylim(-5, 105)

    # Gráfica 3: Términos PID
    ax3 = axes[2]
    ax3.set_title("Términos PID")
    ax3.set_ylabel("Contribución")
    ax3.set_xlabel("Tiempo (s)")
    line_p, = ax3.plot([], [], color="#44AAFF", linewidth=1.2, label="P")
    line_i, = ax3.plot([], [], color="#44FF44", linewidth=1.2, label="I")
    line_d, = ax3.plot([], [], color="#FF44FF", linewidth=1.2, label="D")
    ax3.legend(loc="upper left", fontsize=8, facecolor="#333333", edgecolor="#555555", labelcolor="#cccccc")

    # Texto de estado actual
    state_text = fig.text(0.98, 0.97, "Estado: ---", ha="right", va="top",
                          fontsize=12, fontweight="bold", color="#00FF00",
                          fontfamily="monospace",
                          bbox=dict(boxstyle="round,pad=0.3", facecolor="#333333", edgecolor="#555555"))

    plt.tight_layout(rect=[0, 0, 1, 0.95])

    lines = {
        "temp": line_temp, "sp": line_sp, "duty": line_duty,
        "p": line_p, "i": line_i, "d": line_d,
    }
    return fig, axes, lines, state_text


# ─── Función de actualización (llamada por animation) ───────────
def make_update(ser, axes, lines, state_text):
    def update(frame):
        # Leer todas las líneas disponibles en el buffer serial
        while ser.in_waiting:
            try:
                raw = ser.readline().decode("utf-8", errors="replace")
            except Exception:
                continue
            data = parse_line(raw)
            if data is None:
                continue

            t_sec.append(data["t_sec"])
            temp.append(data["temp"])
            setpoint.append(data["setpoint"])
            duty.append(data["duty"])
            p_term.append(data["p_term"])
            i_term.append(data["i_term"])
            d_term.append(data["d_term"])
            states.append(data["state"])

            if csv_writer:
                csv_writer.writerow([
                    f"{data['t_sec']:.1f}", f"{data['temp']:.2f}",
                    f"{data['setpoint']:.1f}", f"{data['duty']:.1f}",
                    data["state"],
                    f"{data['p_term']:.4f}", f"{data['i_term']:.4f}",
                    f"{data['d_term']:.4f}",
                ])

        if len(t_sec) < 2:
            return lines.values()

        t_list = list(t_sec)

        # Actualizar líneas
        lines["temp"].set_data(t_list, list(temp))
        lines["sp"].set_data(t_list, list(setpoint))
        lines["duty"].set_data(t_list, list(duty))
        lines["p"].set_data(t_list, list(p_term))
        lines["i"].set_data(t_list, list(i_term))
        lines["d"].set_data(t_list, list(d_term))

        # Ajustar ejes X
        x_min = t_list[0]
        x_max = t_list[-1]
        margin = max(5, (x_max - x_min) * 0.02)
        for ax in axes:
            ax.set_xlim(x_min - margin, x_max + margin)

        # Ajustar eje Y de temperatura dinámicamente
        t_min = min(temp)
        t_max = max(temp)
        sp_max = max(setpoint) if setpoint else 0
        y_hi = max(t_max, sp_max, 95) + 5
        y_lo = max(t_min - 5, 0)
        axes[0].set_ylim(y_lo, y_hi)

        # Ajustar eje Y de PID
        all_pid = list(p_term) + list(i_term) + list(d_term)
        if all_pid:
            pid_min = min(all_pid)
            pid_max = max(all_pid)
            pid_margin = max(0.05, (pid_max - pid_min) * 0.1)
            axes[2].set_ylim(pid_min - pid_margin, pid_max + pid_margin)

        # Actualizar texto de estado
        current_state = states[-1] if states else "---"
        color = STATE_COLORS.get(current_state, "#FFFFFF")
        state_text.set_text(f"Estado: {current_state}")
        state_text.set_color(color)

        return lines.values()

    return update


# ─── Main ────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(description="Graficador en tiempo real para PlanchaVapor")
    parser.add_argument("port", nargs="?", default=None, help="Puerto serial (ej: /dev/cu.usbserial-0001)")
    parser.add_argument("--baud", type=int, default=115200, help="Baudrate (default: 115200)")
    parser.add_argument("--no-save", action="store_true", help="No guardar CSV")
    args = parser.parse_args()

    global SAVE_CSV
    if args.no_save:
        SAVE_CSV = False

    # Encontrar puerto
    port = args.port or find_serial_port()
    if port is None:
        print("ERROR: No se encontró puerto serial.")
        print("  Conecta el ESP32 por USB y reintenta, o especifica el puerto:")
        print("  python3 plot_plancha.py /dev/cu.usbserial-XXXX")
        print("\nPuertos disponibles:")
        from serial.tools import list_ports
        for p in list_ports.comports():
            print(f"  {p.device} — {p.description}")
        sys.exit(1)

    print(f"╔══════════════════════════════════════════════╗")
    print(f"║  PlanchaVapor — Monitor en Tiempo Real       ║")
    print(f"╠══════════════════════════════════════════════╣")
    print(f"║  Puerto:   {port:<34}║")
    print(f"║  Baud:     {args.baud:<34}║")
    print(f"╚══════════════════════════════════════════════╝")
    print(f"  Esperando datos (líneas con prefijo DATA,)...")
    print(f"  Cierra la ventana de gráficas para terminar.\n")

    # Abrir serial
    try:
        ser = serial.Serial(port, args.baud, timeout=0.1)
        time.sleep(2)  # Esperar reset del ESP32
        ser.reset_input_buffer()
    except serial.SerialException as e:
        print(f"ERROR abriendo puerto: {e}")
        print("  Asegúrate de que el monitor serial de idf.py NO esté abierto.")
        sys.exit(1)

    init_csv_log()

    # Setup gráficas
    fig, axes, lines, state_text = setup_plots()

    # Animación
    update_fn = make_update(ser, axes, lines, state_text)
    ani = animation.FuncAnimation(
        fig, update_fn, interval=INTERVAL_MS, blit=False, cache_frame_data=False
    )

    try:
        plt.show()
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()
        if csv_file:
            csv_file.close()
        print("\nMonitor cerrado.")


if __name__ == "__main__":
    main()
