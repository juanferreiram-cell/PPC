#!/usr/bin/env python3
"""
Robert 2.0 - Raspberry Pi 5

Control de motores con jumpers ENA/ENB, dirección con 4 servos PCA9685,
bomba, cámara USB y RPLIDAR con aviso de precaución a 30 cm.
"""

from __future__ import annotations

import asyncio
import atexit
import glob
import json
import threading
import time
from typing import Any

import cv2
from flask import Flask, Response, jsonify, request
from rplidar import RPLidar
from websockets.asyncio.server import serve
from websockets.exceptions import ConnectionClosed

from gpiozero import Device, DigitalOutputDevice
from gpiozero.pins.lgpio import LGPIOFactory
from adafruit_servokit import ServoKit


# Configuración general del servidor Flask y del WebSocket del RPLIDAR.
HOST = "0.0.0.0"
PUERTO_FLASK = 5000
PUERTO_LIDAR_WS = 8765

# Configuración de la cámara USB.
CAMERA_INDEX = 0
ANCHO_CAMARA = 640
ALTO_CAMARA = 360
FPS_CAMARA = 20

# Puertos posibles del RPLIDAR.
PUERTO_LIDAR_PREFERIDO = (
    "/dev/serial/by-id/"
    "usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
)
PUERTO_LIDAR_RESPALDO = "/dev/ttyUSB0"

# Distancias de trabajo del RPLIDAR.
DISTANCIA_MAXIMA_LIDAR_MM = 1000.0
DISTANCIA_PRECAUCION_MM = 300.0

# Tiempo de activación de la bomba en segundos.
TIEMPO_BOMBA_S = 0.400

# Movimiento suave de la dirección.
INTERVALO_DIRECCION_S = 0.030
PASO_DIRECCION = 1

# Si la app deja de mandar comandos, el rover se detiene por seguridad.
TIEMPO_WATCHDOG_S = 0.90


# Se configura gpiozero para Raspberry Pi 5 usando lgpio.
Device.pin_factory = LGPIOFactory(chip=4)

# Puente H con jumpers puestos en ENA y ENB.
# Eso significa que ENA y ENB no se conectan a la Raspberry
# y los motores quedan habilitados siempre.

# Motor izquierdo.
GPIO_IN1_IZQ = 27
GPIO_IN2_IZQ = 22

# Motor derecho.
GPIO_IN1_DER = 23
GPIO_IN2_DER = 24

# Relé de la bomba.
# Es un relé activo en LOW.
GPIO_RELE_BOMBA = 17

in1_izq = DigitalOutputDevice(GPIO_IN1_IZQ, initial_value=False)
in2_izq = DigitalOutputDevice(GPIO_IN2_IZQ, initial_value=False)
in1_der = DigitalOutputDevice(GPIO_IN1_DER, initial_value=False)
in2_der = DigitalOutputDevice(GPIO_IN2_DER, initial_value=False)

# active_high=False hace que on() mande LOW y active el relé.
rele_bomba = DigitalOutputDevice(
    GPIO_RELE_BOMBA,
    active_high=False,
    initial_value=False,
)


# Dirección I2C del PCA9685.
DIRECCION_PCA9685 = 0x40

# Canales usados por los servos en el PCA9685.
SERVO_DER_DEL = 0
SERVO_DER_TRAS = 7
SERVO_IZQ_DEL = 4
SERVO_IZQ_TRAS = 12

CANALES_SERVOS = (
    SERVO_DER_DEL,
    SERVO_DER_TRAS,
    SERVO_IZQ_DEL,
    SERVO_IZQ_TRAS,
)

# Centros calibrados de cada servo.
CENTRO_DER_DEL = 75
CENTRO_DER_TRAS = 35
CENTRO_IZQ_DEL = 125
CENTRO_IZQ_TRAS = 70

# Desvíos máximos hacia izquierda y derecha.
DESVIO_MAX_IZQUIERDA = 35
DESVIO_MAX_DERECHA = 50

# Recorrido individual de cada servo al girar totalmente a la derecha.
GIRO_DER_DEL_MAX = 55
GIRO_IZQ_DEL_MAX = 50
GIRO_DER_TRAS_MAX = 23
GIRO_IZQ_TRAS_MAX = 55

# Evita que el servo trasero derecho llegue al extremo físico de 0°.
MINIMO_DER_TRAS = 12

# Rango de pulsos para servos.
SERVO_MIN_PULSE_US = 1000
SERVO_MAX_PULSE_US = 2000

# Inicialización del PCA9685 a través de ServoKit.
kit_servo = ServoKit(channels=16, address=DIRECCION_PCA9685)

# Se define el rango de pulsos para cada servo usado.
for canal in CANALES_SERVOS:
    kit_servo.servo[canal].set_pulse_width_range(
        SERVO_MIN_PULSE_US,
        SERVO_MAX_PULSE_US,
    )

# Locks para proteger acceso concurrente desde distintos hilos.
servo_lock = threading.Lock()
control_lock = threading.RLock()

# Estado actual de los ángulos de los servos.
angulos_servos: dict[int, int] = {
    SERVO_DER_DEL: CENTRO_DER_DEL,
    SERVO_DER_TRAS: CENTRO_DER_TRAS,
    SERVO_IZQ_DEL: CENTRO_IZQ_DEL,
    SERVO_IZQ_TRAS: CENTRO_IZQ_TRAS,
}

# Estado actual y objetivo de la dirección.
desvio_actual = 0
desvio_objetivo = 0


# Variables generales del estado del rover.
movimiento_actual = "stop"
bomba_activa = False
temporizador_bomba: threading.Timer | None = None

ultimo_comando_monotonic = time.monotonic()
ultimo_comando = "S"

# Estado compartido del RPLIDAR.
estado_lidar_lock = threading.Lock()
estado_lidar: dict[str, Any] = {
    "connected": False,
    "port": None,
    "warning": False,
    "min_distance_mm": None,
    "message": "RPLIDAR sin conexión",
    "points": 0,
}


def limitar_angulo(angulo: float | int) -> int:
    """Limita cualquier ángulo al rango válido de 0 a 180 grados."""
    return max(0, min(180, int(round(float(angulo)))))


def mapear(
    valor: int,
    entrada_min: int,
    entrada_max: int,
    salida_min: int,
    salida_max: int,
) -> int:
    """Hace un mapeo lineal parecido al map() de Arduino."""
    if entrada_max == entrada_min:
        return salida_min

    proporcion = (valor - entrada_min) / (entrada_max - entrada_min)
    resultado = salida_min + proporcion * (salida_max - salida_min)
    return int(round(resultado))


def mover_servo(canal: int, angulo: int) -> None:
    """Mueve un servo del PCA9685 y guarda el ángulo actual."""
    if canal not in CANALES_SERVOS:
        raise ValueError(f"Canal de servo no permitido: {canal}")

    angulo_seguro = limitar_angulo(angulo)

    with servo_lock:
        kit_servo.servo[canal].angle = angulo_seguro
        angulos_servos[canal] = angulo_seguro


def calcular_angulos_direccion(desvio: int) -> dict[int, int]:
    """
    Calcula la posición de los 4 servos en función del desvío deseado.

    Si el desvío es negativo, gira a la izquierda.
    Si el desvío es positivo, gira a la derecha.
    """
    if desvio < 0:
        giro_izquierda = min(-desvio, DESVIO_MAX_IZQUIERDA)

        return {
            SERVO_DER_DEL: CENTRO_DER_DEL - giro_izquierda,
            SERVO_IZQ_DEL: CENTRO_IZQ_DEL - giro_izquierda,
            SERVO_DER_TRAS: CENTRO_DER_TRAS + giro_izquierda,
            SERVO_IZQ_TRAS: CENTRO_IZQ_TRAS + giro_izquierda,
        }

    giro_derecha = min(desvio, DESVIO_MAX_DERECHA)

    giro_der_del = mapear(
        giro_derecha, 0, DESVIO_MAX_DERECHA, 0, GIRO_DER_DEL_MAX
    )
    giro_izq_del = mapear(
        giro_derecha, 0, DESVIO_MAX_DERECHA, 0, GIRO_IZQ_DEL_MAX
    )
    giro_der_tras = mapear(
        giro_derecha, 0, DESVIO_MAX_DERECHA, 0, GIRO_DER_TRAS_MAX
    )
    giro_izq_tras = mapear(
        giro_derecha, 0, DESVIO_MAX_DERECHA, 0, GIRO_IZQ_TRAS_MAX
    )

    return {
        SERVO_DER_DEL: CENTRO_DER_DEL + giro_der_del,
        SERVO_IZQ_DEL: CENTRO_IZQ_DEL + giro_izq_del,
        SERVO_DER_TRAS: max(
            MINIMO_DER_TRAS,
            CENTRO_DER_TRAS - giro_der_tras,
        ),
        SERVO_IZQ_TRAS: CENTRO_IZQ_TRAS - giro_izq_tras,
    }


def aplicar_direccion(desvio: int) -> None:
    """Aplica a los cuatro servos los ángulos calculados para el giro."""
    angulos = calcular_angulos_direccion(desvio)

    mover_servo(SERVO_DER_DEL, angulos[SERVO_DER_DEL])
    mover_servo(SERVO_IZQ_DEL, angulos[SERVO_IZQ_DEL])
    mover_servo(SERVO_DER_TRAS, angulos[SERVO_DER_TRAS])
    mover_servo(SERVO_IZQ_TRAS, angulos[SERVO_IZQ_TRAS])


def establecer_desvio_objetivo(nuevo_objetivo: int) -> None:
    """Guarda el nuevo objetivo de giro, limitado al rango permitido."""
    global desvio_objetivo

    with control_lock:
        desvio_objetivo = max(
            -DESVIO_MAX_IZQUIERDA,
            min(DESVIO_MAX_DERECHA, int(nuevo_objetivo)),
        )


def hilo_direccion() -> None:
    """
    Hilo que mueve gradualmente la dirección hasta alcanzar
    el valor objetivo, para que el giro sea suave.
    """
    global desvio_actual

    while True:
        cambio = False

        with control_lock:
            objetivo = desvio_objetivo

            if desvio_actual < objetivo:
                desvio_actual = min(
                    desvio_actual + PASO_DIRECCION,
                    objetivo,
                )
                cambio = True

            elif desvio_actual > objetivo:
                desvio_actual = max(
                    desvio_actual - PASO_DIRECCION,
                    objetivo,
                )
                cambio = True

            desvio_a_aplicar = desvio_actual

        if cambio:
            try:
                aplicar_direccion(desvio_a_aplicar)
            except Exception as error:
                print(f"Error moviendo servos: {error}")

        time.sleep(INTERVALO_DIRECCION_S)


def centrar_servos_al_inicio() -> None:
    """Centra los 4 servos uno por uno al iniciar el programa."""
    posiciones = (
        (SERVO_DER_DEL, CENTRO_DER_DEL),
        (SERVO_IZQ_DEL, CENTRO_IZQ_DEL),
        (SERVO_DER_TRAS, CENTRO_DER_TRAS),
        (SERVO_IZQ_TRAS, CENTRO_IZQ_TRAS),
    )

    for canal, angulo in posiciones:
        mover_servo(canal, angulo)
        time.sleep(0.15)


def motores_stop() -> None:
    """
    Detiene ambos motores.

    Como ENA y ENB están puenteados, para detener el rover
    se colocan en LOW las cuatro entradas del puente H.
    """
    global movimiento_actual

    in1_izq.off()
    in2_izq.off()
    in1_der.off()
    in2_der.off()

    movimiento_actual = "stop"


def avanzar() -> None:
    """Hace avanzar ambos motores."""
    global movimiento_actual

    in1_izq.on()
    in2_izq.off()
    in1_der.on()
    in2_der.off()

    movimiento_actual = "avance"


def retroceder() -> None:
    """Hace retroceder ambos motores."""
    global movimiento_actual

    in1_izq.off()
    in2_izq.on()
    in1_der.off()
    in2_der.on()

    movimiento_actual = "retroceso"


def apagar_bomba() -> None:
    """Apaga la bomba y cancela cualquier temporizador pendiente."""
    global bomba_activa, temporizador_bomba

    with control_lock:
        if temporizador_bomba is not None:
            temporizador_bomba.cancel()
            temporizador_bomba = None

        rele_bomba.off()
        bomba_activa = False


def activar_bomba() -> None:
    """Activa la bomba durante el tiempo definido en TIEMPO_BOMBA_S."""
    global bomba_activa, temporizador_bomba

    with control_lock:
        if temporizador_bomba is not None:
            temporizador_bomba.cancel()

        rele_bomba.on()
        bomba_activa = True

        temporizador_bomba = threading.Timer(
            TIEMPO_BOMBA_S,
            apagar_bomba,
        )
        temporizador_bomba.daemon = True
        temporizador_bomba.start()


def procesar_comando(comando: str) -> dict[str, Any]:
    """
    Procesa los comandos del rover.

    F  = adelante
    B  = atrás
    L  = gira dirección a la izquierda
    R  = gira dirección a la derecha
    Q  = adelante izquierda
    E  = adelante derecha
    Z  = atrás izquierda
    C  = atrás derecha
    X  = activa bomba
    S  = stop
    """
    global ultimo_comando_monotonic, ultimo_comando

    comando = str(comando).strip().upper()

    if len(comando) != 1:
        raise ValueError("El comando debe ser una sola letra")

    permitidos = {"F", "B", "L", "R", "Q", "E", "Z", "C", "X", "S"}

    if comando not in permitidos:
        raise ValueError(f"Comando no reconocido: {comando}")

    ultimo_comando_monotonic = time.monotonic()
    ultimo_comando = comando

    if comando == "F":
        establecer_desvio_objetivo(0)
        avanzar()

    elif comando == "B":
        establecer_desvio_objetivo(0)
        retroceder()

    elif comando == "L":
        motores_stop()
        establecer_desvio_objetivo(-DESVIO_MAX_IZQUIERDA)

    elif comando == "R":
        motores_stop()
        establecer_desvio_objetivo(DESVIO_MAX_DERECHA)

    elif comando == "Q":
        establecer_desvio_objetivo(-DESVIO_MAX_IZQUIERDA)
        avanzar()

    elif comando == "E":
        establecer_desvio_objetivo(DESVIO_MAX_DERECHA)
        avanzar()

    elif comando == "Z":
        establecer_desvio_objetivo(-DESVIO_MAX_IZQUIERDA)
        retroceder()

    elif comando == "C":
        establecer_desvio_objetivo(DESVIO_MAX_DERECHA)
        retroceder()

    elif comando == "X":
        activar_bomba()

    elif comando == "S":
        motores_stop()
        establecer_desvio_objetivo(0)
        apagar_bomba()

    return obtener_estado_control()


def obtener_estado_control() -> dict[str, Any]:
    """Devuelve un resumen del estado actual del rover."""
    with control_lock:
        desvio = desvio_actual
        objetivo = desvio_objetivo
        bomba = bomba_activa

    return {
        "ok": True,
        "command": ultimo_comando,
        "movement": movimiento_actual,
        "speed": 1.0 if movimiento_actual != "stop" else 0.0,
        "speed_control": "jumpers_ena_enb",
        "direction_deviation": desvio,
        "direction_target": objetivo,
        "pump": bomba,
        "servos": dict(angulos_servos),
    }


def hilo_watchdog() -> None:
    """
    Vigila que la app siga mandando comandos.

    Si pasa demasiado tiempo sin recibir nada,
    detiene motores y centra dirección.
    """
    while True:
        tiempo_sin_comandos = time.monotonic() - ultimo_comando_monotonic

        with control_lock:
            direccion_fuera_del_centro = desvio_objetivo != 0

        if tiempo_sin_comandos > TIEMPO_WATCHDOG_S:
            if movimiento_actual != "stop" or direccion_fuera_del_centro:
                motores_stop()
                establecer_desvio_objetivo(0)

        time.sleep(0.10)


# Variables globales de la cámara.
frame_lock = threading.Lock()
ultimo_frame_jpeg: bytes | None = None
camara_activa = False


def abrir_camara() -> cv2.VideoCapture:
    """Abre la cámara USB y aplica la configuración deseada."""
    camara = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_V4L2)
    camara.set(cv2.CAP_PROP_FRAME_WIDTH, ANCHO_CAMARA)
    camara.set(cv2.CAP_PROP_FRAME_HEIGHT, ALTO_CAMARA)
    camara.set(cv2.CAP_PROP_FPS, FPS_CAMARA)
    camara.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    return camara


def hilo_camara() -> None:
    """
    Lee continuamente la cámara, genera frames JPEG
    y los deja listos para ser enviados por Flask.
    """
    global ultimo_frame_jpeg, camara_activa

    camara: cv2.VideoCapture | None = None

    while True:
        if camara is None or not camara.isOpened():
            if camara is not None:
                camara.release()

            camara = abrir_camara()
            camara_activa = camara.isOpened()

            if not camara_activa:
                print("No se pudo abrir la cámara. Reintentando...")
                time.sleep(1.0)
                continue

            print(f"Cámara abierta en índice {CAMERA_INDEX}")

        ok, frame = camara.read()

        if not ok:
            camara_activa = False
            camara.release()
            camara = None
            time.sleep(0.20)
            continue

        frame = cv2.resize(frame, (ANCHO_CAMARA, ALTO_CAMARA))

        ok, buffer = cv2.imencode(
            ".jpg",
            frame,
            [int(cv2.IMWRITE_JPEG_QUALITY), 80],
        )

        if not ok:
            continue

        with frame_lock:
            ultimo_frame_jpeg = buffer.tobytes()

        time.sleep(1.0 / FPS_CAMARA)


def generar_frames():
    """Generador MJPEG usado por Flask para transmitir la cámara."""
    while True:
        with frame_lock:
            frame = ultimo_frame_jpeg

        if frame is None:
            time.sleep(0.05)
            continue

        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n"
            b"Cache-Control: no-cache\r\n\r\n"
            + frame
            + b"\r\n"
        )

        time.sleep(1.0 / FPS_CAMARA)


# Aplicación Flask principal.
app = Flask(__name__)


@app.after_request
def agregar_cors(response):
    """Agrega cabeceras CORS para permitir acceso desde Flutter."""
    response.headers["Access-Control-Allow-Origin"] = "*"
    response.headers["Access-Control-Allow-Methods"] = "GET, POST, OPTIONS"
    response.headers["Access-Control-Allow-Headers"] = "Content-Type"
    response.headers["Cache-Control"] = "no-store"
    return response


@app.route("/")
def inicio():
    """Ruta inicial informativa del servidor."""
    return jsonify(
        {
            "name": "Robert 2.0 Raspberry Pi 5",
            "camera": "/video",
            "control": "/control/comando",
            "status": "/control/status",
            "lidar": f"ws://<IP_RASPBERRY>:{PUERTO_LIDAR_WS}",
            "commands": ["F", "B", "L", "R", "Q", "E", "Z", "C", "X", "S"],
        }
    )


@app.route("/video")
def video():
    """Devuelve el stream MJPEG de la cámara."""
    return Response(
        generar_frames(),
        mimetype="multipart/x-mixed-replace; boundary=frame",
    )


@app.route("/control/comando", methods=["POST", "OPTIONS"])
def ruta_control_comando():
    """Recibe un comando del rover desde Flutter o desde otra app."""
    if request.method == "OPTIONS":
        return ("", 204)

    datos = request.get_json(silent=True) or {}
    comando = datos.get("command", "")

    try:
        return jsonify(procesar_comando(comando))
    except ValueError as error:
        return jsonify({"ok": False, "error": str(error)}), 400
    except Exception as error:
        print(f"Error procesando comando: {error}")
        return jsonify({"ok": False, "error": str(error)}), 500


@app.route("/control/status")
def ruta_control_status():
    """Devuelve el estado completo del rover, cámara y RPLIDAR."""
    respuesta = obtener_estado_control()

    with estado_lidar_lock:
        respuesta["lidar"] = dict(estado_lidar)

    respuesta["camera"] = {
        "active": camara_activa,
        "index": CAMERA_INDEX,
        "resolution": [ANCHO_CAMARA, ALTO_CAMARA],
    }

    return jsonify(respuesta)


@app.route("/bomba/on", methods=["GET", "POST"])
def ruta_bomba_on():
    """Activa la bomba durante 400 ms."""
    activar_bomba()
    return jsonify({"ok": True, "pump": True, "duration_ms": 400})


@app.route("/bomba/off", methods=["GET", "POST"])
def ruta_bomba_off():
    """Apaga la bomba."""
    apagar_bomba()
    return jsonify({"ok": True, "pump": False})


@app.route("/health")
def health():
    """Ruta simple para verificar si el sistema está funcionando."""
    with estado_lidar_lock:
        lidar = dict(estado_lidar)

    return jsonify(
        {
            "ok": True,
            "camera": camara_activa,
            "lidar": lidar,
            "movement": movimiento_actual,
        }
    )


def iniciar_servidor_flask() -> None:
    """Inicia Flask en un hilo aparte."""
    print(f"Servidor Flask iniciado en http://{HOST}:{PUERTO_FLASK}")
    print(app.url_map)

    app.run(
        host=HOST,
        port=PUERTO_FLASK,
        debug=False,
        use_reloader=False,
        threaded=True,
    )


def detectar_puerto_lidar() -> str:
    """
    Busca el puerto correcto del RPLIDAR.

    Primero intenta el puerto por-id, luego /dev/ttyUSB0
    y finalmente cualquier /dev/ttyUSB* disponible.
    """
    if glob.glob(PUERTO_LIDAR_PREFERIDO):
        return PUERTO_LIDAR_PREFERIDO

    if glob.glob(PUERTO_LIDAR_RESPALDO):
        return PUERTO_LIDAR_RESPALDO

    puertos = sorted(glob.glob("/dev/ttyUSB*"))

    if puertos:
        return puertos[0]

    return PUERTO_LIDAR_RESPALDO


def actualizar_estado_lidar(
    *,
    connected: bool,
    port: str | None,
    warning: bool,
    min_distance_mm: float | None,
    message: str,
    points: int,
) -> None:
    """Actualiza el estado compartido del RPLIDAR."""
    with estado_lidar_lock:
        estado_lidar.update(
            {
                "connected": connected,
                "port": port,
                "warning": warning,
                "min_distance_mm": min_distance_mm,
                "message": message,
                "points": points,
            }
        )


async def enviar_lidar(websocket) -> None:
    """
    Atiende un cliente WebSocket y le va enviando
    continuamente los puntos del RPLIDAR.
    """
    print("Cliente Flutter conectado al RPLIDAR")

    while True:
        lidar: RPLidar | None = None
        puerto = detectar_puerto_lidar()

        try:
            lidar = RPLidar(puerto)

            print(f"RPLIDAR conectado en {puerto}")
            print("Info:", lidar.get_info())
            print("Health:", lidar.get_health())

            actualizar_estado_lidar(
                connected=True,
                port=puerto,
                warning=False,
                min_distance_mm=None,
                message="RPLIDAR conectado",
                points=0,
            )

            for scan in lidar.iter_scans(max_buf_meas=500):
                puntos = []
                distancia_minima: float | None = None

                # Se filtran solamente los puntos dentro del rango útil.
                for calidad, angulo, distancia in scan:
                    distancia = float(distancia)

                    if 0 < distancia <= DISTANCIA_MAXIMA_LIDAR_MM:
                        puntos.append(
                            {
                                "angle": float(angulo),
                                "distance": distancia,
                                "quality": int(calidad),
                            }
                        )

                        if distancia_minima is None or distancia < distancia_minima:
                            distancia_minima = distancia

                # Se considera advertencia si detecta algo a 30 cm o menos.
                precaucion = (
                    distancia_minima is not None
                    and distancia_minima <= DISTANCIA_PRECAUCION_MM
                )

                if precaucion:
                    mensaje = (
                        "PRECAUCIÓN: objeto a "
                        f"{distancia_minima / 10.0:.1f} cm"
                    )
                elif distancia_minima is not None:
                    mensaje = (
                        "Zona despejada. Objeto más cercano a "
                        f"{distancia_minima / 10.0:.1f} cm"
                    )
                else:
                    mensaje = "Sin objetos dentro de 100 cm"

                actualizar_estado_lidar(
                    connected=True,
                    port=puerto,
                    warning=precaucion,
                    min_distance_mm=distancia_minima,
                    message=mensaje,
                    points=len(puntos),
                )

                await websocket.send(
                    json.dumps(
                        {
                            "points": puntos,
                            "warning": precaucion,
                            "min_distance_mm": distancia_minima,
                            "message": mensaje,
                            "warning_distance_mm": DISTANCIA_PRECAUCION_MM,
                            "max_distance_mm": DISTANCIA_MAXIMA_LIDAR_MM,
                        }
                    )
                )

                await asyncio.sleep(0.02)

        except ConnectionClosed:
            print("Cliente Flutter desconectado del RPLIDAR")
            break

        except Exception as error:
            mensaje_error = f"Error RPLIDAR: {error}"
            print(mensaje_error)

            actualizar_estado_lidar(
                connected=False,
                port=puerto,
                warning=False,
                min_distance_mm=None,
                message=mensaje_error,
                points=0,
            )

            try:
                await websocket.send(
                    json.dumps(
                        {
                            "points": [],
                            "warning": False,
                            "min_distance_mm": None,
                            "message": mensaje_error,
                            "error": True,
                        }
                    )
                )
            except ConnectionClosed:
                break

            await asyncio.sleep(1.0)

        finally:
            if lidar is not None:
                try:
                    lidar.stop()
                except Exception:
                    pass

                try:
                    lidar.stop_motor()
                except Exception:
                    pass

                try:
                    lidar.disconnect()
                except Exception:
                    pass


async def iniciar_servidor_lidar() -> None:
    """Inicia el servidor WebSocket del RPLIDAR."""
    print(
        "WebSocket RPLIDAR iniciado en "
        f"ws://{HOST}:{PUERTO_LIDAR_WS}"
    )

    async with serve(
        enviar_lidar,
        HOST,
        PUERTO_LIDAR_WS,
        ping_interval=20,
        ping_timeout=20,
    ):
        await asyncio.Future()


def cerrar_hardware() -> None:
    """
    Apaga motores, bomba y centra dirección
    antes de cerrar el programa.
    """
    try:
        motores_stop()
        apagar_bomba()
        establecer_desvio_objetivo(0)
        aplicar_direccion(0)
    except Exception:
        pass

    for dispositivo in (
        in1_izq,
        in2_izq,
        in1_der,
        in2_der,
        rele_bomba,
    ):
        try:
            dispositivo.close()
        except Exception:
            pass


# Se registra la rutina de cierre seguro para que se ejecute al salir.
atexit.register(cerrar_hardware)


if __name__ == "__main__":
    # Secuencia principal de arranque del rover.
    print("Iniciando Robert 2.0 en Raspberry Pi 5...")

    motores_stop()
    apagar_bomba()
    centrar_servos_al_inicio()

    # Hilo para mover suavemente la dirección.
    threading.Thread(
        target=hilo_direccion,
        daemon=True,
        name="direccion",
    ).start()

    # Hilo que detiene el rover si la app deja de mandar comandos.
    threading.Thread(
        target=hilo_watchdog,
        daemon=True,
        name="watchdog",
    ).start()

    # Hilo de captura de cámara.
    threading.Thread(
        target=hilo_camara,
        daemon=True,
        name="camara",
    ).start()

    # Hilo del servidor Flask.
    threading.Thread(
        target=iniciar_servidor_flask,
        daemon=True,
        name="flask",
    ).start()

    # Servidor WebSocket del RPLIDAR.
    asyncio.run(iniciar_servidor_lidar())