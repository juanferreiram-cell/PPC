from flask import Flask, Response, jsonify, request
import cv2
import asyncio
import json
from threading import Thread

from rplidar import RPLidar
from websockets.asyncio.server import serve

from gpiozero import Device, DigitalOutputDevice, PWMOutputDevice
from gpiozero.pins.lgpio import LGPIOFactory


# ============================================================
# CONFIGURACIÓN GENERAL
# ============================================================

HOST = "0.0.0.0"

PUERTO_CAMARA = 5000
PUERTO_LIDAR_WS = 8765

# Puerto del RPLIDAR en Raspberry
PUERTO_LIDAR = "/dev/ttyUSB0"

# Resolución de cámara
ANCHO_CAMARA = 640
ALTO_CAMARA = 360

# RPLIDAR limitado a 100 cm
DISTANCIA_MAXIMA_LIDAR_MM = 1000


# ============================================================
# PINES DEL PUENTE H - L298N / TB6612 / Similar
# ============================================================

# Motor izquierdo
GPIO_IN1_IZQ = 27
GPIO_IN2_IZQ = 22
GPIO_PWM_IZQ = 18

# Motor derecho
GPIO_IN1_DER = 23
GPIO_IN2_DER = 24
GPIO_PWM_DER = 13

FRECUENCIA_PWM = 1000


# ============================================================
# GPIO RASPBERRY PI 5
# ============================================================

# En Raspberry Pi 5 venías usando chip=4.
# Si te diera error de GPIO, probá con:
# Device.pin_factory = LGPIOFactory()
Device.pin_factory = LGPIOFactory(chip=4)

# Motor izquierdo
in1_izq = DigitalOutputDevice(GPIO_IN1_IZQ)
in2_izq = DigitalOutputDevice(GPIO_IN2_IZQ)
pwm_izq = PWMOutputDevice(
    GPIO_PWM_IZQ,
    frequency=FRECUENCIA_PWM,
    initial_value=0
)

# Motor derecho
in1_der = DigitalOutputDevice(GPIO_IN1_DER)
in2_der = DigitalOutputDevice(GPIO_IN2_DER)
pwm_der = PWMOutputDevice(
    GPIO_PWM_DER,
    frequency=FRECUENCIA_PWM,
    initial_value=0
)

velocidad_actual = 0.50
movimiento_actual = "stop"


def limitar_velocidad(valor):
    try:
        valor = float(valor)
    except:
        valor = 0.0

    if valor < 0:
        valor = 0.0

    if valor > 1:
        valor = 1.0

    return valor


def motor_izquierdo_adelante(velocidad):
    in1_izq.on()
    in2_izq.off()
    pwm_izq.value = velocidad


def motor_izquierdo_atras(velocidad):
    in1_izq.off()
    in2_izq.on()
    pwm_izq.value = velocidad


def motor_izquierdo_stop():
    in1_izq.off()
    in2_izq.off()
    pwm_izq.value = 0


def motor_derecho_adelante(velocidad):
    in1_der.on()
    in2_der.off()
    pwm_der.value = velocidad


def motor_derecho_atras(velocidad):
    in1_der.off()
    in2_der.on()
    pwm_der.value = velocidad


def motor_derecho_stop():
    in1_der.off()
    in2_der.off()
    pwm_der.value = 0


def motores_stop():
    motor_izquierdo_stop()
    motor_derecho_stop()


def aplicar_movimiento(movimiento, velocidad=None):
    global velocidad_actual, movimiento_actual

    if velocidad is not None:
        velocidad_actual = limitar_velocidad(velocidad)

    v = velocidad_actual

    if movimiento == "avance":
        motor_izquierdo_adelante(v)
        motor_derecho_adelante(v)
        movimiento_actual = "avance"

    elif movimiento == "retroceso":
        motor_izquierdo_atras(v)
        motor_derecho_atras(v)
        movimiento_actual = "retroceso"

    elif movimiento == "izquierda":
        # Giro sobre su propio eje hacia la izquierda
        motor_izquierdo_atras(v)
        motor_derecho_adelante(v)
        movimiento_actual = "izquierda"

    elif movimiento == "derecha":
        # Giro sobre su propio eje hacia la derecha
        motor_izquierdo_adelante(v)
        motor_derecho_atras(v)
        movimiento_actual = "derecha"

    else:
        motores_stop()
        movimiento_actual = "stop"


def actualizar_pwm(velocidad):
    global velocidad_actual

    velocidad_actual = limitar_velocidad(velocidad)

    if movimiento_actual == "stop":
        motores_stop()
    else:
        aplicar_movimiento(movimiento_actual, velocidad_actual)


# Por seguridad, arranca apagado
motores_stop()


# ============================================================
# SERVIDOR FLASK - CÁMARA + CONTROL MOTOR
# ============================================================

app = Flask(__name__)

camara = cv2.VideoCapture(0)


def generar_frames():
    while True:
        ok, frame = camara.read()

        if not ok:
            continue

        frame = cv2.resize(frame, (ANCHO_CAMARA, ALTO_CAMARA))

        ok, buffer = cv2.imencode(".jpg", frame)

        if not ok:
            continue

        frame_bytes = buffer.tobytes()

        yield (
            b"--frame\r\n"
            b"Content-Type: image/jpeg\r\n\r\n" + frame_bytes + b"\r\n"
        )


@app.route("/")
def inicio():
    return "Servidor cámara + RPLIDAR + 2 motores PWM funcionando"


@app.route("/video")
def video():
    return Response(
        generar_frames(),
        mimetype="multipart/x-mixed-replace; boundary=frame"
    )


@app.route("/motor/avance")
def ruta_motor_avance():
    velocidad = request.args.get("speed", velocidad_actual)
    aplicar_movimiento("avance", velocidad)

    return jsonify({
        "motor": movimiento_actual,
        "speed": velocidad_actual
    })


@app.route("/motor/retroceso")
def ruta_motor_retroceso():
    velocidad = request.args.get("speed", velocidad_actual)
    aplicar_movimiento("retroceso", velocidad)

    return jsonify({
        "motor": movimiento_actual,
        "speed": velocidad_actual
    })


@app.route("/motor/izquierda")
def ruta_motor_izquierda():
    velocidad = request.args.get("speed", velocidad_actual)
    aplicar_movimiento("izquierda", velocidad)

    return jsonify({
        "motor": movimiento_actual,
        "speed": velocidad_actual
    })


@app.route("/motor/derecha")
def ruta_motor_derecha():
    velocidad = request.args.get("speed", velocidad_actual)
    aplicar_movimiento("derecha", velocidad)

    return jsonify({
        "motor": movimiento_actual,
        "speed": velocidad_actual
    })


@app.route("/motor/stop")
def ruta_motor_stop():
    aplicar_movimiento("stop")

    return jsonify({
        "motor": movimiento_actual,
        "speed": 0
    })


@app.route("/motor/pwm")
def ruta_motor_pwm():
    velocidad = request.args.get("speed", velocidad_actual)
    actualizar_pwm(velocidad)

    return jsonify({
        "motor": movimiento_actual,
        "speed": velocidad_actual
    })


@app.route("/motor/status")
def ruta_motor_status():
    return jsonify({
        "motor": movimiento_actual,
        "speed": velocidad_actual
    })


@app.route("/rutas")
def rutas():
    return str(app.url_map)


def iniciar_servidor_flask():
    print("Servidor Flask iniciado")
    print(app.url_map)

    app.run(
        host=HOST,
        port=PUERTO_CAMARA,
        debug=False,
        use_reloader=False
    )


# ============================================================
# SERVIDOR WEBSOCKET - RPLIDAR
# ============================================================

async def enviar_lidar(websocket):
    print("Cliente conectado al RPLIDAR")

    lidar = RPLidar(PUERTO_LIDAR)

    try:
        print("Info:", lidar.get_info())
        print("Health:", lidar.get_health())

        for scan in lidar.iter_scans():
            puntos = []

            for calidad, angulo, distancia in scan:
                if 0 < distancia <= DISTANCIA_MAXIMA_LIDAR_MM:
                    puntos.append({
                        "angle": angulo,
                        "distance": distancia,
                        "quality": calidad
                    })

            mensaje = json.dumps({
                "points": puntos
            })

            await websocket.send(mensaje)

            await asyncio.sleep(0.02)

    except Exception as e:
        print("Error RPLIDAR:", e)

    finally:
        print("Cerrando RPLIDAR")

        try:
            lidar.stop()
            lidar.stop_motor()
            lidar.disconnect()
        except:
            pass


async def iniciar_servidor_lidar():
    print(f"Servidor WebSocket RPLIDAR iniciado en ws://{HOST}:{PUERTO_LIDAR_WS}")

    async with serve(enviar_lidar, HOST, PUERTO_LIDAR_WS):
        await asyncio.Future()


# ============================================================
# MAIN
# ============================================================

if __name__ == "__main__":
    print("Iniciando servidor unificado...")

    hilo_flask = Thread(target=iniciar_servidor_flask)
    hilo_flask.daemon = True
    hilo_flask.start()

    asyncio.run(iniciar_servidor_lidar())