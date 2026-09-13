import 'dart:async';
import 'dart:convert';
import 'dart:math' as math;

import 'package:flutter/material.dart';
import 'package:flutter/services.dart';
import 'package:http/http.dart' as http;
import 'package:mjpeg_view/mjpeg_view.dart';
import 'package:web_socket_channel/web_socket_channel.dart';

Future<void> main() async {
  WidgetsFlutterBinding.ensureInitialized();

  // La aplicación funciona únicamente en orientación horizontal.
  await SystemChrome.setPreferredOrientations([
    DeviceOrientation.landscapeLeft,
    DeviceOrientation.landscapeRight,
  ]);

  runApp(const RobotApp());
}

// Aplicación principal de Robert.
class RobotApp extends StatelessWidget {
  const RobotApp({super.key});

  @override
  Widget build(BuildContext context) {
    return MaterialApp(
      debugShowCheckedModeBanner: false,
      title: 'Robert Rover',
      theme: ThemeData.dark().copyWith(
        scaffoldBackgroundColor: const Color(0xFF080A0D),
      ),
      home: const RobotScreen(),
    );
  }
}

// Pantalla principal de control del Rover.
class RobotScreen extends StatefulWidget {
  const RobotScreen({super.key});

  @override
  State<RobotScreen> createState() => _RobotScreenState();
}

class _RobotScreenState extends State<RobotScreen> {
  // IP local de la Raspberry Pi 5.
  // Solamente debe cambiarse si cambia la dirección IP de la Raspberry.
  static const String ipRaspberry = '192.168.137.87';

  // Direcciones utilizadas para comunicarse con Python y Flask.
  static const String urlCamara = 'http://$ipRaspberry:5000/video';

  static const String urlLidar = 'ws://$ipRaspberry:8765';

  static const String urlControlJoystick =
      'http://$ipRaspberry:5000/control/joystick';

  static const String urlControlComando =
      'http://$ipRaspberry:5000/control/comando';

  static const String urlBombaOn = 'http://$ipRaspberry:5000/bomba/on';

  // Distancia a partir de la cual se muestra la advertencia.
  // 300 mm equivalen a 30 cm.
  static const double distanciaAdvertenciaMm = 300.0;

  WebSocketChannel? channel;
  StreamSubscription? subscription;
  Timer? timerEnvioJoystick;

  List<LidarPoint> puntos = [];

  String estadoLidar = 'Conectando al RPLIDAR...';
  String mensajeAdvertencia = '';
  String estadoControl = 'Joystick centrado';
  String estadoBomba = 'Bomba lista';

  double joystickX = 0.0;
  double joystickY = 0.0;
  double intensidad = 0.0;
  double? distanciaMinimaMm;

  String movimientoActual = 'stop';

  bool advertenciaObjeto = false;
  bool hayComandoPendiente = false;
  bool envioEnCurso = false;
  bool joystickActivo = false;
  bool enviandoBomba = false;

  final String idCliente = DateTime.now().microsecondsSinceEpoch.toString();

  int secuenciaControl = 0;

  // Al iniciar la pantalla se conecta al RPLIDAR y crea
  // un temporizador para enviar periódicamente la posición del joystick.
  @override
  void initState() {
    super.initState();

    conectarLidar();

    // Mientras el joystick se mantiene presionado,
    // su posición se envía aproximadamente cada 70 ms.
    timerEnvioJoystick = Timer.periodic(const Duration(milliseconds: 70), (_) {
      if ((joystickActivo || hayComandoPendiente) && !envioEnCurso) {
        hayComandoPendiente = false;

        enviarJoystick(joystickX, joystickY);
      }
    });
  }

  // Abre la comunicación WebSocket con la Raspberry Pi
  // para recibir continuamente los puntos del RPLIDAR.
  void conectarLidar() {
    try {
      channel = WebSocketChannel.connect(Uri.parse(urlLidar));

      subscription = channel!.stream.listen(
        (mensaje) {
          try {
            // Convierte el mensaje JSON recibido desde Python.
            final dynamic decodificado = jsonDecode(mensaje.toString());

            if (decodificado is! Map<String, dynamic>) {
              throw const FormatException('Formato RPLIDAR inválido');
            }

            final dynamic listaRaw = decodificado['points'];

            final List<dynamic> lista = listaRaw is List<dynamic>
                ? listaRaw
                : <dynamic>[];

            // Convierte cada punto recibido en un objeto LidarPoint.
            final List<LidarPoint> nuevosPuntos = lista
                .whereType<Map>()
                .map((p) {
                  return LidarPoint(
                    angle: (p['angle'] as num?)?.toDouble() ?? 0.0,
                    distance: (p['distance'] as num?)?.toDouble() ?? 0.0,
                    quality: (p['quality'] as num?)?.toDouble() ?? 0.0,
                  );
                })
                .where((p) => p.distance > 0)
                .toList();

            double? nuevaDistanciaMinima;

            final dynamic minRaw = decodificado['min_distance_mm'];

            // Utiliza la distancia mínima enviada por Python.
            // Si no está disponible, la calcula usando los puntos recibidos.
            if (minRaw is num) {
              nuevaDistanciaMinima = minRaw.toDouble();
            } else if (nuevosPuntos.isNotEmpty) {
              nuevaDistanciaMinima = nuevosPuntos
                  .map((p) => p.distance)
                  .reduce(math.min);
            }

            final dynamic warningRaw = decodificado['warning'];

            // Determina si existe un obstáculo a 30 cm o menos.
            final bool nuevaAdvertencia = warningRaw is bool
                ? warningRaw
                : nuevaDistanciaMinima != null &&
                      nuevaDistanciaMinima <= distanciaAdvertenciaMm;

            final dynamic messageRaw = decodificado['message'];

            String nuevoMensaje = '';

            if (nuevaAdvertencia) {
              if (messageRaw is String && messageRaw.trim().isNotEmpty) {
                nuevoMensaje = messageRaw;
              } else if (nuevaDistanciaMinima != null) {
                nuevoMensaje =
                    'PRECAUCIÓN: objeto a '
                    '${(nuevaDistanciaMinima / 10).toStringAsFixed(1)} cm';
              } else {
                nuevoMensaje = 'PRECAUCIÓN: objeto cercano';
              }
            }

            if (!mounted) return;

            // Actualiza la información mostrada en pantalla.
            setState(() {
              puntos = nuevosPuntos;
              distanciaMinimaMm = nuevaDistanciaMinima;

              advertenciaObjeto = nuevaAdvertencia;

              mensajeAdvertencia = nuevoMensaje;

              if (nuevaDistanciaMinima != null) {
                estadoLidar =
                    '${puntos.length} puntos | '
                    'Mínimo: '
                    '${(nuevaDistanciaMinima / 10).toStringAsFixed(1)} cm';
              } else {
                estadoLidar = '${puntos.length} puntos | Alcance: 100 cm';
              }
            });
          } catch (error) {
            if (!mounted) return;

            setState(() {
              estadoLidar = 'Datos inválidos del RPLIDAR';

              advertenciaObjeto = false;
              mensajeAdvertencia = '';
            });
          }
        },

        // Se ejecuta si ocurre un error en el WebSocket.
        onError: (Object error) {
          if (!mounted) return;

          setState(() {
            estadoLidar = 'Error RPLIDAR: $error';

            advertenciaObjeto = false;
            mensajeAdvertencia = '';
          });
        },

        // Se ejecuta si la Raspberry cierra la conexión.
        onDone: () {
          if (!mounted) return;

          setState(() {
            estadoLidar = 'Conexión RPLIDAR cerrada';

            advertenciaObjeto = false;
            mensajeAdvertencia = '';
          });
        },
      );
    } catch (error) {
      setState(() {
        estadoLidar = 'No se pudo conectar al RPLIDAR';

        advertenciaObjeto = false;
      });
    }
  }

  // Recibe la nueva posición del joystick y determina
  // qué comando de movimiento representa.
  void actualizarJoystick(Offset valor) {
    joystickActivo = true;

    final double nuevoX = valor.dx.clamp(-1.0, 1.0).toDouble();

    final double nuevoY = valor.dy.clamp(-1.0, 1.0).toDouble();

    // Evita que pequeñas variaciones cerca del centro
    // sean interpretadas como movimientos.
    const double zonaMuerta = 0.12;

    final bool izquierda = nuevoX < -zonaMuerta;

    final bool derecha = nuevoX > zonaMuerta;

    final bool adelante = nuevoY > zonaMuerta;

    final bool atras = nuevoY < -zonaMuerta;

    setState(() {
      joystickX = nuevoX;
      joystickY = nuevoY;

      intensidad = math.max(nuevoX.abs(), nuevoY.abs());

      // Determina el comando según la dirección del joystick.
      if (izquierda && adelante) {
        movimientoActual = 'avance';
        estadoControl = 'Q · Adelante hacia la izquierda';
      } else if (derecha && adelante) {
        movimientoActual = 'avance';
        estadoControl = 'E · Adelante hacia la derecha';
      } else if (izquierda && atras) {
        movimientoActual = 'retroceso';
        estadoControl = 'Z · Atrás hacia la izquierda';
      } else if (derecha && atras) {
        movimientoActual = 'retroceso';
        estadoControl = 'C · Atrás hacia la derecha';
      } else if (adelante) {
        movimientoActual = 'avance';
        estadoControl = 'F · Avanzando';
      } else if (atras) {
        movimientoActual = 'retroceso';
        estadoControl = 'B · Retrocediendo';
      } else if (izquierda) {
        movimientoActual = 'direccion';
        estadoControl = 'L · Dirección hacia la izquierda';
      } else if (derecha) {
        movimientoActual = 'direccion';
        estadoControl = 'R · Dirección hacia la derecha';
      } else {
        movimientoActual = 'stop';
        estadoControl = 'S · Joystick centrado';
      }
    });

    hayComandoPendiente = true;
  }

  // Al soltar el joystick vuelve al centro y envía
  // inmediatamente una orden de detención a la Raspberry.
  void soltarJoystick() {
    joystickActivo = false;

    setState(() {
      joystickX = 0.0;
      joystickY = 0.0;
      intensidad = 0.0;

      movimientoActual = 'stop';
      estadoControl = 'S · Rover detenido';
    });

    hayComandoPendiente = false;

    enviarJoystick(0.0, 0.0, forzarEnvio: true);
  }

  // Envía las coordenadas X e Y del joystick
  // al servidor Flask de la Raspberry Pi.
  Future<void> enviarJoystick(
    double x,
    double y, {
    bool forzarEnvio = false,
  }) async {
    // Evita enviar varias solicitudes HTTP simultáneamente.
    if (envioEnCurso && !forzarEnvio) {
      hayComandoPendiente = true;
      return;
    }

    final int secuenciaEnviada = ++secuenciaControl;

    if (!forzarEnvio) {
      envioEnCurso = true;
    }

    try {
      final http.Response respuesta = await http
          .post(
            Uri.parse(urlControlJoystick),
            headers: const {'Content-Type': 'application/json'},
            body: jsonEncode({
              'x': x,
              'y': y,
              'seq': secuenciaEnviada,
              'client_id': idCliente,
            }),
          )
          .timeout(const Duration(milliseconds: 900));

      // Ignora respuestas correspondientes a posiciones
      // anteriores del joystick.
      if (secuenciaEnviada != secuenciaControl || !mounted) {
        return;
      }

      if (respuesta.statusCode != 200) {
        setState(() {
          estadoControl =
              'Error del servidor: '
              '${respuesta.statusCode}';
        });
      }
    } on TimeoutException {
      if (secuenciaEnviada == secuenciaControl && mounted) {
        setState(() {
          estadoControl = 'La Raspberry no respondió';
        });
      }
    } catch (error) {
      if (secuenciaEnviada == secuenciaControl && mounted) {
        setState(() {
          estadoControl = 'Error de conexión con la Raspberry';
        });
      }
    } finally {
      if (!forzarEnvio) {
        envioEnCurso = false;
      }
    }
  }

  // Envía el comando X al servidor Flask para activar la bomba.
  // En esta versión la Raspberry mantiene la bomba activa durante 400 ms.
  Future<void> activarBomba() async {
    if (enviandoBomba) return;

    setState(() {
      enviandoBomba = true;
      estadoBomba = 'Activando bomba...';
    });

    try {
      // Primero intenta activar la bomba mediante
      // la ruta general de comandos del Rover.
      http.Response respuesta = await http
          .post(
            Uri.parse(urlControlComando),
            headers: const {'Content-Type': 'application/json'},
            body: jsonEncode({'command': 'X'}),
          )
          .timeout(const Duration(seconds: 2));

      // Si la ruta general falla, intenta utilizar
      // directamente la ruta específica de la bomba.
      if (respuesta.statusCode < 200 || respuesta.statusCode >= 300) {
        respuesta = await http
            .post(Uri.parse(urlBombaOn))
            .timeout(const Duration(seconds: 2));
      }

      if (!mounted) return;

      if (respuesta.statusCode >= 200 && respuesta.statusCode < 300) {
        setState(() {
          estadoBomba = 'Bomba activada durante 400 ms';
        });

        // Después de mostrar el estado durante un momento,
        // vuelve a indicar que la bomba está lista.
        Future<void>.delayed(const Duration(milliseconds: 650), () {
          if (!mounted) return;

          setState(() {
            estadoBomba = 'Bomba lista';
          });
        });
      } else {
        setState(() {
          estadoBomba =
              'Error de bomba: '
              '${respuesta.statusCode}';
        });
      }
    } on TimeoutException {
      if (!mounted) return;

      setState(() {
        estadoBomba = 'La Raspberry no respondió';
      });
    } catch (error) {
      if (!mounted) return;

      setState(() {
        estadoBomba = 'Error al activar la bomba';
      });
    } finally {
      if (mounted) {
        setState(() {
          enviandoBomba = false;
        });
      }
    }
  }

  // Cierra las conexiones y detiene el Rover
  // cuando se sale de la pantalla.
  @override
  void dispose() {
    enviarJoystick(0.0, 0.0, forzarEnvio: true);

    timerEnvioJoystick?.cancel();
    subscription?.cancel();
    channel?.sink.close();

    super.dispose();
  }

  // Selecciona un color según el movimiento actual del Rover.
  Color colorEstadoControl() {
    if (movimientoActual == 'avance') {
      return Colors.greenAccent;
    }

    if (movimientoActual == 'retroceso') {
      return Colors.orangeAccent;
    }

    if (movimientoActual == 'direccion') {
      return Colors.cyanAccent;
    }

    return Colors.white70;
  }

  // Genera las pequeñas etiquetas utilizadas para identificar
  // cámara, RPLIDAR y joystick.
  Widget etiquetaPanel(String texto, {IconData? icono}) {
    return Container(
      padding: const EdgeInsets.symmetric(horizontal: 10, vertical: 6),
      decoration: BoxDecoration(
        color: Colors.black.withOpacity(0.74),
        borderRadius: BorderRadius.circular(8),
        border: Border.all(color: Colors.white24),
      ),
      child: Row(
        mainAxisSize: MainAxisSize.min,
        children: [
          if (icono != null) ...[
            Icon(icono, size: 17, color: Colors.white70),
            const SizedBox(width: 6),
          ],
          Text(
            texto,
            style: const TextStyle(
              color: Colors.white,
              fontSize: 13,
              fontWeight: FontWeight.bold,
              letterSpacing: 0.8,
            ),
          ),
        ],
      ),
    );
  }

  // Muestra la transmisión MJPEG enviada desde
  // la cámara conectada a la Raspberry Pi.
  Widget bloqueCamaraHorizontal() {
    return Container(
      decoration: BoxDecoration(
        color: Colors.black,
        border: Border.all(color: Colors.white12),
        borderRadius: BorderRadius.circular(12),
      ),
      clipBehavior: Clip.antiAlias,
      child: Stack(
        fit: StackFit.expand,
        children: [
          MjpegView(
            uri: urlCamara,
            fit: BoxFit.contain,

            // Se muestra mientras se establece la conexión.
            loadingWidget: (context) {
              return const Center(
                child: Column(
                  mainAxisSize: MainAxisSize.min,
                  children: [
                    CircularProgressIndicator(),
                    SizedBox(height: 10),
                    Text(
                      'Conectando con la cámara...',
                      style: TextStyle(color: Colors.white70),
                    ),
                  ],
                ),
              );
            },

            // Se muestra si la transmisión de cámara falla.
            errorWidget: (context) {
              return const Center(
                child: Text(
                  'No se pudo cargar la cámara',
                  style: TextStyle(color: Colors.white70),
                ),
              );
            },

            onError: (error, stackTrace) {
              debugPrint('Error cámara: $error');
            },
          ),

          Positioned(
            top: 10,
            left: 10,
            child: etiquetaPanel('CÁMARA', icono: Icons.videocam_outlined),
          ),
        ],
      ),
    );
  }

  // Muestra gráficamente los puntos enviados por el RPLIDAR
  // y cambia el borde a rojo si existe un obstáculo cercano.
  Widget bloqueLidarHorizontal() {
    final Color borde = advertenciaObjeto ? Colors.redAccent : Colors.white12;

    return AnimatedContainer(
      duration: const Duration(milliseconds: 180),
      decoration: BoxDecoration(
        color: Colors.black,
        border: Border.all(color: borde, width: advertenciaObjeto ? 2.0 : 1.0),
        borderRadius: BorderRadius.circular(12),
      ),
      clipBehavior: Clip.antiAlias,
      child: Stack(
        fit: StackFit.expand,
        children: [
          // Dibuja el radar dentro del panel.
          Padding(
            padding: const EdgeInsets.only(top: 38, bottom: 28),
            child: CustomPaint(
              painter: LidarPainter(
                puntos,
                distanciaAdvertenciaMm: distanciaAdvertenciaMm,
              ),
              child: const SizedBox.expand(),
            ),
          ),

          Positioned(
            top: 10,
            left: 10,
            child: etiquetaPanel('SENSOR RPLIDAR', icono: Icons.radar),
          ),

          // Muestra una advertencia roja cuando
          // un objeto se encuentra a 30 cm o menos.
          if (advertenciaObjeto)
            Positioned(
              top: 9,
              right: 10,
              child: Container(
                padding: const EdgeInsets.symmetric(
                  horizontal: 10,
                  vertical: 6,
                ),
                decoration: BoxDecoration(
                  color: Colors.redAccent.withOpacity(0.90),
                  borderRadius: BorderRadius.circular(8),
                ),
                child: const Row(
                  mainAxisSize: MainAxisSize.min,
                  children: [
                    Icon(
                      Icons.warning_amber_rounded,
                      color: Colors.white,
                      size: 17,
                    ),
                    SizedBox(width: 5),
                    Text(
                      'PRECAUCIÓN',
                      style: TextStyle(
                        fontWeight: FontWeight.bold,
                        fontSize: 12,
                      ),
                    ),
                  ],
                ),
              ),
            ),

          // Muestra la distancia mínima o el estado del sensor.
          Positioned(
            left: 10,
            right: 10,
            bottom: 7,
            child: Text(
              advertenciaObjeto ? mensajeAdvertencia : estadoLidar,
              maxLines: 1,
              overflow: TextOverflow.ellipsis,
              style: TextStyle(
                color: advertenciaObjeto ? Colors.redAccent : Colors.white60,
                fontSize: 11,
                fontWeight: advertenciaObjeto
                    ? FontWeight.bold
                    : FontWeight.normal,
              ),
              textAlign: TextAlign.center,
            ),
          ),
        ],
      ),
    );
  }

  // Botón utilizado para enviar el comando X
  // y activar la bomba desde la aplicación.
  Widget botonBomba() {
    return SizedBox(
      width: double.infinity,
      height: 62,
      child: ElevatedButton(
        onPressed: enviandoBomba ? null : activarBomba,
        style: ElevatedButton.styleFrom(
          backgroundColor: const Color(0xFFB3261E),
          foregroundColor: Colors.white,
          disabledBackgroundColor: const Color(0xFF5C2521),
          shape: RoundedRectangleBorder(
            borderRadius: BorderRadius.circular(14),
            side: BorderSide(
              color: enviandoBomba ? Colors.white24 : Colors.redAccent,
              width: 1.5,
            ),
          ),
          elevation: 5,
        ),
        child: Row(
          mainAxisAlignment: MainAxisAlignment.center,
          children: [
            if (enviandoBomba)
              const SizedBox(
                width: 22,
                height: 22,
                child: CircularProgressIndicator(
                  strokeWidth: 2.5,
                  color: Colors.white,
                ),
              )
            else
              Container(
                width: 34,
                height: 34,
                alignment: Alignment.center,
                decoration: BoxDecoration(
                  color: Colors.white.withOpacity(0.14),
                  shape: BoxShape.circle,
                  border: Border.all(color: Colors.white54),
                ),
                child: const Text(
                  'X',
                  style: TextStyle(fontSize: 21, fontWeight: FontWeight.w900),
                ),
              ),

            const SizedBox(width: 12),

            Flexible(
              child: Text(
                enviandoBomba ? 'ACTIVANDO...' : 'ACTIVAR BOMBA · 400 ms',
                maxLines: 1,
                overflow: TextOverflow.ellipsis,
                style: const TextStyle(
                  fontSize: 15,
                  fontWeight: FontWeight.bold,
                  letterSpacing: 0.4,
                ),
              ),
            ),
          ],
        ),
      ),
    );
  }

  // Panel derecho que contiene el joystick,
  // el estado de movimiento y el botón de la bomba.
  Widget bloqueControlHorizontal() {
    return Container(
      decoration: BoxDecoration(
        color: const Color(0xFF11151B),
        border: Border.all(color: Colors.white12),
        borderRadius: BorderRadius.circular(12),
      ),
      child: Padding(
        padding: const EdgeInsets.fromLTRB(12, 10, 12, 10),
        child: Column(
          children: [
            Row(
              children: [
                etiquetaPanel('JOYSTICK', icono: Icons.control_camera_outlined),

                const Spacer(),

                // Muestra la intensidad del desplazamiento
                // actual del joystick.
                Container(
                  padding: const EdgeInsets.symmetric(
                    horizontal: 10,
                    vertical: 6,
                  ),
                  decoration: BoxDecoration(
                    color: Colors.black.withOpacity(0.45),
                    borderRadius: BorderRadius.circular(8),
                  ),
                  child: Text(
                    'INT '
                    '${(intensidad * 100).round()}%',
                    style: const TextStyle(
                      color: Colors.white70,
                      fontSize: 12,
                      fontWeight: FontWeight.w600,
                    ),
                  ),
                ),
              ],
            ),

            const SizedBox(height: 4),

            // El joystick ocupa todo el espacio disponible
            // en el centro del panel derecho.
            Expanded(
              child: LayoutBuilder(
                builder: (context, constraints) {
                  final double tamanioJoystick = math
                      .min(
                        constraints.maxWidth * 0.82,
                        constraints.maxHeight * 0.92,
                      )
                      .clamp(150.0, 350.0)
                      .toDouble();

                  return Center(
                    child: JoystickControl(
                      size: tamanioJoystick,
                      value: Offset(joystickX, joystickY),
                      onChanged: actualizarJoystick,
                      onReleased: soltarJoystick,
                    ),
                  );
                },
              ),
            ),

            // Indica el comando o movimiento actual.
            Text(
              estadoControl,
              style: TextStyle(
                color: colorEstadoControl(),
                fontSize: 13,
                fontWeight: FontWeight.bold,
              ),
              maxLines: 1,
              overflow: TextOverflow.ellipsis,
              textAlign: TextAlign.center,
            ),

            const SizedBox(height: 8),

            botonBomba(),

            const SizedBox(height: 5),

            Text(
              estadoBomba,
              style: const TextStyle(color: Colors.white60, fontSize: 11),
              maxLines: 1,
              overflow: TextOverflow.ellipsis,
              textAlign: TextAlign.center,
            ),
          ],
        ),
      ),
    );
  }

  // Distribución principal en orientación horizontal.
  //
  // Izquierda:
  // cámara arriba y RPLIDAR abajo.
  //
  // Derecha:
  // joystick, estado de movimiento y bomba.
  @override
  Widget build(BuildContext context) {
    return Scaffold(
      body: SafeArea(
        child: Padding(
          padding: const EdgeInsets.all(4),
          child: Row(
            children: [
              // Panel izquierdo.
              Expanded(
                flex: 3,
                child: Column(
                  children: [
                    Expanded(flex: 3, child: bloqueCamaraHorizontal()),

                    const SizedBox(height: 4),

                    Expanded(flex: 2, child: bloqueLidarHorizontal()),
                  ],
                ),
              ),

              const SizedBox(width: 4),

              // Panel derecho de control.
              Expanded(flex: 2, child: bloqueControlHorizontal()),
            ],
          ),
        ),
      ),
    );
  }
}

// Joystick táctil utilizado para obtener
// la dirección y magnitud del movimiento.
class JoystickControl extends StatelessWidget {
  const JoystickControl({
    required this.size,
    required this.value,
    required this.onChanged,
    required this.onReleased,
    super.key,
  });

  final double size;
  final Offset value;

  final ValueChanged<Offset> onChanged;
  final VoidCallback onReleased;

  // Tamaño de la perilla central.
  double get radioPerilla => size * 0.145;

  // Distancia máxima que puede desplazarse la perilla.
  double get recorridoMaximo => size / 2 - radioPerilla - size * 0.045;

  // Convierte la posición del dedo dentro del joystick
  // a valores normalizados entre -1 y 1.
  void procesarPosicion(Offset posicionLocal) {
    final Offset centro = Offset(size / 2, size / 2);

    Offset desplazamiento = posicionLocal - centro;

    // Evita que la perilla salga fuera del círculo.
    if (desplazamiento.distance > recorridoMaximo) {
      desplazamiento = Offset.fromDirection(
        desplazamiento.direction,
        recorridoMaximo,
      );
    }

    final double x = desplazamiento.dx / recorridoMaximo;

    // Flutter aumenta Y hacia abajo.
    // Se invierte para que arriba sea positivo
    // y abajo sea negativo.
    final double y = -desplazamiento.dy / recorridoMaximo;

    onChanged(Offset(x, y));
  }

  // Construye gráficamente el joystick
  // y detecta los movimientos del dedo.
  @override
  Widget build(BuildContext context) {
    final Offset desplazamientoVisual = Offset(
      value.dx * recorridoMaximo,
      -value.dy * recorridoMaximo,
    );

    return GestureDetector(
      behavior: HitTestBehavior.opaque,

      // Detecta cuando se toca inicialmente el joystick.
      onPanDown: (details) {
        procesarPosicion(details.localPosition);
      },

      // Actualiza continuamente la posición mientras
      // el dedo se mueve.
      onPanUpdate: (details) {
        procesarPosicion(details.localPosition);
      },

      // Al retirar el dedo se envía el comando de parada.
      onPanEnd: (_) {
        onReleased();
      },

      onPanCancel: onReleased,

      child: SizedBox(
        width: size,
        height: size,
        child: Stack(
          alignment: Alignment.center,
          children: [
            // Círculo exterior.
            Container(
              width: size,
              height: size,
              decoration: BoxDecoration(
                shape: BoxShape.circle,
                color: const Color(0xFF222934),
                border: Border.all(color: Colors.white30, width: 2),
                boxShadow: const [
                  BoxShadow(
                    color: Colors.black54,
                    blurRadius: 18,
                    offset: Offset(0, 8),
                  ),
                ],
              ),
            ),

            // Círculo interior de referencia.
            Container(
              width: size * 0.58,
              height: size * 0.58,
              decoration: BoxDecoration(
                shape: BoxShape.circle,
                border: Border.all(color: Colors.white10),
              ),
            ),

            // Indicador superior.
            Positioned(
              top: size * 0.045,
              child: Icon(
                Icons.keyboard_arrow_up,
                color: Colors.white54,
                size: size * 0.14,
              ),
            ),

            // Indicador inferior.
            Positioned(
              bottom: size * 0.045,
              child: Icon(
                Icons.keyboard_arrow_down,
                color: Colors.white54,
                size: size * 0.14,
              ),
            ),

            // Indicador izquierdo.
            Positioned(
              left: size * 0.045,
              child: Icon(
                Icons.keyboard_arrow_left,
                color: Colors.white54,
                size: size * 0.14,
              ),
            ),

            // Indicador derecho.
            Positioned(
              right: size * 0.045,
              child: Icon(
                Icons.keyboard_arrow_right,
                color: Colors.white54,
                size: size * 0.14,
              ),
            ),

            // Punto que marca el centro.
            Container(
              width: size * 0.028,
              height: size * 0.028,
              decoration: const BoxDecoration(
                shape: BoxShape.circle,
                color: Colors.white24,
              ),
            ),

            // Perilla móvil del joystick.
            Transform.translate(
              offset: desplazamientoVisual,
              child: Container(
                width: radioPerilla * 2,
                height: radioPerilla * 2,
                decoration: BoxDecoration(
                  shape: BoxShape.circle,
                  color: const Color(0xFF596776),
                  border: Border.all(color: Colors.cyanAccent, width: 2),
                  boxShadow: const [
                    BoxShadow(
                      color: Colors.black87,
                      blurRadius: 12,
                      offset: Offset(0, 5),
                    ),
                  ],
                ),
                child: Icon(
                  Icons.control_camera,
                  color: Colors.white,
                  size: radioPerilla * 0.9,
                ),
              ),
            ),
          ],
        ),
      ),
    );
  }
}

// Cada objeto LidarPoint representa una medición
// enviada por el RPLIDAR.
class LidarPoint {
  const LidarPoint({
    required this.angle,
    required this.distance,
    required this.quality,
  });

  // Ángulo de la medición en grados.
  final double angle;

  // Distancia medida en milímetros.
  final double distance;

  // Calidad informada por el sensor.
  final double quality;
}

// Se encarga de dibujar gráficamente
// las mediciones recibidas desde el RPLIDAR.
class LidarPainter extends CustomPainter {
  const LidarPainter(this.puntos, {required this.distanciaAdvertenciaMm});

  final List<LidarPoint> puntos;

  final double distanciaAdvertenciaMm;

  @override
  void paint(Canvas canvas, Size size) {
    // Centro gráfico del radar.
    final Offset centro = Offset(size.width / 2, size.height / 2);

    // Elementos utilizados para dibujar
    // la grilla y los puntos del sensor.
    final Paint paintGrilla = Paint()
      ..color = Colors.grey.withOpacity(0.35)
      ..style = PaintingStyle.stroke
      ..strokeWidth = 1;

    final Paint paintEjes = Paint()
      ..color = Colors.grey.withOpacity(0.60)
      ..strokeWidth = 1;

    final Paint paintPuntosNormales = Paint()
      ..color = Colors.greenAccent
      ..style = PaintingStyle.fill;

    final Paint paintPuntosCercanos = Paint()
      ..color = Colors.redAccent
      ..style = PaintingStyle.fill;

    final Paint paintCentro = Paint()
      ..color = Colors.white
      ..style = PaintingStyle.fill;

    final Paint paintZonaAdvertencia = Paint()
      ..color = Colors.redAccent.withOpacity(0.45)
      ..style = PaintingStyle.stroke
      ..strokeWidth = 1.5;

    // La representación muestra hasta un metro de distancia.
    const double distanciaMaximaMm = 1000.0;

    final double radioMaximo = math.min(size.width, size.height) * 0.44;

    // Convierte milímetros reales a píxeles en pantalla.
    final double escala = radioMaximo / distanciaMaximaMm;

    // Dibuja cuatro círculos de referencia.
    for (int i = 1; i <= 4; i++) {
      canvas.drawCircle(centro, radioMaximo * i / 4, paintGrilla);
    }

    // Dibuja el círculo correspondiente
    // al límite de precaución de 30 cm.
    canvas.drawCircle(
      centro,
      distanciaAdvertenciaMm * escala,
      paintZonaAdvertencia,
    );

    // Dibuja el eje horizontal.
    canvas.drawLine(
      Offset(centro.dx - radioMaximo, centro.dy),
      Offset(centro.dx + radioMaximo, centro.dy),
      paintEjes,
    );

    // Dibuja el eje vertical.
    canvas.drawLine(
      Offset(centro.dx, centro.dy - radioMaximo),
      Offset(centro.dx, centro.dy + radioMaximo),
      paintEjes,
    );

    // Marca la posición del Rover en el centro.
    canvas.drawCircle(centro, 4, paintCentro);

    // Recorre todas las mediciones recibidas
    // y las ubica utilizando su ángulo y distancia.
    for (final LidarPoint punto in puntos) {
      if (punto.distance <= 0 || punto.distance > distanciaMaximaMm) {
        continue;
      }

      // Convierte el ángulo de grados a radianes.
      final double rad = punto.angle * math.pi / 180.0;

      // Obtiene las coordenadas cartesianas del punto.
      final double x = math.cos(rad) * punto.distance * escala;

      final double y = math.sin(rad) * punto.distance * escala;

      final Offset puntoPantalla = Offset(centro.dx + x, centro.dy - y);

      // Los obstáculos dentro de los 30 cm
      // se dibujan en rojo.
      final bool cercano = punto.distance <= distanciaAdvertenciaMm;

      canvas.drawCircle(
        puntoPantalla,
        cercano ? 2.8 : 2.2,
        cercano ? paintPuntosCercanos : paintPuntosNormales,
      );
    }
  }

  // Obliga a redibujar el radar cuando cambian
  // los puntos recibidos o la distancia de advertencia.
  @override
  bool shouldRepaint(covariant LidarPainter oldDelegate) {
    return oldDelegate.puntos != puntos ||
        oldDelegate.distanciaAdvertenciaMm != distanciaAdvertenciaMm;
  }
}
