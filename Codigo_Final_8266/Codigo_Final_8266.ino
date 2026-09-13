#include <ESP8266WiFi.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>


// CONFIGURACIÓN WIFI
const char* NOMBRE_WIFI = "leo";
const char* CLAVE_WIFI = "12345678";

/*
  IP del celular donde está funcionando ArduinoCar.
  Cambiala solamente si el celular cambia de IP.
*/
IPAddress IP_APLICACION(192, 168, 137, 186);

const uint16_t PUERTO_APLICACION = 50123;

WiFiClient cliente;

unsigned long ultimoIntentoConexion = 0;
const unsigned long INTERVALO_RECONEXION_MS = 2000;


// PCA9685

Adafruit_PWMServoDriver pca9685(0x40);

const int PIN_SDA = D2;  // GPIO4
const int PIN_SCL = D1;  // GPIO5

const int FRECUENCIA_SERVOS = 50;


// Valores de pulso utilizados para los servos.
const int SERVOMIN = 205;
const int SERVOMAX = 410;

// Canales del PCA9685.
const int SERVO_DER_DEL = 3;
const int SERVO_DER_TRAS = 7;
const int SERVO_IZQ_DEL = 2;
const int SERVO_IZQ_TRAS = 12;

// CENTROS DE LOS SERVOS
const int CENTRO_DER_DEL = 75;
const int CENTRO_DER_TRAS = 35;
const int CENTRO_IZQ_DEL = 125;
const int CENTRO_IZQ_TRAS = 70;

// Giro hacia la izquierda.
const int DESVIO_MAX_IZQUIERDA = 35;

// Giro individual hacia la derecha.
const int GIRO_DERECHA_DER_DEL = 55;
const int GIRO_DERECHA_IZQ_DEL = 50;
const int GIRO_DERECHA_DER_TRAS = 23;
const int GIRO_DERECHA_IZQ_TRAS = 55;

// Movimiento progresivo.
const int PASO_DIRECCION = 1;
const unsigned long INTERVALO_DIRECCION_MS = 30;

/*
  -35 = izquierda
     0 = centro
   +55 = derecha
*/
int desvioActual = 0;
int desvioObjetivo = 0;

unsigned long ultimoPasoDireccion = 0;


// PUENTE H Y MOTORES DC
const int IN1 = D5;  // GPIO14
const int IN2 = D6;  // GPIO12
const int IN3 = D7;  // GPIO13
const int IN4 = D8;  // GPIO15

/*
  ENA y ENB quedan con los jumpers colocados.

  Por eso no se conectan a la ESP8266 y los motores trabajan
  a la velocidad permitida por la alimentación del puente H.
*/


// RELÉ Y BOMBA
const int RELE_BOMBA = D0;  // GPIO16

/*
  Para el módulo de relé activo en LOW:

  LOW  = relé activado
  HIGH = relé apagado
*/
const int RELE_ENCENDIDO = LOW;
const int RELE_APAGADO = HIGH;

bool bombaActiva = false;


// ESTADO DEL MOVIMIENTO
enum EstadoMovimiento {
  DETENIDO,
  ADELANTE,
  ATRAS
};

EstadoMovimiento estadoMovimiento = DETENIDO;


// PROTOTIPOS DE FUNCIONES
void conectarWifi();
void conectarAplicacion();
void detenerPorSeguridad();

void procesarComando(char comando);

int anguloAPwm(int angulo);
void moverServo(int canal, int angulo);

void aplicarDireccionActual();
void actualizarDireccionSuave();
void centrarServosAlInicio();

void avanzar();
void retroceder();
void pararMotores();

void activarBomba();
void apagarBomba();


// SETUP
void setup() {
  Serial.begin(115200);
  delay(300);

  Serial.println();
  Serial.println("Iniciando Robert con ESP8266");

  
  // Puente H
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pararMotores();

 
  // Relé
  pinMode(RELE_BOMBA, OUTPUT);

  // La bomba comienza apagada.
  digitalWrite(RELE_BOMBA, RELE_APAGADO);
  bombaActiva = false;

  
  // PCA9685
  Wire.begin(PIN_SDA, PIN_SCL);

  pca9685.begin();
  pca9685.setPWMFreq(FRECUENCIA_SERVOS);

  delay(500);

  centrarServosAlInicio();

  desvioActual = 0;
  desvioObjetivo = 0;

  
  // WiFi
  conectarWifi();

  Serial.println();
  Serial.println("Comandos disponibles:");
  Serial.println("F = adelante recto");
  Serial.println("B = atras recto");
  Serial.println("L = servos izquierda");
  Serial.println("R = servos derecha");
  Serial.println("Q = adelante izquierda");
  Serial.println("E = adelante derecha");
  Serial.println("Z = atras izquierda");
  Serial.println("C = atras derecha");
  Serial.println("S = detener y centrar");
  Serial.println("X = encender bomba");
  Serial.println("x = apagar bomba");
}


// LOOP
void loop() {
  /*
    Si se perdió el WiFi, detiene todo y vuelve a conectarse.
  */
  if (WiFi.status() != WL_CONNECTED) {
    detenerPorSeguridad();
    conectarWifi();
  }

  /*
    Si no está conectado a ArduinoCar, intenta reconectarse.
  */
  if (!cliente.connected()) {
    detenerPorSeguridad();
    conectarAplicacion();
  }

  /*
    Lee todos los comandos recibidos desde ArduinoCar.
  */
  while (cliente.connected() && cliente.available() > 0) {
    char comando = cliente.read();

    if (
      comando == '\n' ||
      comando == '\r' ||
      comando == ' '
    ) {
      continue;
    }

    procesarComando(comando);
  }

  actualizarDireccionSuave();

  delay(2);
}


// CONEXIÓN AL WIFI
void conectarWifi() {
  if (WiFi.status() == WL_CONNECTED) {
    return;
  }

  Serial.print("Conectando a WiFi: ");
  Serial.println(NOMBRE_WIFI);

  WiFi.mode(WIFI_STA);
  WiFi.persistent(false);
  WiFi.setAutoReconnect(true);

  WiFi.begin(
    NOMBRE_WIFI,
    CLAVE_WIFI
  );

  unsigned long inicio = millis();

  while (
    WiFi.status() != WL_CONNECTED &&
    millis() - inicio < 15000
  ) {
    delay(300);
    Serial.print(".");
  }

  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("WiFi conectado");

    Serial.print("IP de la ESP8266: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("No se pudo conectar al WiFi");
  }
}


// CONEXIÓN CON LA APLICACIÓN
void conectarAplicacion() {
  if (WiFi.status() != WL_CONNECTED) {
    return;
  }

  unsigned long ahora = millis();

  if (
    ahora - ultimoIntentoConexion <
    INTERVALO_RECONEXION_MS
  ) {
    return;
  }

  ultimoIntentoConexion = ahora;

  Serial.print("Conectando a ArduinoCar en ");
  Serial.print(IP_APLICACION);
  Serial.print(":");
  Serial.println(PUERTO_APLICACION);

  cliente.stop();

  if (
    cliente.connect(
      IP_APLICACION,
      PUERTO_APLICACION
    )
  ) {
    cliente.setNoDelay(true);

    Serial.println("Conectado a ArduinoCar");
  } else {
    Serial.println("No se pudo conectar a ArduinoCar");
  }
}


// SEGURIDAD AL DESCONECTAR
void detenerPorSeguridad() {
  pararMotores();

  desvioObjetivo = 0;

  apagarBomba();
}


// PROCESAMIENTO DE COMANDOS
void procesarComando(char comando) {
  /*
    Convierte cualquier letra minúscula en mayúscula.
    Así funcionan tanto X/Y como x/y.
  */
  if (
    comando >= 'a' &&
    comando <= 'z'
  ) {
    comando = comando - ('a' - 'A');
  }

  Serial.print("Comando recibido: ");
  Serial.println(comando);

  switch (comando) {

    case 'F':
      desvioObjetivo = 0;
      avanzar();
      break;

    case 'B':
      desvioObjetivo = 0;
      retroceder();
      break;

    case 'L':
      pararMotores();
      desvioObjetivo = -DESVIO_MAX_IZQUIERDA;
      break;

    case 'R':
      pararMotores();
      desvioObjetivo = GIRO_DERECHA_DER_DEL;
      break;

    case 'Q':
      desvioObjetivo = -DESVIO_MAX_IZQUIERDA;
      avanzar();
      break;

    case 'E':
      desvioObjetivo = GIRO_DERECHA_DER_DEL;
      avanzar();
      break;

    case 'Z':
      desvioObjetivo = -DESVIO_MAX_IZQUIERDA;
      retroceder();
      break;

    case 'C':
      desvioObjetivo = GIRO_DERECHA_DER_DEL;
      retroceder();
      break;

    case 'S':
      pararMotores();
      desvioObjetivo = 0;

      // También apaga la bomba por seguridad.
      apagarBomba();
      break;

    case 'X':
      // Enciende la bomba y queda encendida.
      activarBomba();
      break;

    case 'Y':
      // Apaga la bomba al soltar el botón.
      apagarBomba();
      break;

    default:
      Serial.println("Comando desconocido");
      break;
  }
}


// CONTROL DE LA BOMBA
void activarBomba() {
  digitalWrite(
    RELE_BOMBA,
    RELE_ENCENDIDO
  );

  if (!bombaActiva) {
    bombaActiva = true;

    Serial.println("Bomba: ENCENDIDA");
  }
}

void apagarBomba() {
  digitalWrite(
    RELE_BOMBA,
    RELE_APAGADO
  );

  if (bombaActiva) {
    bombaActiva = false;

    Serial.println("Bomba: APAGADA");
  }
}


// MOTORES DC
void avanzar() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  estadoMovimiento = ADELANTE;

  Serial.println("Motores: ADELANTE");
}

void retroceder() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  estadoMovimiento = ATRAS;

  Serial.println("Motores: ATRAS");
}

void pararMotores() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  estadoMovimiento = DETENIDO;
}


// CONVERSIÓN ÁNGULO/PWM

int anguloAPwm(int angulo) {
  angulo = constrain(
    angulo,
    0,
    180
  );

  return map(
    angulo,
    0,
    180,
    SERVOMIN,
    SERVOMAX
  );
}

void moverServo(int canal, int angulo) {
  angulo = constrain(
    angulo,
    0,
    180
  );

  int pulso = anguloAPwm(angulo);

  pca9685.setPWM(
    canal,
    0,
    pulso
  );
}


// DIRECCIÓN DE LOS SERVOS
void aplicarDireccionActual() {
  int anguloDerDel;
  int anguloIzqDel;
  int anguloDerTras;
  int anguloIzqTras;

  if (desvioActual < 0) {
    /*
      Giro hacia la izquierda.
    */
    int giroIzquierda = -desvioActual;

    anguloDerDel =
      CENTRO_DER_DEL - giroIzquierda;

    anguloIzqDel =
      CENTRO_IZQ_DEL - giroIzquierda;

    anguloDerTras =
      CENTRO_DER_TRAS + giroIzquierda;

    anguloIzqTras =
      CENTRO_IZQ_TRAS + giroIzquierda;
  } else {
    /*
      Giro hacia la derecha.

      Cada servo tiene su límite propio para evitar
      posiciones donde pueda vibrar o trabarse.
    */
    double porcentajeDerecha =
      desvioActual /
      (double)GIRO_DERECHA_DER_DEL;

    porcentajeDerecha = constrain(
      porcentajeDerecha,
      0.0,
      1.0
    );

    anguloDerDel =
      CENTRO_DER_DEL +
      round(
        GIRO_DERECHA_DER_DEL *
        porcentajeDerecha
      );

    anguloIzqDel =
      CENTRO_IZQ_DEL +
      round(
        GIRO_DERECHA_IZQ_DEL *
        porcentajeDerecha
      );

    anguloDerTras =
      CENTRO_DER_TRAS -
      round(
        GIRO_DERECHA_DER_TRAS *
        porcentajeDerecha
      );

    anguloIzqTras =
      CENTRO_IZQ_TRAS -
      round(
        GIRO_DERECHA_IZQ_TRAS *
        porcentajeDerecha
      );
  }

  moverServo(
    SERVO_DER_DEL,
    anguloDerDel
  );

  moverServo(
    SERVO_IZQ_DEL,
    anguloIzqDel
  );

  moverServo(
    SERVO_DER_TRAS,
    anguloDerTras
  );

  moverServo(
    SERVO_IZQ_TRAS,
    anguloIzqTras
  );
}

// MOVIMIENTO PROGRESIVO
void actualizarDireccionSuave() {
  unsigned long ahora = millis();

  if (
    ahora - ultimoPasoDireccion <
    INTERVALO_DIRECCION_MS
  ) {
    return;
  }

  ultimoPasoDireccion = ahora;

  if (desvioActual < desvioObjetivo) {
    desvioActual += PASO_DIRECCION;

    if (desvioActual > desvioObjetivo) {
      desvioActual = desvioObjetivo;
    }

    aplicarDireccionActual();
  } else if (desvioActual > desvioObjetivo) {
    desvioActual -= PASO_DIRECCION;

    if (desvioActual < desvioObjetivo) {
      desvioActual = desvioObjetivo;
    }

    aplicarDireccionActual();
  }
}


// CENTRADO INICIALO9
void centrarServosAlInicio() {
  moverServo(
    SERVO_DER_DEL,
    CENTRO_DER_DEL
  );

  delay(150);

  moverServo(
    SERVO_IZQ_DEL,
    CENTRO_IZQ_DEL
  );

  delay(150);

  moverServo(
    SERVO_DER_TRAS,
    CENTRO_DER_TRAS
  );

  delay(150);

  moverServo(
    SERVO_IZQ_TRAS,
    CENTRO_IZQ_TRAS
  );

  delay(150);
}