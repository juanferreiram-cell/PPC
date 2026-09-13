// Inclusion de librerias
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// Nombre Bluetooth que aparece en el celular
const char* NOMBRE_BLE = "ROBERT_ESP32";

// UUID del servicio BLE UART
#define SERVICE_UUID "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"

// RX: el celular envía comandos hacia la ESP32
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"

// TX: la ESP32 puede enviar información al celular
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

BLEServer* servidorBLE = nullptr;
BLECharacteristic* caracteristicaTX = nullptr;

bool dispositivoConectado = false;


// PCA9685
Adafruit_PWMServoDriver pca9685(0x40);

// Pines I2C de la ESP32
const int PIN_SDA = 21;
const int PIN_SCL = 22;

// Frecuencia utilizada por los servomotores
const int FRECUENCIA_SERVOS = 50;

// Valores PWM mínimo y máximo
const int SERVOMIN = 205;
const int SERVOMAX = 410;


// Canales de los servomotores en el PCA9685
const int SERVO_DER_DEL = 0;
const int SERVO_DER_TRAS = 7;
const int SERVO_IZQ_DEL = 4;
const int SERVO_IZQ_TRAS = 12;


// Posiciones centrales de cada rueda
const int CENTRO_DER_DEL = 75;
const int CENTRO_DER_TRAS = 35;
const int CENTRO_IZQ_DEL = 125;
const int CENTRO_IZQ_TRAS = 70;


// Máximo giro hacia la izquierda
const int DESVIO_MAX_IZQUIERDA = 35;

// Máximo giro hacia la derecha de cada servo
const int GIRO_DERECHA_DER_DEL = 55;
const int GIRO_DERECHA_IZQ_DEL = 50;
const int GIRO_DERECHA_DER_TRAS = 23;
const int GIRO_DERECHA_IZQ_TRAS = 55;


// Configuración del movimiento suave de dirección
const int PASO_DIRECCION = 1;

const unsigned long INTERVALO_DIRECCION_MS = 30;

int desvioActual = 0;
int desvioObjetivo = 0;

unsigned long ultimoPasoDireccion = 0;


// Pines del puente H
const int IN1 = 27;
const int IN2 = 26;

const int IN3 = 25;
const int IN4 = 33;

// Pines Enable del puente H
const int ENA = 32;
const int ENB = 18;


// Pin del relé de la bomba
const int RELE_BOMBA = 19;

// El módulo de relé utilizado es activo en LOW
const int RELE_ENCENDIDO = LOW;
const int RELE_APAGADO = HIGH;

bool bombaActiva = false;


// Estado actual de los motores
enum EstadoMovimiento {
  DETENIDO,
  ADELANTE,
  ATRAS
};

EstadoMovimiento estadoMovimiento = DETENIDO;


// Prototipos de funciones
void procesarComando(char comando);

void avanzar();
void retroceder();
void pararMotores();

void activarBomba();
void apagarBomba();

void detenerPorSeguridad();

int anguloAPwm(int angulo);

void moverServo(int canal, int angulo);

void aplicarDireccionActual();

void actualizarDireccionSuave();

void centrarServosAlInicio();

void enviarMensajeBLE(String mensaje);


// Detecta cuando el celular se conecta o desconecta
class CallbacksServidor : public BLEServerCallbacks {

  void onConnect(BLEServer* servidor) override {

    dispositivoConectado = true;

    Serial.println();
    Serial.println("Celular conectado por BLE");

    enviarMensajeBLE("ROBERT conectado");
  }

  void onDisconnect(BLEServer* servidor) override {

    dispositivoConectado = false;

    Serial.println();
    Serial.println("BLE desconectado");
    Serial.println("Deteniendo Robert por seguridad");

    // Si se pierde Bluetooth:
    // detiene motores,
    // centra dirección
    // y apaga la bomba
    detenerPorSeguridad();

    delay(300);

    // Vuelve a anunciar el dispositivo para permitir
    // que el celular pueda conectarse nuevamente
    BLEDevice::startAdvertising();

    Serial.println("Esperando nueva conexion BLE...");
  }
};


// Recibe los comandos enviados desde el celular
class CallbacksRX : public BLECharacteristicCallbacks {

  void onWrite(BLECharacteristic* caracteristica) override {

    String valor = caracteristica->getValue().c_str();

    if (valor.length() == 0) {
      return;
    }

    // Procesa todos los caracteres recibidos
    for (int i = 0; i < valor.length(); i++) {

      char comando = valor.charAt(i);

      // Ignora saltos de línea y espacios
      if (
        comando == '\n' ||
        comando == '\r' ||
        comando == ' '
      ) {
        continue;
      }

      procesarComando(comando);
    }
  }
};


void setup() {

  Serial.begin(115200);

  delay(500);

  Serial.println();
  Serial.println("Iniciando Robert con ESP32 BLE");


  // Configuración de los pines del puente H
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  // Los motores trabajan a velocidad completa.
  // Es equivalente a utilizar los jumpers ENA y ENB.
  digitalWrite(ENA, HIGH);
  digitalWrite(ENB, HIGH);

  pararMotores();


  // Configuración del relé
  pinMode(RELE_BOMBA, OUTPUT);

  // La bomba comienza apagada
  digitalWrite(RELE_BOMBA, RELE_APAGADO);

  bombaActiva = false;


  // Inicialización de I2C
  Wire.begin(PIN_SDA, PIN_SCL);

  // Inicialización del PCA9685
  pca9685.begin();

  pca9685.setPWMFreq(FRECUENCIA_SERVOS);

  delay(500);

  // Coloca las cuatro ruedas en sus posiciones centrales
  centrarServosAlInicio();

  desvioActual = 0;
  desvioObjetivo = 0;


  // Inicialización de Bluetooth BLE
  BLEDevice::init(NOMBRE_BLE);

  servidorBLE = BLEDevice::createServer();

  servidorBLE->setCallbacks(
    new CallbacksServidor()
  );


  // Creación del servicio BLE
  BLEService* servicio =
    servidorBLE->createService(
      SERVICE_UUID
    );


  // Característica TX para enviar información al celular
  caracteristicaTX =
    servicio->createCharacteristic(
      CHARACTERISTIC_UUID_TX,
      BLECharacteristic::PROPERTY_NOTIFY
    );

  caracteristicaTX->addDescriptor(
    new BLE2902()
  );


  // Característica RX para recibir comandos desde el celular
  BLECharacteristic* caracteristicaRX =
    servicio->createCharacteristic(
      CHARACTERISTIC_UUID_RX,
      BLECharacteristic::PROPERTY_WRITE
    );

  caracteristicaRX->setCallbacks(
    new CallbacksRX()
  );


  // Inicia el servicio BLE
  servicio->start();


  // Configuración del advertising BLE
  BLEAdvertising* advertising =
    BLEDevice::getAdvertising();

  advertising->addServiceUUID(
    SERVICE_UUID
  );

  advertising->setScanResponse(true);

  BLEDevice::startAdvertising();


  Serial.println();
  Serial.println("BLE listo");

  Serial.print("Nombre Bluetooth: ");
  Serial.println(NOMBRE_BLE);

  Serial.println();
  Serial.println("Comandos disponibles:");

  Serial.println("F = avanzar");
  Serial.println("B = retroceder");

  Serial.println("L = izquierda");
  Serial.println("R = derecha");

  Serial.println("Q = adelante izquierda");
  Serial.println("E = adelante derecha");

  Serial.println("Z = atras izquierda");
  Serial.println("C = atras derecha");

  Serial.println("S = detener");

  Serial.println("X = encender bomba");
  Serial.println("Y = apagar bomba");

  Serial.println();
  Serial.println("Esperando conexion BLE...");
}


void loop() {

  // El Bluetooth funciona mediante callbacks.
  // En el loop solamente se actualiza progresivamente
  // la posición de los servomotores.

  actualizarDireccionSuave();

  delay(2);
}


// Procesa los comandos enviados por Bluetooth
void procesarComando(char comando) {

  // Convierte letras minúsculas en mayúsculas
  if (
    comando >= 'a' &&
    comando <= 'z'
  ) {

    comando =
      comando - ('a' - 'A');
  }

  Serial.print("Comando recibido: ");
  Serial.println(comando);


  switch (comando) {

    // Avanzar recto
    case 'F':

      desvioObjetivo = 0;

      avanzar();

      enviarMensajeBLE("F");

      break;


    // Retroceder recto
    case 'B':

      desvioObjetivo = 0;

      retroceder();

      enviarMensajeBLE("B");

      break;


    // Girar solamente las ruedas hacia la izquierda
    case 'L':

      pararMotores();

      desvioObjetivo =
        -DESVIO_MAX_IZQUIERDA;

      enviarMensajeBLE("L");

      break;


    // Girar solamente las ruedas hacia la derecha
    case 'R':

      pararMotores();

      desvioObjetivo =
        GIRO_DERECHA_DER_DEL;

      enviarMensajeBLE("R");

      break;


    // Avanzar hacia la izquierda
    case 'Q':

      desvioObjetivo =
        -DESVIO_MAX_IZQUIERDA;

      avanzar();

      enviarMensajeBLE("Q");

      break;


    // Avanzar hacia la derecha
    case 'E':

      desvioObjetivo =
        GIRO_DERECHA_DER_DEL;

      avanzar();

      enviarMensajeBLE("E");

      break;


    // Retroceder hacia la izquierda
    case 'Z':

      desvioObjetivo =
        -DESVIO_MAX_IZQUIERDA;

      retroceder();

      enviarMensajeBLE("Z");

      break;


    // Retroceder hacia la derecha
    case 'C':

      desvioObjetivo =
        GIRO_DERECHA_DER_DEL;

      retroceder();

      enviarMensajeBLE("C");

      break;


    // Detener motores y centrar ruedas
    case 'S':

      pararMotores();

      desvioObjetivo = 0;

      // También apaga la bomba por seguridad
      apagarBomba();

      enviarMensajeBLE("S");

      break;


    // Encender bomba
    case 'X':

      activarBomba();

      enviarMensajeBLE("BOMBA_ON");

      break;


    // Apagar bomba
    case 'Y':

      apagarBomba();

      enviarMensajeBLE("BOMBA_OFF");

      break;


    default:

      Serial.println("Comando desconocido");

      break;
  }
}


// Hace avanzar los dos motores
void avanzar() {

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  estadoMovimiento = ADELANTE;

  Serial.println("Motores: ADELANTE");
}


// Hace retroceder los dos motores
void retroceder() {

  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  estadoMovimiento = ATRAS;

  Serial.println("Motores: ATRAS");
}


// Detiene los dos motores
void pararMotores() {

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);

  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  estadoMovimiento = DETENIDO;
}


// Enciende la bomba
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


// Apaga la bomba
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


// Detiene el Rover cuando se pierde la conexión BLE
void detenerPorSeguridad() {

  pararMotores();

  desvioObjetivo = 0;

  apagarBomba();

  Serial.println(
    "Sistema detenido por seguridad"
  );
}


// Convierte el ángulo de 0-180 grados al valor PWM
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


// Mueve un servo conectado al PCA9685
void moverServo(int canal, int angulo) {

  angulo = constrain(
    angulo,
    0,
    180
  );

  int pulso =
    anguloAPwm(
      angulo
    );

  pca9685.setPWM(
    canal,
    0,
    pulso
  );
}


// Calcula la posición individual de los cuatro servos
void aplicarDireccionActual() {

  int anguloDerDel;
  int anguloIzqDel;

  int anguloDerTras;
  int anguloIzqTras;


  // Giro hacia la izquierda
  if (desvioActual < 0) {

    int giroIzquierda =
      -desvioActual;

    anguloDerDel =
      CENTRO_DER_DEL -
      giroIzquierda;

    anguloIzqDel =
      CENTRO_IZQ_DEL -
      giroIzquierda;

    anguloDerTras =
      CENTRO_DER_TRAS +
      giroIzquierda;

    anguloIzqTras =
      CENTRO_IZQ_TRAS +
      giroIzquierda;
  }

  // Giro hacia la derecha
  else {

    double porcentajeDerecha =
      desvioActual /
      (double)GIRO_DERECHA_DER_DEL;

    porcentajeDerecha =
      constrain(
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


  // Envía las posiciones calculadas al PCA9685
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


// Mueve progresivamente los servos hasta llegar
// al ángulo solicitado
void actualizarDireccionSuave() {

  unsigned long ahora =
    millis();

  if (
    ahora - ultimoPasoDireccion <
    INTERVALO_DIRECCION_MS
  ) {
    return;
  }

  ultimoPasoDireccion = ahora;


  // Movimiento hacia valores mayores
  if (
    desvioActual <
    desvioObjetivo
  ) {

    desvioActual += PASO_DIRECCION;

    if (
      desvioActual >
      desvioObjetivo
    ) {

      desvioActual =
        desvioObjetivo;
    }

    aplicarDireccionActual();
  }


  // Movimiento hacia valores menores
  else if (
    desvioActual >
    desvioObjetivo
  ) {

    desvioActual -= PASO_DIRECCION;

    if (
      desvioActual <
      desvioObjetivo
    ) {

      desvioActual =
        desvioObjetivo;
    }

    aplicarDireccionActual();
  }
}


// Coloca las cuatro ruedas en sus posiciones centrales
// cuando se enciende la ESP32
void centrarServosAlInicio() {

  Serial.println("Centrando servos...");

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

  Serial.println("Servos centrados");
}


// Envía información hacia el celular por BLE
void enviarMensajeBLE(String mensaje) {

  if (
    !dispositivoConectado ||
    caracteristicaTX == nullptr
  ) {
    return;
  }

  caracteristicaTX->setValue(
    mensaje.c_str()
  );

  caracteristicaTX->notify();
}