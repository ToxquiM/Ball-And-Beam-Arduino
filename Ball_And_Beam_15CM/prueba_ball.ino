#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <PID_v1.h>
#include <NewPing.h>

// --- Configuración de Componentes ---

// Configuración del Servo y Driver PCA9685
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVOMIN  150 // Pulso mínimo para el servo (ajustar si es necesario)
#define SERVOMAX  600 // Pulso máximo para el servo (ajustar si es necesario)
#define SERVO_FREQ 50 // Frecuencia estándar para servos analógicos

// Configuración del Sensor de Ultrasonido HC-SR04
#define TRIGGER_PIN  8
#define ECHO_PIN     9
#define MAX_DISTANCE 40 // Distancia máxima a medir en cm (tu viga mide 30cm)
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

// --- Configuración del Control PID ---

// Define las variables que usaremos en el PID
double Setpoint, Input, Output;

// ¡IMPORTANTE! Estas son las constantes del PID. Tendrás que "afinarlas".
// Comienza con estos valores y ajústalos poco a poco.
// Kp: Reacción al error actual. Un valor más alto da una reacción más fuerte.
// Ki: Corrige errores acumulados. Ayuda a eliminar el error estacionario.
// Kd: Amortigua la reacción. Previene que el sistema oscile demasiado.
double Kp = 25.0;  // Comienza con este valor
double Ki = 5.0;   // Comienza con este valor
double Kd = 7.0;   // Comienza con este valor

// Crea una instancia del controlador PID
// El PID funcionará en modo DIRECT, lo que significa que un Output positivo
// moverá el servo para aumentar el Input (la distancia medida).
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// --- Variables del Sistema ---
int servoAngleMiddle = 90; // Ángulo central del servo en grados
int servoPulseMiddle;      // El pulso correspondiente al ángulo central

void setup() {
  Serial.begin(9600);
  Serial.println("Iniciando sistema de Bola y Viga...");

  // Inicializar el driver de servos PCA9685
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);

  // Calcular el pulso central para el servo
  servoPulseMiddle = map(servoAngleMiddle, 0, 180, SERVOMIN, SERVOMAX);

  // --- Configuración Inicial del PID ---
  
  // Define el Setpoint (la posición deseada para la bola en cm)
  // Para una viga de 30cm, el centro es 15cm.
  Setpoint = 15.0;

  // Enciende el PID
  myPID.SetMode(AUTOMATIC);
  
  // Limita el rango del Output del PID. Esto es crucial para no mover
  // el servo de forma demasiado brusca. El valor 150 es un rango de pulso.
  // Puedes experimentar con este valor. Un valor más grande = movimientos más amplios.
  myPID.SetOutputLimits(-160, 160); 

  Serial.println("Sistema listo. Coloca la bola en la viga.");
}

void loop() {
  // 1. MEDIR: Obtener la posición de la bola
  // Usamos sonar.ping_cm() que devuelve la distancia en centímetros.
  // Se hace un pequeño filtro promediando varias lecturas para mayor estabilidad.
  delay(29); // Espera 29ms entre pings para evitar interferencias.
  Input = sonar.ping_cm();

  // Si el sensor no detecta nada (devuelve 0), mantenemos la última lectura válida.
  static double lastGoodInput = Setpoint;
  if (Input == 0) {
    Input = lastGoodInput;
  } else {
    lastGoodInput = Input;
  }

  // 2. COMPARAR Y 3. CALCULAR: El PID hace su trabajo
  myPID.Compute();

  // 4. ACTUAR: Mover el servo
  // El 'Output' del PID es la corrección que debemos aplicar.
  // Lo sumamos (o restamos) al pulso central del servo.
  int servoPulse = servoPulseMiddle + Output;

  // Nos aseguramos de que el pulso final esté dentro de los límites del servo.
  servoPulse = constrain(servoPulse, SERVOMIN, SERVOMAX);
  
  // Enviamos el comando de pulso al servo en el canal 0 del PCA9685
  pwm.setPWM(0, 0, servoPulse);

  // Opcional: Imprimir datos en el Monitor Serie para depurar y afinar
  printSerialData();
}

void printSerialData() {
  Serial.print("Setpoint: "); Serial.print(Setpoint);
  Serial.print(" cm,  Posicion: "); Serial.print(Input);
  Serial.print(" cm,  Output PID: "); Serial.println(Output);
}