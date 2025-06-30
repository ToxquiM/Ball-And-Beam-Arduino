#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <PID_v1.h>
#include <NewPing.h>

// --- CONFIGURACIÓN DE COMPONENTES ---
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
#define SERVO_CHANNEL 0
#define SERVO_FREQ 50

#define SERVO_MIN_PULSE 150
#define SERVO_MAX_PULSE 550
#define SERVO_NEUTRAL_PULSE 350

#define TRIGGER_PIN 8
#define ECHO_PIN 9
#define MAX_DISTANCE_CM 35
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE_CM);

// --- CONTROL PID ---
#define TOLERANCE_CM 1.0

double Kp = 25.0;
double Ki = 5.0;
double Kd = 7.0;

double Setpoint;
double Input;
double Output;

PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

int intervalo_envio_ms = 40; // se puede cambiar desde Python

void setup() {
  Serial.begin(9600);
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);
  delay(10);

  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  pwm.setPWM(SERVO_CHANNEL, 0, SERVO_NEUTRAL_PULSE);
  delay(1000);

  Setpoint = 15.0;
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(0, 255);
  myPID.SetSampleTime(intervalo_envio_ms);

  Serial.println("Sistema iniciado");
}

void loop() {
  revisarSerial();

  Input = sonar.ping_cm();
  if (Input == 0) Input = Setpoint;

  double error = abs(Input - Setpoint);

  if (error <= TOLERANCE_CM) {
    pwm.setPWM(SERVO_CHANNEL, 0, SERVO_NEUTRAL_PULSE);
  } else {
    myPID.Compute();
    int servoPulse = map(Output, 0, 255, SERVO_MIN_PULSE, SERVO_MAX_PULSE);
    pwm.setPWM(SERVO_CHANNEL, 0, servoPulse);
  }

  // Enviar datos a Python para graficar
  Serial.print("Posición: "); Serial.print(Input);
  Serial.print(" cm\tError: "); Serial.print(Setpoint - Input);
  Serial.print(" cm\tOutput: "); Serial.println(Output);

  delay(intervalo_envio_ms);
}

// === PROCESAR COMANDOS DESDE PYTHON ===
void revisarSerial() {
  if (Serial.available()) {
    String comando = Serial.readStringUntil('\n');
    comando.trim();

    if (comando.startsWith("S")) {
      float nuevoSetpoint = comando.substring(1).toFloat();
      if (nuevoSetpoint >= 5 && nuevoSetpoint <= 30) {
        Setpoint = nuevoSetpoint;
        Serial.print("Nuevo setpoint: ");
        Serial.println(Setpoint);
      }
    } else if (comando.startsWith("F")) {
      int nuevoDelay = comando.substring(1).toInt();
      if (nuevoDelay >= 20 && nuevoDelay <= 200) {
        intervalo_envio_ms = nuevoDelay;
        myPID.SetSampleTime(nuevoDelay);
        Serial.print("Nuevo intervalo de muestreo: ");
        Serial.print(intervalo_envio_ms);
        Serial.println(" ms");
      }
    }
  }
}