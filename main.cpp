// #include <Arduino.h>
// #include <AccelStepper.h>
// #include "Wire.h"
// #include "AS5600.h"
// #include "PID_v1.h"

// // Definición de pines
// #define DIR_PIN 12
// #define STEP_PIN 14
// #define RST_PIN 27
// #define SLEEP_PIN 26
// #define BUZZER_PIN 15

// AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);
// AS5600 as5600;  // use default Wire

// // Definición de parámetros del motor
// #define ACCELERATION 100
// #define STEPS_PER_REVOLUTION 200
// #define MICROSTEPS 16
// #define SCREW_PITCH 8
// #define JERINGA_DIAMETRO 30
// #define relacion_engranaje 4  // 60 a 15

// // Variables Globales
// float pasos_x_1mL_engranaje;
// double Setpoint, Input, Output;
// double Kp = 0.1, Ki = 0.05, Kd = 0.01;  // Ajusta estos valores según las pruebas y calibración
// PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// void actualizarFlujoDeseado(float flujoDeseado_ml_h) {
//   // Convertir flujo de ml/h a ml/s
//   float flujoDeseado_ml_s = abs(flujoDeseado_ml_h) / 3600.0;
//   Serial.print("Flujo Deseado ml/s: "); Serial.println(flujoDeseado_ml_s, 6);

//   // Calcular la velocidad teórica en pasos/segundo
//   float speed_teoretical = (STEPS_PER_REVOLUTION * MICROSTEPS / (SCREW_PITCH * PI * pow((JERINGA_DIAMETRO / 2.0), 2) * relacion_engranaje)) * flujoDeseado_ml_s;
//   Serial.print("Velocidad teórica: "); Serial.println(speed_teoretical, 6);

//   // Calcular grados por paso
//   float grados_por_paso = 360.0 / (STEPS_PER_REVOLUTION * MICROSTEPS);
//   Serial.print("Grados por paso: "); Serial.println(grados_por_paso, 6);

//   // Calcular el setpoint en grados por segundo
//   Setpoint = speed_teoretical * grados_por_paso;  
//   Serial.print("Setpoint calculado: "); Serial.println(Setpoint, 6);
// }

// void setup() {
//   Serial.begin(115200);

//   // Configuración de la librería AS5600
//   Wire.begin();
//   as5600.begin(22, 21); //SCL , SDA

//   as5600.begin(4);  // set direction pin.
//   as5600.setDirection(AS5600_CLOCK_WISE);  // default, just be explicit.

//  // Configuración de pines y motor
//   pinMode(DIR_PIN, OUTPUT);
//   pinMode(STEP_PIN, OUTPUT);
//   pinMode(RST_PIN, OUTPUT);
//   pinMode(SLEEP_PIN, OUTPUT);
//   pinMode(BUZZER_PIN, OUTPUT);
//   digitalWrite(RST_PIN, HIGH);
//   digitalWrite(SLEEP_PIN, HIGH);

//   stepper.setAcceleration(ACCELERATION);

//   // Configuración inicial del PID
//   myPID.SetMode(AUTOMATIC);
//   myPID.SetOutputLimits(-16000, 16000);  // Ajusta estos límites según sea necesario

//   // Setpoint inicial
//   actualizarFlujoDeseado(100.0);
// }

// void loop() {
//   static unsigned long lastPrintTime = 0;
//   unsigned long currentMillis = millis();

//   if (Serial.available() > 0) {
//     float nuevoFlujo = Serial.parseFloat();
//     actualizarFlujoDeseado(nuevoFlujo);
//     Serial.print("Nuevo flujo deseado: ");
//     Serial.print(nuevoFlujo);
//     Serial.println(" ml/h");
//     Serial.flush();
//     Serial.println(); //  línea en blanco para separar las lecturas

//   }

//   Input = as5600.getAngularSpeed();
//   // Serial.print("Velocidad Angular Actual: "); Serial.println(Input, 6);

//   myPID.Compute();
//   // Serial.print("Salida PID: "); Serial.println(Output, 6);

//   float grados_por_paso = 360.0 / (STEPS_PER_REVOLUTION * MICROSTEPS);
//   float speed_pasos_por_segundo = Output / grados_por_paso;
//   // Serial.print("Velocidad en pasos por segundo(1): "); Serial.println(speed_pasos_por_segundo, 6);

//   stepper.setSpeed(speed_pasos_por_segundo);
//   stepper.runSpeed();  

//   // Solo imprime cada segundo
//   if (currentMillis - lastPrintTime >= 1000) {
//     Serial.print("Velocidad en pasos por segundo(2): "); Serial.println(speed_pasos_por_segundo);
//     Serial.print("Velocidad Angular Actual: "); Serial.println(Input);
//     Serial.print("Salida PID: "); Serial.println(Output);
//     Serial.print("Setpoint: "); Serial.println(Setpoint);
//     lastPrintTime = currentMillis;
//   }
// }






#include <Arduino.h>
#include <AccelStepper.h>
#include "Wire.h"
#include "AS5600.h"

AS5600 as5600;   

// Definición de pines
#define DIR_PIN 12
#define STEP_PIN 14
#define RST_PIN 27
#define SLEEP_PIN 26
#define BUZZER_PIN 15

AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// Variables Globales
float pasos_x_1mL_engranaje;

// Definición de parámetros del motor
#define ACCELERATION 100
#define STEPS_PER_REVOLUTION 200
#define MICROSTEPS 16
#define SCREW_PITCH 8
#define JERINGA_DIAMETRO 30
#define relacion_engranaje 4 // 60 a 15

// Variables para el control
double Kp = 1; // para probar, no me acuerdo control 
double Setpoint, Input, Output, Error;

// Relación lineal entre el flujo deseado y la velocidad angular
double pendiente = -0.06598; // Pendiente de la línea
double interseccionY = 0.3589; // Intersección con el eje Y

void actualizarFlujoDeseado(float flujoDeseado_ml_h) {
  // Convertir flujo deseado a velocidad angular esperada
  Setpoint = flujoDeseado_ml_h * pendiente + interseccionY;
 // si el flujo(ml/h) ingresado es 100 el setpoint sera aproximadamente 6.5
}


void controlProporcional() {
  Input = as5600.getAngularSpeed(); // vel actual
  Error = Setpoint - Input; // error a corrregir
  Output = Kp * Error; // salida control kp
  
  float flujoDeseado_ml_h = (Output - interseccionY) / pendiente; // conversion inversa

  float flujoDeseado_ml_s = flujoDeseado_ml_h / 3600.0; // ml_h a ml_s

  float velocidadMotorPasosPorSegundo = flujoDeseado_ml_s * pasos_x_1mL_engranaje; // conversion original
  
  velocidadMotorPasosPorSegundo = constrain(velocidadMotorPasosPorSegundo, -16000, 16000); // min,max ... ( no se si es necesario)
  
  stepper.setSpeed(velocidadMotorPasosPorSegundo); // ahora si entiende las unidades del motor
}


// void controlProporcional() {
//   Input = as5600.getAngularSpeed(); // Obtén la velocidad angular actual
//   Error = Setpoint - Input; // Calcula el error
//   Output = Kp * Error; // Calcula la salida del control proporcional
  
//   // Convertir la salida a una velocidad comprensible para el motor
//   float velocidadMotor = Output * conversion;
  
//   // Limitar la velocidad del motor a los valores máximos y mínimos permitidos
//   // velocidadMotor = constrain(velocidadMotor, -1000, 1000);
  
//   // Aplicar la velocidad al motor
//   stepper.setSpeed(velocidadMotor);
// }


// void actualizarFlujoDeseado(float flujoDeseado_ml_h) {
//   float flujoDeseado_ml_s = abs(flujoDeseado_ml_h) / 3600.0;
//   float speed = pasos_x_1mL_engranaje * flujoDeseado_ml_s * (flujoDeseado_ml_h >= 0 ? 1 : -1);
//   stepper.setSpeed(speed);
//   Serial.print("Nuevo speed: ");
//   Serial.println(speed);



void setup() {
  Serial.begin(115200);
  Wire.begin();
  as5600.begin(22, 21); // SCL, SDA
  as5600.setDirection(AS5600_CLOCK_WISE);

  // Calculo de pasos por mL para el engranaje
  float pasos_x_1mm = STEPS_PER_REVOLUTION * MICROSTEPS / SCREW_PITCH;
  float areaJeringa = PI * (JERINGA_DIAMETRO / 2) * (JERINGA_DIAMETRO / 2);
  float altura = 1000 / areaJeringa;
  pasos_x_1mL_engranaje = pasos_x_1mm * altura * relacion_engranaje;

  // Configuración de pines
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(RST_PIN, OUTPUT);
  pinMode(SLEEP_PIN, OUTPUT);
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(RST_PIN, HIGH);
  digitalWrite(SLEEP_PIN, HIGH);

  // Configuración del motor
  stepper.setAcceleration(ACCELERATION);
  stepper.setMaxSpeed(pasos_x_1mL_engranaje * 2000.0 / 3600.0); // Basado en el flujo máximo de 2000 ml/h

  actualizarFlujoDeseado(20.0); // Establecer un flujo deseado inicial
}

void loop() {
  static unsigned long lastPrintTime = 0; // Tiempo desde el último print
  unsigned long currentMillis = millis(); // Tiempo actual

  // primera ejecucion del loop para mantener constante el flujo
  stepper.runSpeed();

  // llamada para corregir el flujo
  controlProporcional();

  // imprimir cada 500ms
  if (currentMillis - lastPrintTime >= 500) {
    Serial.print("Setpoint (rad/s): ");
    Serial.print(Setpoint, 4);
    Serial.print("\tInput (rad/s): ");
    Serial.print(Input, 4);
    Serial.print("\tError: ");
    Serial.print(Error, 4);
    Serial.print("\tOutput (pasos/s): ");
    Serial.print(stepper.speed(), 4);
    Serial.print("\tKp Value: ");
    Serial.println(Kp, 4);
    lastPrintTime = currentMillis;
  }
 // solo para recibir los datos del flujo deseado en ml/h
  if (Serial.available() > 0) {
    float nuevoFlujo = Serial.parseFloat();
    actualizarFlujoDeseado(nuevoFlujo);
    Serial.print("Nuevo flujo deseado: ");
    Serial.print(nuevoFlujo);
    Serial.println(" ml/h");
    while(Serial.available() > 0) {
      Serial.read();
    }
  }
}









// #include <Arduino.h>
// #include <AccelStepper.h>
// #include "Wire.h"
// #include "AS5600.h"


// AS5600 as5600;   //  use default Wire

// // Definición de pines
// #define DIR_PIN 12
// #define STEP_PIN 14
// #define RST_PIN 27
// #define SLEEP_PIN 26
// #define BUZZER_PIN 15

// AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// // Variables Globales
// float pasos_x_1mL_engranaje; 

// // Definición de parámetros del motor
// #define ACCELERATION 100
// #define STEPS_PER_REVOLUTION 200
// #define MICROSTEPS 16
// #define SCREW_PITCH 8
// #define JERINGA_DIAMETRO 30
// #define relacion_engranaje 4 // 60 a 15

// void actualizarFlujoDeseado(float flujoDeseado_ml_h) {
//   float flujoDeseado_ml_s = abs(flujoDeseado_ml_h) / 3600.0;
//   float speed = pasos_x_1mL_engranaje * flujoDeseado_ml_s * (flujoDeseado_ml_h >= 0 ? 1 : -1);
//   stepper.setSpeed(speed);
//   Serial.print("Nuevo speed: ");
//   Serial.println(speed);

// }

// void setup() {
//   Serial.begin(115200);

//  //LIBRERIA AS5600

//   Serial.println(__FILE__);
//   Serial.print("AS5600_LIB_VERSION: ");
//   Serial.println(AS5600_LIB_VERSION);

//   Wire.begin();
//   as5600.begin(22, 21); //SCL , SDA

//   // as5600.begin(4);  //  set direction pin.
//   as5600.setDirection(AS5600_CLOCK_WISE);  // default, just be explicit.

//   Serial.println(as5600.getAddress());

//   // as5600.setAddress(0x40);  // AS5600L only

//   int b = as5600.isConnected();
//   Serial.print("Connect: ");
//   Serial.println(b);

//   delay(1000);

//  //LIBRERIA AS5600



//   float pasos_x_1mm = STEPS_PER_REVOLUTION * MICROSTEPS * (1.0 / SCREW_PITCH);
//   float altura = 1000 / (PI * pow((JERINGA_DIAMETRO * 0.5), 2));
//   pasos_x_1mL_engranaje = pasos_x_1mm * altura * relacion_engranaje;

// // flujo máximo definido en 2000 ml/h
//   float flujoMaximo_ml_s = 2000.0 / 3600.0;
//   float maxSpeed = pasos_x_1mL_engranaje * flujoMaximo_ml_s;
  
//   pinMode(DIR_PIN, OUTPUT);
//   pinMode(STEP_PIN, OUTPUT);
//   pinMode(RST_PIN, OUTPUT);
//   pinMode(SLEEP_PIN, OUTPUT);
//   pinMode(BUZZER_PIN, OUTPUT);
//   digitalWrite(RST_PIN, HIGH);
//   digitalWrite(SLEEP_PIN, HIGH);
  
//   stepper.setAcceleration(ACCELERATION);
//   stepper.setMaxSpeed(maxSpeed);
  
//   actualizarFlujoDeseado(20.0);
// }

// void loop() {
//   static unsigned long lastPrintTime = 0; // Tiempo desde el último print
//   unsigned long currentMillis = millis(); // Tiempo actual

//   // Ejecuta esta parte siempre lo más rápido posible
//   stepper.runSpeed();

//   // Chequea si han pasado 100ms desde la última vez
//   if (currentMillis - lastPrintTime >= 500) {
//     Serial.print("\tω = ");
//     Serial.println(as5600.getAngularSpeed());
//     // Actualiza el último tiempo de impresión
//     lastPrintTime = currentMillis;
//   }

//   // La parte de recepción de datos serial no necesita delay
//   if (Serial.available() > 0) {
//     float nuevoFlujo = Serial.parseFloat();
//     actualizarFlujoDeseado(nuevoFlujo);
//     Serial.print("Nuevo flujo deseado: ");
//     Serial.print(nuevoFlujo);
//     Serial.println(" ml/h");
//     Serial.flush();
//   }
// }













// #include <Arduino.h>
// #include <AccelStepper.h>
// #include "Wire.h"

// // Definición de pines
// #define DIR_PIN 12
// #define STEP_PIN 14
// #define RST_PIN 27
// #define SLEEP_PIN 26
// #define BUZZER_PIN 15

// AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

// // Variables Globales
// float pasos_x_1mL_engranaje; 

// // Definición de parámetros del motor
// #define ACCELERATION 100
// #define STEPS_PER_REVOLUTION 200
// #define MICROSTEPS 16
// #define SCREW_PITCH 8
// #define JERINGA_DIAMETRO 30
// #define relacion_engranaje 4

// void actualizarFlujoDeseado(float flujoDeseado_ml_h) {
//   float flujoDeseado_ml_s = abs(flujoDeseado_ml_h) / 3600.0;
//   float speed = pasos_x_1mL_engranaje * flujoDeseado_ml_s * (flujoDeseado_ml_h >= 0 ? 1 : -1);
//   stepper.setSpeed(speed);
//   Serial.print("Nuevo speed: ");
//   Serial.println(speed);
// }

// void setup() {
//   Serial.begin(115200);

//   float pasos_x_1mm = STEPS_PER_REVOLUTION * MICROSTEPS * (1.0 / SCREW_PITCH);
//   float altura = 1000 / (PI * pow((JERINGA_DIAMETRO * 0.5), 2));
//   pasos_x_1mL_engranaje = pasos_x_1mm * altura * relacion_engranaje;

//   float flujoMaximo_ml_s = 2000.0 / 3600.0;
//   float maxSpeed = pasos_x_1mL_engranaje * flujoMaximo_ml_s;
  
//   pinMode(DIR_PIN, OUTPUT);
//   pinMode(STEP_PIN, OUTPUT);
//   pinMode(RST_PIN, OUTPUT);
//   pinMode(SLEEP_PIN, OUTPUT);
//   pinMode(BUZZER_PIN, OUTPUT);
//   digitalWrite(RST_PIN, HIGH);
//   digitalWrite(SLEEP_PIN, HIGH);
  
//   stepper.setAcceleration(ACCELERATION);
//   stepper.setMaxSpeed(maxSpeed);
  
//   actualizarFlujoDeseado(20.0);
// }

// void loop() {
//   stepper.runSpeed();
  
//   if (Serial.available() > 0) {
//     float nuevoFlujo = Serial.parseFloat();
//     actualizarFlujoDeseado(nuevoFlujo);
//     Serial.print("Nuevo flujo deseado: ");
//     Serial.print(nuevoFlujo);
//     Serial.println(" ml/h");
//     Serial.flush();
//   }
// }
