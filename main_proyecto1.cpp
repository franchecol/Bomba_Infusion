  #include <Arduino.h>
  #include <AccelStepper.h>

  // Definición de pines
  #define DIR_PIN 12
  #define STEP_PIN 14
  #define RST_PIN 27
  #define SLEEP_PIN 26
  #define BUZZER_PIN 15

  int16_t FLOW_RATE1;

  // Definición de parámetros del motor
  #define MAX_SPEED 64000// 1000 * 16
  #define ACCELERATION 32000// 500 * 16


  #define STEPS_PER_REVOLUTION 200 // Pasos por revolución para tu motor
  #define MICROSTEPS 16 // Número de micropasos por paso
  #define SCREW_PITCH 0.8 // Paso del tornillo en mm
  #define JERINGA_DIAMETRO 30 // Diámetro de la jeringa en mm
  


  #define SECONDS_PER_HOUR 3600.0

  AccelStepper stepper(AccelStepper::DRIVER, STEP_PIN, DIR_PIN);

  unsigned long lastStepTime = 0;
  unsigned long cronometro_prevMillis = 0;
  int16_t valor_cronometro = 0;
  bool motorRunning = true; // Añade una nueva variable de bandera al inicio de tu código
  int16_t registro_start_stop;
    
  int stepInterval;


  /* Definición de los registros */ 
  #define REG_CARGA_BATERIA_WRITE  0x5000
  #define REG_VOLUMEN_READ 0x5100
  #define REG_TIEMPO_READ 0x5200
  #define REG_FLUJO_READ 0x5300
  #define REG_CRONOMETRO_WRITE 0x5400
  #define REG_FLUJO_SENSADO_WRITE 0x5500
  #define REG_INICIO_BOLO 0x5600 
  #define REG_INICIO_INFUSION 0x5700
  #define REG_INICIO_PURGA 0x5800
  #define REG_REACTIVAR_TERAPIA 0x5900

  /*Registros adicionales */
  #define REG_ALIMENTACION 0x6000
  #define REG_NOMBRE_TERAPIA 0x6100
  #define REG_ID_TERAPIA 0x6200
  #define REG_GUARDAR_TERAPIA 0x6300

  int valorFlujo;
  int16_t dato_lectura_flujo;
  int16_t valor_actual_flujo;
  int16_t dato_lectura_volumen;
  int16_t valor_actual_volumen;

  int16_t leerRegistro(uint16_t registro) {
    // Comando para leer un registro
    uint8_t cmd[] = {0x5A, 0xA5, 0x04, 0x83, (uint8_t)(registro >> 8), (uint8_t)registro, 0x01};

    // Envía el comando al dispositivo
    Serial2.write(cmd, sizeof(cmd));

    // Espera a que los datos estén disponibles
    uint32_t startTime = millis();
    while (Serial2.available() < 7) {
      if (millis() - startTime > 1000) { // Tiempo de espera de 1 segundo
        return -1; // Devuelve -1 si no hay respuesta después de 1 segundo
      }
    }

    // Lee todos los datos disponibles
    uint8_t response[200];
    int len = 0;
    while (Serial2.available()) {
      response[len++] = Serial2.read();
    }

    // Envía el comando de nuevo
    Serial2.write(cmd, sizeof(cmd));

    // Espera a que los datos estén disponibles
    uint32_t startTime1 = millis();
    while (Serial2.available() < 7) {
      if (millis() - startTime1 > 1000) { // Tiempo de espera de 1 segundo
        return -3; // Devuelve -1 si no hay respuesta después de 1 segundo
      }
    }

    // Lee todos los datos disponibles
    uint8_t response1[200];
    int len1 = 0;
    while (Serial2.available()) {
      response1[len++] = Serial2.read();
    }

    // Imprime los primeros 30 bytes de la respuesta en el Monitor Serial
    Serial.println("Segundos 30 bytes de la respuesta: ");
    for (int i = 0; i < 30 && i < len; i++) {
      if(response1[i] < 0x10) Serial.print('0'); // Asegura que los números hexadecimales siempre se impriman con dos dígitos
      Serial.print(response1[i], HEX);
      Serial.print(" ");
    }
    Serial.println();

    // Extrae y devuelve el valor del registro
    uint16_t valor = response1[17]; // Cambiado para leer el valor en la posición 26 
    return valor;
  }




  // Función para cambiar la pantalla
  void EscribirPantalla(uint16_t registropantalla, uint16_t valor_registro) {
    // Comando para escribir en la pantalla
    uint8_t cmd[] = {0x5A, 0xA5, 0x05, 0x82, (uint8_t)(registropantalla >> 8), (uint8_t)registropantalla, (uint8_t)(valor_registro >> 8), (uint8_t)valor_registro};

    // Envía el comando al dispositivo
    Serial2.write(cmd, sizeof(cmd));
    
    Serial.println("escribir registro");
  }


  // Función para cambiar la pantalla
  void cambiarPantalla(uint16_t numeroPantalla) {
    // Comando para cambiar la pantalla
    uint8_t cmd[] = {0x5A, 0xA5, 0x07, 0x82, 0x00, 0x84, 0x5A, 0x01, (uint8_t)(numeroPantalla >> 8), (uint8_t)numeroPantalla};

    // Envía el comando al dispositivo
    Serial2.write(cmd, sizeof(cmd));

    
  }

  // Función para poner los registros a cero
  void registros_a_cero(){  
    EscribirPantalla(REG_FLUJO_READ,0x0000); 
    EscribirPantalla(REG_VOLUMEN_READ,0x0000); 
    EscribirPantalla(REG_CRONOMETRO_WRITE,0x0000);
    EscribirPantalla(REG_FLUJO_SENSADO_WRITE,0x0000);   
    EscribirPantalla(REG_INICIO_INFUSION,0x0000);
    
  }


  void reiniciar() {

    int16_t valor_inicial = 0;


    // registros_a_cero();
    // cambiarPantalla(0x0000); // pantalla de inicio

    while (true) { // Ciclo infinito
      valor_actual_flujo = leerRegistro(REG_FLUJO_READ);
      valor_actual_volumen = leerRegistro(REG_VOLUMEN_READ);
      registro_start_stop = leerRegistro(REG_INICIO_INFUSION);

      if (valor_actual_flujo != valor_inicial && valor_actual_volumen != valor_inicial && registro_start_stop == 0x0001) {
        dato_lectura_flujo =valor_actual_flujo;
        dato_lectura_volumen = valor_actual_volumen;

        Serial.println("valor de lectura flujo :");
        Serial.print(dato_lectura_flujo);
        Serial.println("valor de lectura volumen :");
        Serial.print(dato_lectura_volumen);      
        break;
      }
      delay(1000);
    }
    Serial.println("SALIO DEL BUCLE");      
    FLOW_RATE1 = dato_lectura_flujo*200;
    valor_cronometro = round(((float)dato_lectura_volumen/dato_lectura_flujo)*60);
    EscribirPantalla(REG_CRONOMETRO_WRITE,valor_cronometro);
    EscribirPantalla(REG_FLUJO_SENSADO_WRITE,dato_lectura_flujo);
    cambiarPantalla(0x0006); // pantalla infusion en proceso

    float microstepsPerSecond = ((FLOW_RATE1) / FLUID_PER_REVOLUTION) * MICROSTEPS_PER_REVOLUTION / SECONDS_PER_HOUR;
    stepInterval = round(1000.0 / microstepsPerSecond);

    Serial.print("Flow rate: ");
    Serial.print(FLOW_RATE1);
    Serial.println(" ml/h");
    Serial.print("Microsteps per second: ");
    Serial.println(microstepsPerSecond);
    Serial.print("Microstep interval: ");
    Serial.println(stepInterval);
  }






  float stepsPerMl;


  void setup() {
    Serial.begin(115200);
    Serial2.begin(115200, SERIAL_8N1, 16, 17); // Elige la velocidad de baudios correcta y los pines correspondientes




    int16_t ultimoValorFlujo = -1; // Almacena el último valor del flujo leído
    int16_t datolectura;
    int16_t valor_inicial = 0;




    pinMode(DIR_PIN, OUTPUT);
    pinMode(STEP_PIN, OUTPUT);
    pinMode(RST_PIN, OUTPUT);
    pinMode(SLEEP_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(RST_PIN, HIGH);   // Mantén el controlador fuera del estado de reinicio
    digitalWrite(SLEEP_PIN, HIGH); // Mantén el controlador fuera del estado de suspensión
    stepper.setMaxSpeed(MAX_SPEED);
    stepper.setAcceleration(ACCELERATION);

    reiniciar();

  }


//   void loop() {
//   // ... (resto del código sin cambios)
//   // Actualización del número total de pasos
//   float desiredMl = leerRegistro(REG_VOLUMEN_READ); // Suponiendo que esto devuelve el volumen deseado en mL
//   long totalSteps = stepsPerMl * desiredMl;
//   stepper.move(totalSteps);
//   // ... (resto del código sin cambios)
// }

  void loop() {

      // Comprueba el estado del registro
    int16_t valor_actual_flujo1 = leerRegistro(REG_FLUJO_READ);
    int16_t valor_actual_volumen1 = leerRegistro(REG_VOLUMEN_READ);
    int16_t registro_start_stop = leerRegistro(REG_INICIO_INFUSION);

    float desiredMl = leerRegistro(REG_VOLUMEN_READ); // Suponiendo que esto devuelve el volumen deseado en mL
    long totalSteps = stepsPerMl * desiredMl;
    stepper.move(totalSteps);


    // Revisa si hay cambios en los registros desde la pantalla principal
    // Y si es así, asume que el usuario quiere iniciar una nueva infusión
    if (valor_actual_flujo1 != valor_actual_flujo && valor_actual_volumen1 != valor_actual_volumen) {
      reiniciar();
    }


    if (registro_start_stop == 0x0001) {
      motorRunning = true; // Actualiza la bandera
    }


    // Si el registro es 0, detén el motor
    if (registro_start_stop == 0x0000 && motorRunning) {
      Serial.println("Se detuvo el motor");
      stepper.stop(); // Detén el motor
      motorRunning = false; // Actualiza la bandera
      cambiarPantalla(0x0009); // pantalla infusion finalizada
    }
    int16_t reactivar_terapia = leerRegistro(REG_REACTIVAR_TERAPIA); 
    if (reactivar_terapia == 0x0001 ){
      cambiarPantalla(0x0006); // pantalla infusion en proceso
      EscribirPantalla(REG_REACTIVAR_TERAPIA,0x0000);
      motorRunning = true;
      
    }



    // Comprueba si ha pasado suficiente tiempo desde el último micropaso
    if (millis() - lastStepTime >= stepInterval && motorRunning) {
      // si es asi mueve el motor un micropaso
      stepper.move(-1);
      stepper.run();
      lastStepTime = millis();
    }

    // Código para actualizar el contador
    unsigned long currentMillis = millis(); // Guarda el tiempo actual
    // Código para actualizar el contador
    if (currentMillis - cronometro_prevMillis >= 1000) { // Verifica si ha pasado 1 segundo
      cronometro_prevMillis = currentMillis; // Actualiza el tiempo de la última actualización
      // Serial.println("Ha pasado un segundo"); // Imprime el mensaje
      if (valor_cronometro > 0) { // Verifica si el contador no es 0
        valor_cronometro--; // Disminuye el contador
        Serial.println(valor_cronometro); // Imprime el contador en el monitor serial
        EscribirPantalla(REG_CRONOMETRO_WRITE,valor_cronometro);
        if(valor_cronometro <5){
          digitalWrite(BUZZER_PIN, HIGH);
          // cambiarPantalla(0x000C); // pantalla error


        }
        if (valor_cronometro <= 0 && motorRunning) {
        stepper.stop(); // Si es así, detén el motor
        digitalWrite(BUZZER_PIN, LOW);
        motorRunning = false; // Y actualiza la bandera
        cambiarPantalla(0x000A); // pantalla infusion finalizada
        registros_a_cero(); 
      }
      
    }
    }
  }
