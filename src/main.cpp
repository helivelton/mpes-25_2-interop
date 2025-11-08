 #include <Arduino.h>
 #include <WiFi.h>
 #include <FirebaseESP32.h>
 #include <Wire.h> 
 #include <SparkFun_APDS9960.h> 
 #include <ESP32Servo.h> 
 
 // Provide the RTDB payload printing info and other helper functions.
 #include <addons/RTDBHelper.h>
 
 /* 1. Defina as credenciais do WiFi */
 #define WIFI_SSID "uaifai-brum"
 #define WIFI_PASSWORD "bemvindoaocesar"
 
 /* 2. Defina a URL do RTDB e o segredo do banco de dados */
 #define DATABASE_URL "mpes-2025-2-interop-1-default-rtdb.firebaseio.com" 
 #define DATABASE_SECRET "AcvPwRJ76bEPaNtLaTbGCat0Br9LKaswPIoDNAkJ"
 
 /* 3. Defina o objeto de Dados do Firebase */
 FirebaseData fbdo;
 FirebaseAuth auth;
 FirebaseConfig config;
 
 // ⬅️ Pinos e Definições de Sensores e Atuadores
 #define POT_PIN 39  // Pino analógico para o potenciômetro (Atualizado de 34)
 #define SERVO_PIN 25 // Pino para o Servo

 // ⬅️ Pinos e Canais para Controle do LED RGB (PWM/LEDC)
 #define RED_LED_PIN 19
 #define GREEN_LED_PIN 23
 #define BLUE_LED_PIN 18

 #define LED_CHANNEL_R 0 // Canal 0 para o Vermelho
 #define LED_CHANNEL_G 1 // Canal 1 para o Verde
 #define LED_CHANNEL_B 2 // Canal 2 para o Azul
 #define LED_PWM_FREQ 5000 // Frequência do PWM em Hz
 #define LED_PWM_RES 10    // Resolução do PWM em bits (0 a 1023)
 #define LED_MAX_DUTY (1 << LED_PWM_RES) - 1 // 1023
 
 // ⬅️ Posições do Servo
 const int open_servo = 0;   // Ângulo para 'Aberto' (Estado false)
 const int close_servo = 180; // Ângulo para 'Fechado' (Estado true)

 // ⬅️ Objetos e Variáveis de Controle
 SparkFun_APDS9960 apds = SparkFun_APDS9960();
 Servo myservo; // Objeto Servo
 
 unsigned long dataMillis = 0;
 unsigned long sensorReadMillis = 0;
 unsigned long servoReadMillis = 0; 
 const long updateInterval = 5000; // Intervalo de atualização do Firebase (5 segundos)
 const long sensorInterval = 1000; // Intervalo de leitura do sensor/potenciômetro
 const long servoInterval = 500; // Intervalo de leitura do Firebase para o Servo (0.5s)
 
 // Variáveis globais de Sensores
 uint16_t amb_light = 0; 
 uint16_t red_light = 0;   
 uint16_t green_light = 0; 
 uint16_t blue_light = 0;  
 int pot_value = 0; // ⬅️ Valor do Potenciômetro
 
 // Variáveis globais de Contagem e Estado do Atuador
 int entradas = 0;
 int saidas = 0;
 int last_gesture = 0; 
 bool previousState = false; // Estado anterior do Firebase para controle do Servo
 
   
 void setup()
 {
     Serial.begin(9600); 
 
     WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
     Serial.print("Conectando ao Wi-Fi");
     while (WiFi.status() != WL_CONNECTED) {
         Serial.print(".");
         delay(300);
     }
     Serial.println();
     Serial.print("Conectado com IP: ");
     Serial.println(WiFi.localIP());
     Serial.println();
 
     // ⬅️ DELAY ADICIONAL para estabilizar a fonte e evitar Brownout ao iniciar periféricos
     delay(2000); 

     Serial.printf("Firebase Client v%s\n\n", FIREBASE_CLIENT_VERSION);
 
     // ⬅️ Inicialização de Hardware
     Wire.begin(21, 22);
     analogReadResolution(12); // Para o Potenciômetro (0-4095)
 
     // ⬅️ Inicialização do Servo
     myservo.setPeriodHertz(50);
     myservo.attach(SERVO_PIN, 1000, 2000); 
     myservo.write(close_servo);
     Serial.println("Servo: Anexado no Setup.");

     // ⬅️ Configuração do LEDC (PWM) para o RGB
     // Lembre-se de usar resistores limitadores de corrente nos pinos 19, 23 e 18!
     ledcSetup(LED_CHANNEL_R, LED_PWM_FREQ, LED_PWM_RES);
     ledcAttachPin(RED_LED_PIN, LED_CHANNEL_R);
     ledcSetup(LED_CHANNEL_G, LED_PWM_FREQ, LED_PWM_RES);
     ledcAttachPin(GREEN_LED_PIN, LED_CHANNEL_G);
     ledcSetup(LED_CHANNEL_B, LED_PWM_FREQ, LED_PWM_RES);
     ledcAttachPin(BLUE_LED_PIN, LED_CHANNEL_B);
     // Inicializa LEDs desligados
     ledcWrite(LED_CHANNEL_R, 0);
     ledcWrite(LED_CHANNEL_G, 0);
     ledcWrite(LED_CHANNEL_B, 0);
 
     // ⬅️ Inicialização do sensor APDS-9960
     if (apds.init()) {
         Serial.println("APDS-9960 Inicializado!");
     } else {
         Serial.println("Erro ao inicializar APDS-9960. Verifique as ligações e a biblioteca.");
     }
 
     // ⬅️ Habilita as funcionalidades desejadas
     if (apds.enableProximitySensor(true)) { 
         Serial.println("Proximidade Habilitada");
     }
     if (apds.enableGestureSensor(true)) { 
         Serial.println("Gestos Habilitados");
     }
     if (apds.enableLightSensor(true)) { 
         Serial.println("Sensor de Luz Habilitado");
     }
 
     config.database_url = DATABASE_URL;
     config.signer.tokens.legacy_token = DATABASE_SECRET;
 
     Firebase.reconnectNetwork(true);
     fbdo.setBSSLBufferSize(4096, 1024);
     
     /* Inicializa a biblioteca com o Firebase authen e config */
     Firebase.begin(&config, &auth);
 }
 
 // ⬅️ Funções Auxiliares
 char* getGestureText(int gestureCode) {
  switch (gestureCode) {
      case DIR_UP: return "UP";
      case DIR_DOWN: return "DOWN";
      case DIR_LEFT: return "LEFT";
      case DIR_RIGHT: return "RIGHT";
      case DIR_NEAR: return "NEAR";
      case DIR_FAR: return "FAR";
      default: return "NONE"; // 0
  }
}
 
void dumpGesture(int gesture) {
  char* gesture_text = getGestureText(gesture);
  
  if (gesture == DIR_UP || gesture == DIR_LEFT) {
    entradas++;
    Serial.printf("Set ENTRADA (%s -> %d)... %s\n", gesture_text, entradas,
      Firebase.setInt(fbdo, "/sensor/entrada", entradas) ? 
      "ok" : fbdo.errorReason().c_str());
  }

  if (gesture == DIR_RIGHT || gesture == DIR_DOWN) {
    saidas++;
    Serial.printf("Set SAÍDA (%s -> %d)... %s\n", gesture_text, saidas,
      Firebase.setInt(fbdo, "/sensor/saida", saidas) ? 
      "ok" : fbdo.errorReason().c_str());
  }
}

int readGesture() {
  if (apds.isGestureAvailable()) {
    return apds.readGesture();
  }
  return 0; 
}

void readPotentiometer() {
    pot_value = analogRead(POT_PIN);
    // ⬅️ DEBUG ADICIONADO: Ajuda a diagnosticar o valor travado em 4095
    Serial.printf("DEBUG Potenciometro RAW: %d\n", pot_value); 
}

void readAmbientLight() {
  uint16_t r, g, b, c;

  const long INPUT_MAX = 1500; 
  const long OUTPUT_MAX = 255;

  if (apds.readAmbientLight(c) && apds.readRedLight(r) && apds.readGreenLight(g) && apds.readBlueLight(b)) {
      amb_light = map(c, 0, INPUT_MAX, 0, OUTPUT_MAX);
      red_light = map(r, 0, INPUT_MAX, 0, OUTPUT_MAX);
      green_light = map(g, 0, INPUT_MAX, 0, OUTPUT_MAX);
      blue_light = map(b, 0, INPUT_MAX, 0, OUTPUT_MAX);
  }
}

void dumpSensorData() {
    // 1. Cria o objeto JSON para os dados de luminosidade
    FirebaseJson lightData;
    lightData.add("clear", amb_light);
    lightData.add("red", red_light);
    lightData.add("green", green_light);
    lightData.add("blue", blue_light);
    
    // 2. Cria o payload de atualização, incluindo a luminosidade e o potenciômetro.
    FirebaseJson updatePayload;
    updatePayload.add("luminosity", lightData);
    updatePayload.add("potentiometer", pot_value); 
    
    // 3. Usa updateNode para não apagar 'entrada' e 'saida' que estão em /sensor
    Serial.printf("Update Sensores (Luz + Pot)... R:%d G:%d B:%d A:%d P:%d | %s\n", 
      red_light, green_light, blue_light, amb_light, pot_value,
      Firebase.updateNode(fbdo, "/sensor", updatePayload) ? "ok" : fbdo.errorReason().c_str()
    );
}

void detectGesture() {
  int current_gesture = readGesture();
  
  if (current_gesture != last_gesture) {
    
    dumpGesture(current_gesture);
    
    Serial.printf("Gesto Alterado: %s\n", getGestureText(current_gesture));
    
    last_gesture = current_gesture;
  }
}

void detectLight() {
  if (millis() - sensorReadMillis > sensorInterval) {
    sensorReadMillis = millis();

    readAmbientLight(); 
    readPotentiometer(); // ⬅️ Lendo o Potenciômetro aqui (sincronizado com o dump e o LED)
    dumpSensorData();
  }
}

void detectServo() {
  // 1. Time interval check
  if (millis() - servoReadMillis > servoInterval) {
      servoReadMillis = millis();

      // 2. Read the Firebase value for the actuator state
      if (Firebase.getBool(fbdo, "/atuador/estado")) {
          bool currentState = fbdo.boolData();
          
          Serial.printf("Servo: Valor lido do Firebase: %s\n", currentState ? "TRUE (Fechar)" : "FALSE (Abrir)");

          // 3. Check if the state has changed
          if (previousState != currentState) {
              
              // 4. Move the servo (it is attached in setup())
              if (currentState) {
                  // Firebase state is TRUE (Fecha)
                  myservo.write(close_servo);
                  Serial.println("Servo: Fechando (CLOSE) -> Angulo: 180");
              } else {
                  // Firebase state is FALSE (Abre)
                  myservo.write(open_servo);
                  Serial.println("Servo: Abrindo (OPEN) -> Angulo: 0");
              }
              
              // 5. Update the previous state
              previousState = currentState; 
          }
          
      } else {
          // Handle Firebase read error
          Serial.printf("ERRO Firebase Servo: %s\n", fbdo.errorReason().c_str());
      }
  }
}

// ⬅️ FUNÇÃO: Controla o Brilho dos LEDs RGB com o Potenciômetro
void controlRGBWithPot() {
  // O pot_value é de 0 a 4095 (resolução de 12 bits)
  // O PWM do LEDC está configurado para 10 bits (0 a 1023)
  
  // Mapeia o valor analógico do potenciômetro para o duty cycle do PWM
  int mapped_brightness = map(pot_value, 0, 4095, 0, LED_MAX_DUTY);
  
  // Define a mesma intensidade (brightness) para R, G e B, criando uma luz neutra.
  ledcWrite(LED_CHANNEL_R, mapped_brightness);
  ledcWrite(LED_CHANNEL_G, mapped_brightness);
  ledcWrite(LED_CHANNEL_B, mapped_brightness);
  
  //Serial.printf("Pot: %d -> PWM Duty Cycle: %d\n", pot_value, mapped_brightness);
}

void loop()
{
  if (!Firebase.ready()) {
    Serial.println("Firebase não está pronto. Aguardando...");
    delay(1000);
    return;
  }
  
  detectServo();
  detectGesture();
  detectLight();

  controlRGBWithPot();
}