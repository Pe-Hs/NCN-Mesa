#include <LiquidCrystal_I2C.h>
#include <Arduino_PortentaBreakout.h>

#include <SPI.h>
#include <stdlib.h>

#include <PortentaEthernet.h>
#include <Ethernet.h>
#include <WiFi.h>
#include <ArduinoJson.h>

#include "SDMMCBlockDevice.h"
#include "FATFileSystem.h"
#include "mbed.h"
#include "platform/mbed_retarget.h"

LiquidCrystal_I2C lcd(0x27, 20, 4);  // 0x3f or 0x27

// Configuración de la tarjeta SD
SDMMCBlockDevice sd;
mbed::FATFileSystem fs("fs");

// CONEXION ETHERNET
byte mac[] = { 0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(192, 168, 1, 170);  //Direccion IP por defecto
EthernetServer server(80);       //Habilitar puerto 80

// CONEXION WIFI
char ssid[] = "NCN_MESA_01";  // your network SSID (name)
char pass[] = "mesamesa";     // Contraseña WIFI
int keyIndex = 0;             // Flag para usar WEP

int status = WL_IDLE_STATUS;  // Estado del WIFI

IPAddress apIP(192, 168, 1, 160);      // IP del punto de acceso
IPAddress apGateway(192, 168, 1, 1);   // Puerta de enlace
IPAddress apSubnet(255, 255, 255, 0);  // Máscara de subred
WiFiServer wi_server(80);

// PINES DE PORTENTA BREAKOUT
breakoutPin pwmPins[] = { PWM0, PWM1, PWM2, PWM3, PWM4, PWM5, PWM6, PWM7, PWM8, PWM9 };
breakoutPin analogPins[] = { ANALOG_A0, ANALOG_A1, ANALOG_A2, ANALOG_A3, ANALOG_A4, ANALOG_A5, ANALOG_A6, ANALOG_A7 };
breakoutPin gpioPins[] = { GPIO_0, GPIO_1, GPIO_2, GPIO_3, GPIO_4, GPIO_5, GPIO_6 };

// DEFINIR VALOR MIN Y MAX DEL POTENCIOMETRO 10K
#define POT_MIN 0
#define POT_MAX 1023

// PINES MOTOR
byte dirPin = 7;
byte stepPin = 8;
byte freePin = 9;

// PINES CONTROL MANUAL MOTOR
byte potAmp = 7;
byte potFre = 6;

//PIN DE CONTROL
byte btnSel = 5;

byte limitSwitchPin_1 = 4;
byte limitSwitchPin_2 = 3;

//VARIABLES SELECCION MODO
const unsigned long selectionDelay = 5000;  // Tiempo de espera  (5 segundos)

int buttonState = 0;
int lastButtonState = 0;
int clickCount = 0;

unsigned long startTime;
unsigned long elapsedTime = 5000;  // Tiempo en milisegundos (5 segundos en este caso)
unsigned long clickStartTime;      // Tiempo de inicio del clic

bool isCountingDown = false;           // Bandera para saber si el conteo regresivo está en marcha
unsigned long countdownStartTime = 0;  // Tiempo en el que comenzo el conteo regresivo
bool functionActive = false;           // Bandera para saber si la funcion está activa
bool useWiFi = false;                  // Control del uso del Servidor WIFI
bool useEthe = false;                  // Control del uso del Servidor Ethernet

// VARIABLES MOTOR
// Parámetros del movimiento
float amplitude = 15.0;                          // Amplitud en mm
float freq_ang = 0.16 * (2 * PI);                // Frecuencia (Velocidad Angular) 6 -> 2 * PI
const float freq_ang_max = 3.0 * (2 * PI);       // Frecuencia max (Velocidad Angular) 6 -> 2 * PI
const float stepsPerMM = 50.0;                    // Pasos por Revolucion (500 -> 360/0.72*) / 10 mm (distancia recorrida en una revolucion)
int stepsForAmplitude = amplitude * stepsPerMM;  // Pasos correspondientes a la amplitud
const float distCentro = 25.0;                   // Distancia hacia el centro en mm
int stepsToCenter = distCentro * stepsPerMM;     // Pasos hacia el centro

bool limitSwitchDetected = false;

float dvs, dvb;  // Segmentos de subida y bajada
int h_max, h_min;

// Variables de estado
bool isProcessing = false;
bool requestPending = false;
bool newPostDetected = false;

String getFunctionName(int select) {
  switch (select) {
    case 0: return "Modo MANUAL";
    case 1: return "Modo WIFI";
    case 2: return "Modo USB";
    case 3: return "Modo RED";
    default: return "NO MOD";
  }
}

void setup() {
  Serial.begin(9600);

  lcd.init();
  lcd.backlight();

  // Inicializa la tarjeta SD
  int err = sd.init();

  if (err) {
    Serial.print("Error SD");
    while (true) {
      delay(1);
    }
  }

  err = fs.mount(&sd);

  if (err) {
    Serial.print("Error FS");
    while (true) {
      delay(1);
    }
  }

  pinMode(pwmPins[btnSel], INPUT);
  startTime = millis();

  pinMode(pwmPins[dirPin], OUTPUT);
  pinMode(pwmPins[stepPin], OUTPUT);
  pinMode(pwmPins[freePin], OUTPUT);

  pinMode(pwmPins[limitSwitchPin_1], INPUT_PULLUP);
  pinMode(pwmPins[limitSwitchPin_2], INPUT_PULLUP);

  Breakout.digitalWrite(pwmPins[freePin], HIGH);

  lcd.setCursor(5, 1);
  lcd.print("Powered By NCN!");

  delay(2000);
  lcd.clear();

  centrar();

  lcd.setCursor(5, 1);
  lcd.print("Seleccionar");
  lcd.setCursor(8, 2);
  lcd.print("Modo");
  delay(2000);
  Serial.println("------");
}


void loop() {
  // int a = Breakout.digitalRead(pwmPins[limitSwitchPin_1]);

  // int b = Breakout.digitalRead(pwmPins[limitSwitchPin_2]);

  // Serial.print(a);
  // Serial.print(" ---- ");
  // Serial.println(b);

  

  int buttonState = Breakout.digitalRead(pwmPins[btnSel]);

  if (buttonState == HIGH && lastButtonState == LOW) {
    clickCount++;
    lcd.clear();

    clickStartTime = millis();
    countdownStartTime = millis();

    functionActive = false;
    isCountingDown = true;

    if (clickCount >= 4) {
      clickCount = 0;
    }
  }

  if (buttonState == LOW && lastButtonState == HIGH) {}  //Evento de liberación del botón.

  lastButtonState = buttonState;

  if (isCountingDown) {
    unsigned long elapsedTime = millis() - countdownStartTime;
    unsigned long timeLeft = selectionDelay - elapsedTime;

    Serial.println(getFunctionName(clickCount));
    lcd.setCursor(5, 1);
    lcd.print(getFunctionName(clickCount));
    lcd.setCursor(7, 2);
    lcd.print(timeLeft / 1000);
    lcd.setCursor(9, 2);
    lcd.print("seg.");

    if (elapsedTime % 1000 == 0) { ; }

    if (elapsedTime >= selectionDelay) {

      isCountingDown = false;
      functionActive = true;

      if (clickCount == 0) {
        lcd.clear();
        limitSwitchDetected = false;
        controlManual();
      } else if (clickCount == 1) {
        lcd.clear();
        Serial.println("WIFI");
        modoWifi();
      } else if (clickCount == 2) {
        lcd.clear();
        Serial.println("USB");
        modoUSB();
      } else if (clickCount == 3) {
        lcd.clear();
        Serial.println("UTP");
        modoEthernet();
      }
    }
  }
}

void controlManual_OG() {

  lcd.setCursor(0, 0);
  lcd.print("Amplitud: ");
  lcd.setCursor(4, 1);
  lcd.print("mm");
  lcd.setCursor(0, 2);
  lcd.print("Frecuencia: ");
  lcd.setCursor(7, 3);
  lcd.print("Hz");

  while (functionActive) {

    int buttonState = Breakout.digitalRead(pwmPins[btnSel]);

    if (buttonState == HIGH && lastButtonState == LOW) {
      functionActive = false;
      centrar();
      break;
    }

    lastButtonState = buttonState;

    // int potAmpValue = Breakout.analogRead(analogPins[potAmp]);
    // int potFreValue = Breakout.analogRead(analogPins[potFre]);

    // int amplitud_in = map(potAmpValue, 0, POT_MAX, 1, 10);
    // float frecuencia_in = (potFreValue / 1023.0) * 5.0;

    // freq_ang = frecuencia_in * (2 * PI);

    // clearRow(1, 4, "mm");
    // lcd.setCursor(1, 1);
    // lcd.print(amplitud_in);

    // clearRow(3, 5, "Hz");
    // lcd.setCursor(0, 3);
    // lcd.print(frecuencia_in, 2);

    // stepsForAmplitude = amplitud_in * stepsPerMM;

    if (!limitSwitchDetected) {

      // int potAmpValue = Breakout.analogRead(analogPins[potAmp]);
      // int potFreValue = Breakout.analogRead(analogPins[potFre]);

      // int amplitud_in = map(potAmpValue, 0, POT_MAX, 1, 10);

      // float frecuencia_in = map(potFreValue, 1, POT_MAX, 0, 1);

      // float frecuencia_in = (potFreValue / 1023.0) * 5.0;

      // freq_ang = frecuencia_in * (2 * PI);

      // lcd.setCursor(1, 1);
      // lcd.print(amplitud_in);

      // lcd.setCursor(0, 3);
      // lcd.print(frecuencia_in, 2);

      // stepsForAmplitude = amplitud_in * stepsPerMM;



      if (!checkLimitSwitch()) {
        moveSteps_OG(500, 1.20, true);
      }

      if (!checkLimitSwitch()) {
        moveSteps_OG(500, 1.20, false);
      }

      if (!checkLimitSwitch()) {
        moveSteps_OG(500, 1.20, false);
      }

      if (!checkLimitSwitch()) {
        moveSteps_OG(500, 1.20, true);
      }
    }
  }
}

void controlManual() {

  lcd.setCursor(0, 0);
  lcd.print("Amplitud: ");
  lcd.setCursor(4, 1);
  lcd.print("mm");
  lcd.setCursor(0, 2);
  lcd.print("Frecuencia: ");
  lcd.setCursor(7, 3);
  lcd.print("Hz");

  while (functionActive) {

    unsigned long mytime;
    mytime = micros();

    int buttonState = Breakout.digitalRead(pwmPins[btnSel]);

    int potAmpValue = Breakout.analogRead(analogPins[potAmp]);
    int potFreValue = Breakout.analogRead(analogPins[potFre]);

    int amplitud_in = map(potAmpValue, 0, POT_MAX, 0, 50);
    float frecuencia_in = (potFreValue / 1023.0) * 5.0;

    freq_ang = frecuencia_in * 6.28;
    stepsForAmplitude = amplitud_in * stepsPerMM;

    Serial.print("Amplitud: ");
    Serial.print(amplitud_in);
    Serial.print(" mm ");
    Serial.print(" Frecuencia: ");
    Serial.print(frecuencia_in);
    Serial.println(" Hz");

    if (buttonState == HIGH && lastButtonState == LOW) {
      functionActive = false;
      centrar();
      break;
    }

    lastButtonState = buttonState;

    bool dir = true;

    if (!limitSwitchDetected) {
      moveSteps(stepsForAmplitude, freq_ang);
    }
  }
}

void modoWifi() {
  WiFi.config(apIP, apGateway, apSubnet);  // Establecer IP WIFI server

  lcd.setCursor(5, 1);
  lcd.print("Modo WIFI");
  delay(5000);
  lcd.clear();

  WiFi.beginAP(ssid, pass);

  int attempts = 0;

  lcd.setCursor(7, 1);
  lcd.print("Espere");

  while (WiFi.status() != WL_CONNECTED && attempts < 10) {
    delay(500);
    lcd.setCursor(attempts + 5, 2);
    lcd.print(".");
    attempts++;
  }

  if (WiFi.status() != WL_CONNECTED) {
    lcd.setCursor(4, 1);
    lcd.print("No se pudo conectar");
    lcd.setCursor(4, 2);
    lcd.print(" Wi-Fi");
  }

  wi_server.begin();
  lcd.clear();

  lcd.setCursor(6, 0);
  lcd.print("IP WI-FI");
  lcd.setCursor(3, 1);
  lcd.print(WiFi.localIP());
  lcd.setCursor(7, 2);
  lcd.print("SSID");
  lcd.setCursor(4, 3);
  lcd.print(ssid);

  while (functionActive) {
    String sendPostData = "";

    int buttonState = Breakout.digitalRead(pwmPins[btnSel]);

    if (buttonState == HIGH && lastButtonState == LOW) {
      functionActive = false;
      break;
    }

    lastButtonState = buttonState;

    WiFiClient client = wi_server.available();

    if (client) {

      Serial.println("----------------------------");
      Serial.println("WIFI - Cliente conectado");

      lcd.setCursor(2, 3);
      lcd.print("                    ");
      lcd.setCursor(2, 3);
      lcd.print("Inicio de Consulta");

      String request = "";
      String postData = "";

      boolean isPost = false;
      boolean isGet = false;

      bool headersEnded = false;
      int contentLength = 0;

      int bytesRead = 0;
      int bytesReceived = 0;

      while (client.connected() && !client.available()) {
        delay(1);
      }

      while (client.available()) {
        char c = client.read();
        request += c;

        if (request.endsWith("\r\n\r\n")) {
          headersEnded = true;
          if (request.startsWith("POST")) {
            isPost = true;
          } else if (request.startsWith("GET")) {
            isGet = true;
          }
          int index = request.indexOf("Content-Length:");
          if (index != -1) {
            contentLength = request.substring(index + 15).toInt();
          }
        }

        if (headersEnded) {
          bytesRead++;
          sendPostData += c;
          if (bytesRead >= contentLength) {
            break;
          }
        }
      }

      if (isPost) {
        if (sendPostData.startsWith("\n{")) {
          sendPostData += "\n}";
          handlePOST(client, sendPostData);
        } else {
          handlePOST(client, sendPostData);
        }
      } else if (isGet) {
        handleGET(client, request);
      }

      Serial.println("----------------------------");

      lcd.setCursor(2, 3);
      lcd.print("                    ");
      lcd.setCursor(2, 3);
      lcd.print("Datos Recibidos");
    }
  }
}

void modoUSB() {
  lcd.setCursor(6, 1);
  lcd.print("Modo USB");
}

void modoEthernet() {

  lcd.setCursor(3, 1);
  lcd.print("Modo Internet");

  Ethernet.begin(mac, ip);

  if (Ethernet.linkStatus() == LinkOFF) {
    lcd.clear();
    lcd.setCursor(0, 1);
    lcd.print("Internet");
    lcd.setCursor(0, 2);
    lcd.print("Desconectado");
  }

  if (Ethernet.localIP() == IPAddress(0, 0, 0, 0)) {
    lcd.clear();
    lcd.setCursor(4, 1);
    lcd.print("Sin Cable");
    lcd.setCursor(7, 2);
    lcd.print("de Red");
    return;
  }

  server.begin();

  delay(5000);
  lcd.clear();

  lcd.setCursor(4, 0);
  lcd.print("IP Servidor");
  lcd.setCursor(3, 1);
  lcd.print(Ethernet.localIP());

  while (functionActive) {

    String sendPostData = "";

    int buttonState = Breakout.digitalRead(pwmPins[btnSel]);

    if (buttonState == HIGH && lastButtonState == LOW) {
      functionActive = false;
      break;
    }

    lastButtonState = buttonState;

    EthernetClient client = server.available();

    if (client) {
      Serial.println("----------------------------");
      Serial.println("ETHERNET - Cliente conectado");
      lcd.setCursor(2, 3);
      lcd.print("                    ");
      lcd.setCursor(2, 3);
      lcd.print("Inicio de Consulta");

      String request = "";
      String postData = "";

      boolean isPost = false;
      boolean isGet = false;

      bool headersEnded = false;
      int contentLength = 0;

      int bytesRead = 0;
      int bytesReceived = 0;

      while (client.connected() && !client.available()) {
        delay(1);
      }

      while (client.available()) {
        char c = client.read();
        request += c;

        if (request.endsWith("\r\n\r\n")) {
          headersEnded = true;
          if (request.startsWith("POST")) {
            isPost = true;
          } else if (request.startsWith("GET")) {
            isGet = true;
          }
          int index = request.indexOf("Content-Length:");
          if (index != -1) {
            contentLength = request.substring(index + 15).toInt();
          }
        }

        if (headersEnded) {
          bytesRead++;
          sendPostData += c;
          if (bytesRead >= contentLength) {
            break;
          }
        }
      }

      if (isPost) {
        if (sendPostData.startsWith("\n{")) {
          sendPostData += "\n}";
          handlePOST(client, sendPostData);
        } else {
          handlePOST(client, sendPostData);
        }
      } else if (isGet) {
        handleGET(client, request);
      }

      Serial.println("----------------------------");

      lcd.setCursor(2, 3);
      lcd.print("                    ");
      lcd.setCursor(2, 3);
      lcd.print("Datos Recibidos");
    }
  }
}

// ---------------------------

void clearRow(int row, int col, String und) {
  lcd.setCursor(0, row);
  lcd.print("                    ");
  lcd.setCursor(col, row);
  lcd.print(und);
}

void mostrarData(float time, float dist) {
  Serial.print("Tiempo: ");
  Serial.print(time, 3);
  Serial.print(" seg || Distancia: ");
  Serial.print(dist, 4);
  Serial.print(" cm || Velocidad: ");
  Serial.println("0.00");
}

void processLoop(const String& postData) {
  while (isProcessing) {

    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, postData);

    if (error) {
      return;
    }

    float amp = doc["amp"];
    float freq = doc["freq"];

    freq_ang = freq * (2 * PI);

    stepsForAmplitude = amp * stepsPerMM;

    moveSteps(stepsForAmplitude, freq_ang);

    EthernetClient client = server.available();

    if (client) {
      String request = "";
      while (client.available()) {
        char c = client.read();
        request += c;
        if (request.endsWith("\r\n\r\n")) {
          if (request.startsWith("POST")) {
            isProcessing = false;
            return;
          }
          break;
        }
      }
      client.stop();
    }
  }
}

void handlePOST(Client& client, const String& postData) {
  sendResponsePlain(client, "HTTP/1.1 200 OK", "Datos Recibidos Correctamente");  // postData

  if (postData.startsWith("------WebKitFormBoundary") || postData.indexOf("Content-Disposition: form-data") != -1) {
    String newPost = postData;

    for (int i = 0; i < 5; i++) {
      int newlineIndex = newPost.indexOf('\n');
      if (newlineIndex == -1) {
        return;
      }
      newPost = newPost.substring(newlineIndex + 1);
    }

    while (newPost.length() > 0) {
      int newlineIndex = newPost.indexOf('\n');
      String line;
      if (newlineIndex != -1) {
        line = newPost.substring(0, newlineIndex);
        newPost = newPost.substring(newlineIndex + 1);
      } else {
        line = newPost;
        newPost = "";
      }

      line.trim();

      if (line.length() == 0) {
        break;
      }

      int spaceIndex = line.indexOf(' ');

      if (spaceIndex != -1) {
        String timeString = line.substring(0, spaceIndex);
        String valueString = line.substring(spaceIndex + 1);
        timeString.trim();
        valueString.trim();

        float time = timeString.toFloat();
        float value = valueString.toFloat();

        mostrarData(time, value);
      }
    }

  } else if (postData.startsWith("\n{")) {

    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, postData);

    if (error) {
      return;
    }

    if (doc["amp"] || doc["freq"]) {
      float amp = doc["amp"];
      float freq = doc["freq"];

      Serial.print("Amp: ");
      Serial.print(amp);
      Serial.print(" Freq: ");
      Serial.println(freq);

      return;
    }

    if (doc["filename"]) {
      String filename = doc["filename"];
      Serial.print("Filenmae Selected: ");
      Serial.println(filename);

      return;
    }
  }
}

void handleGET(Client& client, const String& request) {

  String response = "";

  DIR* dir;
  struct dirent* ent;
  int dirIndex = 0;

  DynamicJsonDocument doc(1024);
  JsonArray filesArray = doc.to<JsonArray>();

  if ((dir = opendir("/fs")) != NULL) {
    while ((ent = readdir(dir)) != NULL) {
      if (strstr(ent->d_name, ".txt") != NULL) {
        filesArray.add(ent->d_name);
        dirIndex++;
      }
    }
    closedir(dir);
  } else {
    lcd.setCursor(2, 3);
    lcd.print("                    ");
    lcd.setCursor(4, 3);
    lcd.print("Error al Leer");
    while (1)
      ;
  }
  if (dirIndex == 0) {
    lcd.setCursor(2, 3);
    lcd.print("                    ");
    lcd.setCursor(2, 3);
    lcd.print("No hay Archivos");
  }

  serializeJson(doc, response);

  sendResponsePlain(client, "HTTP/1.1 200 OK", response);
}

void sendResponsePlain(Client& client, const String& status, const String& message) {
  client.println(status);
  client.println("Content-Type: text/plain");
  client.println("Connection: close");
  client.println();
  client.println(message);
  isProcessing = true;
  requestPending = true;
}

// ----------------------------------------------------------------------------------------

void moveSteps_OG(int steps, float frequency, bool direction) {
  long pasossubida, pasosbajada, pasosresto;
  float vel;

  Breakout.digitalWrite(pwmPins[dirPin], direction ? HIGH : LOW);

  if (frequency > freq_ang_max) {
    pasossubida = 30 * steps / 100.0;
    pasosbajada = 30 * steps / 100.0;
    pasosresto = steps - pasossubida - pasosbajada;
    dvs = (frequency - freq_ang_max) / pasossubida;
    dvb = (frequency - freq_ang_max) / pasosbajada;
    vel = freq_ang_max;
  } else {
    pasossubida = 0;
    pasosbajada = 0;
    pasosresto = steps;
    dvs = 0;
    dvb = 0;
    vel = frequency;
  }

  float pulseDelay_v = 1000000 / (vel * steps * 2);

  for (int i = 0; i < pasossubida; i++) {

    if (checkLimitSwitch()) {
      stopMotion();
      limitSwitchDetected = true;
      return;
    }

    Breakout.digitalWrite(pwmPins[stepPin], HIGH);
    delayMicroseconds(pulseDelay_v);
    Breakout.digitalWrite(pwmPins[stepPin], LOW);
    delayMicroseconds(pulseDelay_v);

    vel = vel + dvs;

    if (checkLimitSwitch()) {
      stopMotion();
      limitSwitchDetected = true;
      return;
    }
  }

  for (int i = 0; i < pasosresto; i++) {

    if (checkLimitSwitch()) {
      stopMotion();
      limitSwitchDetected = true;
      return;
    }

    Breakout.digitalWrite(pwmPins[stepPin], HIGH);
    delayMicroseconds(pulseDelay_v);
    Breakout.digitalWrite(pwmPins[stepPin], LOW);
    delayMicroseconds(pulseDelay_v);

    if (checkLimitSwitch()) {
      stopMotion();
      limitSwitchDetected = true;
      return;
    }
  }

  for (int i = 0; i < pasosbajada; i++) {

    if (checkLimitSwitch()) {
      stopMotion();
      limitSwitchDetected = true;
      return;
    }

    Breakout.digitalWrite(pwmPins[stepPin], HIGH);
    delayMicroseconds(pulseDelay_v);
    Breakout.digitalWrite(pwmPins[stepPin], LOW);
    delayMicroseconds(pulseDelay_v);

    vel = vel - dvb;

    if (checkLimitSwitch()) {
      stopMotion();
      limitSwitchDetected = true;
      return;
    }
  }
}

void moveSteps(int steps, float frequency) {
  long pasossubida, pasosbajada, pasosresto;
  float vel;

  if (frequency > freq_ang_max) {
    pasossubida = 30 * steps / 100.0;
    pasosbajada = 30 * steps / 100.0;
    pasosresto = steps - pasossubida - pasosbajada;
    dvs = (frequency - freq_ang_max) / pasossubida;
    dvb = (frequency - freq_ang_max) / pasosbajada;
    vel = freq_ang_max;
  } else {
    pasossubida = 0;
    pasosbajada = 0;
    pasosresto = steps;
    dvs = 0;
    dvb = 0;
    vel = frequency;
  }

  float pulseDelay_v = 1000000 / (vel * steps * 2);

  Breakout.digitalWrite(pwmPins[dirPin], HIGH);
  for (int i = 1; i <= pasossubida; i++) {
    if (checkLimitSwitch()) {
      stopMotion();
      limitSwitchDetected = true;
      return;
    }

    Breakout.digitalWrite(pwmPins[stepPin], HIGH);
    delayMicroseconds(pulseDelay_v);
    Breakout.digitalWrite(pwmPins[stepPin], LOW);
    delayMicroseconds(pulseDelay_v);

    vel = vel + dvs;
  }

  for (int i = 1; i <= pasosresto; i++) {
    if (checkLimitSwitch()) {
      stopMotion();
      limitSwitchDetected = true;
      return;
    }

    Breakout.digitalWrite(pwmPins[stepPin], HIGH);
    delayMicroseconds(pulseDelay_v);
    Breakout.digitalWrite(pwmPins[stepPin], LOW);
    delayMicroseconds(pulseDelay_v);
  }

  for (int i = 1; i <= pasosbajada; i++) {
    if (checkLimitSwitch()) {
      stopMotion();
      limitSwitchDetected = true;
      return;
    }

    Breakout.digitalWrite(pwmPins[stepPin], HIGH);
    delayMicroseconds(pulseDelay_v);
    Breakout.digitalWrite(pwmPins[stepPin], LOW);
    delayMicroseconds(pulseDelay_v);

    vel = vel - dvb;
  }

  Breakout.digitalWrite(pwmPins[dirPin], LOW);
  for (int i = 1; i <= pasossubida; i++) {
    if (checkLimitSwitch()) {
      stopMotion();
      limitSwitchDetected = true;
      return;
    }

    Breakout.digitalWrite(pwmPins[stepPin], HIGH);
    delayMicroseconds(pulseDelay_v);
    Breakout.digitalWrite(pwmPins[stepPin], LOW);
    delayMicroseconds(pulseDelay_v);

    vel = vel + dvs;
  }

  for (int i = 1; i <= pasosresto; i++) {
    if (checkLimitSwitch()) {
      stopMotion();
      limitSwitchDetected = true;
      return;
    }

    Breakout.digitalWrite(pwmPins[stepPin], HIGH);
    delayMicroseconds(pulseDelay_v);
    Breakout.digitalWrite(pwmPins[stepPin], LOW);
    delayMicroseconds(pulseDelay_v);
  }

  for (int i = 1; i <= pasosbajada; i++) {
    if (checkLimitSwitch()) {
      stopMotion();
      limitSwitchDetected = true;
      return;
    }

    Breakout.digitalWrite(pwmPins[stepPin], HIGH);
    delayMicroseconds(pulseDelay_v);
    Breakout.digitalWrite(pwmPins[stepPin], LOW);
    delayMicroseconds(pulseDelay_v);

    vel = vel - dvb;
  }

  Breakout.digitalWrite(pwmPins[dirPin], LOW);
  for (int i = 1; i <= pasossubida; i++) {
    if (checkLimitSwitch()) {
      stopMotion();
      limitSwitchDetected = true;
      return;
    }

    Breakout.digitalWrite(pwmPins[stepPin], HIGH);
    delayMicroseconds(pulseDelay_v);
    Breakout.digitalWrite(pwmPins[stepPin], LOW);
    delayMicroseconds(pulseDelay_v);

    vel = vel + dvs;
  }

  for (int i = 1; i <= pasosresto; i++) {
    if (checkLimitSwitch()) {
      stopMotion();
      limitSwitchDetected = true;
      return;
    }

    Breakout.digitalWrite(pwmPins[stepPin], HIGH);
    delayMicroseconds(pulseDelay_v);
    Breakout.digitalWrite(pwmPins[stepPin], LOW);
    delayMicroseconds(pulseDelay_v);
  }

  for (int i = 1; i <= pasosbajada; i++) {
    if (checkLimitSwitch()) {
      stopMotion();
      limitSwitchDetected = true;
      return;
    }

    Breakout.digitalWrite(pwmPins[stepPin], HIGH);
    delayMicroseconds(pulseDelay_v);
    Breakout.digitalWrite(pwmPins[stepPin], LOW);
    delayMicroseconds(pulseDelay_v);

    vel = vel - dvb;
  }

  Breakout.digitalWrite(pwmPins[dirPin], HIGH);
  for (int i = 1; i <= pasossubida; i++) {
    if (checkLimitSwitch()) {
      stopMotion();
      limitSwitchDetected = true;
      return;
    }

    Breakout.digitalWrite(pwmPins[stepPin], HIGH);
    delayMicroseconds(pulseDelay_v);
    Breakout.digitalWrite(pwmPins[stepPin], LOW);
    delayMicroseconds(pulseDelay_v);

    vel = vel + dvs;
  }

  for (int i = 1; i <= pasosresto; i++) {
    if (checkLimitSwitch()) {
      stopMotion();
      limitSwitchDetected = true;
      return;
    }

    Breakout.digitalWrite(pwmPins[stepPin], HIGH);
    delayMicroseconds(pulseDelay_v);
    Breakout.digitalWrite(pwmPins[stepPin], LOW);
    delayMicroseconds(pulseDelay_v);
  }

  for (int i = 1; i <= pasosbajada; i++) {
    if (checkLimitSwitch()) {
      stopMotion();
      limitSwitchDetected = true;
      return;
    }

    Breakout.digitalWrite(pwmPins[stepPin], HIGH);
    delayMicroseconds(pulseDelay_v);
    Breakout.digitalWrite(pwmPins[stepPin], LOW);
    delayMicroseconds(pulseDelay_v);

    vel = vel - dvb;
  }
}

// ------------------------------

bool checkLimitSwitch() {
  return Breakout.digitalRead(pwmPins[limitSwitchPin_1]) == 0 || digitalRead(pwmPins[limitSwitchPin_2]) == 0;
}

void stopMotion() {
  int stop_pin_1 = Breakout.digitalRead(pwmPins[limitSwitchPin_1]);
  int stop_pin_2 = Breakout.digitalRead(pwmPins[limitSwitchPin_2]);
  functionActive = false;
  if (stop_pin_1 == 0 && stop_pin_2 == 1) {
    centrar();
  } else if (stop_pin_1 == 1 && stop_pin_2 == 0) {
    centrar();
  }
}

float distancia(int n) {
  long suma = 0;
  for (int i = 0; i < n; i++) {
    suma = suma + Breakout.analogRead(analogPins[5]);
  }
  float adc = suma / n;
  //float distancia_mm = pow(30274.0 / adc, 1.2134);
  return (adc);
}

void centrar() {
  int h1, h2, H, tope = 0;
  bool dir = true;

  lcd.clear();
  lcd.setCursor(5, 1);
  lcd.print("Centrando");
  lcd.setCursor(7, 2);
  lcd.print("Mesa");

  int delay = 500;

  do {
    h2 = distancia(20);

    if (dir) {
      tope = Breakout.digitalRead(pwmPins[limitSwitchPin_1]);
    } else {
      tope = Breakout.digitalRead(pwmPins[limitSwitchPin_2]);
    }

    if (tope == 0) {
      dir = !dir;
    }

    Breakout.digitalWrite(pwmPins[dirPin], dir ? HIGH : LOW);
    for (int i = 0; i < 10; i++) {
      Breakout.digitalWrite(pwmPins[stepPin], HIGH);
      delayMicroseconds(delay);
      Breakout.digitalWrite(pwmPins[stepPin], LOW);
      delayMicroseconds(delay);
    }
  } while (h2 < 570);

  // 0 -> derecha || 1 -> izquierda

  move(3000, 0.85, dir);

  do {
    h2 = distancia(20);

    if (dir) {
      tope = Breakout.digitalRead(pwmPins[limitSwitchPin_1]);
    } else {
      tope = Breakout.digitalRead(pwmPins[limitSwitchPin_2]);
    }

    if (tope == 0) {
      dir = !dir;
    }

    Breakout.digitalWrite(pwmPins[dirPin], dir ? HIGH : LOW);
    for (int i = 0; i < 10; i++) {
      Breakout.digitalWrite(pwmPins[stepPin], HIGH);
      delayMicroseconds(delay);
      Breakout.digitalWrite(pwmPins[stepPin], LOW);
      delayMicroseconds(delay);
    }
  } while (h2 < 580);

  move(stepsToCenter, 0.3, dir);

  limitSwitchDetected = false;
  functionActive = true;

  lcd.clear();
  lcd.setCursor(5, 1);
  lcd.print("Seleccionar");
  lcd.setCursor(8, 2);
  lcd.print("Modo");
}

void move(int steps, float frequency, bool direction) {
  long pasossubida, pasosbajada, pasosresto;
  float vel;

  Breakout.digitalWrite(pwmPins[dirPin], direction ? HIGH : LOW);

  if (frequency > freq_ang_max) {
    pasossubida = 30 * steps / 100.0;
    pasosbajada = 30 * steps / 100.0;
    pasosresto = steps - pasossubida - pasosbajada;
    dvs = (frequency - freq_ang_max) / pasossubida;
    dvb = (frequency - freq_ang_max) / pasosbajada;
    vel = freq_ang_max;
  } else {
    pasossubida = 0;
    pasosbajada = 0;
    pasosresto = steps;
    dvs = 0;
    dvb = 0;
    vel = frequency;
  }

  float pulseDelay_v = 1000000 / (vel * steps * 2);

  for (int i = 0; i < pasossubida; i++) {
    Breakout.digitalWrite(pwmPins[stepPin], HIGH);
    delayMicroseconds(pulseDelay_v);
    Breakout.digitalWrite(pwmPins[stepPin], LOW);
    delayMicroseconds(pulseDelay_v);

    vel = vel + dvs;
  }

  for (int i = 0; i < pasosresto; i++) {
    Breakout.digitalWrite(pwmPins[stepPin], HIGH);
    delayMicroseconds(pulseDelay_v);
    Breakout.digitalWrite(pwmPins[stepPin], LOW);
    delayMicroseconds(pulseDelay_v);
  }

  for (int i = 0; i < pasosbajada; i++) {
    Breakout.digitalWrite(pwmPins[stepPin], HIGH);
    delayMicroseconds(pulseDelay_v);
    Breakout.digitalWrite(pwmPins[stepPin], LOW);
    delayMicroseconds(pulseDelay_v);

    vel = vel - dvb;
  }
}
