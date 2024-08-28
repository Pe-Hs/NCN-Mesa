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


// Configuración de la tarjeta SD
SDMMCBlockDevice sd;
mbed::FATFileSystem fs("fs");

LiquidCrystal_I2C lcd(0x27, 20, 4);

// ---------------------------------------------------------------

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

// ----------------------------------------------------------------

breakoutPin pwmPins[] = { PWM0, PWM1, PWM2, PWM3, PWM4, PWM5, PWM6, PWM7, PWM8, PWM9 };
breakoutPin analogPins[] = { ANALOG_A0, ANALOG_A1, ANALOG_A2, ANALOG_A3, ANALOG_A4, ANALOG_A5, ANALOG_A6, ANALOG_A7 };
breakoutPin gpioPins[] = { GPIO_0, GPIO_1, GPIO_2, GPIO_3, GPIO_4, GPIO_5, GPIO_6 };

byte dirPin = 7;
byte stepPin = 8;
byte freePin = 9;

byte potAmp = 7;
byte potFre = 6;

byte limitSwitchPin_1 = 4;
byte limitSwitchPin_2 = 3;

byte SwitchPin = 5;

int VUELTAS = 3;
int PASOS = 500;                       // Numero de pasos del driver del motor
int MMXVUELTA = 10;                    // mm de avance por revolucion
int psubida = 30;                      // Porcentaje de subida
int pbajada = 30;                      // Porcentaje de bajada
int vo = 60;                           // Velocidad de inicio maximo
float dvs, dvb;                        // Segmentos de subida y bajada
int SwitchStatus = 0;                  // Estado del switch, defecto manual 0
int screen1 = 0, screen2 = 0;          // Modos de pantalla en LCD
long p_speed, p_dist;                  // Valores de perillas de velocidad y distancia
float TotalLen = VUELTAS * MMXVUELTA;  // Longitud total en mm del equipo
long s_left, s_right;                  // Sensores de fin de carrera
int leefile = 0;                       // Indicador de lectura de archivos en MicroSD
float p = 0;                           // Punto temporal de seÃ±al
int h_max, h_min, resol = 5;           // Altura minimo y maximo del sensor
int pasocentro = 10;                   // MM de paso para recentrar
int totalpc = 0, Maxtotalpc = 6;       // Maxima iteracion para buscar el centro  ---* NO SE USA
int flagmitad = 0;                     // Controla inicar con la mitad de recorrido al inicio
float m_dist_old = 0, mmsin = 0.5;     // Distancia antigua para correr la mitad y resolucion de diferencias

//File myFile;    // Variable archivo del sismo
String cadena;  // Cadena para mensajes en pantalla
// ----------------------------------------------------------------------------------------------------------

const float distCentro = 25.0;              // Distancia hacia el centro en mm
int stepsToCenter = distCentro * 50;        // Pasos hacia el centro
const float freq_ang_max = 3.0 * (2 * PI);  // Frecuencia max (Velocidad Angular) 6 -> 2 * PI

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

// Variables de estado
bool isProcessing = false;
bool stopRequestReceived = false;  // Bandera global

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

  //Pines del stepper motor
  pinMode(pwmPins[dirPin], OUTPUT);
  pinMode(pwmPins[stepPin], OUTPUT);
  pinMode(pwmPins[freePin], OUTPUT);

  Breakout.digitalWrite(pwmPins[freePin], HIGH);

  pinMode(pwmPins[SwitchPin], INPUT_PULLUP);

  pinMode(pwmPins[limitSwitchPin_1], INPUT_PULLUP);
  pinMode(pwmPins[limitSwitchPin_2], INPUT_PULLUP);

  lcd.init();
  lcd.backlight();

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

  // cadena = String();

  centrar_texto(1, "MESA VIBRADORA");
  centrar_texto(2, "PROPIEDAD DE");
  delay(2000);
  lcd.clear();

  centrar_texto(1, "NUEVO CONTROL");
  centrar_texto(2, "EIRL");
  delay(3000);
  lcd.clear();

  centrar_texto(1, "www.ncn.pe");
  centrar_texto(2, "informes@ncn.pe");
  delay(3000);
  lcd.clear();

  centrar_texto(1, "CENTRANDO LA");
  centrar_texto(2, "PLATAFORMA");
  centrar();

  lcd.clear();
  centrar_texto(1, "Seleccionar");
  centrar_texto(2, "Modo");
}

// Funcion loop del programa
void loop() {
  int buttonState = Breakout.digitalRead(pwmPins[SwitchPin]);

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
        modomanual();
      } else if (clickCount == 1) {
        lcd.clear();
        Serial.println("WIFI");
        modoWifi();
      } else if (clickCount == 2) {
        lcd.clear();
        Serial.println("USB");
      } else if (clickCount == 3) {
        lcd.clear();
        Serial.println("UTP");
        modoEthernet();
      }
    }
  }
}

// --------------------------------------------------------

void modomanual() {
  float frec, myv, m_dist, max_velo, m_speed, n_p_dist, n_p_speed;
  unsigned long mytime;

  while (functionActive) {

    int buttonState = Breakout.digitalRead(pwmPins[SwitchPin]);

    if (buttonState == HIGH && lastButtonState == LOW) {
      functionActive = false;
      lcd.clear();
      centrar_texto(1, "Deteniendo");
      centrar_texto(2, "Modo");
      centrar();
      lcd.clear();
      centrar_texto(1, "Seleccionar");
      centrar_texto(2, "Modo");
      break;
    }

    lastButtonState = buttonState;

    mytime = micros();
    p_dist = Breakout.analogRead(analogPins[potAmp]);   // analogRead(A0);   // Distancia analoga de perilla
    p_speed = Breakout.analogRead(analogPins[potFre]);  // analogRead(A1);  // Velocidad analoga de perilla

    n_p_dist = reduce_noise(p_dist, 100);
    n_p_speed = reduce_noise(p_speed, 100);

    m_dist = mapf(int(n_p_dist), 30, 1015, 0, TotalLen);
    max_velo = getMaxVel(m_dist);  // Extrae velocidad maxima segun distancia a recorrer
    m_speed = mapf(int(n_p_speed), 20, 1015, 0, max_velo);
    Serial.print(p_dist);
    Serial.print(" ----- ");
    Serial.print(n_p_dist);
    Serial.print(" ----- ");
    Serial.print(m_speed);
    Serial.print(" ----- ");
    Serial.println(max_velo);
    if (m_dist > 0 && m_speed > 0) {
      RunSin(m_dist, m_speed);
      mytime = micros() - mytime;
      frec = 1000000.0 / mytime;
      pantalla("Dist: " + String(m_dist, 2) + "mm", "Frec: " + String(frec, 2) + "Hz");
    } else {
      flagmitad = 0;
      pantalla("Dist: 00.00mm", "Frec: 00.00Hz");
      delay(500);
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

    int buttonState = Breakout.digitalRead(pwmPins[SwitchPin]);

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

        if (isProcessing) {
          stopRequestReceived = true;
        }

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

    int buttonState = Breakout.digitalRead(pwmPins[SwitchPin]);

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
      lcd.setCursor(1, 3);
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

        if (isProcessing) {
          stopRequestReceived = true;
        }

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

// --------------------------------------------------------

void handlePOST(Client& client, const String& postData) {

  if (postData.startsWith("------WebKitFormBoundary") || postData.indexOf("Content-Disposition: form-data") != -1) {
    sendResponsePlain(client, "HTTP/1.1 200 OK", "Datos Recibidos Correctamente");  // postData

    String newPost = postData;
    String content_filename = extractFilename(postData);

    for (int i = 0; i < 5; i++) {
      int newlineIndex = newPost.indexOf('\n');
      if (newlineIndex == -1) {
        return;
      }
      newPost = newPost.substring(newlineIndex + 1);
    }

    String boundaryMarker = "\r\n--";

    int boundaryIndex = newPost.lastIndexOf(boundaryMarker);

    if (boundaryIndex != -1) {
      newPost = newPost.substring(0, boundaryIndex);
    }

    String filePath = "/fs/" + content_filename;

    FILE* file = fopen(filePath.c_str(), "ab");

    if (file) {
      fwrite(newPost.c_str(), 1, newPost.length(), file);
      fclose(file);
      sendResponsePlain(client, "HTTP/1.1 200 OK", "Archivos Escritos");
    } else {
      sendResponsePlain(client, "HTTP/1.1 500 Internal Server Error", "Error al escribir el archivo");
    }

  } else if (postData.startsWith("\n{")) {

    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, postData);

    if (error) {
      sendResponsePlain(client, "HTTP/1.1 200 OK", "Datos Recibidos Correctamente");
      return;
    }

    if (doc["amp"] && doc["freq"]) {

      float amp = doc["amp"];
      float freq = doc["freq"];

      sendResponsePlain(client, "HTTP/1.1 200 OK", "Datos Recibidos Correctamente");
      isProcessing = true;
      processLoop(client, amp, freq);
      return;
    }

    if (doc["filename"] && doc["limit"]) {

      String filename = doc["filename"];
      size_t limit = doc["limit"];
      size_t startIndex = doc["index"] ? doc["index"] : 0;

      String filePath = String("/fs/") + filename;
      FILE* file = fopen(filePath.c_str(), "r");

      if (file) {
        fseek(file, startIndex, SEEK_SET);

        char buffer[limit + 1];
        size_t bytesRead = fread(buffer, 1, limit, file);
        buffer[bytesRead] = '\0';

        DynamicJsonDocument doc(1024);
        doc["fileName"] = filename;
        doc["startIndex"] = startIndex;
        doc["endIndex"] = startIndex + bytesRead;
        doc["data"] = buffer;

        String jsonResponse;
        serializeJson(doc, jsonResponse);
        sendResponsePlain(client, "HTTP/1.1 200 OK", jsonResponse);
      }

      fclose(file);

      return;
    }

    if (doc["filename"] && doc["file"]) {
      String filename = doc["filename"];
      String filePath = "/fs/" + filename;

      FILE* file = fopen(filePath.c_str(), "wb");

      fwrite(postData.c_str(), 1, postData.length(), file);

      sendResponsePlain(client, "HTTP/1.1 200 OK", "Archivos Escritos");

      fclose(file);
    }

    if (doc["filename"] && doc["action"]) {
      String filename = doc["filename"];
      String action = doc["action"];
      String filePath = "/fs/" + filename;

      if (remove(filePath.c_str()) == 0 && action == "delete") {
        sendResponsePlain(client, "HTTP/1.1 200 OK", "Archivo Borrado");
      } else {
        sendResponsePlain(client, "HTTP/1.1 500 Internal Server Error", "Error al Borrar Archivo");
      }
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

        String filePath = String("/fs/") + ent->d_name;

        FILE* file = fopen(filePath.c_str(), "r");

        if (file) {
          fseek(file, 0, SEEK_END);
          long fileSize = ftell(file);
          fseek(file, 0, SEEK_SET);
          fclose(file);

          JsonObject fileObject = filesArray.createNestedObject();

          fileObject["filename"] = ent->d_name;
          fileObject["size"] = fileSize;
        } else {
          lcd.setCursor(2, 3);
          lcd.print("                    ");
          lcd.setCursor(1, 3);
          lcd.print("Error al Abrir Archivo");
        }

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
}

// --------------------------------------------------------

String extractFilename(String contentDisposition) {
  String searchStr = "filename=\"";
  int startIndex = contentDisposition.indexOf(searchStr);
  if (startIndex == -1) {
    return "";
  }

  startIndex += searchStr.length();
  int endIndex = contentDisposition.indexOf("\"", startIndex);

  if (endIndex == -1) {
    return "";
  }

  return contentDisposition.substring(startIndex, endIndex);
}

void processLoop(Client& client, float amplitud, float velocidad) {
  float frec, myv, m_dist, max_velo, m_speed;
  unsigned long mytime;
  unsigned long lastCheckTime = millis();

  while (isProcessing) {

    if (millis() - lastCheckTime >= 100) {
      lastCheckTime = millis();

      EthernetClient newEthernetClient = server.available();
      if (newEthernetClient) {
        isProcessing = false;
        //client.stop();
        return;
      }

      WiFiClient newWiFiClient = wi_server.available();
      if (newWiFiClient) {
        isProcessing = false;
        //client.stop();
        return;
      }
    }

    int buttonState = Breakout.digitalRead(pwmPins[SwitchPin]);

    if (buttonState == HIGH && lastButtonState == LOW) {
      isProcessing = false;
      lcd.setCursor(0, 2);
      lcd.print("                    ");
      centrar();
      return;
    }

    lastButtonState = buttonState;

    mytime = micros();
    p_dist = amplitud;
    p_speed = velocidad;

    m_dist = mapf(p_dist, 30, 1015, 0, TotalLen);
    max_velo = getMaxVel(m_dist);
    m_speed = mapf(p_speed, 20, 1015, 0, max_velo);

    if (m_dist > 0) {
      RunSin(m_dist, m_speed);
      mytime = micros() - mytime;
      frec = 1000000.0 / mytime;
      pantalla_loop(String(m_dist, 2) + "mm", String(frec, 2) + "Hz");
    } else {
      flagmitad = 0;
      pantalla_loop("00.00mm ", " 00.00Hz");
      delay(500);
    }
  }
}

// --------------------------------------------------------

// Funcion RunSin que hace movimiento de ida y vuelta
void RunSin(float dist, int v) {
  float tmp_dist, o_tmp_dist, v_dist;

  tmp_dist = abs(dist - m_dist_old);
  //Serial.println("resol:"+String(tmp_dist));
  if (tmp_dist > mmsin) {
    o_tmp_dist = m_dist_old / 2.0;
    m_dist_old = dist;
    tmp_dist = dist / 2.0;
    if (flagmitad == 0) {
      avanza(tmp_dist, 10, 1);
      flagmitad = 1;
    } else {
      v_dist = o_tmp_dist + tmp_dist;
      avanza(v_dist, v, 1);
    }
  } else
    avanza(dist, v, 1);
  delay(10);
  avanza(-1.0 * dist, v, 1);
  delay(10);
}

// Funcion que ejecuta una seÃ±al de sismo
void RunSismo(float mydesp, float myvelo) {
  float newdesp;
  newdesp = mydesp - p;
  avanza(newdesp, myvelo, 1);
  p = mydesp;
}

// Funcion de avance de carro a una distancia y velocidad especifica
// Con aceleracion y desaceleracion
void avanza(float dist, float velocidad, int seg) {
  long pasostotales = abs(dist) * PASOS / MMXVUELTA;
  long pasossubida, pasosbajada, pasosresto;
  int i, op = 0;
  float vel;

  // Chequea sensores de fin de carrera por seguridad solo si seg = 1
  if (seg == 1) {
    // s_left = analogRead(A2);
    s_left = Breakout.digitalRead(pwmPins[limitSwitchPin_1]);
    //Serial.println("Iz:"+String(s_left));
    if (s_left == 0) {
      pantalla("ERROR #001", "Limite riel 1");
      delay(4000);
      reposicion();
      op = 1;
    }
    //s_right = analogRead(A3);
    s_right = Breakout.digitalRead(pwmPins[limitSwitchPin_2]);
    //Serial.println("De:"+String(s_right));
    if (s_right == 0) {
      pantalla("ERROR #002", "Limite riel 2");
      delay(4000);
      reposicion();
      op = 1;
    }
  }
  //Determina direccion del movimiento + o -
  if (op == 0) {
    if (dist > 0)
      Breakout.digitalWrite(pwmPins[dirPin], HIGH);
    else
      Breakout.digitalWrite(pwmPins[dirPin], LOW);

    //Determina variacion de velocidad
    if (velocidad > vo) {
      //Determinacion de pasos de aceleracion y desaceleracion
      pasossubida = psubida * pasostotales / 100.0;
      pasosbajada = pbajada * pasostotales / 100.0;
      pasosresto = pasostotales - pasossubida - pasosbajada;
      dvs = (velocidad - vo) / pasossubida;
      dvb = (velocidad - vo) / pasosbajada;
      vel = vo;
    } else {
      pasossubida = 0;
      pasosbajada = 0;
      pasosresto = pasostotales;
      dvs = 0;
      dvb = 0;
      vel = velocidad;
    }

    //Aceleracion del motor
    for (i = 1; i <= pasossubida; i++) {
      step(vel);
      //Serial.println(velocidad);
      vel = vel + dvs;
    }

    //Velocidad constante
    for (i = 1; i <= pasosresto; i++) {
      step(vel);
      //Serial.println(velocidad);
    }

    //Desaceleracion del motor
    for (i = 1; i <= pasosbajada; i++) {
      step(vel);
      //Serial.println(velocidad);
      vel = vel - dvb;
    }
  }
}

// Funcion que da un paso a una velocidad especifica
void step(long speed) {
  int stepDelay = 1000000 * MMXVUELTA / (2 * PASOS * speed);
  Breakout.digitalWrite(pwmPins[stepPin], HIGH);
  delayMicroseconds(stepDelay);
  Breakout.digitalWrite(pwmPins[stepPin], LOW);
  delayMicroseconds(stepDelay);
}

// Funcion que muestra en pantalla un mensaje
void pantalla(String msg1, String msg2) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(msg1);
  lcd.setCursor(0, 1);
  lcd.print(msg2);
}

void pantalla_loop(String msg1, String msg2) {
  lcd.setCursor(0, 2);
  lcd.print("                    ");
  lcd.setCursor(2, 2);
  lcd.print(msg1);
  lcd.setCursor(11, 2);
  lcd.print(msg2);
}

void centrar_texto(int fila, String texto) {
  int largoTexto = texto.length();
  int columnasLCD = 20;
  int espacio = (columnasLCD - largoTexto) / 2;

  lcd.setCursor(espacio, fila);
  lcd.print(texto);
}

// Funcion que lee distancia de sensor
float distancia(int n) {
  long suma = 0;
  for (int i = 0; i < n; i++) {
    suma = suma + Breakout.analogRead(analogPins[5]);
  }
  float adc = suma / n;
  // float distancia_mm = pow(30274.0 / adc, 1.2134);
  return (adc);
}

float reduce_noise(int value, int n) {
  long suma = 0;
  for (int i = 0; i < n; i++) {
    suma = suma + value;
  }
  float adc = suma / n;
  // float distancia_mm = pow(30274.0 / adc, 1.2134);
  return (adc);
}

//Escanea centro
void centrar() {
  int h1, h2, H, tope = 0;
  bool dir = true;

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

  move(2500, 0.85, dir);

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
}

// Funcion de mapeo de valor flotante
double mapf(double val, double in_min, double in_max, double out_min, double out_max) {
  return (val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Funcion que calcula la velocidad maxima permitida segun el desplazamiento
float getMaxVel(float dist) {
  float y = (0.00159962) * pow(dist, 3) - 0.41303215 * pow(dist, 2) + 20.83780711 * dist + 103.82640002;
  return y;
}

// Funcion que reposiciona la mesa al inicio
void reposicion() {
  pantalla("Reposicionando", "plataforma ...");
  centrar();
  delay(1000);
}

// Funcion de modo de operacion por computadora
// void modocomputadora() {
//   float segundos, mydesp, myvelo;  // Desplazamiento y velocidad
//   char mdesp[8], mvelo[8];         // Variables para convertir cadenas a numeros
//   int i = 0;
//   unsigned long mytime;

//   if (leefile == 0) {
//     pantalla("Iniciando", "memoria interna");
//     if (!SD.begin(53)) {
//       pantalla("ERROR #003", "En MicroSD");
//       return;
//     }
//     pantalla("Inicio exitoso", "abriendo archivo");
//     myFile = SD.open("test.txt");
//     delay(100);  // Una pausa para acceder al MicroSD
//     mytime = micros();
//     if (myFile) {
//       pantalla("Leyendo datos", "desde archivo");
//       pantalla("Reproduciendo", "registro sismico");
//       while (myFile.available()) {
//         leelinea(mdesp);
//         mydesp = atof(mdesp);
//         leelinea(mvelo);
//         myvelo = atof(mvelo);

//         RunSismo(mydesp, myvelo);
//         i++;
//       }
//       myFile.close();
//       segundos = (micros() - mytime) / 1000000.0;
//       leefile = 1;
//     } else {
//       pantalla("ERROR #004", "Abrir archivo");
//     }
//   }
// }

// String leelinea(char o[8]) {
//   char c;
//   int i = 0, t = 1;
//   o[0] = '\x0';
//   do {
//     c = myFile.read();
//     if (c != ',' && c != '\n') {
//       o[i] = c;
//       i++;
//     } else {
//       o[i] = '\x0';
//       t = 0;
//       break;
//     }
//   } while (t);
//   return o;
// }

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
