// #include "arduino_secrets.h"

/*
Por Lucio Estacio, diciembre del 2018
lestaciof@uni.pe
*/
// #include <LiquidCrystal.h>
#include <LiquidCrystal_I2C.h>
#include <SD.h>

#define STEP_PIN 3  // Pin de pulso
#define DIRE_PIN 4  // Pin de direccion
#define FREE_PIN 2  // Pin de direccion

int VUELTAS = 3;
int PASOS = 400;                       // Numero de pasos del driver del motor
int MMXVUELTA = 10;                    // mm de avance por revolucion
int psubida = 30;                      // Porcentaje de subida
int pbajada = 30;                      // Porcentaje de bajada
int vo = 60;                           // Velocidad de inicio maximo
float dvs, dvb;                        // Segmentos de subida y bajada
const int SwitchPin = 13;              // Switch para modo manual o PC
String cadena;                         // Cadena para mensajes en pantalla
int SwitchStatus = 0;                  // Estado del switch, defecto manual 0
int screen1 = 0, screen2 = 0;          // Modos de pantalla en LCD
long p_speed, p_dist;                  // Valores de perillas de velocidad y distancia
float TotalLen = VUELTAS * MMXVUELTA;  // Longitud total en mm del equipo
long s_left, s_right;                  // Sensores de fin de carrera
int leefile = 0;                       // Indicador de lectura de archivos en MicroSD
float p = 0;                           // Punto temporal de seÃ±al
int h_max, h_min, resol = 5;           // Altura minimo y maximo del sensor
int pasocentro = 10;                   // MM de paso para recentrar
int totalpc = 0, Maxtotalpc = 6;       // Maxima iteracion para buscar el centro
int flagmitad = 0;                     // Controla inicar con la mitad de recorrido al inicio
float m_dist_old = 0, mmsin = 0.5;     // Distancia antigua para correr la mitad y resolucion de diferencias
File myFile;                           // Variable archivo del sismo

//Pantalla liquida
//LiquidCrystal lcd(4, 5, 6, 10, 11, 12);
LiquidCrystal_I2C lcd(0x27, 20, 4);
//LiquidCrystal lcd(12,11,10,6,5,4);
#define COLS 16
#define ROWS 2

void setup() {
  Serial.begin(9600);

  //Pines del stepper motor
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIRE_PIN, OUTPUT);
  pinMode(FREE_PIN, OUTPUT);

  digitalWrite(FREE_PIN, HIGH);

  //Pantalla LCD
  //analogWrite(6, 20);
  //lcd.begin(COLS, ROWS);
  lcd.init();
  lcd.backlight();

  //Switch de modo
  pinMode(SwitchPin, INPUT);
  cadena = String();

  pinMode(8, INPUT_PULLUP);
  pinMode(9, INPUT_PULLUP);

  pantalla("MESA VIBRADORA", "PROPIEDAD DE");
  delay(2000);
  pantalla("NUEVO CONTROL", "EIRL");
  delay(3000);
  pantalla("www.ncn.pe", "informes@ncn.pe");
  delay(3000);

  pantalla("Centrando la", "plataforma ...");
  totalpc = 0;
  escaneacentro();
}

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
    s_left = digitalRead(8);
    //Serial.println("Iz:"+String(s_left));
    if (s_left == 0) {
      pantalla("ERROR #001", "Limite riel 1");
      delay(4000);
      reposicion();
      op = 1;
    }
    //s_right = analogRead(A3);
    s_right = digitalRead(9);
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
      digitalWrite(DIRE_PIN, HIGH);
    else
      digitalWrite(DIRE_PIN, LOW);

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
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(stepDelay);
  digitalWrite(STEP_PIN, LOW);
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

// Funcion que lee distancia de sensor
float distancia(int n) {
  long suma = 0;
  for (int i = 0; i < n; i++) {
    suma = suma + analogRead(A3);
  }
  float adc = suma / n;
  float distancia_mm = pow(30274.0 / adc, 1.2134);
  return (distancia_mm);
}

// Donde estoy
int dondeestoy(int d) {
  int estoy = 0;  // 0 = izquierda, 1 = derecha
  if ((d > (h_min - resol)) && (d < (h_min + resol)))
    estoy = 0;
  if ((d > (h_max - resol)) && (d < (h_max + resol)))
    estoy = 1;
  return estoy;
}

//Escanea centro
void escaneacentro() {
  int X = 0, h1, h2, tope = 0, sentido = 1, H, i;
  h1 = distancia(500);
  do {
    if (sentido == 1)
      tope = digitalRead(7);
    else
      tope = digitalRead(8);
    if (tope == 0) {
      sentido = -1 * sentido;
    }
    avanza(sentido * pasocentro, 10, 0);
    h2 = distancia(500);
    H = abs(h1 - h2);
  } while (H < 2 * resol);
  h_max = h1;
  h_min = h2;
  if (h2 > h_max) {
    h_max = h2;
    h_min = h1;
  }
  //ponecentro();
  sentido = -1 * sentido;
  for (i = 0; i < 10; i++) {
    if (pasocentro > 2)
      pasocentro = pasocentro - 2;
    else
      pasocentro = 1;
    avanza(sentido * pasocentro, 10, 0);
    sentido = -1 * sentido;
  }
}

// Funcion loop del programa
void loop() {
  // Chequeo estado de switch para modos de operacion
  SwitchStatus = digitalRead(SwitchPin);
  if (SwitchStatus == 1) {
    if (screen1 == 0) {
      Serial.println("Modo Manual");
      pantalla("Modo:", "Manual");
      screen1 = 1;
      screen2 = 0;
      delay(2000);
    }
    modomanual();
  } else {
    if (screen2 == 0) {
      Serial.println("Modo Computadora");
      pantalla("Modo:", "Computadora");
      screen2 = 1;
      screen1 = 0;
      delay(2000);
    }
    modocomputadora();
  }
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
  escaneacentro();
  delay(1000);
}

// Funcion de modo de operacion manual
void modomanual() {
  float frec, myv, m_dist, max_velo, m_speed;
  unsigned long mytime;

  mytime = micros();
  p_dist = analogRead(A0);   // Distancia analoga de perilla
  p_speed = analogRead(A1);  // Velocidad analoga de perilla

  m_dist = mapf(p_dist, 30, 1015, 0, TotalLen);
  max_velo = getMaxVel(m_dist);  // Extrae velocidad maxima segun distancia a recorrer
  m_speed = mapf(p_speed, 20, 1015, 0, max_velo);
  if (m_dist > 0) {
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

// Funcion de modo de operacion por computadora
void modocomputadora() {
  float segundos, mydesp, myvelo;  // Desplazamiento y velocidad
  char mdesp[8], mvelo[8];         // Variables para convertir cadenas a numeros
  int i = 0;
  unsigned long mytime;

  if (leefile == 0) {
    pantalla("Iniciando", "memoria interna");
    if (!SD.begin(53)) {
      pantalla("ERROR #003", "En MicroSD");
      return;
    }
    pantalla("Inicio exitoso", "abriendo archivo");
    myFile = SD.open("shake.csv");
    delay(100);  // Una pausa para acceder al MicroSD
    mytime = micros();
    if (myFile) {
      pantalla("Leyendo datos", "desde archivo");
      pantalla("Reproduciendo", "registro sismico");
      while (myFile.available()) {
        leelinea(mdesp);
        mydesp = atof(mdesp);
        leelinea(mvelo);
        myvelo = atof(mvelo);

        RunSismo(mydesp, myvelo);
        i++;
      }
      myFile.close();
      segundos = (micros() - mytime) / 1000000.0;
      leefile = 1;
    } else {
      pantalla("ERROR #004", "Abrir archivo");
    }
  }
}

String leelinea(char o[8]) {
  char c;
  int i = 0, t = 1;
  o[0] = '\x0';
  do {
    c = myFile.read();
    if (c != ',' && c != '\n') {
      o[i] = c;
      i++;
    } else {
      o[i] = '\x0';
      t = 0;
      break;
    }
  } while (t);
  return o;
}