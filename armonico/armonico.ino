#include <SharpDistSensor.h>

// Define los pines
const int dirPin = 2;    // Pin para la dirección
const int stepPin = 3;   // Pin para el pulso
const int freePin = 13;  // Pin para el LED

const int limitSwitchPin_1 = 4;
const int limitSwitchPin_2 = 5;

// Parámetros del movimiento
const float amplitude = 10.0;                          // Amplitud en mm
const float freq_ang = 1.0 * (2 * PI);                 // Frecuencia (Velocidad Angular) 6 -> 2 * PI
const float freq_ang_max = 2.0 * (2 * PI);             // Frecuencia max (Velocidad Angular) 6 -> 2 * PI
const float stepsPerMM = 50.0;                         // Paso por Pasos por Revolucion (500 -> 360/0.72*) / 10 mm (distancia recorrida en una revolucion)
const int stepsForAmplitude = amplitude * stepsPerMM;  // Pasos correspondientes a la amplitud
const float distCentro = 25.0;                         // Distancia hacia el centro en mm
const int stepsToCenter = distCentro * stepsPerMM;     // Pasos hacia el centro


bool limitSwitchDetected = false;


const byte sensorPin = A4;
const byte medianFilterWindowSize = 5;
SharpDistSensor sensor(sensorPin, medianFilterWindowSize);

float dvs, dvb;  // Segmentos de subida y bajada
int h_max, h_min;


void setup() {
  Serial.begin(9600);

  pinMode(dirPin, OUTPUT);
  pinMode(stepPin, OUTPUT);
  pinMode(freePin, OUTPUT);

  pinMode(limitSwitchPin_1, INPUT_PULLUP);
  pinMode(limitSwitchPin_2, INPUT_PULLUP);

  digitalWrite(freePin, HIGH);

  sensor.setModel(SharpDistSensor::GP2Y0A51SK0F_5V_DS);
  centrar();
  delay(1000);
}

void loop() {

  //Serial.println(distancia(500));

  if (!limitSwitchDetected) {
    if (!checkLimitSwitch()) {
      moveSteps(stepsForAmplitude, freq_ang, true);
    }

    if (!checkLimitSwitch()) {
      moveSteps(stepsForAmplitude, freq_ang, false);
    }

    if (!checkLimitSwitch()) {
      moveSteps(stepsForAmplitude, freq_ang, false);
    }

    if (!checkLimitSwitch()) {
      moveSteps(stepsForAmplitude, freq_ang, true);
    }
  }
}

void moveSteps(int steps, float frequency, bool direction) {
  long pasossubida, pasosbajada, pasosresto;
  float vel;

  // float pulseDelay = 1000000 / (frequency * steps * 2);

  digitalWrite(dirPin, direction ? HIGH : LOW);

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
      Serial.println("A");
      limitSwitchDetected = true;
      return;
    }

    digitalWrite(stepPin, HIGH);
    delayMicroseconds(pulseDelay_v);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(pulseDelay_v);

    vel = vel + dvs;

    if (checkLimitSwitch()) {
      stopMotion();
      Serial.println("B");
      limitSwitchDetected = true;
      return;
    }
  }

  for (int i = 0; i < pasosresto; i++) {

    if (checkLimitSwitch()) {
      stopMotion();
      Serial.println("A");
      limitSwitchDetected = true;
      return;
    }

    digitalWrite(stepPin, HIGH);
    delayMicroseconds(pulseDelay_v);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(pulseDelay_v);

    if (checkLimitSwitch()) {
      stopMotion();
      Serial.println("B");
      limitSwitchDetected = true;
      return;
    }
  }

  for (int i = 0; i < pasosbajada; i++) {

    if (checkLimitSwitch()) {
      stopMotion();
      Serial.println("A");
      limitSwitchDetected = true;
      return;
    }

    digitalWrite(stepPin, HIGH);
    delayMicroseconds(pulseDelay_v);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(pulseDelay_v);

    vel = vel - dvb;

    if (checkLimitSwitch()) {
      stopMotion();
      Serial.println("B");
      limitSwitchDetected = true;
      return;
    }
  }
}

bool checkLimitSwitch() {
  return digitalRead(limitSwitchPin_1) == 0 || digitalRead(limitSwitchPin_2) == 0;
}

void stopMotion() {
  int stop_pin_1 = digitalRead(limitSwitchPin_1);
  int stop_pin_2 = digitalRead(limitSwitchPin_2);

  if (stop_pin_1 == 0 && stop_pin_2 == 1) {
    Serial.println("Detenido. Llendo a la Derecha");
    centrar();
  } else if (stop_pin_1 == 1 && stop_pin_2 == 0) {
    Serial.println("Detenido. Llendo a la Izquierda");
    centrar();
  }
}

float distancia(int n) {
  long suma = 0;
  sensor.getDist();
  for (int i = 0; i < n; i++) {
    suma = suma + analogRead(A4);
  }
  float adc = suma / n;
  float distancia_mm = pow(30274.0 / adc, 1.2134);
  return (distancia_mm);
}

// dir = 1 -> Izquierda || 0 -> Derecha
void centrar() {
  int h1, h2, H, tope = 0;
  bool dir = true;

  h1 = distancia(100);
  int delay = 50000;

  // digitalWrite(dirPin, dir ? HIGH : LOW);

  do {
    if (dir) {
      tope = digitalRead(limitSwitchPin_1);
    } else {
      tope = digitalRead(limitSwitchPin_2);
    }

    if (tope == 0) {
      dir = !dir;
    }

    digitalWrite(dirPin, dir ? HIGH : LOW);

    digitalWrite(stepPin, HIGH);
    delayMicroseconds(delay);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(delay);

    h2 = distancia(100);
    H = abs(h1 - h2);
    Serial.println(H);
  } while (H < 120);

  Serial.print("Fuera del While");

  h_max = h1;
  h_min = h2;

  if (h2 > h_max) {
    h_max = h2;
    h_min = h1;
  }

  move(stepsToCenter, 0.3, dir);
}

void move(int steps, float frequency, bool direction) {
  long pasossubida, pasosbajada, pasosresto;
  float vel;

  digitalWrite(dirPin, direction ? HIGH : LOW);

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
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(pulseDelay_v);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(pulseDelay_v);

    vel = vel + dvs;
  }

  for (int i = 0; i < pasosresto; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(pulseDelay_v);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(pulseDelay_v);
  }

  for (int i = 0; i < pasosbajada; i++) {
    digitalWrite(stepPin, HIGH);
    delayMicroseconds(pulseDelay_v);
    digitalWrite(stepPin, LOW);
    delayMicroseconds(pulseDelay_v);

    vel = vel - dvb;
  }
}
