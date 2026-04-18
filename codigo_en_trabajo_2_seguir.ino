// ===============================
// ===== PINES MUX + LED =====
// ===============================
//blanco
//negro
const byte S0 = A3;
const byte S1 = A4;
const byte S2 = A5;
const byte MUX_OUT = A0;
const byte LED_IZ = A2;
const byte LED_DR = A1;

// ===============================
// ===== PINES MOTOR TB6612 =====
// ===============================

#define PWMA  3
#define AIN1  5
#define AIN2  4
#define STBY  6
#define BIN1  8
#define BIN2  7
#define PWMB  11

// ===============================
// 0 = linea negra fondo blanco
// 1 = linea blanca fondo negro
// ===============================
unsigned long tiempoSinLinea = 0;
#define TIPO_LINEA 1

const int TH = 650;

int sensores[8];
const int pos_sensor[8] = { -4,-3,-2,-1,1,2,3,4 };

float pos_perdida = 5.5;
float posa = 0;

// =========================================
// PID + velocidades base
// KP  = volante del coche
// KD  = frena el volante
// -----------------------------------------
// Oscila en recta          → sube/baja KD o KP un solo valor
// Tarda en corregir        → falta KP
// Corrige pero se pasa     → sube KD
// No termina 180 grados    → baja umbralExtremo + sube Kp_extremo
// Va lento en recta        → sube vel_recta y compensa subiendo Kd_recta
// Ratios ideales KD/KP     → 1.3 a 2.0 (más → frenas/oscillas mucho)
// =========================================
float Ki = 0;

// Recta
float Kp_recta  = 120;//90
float Kd_recta  = 450;   // ratio 1.8/195
int   vel_recta = 100;//80

// Curva normal (45-90°)
float Kp_medio  = 730;//230
float Kd_medio  = 636;   // ratio 1.55//496
int   vel_media = 65;//60

// Curva 180° (semicírculo)
float Kp_extremo = 750;//380
float Kd_extremo = 920;//820
int   vel_extrema = 42;  // muy baja → no gira en inverso//40

// Umbrales de detección de curva
float umbralMedio   = 1.8;  // detecta curva antes
float umbralExtremo = 3.0;  // entra a 180° antes

// =========================================
// AMORTIGUADOR DE VELOCIDAD (error > 3)
// -----------------------------------------
// Cuando el error supera umbralExtremo, la
// velocidad baja proporcionalmente al exceso:
//   vel = vel_extrema * (1 - exceso * factorAmort)
// Limitado para no bajar de velMinCurva.
//
// Ajustes:
//   factorAmort  ↑  →  frena más agresivo
//   velMinCurva  ↑  →  más fuerza en giro cerrado
// =========================================
const float factorAmort = 0.15;  // 15 % de reducción por unidad de exceso
const int   velMinCurva = 30;    // velocidad mínima en curva extrema

// =========================================
float error = 0;
float errorAnterior = 0;
float integral = 0;

// =======================================

void setMuxChannel(byte ch){
  digitalWrite(S0, ch & 0x01);
  digitalWrite(S1, ch & 0x02);
  digitalWrite(S2, ch & 0x04);
}

// =======================================

float leerPosicion(){
  for(byte ch = 0; ch < 8; ch++){
    setMuxChannel(ch);
    delayMicroseconds(8);
    analogRead(MUX_OUT);               // lectura de descarte
    sensores[ch] = analogRead(MUX_OUT);
  }

  float sumaPos = 0;
  float cuenta  = 0;

  for(int i = 0; i < 8; i++){
    bool detecta;
    if(TIPO_LINEA == 0)
      detecta = sensores[i] > TH;
    else
      detecta = sensores[i] < TH;

    if(detecta){
      sumaPos += pos_sensor[i];
      cuenta++;
    }
  }

  float posLocal;
  if(cuenta > 0){
    posLocal = sumaPos / cuenta;
  } else {
    // Sin línea: proyecta hacia el último lado visto
    if(posa > 0)      posLocal =  pos_perdida;
    else if(posa < 0) posLocal = -pos_perdida;
    else              posLocal =  0;
  }

  posa = posLocal;
  return posLocal;
}

// =======================================

void moverMotores(int velIzq, int velDer){
  if(velIzq >= 0){
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);
    analogWrite(PWMA, constrain(velIzq, 0, 255));
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, constrain(-velIzq, 0, 255));
  }

  if(velDer >= 0){
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);
    analogWrite(PWMB, constrain(velDer, 0, 255));
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, constrain(-velDer, 0, 255));
  }
}

// =======================================

void frenarMotores(){
  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);
  analogWrite(PWMA, 0);
  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
  analogWrite(PWMB, 0);
}

// =======================================

void setup(){
  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);
  pinMode(MUX_OUT, INPUT);
  pinMode(LED_IZ, OUTPUT);
  pinMode(LED_DR, OUTPUT);

  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);

  digitalWrite(STBY, HIGH);
  Serial.begin(115200);
}

// =======================================

void loop(){

  // 1) Leer posición primero
  float posicion = leerPosicion();

  // 2) Verificar si hay línea
  bool hayLinea = false;
  for(int i = 0; i < 8; i++){
    bool detecta = (TIPO_LINEA == 0) ? sensores[i] > TH : sensores[i] < TH;
    if(detecta){ hayLinea = true; break; }
  }

  // 3) Sin línea → giro de búsqueda, sin PID
  if(!hayLinea){
    if(tiempoSinLinea == 0) tiempoSinLinea = millis();

    if(millis() - tiempoSinLinea > 200){
      frenarMotores();
      return;
    }

    int busqueda = 40;
    if(posa > 0) moverMotores( busqueda, -busqueda);
    else         moverMotores(-busqueda,  busqueda);
    delay(6);
    return;
  }

  tiempoSinLinea = 0;

  // 4) PID adaptativo con amortiguador de velocidad
  error = posicion;

  float Kp, Kd;
  int   velActual;

  if(abs(error) > umbralExtremo){

    Kp = Kp_extremo;
    Kd = Kd_extremo;

    // --- AMORTIGUADOR ---
    // Cuánto supera el umbral de 3.0
    float exceso = abs(error) - umbralExtremo;

    // Factor de reducción: máximo 50 % para no parar el robot
    float factor = 1.0 - constrain(exceso * factorAmort, 0.0, 0.5);

    velActual = (int)(vel_extrema * factor);

    // Garantiza fuerza mínima para completar el giro
    velActual = max(velActual, velMinCurva);

  } else if(abs(error) > umbralMedio){

    Kp        = Kp_medio;
    Kd        = Kd_medio;
    velActual = vel_media;

  } else {

    Kp        = Kp_recta;
    Kd        = Kd_recta;
    velActual = vel_recta;

  }

  // 5) Cálculo PID
  integral = constrain(integral + error, -50, 50);
  float derivada  = error - errorAnterior;
  float correccion = Kp*error + Kd*derivada + Ki*integral;
  correccion = constrain(correccion, -255, 255);

  errorAnterior = error;

  // 6) Aplicar corrección a cada motor
  int velIzq = velActual + correccion;
  int velDer = velActual - correccion;

  moverMotores(velIzq, velDer);
  //delay(6);
}