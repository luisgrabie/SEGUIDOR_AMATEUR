// ===============================
// CONFIGURACIÓN PRINCIPAL
// ===============================
#define TIPO_LINEA  1      // 0 = línea negra fondo blanco
                           // 1 = línea blanca fondo negro
#define VELOCIDAD   100    // 0-255, velocidad base

// PID normalizado (error va de -1.0 a +1.0)
float Kp = 1.2;
float Kd = 1.8;
float Ki = 0.0;

// ===============================
// PINES MUX + LED
// ===============================
const byte S0      = A3;
const byte S1      = A4;
const byte S2      = A5;
const byte MUX_OUT = A0;
const byte LED_IZ  = A2;
const byte LED_DR  = A1;

// ===============================
// PINES MOTOR TB6612
// ===============================
#define PWMA 3
#define AIN1 5
#define AIN2 4
#define STBY 6
#define BIN1 8
#define BIN2 7
#define PWMB 11

// ===============================
// VARIABLES INTERNAS
// ===============================
const int TH = 650;
const int pos_sensor[8] = { -4,-3,-2,-1,1,2,3,4 };
int sensores[8];

float error         = 0;
float errorAnterior = 0;
float integral      = 0;
float ultimaPosicion = 0;

// ===============================

void setMuxChannel(byte ch) {
  digitalWrite(S0, ch & 0x01);
  digitalWrite(S1, ch & 0x02);
  digitalWrite(S2, ch & 0x04);
}

// ===============================

// Devuelve posición normalizada: -1.0 (izquierda) a +1.0 (derecha)
float leerPosicion() {
  for (byte ch = 0; ch < 8; ch++) {
    setMuxChannel(ch);
    delayMicroseconds(8);
    analogRead(MUX_OUT);
    sensores[ch] = analogRead(MUX_OUT);
  }

  float sumaPos = 0;
  float cuenta  = 0;

  for (int i = 0; i < 8; i++) {
    bool detecta = (TIPO_LINEA == 0) ? sensores[i] > TH : sensores[i] < TH;
    if (detecta) {
      sumaPos += pos_sensor[i];
      cuenta++;
    }
  }

  float posicion;
  if (cuenta > 0) {
    posicion = sumaPos / cuenta;      // rango real: -4.0 a +4.0
    posicion = posicion / 4.0;        // normalizado: -1.0 a +1.0
    ultimaPosicion = posicion;
  } else {
    // Sin línea: mantiene último lado conocido
    posicion = (ultimaPosicion > 0) ? 1.0 : -1.0;
  }

  return posicion;
}

// ===============================

void moverMotores(int velIzq, int velDer) {
  // Motor izquierdo
  if (velIzq >= 0) {
    digitalWrite(AIN1, HIGH); digitalWrite(AIN2, LOW);
    analogWrite(PWMA, constrain(velIzq, 0, 255));
  } else {
    digitalWrite(AIN1, LOW);  digitalWrite(AIN2, HIGH);
    analogWrite(PWMA, constrain(-velIzq, 0, 255));
  }

  // Motor derecho
  if (velDer >= 0) {
    digitalWrite(BIN1, HIGH); digitalWrite(BIN2, LOW);
    analogWrite(PWMB, constrain(velDer, 0, 255));
  } else {
    digitalWrite(BIN1, LOW);  digitalWrite(BIN2, HIGH);
    analogWrite(PWMB, constrain(-velDer, 0, 255));
  }
}

// ===============================

void frenarMotores() {
  digitalWrite(AIN1, LOW); digitalWrite(AIN2, LOW); analogWrite(PWMA, 0);
  digitalWrite(BIN1, LOW); digitalWrite(BIN2, LOW); analogWrite(PWMB, 0);
}

// ===============================

void setup() {
  pinMode(S0, OUTPUT); pinMode(S1, OUTPUT); pinMode(S2, OUTPUT);
  pinMode(MUX_OUT, INPUT);
  pinMode(LED_IZ, OUTPUT); pinMode(LED_DR, OUTPUT);

  pinMode(PWMA, OUTPUT); pinMode(AIN1, OUTPUT); pinMode(AIN2, OUTPUT);
  pinMode(PWMB, OUTPUT); pinMode(BIN1, OUTPUT); pinMode(BIN2, OUTPUT);
  pinMode(STBY, OUTPUT);

  digitalWrite(STBY, HIGH);
  Serial.begin(115200);
}

// ===============================

void loop() {
  float posicion = leerPosicion();

  // Sin línea → frenar después de 200ms
  bool hayLinea = false;
  for (int i = 0; i < 8; i++) {
    bool detecta = (TIPO_LINEA == 0) ? sensores[i] > TH : sensores[i] < TH;
    if (detecta) { hayLinea = true; break; }
  }

  if (!hayLinea) {
    frenarMotores();
    return;
  }

  // PID normalizado
  error    = posicion;                                  // -1.0 a +1.0
  integral = constrain(integral + error, -1.0, 1.0);
  float derivada   = error - errorAnterior;
  float correccion = Kp*error + Ki*integral + Kd*derivada;  // -1.0 a ~+1.0

  errorAnterior = error;

  // Convertir corrección normalizada a PWM
  int ajuste = (int)(correccion * 255);               // escala a PWM
  ajuste = constrain(ajuste, -255, 255);

  int velIzq = VELOCIDAD + ajuste;
  int velDer = VELOCIDAD - ajuste;

  moverMotores(velIzq, velDer);
}
