
// ============================================================
//       SEGUIDOR DE LINEA 8 SENSORES + CD4051
//       VERSION ESTABLE PARA PRIMERAS PRUEBAS
// ============================================================
//
// D2:
//   Primer pulsado  -> calibracion
//   Segundo pulsado -> inicia el robot
//
// Durante la calibracion mueve la regleta sobre:
//   BLANCO + NEGRO
//
// ============================================================


// ============================================================
// ================= PINES MUX + LED ==========================
// ============================================================

const byte S0 = A3;
const byte S1 = A4;
const byte S2 = A5;

const byte MUX_OUT = A0;

const byte LED_IZ = A2;
const byte LED_DR = A1;


// ============================================================
// ================= PULSADOR ================================
// ============================================================

const byte BOTON = 2;


// ============================================================
// ================= PINES MOTOR TB6612 =======================
// ============================================================

#define PWMA 3
#define AIN1 5
#define AIN2 4

#define STBY 6

#define BIN1 8
#define BIN2 7
#define PWMB 11


// ============================================================
// ================= TIPO DE LINEA ============================
// ============================================================
//
// 0 = linea negra sobre fondo blanco
// 1 = linea blanca sobre fondo negro
//
// Tu codigo original utilizaba 1.
//

#define TIPO_LINEA 0


// ============================================================
// ================= PID ======================================
// ============================================================
//
// Un solo PID.
//
// IMPORTANTE:
// La posicion ahora esta aproximadamente entre
// -3500 y +3500.
//
// Por eso estos valores son pequeños.
//
// Primero queremos ESTABILIDAD, no velocidad.
//

//85,110,55 --- 0.52,0.0,1.9
//100, 130, 75, --- 0.72,0.0,2.5

float Kp = 0.72;//0.2
float Ki = 0.0;
float Kd = 2.5;//0.6


// ============================================================
// ================= VELOCIDADES ==============================
// ============================================================

const int VELOCIDAD_BASE = 100;//65

const int VELOCIDAD_MAX = 130;

const int VELOCIDAD_MIN = 75;


// ============================================================
// ================= BUSQUEDA DE LINEA ========================
// ============================================================

const int VELOCIDAD_BUSQUEDA = 45;


// ============================================================
// ================= SENSORES ================================
// ============================================================

int sensores[8];


// Posicion fisica de cada sensor.
//
// Izquierda  <---- 0 ---->  Derecha
//

const int posicionSensor[8] = {
  -3500,
  -2500,
  -1500,
  -500,
  500,
  1500,
  2500,
  3500
};


// ============================================================
// ================= CALIBRACION =============================
// ============================================================

int minimoSensor[8];
int maximoSensor[8];


// ============================================================
// ================= PID ======================================
// ============================================================

float error = 0;
float errorAnterior = 0;
float integral = 0;


// ============================================================
// ================= ESTADO ===================================
// ============================================================

bool calibrado = false;
bool funcionando = false;


// ============================================================
// ================= ULTIMA POSICION ==========================
// ============================================================

float ultimaPosicion = 0;


// ============================================================
// ================= MUX ======================================
// ============================================================

void setMuxChannel(byte ch) {
  digitalWrite(S0, ch & 0x01);
  digitalWrite(S1, ch & 0x02);
  digitalWrite(S2, ch & 0x04);
}


// ============================================================
// ================= LEER SENSOR ==============================
// ============================================================

int leerSensor(byte canal) {
  setMuxChannel(canal);

  // Tiempo para que el CD4051 se estabilice
  delayMicroseconds(10);

  // Lectura de descarte
  analogRead(MUX_OUT);

  // Lectura real
  return analogRead(MUX_OUT);
}


// ============================================================
// ================= CALIBRACION =============================
// ============================================================

void calibrarSensores() {
  Serial.println();
  Serial.println("================================");
  Serial.println("       CALIBRANDO SENSORES");
  Serial.println("================================");

  Serial.println();
  Serial.println("Mueve la regleta sobre BLANCO");
  Serial.println("y NEGRO durante 5 segundos.");
  Serial.println();


  // Encender LEDs
  digitalWrite(LED_IZ, HIGH);
  digitalWrite(LED_DR, HIGH);


  // Inicializar valores
  for (int i = 0; i < 8; i++) {
    minimoSensor[i] = 1023;
    maximoSensor[i] = 0;
  }


  unsigned long inicio = millis();


  // ==========================================================
  // CALIBRACION DURANTE 5 SEGUNDOS
  // ==========================================================

  while (millis() - inicio < 15000) {
    for (byte i = 0; i < 8; i++) {
      int valor = leerSensor(i);


      if (valor < minimoSensor[i]) {
        minimoSensor[i] = valor;
      }


      if (valor > maximoSensor[i]) {
        maximoSensor[i] = valor;
      }
    }
  }


  // Apagar LEDs
  digitalWrite(LED_IZ, LOW);
  digitalWrite(LED_DR, LOW);


  // ==========================================================
  // MOSTRAR RESULTADOS
  // ==========================================================

  Serial.println();
  Serial.println("CALIBRACION TERMINADA");
  Serial.println();

  Serial.println("Sensor    MIN    MAX");
  Serial.println("--------------------");


  for (int i = 0; i < 8; i++) {
    Serial.print("   ");
    Serial.print(i);
    Serial.print("      ");

    Serial.print(minimoSensor[i]);

    Serial.print("    ");

    Serial.println(maximoSensor[i]);
  }


  Serial.println();
  Serial.println("Presiona D2 nuevamente");
  Serial.println("para iniciar el robot.");
  Serial.println();


  calibrado = true;
}


// ============================================================
// ================= NORMALIZAR ===============================
// ============================================================
//
// Convierte cada sensor a:
//
// 0    = no detecta linea
// 1000 = detecta linea fuertemente
//
// Cada sensor utiliza SU PROPIA calibracion.
//

int normalizarSensor(int valor, byte sensor) {
  int minimo = minimoSensor[sensor];
  int maximo = maximoSensor[sensor];


  // Evitar division / rango incorrecto
  if (maximo <= minimo + 5) {
    return 0;
  }


  int valorNormalizado;


  if (TIPO_LINEA == 1) {
    // --------------------------------------------------------
    // LINEA BLANCA / FONDO NEGRO
    // --------------------------------------------------------
    //
    // Valor alto = blanco
    //

    valorNormalizado =
      map(
        valor,
        minimo,
        maximo,
        0,
        1000);
  } else {
    // --------------------------------------------------------
    // LINEA NEGRA / FONDO BLANCO
    // --------------------------------------------------------
    //
    // Valor bajo = negro
    //

    valorNormalizado =
      map(
        valor,
        minimo,
        maximo,
        1000,
        0);
  }


  valorNormalizado =
    constrain(
      valorNormalizado,
      0,
      1000);


  return valorNormalizado;
}


// ============================================================
// ================= LEER POSICION ============================
// ============================================================
//
// Calcula una posicion CONTINUA.
//
// Ya no tenemos:
//
// -4 -3 -2 -1 1 2 3 4
//
// Ahora podemos obtener:
//
// -1830
// -750
// -120
// +340
// +1250
// etc.
//
// Esto hace que el movimiento sea mucho mas suave.
//

float leerPosicion() {
  long suma = 0;

  long sumaPonderada = 0;


  // ==========================================================
  // LEER LOS 8 SENSORES
  // ==========================================================

  for (byte i = 0; i < 8; i++) {
    sensores[i] = leerSensor(i);


    int valor =
      normalizarSensor(
        sensores[i],
        i);


    suma += valor;


    sumaPonderada +=
      (long)valor * posicionSensor[i];
  }


  // ==========================================================
  // NO HAY LINEA
  // ==========================================================

  if (suma < 100) {
    if (ultimaPosicion > 0) {
      return 3500;
    }


    if (ultimaPosicion < 0) {
      return -3500;
    }


    return 0;
  }


  // ==========================================================
  // POSICION PONDERADA
  // ==========================================================

  float posicion =
    (float)sumaPonderada / (float)suma;


  ultimaPosicion = posicion;


  return posicion;
}


// ============================================================
// ================= MOTORES =================================
// ============================================================

void moverMotores(
  int velIzq,
  int velDer) {
  // Limitar motores
  velIzq =
    constrain(
      velIzq,
      -VELOCIDAD_MAX,
      VELOCIDAD_MAX);


  velDer =
    constrain(
      velDer,
      -VELOCIDAD_MAX,
      VELOCIDAD_MAX);


  // ==========================================================
  // MOTOR IZQUIERDO
  // ==========================================================

  if (velIzq >= 0) {
    digitalWrite(AIN1, HIGH);
    digitalWrite(AIN2, LOW);

    analogWrite(
      PWMA,
      velIzq);
  } else {
    digitalWrite(AIN1, LOW);
    digitalWrite(AIN2, HIGH);

    analogWrite(
      PWMA,
      -velIzq);
  }


  // ==========================================================
  // MOTOR DERECHO
  // ==========================================================

  if (velDer >= 0) {
    digitalWrite(BIN1, HIGH);
    digitalWrite(BIN2, LOW);

    analogWrite(
      PWMB,
      velDer);
  } else {
    digitalWrite(BIN1, LOW);
    digitalWrite(BIN2, HIGH);

    analogWrite(
      PWMB,
      -velDer);
  }
}


// ============================================================
// ================= FRENAR ==================================
// ============================================================

void frenarMotores() {
  analogWrite(PWMA, 0);
  analogWrite(PWMB, 0);

  digitalWrite(AIN1, LOW);
  digitalWrite(AIN2, LOW);

  digitalWrite(BIN1, LOW);
  digitalWrite(BIN2, LOW);
}


// ============================================================
// ================= BOTON ====================================
// ============================================================

void esperarSoltarBoton() {
  while (digitalRead(BOTON) == LOW) {
    delay(10);
  }

  delay(50);
}


// ============================================================
// ================= SETUP ====================================
// ============================================================

void setup() {
  // ==========================================================
  // MUX
  // ==========================================================

  pinMode(S0, OUTPUT);
  pinMode(S1, OUTPUT);
  pinMode(S2, OUTPUT);

  pinMode(MUX_OUT, INPUT);


  // ==========================================================
  // LED
  // ==========================================================

  pinMode(LED_IZ, OUTPUT);
  pinMode(LED_DR, OUTPUT);


  // ==========================================================
  // BOTON
  // ==========================================================

  pinMode(
    BOTON,
    INPUT_PULLUP);


  // ==========================================================
  // MOTORES
  // ==========================================================

  pinMode(PWMA, OUTPUT);
  pinMode(AIN1, OUTPUT);
  pinMode(AIN2, OUTPUT);

  pinMode(PWMB, OUTPUT);
  pinMode(BIN1, OUTPUT);
  pinMode(BIN2, OUTPUT);

  pinMode(STBY, OUTPUT);


  // Activar TB6612

  digitalWrite(
    STBY,
    HIGH);


  // Motores apagados

  frenarMotores();


  // ==========================================================
  // SERIAL
  // ==========================================================

  Serial.begin(115200);


  Serial.println();
  Serial.println("================================");
  Serial.println("    SEGUIDOR DE LINEA");
  Serial.println("    MODO ESTABLE");
  Serial.println("================================");
  Serial.println();

  Serial.println("Presiona D2 para calibrar.");
}


// ============================================================
// ================= LOOP =====================================
// ============================================================

void loop() {
  // ==========================================================
  // ESPERANDO CALIBRACION
  // ==========================================================

  if (!calibrado) {
    if (digitalRead(BOTON) == LOW) {
      delay(50);


      if (digitalRead(BOTON) == LOW) {
        esperarSoltarBoton();

        calibrarSensores();
      }
    }


    return;
  }


  // ==========================================================
  // ESPERANDO INICIO
  // ==========================================================

  if (!funcionando) {
    if (digitalRead(BOTON) == LOW) {
      delay(50);


      if (digitalRead(BOTON) == LOW) {
        esperarSoltarBoton();


        funcionando = true;


        // Reiniciar PID

        error = 0;
        errorAnterior = 0;
        integral = 0;


        Serial.println();
        Serial.println("================================");
        Serial.println("       ROBOT ACTIVADO");
        Serial.println("================================");
        Serial.println();
      }
    }


    return;
  }


  // ==========================================================
  // LEER POSICION
  // ==========================================================

  float posicion =
    leerPosicion();


  // ==========================================================
  // DETERMINAR SI HAY LINEA
  // ==========================================================

  bool hayLinea = false;


  for (byte i = 0; i < 8; i++) {
    int valor =
      normalizarSensor(
        sensores[i],
        i);


    if (valor > 100) {
      hayLinea = true;
      break;
    }
  }


  // ==========================================================
  // PERDIO LA LINEA
  // ==========================================================

  if (!hayLinea) {
    //
    // Buscar MUY lentamente.
    //
    // Si la linea estaba a la derecha:
    // gira a la derecha.
    //
    // Si estaba a la izquierda:
    // gira a la izquierda.
    //

    if (ultimaPosicion > 0) {
      moverMotores(
        VELOCIDAD_BUSQUEDA,
        -VELOCIDAD_BUSQUEDA);
    } else {
      moverMotores(
        -VELOCIDAD_BUSQUEDA,
        VELOCIDAD_BUSQUEDA);
    }


    return;
  }


  // ==========================================================
  // ERROR
  // ==========================================================

  error = posicion;


  // ==========================================================
  // INTEGRAL
  // ==========================================================

  integral += error;


  // Limitar integral para evitar wind-up

  integral =
    constrain(
      integral,
      -5000,
      5000);


  // ==========================================================
  // DERIVADA
  // ==========================================================

  float derivada =
    error - errorAnterior;


  // ==========================================================
  // PID
  // ==========================================================

  float correccion =
    Kp * error
    + Ki * integral
    + Kd * derivada;


  // ==========================================================
  // LIMITAR CORRECCION
  // ==========================================================

  correccion =
    constrain(
      correccion,
      -VELOCIDAD_MAX,
      VELOCIDAD_MAX);


  errorAnterior = error;


  // ==========================================================
  // VELOCIDAD ADAPTATIVA
  // ==========================================================
  //
  // Error pequeño:
  //
  //     velocidad ≈ 80
  //
  // Error grande:
  //
  //     velocidad disminuye
  //
  // Nunca baja de 55.
  //

  float magnitudError =
    abs(error);


  float porcentajeError =
    magnitudError / 3500.0;


  porcentajeError =
    constrain(
      porcentajeError,
      0.0,
      1.0);


  int velocidad =
    VELOCIDAD_BASE - (int)(porcentajeError * (VELOCIDAD_BASE - VELOCIDAD_MIN));


  velocidad =
    constrain(
      velocidad,
      VELOCIDAD_MIN,
      VELOCIDAD_BASE);


  // ==========================================================
  // VELOCIDADES FINALES
  // ==========================================================

  int velIzq =
    velocidad + correccion;


  int velDer =
    velocidad - correccion;


  // ==========================================================
  // LIMITAR SIN PASAR DE 110
  // ==========================================================

  velIzq =
    constrain(
      velIzq,
      -VELOCIDAD_MAX,
      VELOCIDAD_MAX);


  velDer =
    constrain(
      velDer,
      -VELOCIDAD_MAX,
      VELOCIDAD_MAX);


  // ==========================================================
  // EVITAR VELOCIDADES DEMASIADO PEQUEÑAS
  // ==========================================================
  //
  // No forzamos esto cuando la rueda necesita ir hacia atras.
  // Solo aseguramos un minimo cuando esta avanzando.
  //

  if (velIzq > 0 && velIzq < VELOCIDAD_MIN) {
    velIzq = VELOCIDAD_MIN;
  }


  if (velDer > 0 && velDer < VELOCIDAD_MIN) {
    velDer = VELOCIDAD_MIN;
  }


  // ==========================================================
  // MOVER
  // ==========================================================

  moverMotores(
    velIzq,
    velDer);


  // ==========================================================
  // DEBUG
  // ==========================================================
  //
  // Si quieres observar que esta haciendo el PID,
  // puedes quitar los // de estas lineas.
  //

  /*
  Serial.print("Pos=");
  Serial.print(posicion);

  Serial.print(" Error=");
  Serial.print(error);

  Serial.print(" Corr=");
  Serial.print(correccion);

  Serial.print(" Vel=");
  Serial.print(velocidad);

  Serial.print("  IZ=");
  Serial.print(velIzq);

  Serial.print("  DER=");
  Serial.println(velDer);
  */


  delayMicroseconds(500);
}
