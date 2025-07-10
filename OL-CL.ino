/*    developed by Edwin Ruano G.
  Control dinámico OL/LC con entrada de RPM y salida para Serial Plotter
  • OL: Open Loop — PWM = map(RPM,0–1500 → 0–255), solo mide vs consigna.
  • CL: Closed Loop — Control PID para mantener la RPM deseada.
  • Comandos por Serial:
      OL or CL → cambia modo
      <0–1500> → actualiza RPM deseada (0 detiene en CL)
  • Salida (cada 50 ms):
      <RPM_deseada> <RPM_medida>

*/

const int encoderPin     = 2;      // Encoder (600 ppr)
const int pwmPin         = 13;     // PWM al motor
const float pulsesPerRev = 600.0;
const unsigned long Ts   = 50;     // Período muestreo [ms]

// Ganancias PI (ajustar según respuesta)
float Kc = 0.2, Ki = 0.1;

// Variables globales
volatile unsigned int pulseCount = 0;
float measuredRPM = 0, desiredRPM = 0;
float integral    = 0;
int   pwmOut      = 0;

// Modos
enum Mode {UNDEF, OL, CL};
Mode mode = UNDEF;
bool plotEnabled = false;

// Tiempos
unsigned long tMeasure = 0, tControl = 0;

// ISR encoder
void encoderISR() {
  pulseCount++;

}

void setup() {
  Serial.begin(115200);
  pinMode(encoderPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPin), encoderISR, RISING);
  pinMode(pwmPin, OUTPUT);
  analogWrite(pwmPin, 0);
  Serial.println(">> Escribe OL o CL, luego RPM (0–1500):");
}

void loop() {
  unsigned long now = millis();

  // 1) Leer Serial
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();
    cmd.toUpperCase();

    // Cambio de modo
    if (!plotEnabled && (cmd == "OL" || cmd == "CL")) {
      mode = (cmd == "OL") ? OL : CL;
      plotEnabled = true;
      integral = 0;
      if (mode == OL) {
        // PWM inicial en OL
        pwmOut = map((int)desiredRPM, 0, 1500, 0, 255);
        analogWrite(pwmPin, pwmOut);
      } else {
        pwmOut = 0;
        analogWrite(pwmPin, 0);
      }
    }
    // Si ya en modo OL/CL, tratar cmd como RPM
    else if (plotEnabled) {
      int v = cmd.toInt();
      if (v >= 0 && v <= 1500) {
        desiredRPM = v;
        // En OL, actualizar PWM inmediato
        if (mode == OL) {
          pwmOut = map(v, 0, 1500, 0, 255);
          analogWrite(pwmPin, pwmOut);
        }
        // En CL, si v==0 detenemos
        if (mode == CL && v == 0) {
          integral = 0;
          pwmOut = 0;
          analogWrite(pwmPin, 0);
        }
      }
    }
  }

  // 2) Medir RPM cada Ts
  if (now - tMeasure >= Ts) {
    noInterrupts();
    unsigned int p = pulseCount;
    pulseCount = 0;
    interrupts();
    measuredRPM = (p / pulsesPerRev) * (60000.0 / Ts);
    tMeasure = now;
  }

  // 3) Control & Plot cada Ts (solo tras seleccionar modo)
  if (plotEnabled && now - tControl >= Ts) {
    if (mode == CL) {
      float error = desiredRPM - measuredRPM;
      integral += error * (Ts / 1000.0);
      float u = Kc * error + Ki * integral;
      pwmOut = constrain((int)u, 0, 255);
      analogWrite(pwmPin, pwmOut);
    }
    // En OL el pwmOut ya está fijo

    // Salida para Serial Plotter
    Serial.print(desiredRPM, 1);
    Serial.print(' ');
    Serial.println(measuredRPM, 1);

    tControl = now;
  }
}