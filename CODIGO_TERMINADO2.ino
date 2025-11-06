#include <AccelStepper.h>
#include <Servo.h>
#include <Wire.h>
#include <LCD.h>
#include <LiquidCrystal_I2C.h>

// ====== LCD I2C (New-LiquidCrystal) ======
LiquidCrystal_I2C lcd(0x27, 2, 1, 0, 4, 5, 6, 7);

// ====== Servo vertical ======
Servo servoverti;
int servov = 90;
int servovLimitHigh = 180;
int servovLimitLow = 0;
int servoObjetivo = 90;

// ====== Límites del stepper horizontal ======
const float stepperLimitMin = 0.0;
const float stepperLimitMax = 360.0;
const long PASOS_POR_VUELTA = 2048; // Motor 28BYJ-48

// ====== Pines del ULN2003 ======
#define IN1 2
#define IN2 3
#define IN3 4
#define IN4 5

AccelStepper stepper(AccelStepper::FULL4WIRE, IN1, IN3, IN2, IN4);

// ====== Sensibilidad LDR ======
const int ldrThreshold = 30;

// ====== Pines LDR ======
const int LDR_TOP_R = A0;
const int LDR_TOP_L = A1;
const int LDR_BOT_R = A2;
const int LDR_BOT_L = A3;

// ====== Sensor ACS712 ======
const int ACS_PIN = A6;
const float VREF = 5.0;
const int ADC_RES = 1023;
const float SENSITIVITY = 0.185;
float ACS_offset = 2.5;

// ====== Variables LCD ======
unsigned long lastUpdate = 0;
const unsigned long interval = 7000;
bool mostrarLecturaNueva = true;
float ultimaMedicion = 0.0;

// ====== MODO AUTOMÁTICO/MANUAL ======
bool modoAutomatico = false;
bool volviendoACero = false;
String mensaje = "";

// ====== Funciones de conversión ======
// Convierte pasos a grados
float pasosAGrados(long pasos) {
  return (pasos * 360.0) / PASOS_POR_VUELTA;
}

// Convierte grados a pasos
long gradosAPasos(float grados) {
  return (long)((grados * PASOS_POR_VUELTA) / 360.0);
}

// Obtiene el ángulo actual del stepper
float getAnguloActual() {
  return pasosAGrados(stepper.currentPosition());
}

void setup() {
  Serial.begin(9600);
  
  // ====== LCD ======
  lcd.setBacklightPin(3, POSITIVE);
  lcd.setBacklight(HIGH);
  lcd.begin(16, 2);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Panel Solar ON");
  lcd.setCursor(0, 1);
  lcd.print("Modo: MANUAL");
  delay(2000);
  
  // ====== Servo vertical ======
  servoverti.attach(9);
  servoverti.write(servov);
  
  // ====== Stepper ======
  stepper.setMaxSpeed(1200.0);
  stepper.setAcceleration(500.0);
  stepper.setCurrentPosition(0); // Posición inicial = 0 pasos = 0 grados
  
  Serial.println("Arduino listo. Esperando comandos...");
  Serial.println("IMPORTANTE: El stepper está en posición 0° (origen)");
  Serial.print("Estado inicial: Modo ");
  Serial.println(modoAutomatico ? "AUTOMATICO" : "MANUAL");
}

void loop() {
  // ====== Verificar comandos del ESP32 ======
  if (Serial.available()) {
    mensaje = Serial.readStringUntil('\n');
    mensaje.trim();
    
    Serial.print("Recibido: ");
    Serial.println(mensaje);
    
    if (mensaje == "MODOAUTOMATICOON") {
      modoAutomatico = true;
      volviendoACero = false;
      
      float anguloActual = getAnguloActual();
      Serial.println("=== MODO AUTOMATICO ACTIVADO ===");
      Serial.print("Posición actual preservada: ");
      Serial.print(anguloActual, 1);
      Serial.print("° (");
      Serial.print(stepper.currentPosition());
      Serial.println(" pasos)");
      
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Modo: AUTOMATICO");
      lcd.setCursor(0, 1);
      lcd.print("H:");
      lcd.print((int)anguloActual);
      lcd.print("° V:");
      lcd.print(servov);
      lcd.print("°");
      delay(1500);
    }
    else if (mensaje.indexOf("MODOAUTOMATICOFF") != -1 || mensaje.indexOf("MODOAUTOMATICOOFF") != -1) {
      modoAutomatico = false;
      
      float anguloActual = getAnguloActual();
      long pasosActuales = stepper.currentPosition();
      
      Serial.println("=== MODO MANUAL ACTIVADO ===");
      Serial.print("Posición ANTES del cambio: ");
      Serial.print(anguloActual, 1);
      Serial.print(" grados (");
      Serial.print(pasosActuales);
      Serial.println(" pasos)");
      
      // Calcular movimiento necesario para llegar a 180°
      long pasosObjetivo180 = gradosAPasos(180.0);
      long pasosAMover = pasosObjetivo180 - pasosActuales;
      float gradosAMover = 180.0 - anguloActual;
      
      Serial.print("Necesito mover: ");
      Serial.print(gradosAMover, 1);
      Serial.print(" grados (");
      Serial.print(pasosAMover);
      Serial.println(" pasos) para llegar a 180°");
      
      // Comandar el movimiento a posición 180°
      stepper.moveTo(pasosObjetivo180); // moveTo usa posición ABSOLUTA
      volviendoACero = true;
      
      // Servo a posición neutral
      servov = 90;
      servoObjetivo = 90;
      servoverti.write(servov);
      
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("Modo: MANUAL");
      lcd.setCursor(0, 1);
      lcd.print("Volviendo a 0...");
      
      Serial.println("Comando enviado: moveTo(0)");
    }
    
    // Procesar comandos de movimiento
    if (mensaje.indexOf("VERTICAL") != -1 || mensaje.indexOf("HORIZONTAL") != -1) {
      procesarComandosManuales(mensaje);
    }
  }
  
  // Verificar si terminó de volver a cero
  if (volviendoACero && stepper.distanceToGo() == 0) {
    volviendoACero = false;
    Serial.println("✓✓✓ POSICION 0° ALCANZADA ✓✓✓");
    Serial.print("Verificación - Posición actual: ");
    Serial.print(stepper.currentPosition());
    Serial.print(" pasos = ");
    Serial.print(getAnguloActual(), 1);
    Serial.println("°");
    
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Modo: MANUAL");
    lcd.setCursor(0, 1);
    lcd.print("V:90° H:0°");
  }
  
  // ====== CONTROL DE SEGUIMIENTO SOLAR ======
  if (modoAutomatico) {
    trackerAutomatico();
  }
  
  // ====== Movimiento suave del servo ======
  if (!modoAutomatico && servov != servoObjetivo) {
    if (servov < servoObjetivo) {
      servov++;
    } else if (servov > servoObjetivo) {
      servov--;
    }
    servoverti.write(servov);
    delay(15);
  }
  
  // El stepper siempre ejecuta sus pasos
  stepper.run();
  
  // ====== Lectura ACS712 ======
  int raw = analogRead(ACS_PIN);
  float voltage = (raw * VREF) / ADC_RES;
  float current = (voltage + ACS_offset) / SENSITIVITY;
  ultimaMedicion = current;
  
  // ====== Mostrar en LCD ======
  unsigned long ahora = millis();
  if (ahora - lastUpdate >= interval && !volviendoACero) {
    lastUpdate = ahora;
    mostrarLecturaNueva = !mostrarLecturaNueva;
    
    lcd.clear();
    lcd.setCursor(0, 0);
    
    if (modoAutomatico) {
      lcd.print("MODO AUTO");
    } else {
      lcd.print("MODO MANUAL");
    }
    lcd.print(":");
    
    lcd.setCursor(0, 1);
    if (mostrarLecturaNueva) {
      lcd.print(current, 3);
      lcd.print(" Ampers");
    } else {
      lcd.print(ultimaMedicion, 3);
      lcd.print(" Ampers");
    }
  }
  
  delay(10);
}

// ====== Función de seguimiento automático ======
void trackerAutomatico() {
  int topl = analogRead(LDR_TOP_L);
  int topr = analogRead(LDR_TOP_R);
  int botl = analogRead(LDR_BOT_L);
  int botr = analogRead(LDR_BOT_R);
  
  int avgtop = (topl + topr) / 2;
  int avgbot = (botl + botr) / 2;
  int avgleft = (topl + botl) / 2;
  int avgright = (topr + botr) / 2;
  
  // Control vertical
  if (abs(avgtop - avgbot) > ldrThreshold) {
    if (avgtop < avgbot && servov > servovLimitLow) {
      servov--;
      servoObjetivo = servov;
    } else if (avgbot < avgtop && servov < servovLimitHigh) {
      servov++;
      servoObjetivo = servov;
    }
    servoverti.write(servov);
  }
  
  // Control horizontal
  int diff = avgright - avgleft;
  bool modoBottom = (avgbot > avgtop + ldrThreshold * 1.5);
  int diffHorizontal;
  
  if (modoBottom) {
    diffHorizontal = botr - botl;
  } else {
    diffHorizontal = diff;
  }
  
  if (abs(diffHorizontal) > ldrThreshold) {
    int steps = map(abs(diffHorizontal), ldrThreshold, 1023, 10, 150);
    
    // Verificar límites ANTES de mover
    float anguloActual = getAnguloActual();
    float anguloProyectado;
    
    if (diffHorizontal > 0) {
      anguloProyectado = anguloActual - pasosAGrados(steps);
    } else {
      anguloProyectado = anguloActual + pasosAGrados(steps);
    }
    
    // Solo mover si está dentro de límites
    if (anguloProyectado >= stepperLimitMin && anguloProyectado <= stepperLimitMax) {
      if (diffHorizontal > 0) {
        stepper.move(-steps);
      } else {
        stepper.move(steps);
      }
    }
  }
}

// ====== Función para procesar comandos manuales ======
void procesarComandosManuales(String comandos) {
  int comandosProcesados = 0;
  float ultimoVertical = -1;
  float ultimoHorizontal = -1;
  
  Serial.print("Procesando comandos: ");
  Serial.println(comandos);
  
  // Buscar comandos VERTICAL
  int posInicio = 0;
  while (true) {
    int pos = comandos.indexOf("VERTICAL", posInicio);
    if (pos == -1) break;
    
    int inicioValor = pos + 8;
    String valorStr = "";
    
    for (int i = inicioValor; i < comandos.length(); i++) {
      char c = comandos.charAt(i);
      if (isDigit(c) || c == '.' || c == '-') {
        valorStr += c;
      } else {
        break;
      }
    }
    
    if (valorStr.length() > 0) {
      float valor = valorStr.toFloat();
      ultimoVertical = valor;
      comandosProcesados++;
    }
    
    posInicio = pos + 8 + valorStr.length();
  }
  
  // Aplicar comando VERTICAL
  if (ultimoVertical >= 0) {
    servoObjetivo = constrain((int)ultimoVertical, servovLimitLow, servovLimitHigh);
    
    Serial.print("✓ Servo vertical -> ");
    Serial.print(servoObjetivo);
    Serial.println("°");
    
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(" Vertical: ");
    lcd.print(servoObjetivo);
    lcd.print(" grados");
  }
  
  // Buscar comandos HORIZONTAL
  posInicio = 0;
  while (true) {
    int pos = comandos.indexOf("HORIZONTAL", posInicio);
    if (pos == -1) break;
    
    int inicioValor = pos + 10;
    String valorStr = "";
    
    for (int i = inicioValor; i < comandos.length(); i++) {
      char c = comandos.charAt(i);
      if (isDigit(c) || c == '.' || c == '-') {
        valorStr += c;
      } else {
        break;
      }
    }
    
    if (valorStr.length() > 0) {
      float valor = valorStr.toFloat();
      ultimoHorizontal = valor;
      comandosProcesados++;
    }
    
    posInicio = pos + 10 + valorStr.length();
  }
  
  // Aplicar comando HORIZONTAL
  if (ultimoHorizontal >= 0) {
    float anguloLimitado = constrain(ultimoHorizontal, stepperLimitMin, stepperLimitMax);
    
    if (ultimoHorizontal != anguloLimitado) {
      Serial.print("⚠ Límite! Comando: ");
      Serial.print(ultimoHorizontal, 1);
      Serial.print("° → Limitado: ");
      Serial.print(anguloLimitado, 1);
      Serial.println("°");
      
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("LIMITE!");
      lcd.setCursor(0, 1);
      lcd.print("Max: ");
      lcd.print((int)stepperLimitMax);
      lcd.print("°");
      delay(1500);
    }
    
    // Obtener posición actual
    float anguloActual = getAnguloActual();
    long pasosActuales = stepper.currentPosition();
    
    // Calcular posición objetivo en pasos
    long pasosObjetivo = gradosAPasos(anguloLimitado);
    long pasosDiferencia = pasosObjetivo - pasosActuales;
    
    Serial.println("--- Movimiento Horizontal ---");
    Serial.print("Posición actual: ");
    Serial.print(anguloActual, 1);
    Serial.print("° (");
    Serial.print(pasosActuales);
    Serial.println(" pasos)");
    
    Serial.print("Posición objetivo: ");
    Serial.print(anguloLimitado, 1);
    Serial.print("° (");
    Serial.print(pasosObjetivo);
    Serial.println(" pasos)");
    
    Serial.print("Diferencia: ");
    Serial.print(pasosDiferencia);
    Serial.print(" pasos = ");
    Serial.print(anguloLimitado - anguloActual, 1);
    Serial.println("°");
    
    if (abs(pasosDiferencia) > 1) {
      stepper.moveTo(pasosObjetivo); // Usar moveTo (absoluto)
      
      Serial.print("✓ Comando enviado: moveTo(");
      Serial.print(pasosObjetivo);
      Serial.println(")");
    } else {
      Serial.println("→ Ya está en posición");
    }
    
    lcd.setCursor(0, 1);
    lcd.print("Horizontal:");
    lcd.print((int)anguloLimitado);
    lcd.print(" grados   ");
  }
  
  if (comandosProcesados > 0) {
    Serial.print("→ ");
    Serial.print(comandosProcesados);
    Serial.println(" comando(s) procesados");
    Serial.println("---");
  }
}
