/*
 * ESP32 - PUENTE DE COMUNICACIÓN MINIMALISTA
 * Bluetooth (Celular) ↔ ESP32 ↔ UART (Arduino Nano)
 * 
 * Conexiones:
 * - GPIO17 (TX2) → Arduino Nano D0 (RX)
 * - GPIO16 (RX2) → Arduino Nano D1 (TX)
 * - GND → GND
 * 
 * Funcionalidad:
 * - Lo que llega por Bluetooth se reenvía al Arduino
 * - Lo que llega del Arduino se reenvía a Bluetooth
 * - Todo se muestra en Serial Monitor para debug
 */

#include "BluetoothSerial.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth no está habilitado
#endif

// Pines UART2
#define RXD2 16
#define TXD2 17

BluetoothSerial SerialBT;

void setup() {
  // Serial para debug
  Serial.begin(9600);
  Serial.println("ESP32 iniciado");
  
  // Bluetooth
  SerialBT.begin("ESP32_luca ");
  Serial.println("Bluetooth: ESP32_Bridge");
  
  // UART al Arduino
  Serial2.begin(9600, SERIAL_8N1, RXD2, TXD2);
  Serial.println("UART2 iniciado");
}

void loop() {
  // Bluetooth → Arduino
  if (SerialBT.available()) {
    String msg = SerialBT.readStringUntil('\n');
    Serial.print("BT→ARD: ");
    Serial.println(msg);
    Serial2.println(msg);
  }
  
  // Arduino → Bluetooth
  if (Serial2.available()) {
    String msg = Serial2.readStringUntil('\n');
    Serial.print("ARD→BT: ");
    Serial.println(msg);
    SerialBT.println(msg);
  }
}
