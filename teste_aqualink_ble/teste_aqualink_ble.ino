#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <driver/adc.h>

// ---------- UUIDs BLE ----------
#define SERVICE_UUID        "12345678-1234-1234-1234-1234567890ab"
#define CHARACTERISTIC_UUID "abcd1234-ab12-cd34-ef56-1234567890ab"

Adafruit_VL53L0X lox;
BLEServer *pServer;
BLECharacteristic *pCharacteristic;

// ---------- Sensor garrafa ----------
float alturaGarrafa   = 29.4;
float diametroInterno = 6.7;
int   pinoLDR         = 1;  // LDR no GPIO1
float raioInterno     = 6.7 / 2.0;

float aguaInicial      = -1;
float aguaUltimaMedida = -1;

// ---------- Bateria ----------
const int PINO_BAT = 0;  // GPIO0
const float R1 = 10000.0;
const float R2 = 10000.0;

// ---------- Funções ----------
float lerBateriaVolts() {
  uint32_t mv = analogReadMilliVolts(PINO_BAT);
  return (mv / 1000.0) * (R1 + R2) / R2;
}

float bateriaPercent(float v) {
  if (v >= 4.0) return 100.0;
  if (v < 3.0)  return 0.0;
  int step = (int)((v - 3.0) / 0.1 + 0.5);
  if (step > 9) step = 9;
  if (step < 0) step = 0;
  return step * 10.0;
}

float lerLDRPercent() {
  int valor = analogRead(pinoLDR);
  return valor / 4095.0 * 100.0;
}

// Leitura do VL53L0X
String realizarLeituras() {
  float somaDist = 0;
  int leiturasValidas = 0;

  for (int i = 0; i < 50; i++) {
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false);

    if (measure.RangeStatus != 4) {
      float d = measure.RangeMilliMeter / 10.0;
      if (d < 0) d = 0;
      somaDist += d;
      leiturasValidas++;
    }
    delay(50);
  }

  if (leiturasValidas == 0) return "Nenhuma leitura válida obtida.";

  float mediaDistancia = somaDist / leiturasValidas - 3.5;
  if (mediaDistancia >= 26.5) mediaDistancia += 3.5;

  float alturaAgua = alturaGarrafa - mediaDistancia;
  if (alturaAgua < 0) alturaAgua = 0;

  float aguaNaGarrafa = 3.14159 * raioInterno * raioInterno * alturaAgua;

  if (aguaInicial < 0)      aguaInicial      = aguaNaGarrafa;
  if (aguaUltimaMedida < 0) aguaUltimaMedida = aguaNaGarrafa;

  aguaUltimaMedida = aguaNaGarrafa;

  float vbat   = lerBateriaVolts();
  float pbat   = bateriaPercent(vbat);
  float ldrPct = lerLDRPercent();

  char buffer[200];
  snprintf(buffer, sizeof(buffer),
           "{\"distancia\":%.1f,\"volume\":%.1f,\"bateria_v\":%.2f,\"bateria_pct\":%.0f,\"ldr_pct\":%.1f}",
           mediaDistancia, aguaNaGarrafa, vbat, pbat, ldrPct);

  return String(buffer);
}

// ---------- Callbacks ----------
class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pChar) {
    String comando = pChar->getValue();
    comando.trim();
    String resposta;

    if (comando == "1") {
      resposta = realizarLeituras();
    } else {
      resposta = "Comando inválido. Envie '1' para leituras.";
    }

    pChar->setValue(resposta.c_str());
    pChar->notify();

    Serial.println("Resposta enviada via BLE:");
    Serial.println(resposta);
    Serial.println("-------------------------");
  }
};

class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *pServer) {
    Serial.println("Dispositivo conectado!");
  }
  void onDisconnect(BLEServer *pServer) {
    Serial.println("Dispositivo desconectado! Reiniciando advertising...");
    float vbat  = lerBateriaVolts();
    float pbat  = bateriaPercent(vbat);
    Serial.printf("Status bateria: %.2f V  (%.0f%%)\n", vbat, pbat);
    pServer->getAdvertising()->start();
  }
};

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  Wire.begin(4, 3);

  if (!lox.begin()) {
    Serial.println("Falha ao iniciar VL53L0X");
    while (1);
  }

  analogReadResolution(12);
  analogSetPinAttenuation(PINO_BAT, ADC_11db);
  analogSetPinAttenuation(pinoLDR, ADC_11db);

  BLEDevice::init("ESP32C3_AquaLink");
  BLEDevice::setMTU(517);

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_NOTIFY
  );

  pCharacteristic->setValue("Pronto para comandos: '1' = leituras");
  pCharacteristic->setCallbacks(new MyCallbacks());

  pService->start();

  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->setScanResponse(true);
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setName("ESP32C3_AquaLink");
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMaxPreferred(0x12);
  pAdvertising->start();

  delay(500);
  Serial.println("BLE pronto. Conecte pelo celular.");

  float vbat  = lerBateriaVolts();
  float pbat  = bateriaPercent(vbat);
  float ldrPct = lerLDRPercent();
  Serial.printf("Status bateria: %.2f V  (%.0f%%), LDR: %.1f%%\n", vbat, pbat, ldrPct);
}

// ---------- Loop ----------
unsigned long lastCheck = 0;
unsigned long lastLDR   = 0;

void loop() {
  // --- Verificação do server BLE ---
  if (millis() - lastCheck > 1000) {
    lastCheck = millis();
    if (pServer && pServer->getConnectedCount() == 0) {
      pServer->getAdvertising()->start();
    }
  }


  if (millis() - lastLDR > 10000) {
    lastLDR = millis();
    float ldrPct = lerLDRPercent();
    int valorLDR = analogRead(pinoLDR);
    Serial.printf("Valor LDR: %d  (%.2f%%)\n", valorLDR, ldrPct);
  }
}
