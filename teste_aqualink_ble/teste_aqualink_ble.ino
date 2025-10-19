#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <driver/adc.h>
#include "esp_task_wdt.h"

// ---------- UUIDs BLE ----------
#define SERVICE_UUID        "34303c72-4cb1-4d48-98cb-781afece9cd7"
#define CHARACTERISTIC_UUID "5b4ff54f-8297-45b4-9949-7ff95e672aae"

Adafruit_VL53L0X lox;
BLEServer *pServer;
BLECharacteristic *pCharacteristic;

// ---------- Sensor garrafa ----------
float alturaGarrafa   = 24.0;
float diametroInterno = 6.7;
int   pinoLDR         = 4;
float raioInterno     = diametroInterno / 2.0;

float aguaInicial      = -1;
float aguaUltimaMedida = -1;

// ---------- Bateria ----------
const int PINO_BAT = 0;  
const float R1 = 10000.0;
const float R2 = 10000.0;

// ---------- Limiar LDR ----------
const int limiarEscuro = 10;

// ---------- Funções de bateria e LDR ----------
float lerBateriaVolts() {
  uint32_t mv = analogReadMilliVolts(PINO_BAT);
  float adcVolts = mv / 1000.0;
  return adcVolts * ((R1 + R2) / R2);
}

struct BatPoint { float v; int pct; };
const BatPoint batTable[] = {
  {4.10, 100},
  {4.05, 95},
  {4.00, 90},
  {3.95, 82},
  {3.90, 74},
  {3.85, 64},
  {3.80, 55},
  {3.75, 45},
  {3.70, 35},
  {3.60, 18},
  {3.50, 8},
  {3.40, 0}
};
const int BAT_TABLE_N = sizeof(batTable)/sizeof(batTable[0]);

float bateriaPercent(float v) {
  if (v <= batTable[BAT_TABLE_N-1].v) return 0.0f;
  if (v >= batTable[0].v) return 100.0f;

  for (int i = 0; i < BAT_TABLE_N - 1; ++i) {
    float v_hi = batTable[i].v;
    float v_lo = batTable[i+1].v;
    int p_hi = batTable[i].pct;
    int p_lo = batTable[i+1].pct;
    if (v <= v_hi && v >= v_lo) {
      float t = (v - v_lo) / (v_hi - v_lo);
      float pct = p_lo + t * (p_hi - p_lo);
      return pct;
    }
  }
  return 0.0f;
}

float lerLDRPercent() {
  int valor = analogRead(pinoLDR);
  return valor / 4095.0 * 100.0;
}

// ---------- Callbacks BLE ----------
class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pChar) {
    String comando = pChar->getValue();
    comando.trim();
    const char *resposta;

    if (comando == "1") {
      iniciarLeituras();
      resposta = "Iniciando leituras...";
    } else {
      resposta = "Comando invalido.";
    }

    pChar->setValue(resposta);
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

// ---------- Variáveis do loop ----------
unsigned long lastCheck = 0;
unsigned long lastBLEReset = 0;
const unsigned long BLE_RESET_INTERVAL = 60000; 

// ---------- Máquina de estados LDR ----------
enum EstadoCiclo { AGUARDANDO_LUZ, AGUARDANDO_ESCURO, LEITURA_FEITA };
EstadoCiclo estadoCiclo = AGUARDANDO_LUZ;

unsigned long tempoEscuro = 0;
const unsigned long tempoEscuroNecessario = 10000;

// ---------- Leituras ----------
const int totalLeituras = 50;
int leituraAtual = 0;
float somaDist = 0;
unsigned long ultimaLeitura = 0;
bool lendo = false;

// ---------- Funções ----------
void iniciarLeituras() {
  leituraAtual = 0;
  somaDist = 0;
  ultimaLeitura = millis();
  lendo = true;
  Serial.println("Iniciando ciclo de leituras...");
}

void processarLeituras() {
  if (!lendo) return;

  if (millis() - ultimaLeitura >= 50) {
    ultimaLeitura = millis();

    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false);

    if (measure.RangeStatus != 4) {
      float d = measure.RangeMilliMeter / 10.0;

      // ---- FILTRO: ignora leituras acima de 30 cm ----
      if (d <= 30.0) {
        somaDist += d;
        leituraAtual++;
      } else {
        Serial.printf("Leitura acima de 30 cm (%.1f cm), ignorada.\n", d);
        return;
      }
    }

    if (leituraAtual >= totalLeituras) {
      float mediaDistancia = somaDist / totalLeituras - 3.8;

      // Ajustes empíricos
      if(mediaDistancia <= 5) mediaDistancia -= 0.5;
      if(mediaDistancia >= 11) mediaDistancia += 1;
      if (mediaDistancia >= 13.5) mediaDistancia += 1.2;
      if(mediaDistancia >= 16) mediaDistancia += 0.5;
      if (mediaDistancia >= 17.2) mediaDistancia += 1;
      if(mediaDistancia >= 19.7) mediaDistancia += 0.6;

      float alturaAgua = alturaGarrafa - mediaDistancia;
      float aguaNaGarrafa = 3.14159 * raioInterno * raioInterno * alturaAgua;

      if (aguaInicial < 0) aguaInicial = aguaNaGarrafa;
      aguaUltimaMedida = aguaNaGarrafa;

      float vbat   = lerBateriaVolts();
      float pbat   = bateriaPercent(vbat);
      float ldrPct = lerLDRPercent();

      char buffer[200];
      snprintf(buffer, sizeof(buffer),
               "{\"distancia\":%.1f,\"volume\":%.1f,\"bateria_v\":%.2f,\"bateria_pct\":%.0f,\"ldr_pct\":%.1f}",
               mediaDistancia, aguaNaGarrafa, vbat, pbat, ldrPct);

      Serial.println("Leitura automática realizada:");
      Serial.println(buffer);

      if (pCharacteristic && pServer && pServer->getConnectedCount() > 0) {
        pCharacteristic->setValue(buffer);
        pCharacteristic->notify();
      }

      lendo = false;
    }
  }
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  Wire.begin(5, 6);

  if (!lox.begin()) {
    Serial.println("Falha ao iniciar VL53L0X");
    while (1);
  }

  analogReadResolution(12);
  analogSetPinAttenuation(PINO_BAT, ADC_11db);
  analogSetPinAttenuation(pinoLDR, ADC_11db);

  esp_task_wdt_config_t wdt_config = {
      .timeout_ms = 10000,
      .idle_core_mask = 0,
      .trigger_panic = true
  };
  esp_task_wdt_init(&wdt_config);
  esp_task_wdt_add(NULL);

  BLEDevice::init("ESP32C3_AquaLink");
  BLEDevice::setMTU(517);
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_N9);
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_N9);

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
void loop() {
  esp_task_wdt_reset();

  if (millis() - lastCheck > 1000) {
    lastCheck = millis();

    if (pServer && pServer->getConnectedCount() == 0) {
      BLEAdvertising *adv = pServer->getAdvertising();

      if (!adv->isAdvertising()) {
        static int tentativasFalha = 0;
        adv->start();
        delay(200);

        if (!adv->isAdvertising()) {
          tentativasFalha++;
          Serial.printf("Falha ao iniciar advertising (%d)\n", tentativasFalha);

          if (tentativasFalha >= 3) {
            Serial.println("BLE travado, reiniciando ESP...");
            esp_restart();
          }
        } else {
          tentativasFalha = 0;
        }
      }
    }
  }

  if (millis() - lastBLEReset > BLE_RESET_INTERVAL) {
    lastBLEReset = millis();

    if (pServer && pServer->getConnectedCount() == 0) {
      Serial.println("Reiniciando stack BLE preventivamente...");
      BLEDevice::deinit(true);
      delay(2000);
      BLEDevice::init("ESP32C3_AquaLink");

      pServer = BLEDevice::createServer();
      pServer->setCallbacks(new MyServerCallbacks());

      BLEService *pService = pServer->createService(SERVICE_UUID);
      pCharacteristic = pService->createCharacteristic(
        CHARACTERISTIC_UUID,
        BLECharacteristic::PROPERTY_READ |
        BLECharacteristic::PROPERTY_WRITE |
        BLECharacteristic::PROPERTY_NOTIFY
      );

      pCharacteristic->setValue("Reiniciado.");
      pCharacteristic->setCallbacks(new MyCallbacks());
      pService->start();

      BLEAdvertising *pAdvertising = pServer->getAdvertising();
      pAdvertising->setScanResponse(true);
      pAdvertising->addServiceUUID(SERVICE_UUID);
      pAdvertising->setName("ESP32C3_AquaLink");
      pAdvertising->start();

      Serial.println("Stack BLE reiniciada com sucesso.");
    }
  }

  int ldrValorBruto = analogRead(pinoLDR);
  static float ldrValor = 0;
  ldrValor = (ldrValor*3 + ldrValorBruto)/4;

  switch (estadoCiclo) {
    case AGUARDANDO_LUZ:
      if (ldrValor > limiarEscuro) estadoCiclo = AGUARDANDO_ESCURO;
      break;

    case AGUARDANDO_ESCURO:
      if (ldrValor <= limiarEscuro) {
        if (tempoEscuro == 0) tempoEscuro = millis();
        else if (millis() - tempoEscuro >= tempoEscuroNecessario) {
          iniciarLeituras();
          estadoCiclo = LEITURA_FEITA;
        }
      } else tempoEscuro = 0;
      break;

    case LEITURA_FEITA:
      if (!lendo) {
        estadoCiclo = AGUARDANDO_LUZ;
        tempoEscuro = 0;
      }
      break;
  }

  processarLeituras();
}
