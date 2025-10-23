#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include <driver/adc.h>
#include "esp_task_wdt.h"
#include "WiFi.h"

// ---------- UUIDs BLE ----------
#define SERVICE_UUID        "34303c72-4cb1-4d48-98cb-781afece9cd7"
#define CHARACTERISTIC_UUID "5b4ff54f-8297-45b4-9949-7ff95e672aae"

Adafruit_VL53L0X lox;
BLEServer *pServer;
BLECharacteristic *pCharacteristic;
BLEAdvertising *pAdvertising;
const char* BLE_DEVICE_NAME = "ESP32C3_AquaLink";

static bool advertisingAtivo = false;

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

// ---------- Parâmetros BLE ----------
const uint16_t SAFE_MTU = 256;
const uint16_t CONN_INTERVAL = 160;
const uint16_t CONN_TIMEOUT  = 400;

// ---------- Temporizadores ----------
unsigned long lastCheck = 0;
const unsigned long BLE_MONITOR_INTERVAL = 2000; // 2s

hw_timer_t *timerReboot = NULL;
const unsigned long FAILSAFE_INTERVAL_MS = 20 * 60 * 1000UL; // 20 min

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

// ---------- Funções de bateria ----------
float lerBateriaVolts() {
  uint32_t mv = analogReadMilliVolts(PINO_BAT);
  float adcVolts = mv / 1000.0;
  return adcVolts * ((R1 + R2) / R2);
}

struct BatPoint { float v; int pct; };
const BatPoint batTable[] = {
  {4.10, 100}, {4.05, 95}, {4.00, 90}, {3.95, 82}, {3.90, 74},
  {3.85, 64}, {3.80, 55}, {3.75, 45}, {3.70, 35}, {3.60, 18},
  {3.50, 8}, {3.40, 0}
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

// ---------- Leituras ----------
void iniciarLeituras() {
  leituraAtual = 0;
  somaDist = 0;
  ultimaLeitura = millis();
  lendo = true;
  Serial.println("Iniciando ciclo de leituras...");
}

// ---------- Função processarLeituras (modificada) ----------
void processarLeituras() {
  static int leiturasInvalidas = 0; 

  if (!lendo) return;

  if (millis() - ultimaLeitura >= 50) {
    ultimaLeitura = millis();
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false);

    float d = measure.RangeMilliMeter / 10.0; 

 
    if (d > 29.0 || d > 500.0 || d <= 0) {
      leiturasInvalidas++;
      Serial.printf(" Leitura inválida (%.1f cm). Contagem: %d/5\n", d, leiturasInvalidas);

    
      if (leiturasInvalidas >= 5) {
        Serial.println(" Leituras inválidas consecutivas. Ciclo cancelado.");
        if (pCharacteristic && pServer && pServer->getConnectedCount() > 0) {
          pCharacteristic->setValue("{\"aviso\":\"leituras inválidas consecutivas\"}");
          pCharacteristic->notify();
        }
        lendo = false;
        leiturasInvalidas = 0; 
      }
      return; 
    }

 
    leiturasInvalidas = 0;

    // acumula leitura válida
    somaDist += d;
    leituraAtual++;

    // finaliza ciclo após totalLeituras válidas
    if (leituraAtual >= totalLeituras) {
      float mediaDistancia = somaDist / leituraAtual - 3.6;

      // ajustes de calibração
      if(mediaDistancia <= 5.2) mediaDistancia -= 1;
      if(mediaDistancia <= 8.4) mediaDistancia -= 1;
      if(mediaDistancia <= 10.2) mediaDistancia -= 0.25;
      if (mediaDistancia >= 11) mediaDistancia += 1.2;
      if(mediaDistancia >= 13.7) mediaDistancia += 0.5;
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
               "{\"distancia\":%.1f,\"volume\":%.1f,\"bateria_v\":%.2f,"
               "\"bateria_pct\":%.0f,\"ldr_pct\":%.1f}",
               mediaDistancia, aguaNaGarrafa, vbat, pbat, ldrPct);

      Serial.println("✅ Leitura automática realizada:");
      Serial.println(buffer);

      if (pCharacteristic && pServer && pServer->getConnectedCount() > 0) {
        pCharacteristic->setValue(buffer);
        pCharacteristic->notify();
      }

      lendo = false;
    }
  }
}




// ---------- Callbacks BLE ----------
class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pChar) {
    String comando = String(pChar->getValue().c_str());
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
    Serial.println(" Dispositivo conectado!");
    advertisingAtivo = false;
  }

  void onDisconnect(BLEServer *pServer) {
    Serial.println(" Dispositivo desconectado! Reiniciando advertising...");
    delay(100);
    float vbat  = lerBateriaVolts();
    float pbat  = bateriaPercent(vbat);
    Serial.printf("Status bateria: %.2f V  (%.0f%%)\n", vbat, pbat);

    if (pAdvertising) {
      pAdvertising->start();
      advertisingAtivo = true;
    }
  }
};

// ---------- Fail-safe Timer ----------
void IRAM_ATTR onTimer() {
  if (pServer && pServer->getConnectedCount() == 0) {
    Serial.println(" Reboot forçado por timer (fail-safe - BLE desconectado).");
    esp_restart();
  } else {
    timerWrite(timerReboot, 0);
  }
}

// ---------- Setup ----------
void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_OFF);

  Wire.begin(5, 6);

  if (!lox.begin()) {
    Serial.println("Falha ao iniciar VL53L0X");
    while (1);
  }

  analogReadResolution(12);
  analogSetPinAttenuation(PINO_BAT, ADC_11db);
  analogSetPinAttenuation(pinoLDR, ADC_11db);

  esp_task_wdt_init(30, true);
  esp_task_wdt_add(NULL);

  // Timer de fail-safe
  timerReboot = timerBegin(0, 80, true);
  timerAttachInterrupt(timerReboot, &onTimer, true);
  timerAlarmWrite(timerReboot, FAILSAFE_INTERVAL_MS * 1000, true);
  timerAlarmEnable(timerReboot);

  // --- Inicializa BLE ---
  BLEDevice::init(BLE_DEVICE_NAME);
  BLEDevice::setPower(ESP_PWR_LVL_N9);
  BLEDevice::setMTU(SAFE_MTU);

  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_NOTIFY
  );

  // 🔧 descritor obrigatório para ativar notificações no nRF Connect
  pCharacteristic->addDescriptor(new BLE2902());

  pCharacteristic->setValue("Pronto para comandos: '1' = leituras");
  pCharacteristic->setCallbacks(new MyCallbacks());
  pService->start();

  pAdvertising = pServer->getAdvertising();
  pAdvertising->setScanResponse(true);
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMaxPreferred(0x12);
  pAdvertising->start();

  advertisingAtivo = true;
  delay(500);
  Serial.println("BLE pronto. Conecte pelo celular.");
}

// ---------- Loop ----------
void loop() {
  esp_task_wdt_reset();

  if (millis() - lastCheck > BLE_MONITOR_INTERVAL) {
    lastCheck = millis();
    if (pServer && pServer->getConnectedCount() == 0 && !advertisingAtivo) {
      Serial.println(" Advertising inativo. Tentando reiniciar...");
      pAdvertising->start();
      advertisingAtivo = true;
    }
  }

  int ldrValorBruto = analogRead(pinoLDR);
  static float ldrValor = 0;
  ldrValor = (ldrValor * 3 + ldrValorBruto) / 4;

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
