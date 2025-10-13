#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <driver/adc.h>

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
const int limiarEscuro = 10; // Ajuste conforme seu LDR

// ---------- Funções de bateria e LDR ----------
float lerBateriaVolts() {
  uint32_t mv = analogReadMilliVolts(PINO_BAT);
  float adcVolts = mv / 1000.0;
  return adcVolts * ((R1 + R2) / R2) - 0.1;
}

float bateriaPercent(float v) {
  if (v >= 4.20) return 100.0;
  if (v <= 3.30) return 0.0;
  struct PontoBat { float v; float pct; };
  const PontoBat curva[] = {
    {4.20,100},{4.10,90},{4.00,80},{3.90,70},
    {3.80,60},{3.70,50},{3.60,40},{3.50,25},
    {3.40,10},{3.30,0}
  };
  const int n = sizeof(curva)/sizeof(curva[0]);
  for(int i=0;i<n-1;i++){
    if(v<=curva[i].v && v>=curva[i+1].v){
      float x0=curva[i+1].v, y0=curva[i+1].pct;
      float x1=curva[i].v, y1=curva[i].pct;
      return y0 + (v-x0)*(y1-y0)/(x1-x0);
    }
  }
  return 0.0;
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
    String resposta;

    if (comando == "1") {
      iniciarLeituras();
      resposta = "Iniciando leituras...";
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

// ---------- Variáveis do loop ----------
unsigned long lastCheck = 0;

// ---------- Máquina de estados LDR ----------
enum EstadoCiclo { AGUARDANDO_LUZ, AGUARDANDO_ESCURO, LEITURA_FEITA };
EstadoCiclo estadoCiclo = AGUARDANDO_LUZ;

unsigned long tempoEscuro = 0;
const unsigned long tempoEscuroNecessario = 10000; // 10s de escuro

// ---------- Variáveis para leituras não bloqueantes ----------
const int totalLeituras = 50;
int leituraAtual = 0;
float somaDist = 0;
unsigned long ultimaLeitura = 0;
bool lendo = false;

// ---------- Funções de leituras não bloqueantes ----------
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
      if (d < 0) d = 0;
      somaDist += d;
    }

    leituraAtual++;

    if (leituraAtual >= totalLeituras) {
      // Calcular média e gerar JSON
      float mediaDistancia = somaDist / totalLeituras - 3.8;
      if(mediaDistancia <= 5) mediaDistancia -= 0.5;
      if(mediaDistancia >= 11) mediaDistancia += 1;
      if (mediaDistancia >= 13.5) mediaDistancia += 0.8;
      if (mediaDistancia >= 17.2) mediaDistancia += 1.2;
      if(mediaDistancia >= 19.8) mediaDistancia += 0.15;

      float alturaAgua = alturaGarrafa - mediaDistancia;
      if (alturaAgua < 0) alturaAgua = 0;

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

      String dados = String(buffer);
      Serial.println("Leitura automática realizada:");
      Serial.println(dados);

      if (pCharacteristic && pServer && pServer->getConnectedCount() > 0) {
        pCharacteristic->setValue(dados.c_str());
        pCharacteristic->notify();
      }

      lendo = false; // Ciclo finalizado
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

  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_DEFAULT, ESP_PWR_LVL_N18);
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_ADV, ESP_PWR_LVL_N18);
  esp_ble_tx_power_set(ESP_BLE_PWR_TYPE_SCAN, ESP_PWR_LVL_N18);

  delay(500);
  Serial.println("BLE pronto. Conecte pelo celular.");

  float vbat  = lerBateriaVolts();
  float pbat  = bateriaPercent(vbat);
  float ldrPct = lerLDRPercent();
  Serial.printf("Status bateria: %.2f V  (%.0f%%), LDR: %.1f%%\n", vbat, pbat, ldrPct);
}

// ---------- Loop ----------
void loop() {
  // --- BLE advertising ---
  if (millis() - lastCheck > 1000) {
    lastCheck = millis();
    if (pServer && pServer->getConnectedCount() == 0) {
      pServer->getAdvertising()->start();
    }
  }

  // --- LDR leitura ---
  int ldrValorBruto = analogRead(pinoLDR);
  static float ldrValor = 0;
  ldrValor = (ldrValor*3 + ldrValorBruto)/4; // média móvel simples
  Serial.printf("LDR: %.1f (bruto: %d)\n", ldrValor, ldrValorBruto);

  // --- Máquina de estados ciclo luz/escuro ---
  switch (estadoCiclo) {
    case AGUARDANDO_LUZ:
      if (ldrValor > limiarEscuro) {
        Serial.println("Luz suficiente detectada. Aguardando escuro...");
        estadoCiclo = AGUARDANDO_ESCURO;
      }
      break;

    case AGUARDANDO_ESCURO:
      if (ldrValor <= limiarEscuro) { // escuro detectado
        if (tempoEscuro == 0) {
          tempoEscuro = millis();
          Serial.println("Escuro detectado. Contando 10s...");
        } else if (millis() - tempoEscuro >= tempoEscuroNecessario) {
          Serial.println("Escuro confirmado. Iniciando leituras...");
          iniciarLeituras();
          estadoCiclo = LEITURA_FEITA;
        }
      } else {
        tempoEscuro = 0; // volta a monitorar escuro
      }
      break;

    case LEITURA_FEITA:
      if (!lendo) {
        Serial.println("Leitura finalizada. Voltando a monitorar luz...");
        estadoCiclo = AGUARDANDO_LUZ;
        tempoEscuro = 0;
      }
      break;
  }

  // --- Processa leituras não bloqueantes ---
  processarLeituras();
}
