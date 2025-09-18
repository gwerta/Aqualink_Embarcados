#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

// UUIDs para BLE
#define SERVICE_UUID "12345678-1234-1234-1234-1234567890ab"
#define CHARACTERISTIC_UUID "abcd1234-ab12-cd34-ef56-1234567890ab"

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

BLEServer *pServer;
BLECharacteristic *pCharacteristic;

float alturaGarrafa = 29.4;
float diametroInterno = 6.7;
int pinoLDR = 4;
int valorLDR = 0;
float raioInterno = diametroInterno / 2.0;

float aguaInicial = -1;       // volume inicial em mL (na primeira medição)
float aguaUltimaMedida = -1;  // volume da última medição

String realizarLeituras() {
  float somaDistancias = 0;
  int leiturasValidas = 0;

  for (int i = 0; i < 50; i++) {
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false);

    if (measure.RangeStatus != 4) {
      float distancia = (measure.RangeMilliMeter / 10.0);
      if (distancia < 0) distancia = 0;

      somaDistancias += distancia;
      leiturasValidas++;
    }

    delay(50);
  }

  if (leiturasValidas == 0) {
    return "Nenhuma leitura válida obtida.";
  }

  float mediaDistancia = somaDistancias / leiturasValidas - 3.5;
  if (mediaDistancia >= 26.5) {
    mediaDistancia += 3.5;
  }
  float alturaAgua = alturaGarrafa - mediaDistancia;
  if (alturaAgua < 0) alturaAgua = 0;

  float aguaNaGarrafa = 3.14159 * raioInterno * raioInterno * alturaAgua;

  if (aguaInicial < 0) {
    aguaInicial = aguaNaGarrafa;
  }
  if (aguaUltimaMedida < 0) {
    aguaUltimaMedida = aguaNaGarrafa;  // primeira leitura
  }


  aguaUltimaMedida = aguaNaGarrafa;

  char buffer[128];
  snprintf(buffer, sizeof(buffer),
           "{\"distancia\":%.1f,\"volume\":%.1f}",
           mediaDistancia, aguaNaGarrafa);
  return String(buffer);
}

// Callback BLE para receber comandos do celular
class MyCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    String comando = pCharacteristic->getValue();
    comando.trim();

    String resposta;

    if (comando == "1") {
      resposta = realizarLeituras();
    } else {
      resposta = "Comando inválido. Envie '1' para leituras.";
    }

    pCharacteristic->setValue(resposta.c_str());
    pCharacteristic->notify();

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
    pServer->getAdvertising()->start();
  }
};

void setup() {
  Serial.begin(115200);
  Wire.begin(8, 9);

  if (!lox.begin()) {
    Serial.println("Falha ao iniciar VL53L0X");
    while (1);
  }

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

  // Advertising a partir do servidor (agora pServer já existe)
  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->setScanResponse(true);
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setName("ESP32C3_AquaLink");

  // Sugestão: pedir intervalos de conexão menores (valores padrão em unidades de 1.25ms)
  // 0x06 -> 7.5 ms  ; 0x12 -> 37.5 ms (ajuste conforme desejar)
  pAdvertising->setMinPreferred(0x06);
  pAdvertising->setMaxPreferred(0x12);

  pAdvertising->start();

  Serial.println("BLE pronto. Conecte pelo celular.");
}

unsigned long lastCheck = 0;
void loop() {
  // Watchdog simples para garantir que o advertising esteja ativo
  if (millis() - lastCheck > 1000) {
    lastCheck = millis();
    if (pServer && pServer->getConnectedCount() == 0) {
      // start() é idempotente — garante que o dispositivo anuncie
      pServer->getAdvertising()->start();
    }
  }
}
