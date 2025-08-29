#include <Wire.h>
#include <Adafruit_VL53L0X.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>

// UUIDs para BLE
#define SERVICE_UUID        "12345678-1234-1234-1234-1234567890ab"
#define CHARACTERISTIC_UUID "abcd1234-ab12-cd34-ef56-1234567890ab"

Adafruit_VL53L0X lox = Adafruit_VL53L0X();

BLEServer *pServer;
BLECharacteristic *pCharacteristic;

float alturaGarrafa = 24.0;
float diametroInterno = 7.8;
float raioInterno = diametroInterno / 2.0;
float zonaCega = 4.0; // cm zona cega do sensor

float aguaInicial = -1; // volume inicial em mL (na primeira medição)
float mlConsumidosAcumulado = 0; // acumulado do consumo

float aguaUltimaMedida = -1; // volume da última medição

String realizarLeituras() {
  float somaDistancias = 0;
  int leiturasValidas = 0;

  for (int i = 0; i < 10; i++) {
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure, false);

    if (measure.RangeStatus != 4) {
      float distancia = (measure.RangeMilliMeter / 10.0) - zonaCega;
      if (distancia < 0) distancia = 0;

      somaDistancias += distancia;
      leiturasValidas++;
    }

    delay(200);
  }

  if (leiturasValidas == 0) {
    return "Nenhuma leitura válida obtida.";
  }

  float mediaDistancia = somaDistancias / leiturasValidas;
  float alturaAgua = alturaGarrafa - mediaDistancia;
  if (alturaAgua < 0) alturaAgua = 0;

  float aguaNaGarrafa = 3.14159 * raioInterno * raioInterno * alturaAgua;

  if (aguaInicial < 0) {
    aguaInicial = aguaNaGarrafa;
  }
  if (aguaUltimaMedida < 0) {
    aguaUltimaMedida = aguaNaGarrafa; // primeira leitura
  }

  // Consumo baseado na última medição
  float consumoAtual = aguaUltimaMedida - aguaNaGarrafa;
  if (consumoAtual < 0) consumoAtual = 0; // evita valores negativos

  // Acumula no total
  mlConsumidosAcumulado += consumoAtual;

  // Atualiza última medição
  aguaUltimaMedida = aguaNaGarrafa;

  char buffer[256];
  snprintf(buffer, sizeof(buffer),
           "Média distância: %.1f cm\nVolume estimado: %.1f mL\nConsumo desta vez: %.1f mL\nConsumo acumulado: %.1f mL",
           mediaDistancia, aguaNaGarrafa, consumoAtual, mlConsumidosAcumulado);

  return String(buffer);
}


// Callback BLE para receber comandos do celular
class MyCallbacks : public BLECharacteristicCallbacks {
void onWrite(BLECharacteristic *pCharacteristic) {
  String comando = pCharacteristic->getValue();  // pega direto como Arduino String
  comando.trim();

  String resposta;

  if (comando == "1") {
    resposta = realizarLeituras();
  } else if (comando == "0") {
    mlConsumidosAcumulado = 0;
    aguaInicial = -1;
    resposta = "Consumo acumulado zerado.";
  } else {
    resposta = "Comando inválido. Envie '1' para leituras ou '0' para zerar.";
  }

  pCharacteristic->setValue(resposta.c_str());
  pCharacteristic->notify();

  Serial.println("Resposta enviada via BLE:");
  Serial.println(resposta);
  Serial.println("-------------------------");
}

};

void setup() {
  Serial.begin(115200);

  Wire.begin(8, 9); // ajuste seus pinos SDA e SCL se precisar

  if (!lox.begin()) {
    Serial.println("Falha ao iniciar VL53L0X");
    while (1);
  }

  BLEDevice::init("ESP32C3_AquaLink");
  pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
    CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_NOTIFY
  );

  pCharacteristic->setValue("Pronto para comandos: '1' = leituras, '0' = zerar consumo");
  pCharacteristic->setCallbacks(new MyCallbacks());

  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->setScanResponse(true);
  pAdvertising->addServiceUUID(SERVICE_UUID);
  BLEDevice::startAdvertising();

  Serial.println("BLE pronto. Conecte pelo celular.");
}

void loop() {
  // Não precisa de nada no loop, o BLE roda via callback
}
