#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <esp_wpa2.h>

// --- Definições dos Pinos ---
#define trigPin 23
#define echoPin 18
#define pinEnableBomba 5
#define pinSentido1 7
#define pinSentido2 8

// --- Parâmetros Físicos do Tanque ---
#define ALTURA_TOTAL_SENSOR_FUNDO 37.0 // Distância em cm do sensor ao fundo do tanque
#define ALTURA_MAXIMA_AGUA 17.0        // Nível máximo controlável (37cm - 20cm de leitura mínima)

// --- Definições de UUIDs para o BLE ---
#define SERVICE_UUID              "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define WRITE_CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define NOTIFY_CHARACTERISTIC_UUID "1c95d2e3-d343-41a7-9336-55d315632d38"

BLECharacteristic *pNotifyCharacteristic;
bool deviceConnected = false;

// --- Variáveis Globais ---
String wifiType = "", wifiSSID = "", wifiPassword = "", wifiUser = "", mqttBrokerIP = "";
int mqttBrokerPort = 0;
double u = 0.0, y = 0.0, duracao, distancia, dt = 10;
double x_ponto = 0.0, x_ponto_p = 0.0, uss = 0.0, xss = 0.0;
double rss = 0.0;
double K = 1.0, Ke = 1.0, Nx = 1.0, Nu = 1.0;
bool newCredentialsReceived = false;
bool experimentoEmAndamento = false;

WiFiClient espClient;
PubSubClient client(espClient);

void callback(char* topic, byte* payload, unsigned int length);

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) { deviceConnected = true; Serial.println("Dispositivo conectado via BLE."); };
    void onDisconnect(BLEServer* pServer) { deviceConnected = false; Serial.println("Dispositivo desconectado."); BLEDevice::startAdvertising(); }
};

String getValue(String data, char separator, int index) {
  int found = 0, strIndex[] = {0, -1}, maxIndex = data.length() - 1;
  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) { found++; strIndex[0] = strIndex[1] + 1; strIndex[1] = (i == maxIndex) ? i + 1 : i; }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}

void processarDadosRecebidos(std::string dados) {
    String strDados = String(dados.c_str());
    wifiType = getValue(strDados, '|', 0);
    wifiSSID = getValue(strDados, '|', 1);
    wifiPassword = getValue(strDados, '|', 2);
    wifiUser = getValue(strDados, '|', 3);
    mqttBrokerIP = getValue(strDados, '|', 4);
    mqttBrokerPort = getValue(strDados, '|', 5).toInt();
    if (mqttBrokerIP.length() > 0 && wifiSSID.length() > 0) newCredentialsReceived = true;
}

class MyWriteCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      std::string value = pCharacteristic->getValue();
      if (value.length() > 0) processarDadosRecebidos(value);
    }
};

void conectarWiFiEMQTT() {
    Serial.println("\nIniciando processo de conexão...");
    WiFi.disconnect(true); delay(1000);
    if (wifiType == "PSK") { WiFi.begin(wifiSSID.c_str(), wifiPassword.c_str()); } 
    else if (wifiType == "PEAP") {
        esp_wifi_sta_wpa2_ent_set_identity((uint8_t *)wifiUser.c_str(), wifiUser.length());
        esp_wifi_sta_wpa2_ent_set_username((uint8_t *)wifiUser.c_str(), wifiUser.length());
        esp_wifi_sta_wpa2_ent_set_password((uint8_t *)wifiPassword.c_str(), wifiPassword.length());
        esp_wpa2_config_t config = WPA2_CONFIG_INIT_DEFAULT();
        esp_wifi_sta_wpa2_ent_enable(&config);
        WiFi.begin(wifiSSID.c_str());
    }
    int tentativas = 0;
    while (WiFi.status() != WL_CONNECTED && tentativas < 30) { delay(500); Serial.print("."); tentativas++; }
    
    client.setServer(mqttBrokerIP.c_str(), mqttBrokerPort);
    client.setCallback(callback);
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("\nFalha ao conectar ao Wi-Fi.");
        if(client.connect("ESP32-Bancada-Status")) { client.publish("estadoESPWifi", "Falha"); client.disconnect(); }
        return;
    }

    Serial.println("\nWi-Fi conectado!");
    if(client.connect("ESP32-Bancada-Controle")) {
        Serial.println("Conectado ao Broker MQTT!");
        client.publish("estadoESPWifi", "Conectado");
        client.publish("estadoESPBroker", "Conectado");
        client.publish("estadoExperimento", "Parado");
        client.subscribe("observadorKe"); client.subscribe("reguladorK");
        client.subscribe("nx"); client.subscribe("nu");
        client.subscribe("referencia"); client.subscribe("encerraExperimento");
    } else {
        Serial.print("Falha na conexão MQTT, rc="); Serial.println(client.state());
        if(client.connect("ESP32-Bancada-Status")) { client.publish("estadoESPBroker", "Falha"); client.disconnect(); }
    }
}

void callback(char* topic, byte* payload, unsigned int length) {
  String message;
  for (int i = 0; i < length; i++) { message += (char)payload[i]; }
  
  if (strcmp(topic, "reguladorK") == 0) K = message.toDouble();
  else if (strcmp(topic, "observadorKe") == 0) Ke = message.toDouble();
  else if (strcmp(topic, "nx") == 0) Nx = message.toDouble();
  else if (strcmp(topic, "nu") == 0) Nu = message.toDouble();
  else if (strcmp(topic, "referencia") == 0) rss = message.toDouble();
  else if (strcmp(topic, "encerraExperimento") == 0) {
    if (message.equalsIgnoreCase("START")) {
      experimentoEmAndamento = true;
      client.publish("estadoExperimento", "Em Andamento");
    } else if (message.equalsIgnoreCase("STOP")) {
      experimentoEmAndamento = false;
      u = 0;
      analogWrite(pinEnableBomba, 0);
      client.publish("estadoExperimento", "Parado");
    }
  }
}

void setup() {
  Serial.begin(115200);
  BLEDevice::init("ESP32-Bancada-Controle");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  BLECharacteristic *pWriteCharacteristic = pService->createCharacteristic(WRITE_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_WRITE);
  pWriteCharacteristic->setCallbacks(new MyWriteCallbacks());
  pNotifyCharacteristic = pService->createCharacteristic(NOTIFY_CHARACTERISTIC_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  pService->start();
  BLEDevice::getAdvertising()->addServiceUUID(SERVICE_UUID);
  BLEDevice::startAdvertising();
  pinMode(pinEnableBomba, OUTPUT); pinMode(pinSentido1, OUTPUT); pinMode(pinSentido2, OUTPUT);
  digitalWrite(pinSentido1, HIGH); digitalWrite(pinSentido2, LOW);
  pinMode(trigPin, OUTPUT); pinMode(echoPin, INPUT);
}

void loop() {
  if (newCredentialsReceived) { newCredentialsReceived = false; conectarWiFiEMQTT(); }
  if (!client.connected() && WiFi.status() == WL_CONNECTED && mqttBrokerIP.length() > 0) {
      conectarWiFiEMQTT();
  }
  client.loop();

  if (experimentoEmAndamento) {
    unsigned long inicioLoop = millis();
    digitalWrite(trigPin, LOW); delayMicroseconds(2);
    digitalWrite(trigPin, HIGH); delayMicroseconds(10);
    digitalWrite(trigPin, LOW);
    duracao = pulseIn(echoPin, HIGH);
    
    // --- CÁLCULO CORRETO DO NÍVEL ---
    distancia = duracao * 0.0344 / 2;
    y = ALTURA_TOTAL_SENSOR_FUNDO - distancia; // Conversão de distância para altura da coluna
    y = constrain(y, 0, ALTURA_MAXIMA_AGUA);   // Garante que o valor fique dentro dos limites físicos

    xss = Nx * rss; uss = Nu * rss;
    u = uss - K * (x_ponto - xss);
    u = constrain(u, 0, 100);
    analogWrite(pinEnableBomba, map(u, 0, 100, 0, 255));

    x_ponto_p = (-0.0052 * x_ponto) + (0.0197 * u) + Ke * (y - x_ponto);
    x_ponto += x_ponto_p * (dt / 1000.0);

    static unsigned long lastPubTime = 0;
    if (millis() - lastPubTime > 500) {
        lastPubTime = millis();
        char buffer[10];
        dtostrf(y, 4, 2, buffer); client.publish("nivel", buffer);
        dtostrf(u, 4, 2, buffer); client.publish("tensao", buffer);
        dtostrf(x_ponto, 4, 2, buffer); client.publish("estimado", buffer);
        dtostrf(millis() / 1000.0, 4, 2, buffer); client.publish("tempo", buffer);
        if (deviceConnected) { dtostrf(y, 4, 2, buffer); pNotifyCharacteristic->setValue(buffer); pNotifyCharacteristic->notify(); }
    }
    
    while((millis() - inicioLoop) < dt);
  }
}
