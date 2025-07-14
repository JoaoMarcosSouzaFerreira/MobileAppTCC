/**
 * @file ESP32_Bancada_Controller.ino
 * @author Adaptado por Gemini
 * @brief Versão 3.6 - Lógica de callback do ESP-NOW desacoplada e reinicialização do ESP-NOW.
 * @version 3.6
 */

// ==========================================================
// == BIBLIOTECAS
// ==========================================================
#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include "esp_eap_client.h"
#include <PubSubClient.h>

// ==========================================================
// == CONFIGURAÇÃO DOS PINOS
// ==========================================================
#define echoPin 18
#define trigPin 5
#define PUMP_PWM_PIN 23
#define IN1 16 // Pino de direção 1 da bomba
#define IN2 17 // Pino de direção 2 da bomba

// ==========================================================
// == CONFIGURAÇÃO ESP-NOW
// ==========================================================
uint8_t displayAddress[] = {0xE0, 0x5A, 0x1B, 0xAC, 0x23, 0x1C};

enum MessageType {
    WIFI_CREDENTIALS = 1,
    BROKER_CREDENTIALS = 2,
    CONTROL_PARAMETERS = 3,
    STATUS_UPDATE = 4,
    FEEDBACK_MESSAGE = 5,
    MQTT_STATUS_FEEDBACK = 6
};

typedef struct struct_wifi_credentials {
    int messageType;
    char ssid[32];
    char pass[64];
    char email[64];
    bool is_corp;
} struct_wifi_credentials;

typedef struct struct_broker_credentials {
    int messageType;
    char ip[16];
    char port[6];
    char user[64];
    char pass[64];
    bool use_auth;
} struct_broker_credentials;

typedef struct struct_control_parameters {
    int messageType;
    float ref;
    float nu;
    float nx;
    float k;
    float ke;
} struct_control_parameters;

typedef struct struct_status_update {
    int messageType;
    int experiment_id;
    int bench_status;
    float water_level;
    float reference;
    int next_experiment_id;
} struct_status_update;

typedef struct struct_feedback_message {
    int messageType;
    char message[100];
} struct_feedback_message;

typedef struct struct_mqtt_status_feedback {
    int messageType;
    bool success;
} struct_mqtt_status_feedback;

esp_now_peer_info_t peerInfo;

// --- Buffers e Flags para Comunicação Assíncrona ---
struct_wifi_credentials incomingWifiCreds;
struct_broker_credentials incomingBrokerCreds;
volatile bool newWifiCredsReceived = false;
volatile bool newBrokerCredsReceived = false;

// ==========================================================
// == VARIÁVEIS DE CONTROLE E ESTADO
// ==========================================================
const double A = -0.006, B = 0.002, C = 1.0, D = 0.0;
double K = 0.0, Ke = 0.0, Nx = 0.0, Nu = 0.0, r = 0.0;
double x_chap = 0.0, u = 0.0;
long duration;
float distance_cm;
unsigned long last_time_control = 0;
unsigned long last_time_comms = 0;
const long interval_control = 10; // 100Hz
const long interval_comms = 500; // 2Hz

enum ExperimentState { IDLE, RUNNING };
ExperimentState experimentState = IDLE;

// ==========================================================
// == VARIÁVEIS MQTT
// ==========================================================
char mqtt_server[16];
int mqtt_port = 1883;
char mqtt_user[64] = "";
char mqtt_pass[64] = "";
bool use_auth_mqtt = false;

WiFiClient espClient;
PubSubClient client(espClient);

// ==========================================================
// == FUNÇÕES DE CONEXÃO E CALLBACKS
// ==========================================================
void mqtt_callback(char* topic, byte* payload, unsigned int length);
void sendFeedbackToDisplay(const char* msg);
void OnDataRecv(const esp_now_recv_info *info, const uint8_t *incomingData, int len);
void connectToWifi(struct_wifi_credentials creds);
void connectToMqttBroker(struct_broker_credentials creds);
void reinit_esp_now();


void sendMqttStatusToDisplay(bool success) {
    struct_mqtt_status_feedback feedback;
    feedback.messageType = MQTT_STATUS_FEEDBACK;
    feedback.success = success;
    esp_now_send(displayAddress, (uint8_t *) &feedback, sizeof(feedback));
}

void reconnectMQTT() {
    int retries = 0;
    while (!client.connected() && retries < 5) {
        sendFeedbackToDisplay("Tentando conectar ao Broker MQTT...");
        Serial.print("Tentando conectar ao MQTT Broker...");
        
        const char* user = use_auth_mqtt ? mqtt_user : NULL;
        const char* pass = use_auth_mqtt ? mqtt_pass : NULL;

        if (client.connect("esp32_bancada_client", user, pass)) {
            Serial.println("conectado!");
            sendFeedbackToDisplay("Conectado ao Broker com sucesso!");
            client.subscribe("encerraExperimento");
            sendMqttStatusToDisplay(true);
            return;
        } else {
            Serial.print("falhou, rc=");
            Serial.print(client.state());
            Serial.println(" tentando novamente em 2 segundos");
            retries++;
            delay(2000);
        }
    }
    Serial.println("Nao foi possivel conectar ao Broker.");
    sendFeedbackToDisplay("Falha ao conectar ao Broker.");
    sendMqttStatusToDisplay(false);
}

void connectToMqttBroker(struct_broker_credentials creds) {
    strncpy(mqtt_server, creds.ip, sizeof(mqtt_server) - 1);
    mqtt_port = atoi(creds.port);
    use_auth_mqtt = creds.use_auth;
    if(use_auth_mqtt) {
        strncpy(mqtt_user, creds.user, sizeof(mqtt_user) - 1);
        strncpy(mqtt_pass, creds.pass, sizeof(mqtt_pass) - 1);
    }
    
    client.setServer(mqtt_server, mqtt_port);
    client.setCallback(mqtt_callback);
    reconnectMQTT();
}


void connectToWifi(struct_wifi_credentials creds) {
    sendFeedbackToDisplay("Recebidas credenciais Wi-Fi...");
    WiFi.disconnect(true);
    delay(100);

    if (creds.is_corp) {
        esp_eap_client_set_identity((uint8_t *)creds.email, strlen(creds.email));
        esp_eap_client_set_username((uint8_t *)creds.email, strlen(creds.email));
        esp_eap_client_set_password((uint8_t *)creds.pass, strlen(creds.pass));
        esp_wifi_sta_enterprise_enable();
        WiFi.begin(creds.ssid);
    } else {
        WiFi.begin(creds.ssid, creds.pass);
    }

    int retries = 0;
    sendFeedbackToDisplay("Conectando ao Wi-Fi...");
    while (WiFi.status() != WL_CONNECTED && retries < 30) {
        delay(500);
        Serial.print(".");
        retries++;
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nWiFi Conectado!");
        sendFeedbackToDisplay("WiFi Conectado!");
        reinit_esp_now(); // Re-inicializa o ESP-NOW após a conexão
    } else {
        Serial.println("\nFalha ao conectar ao WiFi.");
        sendFeedbackToDisplay("Falha ao conectar ao WiFi.");
    }
}

void reinit_esp_now() {
    esp_now_deinit();
    if (esp_now_init() != ESP_OK) {
        Serial.println("Erro ao re-inicializar ESP-NOW");
        return;
    }
    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv);
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
        Serial.println("Falha ao re-adicionar par (Display)");
        return;
    }
    Serial.println("ESP-NOW re-inicializado com sucesso.");
}


void OnDataRecv(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
    int messageType;
    memcpy(&messageType, incomingData, sizeof(messageType));

    // Apenas copia os dados e define uma flag. O processamento é feito no loop.
    switch (messageType) {
        case WIFI_CREDENTIALS: {
            memcpy(&incomingWifiCreds, incomingData, sizeof(struct_wifi_credentials));
            newWifiCredsReceived = true;
            break;
        }
        case BROKER_CREDENTIALS: {
            memcpy(&incomingBrokerCreds, incomingData, sizeof(struct_broker_credentials));
            newBrokerCredsReceived = true;
            break;
        }
        case CONTROL_PARAMETERS: {
            struct_control_parameters controlData;
            memcpy(&controlData, incomingData, sizeof(controlData));
            r = controlData.ref;
            Nu = controlData.nu;
            Nx = controlData.nx;
            K = controlData.k;
            Ke = controlData.ke;
            Serial.println("Parametros de controle atualizados.");
            sendFeedbackToDisplay("Parametros de controle recebidos.");
            experimentState = IDLE;
            break;
        }
        default:
            Serial.printf("Tipo de mensagem desconhecido recebido: %d\n", messageType);
            break;
    }
}

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    // Callback para confirmar envio de status para o display
}

void mqtt_callback(char* topic, byte* payload, unsigned int length) {
    String message;
    for (unsigned int i = 0; i < length; i++) {
        message += (char)payload[i];
    }

    if (String(topic) == "encerraExperimento") {
        if (message == "START") {
            experimentState = RUNNING;
            sendFeedbackToDisplay("Experimento Iniciado!");
            Serial.println("Comando START recebido. Iniciando experimento.");
        } else if (message == "STOP") {
            experimentState = IDLE;
            u = 0;
            analogWrite(PUMP_PWM_PIN, 0);
            sendFeedbackToDisplay("Experimento Parado.");
            Serial.println("Comando STOP recebido. Parando experimento.");
        }
    }
}

void sendFeedbackToDisplay(const char* msg) {
    struct_feedback_message feedback;
    feedback.messageType = FEEDBACK_MESSAGE;
    strncpy(feedback.message, msg, sizeof(feedback.message) - 1);
    feedback.message[sizeof(feedback.message) - 1] = '\0';
    esp_now_send(displayAddress, (uint8_t *) &feedback, sizeof(feedback));
}


// ==========================================================
// == SETUP
// ==========================================================
void setup() {
    Serial.begin(115200);
    
    pinMode(trigPin, OUTPUT);
    pinMode(echoPin, INPUT);
    
    pinMode(PUMP_PWM_PIN, OUTPUT);
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
    analogWrite(PUMP_PWM_PIN, 0);

    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) {
        Serial.println("Erro ao inicializar ESP-NOW");
        return;
    }
    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv);

    memcpy(peerInfo.peer_addr, displayAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
        Serial.println("Falha ao adicionar par (Display)");
        return;
    }
    
    Serial.println("ESP da Bancada pronto para comunicacao.");
}

// ==========================================================
// == LOOP PRINCIPAL
// ==========================================================
void loop() {
    // --- Processamento de Dados Recebidos via ESP-NOW ---
    if (newWifiCredsReceived) {
        newWifiCredsReceived = false;
        Serial.println("Processando credenciais WiFi recebidas...");
        connectToWifi(incomingWifiCreds);
    }

    if (newBrokerCredsReceived) {
        newBrokerCredsReceived = false;
        Serial.println("Processando credenciais do Broker recebidas...");
        connectToMqttBroker(incomingBrokerCreds);
    }
    
    // --- Lógica MQTT ---
    if (WiFi.status() == WL_CONNECTED && !client.connected()) {
        reconnectMQTT();
    }
    client.loop();

    unsigned long currentMillis = millis();

    // --- Lógica de Controle ---
    if (currentMillis - last_time_control > interval_control) {
        last_time_control = currentMillis;

        digitalWrite(trigPin, LOW);
        delayMicroseconds(2);
        digitalWrite(trigPin, HIGH);
        delayMicroseconds(10);
        digitalWrite(trigPin, LOW);
        duration = pulseIn(echoPin, HIGH);
        distance_cm = duration * 0.0344 / 2;
        double y = distance_cm;

        if (experimentState == RUNNING) {
            double xss = Nx * r;
            double uss = Nu * r;
            u = uss - K * (x_chap - xss);

            u = constrain(u, 0.0, 100.0);
            
            analogWrite(PUMP_PWM_PIN, map(u, 0, 100, 0, 255));

            double x_chap_p = A * x_chap + B * u + Ke * (y - C * x_chap);
            x_chap = x_chap + x_chap_p * (interval_control / 1000.0);
        }
    }

    // --- Lógica de Comunicação (Envio de Status) ---
    if (currentMillis - last_time_comms > interval_comms) {
        last_time_comms = currentMillis;
        
        struct_status_update statusData;
        statusData.messageType = STATUS_UPDATE;
        statusData.water_level = distance_cm;
        statusData.reference = r;
        statusData.bench_status = (experimentState == RUNNING) ? 2 : (WiFi.status() == WL_CONNECTED ? 1 : 0);
        statusData.experiment_id = (experimentState == RUNNING) ? 1 : 0; // Exemplo
        statusData.next_experiment_id = 0; // Exemplo
        esp_now_send(displayAddress, (uint8_t *) &statusData, sizeof(statusData));

        if (client.connected() && experimentState == RUNNING) {
            char payload[10];
            dtostrf(millis() / 1000.0, 6, 2, payload);
            client.publish("tempo", payload);
            dtostrf(distance_cm, 6, 2, payload);
            client.publish("nivel", payload);
            dtostrf(u * (12.0/100.0), 6, 2, payload);
            client.publish("tensao", payload);
            dtostrf(x_chap, 6, 2, payload);
            client.publish("estimado", payload);
        }
    }
}
