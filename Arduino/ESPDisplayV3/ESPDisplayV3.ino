/**
 * @file ESP32_Display_Controller.ino
 * @author Adaptado por Gemini
 * @brief Versão 12.9 - Lógica de UI e comunicação com a bancada refinada.
 * @version 12.9
 */

// ==========================================================
// == BIBLIOTECAS
// ==========================================================
#include <lvgl.h>
#include <TFT_eSPI.h>
#include <XPT2046_Touchscreen.h>
#include <WiFi.h>
#include <esp_now.h>
#include "esp_eap_client.h"
#include <freertos/semphr.h>
#include "time.h"

// ==========================================================
// == CONFIGURAÇÃO DE HARDWARE (DO SEU DISPLAY)
// ==========================================================
#define XPT2046_IRQ 36
#define XPT2046_MOSI 32
#define XPT2046_MISO 39
#define XPT2046_CLK 25
#define XPT2046_CS 33

#define TOUCH_X_MIN 200
#define TOUCH_X_MAX 3700
#define TOUCH_Y_MIN 240
#define TOUCH_Y_MAX 3800

TFT_eSPI tft = TFT_eSPI();
SPIClass touchscreenSPI = SPIClass(VSPI);
XPT2046_Touchscreen touchscreen(XPT2046_CS);

// ==========================================================
// == DEFINIÇÕES E ESTRUTURAS DE DADOS ESP-NOW
// ==========================================================
uint8_t broadcastAddress[] = {0x34, 0xAB, 0x95, 0x9B, 0xE1, 0xEC};

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

// ==========================================================
// == CONFIGURAÇÃO LVGL E GERAL
// ==========================================================
static const uint32_t screenWidth = 320;
static const uint32_t screenHeight = 240;
static const uint32_t WIFI_CONNECT_TIMEOUT_MS = 15000;

static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[screenWidth * 10];

const char *ntpServer = "pool.ntp.org";
const long gmtOffset_sec = -3 * 60 * 60;
const int daylightOffset_sec = 0;

typedef enum {
    NONE, NETWORK_SEARCHING, NETWORK_CONNECTING,
    NETWORK_CONNECTED_POPUP, NETWORK_CONNECT_FAILED_POPUP,
    NETWORK_CONNECTED
} Network_Status_t;
Network_Status_t networkStatus = NONE;

typedef enum {
    MQTT_DISCONNECTED, MQTT_CONNECTING, MQTT_CONNECTED, MQTT_FAILED
} Mqtt_Status_t;
Mqtt_Status_t mqttStatus = MQTT_DISCONNECTED;

typedef enum {
    BENCH_NOT_READY, BENCH_READY, BENCH_RUNNING
} Bench_Status_t;
Bench_Status_t benchStatus = BENCH_NOT_READY;

// --- Ponteiros para as Telas ---
static lv_obj_t *screen_home, *screen_wifi_config, *screen_broker_config, *screen_selection, *screen_topics_config, *screen_experiment_tracking;

// --- Ponteiros para Objetos da UI ---
static lv_obj_t *wifi_list, *popup_universal, *popup_title, *popup_message, *popup_email_container, *popup_password_container, *popup_btn_container;
static lv_obj_t *wifi_email_ta, *wifi_password_ta, *keyboard, *wifi_status_label, *status_bar_bench_status_icon, *status_bar_upload_icon, *status_bar_download_icon;
static lv_obj_t *broker_ip_ta, *broker_port_ta, *broker_user_ta, *broker_password_ta, *broker_auth_container, *selection_broker_status_label;
static lv_obj_t *topics_ref_ta, *topics_nu_ta, *topics_nx_ta, *topics_k_ta, *topics_ke_ta;
static lv_obj_t *exp_water_level_label, *exp_reference_label, *exp_status_label, *exp_current_label, *exp_next_label;
static lv_timer_t *network_timer, *popup_timer, *comm_icon_timer;
static lv_obj_t *popup_info_container, *popup_signal_label, *popup_password_view_ta;

// --- Variáveis de Dados e Estado ---
#define MAX_SSID_LENGTH 32
#define MAX_PASSWORD_LENGTH 64
#define MAX_EMAIL_LENGTH 64
#define MAX_WIFI_NETWORKS 20
#define MAX_BROKER_IP_LENGTH 16
#define MAX_BROKER_PORT_LENGTH 6
#define MAX_BROKER_USER_LENGTH 64
#define MAX_BROKER_PW_LENGTH 64
#define MAX_TOPIC_VALUE_LENGTH 32

char ssidName[MAX_SSID_LENGTH], ssidPW[MAX_PASSWORD_LENGTH], email_user[MAX_EMAIL_LENGTH];
bool is_corporate_network = false;
bool came_from_local_mode = false;
bool is_sending = false, is_receiving = false;

char foundWifiList[MAX_WIFI_NETWORKS][MAX_SSID_LENGTH];
wifi_auth_mode_t foundWifiAuth[MAX_WIFI_NETWORKS];
int foundNetworksCount = 0, displayedNetworksCount = 0;
SemaphoreHandle_t wifi_data_mutex;
wifi_auth_mode_t selected_auth_mode;
TaskHandle_t ntScanTaskHandler = NULL;

// --- Estruturas e Flags para Comunicação Segura entre Tarefas ---
struct_status_update lastReceivedStatus;
struct_feedback_message lastFeedbackMsg;
struct_mqtt_status_feedback lastMqttStatus;
volatile bool newStatusDataAvailable = false;
volatile bool newFeedbackMsgAvailable = false;
volatile bool newMqttStatusAvailable = false;

// --- Declaração das Funções ---
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);
void OnDataRecv(const esp_now_recv_info *info, const uint8_t *incomingData, int len);
void ui_update_timer_cb(lv_timer_t *timer);
void popup_timer_cb(lv_timer_t *timer);
void comm_icon_timer_cb(lv_timer_t *timer);
void show_temporary_popup(const char* title, const char* message);
void create_home_screen();
void create_wifi_config_screen();
void create_broker_config_screen();
void create_selection_screen();
void create_topics_config_screen();
void create_experiment_tracking_screen();
void create_universal_popup();
void show_message_popup(const char* title, const char* msg, const char* btn1_txt, const char* btn2_txt);
void show_credentials_popup();
void show_connected_wifi_popup();
void create_persistent_status_bar();
void create_keyboard();
void main_event_handler(lv_event_t *e);
void textarea_event_handler(lv_event_t *e);
void keyboard_event_handler(lv_event_t *e);
void background_click_event_handler(lv_event_t *e);
void network_timer_cb(lv_timer_t *timer);
void networkScannerTask(void *pvParameters);
void networkConnectorTask(void *pvParameters);
void startWifiScan();
void updateLocalTime();
void update_broker_status_label();
void update_bench_status_icon(Bench_Status_t status);
void update_communication_icons(bool sending, bool receiving);

// ==========================================================
// == FUNÇÕES DE COMUNICAÇÃO LVGL <-> HARDWARE
// ==========================================================
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);
    tft.startWrite();
    tft.setAddrWindow(area->x1, area->y1, w, h);
    tft.pushImage(area->x1, area->y1, w, h, (uint16_t *)&color_p->full);
    tft.endWrite();
    lv_disp_flush_ready(disp);
}

void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data) {
    if (touchscreen.touched()) {
        TS_Point p = touchscreen.getPoint();
        data->point.x = map(p.x, TOUCH_X_MIN, TOUCH_X_MAX, 0, screenWidth);
        data->point.y = map(p.y, TOUCH_Y_MIN, TOUCH_Y_MAX, 0, screenHeight);
        data->state = LV_INDEV_STATE_PR;
    } else {
        data->state = LV_INDEV_STATE_REL;
    }
}

// ==========================================================
// == SETUP E LOOP PRINCIPAL
// ==========================================================
void setup() {
    Serial.begin(115200);
    Serial.println("Iniciando Projeto v12.9 - UI Final com Feedback");

    wifi_data_mutex = xSemaphoreCreateMutex();

    tft.begin();
    tft.setRotation(1);
    tft.setSwapBytes(true);
    tft.fillScreen(TFT_BLACK);

    touchscreenSPI.begin(XPT2046_CLK, XPT2046_MISO, XPT2046_MOSI, XPT2046_CS);
    touchscreen.begin(touchscreenSPI);
    touchscreen.setRotation(1);

    lv_init();
    lv_disp_draw_buf_init(&draw_buf, buf, NULL, screenWidth * 10);

    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = screenWidth;
    disp_drv.ver_res = screenHeight;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &draw_buf;
    lv_disp_drv_register(&disp_drv);

    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touchpad_read;
    lv_indev_drv_register(&indev_drv);
    
    WiFi.mode(WIFI_STA);
    if (esp_now_init() != ESP_OK) {
        Serial.println("Erro ao inicializar ESP-NOW");
        return;
    }
    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv);
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;
    peerInfo.encrypt = false;
    if (esp_now_add_peer(&peerInfo) != ESP_OK){
        Serial.println("Falha ao adicionar par");
        return;
    }

    create_persistent_status_bar();
    create_home_screen();
    create_wifi_config_screen();
    create_broker_config_screen();
    create_selection_screen();
    create_topics_config_screen();
    create_experiment_tracking_screen();
    create_universal_popup();
    create_keyboard();
    
    lv_timer_create(ui_update_timer_cb, 30, NULL);

    lv_scr_load(screen_home);
}

void loop() {
    lv_timer_handler();
    delay(5);
}

// ==========================================================
// == CALLBACKS ESP-NOW E TIMERS
// ==========================================================
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.print("\r\nStatus do ultimo envio: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Entregue com Sucesso" : "Falha na Entrega");
    update_communication_icons(false, is_receiving);
}

void OnDataRecv(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
    int messageType;
    memcpy(&messageType, incomingData, sizeof(messageType));

    switch(messageType) {
        case STATUS_UPDATE:
            memcpy(&lastReceivedStatus, incomingData, sizeof(lastReceivedStatus));
            newStatusDataAvailable = true;
            break;
        case FEEDBACK_MESSAGE:
            memcpy(&lastFeedbackMsg, incomingData, sizeof(lastFeedbackMsg));
            newFeedbackMsgAvailable = true;
            break;
        case MQTT_STATUS_FEEDBACK:
            memcpy(&lastMqttStatus, incomingData, sizeof(lastMqttStatus));
            newMqttStatusAvailable = true;
            break;
    }
}

void ui_update_timer_cb(lv_timer_t *timer) {
    if (newStatusDataAvailable) {
        newStatusDataAvailable = false;

        update_communication_icons(is_sending, true);
        update_bench_status_icon((Bench_Status_t)lastReceivedStatus.bench_status);
        
        char buffer[32];
        sprintf(buffer, "%.2f cm", lastReceivedStatus.water_level);
        lv_label_set_text(exp_water_level_label, buffer);

        sprintf(buffer, "%.2f cm", lastReceivedStatus.reference);
        lv_label_set_text(exp_reference_label, buffer);
        
        if (lastReceivedStatus.experiment_id > 0) {
            sprintf(buffer, "#%d", lastReceivedStatus.experiment_id);
            lv_label_set_text(exp_current_label, buffer);
        } else {
            lv_label_set_text(exp_current_label, "N/A");
        }

        if (lastReceivedStatus.next_experiment_id > 0) {
            sprintf(buffer, "#%d", lastReceivedStatus.next_experiment_id);
            lv_label_set_text(exp_next_label, buffer);
        } else {
            lv_label_set_text(exp_next_label, "N/A");
        }
        
        if(comm_icon_timer) lv_timer_del(comm_icon_timer);
        comm_icon_timer = lv_timer_create(comm_icon_timer_cb, 500, NULL);
        lv_timer_set_repeat_count(comm_icon_timer, 1);
    }

    if (newFeedbackMsgAvailable) {
        newFeedbackMsgAvailable = false;
        show_temporary_popup("Feedback da Bancada", lastFeedbackMsg.message);
    }

    if (newMqttStatusAvailable) {
        newMqttStatusAvailable = false;
        mqttStatus = lastMqttStatus.success ? MQTT_CONNECTED : MQTT_FAILED;
        update_broker_status_label();
    }
}

void popup_timer_cb(lv_timer_t *timer) {
    lv_obj_add_flag(popup_universal, LV_OBJ_FLAG_HIDDEN);
    lv_timer_del(timer);
    popup_timer = NULL;
}

void comm_icon_timer_cb(lv_timer_t *timer) {
    update_communication_icons(is_sending, false);
    lv_timer_del(timer);
    comm_icon_timer = NULL;
}

void show_temporary_popup(const char* title, const char* message) {
    show_message_popup(title, message, NULL, NULL);
    if(popup_timer) lv_timer_del(popup_timer);
    popup_timer = lv_timer_create(popup_timer_cb, 5000, NULL);
    lv_timer_set_repeat_count(popup_timer, 1);
}

// ==========================================================
// == CONSTRUÇÃO DA INTERFACE GRÁFICA (UI)
// ==========================================================

void create_persistent_status_bar() {
    lv_obj_t *statusBar = lv_obj_create(lv_layer_top());
    lv_obj_set_size(statusBar, screenWidth, 25);
    lv_obj_align(statusBar, LV_ALIGN_TOP_MID, 0, 0);
    lv_obj_remove_style(statusBar, NULL, LV_PART_SCROLLBAR | LV_STATE_ANY);
    
    lv_obj_set_style_bg_color(statusBar, lv_color_white(), 0);
    lv_obj_set_style_border_width(statusBar, 0, 0);
    lv_obj_set_style_radius(statusBar, 0, 0);

    wifi_status_label = lv_label_create(statusBar);
    lv_obj_set_style_text_color(wifi_status_label, lv_palette_main(LV_PALETTE_GREY), 0);
    lv_label_set_text(wifi_status_label, LV_SYMBOL_WIFI " /");
    lv_obj_align(wifi_status_label, LV_ALIGN_LEFT_MID, 5, 0);

    status_bar_bench_status_icon = lv_label_create(statusBar);
    lv_obj_align_to(status_bar_bench_status_icon, wifi_status_label, LV_ALIGN_OUT_RIGHT_MID, 10, 0);
    update_bench_status_icon(benchStatus);

    status_bar_download_icon = lv_label_create(statusBar);
    lv_label_set_text(status_bar_download_icon, LV_SYMBOL_DOWNLOAD);
    lv_obj_add_flag(status_bar_download_icon, LV_OBJ_FLAG_HIDDEN);
    lv_obj_align(status_bar_download_icon, LV_ALIGN_RIGHT_MID, -30, 0);

    status_bar_upload_icon = lv_label_create(statusBar);
    lv_label_set_text(status_bar_upload_icon, LV_SYMBOL_UPLOAD);
    lv_obj_add_flag(status_bar_upload_icon, LV_OBJ_FLAG_HIDDEN);
    lv_obj_align(status_bar_upload_icon, LV_ALIGN_RIGHT_MID, -5, 0);
}

void create_home_screen() {
    screen_home = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(screen_home, lv_color_white(), 0);
    
    lv_obj_t *main_panel = lv_obj_create(screen_home);
    lv_obj_set_size(main_panel, screenWidth, screenHeight - 25);
    lv_obj_align(main_panel, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_border_width(main_panel, 0, 0);

    lv_obj_t *title = lv_label_create(main_panel);
    lv_label_set_text(title, "Bancada IoT de Espaco de Estados");
    lv_obj_set_style_text_font(title, &lv_font_montserrat_16, 0);
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 40);

    lv_obj_t *remote_btn = lv_btn_create(main_panel);
    lv_obj_set_size(remote_btn, 220, 45);
    lv_obj_align(remote_btn, LV_ALIGN_CENTER, 0, -10);
    lv_obj_add_event_cb(remote_btn, main_event_handler, LV_EVENT_CLICKED, (void*)"remote_btn");
    lv_obj_t *label = lv_label_create(remote_btn);
    lv_label_set_text(label, LV_SYMBOL_WIFI " Configurar Modo Remoto");
    lv_obj_center(label);

    lv_obj_t *local_btn = lv_btn_create(main_panel);
    lv_obj_set_size(local_btn, 220, 45);
    lv_obj_align_to(local_btn, remote_btn, LV_ALIGN_OUT_BOTTOM_MID, 0, 10);
    lv_obj_add_event_cb(local_btn, main_event_handler, LV_EVENT_CLICKED, (void*)"local_btn");
    lv_obj_set_style_bg_color(local_btn, lv_palette_main(LV_PALETTE_GREEN), 0);
    label = lv_label_create(local_btn);
    lv_label_set_text(label, LV_SYMBOL_DRIVE " Configurar Modo Local");
    lv_obj_center(label);
}

void create_wifi_config_screen() {
    screen_wifi_config = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(screen_wifi_config, lv_color_white(), 0);
    lv_obj_add_event_cb(screen_wifi_config, background_click_event_handler, LV_EVENT_CLICKED, NULL);

    lv_obj_t* nav_panel = lv_obj_create(screen_wifi_config);
    lv_obj_remove_style_all(nav_panel);
    lv_obj_set_size(nav_panel, screenWidth, 50);
    lv_obj_align(nav_panel, LV_ALIGN_TOP_MID, 0, 25);
    lv_obj_set_style_bg_color(nav_panel, lv_color_white(), 0);

    lv_obj_t* back_btn = lv_btn_create(nav_panel);
    lv_obj_set_size(back_btn, 80, 40);
    lv_obj_align(back_btn, LV_ALIGN_LEFT_MID, 5, 0);
    lv_obj_add_event_cb(back_btn, main_event_handler, LV_EVENT_CLICKED, (void*)"wifi_back_btn");
    lv_obj_t* label = lv_label_create(back_btn);
    lv_label_set_text(label, LV_SYMBOL_LEFT " Inicio");
    lv_obj_center(label);

    lv_obj_t* refresh_btn = lv_btn_create(nav_panel);
    lv_obj_set_size(refresh_btn, 60, 40);
    lv_obj_align(refresh_btn, LV_ALIGN_CENTER, 0, 0);
    lv_obj_add_event_cb(refresh_btn, main_event_handler, LV_EVENT_CLICKED, (void*)"wifi_refresh_btn");
    label = lv_label_create(refresh_btn);
    lv_label_set_text(label, LV_SYMBOL_REFRESH);
    lv_obj_center(label);

    lv_obj_t* next_btn = lv_btn_create(nav_panel);
    lv_obj_set_size(next_btn, 80, 40);
    lv_obj_align(next_btn, LV_ALIGN_RIGHT_MID, -5, 0);
    lv_obj_add_event_cb(next_btn, main_event_handler, LV_EVENT_CLICKED, (void*)"broker_btn");
    label = lv_label_create(next_btn);
    lv_label_set_text(label, "Broker " LV_SYMBOL_RIGHT);
    lv_obj_center(label);

    lv_obj_t* content_panel = lv_obj_create(screen_wifi_config);
    lv_obj_set_size(content_panel, screenWidth, screenHeight - 25 - 50);
    lv_obj_align(content_panel, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_border_width(content_panel, 0, 0);

    wifi_list = lv_list_create(content_panel);
    lv_obj_set_size(wifi_list, lv_pct(95), lv_pct(100));
    lv_obj_center(wifi_list);
}

void create_broker_config_screen() {
    screen_broker_config = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(screen_broker_config, lv_color_white(), 0);
    lv_obj_add_event_cb(screen_broker_config, background_click_event_handler, LV_EVENT_CLICKED, NULL);

    lv_obj_t* nav_panel = lv_obj_create(screen_broker_config);
    lv_obj_remove_style_all(nav_panel);
    lv_obj_set_size(nav_panel, screenWidth, 50);
    lv_obj_align(nav_panel, LV_ALIGN_TOP_MID, 0, 25);
    lv_obj_set_style_bg_color(nav_panel, lv_color_white(), 0);

    lv_obj_t* back_btn = lv_btn_create(nav_panel);
    lv_obj_set_width(back_btn, 120);
    lv_obj_align(back_btn, LV_ALIGN_LEFT_MID, 5, 0);
    lv_obj_add_event_cb(back_btn, main_event_handler, LV_EVENT_CLICKED, (void*)"broker_back_btn");
    lv_obj_t* label = lv_label_create(back_btn);
    lv_label_set_text(label, LV_SYMBOL_LEFT " Voltar WiFi");
    lv_obj_center(label);

    lv_obj_t* next_btn = lv_btn_create(nav_panel);
    lv_obj_set_width(next_btn, 120);
    lv_obj_align(next_btn, LV_ALIGN_RIGHT_MID, -5, 0);
    lv_obj_add_event_cb(next_btn, main_event_handler, LV_EVENT_CLICKED, (void*)"send_broker_btn");
    label = lv_label_create(next_btn);
    lv_label_set_text(label, "Enviar e Avancar");
    lv_obj_center(label);

    lv_obj_t* content_panel = lv_obj_create(screen_broker_config);
    lv_obj_set_size(content_panel, screenWidth, screenHeight - 25 - 50);
    lv_obj_align(content_panel, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_border_width(content_panel, 0, 0);
    lv_obj_set_flex_flow(content_panel, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(content_panel, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_row(content_panel, 10, 0);
    lv_obj_set_style_pad_top(content_panel, 10, 0);

    lv_obj_t* title = lv_label_create(content_panel);
    lv_label_set_text(title, "Configurar Broker");

    broker_ip_ta = lv_textarea_create(content_panel);
    lv_obj_set_width(broker_ip_ta, 280);
    lv_textarea_set_one_line(broker_ip_ta, true);
    lv_textarea_set_placeholder_text(broker_ip_ta, "IP do Broker");
    lv_textarea_set_max_length(broker_ip_ta, MAX_BROKER_IP_LENGTH - 1);
    lv_obj_add_event_cb(broker_ip_ta, textarea_event_handler, LV_EVENT_CLICKED, (void*)"numeric");

    broker_port_ta = lv_textarea_create(content_panel);
    lv_obj_set_width(broker_port_ta, 280);
    lv_textarea_set_one_line(broker_port_ta, true);
    lv_textarea_set_placeholder_text(broker_port_ta, "Porta");
    lv_textarea_set_max_length(broker_port_ta, MAX_BROKER_PORT_LENGTH - 1);
    lv_obj_add_event_cb(broker_port_ta, textarea_event_handler, LV_EVENT_CLICKED, (void*)"numeric");

    lv_obj_t* auth_panel = lv_obj_create(content_panel);
    lv_obj_remove_style_all(auth_panel);
    lv_obj_set_width(auth_panel, 280);
    lv_obj_set_height(auth_panel, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(auth_panel, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(auth_panel, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    lv_obj_t* auth_switch = lv_switch_create(auth_panel);
    lv_obj_add_event_cb(auth_switch, main_event_handler, LV_EVENT_VALUE_CHANGED, (void*)"broker_auth_switch");
    
    label = lv_label_create(auth_panel);
    lv_label_set_text(label, " Requer autenticacao?");
    
    broker_auth_container = lv_obj_create(content_panel);
    lv_obj_remove_style_all(broker_auth_container);
    lv_obj_set_width(broker_auth_container, 280);
    lv_obj_set_height(broker_auth_container, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(broker_auth_container, LV_FLEX_FLOW_COLUMN);
    lv_obj_add_flag(broker_auth_container, LV_OBJ_FLAG_HIDDEN);

    broker_user_ta = lv_textarea_create(broker_auth_container);
    lv_obj_set_width(broker_user_ta, lv_pct(100));
    lv_textarea_set_one_line(broker_user_ta, true);
    lv_textarea_set_placeholder_text(broker_user_ta, "Usuario");
    lv_textarea_set_max_length(broker_user_ta, MAX_BROKER_USER_LENGTH - 1);
    lv_obj_add_event_cb(broker_user_ta, textarea_event_handler, LV_EVENT_CLICKED, NULL);

    broker_password_ta = lv_textarea_create(broker_auth_container);
    lv_obj_set_width(broker_password_ta, lv_pct(100));
    lv_textarea_set_one_line(broker_password_ta, true);
    lv_textarea_set_password_mode(broker_password_ta, true);
    lv_textarea_set_placeholder_text(broker_password_ta, "Senha");
    lv_textarea_set_max_length(broker_password_ta, MAX_BROKER_PW_LENGTH - 1);
    lv_obj_add_event_cb(broker_password_ta, textarea_event_handler, LV_EVENT_CLICKED, NULL);
}

void create_selection_screen() {
    screen_selection = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(screen_selection, lv_color_white(), 0);
    lv_obj_add_event_cb(screen_selection, background_click_event_handler, LV_EVENT_CLICKED, NULL);

    lv_obj_t* main_panel = lv_obj_create(screen_selection);
    lv_obj_set_size(main_panel, screenWidth, screenHeight - 25);
    lv_obj_align(main_panel, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_border_width(main_panel, 0, 0);
    lv_obj_clear_flag(main_panel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_flow(main_panel, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(main_panel, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_all(main_panel, 10, 0);

    lv_obj_t* nav_panel = lv_obj_create(main_panel);
    lv_obj_remove_style_all(nav_panel);
    lv_obj_set_width(nav_panel, lv_pct(100));
    lv_obj_set_height(nav_panel, LV_SIZE_CONTENT);
    lv_obj_t* back_btn = lv_btn_create(nav_panel);
    lv_obj_align(back_btn, LV_ALIGN_LEFT_MID, 0, 0);
    lv_obj_add_event_cb(back_btn, main_event_handler, LV_EVENT_CLICKED, (void*)"selection_back_btn");
    lv_obj_t* label = lv_label_create(back_btn);
    lv_label_set_text(label, LV_SYMBOL_LEFT " Broker");
    lv_obj_center(label);

    lv_obj_t* info_panel = lv_obj_create(main_panel);
    lv_obj_remove_style_all(info_panel);
    lv_obj_set_width(info_panel, lv_pct(95));
    lv_obj_set_height(info_panel, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(info_panel, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(info_panel, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_text_align(info_panel, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_set_style_pad_gap(info_panel, 10, 0);

    selection_broker_status_label = lv_label_create(info_panel);
    lv_label_set_long_mode(selection_broker_status_label, LV_LABEL_LONG_WRAP);
    lv_obj_set_style_text_font(selection_broker_status_label, &lv_font_montserrat_14, 0);

    lv_obj_t* question_label1 = lv_label_create(info_panel);
    lv_label_set_text(question_label1, "Escolha o modo de operacao:");
    lv_label_set_long_mode(question_label1, LV_LABEL_LONG_WRAP);

    lv_obj_t* btn_panel = lv_obj_create(main_panel);
    lv_obj_remove_style_all(btn_panel);
    lv_obj_set_width(btn_panel, lv_pct(100));
    lv_obj_set_height(btn_panel, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(btn_panel, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(btn_panel, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    lv_obj_t* insert_data_btn = lv_btn_create(btn_panel);
    lv_obj_add_event_cb(insert_data_btn, main_event_handler, LV_EVENT_CLICKED, (void*)"goto_topics_btn");
    label = lv_label_create(insert_data_btn);
    lv_label_set_text(label, "Inserir Dados");
    lv_obj_center(label);

    lv_obj_t* experiment_mode_btn = lv_btn_create(btn_panel);
    lv_obj_add_event_cb(experiment_mode_btn, main_event_handler, LV_EVENT_CLICKED, (void*)"goto_experiment_btn");
    label = lv_label_create(experiment_mode_btn);
    lv_label_set_text(label, "Modo Experimento");
    lv_obj_center(label);
}

void create_topics_config_screen() {
    screen_topics_config = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(screen_topics_config, lv_color_white(), 0);
    lv_obj_add_event_cb(screen_topics_config, background_click_event_handler, LV_EVENT_CLICKED, NULL);

    lv_obj_t* nav_panel = lv_obj_create(screen_topics_config);
    lv_obj_remove_style_all(nav_panel);
    lv_obj_set_size(nav_panel, screenWidth, 45);
    lv_obj_align(nav_panel, LV_ALIGN_TOP_MID, 0, 25);
    lv_obj_set_style_bg_color(nav_panel, lv_color_white(), 0);
    
    lv_obj_t* back_btn = lv_btn_create(nav_panel);
    lv_obj_align(back_btn, LV_ALIGN_LEFT_MID, 5, 0);
    lv_obj_add_event_cb(back_btn, main_event_handler, LV_EVENT_CLICKED, (void*)"topics_back_btn");
    lv_obj_t* label = lv_label_create(back_btn);
    lv_label_set_text(label, LV_SYMBOL_LEFT " Voltar");
    lv_obj_center(label);

    lv_obj_t* content_panel = lv_obj_create(screen_topics_config);
    lv_obj_set_size(content_panel, screenWidth, screenHeight - 25 - 45);
    lv_obj_align(content_panel, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_border_width(content_panel, 0, 0);
    lv_obj_set_flex_flow(content_panel, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(content_panel, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_row(content_panel, 10, 0);
    lv_obj_set_style_pad_top(content_panel, 10, 0);


    lv_obj_t* title = lv_label_create(content_panel);
    lv_label_set_text(title, "Configurar Topicos");

    const char* placeholders[] = {"Referencia", "Nu", "Nx", "Regulador K", "Observador Ke"};
    lv_obj_t** text_areas[] = {&topics_ref_ta, &topics_nu_ta, &topics_nx_ta, &topics_k_ta, &topics_ke_ta};

    for (int i = 0; i < 5; i++) {
        lv_obj_t* ta = lv_textarea_create(content_panel);
        lv_obj_set_width(ta, lv_pct(90));
        lv_textarea_set_one_line(ta, true);
        lv_textarea_set_placeholder_text(ta, placeholders[i]);
        lv_textarea_set_max_length(ta, MAX_TOPIC_VALUE_LENGTH - 1);
        lv_obj_add_event_cb(ta, textarea_event_handler, LV_EVENT_CLICKED, (void*)"numeric");
        *text_areas[i] = ta;
    }

    lv_obj_t* send_btn = lv_btn_create(content_panel);
    lv_obj_set_width(send_btn, lv_pct(90));
    lv_obj_add_event_cb(send_btn, main_event_handler, LV_EVENT_CLICKED, (void*)"topics_send_btn");
    label = lv_label_create(send_btn);
    lv_label_set_text(label, "Enviar");
    lv_obj_center(label);
}

void create_experiment_tracking_screen() {
    screen_experiment_tracking = lv_obj_create(NULL);
    lv_obj_set_style_bg_color(screen_experiment_tracking, lv_color_white(), 0);

    lv_obj_t* main_panel = lv_obj_create(screen_experiment_tracking);
    lv_obj_set_size(main_panel, screenWidth, screenHeight - 25);
    lv_obj_align(main_panel, LV_ALIGN_BOTTOM_MID, 0, 0);
    lv_obj_set_style_border_width(main_panel, 0, 0);
    lv_obj_clear_flag(main_panel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_flex_flow(main_panel, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(main_panel, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_all(main_panel, 5, 0);

    lv_obj_t* nav_panel = lv_obj_create(main_panel);
    lv_obj_remove_style_all(nav_panel);
    lv_obj_set_size(nav_panel, lv_pct(100), LV_SIZE_CONTENT);
    lv_obj_t* back_btn = lv_btn_create(nav_panel);
    lv_obj_align(back_btn, LV_ALIGN_LEFT_MID, 0, 0);
    lv_obj_add_event_cb(back_btn, main_event_handler, LV_EVENT_CLICKED, (void*)"experiment_back_btn");
    lv_obj_t* label = lv_label_create(back_btn);
    lv_label_set_text(label, LV_SYMBOL_LEFT " Voltar");
    lv_obj_center(label);

    lv_obj_t* content_panel = lv_obj_create(main_panel);
    lv_obj_remove_style_all(content_panel);
    lv_obj_set_width(content_panel, lv_pct(100));
    lv_obj_set_flex_grow(content_panel, 1);
    lv_obj_set_flex_flow(content_panel, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(content_panel, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_style_pad_gap(content_panel, 5, 0);
    
    lv_obj_t* top_data_panel = lv_obj_create(content_panel);
    lv_obj_remove_style_all(top_data_panel);
    lv_obj_set_width(top_data_panel, lv_pct(100));
    lv_obj_set_height(top_data_panel, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(top_data_panel, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(top_data_panel, LV_FLEX_ALIGN_SPACE_AROUND, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    lv_obj_t* water_level_card = lv_obj_create(top_data_panel);
    lv_obj_set_size(water_level_card, lv_pct(48), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(water_level_card, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(water_level_card, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_t* water_level_title = lv_label_create(water_level_card);
    lv_label_set_text(water_level_title, "Nivel da Agua");
    exp_water_level_label = lv_label_create(water_level_card);
    lv_label_set_text(exp_water_level_label, "0.0 cm");
    lv_obj_set_style_text_font(exp_water_level_label, &lv_font_montserrat_16, 0);

    lv_obj_t* ref_card = lv_obj_create(top_data_panel);
    lv_obj_set_size(ref_card, lv_pct(48), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(ref_card, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(ref_card, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_t* ref_title = lv_label_create(ref_card);
    lv_label_set_text(ref_title, "Referencia");
    exp_reference_label = lv_label_create(ref_card);
    lv_label_set_text(exp_reference_label, "0.0 cm");
    lv_obj_set_style_text_font(exp_reference_label, &lv_font_montserrat_16, 0);

    lv_obj_t* bottom_data_panel = lv_obj_create(content_panel);
    lv_obj_remove_style_all(bottom_data_panel);
    lv_obj_set_width(bottom_data_panel, lv_pct(100));
    lv_obj_set_flex_grow(bottom_data_panel, 1);
    lv_obj_set_flex_flow(bottom_data_panel, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(bottom_data_panel, LV_FLEX_ALIGN_SPACE_AROUND, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    lv_obj_t* status_card = lv_obj_create(bottom_data_panel);
    lv_obj_set_size(status_card, lv_pct(32), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(status_card, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(status_card, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_t* status_title = lv_label_create(status_card);
    lv_label_set_text(status_title, "Status");
    exp_status_label = lv_label_create(status_card);
    lv_label_set_text(exp_status_label, "Pronta");
    lv_obj_set_style_text_font(exp_status_label, &lv_font_montserrat_12, 0);

    lv_obj_t* current_card = lv_obj_create(bottom_data_panel);
    lv_obj_set_size(current_card, lv_pct(32), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(current_card, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(current_card, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_t* current_title = lv_label_create(current_card);
    lv_label_set_text(current_title, "Atual");
    exp_current_label = lv_label_create(current_card);
    lv_label_set_text(exp_current_label, "N/A");
    lv_obj_set_style_text_font(exp_current_label, &lv_font_montserrat_12, 0);

    lv_obj_t* next_card = lv_obj_create(bottom_data_panel);
    lv_obj_set_size(next_card, lv_pct(32), LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(next_card, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(next_card, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_t* next_title = lv_label_create(next_card);
    lv_label_set_text(next_title, "Proximo");
    exp_next_label = lv_label_create(next_card);
    lv_label_set_text(exp_next_label, "N/A");
    lv_obj_set_style_text_font(exp_next_label, &lv_font_montserrat_12, 0);
}

void create_keyboard() {
    keyboard = lv_keyboard_create(lv_layer_top());
    lv_obj_add_flag(keyboard, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_event_cb(keyboard, keyboard_event_handler, LV_EVENT_ALL, NULL);
}

void create_universal_popup() {
    popup_universal = lv_obj_create(lv_layer_top());
    lv_obj_set_size(popup_universal, 280, 220);
    lv_obj_center(popup_universal);
    lv_obj_set_style_bg_color(popup_universal, lv_color_hex(0xE0E0E0), 0);
    lv_obj_set_style_border_width(popup_universal, 2, 0);
    lv_obj_set_style_border_color(popup_universal, lv_palette_main(LV_PALETTE_GREY), 0);
    lv_obj_add_flag(popup_universal, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_flex_flow(popup_universal, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(popup_universal, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    
    popup_title = lv_label_create(popup_universal);
    lv_obj_set_style_text_font(popup_title, &lv_font_montserrat_16, 0);
    lv_obj_set_width(popup_title, lv_pct(100));
    lv_obj_set_style_text_align(popup_title, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(popup_title, "Título");

    // Container para mensagem de texto simples
    popup_message = lv_label_create(popup_universal);
    lv_label_set_long_mode(popup_message, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(popup_message, lv_pct(90));
    lv_obj_set_style_text_align(popup_message, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_add_flag(popup_message, LV_OBJ_FLAG_HIDDEN);

    // Container para inputs de credenciais
    popup_email_container = lv_obj_create(popup_universal);
    lv_obj_remove_style_all(popup_email_container);
    lv_obj_set_width(popup_email_container, lv_pct(90));
    lv_obj_set_height(popup_email_container, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(popup_email_container, LV_FLEX_FLOW_COLUMN);
    lv_obj_add_flag(popup_email_container, LV_OBJ_FLAG_HIDDEN);
    lv_obj_t* email_label = lv_label_create(popup_email_container);
    lv_label_set_text(email_label, "E-mail / Usuario:");
    wifi_email_ta = lv_textarea_create(popup_email_container);
    lv_obj_set_width(wifi_email_ta, lv_pct(100));
    lv_textarea_set_one_line(wifi_email_ta, true);
    lv_textarea_set_max_length(wifi_email_ta, MAX_EMAIL_LENGTH - 1);
    lv_obj_add_event_cb(wifi_email_ta, textarea_event_handler, LV_EVENT_CLICKED, NULL);

    popup_password_container = lv_obj_create(popup_universal);
    lv_obj_remove_style_all(popup_password_container);
    lv_obj_set_width(popup_password_container, lv_pct(90));
    lv_obj_set_height(popup_password_container, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(popup_password_container, LV_FLEX_FLOW_COLUMN);
    lv_obj_add_flag(popup_password_container, LV_OBJ_FLAG_HIDDEN);
    lv_obj_t* pass_label = lv_label_create(popup_password_container);
    lv_label_set_text(pass_label, "Senha:");
    wifi_password_ta = lv_textarea_create(popup_password_container);
    lv_obj_set_width(wifi_password_ta, lv_pct(100));
    lv_textarea_set_one_line(wifi_password_ta, true);
    lv_textarea_set_password_mode(wifi_password_ta, true);
    lv_textarea_set_max_length(wifi_password_ta, MAX_PASSWORD_LENGTH - 1);
    lv_obj_add_event_cb(wifi_password_ta, textarea_event_handler, LV_EVENT_CLICKED, NULL);

    // Container para informações da rede conectada
    popup_info_container = lv_obj_create(popup_universal);
    lv_obj_remove_style_all(popup_info_container);
    lv_obj_set_width(popup_info_container, lv_pct(90));
    lv_obj_set_height(popup_info_container, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(popup_info_container, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_gap(popup_info_container, 5, 0);
    lv_obj_add_flag(popup_info_container, LV_OBJ_FLAG_HIDDEN);
    
    popup_signal_label = lv_label_create(popup_info_container);
    lv_label_set_text(popup_signal_label, "Intensidade: ");

    lv_obj_t* pass_view_label = lv_label_create(popup_info_container);
    lv_label_set_text(pass_view_label, "Senha:");
    lv_obj_t* pass_view_cont = lv_obj_create(popup_info_container);
    lv_obj_remove_style_all(pass_view_cont);
    lv_obj_set_width(pass_view_cont, lv_pct(100));
    lv_obj_set_height(pass_view_cont, LV_SIZE_CONTENT);
    popup_password_view_ta = lv_textarea_create(pass_view_cont);
    lv_obj_set_width(popup_password_view_ta, lv_pct(85));
    lv_textarea_set_one_line(popup_password_view_ta, true);
    lv_textarea_set_password_mode(popup_password_view_ta, true);
    lv_obj_add_state(popup_password_view_ta, LV_STATE_DISABLED);

    lv_obj_t* eye_btn = lv_btn_create(pass_view_cont);
    lv_obj_set_size(eye_btn, lv_pct(15), lv_pct(100));
    lv_obj_align(eye_btn, LV_ALIGN_RIGHT_MID, 0, 0);
    lv_obj_add_event_cb(eye_btn, main_event_handler, LV_EVENT_CLICKED, (void*)"toggle_password_visibility");
    lv_obj_t* eye_label = lv_label_create(eye_btn);
    lv_label_set_text(eye_label, LV_SYMBOL_EYE_OPEN);
    lv_obj_center(eye_label);


    // Container para os botões do popup
    popup_btn_container = lv_obj_create(popup_universal);
    lv_obj_remove_style_all(popup_btn_container);
    lv_obj_set_size(popup_btn_container, lv_pct(100), LV_SIZE_CONTENT);
    lv_obj_set_style_pad_top(popup_btn_container, 20, 0);
    lv_obj_set_flex_flow(popup_btn_container, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(popup_btn_container, LV_FLEX_ALIGN_SPACE_EVENLY, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
}

// ==========================================================
// == LÓGICA DA APLICAÇÃO (Eventos e Tarefas)
// ==========================================================

void update_bench_status_icon(Bench_Status_t status) {
    benchStatus = status;
    switch (benchStatus) {
        case BENCH_READY:
            lv_label_set_text(status_bar_bench_status_icon, LV_SYMBOL_OK);
            lv_obj_set_style_text_color(status_bar_bench_status_icon, lv_palette_main(LV_PALETTE_GREEN), 0);
            break;
        case BENCH_NOT_READY:
            lv_label_set_text(status_bar_bench_status_icon, LV_SYMBOL_CLOSE);
            lv_obj_set_style_text_color(status_bar_bench_status_icon, lv_palette_main(LV_PALETTE_RED), 0);
            break;
        case BENCH_RUNNING:
            lv_label_set_text(status_bar_bench_status_icon, LV_SYMBOL_REFRESH);
            lv_obj_set_style_text_color(status_bar_bench_status_icon, lv_palette_main(LV_PALETTE_BLUE), 0);
            break;
    }
}

void update_communication_icons(bool sending, bool receiving) {
    is_sending = sending;
    is_receiving = receiving;
    if (is_sending) {
        lv_obj_clear_flag(status_bar_upload_icon, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(status_bar_upload_icon, LV_OBJ_FLAG_HIDDEN);
    }

    if (is_receiving) {
        lv_obj_clear_flag(status_bar_download_icon, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(status_bar_download_icon, LV_OBJ_FLAG_HIDDEN);
    }
}

void update_broker_status_label() {
    switch(mqttStatus) {
        case MQTT_CONNECTED:
            lv_label_set_text(selection_broker_status_label, "Bancada online e conectada ao broker.");
            lv_obj_set_style_text_color(selection_broker_status_label, lv_palette_main(LV_PALETTE_GREEN), 0);
            break;
        case MQTT_CONNECTING:
            lv_label_set_text(selection_broker_status_label, "Conectando ao broker...");
            lv_obj_set_style_text_color(selection_broker_status_label, lv_palette_main(LV_PALETTE_BLUE), 0);
            break;
        case MQTT_FAILED:
            lv_label_set_text(selection_broker_status_label, "Falha ao conectar ao broker.\nVerifique os dados e tente novamente.");
            lv_obj_set_style_text_color(selection_broker_status_label, lv_palette_main(LV_PALETTE_RED), 0);
            break;
        case MQTT_DISCONNECTED:
        default:
            lv_label_set_text(selection_broker_status_label, "Bancada desconectada.\nEnvie os dados para conectar.");
            lv_obj_set_style_text_color(selection_broker_status_label, lv_palette_main(LV_PALETTE_GREY), 0);
            break;
    }
}

void show_message_popup(const char* title, const char* msg, const char* btn1_txt, const char* btn2_txt) {
    lv_obj_add_flag(popup_email_container, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(popup_password_container, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(popup_info_container, LV_OBJ_FLAG_HIDDEN);
    
    lv_obj_clear_flag(popup_message, LV_OBJ_FLAG_HIDDEN);
    lv_label_set_text(popup_message, msg);
    lv_label_set_text(popup_title, title);
    
    lv_obj_clean(popup_btn_container);
    if (btn1_txt) {
        lv_obj_t* btn1 = lv_btn_create(popup_btn_container);
        lv_obj_add_event_cb(btn1, main_event_handler, LV_EVENT_CLICKED, (void*)btn1_txt);
        lv_obj_t* label1 = lv_label_create(btn1);
        lv_label_set_text(label1, btn1_txt);
    }
    if (btn2_txt) {
        lv_obj_t* btn2 = lv_btn_create(popup_btn_container);
        lv_obj_add_event_cb(btn2, main_event_handler, LV_EVENT_CLICKED, (void*)btn2_txt);
        lv_obj_t* label2 = lv_label_create(btn2);
        lv_label_set_text(label2, btn2_txt);
    }
    if (keyboard != NULL && !lv_obj_has_flag(keyboard, LV_OBJ_FLAG_HIDDEN)) {
        lv_event_send(keyboard, LV_EVENT_CANCEL, NULL);
    }
    lv_obj_clear_flag(popup_universal, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(popup_universal);
}

void show_credentials_popup() {
    lv_obj_add_flag(popup_message, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(popup_info_container, LV_OBJ_FLAG_HIDDEN);
    
    lv_label_set_text(popup_title, ssidName);
    lv_obj_clear_flag(popup_password_container, LV_OBJ_FLAG_HIDDEN);
    if (is_corporate_network) {
        lv_obj_clear_flag(popup_email_container, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(popup_email_container, LV_OBJ_FLAG_HIDDEN);
    }
    lv_textarea_set_text(wifi_password_ta, "");
    lv_textarea_set_text(wifi_email_ta, "");
    
    lv_obj_clean(popup_btn_container);
    lv_obj_t* cancel_btn = lv_btn_create(popup_btn_container);
    lv_obj_add_event_cb(cancel_btn, main_event_handler, LV_EVENT_CLICKED, (void*)"Cancelar");
    lv_obj_t* label_cancel = lv_label_create(cancel_btn);
    lv_label_set_text(label_cancel, "Cancelar");
    lv_obj_t* connect_btn = lv_btn_create(popup_btn_container);
    lv_obj_add_event_cb(connect_btn, main_event_handler, LV_EVENT_CLICKED, (void*)"Conectar");
    lv_obj_t* label_connect = lv_label_create(connect_btn);
    lv_label_set_text(label_connect, "Conectar");
    
    lv_obj_clear_flag(popup_universal, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(popup_universal);
}

void show_connected_wifi_popup() {
    lv_obj_add_flag(popup_message, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(popup_email_container, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(popup_password_container, LV_OBJ_FLAG_HIDDEN);

    lv_obj_clear_flag(popup_info_container, LV_OBJ_FLAG_HIDDEN);
    lv_label_set_text(popup_title, WiFi.SSID().c_str());

    long rssi = WiFi.RSSI();
    char signal_str[50];
    const char* strength;
    if (rssi > -67) strength = "Excelente";
    else if (rssi > -70) strength = "Boa";
    else if (rssi > -80) strength = "OK";
    else strength = "Fraca";
    sprintf(signal_str, "Intensidade: %s (%ld dBm)", strength, rssi);
    lv_label_set_text(popup_signal_label, signal_str);

    lv_textarea_set_text(popup_password_view_ta, ssidPW);

    lv_obj_clean(popup_btn_container);
    lv_obj_t* disconnect_btn = lv_btn_create(popup_btn_container);
    lv_obj_set_style_bg_color(disconnect_btn, lv_palette_main(LV_PALETTE_RED), 0);
    lv_obj_add_event_cb(disconnect_btn, main_event_handler, LV_EVENT_CLICKED, (void*)"wifi_disconnect_btn");
    lv_obj_t* label_disconnect = lv_label_create(disconnect_btn);
    lv_label_set_text(label_disconnect, "Desconectar");

    lv_obj_t* ok_btn = lv_btn_create(popup_btn_container);
    lv_obj_add_event_cb(ok_btn, main_event_handler, LV_EVENT_CLICKED, (void*)"Ok");
    lv_obj_t* label_ok = lv_label_create(ok_btn);
    lv_label_set_text(label_ok, "Ok");

    lv_obj_clear_flag(popup_universal, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(popup_universal);
}

void main_event_handler(lv_event_t *e) {
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *obj = lv_event_get_target(e);
    const char* user_data = (const char*)lv_event_get_user_data(e);

    if (code == LV_EVENT_CLICKED) {
        // --- Navegação entre telas ---
        if (strcmp(user_data, "remote_btn") == 0) {
            lv_scr_load_anim(screen_wifi_config, LV_SCR_LOAD_ANIM_MOVE_LEFT, 300, 0, false);
            startWifiScan();
        } else if (strcmp(user_data, "local_btn") == 0) {
            came_from_local_mode = true;
            lv_scr_load_anim(screen_experiment_tracking, LV_SCR_LOAD_ANIM_MOVE_LEFT, 300, 0, false);
        } else if (strcmp(user_data, "wifi_back_btn") == 0) {
            lv_scr_load_anim(screen_home, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 300, 0, false);
        } else if (strcmp(user_data, "broker_btn") == 0) {
            lv_scr_load_anim(screen_broker_config, LV_SCR_LOAD_ANIM_MOVE_LEFT, 300, 0, false);
        } else if (strcmp(user_data, "broker_back_btn") == 0) {
            lv_scr_load_anim(screen_wifi_config, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 300, 0, false);
        } else if (strcmp(user_data, "send_broker_btn") == 0) {
            if (strlen(lv_textarea_get_text(broker_ip_ta)) == 0 || strlen(lv_textarea_get_text(broker_port_ta)) == 0) {
                show_message_popup("Erro", "IP do Broker e Porta sao obrigatorios.", "Ok", NULL);
                return;
            }
            bool use_auth = lv_obj_has_state(lv_obj_get_child(broker_auth_container, 0), LV_STATE_CHECKED);
            if (use_auth && (strlen(lv_textarea_get_text(broker_user_ta)) == 0 || strlen(lv_textarea_get_text(broker_password_ta)) == 0)) {
                show_message_popup("Erro", "Usuario e Senha sao obrigatorios para autenticacao.", "Ok", NULL);
                return;
            }

            struct_broker_credentials broker_creds;
            broker_creds.messageType = BROKER_CREDENTIALS;
            strncpy(broker_creds.ip, lv_textarea_get_text(broker_ip_ta), sizeof(broker_creds.ip) - 1);
            strncpy(broker_creds.port, lv_textarea_get_text(broker_port_ta), sizeof(broker_creds.port) - 1);
            broker_creds.use_auth = use_auth;
            if (use_auth) {
                strncpy(broker_creds.user, lv_textarea_get_text(broker_user_ta), sizeof(broker_creds.user) - 1);
                strncpy(broker_creds.pass, lv_textarea_get_text(broker_password_ta), sizeof(broker_creds.pass) - 1);
            } else {
                broker_creds.user[0] = '\0';
                broker_creds.pass[0] = '\0';
            }
            esp_now_send(broadcastAddress, (uint8_t *) &broker_creds, sizeof(broker_creds));
            mqttStatus = MQTT_CONNECTING;
            update_broker_status_label();
            lv_scr_load_anim(screen_selection, LV_SCR_LOAD_ANIM_MOVE_LEFT, 300, 0, false);

        } else if (strcmp(user_data, "selection_back_btn") == 0) {
            lv_scr_load_anim(screen_broker_config, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 300, 0, false);
        } else if (strcmp(user_data, "goto_topics_btn") == 0) {
            lv_scr_load_anim(screen_topics_config, LV_SCR_LOAD_ANIM_MOVE_LEFT, 300, 0, false);
        } else if (strcmp(user_data, "goto_experiment_btn") == 0) {
            came_from_local_mode = false;
            lv_scr_load_anim(screen_experiment_tracking, LV_SCR_LOAD_ANIM_MOVE_LEFT, 300, 0, false);
        } else if (strcmp(user_data, "topics_back_btn") == 0) {
            lv_scr_load_anim(screen_selection, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 300, 0, false);
        } else if (strcmp(user_data, "experiment_back_btn") == 0) {
            if (came_from_local_mode) {
                lv_scr_load_anim(screen_home, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 300, 0, false);
            } else {
                lv_scr_load_anim(screen_selection, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 300, 0, false);
            }
        } else if (strcmp(user_data, "topics_send_btn") == 0) {
            struct_control_parameters params_to_send;
            params_to_send.messageType = CONTROL_PARAMETERS;
            params_to_send.ref = atof(lv_textarea_get_text(topics_ref_ta));
            params_to_send.nu = atof(lv_textarea_get_text(topics_nu_ta));
            params_to_send.nx = atof(lv_textarea_get_text(topics_nx_ta));
            params_to_send.k = atof(lv_textarea_get_text(topics_k_ta));
            params_to_send.ke = atof(lv_textarea_get_text(topics_ke_ta));
            
            update_communication_icons(true, is_receiving);
            esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &params_to_send, sizeof(params_to_send));
            if (result == ESP_OK) {
                show_temporary_popup("Sucesso", "Dados enviados para a bancada.");
            } else {
                show_message_popup("Erro", "Falha ao enviar dados para a bancada.", "Ok", NULL);
                update_communication_icons(false, is_receiving);
            }

        } else if (strcmp(user_data, "wifi_refresh_btn") == 0) {
            startWifiScan();
        }
        
        // --- Lógica do Pop-up Universal ---
        else if (strcmp(user_data, "Conectar") == 0) {
            strncpy(ssidPW, lv_textarea_get_text(wifi_password_ta), MAX_PASSWORD_LENGTH - 1);
            ssidPW[MAX_PASSWORD_LENGTH - 1] = '\0';
            if (is_corporate_network) {
                strncpy(email_user, lv_textarea_get_text(wifi_email_ta), MAX_EMAIL_LENGTH - 1);
                email_user[MAX_EMAIL_LENGTH - 1] = '\0';
                if (strlen(email_user) == 0 || strlen(ssidPW) == 0) {
                    show_message_popup("Atencao", "Preencha e-mail e senha.", "Tentar Novamente", "Cancelar");
                    return;
                }
            } else {
                if (selected_auth_mode != WIFI_AUTH_OPEN && strlen(ssidPW) == 0) {
                    show_message_popup("Atencao", "Insira a senha da rede.", "Tentar Novamente", "Cancelar");
                    return;
                }
            }
            networkStatus = NETWORK_CONNECTING;
            show_message_popup("Conectando...", "Aguarde...", NULL, NULL);
            xTaskCreate(networkConnectorTask, "ConnectorTask", 8192, NULL, 2, NULL);
        } else if (strcmp(user_data, "Cancelar") == 0) {
            lv_obj_add_flag(popup_universal, LV_OBJ_FLAG_HIDDEN);
        } else if (strcmp(user_data, "Ok") == 0) {
            lv_obj_add_flag(popup_universal, LV_OBJ_FLAG_HIDDEN);
        } else if (strcmp(user_data, "Tentar Novamente") == 0) {
            show_credentials_popup();
        } else if (strcmp(user_data, "wifi_disconnect_btn") == 0) {
            WiFi.disconnect(true);
            networkStatus = NONE;
            lv_label_set_text(wifi_status_label, LV_SYMBOL_WIFI " /");
            lv_obj_set_style_text_color(wifi_status_label, lv_palette_main(LV_PALETTE_GREY), 0);
            lv_obj_add_flag(popup_universal, LV_OBJ_FLAG_HIDDEN);
            startWifiScan(); // Refresh the list to show it's disconnected
        } else if (strcmp(user_data, "toggle_password_visibility") == 0) {
            bool is_hidden = lv_textarea_get_password_mode(popup_password_view_ta);
            lv_textarea_set_password_mode(popup_password_view_ta, !is_hidden);
            lv_obj_t* eye_label = lv_obj_get_child(obj, 0);
            lv_label_set_text(eye_label, is_hidden ? LV_SYMBOL_EYE_CLOSE : LV_SYMBOL_EYE_OPEN);
        }

    } else if (code == LV_EVENT_VALUE_CHANGED) {
        if (strcmp(user_data, "broker_auth_switch") == 0) {
            bool use_auth = lv_obj_has_state(obj, LV_STATE_CHECKED);
            if (use_auth) {
                lv_obj_clear_flag(broker_auth_container, LV_OBJ_FLAG_HIDDEN);
            } else {
                lv_obj_add_flag(broker_auth_container, LV_OBJ_FLAG_HIDDEN);
            }
        }
    }
}

void wifi_list_event_handler(lv_event_t* e) {
    lv_obj_t* btn = lv_event_get_target(e);
    if (!btn) return;
    const char* txt = lv_list_get_btn_text(wifi_list, btn);
    
    // Check if the clicked network is the currently connected one
    if (networkStatus == NETWORK_CONNECTED && strstr(txt, LV_SYMBOL_OK)) {
        show_connected_wifi_popup();
    } else {
        uint32_t idx = lv_obj_get_index(btn);
        if (xSemaphoreTake(wifi_data_mutex, (TickType_t)10) == pdTRUE) {
            if (idx < foundNetworksCount) {
                strncpy(ssidName, foundWifiList[idx], MAX_SSID_LENGTH - 1);
                ssidName[MAX_SSID_LENGTH - 1] = '\0';
                selected_auth_mode = foundWifiAuth[idx];
                is_corporate_network = (selected_auth_mode == WIFI_AUTH_WPA2_ENTERPRISE);
            }
            xSemaphoreGive(wifi_data_mutex);
        }
        show_credentials_popup();
    }
}

void textarea_event_handler(lv_event_t *e) {
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t *ta = lv_event_get_target(e);
    const char* user_data = (const char*)lv_event_get_user_data(e);
    if (code == LV_EVENT_CLICKED) {
        if (keyboard != NULL) {
            if (user_data && strcmp(user_data, "numeric") == 0) {
                lv_keyboard_set_mode(keyboard, LV_KEYBOARD_MODE_NUMBER);
            } else {
                lv_keyboard_set_mode(keyboard, LV_KEYBOARD_MODE_TEXT_LOWER);
            }
            lv_keyboard_set_textarea(keyboard, ta);
            lv_obj_clear_flag(keyboard, LV_OBJ_FLAG_HIDDEN);
            lv_obj_move_foreground(keyboard);
        }
    }
}

void keyboard_event_handler(lv_event_t *e) {
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t* kb = lv_event_get_target(e);
    if (code == LV_EVENT_READY || code == LV_EVENT_CANCEL) {
        lv_obj_t* ta = lv_keyboard_get_textarea(kb);
        if (ta != NULL) {
            lv_obj_clear_state(ta, LV_STATE_FOCUSED);
        }
        lv_obj_add_flag(kb, LV_OBJ_FLAG_HIDDEN);
        lv_keyboard_set_textarea(kb, NULL);
    }
}

void background_click_event_handler(lv_event_t *e) {
    lv_event_code_t code = lv_event_get_code(e);
    lv_obj_t* target = lv_event_get_target(e);
    lv_obj_t* current_target = lv_event_get_current_target(e);
    if (target == current_target) {
        if (keyboard != NULL && !lv_obj_has_flag(keyboard, LV_OBJ_FLAG_HIDDEN)) {
            lv_event_send(keyboard, LV_EVENT_CANCEL, NULL);
        }
    }
}

void network_timer_cb(lv_timer_t *timer) {
    if(networkStatus == NETWORK_CONNECTING) return;
    if(networkStatus == NETWORK_CONNECTED_POPUP) {
        show_message_popup("Sucesso!", "Conectado a rede Wi-Fi.", "Ok", NULL);
        networkStatus = NETWORK_CONNECTED;
        
        struct_wifi_credentials creds;
        creds.messageType = WIFI_CREDENTIALS;
        strncpy(creds.ssid, ssidName, sizeof(creds.ssid) - 1);
        creds.ssid[sizeof(creds.ssid)-1] = '\0';
        strncpy(creds.pass, ssidPW, sizeof(creds.pass) - 1);
        creds.pass[sizeof(creds.pass)-1] = '\0';
        strncpy(creds.email, email_user, sizeof(creds.email) - 1);
        creds.email[sizeof(creds.email)-1] = '\0';
        creds.is_corp = is_corporate_network;
        esp_now_send(broadcastAddress, (uint8_t *) &creds, sizeof(creds));
        startWifiScan(); // Refresh list to show connected status
    } else if (networkStatus == NETWORK_CONNECT_FAILED_POPUP) {
        show_message_popup("Ops!", "Nao foi possivel conectar. Verifique os dados e tente novamente.", "Tentar Novamente", "Cancelar");
        networkStatus = NONE;
        lv_label_set_text(wifi_status_label, LV_SYMBOL_WIFI " /");
        lv_obj_set_style_text_color(wifi_status_label, lv_palette_main(LV_PALETTE_GREY), 0);
    }
    if (networkStatus == NETWORK_CONNECTED) {
        updateLocalTime();
    }
    if (xSemaphoreTake(wifi_data_mutex, (TickType_t)10) == pdTRUE) {
        if (displayedNetworksCount != foundNetworksCount) {
            lv_obj_clean(wifi_list);
            for (int i = 0; i < foundNetworksCount; i++) {
                char item_text[MAX_SSID_LENGTH + 10];
                const char* icon = LV_SYMBOL_WIFI;
                if (foundWifiAuth[i] == WIFI_AUTH_WPA2_ENTERPRISE) {
                    icon = LV_SYMBOL_SETTINGS;
                }

                if (networkStatus == NETWORK_CONNECTED && strcmp(foundWifiList[i], WiFi.SSID().c_str()) == 0) {
                    snprintf(item_text, sizeof(item_text), "%s %s %s", LV_SYMBOL_OK, icon, foundWifiList[i]);
                } else {
                    snprintf(item_text, sizeof(item_text), "%s %s", icon, foundWifiList[i]);
                }
                
                lv_obj_t *list_btn = lv_list_add_btn(wifi_list, NULL, item_text);
                lv_obj_add_event_cb(list_btn, wifi_list_event_handler, LV_EVENT_CLICKED, NULL);
            }
            displayedNetworksCount = foundNetworksCount;
        }
        xSemaphoreGive(wifi_data_mutex);
    }
}

void startWifiScan() {
    lv_obj_clean(wifi_list);
    lv_list_add_text(wifi_list, "Procurando redes...");
    if (xSemaphoreTake(wifi_data_mutex, (TickType_t)10) == pdTRUE) {
        foundNetworksCount = 0;
        displayedNetworksCount = -1;
        xSemaphoreGive(wifi_data_mutex);
    }
    networkStatus = NETWORK_SEARCHING;
    if(network_timer != NULL) lv_timer_del(network_timer);
    network_timer = lv_timer_create(network_timer_cb, 500, NULL);
    xTaskCreate(networkScannerTask, "ScannerTask", 4096, NULL, 1, &ntScanTaskHandler);
}

void networkScannerTask(void *pvParameters) {
    WiFi.mode(WIFI_STA);
    // Don't disconnect here if already connected, as it breaks the flow
    // WiFi.disconnect(); 
    delay(100);
    int n = WiFi.scanNetworks();
    if (xSemaphoreTake(wifi_data_mutex, portMAX_DELAY) == pdTRUE) {
        foundNetworksCount = 0;
        for (int i = 0; i < n && foundNetworksCount < MAX_WIFI_NETWORKS; ++i) {
            if(WiFi.SSID(i).length() > 0) {
                strncpy(foundWifiList[foundNetworksCount], WiFi.SSID(i).c_str(), MAX_SSID_LENGTH - 1);
                foundWifiList[foundNetworksCount][MAX_SSID_LENGTH - 1] = '\0';
                foundWifiAuth[foundNetworksCount] = WiFi.encryptionType(i);
                foundNetworksCount++;
            }
        }
        xSemaphoreGive(wifi_data_mutex);
    }
    ntScanTaskHandler = NULL;
    vTaskDelete(NULL);
}

void networkConnectorTask(void *pvParameters) {
    WiFi.disconnect(true);
    delay(100);

    if (is_corporate_network) {
        Serial.println("Configurando WPA2-Enterprise...");
        esp_eap_client_set_identity((uint8_t *)email_user, strlen(email_user));
        esp_eap_client_set_username((uint8_t *)email_user, strlen(email_user));
        esp_eap_client_set_password((uint8_t *)ssidPW, strlen(ssidPW));
        esp_wifi_sta_enterprise_enable();
        WiFi.begin(ssidName);
    } else {
        WiFi.begin(ssidName, ssidPW);
    }

    Serial.printf("Tentando conectar a %s...\n", ssidName);
    unsigned long startingTime = millis();
    while (WiFi.status() != WL_CONNECTED && (millis() - startingTime) < WIFI_CONNECT_TIMEOUT_MS) {
        delay(500);
        Serial.print(".");
    }

    if (WiFi.status() == WL_CONNECTED) {
        Serial.println("\nConectado!");

        esp_now_deinit();
        if (esp_now_init() != ESP_OK) {
            Serial.println("Falha ao re-inicializar o ESP-NOW!");
            networkStatus = NETWORK_CONNECT_FAILED_POPUP;
        } else {
            esp_now_register_send_cb(OnDataSent);
            esp_now_register_recv_cb(OnDataRecv);
            if (esp_now_add_peer(&peerInfo) != ESP_OK){
                Serial.println("Falha ao re-adicionar o par ESP-NOW");
                networkStatus = NETWORK_CONNECT_FAILED_POPUP;
            } else {
                 networkStatus = NETWORK_CONNECTED_POPUP;
                 configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
            }
        }
    } else {
        Serial.println("\nFalha na conexao.");
        networkStatus = NETWORK_CONNECT_FAILED_POPUP;
        WiFi.disconnect(true);
    }

    if (is_corporate_network) {
        esp_wifi_sta_enterprise_disable();
    }
    vTaskDelete(NULL);
}

void updateLocalTime() {
    struct tm timeinfo;
    if (!getLocalTime(&timeinfo)) {
        lv_label_set_text(wifi_status_label, LV_SYMBOL_WIFI);
        lv_obj_set_style_text_color(wifi_status_label, lv_color_black(), 0);
        return;
    }
    char time_str[16];
    char time_buffer[6];
    strftime(time_buffer, 6, "%H:%M", &timeinfo);
    sprintf(time_str, "%s %s", time_buffer, LV_SYMBOL_WIFI);
    lv_label_set_text(wifi_status_label, time_str);
    lv_obj_set_style_text_color(wifi_status_label, lv_color_black(), 0);
}
