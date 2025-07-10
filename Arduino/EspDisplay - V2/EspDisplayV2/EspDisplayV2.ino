/**
 * @file esp_display_ui_final.ino
 * @author Seu Nome
 * @brief VERSÃO FINAL CORRIGIDA - Base do usuário + Lógica de captura com os nomes de botões atualizados.
 * @version 18.0
 * * ESTA VERSÃO UTILIZA O CÓDIGO-BASE FUNCIONAL FORNECIDO PELO USUÁRIO E REALIZA AS SEGUINTES AÇÕES:
 * - Corrige os nomes das variáveis dos botões nos registros de evento para corresponder à nova UI.
 * - Adiciona as callbacks e registros de evento para os botões 'botaoEncheReservatorio' e 'botaoEsvaziaReservatorio'.
 */

// ------------------- BIBLIOTECAS -------------------
#include <lvgl.h>
#include <TFT_eSPI.h>
#include <XPT2046_Touchscreen.h>

// ------------------- ARQUIVOS DA UI (com wrapper de segurança) -------------------
extern "C" {
  #include "ui.h"
}

// ==========================================================
// == VARIÁVEIS GLOBAIS PARA ARMAZENAR OS DADOS DA UI    ====
// ==========================================================
char wifi_ssid[32];
char wifi_password[64];
char wifi_user_email[64];
char broker_ip_remoto[16];
int  broker_port_remoto = 0;
char broker_ip_local[16];
int  broker_port_local = 0;
char ke_value[20], k_value[20], n_estados_value[5], n_entradas_value[5], referencia_value[10];

// ------------------- CONFIGURAÇÃO DO HARDWARE (RNT) -------------------
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

// ------------------- CONFIGURAÇÃO DO LVGL -------------------
static const uint16_t screenWidth = 320;
static const uint16_t screenHeight = 240;
static lv_disp_draw_buf_t disp_draw_buf;
static lv_color_t buf[screenWidth * screenHeight / 10];


// =================================================================================
// == FUNÇÕES DE CALLBACK (AÇÕES DOS BOTÕES) =======================================
// =================================================================================

// --- Callback para a tela "Wi-Fi Caseira" ---
void botaoConectaWifi1_cb(lv_event_t * e) {
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        Serial.println("\n==> Botão 'botaoConectaWifi1' (Wi-Fi Caseiro) pressionado!");
        strcpy(wifi_ssid, lv_textarea_get_text(ui_inputWifi));
        strcpy(wifi_password, lv_textarea_get_text(ui_inputSenhaC));
        Serial.printf("SSID (Caseiro) capturado: %s\n", wifi_ssid);
        Serial.printf("Senha (Caseira) capturada: [OCULTA]\n");
    }
}

// --- Callback para a tela "Wi-Fi Estudante" ---
void botaoConectaWifi2_cb(lv_event_t * e) {
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        Serial.println("\n==> Botão 'botaoConectaWifi2' (Wi-Fi Estudante) pressionado!");
        strcpy(wifi_ssid, lv_textarea_get_text(ui_inputWifiEstudante));
        strcpy(wifi_user_email, lv_textarea_get_text(ui_inputEmailEstudante));
        strcpy(wifi_password, lv_textarea_get_text(ui_inputsenhaEstudante));
        Serial.printf("SSID (Estudante) capturado: %s\n", wifi_ssid);
        Serial.printf("E-mail (Estudante) capturado: %s\n", wifi_user_email);
        Serial.printf("Senha (Estudante) capturada: [OCULTA]\n");
    }
}

// --- Callback para a tela "Broker Remoto" ---
void botaoConectaBroker2_cb(lv_event_t * e) {
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        Serial.println("\n==> Botão 'botaoConectaBroker2' (Remoto) pressionado!");
        strcpy(broker_ip_remoto, lv_textarea_get_text(ui_inputIP));
        broker_port_remoto = atoi(lv_textarea_get_text(ui_inputPorta));
        Serial.printf("IP do Broker Remoto capturado: %s\n", broker_ip_remoto);
        Serial.printf("Porta do Broker Remoto capturada: %d\n", broker_port_remoto);
    }
}

// --- Callback para a tela "Broker Local" ---
void botaoConectaBroker1_cb(lv_event_t * e) {
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        Serial.println("\n==> Botão 'botaoConectaBroker1' (Local) pressionado!");
        strcpy(broker_ip_local, lv_textarea_get_text(ui_inputIP1));
        broker_port_local = atoi(lv_textarea_get_text(ui_inputPorta1));
        Serial.printf("IP do Broker Local capturado: %s\n", broker_ip_local);
        Serial.printf("Porta do Broker Local capturada: %d\n", broker_port_local);
    }
}

// --- Callback para a tela "Parâmetros do Experimento" ---
void botaoEnviaDados_cb(lv_event_t * e) {
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        Serial.println("\n==> Botão 'botaoEnviaDados' pressionado!");
        strcpy(ke_value, lv_textarea_get_text(ui_inputObserKe));
        strcpy(k_value, lv_textarea_get_text(ui_inputRegulaK));
        strcpy(n_estados_value, lv_textarea_get_text(ui_inputNEstados));
        strcpy(n_entradas_value, lv_textarea_get_text(ui_inputNEntradas));
        strcpy(referencia_value, lv_textarea_get_text(ui_inputReferencia));
        Serial.printf("Ke: %s, K: %s, N Estados: %s, N Entradas: %s, Ref: %s\n",
                      ke_value, k_value, n_estados_value, n_entradas_value, referencia_value);
    }
}

// --- Callback para os botões da tela Reservatorios ---
void botaoEncheReservatorio_cb(lv_event_t * e) {
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        Serial.println("\n==> Botão 'Encher Reservatório' pressionado!");
    }
}

void botaoEsvaziaReservatorio_cb(lv_event_t * e) {
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        Serial.println("\n==> Botão 'Esvaziar Reservatório' pressionado!");
    }
}


// --- PONTES DE COMUNICAÇÃO LVGL <-> HARDWARE ---
void my_disp_flush(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p) {
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);
    tft.startWrite();
    tft.setAddrWindow(area->x1, area->y1, w, h);
    tft.pushImage(area->x1, area->y1, w, h, (uint16_t *)&color_p->full);
    tft.endWrite();
    lv_disp_flush_ready(disp_drv);
}

void my_touchpad_read(lv_indev_drv_t *indev_drv, lv_indev_data_t *data) {
    if (touchscreen.tirqTouched() && touchscreen.touched()) {
        TS_Point p = touchscreen.getPoint();
        data->point.x = map(p.x, TOUCH_X_MIN, TOUCH_X_MAX, 0, screenWidth);
        data->point.y = map(p.y, TOUCH_Y_MIN, TOUCH_Y_MAX, 0, screenHeight);
        data->state = LV_INDEV_STATE_PR;
    } else {
        data->state = LV_INDEV_STATE_REL;
    }
}


// ------------------- FUNÇÃO SETUP -------------------
void setup() {
    Serial.begin(115200);
    Serial.println("Iniciando o projeto de UI - Versão Final com Nomes Atualizados");

    lv_init();

    touchscreenSPI.begin(XPT2046_CLK, XPT2046_MISO, XPT2046_MOSI, XPT2046_CS);
    touchscreen.begin(touchscreenSPI);
    touchscreen.setRotation(1);

    tft.begin();
    tft.setRotation(1);
    tft.setSwapBytes(true);

    lv_disp_draw_buf_init(&disp_draw_buf, buf, NULL, screenWidth * screenHeight / 10);

    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = screenWidth;
    disp_drv.ver_res = screenHeight;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &disp_draw_buf;
    lv_disp_drv_register(&disp_drv);

    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touchpad_read;
    lv_indev_drv_register(&indev_drv);

    ui_init();

    // =======================================================================
    // == ANEXA TODAS AS FUNÇÕES DE CAPTURA AOS BOTÕES COM NOMES ATUALIZADOS ===
    // =======================================================================
    // --- Captura de dados dos formulários ---
    lv_obj_add_event_cb(ui_botaoConectaWifi1, botaoConectaWifi1_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(ui_botaoConectaWifi2, botaoConectaWifi2_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(ui_botaoConectaBroker2, botaoConectaBroker2_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(ui_botaoConectaBroker1, botaoConectaBroker1_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(ui_botaoEnviaDados, botaoEnviaDados_cb, LV_EVENT_CLICKED, NULL);

    // --- Captura dos botões de ação da tela Reservatórios ---
    lv_obj_add_event_cb(ui_botaoEncheReservatorio, botaoEncheReservatorio_cb, LV_EVENT_CLICKED, NULL);
    lv_obj_add_event_cb(ui_botaoEsvaziaReservatorio, botaoEsvaziaReservatorio_cb, LV_EVENT_CLICKED, NULL);

    Serial.println("Setup completo. UI deve estar visível e funcional.");
}

// ------------------- FUNÇÃO LOOP -------------------
void loop() {
    lv_timer_handler(); 
    delay(5); 
}