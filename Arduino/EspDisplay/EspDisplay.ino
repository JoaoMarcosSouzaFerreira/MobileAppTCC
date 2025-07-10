/**
 * @file esp_display_ui.ino
 * @author Seu Nome
 * @brief CÓDIGO CORRIGIDO E REVISADO para carregar a UI do SquareLine no ESP32-2432S028R.
 * @version 3.0
 * * ALTERAÇÕES DESTA VERSÃO:
 * - CORREÇÃO DA INTERAÇÃO: Adicionada a função de calibração de toque (map()) baseada no exemplo do RNT.
 * Isso deve fazer com que os teclados e botões voltem a funcionar.
 * - ESTRUTURA DE ARQUIVOS: Simplificada para que todos os arquivos (.c, .h, .ino) fiquem na mesma pasta.
 * - INICIALIZAÇÃO EXPLÍCITA: Adicionada a inicialização explícita da biblioteca XPT2046 para mais controle.
 * - WRAPPER 'extern "C"': Adicionado para garantir a correta vinculação dos arquivos C da UI com o código C++ do Arduino.
 */

// ------------------- BIBLIOTECAS -------------------
#include <lvgl.h>
#include <TFT_eSPI.h>
#include <XPT2046_Touchscreen.h>

// ------------------- ARQUIVOS DA UI (com wrapper de segurança) -------------------
// Este wrapper garante que o compilador C++ (Arduino) consiga "conversar"
// corretamente com os arquivos C gerados pelo SquareLine.
extern "C" {
  #include "ui.h"
}

// ------------------- CONFIGURAÇÃO DO HARDWARE (RNT) -------------------
// Pinos do Touchscreen
#define XPT2046_IRQ 36
#define XPT2046_MOSI 32
#define XPT2046_MISO 39
#define XPT2046_CLK 25
#define XPT2046_CS 33

// Calibração do Touchscreen - Valores obtidos do exemplo do RNT
// Estes valores podem precisar de ajuste fino para o seu display específico
#define TOUCH_X_MIN 200
#define TOUCH_X_MAX 3700
#define TOUCH_Y_MIN 240
#define TOUCH_Y_MAX 3800

// Instâncias dos drivers
TFT_eSPI tft = TFT_eSPI();
SPIClass touchscreenSPI = SPIClass(VSPI);
XPT2046_Touchscreen touchscreen(XPT2046_CS);

// ------------------- CONFIGURAÇÃO DO LVGL -------------------
static const uint16_t screenWidth = 320;
static const uint16_t screenHeight = 240;

static lv_disp_draw_buf_t disp_draw_buf;
static lv_color_t buf[screenWidth * screenHeight / 10];

// --- PONTES DE COMUNICAÇÃO LVGL <-> HARDWARE (CORRIGIDAS) ---

/* Ponte de comunicação: LVGL -> Display */
void my_disp_flush(lv_disp_drv_t *disp_drv, const lv_area_t *area, lv_color_t *color_p) {
    uint32_t w = (area->x2 - area->x1 + 1);
    uint32_t h = (area->y2 - area->y1 + 1);

    tft.startWrite();
    tft.setAddrWindow(area->x1, area->y1, w, h);
    tft.pushImage(area->x1, area->y1, w, h, (uint16_t *)&color_p->full);
    tft.endWrite();

    lv_disp_flush_ready(disp_drv);
}

/* Ponte de comunicação: Touch -> LVGL (COM CALIBRAÇÃO) */
void my_touchpad_read(lv_indev_drv_t *indev_drv, lv_indev_data_t *data) {
    if (touchscreen.tirqTouched() && touchscreen.touched()) {
        TS_Point p = touchscreen.getPoint();

        // Mapeia as coordenadas brutas do touch para as coordenadas da tela
        data->point.x = map(p.x, TOUCH_X_MIN, TOUCH_X_MAX, 0, screenWidth);
        data->point.y = map(p.y, TOUCH_Y_MIN, TOUCH_Y_MAX, 0, screenHeight);
        data->state = LV_INDEV_STATE_PR;
    } else {
        data->state = LV_INDEV_STATE_REL;
    }
}


// ------------------- FUNÇÕES PRINCIPAIS (SETUP E LOOP) -------------------
void setup() {
    Serial.begin(115200);
    Serial.println("Iniciando o projeto de UI - Versão Corrigida");

    // 1. Inicializa a biblioteca LVGL
    lv_init();

    // 2. Inicializa o SPI para o Touchscreen
    touchscreenSPI.begin(XPT2046_CLK, XPT2046_MISO, XPT2046_MOSI, XPT2046_CS);
    touchscreen.begin(touchscreenSPI);
    touchscreen.setRotation(1); // Tente 1 ou 3 para paisagem

    // 3. Inicializa o display com TFT_eSPI
    tft.begin();
    tft.setRotation(1); // Deve ser o mesmo que a rotação do touch
    tft.setSwapBytes(true);

    // 4. Configura o buffer de desenho do LVGL
    lv_disp_draw_buf_init(&disp_draw_buf, buf, NULL, screenWidth * screenHeight / 10);

    // 5. Inicializa o driver de display do LVGL
    static lv_disp_drv_t disp_drv;
    lv_disp_drv_init(&disp_drv);
    disp_drv.hor_res = screenWidth;
    disp_drv.ver_res = screenHeight;
    disp_drv.flush_cb = my_disp_flush;
    disp_drv.draw_buf = &disp_draw_buf;
    lv_disp_drv_register(&disp_drv);

    // 6. Inicializa o driver de entrada (touch) do LVGL
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init(&indev_drv);
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = my_touchpad_read;
    lv_indev_drv_register(&indev_drv);

    // 7. CHAMA A FUNÇÃO PRINCIPAL DA UI GERADA PELO SQUARELINE
    ui_init();

    Serial.println("Setup completo. UI deve estar visível e funcional.");
}

void loop() {
    // Processa as tarefas do LVGL (animações, eventos, etc.)
    lv_timer_handler(); 
    
    // Pequeno delay para estabilidade
    delay(5); 
}