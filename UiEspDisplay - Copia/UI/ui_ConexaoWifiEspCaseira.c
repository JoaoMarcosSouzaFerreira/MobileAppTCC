// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.5.3
// LVGL version: 8.3.11
// Project name: espDisplayUi

#include "ui.h"

lv_obj_t * ui_ConexaoWifiEspCaseira = NULL;
lv_obj_t * ui_painelIputs3 = NULL;
lv_obj_t * ui_inputWifi = NULL;
lv_obj_t * ui_inputSenhaC = NULL;
lv_obj_t * ui_modoLocalRemoto2 = NULL;
lv_obj_t * ui_labelLocalRemoto2 = NULL;
lv_obj_t * ui_caseiraEstudante = NULL;
lv_obj_t * ui_tituloTelaParametrosBroker3 = NULL;
lv_obj_t * ui_botaoConectaWifi1 = NULL;
lv_obj_t * ui_labelBotaoConecta2 = NULL;
lv_obj_t * ui_Spinner6 = NULL;
lv_obj_t * ui_botaoReseta1 = NULL;
lv_obj_t * ui_Label43 = NULL;
lv_obj_t * ui_KeyboardWifiCaseira = NULL;
// event funtions
void ui_event_ConexaoWifiEspCaseira(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_SCREEN_LOAD_START) {
        _ui_flag_modify(ui_Spinner6, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_ADD);
    }
}

void ui_event_inputWifi(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_CLICKED) {
        _ui_flag_modify(ui_KeyboardWifiCaseira, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_REMOVE);
    }
    if(event_code == LV_EVENT_FOCUSED) {
        _ui_keyboard_set_target(ui_KeyboardWifiCaseira,  ui_inputWifi);
    }
}

void ui_event_inputSenhaC(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_CLICKED) {
        _ui_flag_modify(ui_KeyboardWifiCaseira, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_REMOVE);
    }
    if(event_code == LV_EVENT_FOCUSED) {
        _ui_keyboard_set_target(ui_KeyboardWifiCaseira,  ui_inputSenhaC);
    }
    if(event_code == LV_EVENT_CLICKED) {
        _ui_basic_set_property(ui_painelIputs3, _UI_BASIC_PROPERTY_POSITION_Y,  -75);
        _ui_basic_set_property(ui_tituloTelaParametrosBroker3, _UI_BASIC_PROPERTY_POSITION_Y,  -150);
        _ui_basic_set_property(ui_botaoReseta1, _UI_BASIC_PROPERTY_POSITION_Y,  -150);
    }
}

void ui_event_caseiraEstudante(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_CLICKED) {
        _ui_screen_change(&ui_ConexaoWifiEspEstudante, LV_SCR_LOAD_ANIM_NONE, 0, 0, &ui_ConexaoWifiEspEstudante_screen_init);
        _ui_state_modify(ui_caseiraEstudante, LV_STATE_CHECKED, _UI_MODIFY_STATE_REMOVE);
    }
}

void ui_event_botaoConectaWifi1(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_CLICKED) {
        _ui_flag_modify(ui_Spinner6, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_REMOVE);
        _ui_screen_change(&ui_ParametrosEspRemoto, LV_SCR_LOAD_ANIM_OVER_LEFT, 500, 8000, &ui_ParametrosEspRemoto_screen_init);
    }
}

void ui_event_botaoReseta1(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_CLICKED) {
        _ui_screen_change(&ui_Inicio, LV_SCR_LOAD_ANIM_OVER_RIGHT, 500, 0, &ui_Inicio_screen_init);
    }
}

void ui_event_KeyboardWifiCaseira(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_READY) {
        _ui_flag_modify(ui_KeyboardWifiCaseira, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_ADD);
        _ui_basic_set_property(ui_painelIputs3, _UI_BASIC_PROPERTY_POSITION_Y,  -8);
        _ui_basic_set_property(ui_tituloTelaParametrosBroker3, _UI_BASIC_PROPERTY_POSITION_Y,  -100);
        _ui_basic_set_property(ui_botaoReseta1, _UI_BASIC_PROPERTY_POSITION_Y,  -104);
    }
}

// build funtions

void ui_ConexaoWifiEspCaseira_screen_init(void)
{
    ui_ConexaoWifiEspCaseira = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_ConexaoWifiEspCaseira,
                      LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM | LV_OBJ_FLAG_SCROLL_CHAIN);     /// Flags
    lv_obj_set_scrollbar_mode(ui_ConexaoWifiEspCaseira, LV_SCROLLBAR_MODE_ACTIVE);
    lv_obj_set_scroll_dir(ui_ConexaoWifiEspCaseira, LV_DIR_TOP);
    lv_obj_set_scroll_snap_x(ui_ConexaoWifiEspCaseira, LV_SCROLL_SNAP_CENTER);
    lv_obj_set_style_bg_color(ui_ConexaoWifiEspCaseira, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_ConexaoWifiEspCaseira, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_painelIputs3 = lv_obj_create(ui_ConexaoWifiEspCaseira);
    lv_obj_set_width(ui_painelIputs3, 300);
    lv_obj_set_height(ui_painelIputs3, 137);
    lv_obj_set_x(ui_painelIputs3, 0);
    lv_obj_set_y(ui_painelIputs3, -8);
    lv_obj_set_align(ui_painelIputs3, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_painelIputs3, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_painelIputs3, lv_color_hex(0xE0DFFD), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_painelIputs3, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_inputWifi = lv_textarea_create(ui_painelIputs3);
    lv_obj_set_width(ui_inputWifi, 250);
    lv_obj_set_height(ui_inputWifi, LV_SIZE_CONTENT);    /// 25
    lv_obj_set_x(ui_inputWifi, -8);
    lv_obj_set_y(ui_inputWifi, -11);
    lv_obj_set_align(ui_inputWifi, LV_ALIGN_CENTER);
    lv_textarea_set_max_length(ui_inputWifi, 100);
    lv_textarea_set_placeholder_text(ui_inputWifi, "Rede Wi-fi");
    lv_textarea_set_one_line(ui_inputWifi, true);

    ui_inputSenhaC = lv_textarea_create(ui_painelIputs3);
    lv_obj_set_width(ui_inputSenhaC, 250);
    lv_obj_set_height(ui_inputSenhaC, LV_SIZE_CONTENT);    /// 25
    lv_obj_set_x(ui_inputSenhaC, -9);
    lv_obj_set_y(ui_inputSenhaC, 39);
    lv_obj_set_align(ui_inputSenhaC, LV_ALIGN_CENTER);
    lv_textarea_set_max_length(ui_inputSenhaC, 100);
    lv_textarea_set_placeholder_text(ui_inputSenhaC, "Senha");
    lv_textarea_set_one_line(ui_inputSenhaC, true);

    ui_modoLocalRemoto2 = lv_obj_create(ui_painelIputs3);
    lv_obj_remove_style_all(ui_modoLocalRemoto2);
    lv_obj_set_width(ui_modoLocalRemoto2, 281);
    lv_obj_set_height(ui_modoLocalRemoto2, 33);
    lv_obj_set_x(ui_modoLocalRemoto2, 2);
    lv_obj_set_y(ui_modoLocalRemoto2, -49);
    lv_obj_set_align(ui_modoLocalRemoto2, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_modoLocalRemoto2, LV_OBJ_FLAG_CLICKABLE | LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_labelLocalRemoto2 = lv_label_create(ui_modoLocalRemoto2);
    lv_obj_set_width(ui_labelLocalRemoto2, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_labelLocalRemoto2, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_labelLocalRemoto2, -29);
    lv_obj_set_y(ui_labelLocalRemoto2, -2);
    lv_obj_set_align(ui_labelLocalRemoto2, LV_ALIGN_CENTER);
    lv_label_set_text(ui_labelLocalRemoto2, "Utilizando rede corporativa?");

    ui_caseiraEstudante = lv_switch_create(ui_modoLocalRemoto2);
    lv_obj_set_width(ui_caseiraEstudante, 50);
    lv_obj_set_height(ui_caseiraEstudante, 25);
    lv_obj_set_x(ui_caseiraEstudante, 114);
    lv_obj_set_y(ui_caseiraEstudante, -2);
    lv_obj_set_align(ui_caseiraEstudante, LV_ALIGN_CENTER);
    lv_obj_set_style_bg_color(ui_caseiraEstudante, lv_color_hex(0xABAAAD), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_caseiraEstudante, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_caseiraEstudante, lv_color_hex(0xE0DFFD), LV_PART_MAIN | LV_STATE_CHECKED);
    lv_obj_set_style_bg_opa(ui_caseiraEstudante, 255, LV_PART_MAIN | LV_STATE_CHECKED);

    lv_obj_set_style_bg_color(ui_caseiraEstudante, lv_color_hex(0xFFFFFF), LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_caseiraEstudante, 255, LV_PART_KNOB | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_caseiraEstudante, lv_color_hex(0xFFFFFF), LV_PART_KNOB | LV_STATE_CHECKED);
    lv_obj_set_style_bg_opa(ui_caseiraEstudante, 255, LV_PART_KNOB | LV_STATE_CHECKED);
    lv_obj_set_style_bg_color(ui_caseiraEstudante, lv_color_hex(0xFFFFFF), LV_PART_KNOB | LV_STATE_PRESSED);
    lv_obj_set_style_bg_opa(ui_caseiraEstudante, 255, LV_PART_KNOB | LV_STATE_PRESSED);

    ui_tituloTelaParametrosBroker3 = lv_label_create(ui_ConexaoWifiEspCaseira);
    lv_obj_set_width(ui_tituloTelaParametrosBroker3, 235);
    lv_obj_set_height(ui_tituloTelaParametrosBroker3, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_tituloTelaParametrosBroker3, 0);
    lv_obj_set_y(ui_tituloTelaParametrosBroker3, -100);
    lv_obj_set_align(ui_tituloTelaParametrosBroker3, LV_ALIGN_CENTER);
    lv_label_set_text(ui_tituloTelaParametrosBroker3, "Insira os dados abaixo para conectar a \nbancada na internet");
    lv_obj_set_style_text_align(ui_tituloTelaParametrosBroker3, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui_tituloTelaParametrosBroker3, &lv_font_montserrat_12, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_botaoConectaWifi1 = lv_btn_create(ui_ConexaoWifiEspCaseira);
    lv_obj_set_width(ui_botaoConectaWifi1, 174);
    lv_obj_set_height(ui_botaoConectaWifi1, 32);
    lv_obj_set_x(ui_botaoConectaWifi1, 61);
    lv_obj_set_y(ui_botaoConectaWifi1, 90);
    lv_obj_set_align(ui_botaoConectaWifi1, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_botaoConectaWifi1, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_botaoConectaWifi1, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_botaoConectaWifi1, lv_color_hex(0x0FD93F), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_botaoConectaWifi1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_color(ui_botaoConectaWifi1, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_opa(ui_botaoConectaWifi1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_ofs_x(ui_botaoConectaWifi1, 3, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_ofs_y(ui_botaoConectaWifi1, 2, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_botaoConectaWifi1, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_botaoConectaWifi1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui_botaoConectaWifi1, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_labelBotaoConecta2 = lv_label_create(ui_botaoConectaWifi1);
    lv_obj_set_width(ui_labelBotaoConecta2, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_labelBotaoConecta2, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_labelBotaoConecta2, LV_ALIGN_CENTER);
    lv_label_set_text(ui_labelBotaoConecta2, "Configurar Broker");

    ui_Spinner6 = lv_spinner_create(ui_ConexaoWifiEspCaseira, 1000, 90);
    lv_obj_set_width(ui_Spinner6, 33);
    lv_obj_set_height(ui_Spinner6, 26);
    lv_obj_set_x(ui_Spinner6, -126);
    lv_obj_set_y(ui_Spinner6, 92);
    lv_obj_set_align(ui_Spinner6, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Spinner6, LV_OBJ_FLAG_CLICKABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_Spinner6, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Spinner6, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_color(ui_Spinner6, lv_color_hex(0xFFFFFF), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_main_stop(ui_Spinner6, 0, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_stop(ui_Spinner6, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_arc_width(ui_Spinner6, 2, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_set_style_arc_color(ui_Spinner6, lv_color_hex(0x4040FF), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_arc_opa(ui_Spinner6, 255, LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_arc_width(ui_Spinner6, 2, LV_PART_INDICATOR | LV_STATE_DEFAULT);

    ui_botaoReseta1 = lv_btn_create(ui_ConexaoWifiEspCaseira);
    lv_obj_set_width(ui_botaoReseta1, 25);
    lv_obj_set_height(ui_botaoReseta1, 25);
    lv_obj_set_x(ui_botaoReseta1, 141);
    lv_obj_set_y(ui_botaoReseta1, -104);
    lv_obj_set_align(ui_botaoReseta1, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_botaoReseta1, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_botaoReseta1, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_botaoReseta1, lv_color_hex(0xF30606), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_botaoReseta1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_color(ui_botaoReseta1, lv_color_hex(0x000000), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_opa(ui_botaoReseta1, 255, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_ofs_x(ui_botaoReseta1, 3, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_ofs_y(ui_botaoReseta1, 2, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label43 = lv_label_create(ui_botaoReseta1);
    lv_obj_set_width(ui_Label43, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label43, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_Label43, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label43, "x");

    ui_KeyboardWifiCaseira = lv_keyboard_create(ui_ConexaoWifiEspCaseira);
    lv_obj_set_width(ui_KeyboardWifiCaseira, 316);
    lv_obj_set_height(ui_KeyboardWifiCaseira, 120);
    lv_obj_set_x(ui_KeyboardWifiCaseira, 2);
    lv_obj_set_y(ui_KeyboardWifiCaseira, 58);
    lv_obj_set_align(ui_KeyboardWifiCaseira, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_KeyboardWifiCaseira, LV_OBJ_FLAG_HIDDEN);     /// Flags
    lv_obj_set_style_radius(ui_KeyboardWifiCaseira, 15, LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_KeyboardWifiCaseira, lv_color_hex(0x223E50), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_KeyboardWifiCaseira, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_set_style_radius(ui_KeyboardWifiCaseira, 10, LV_PART_ITEMS | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_KeyboardWifiCaseira, lv_color_hex(0x366381), LV_PART_ITEMS | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_KeyboardWifiCaseira, 255, LV_PART_ITEMS | LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui_KeyboardWifiCaseira, lv_color_hex(0xFFFFFF), LV_PART_ITEMS | LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui_KeyboardWifiCaseira, 255, LV_PART_ITEMS | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui_KeyboardWifiCaseira, lv_color_hex(0xD5EFFF), LV_PART_ITEMS | LV_STATE_CHECKED);
    lv_obj_set_style_bg_opa(ui_KeyboardWifiCaseira, 255, LV_PART_ITEMS | LV_STATE_CHECKED);
    lv_obj_set_style_bg_color(ui_KeyboardWifiCaseira, lv_color_hex(0x224359), LV_PART_ITEMS | LV_STATE_PRESSED);
    lv_obj_set_style_bg_opa(ui_KeyboardWifiCaseira, 255, LV_PART_ITEMS | LV_STATE_PRESSED);
    lv_obj_set_style_outline_color(ui_KeyboardWifiCaseira, lv_color_hex(0x009FC7), LV_PART_ITEMS | LV_STATE_PRESSED);
    lv_obj_set_style_outline_opa(ui_KeyboardWifiCaseira, 255, LV_PART_ITEMS | LV_STATE_PRESSED);
    lv_obj_set_style_outline_width(ui_KeyboardWifiCaseira, 3, LV_PART_ITEMS | LV_STATE_PRESSED);
    lv_obj_set_style_outline_pad(ui_KeyboardWifiCaseira, 0, LV_PART_ITEMS | LV_STATE_PRESSED);
    lv_obj_set_style_text_color(ui_KeyboardWifiCaseira, lv_color_hex(0xFFFFFF), LV_PART_ITEMS | LV_STATE_PRESSED);
    lv_obj_set_style_text_opa(ui_KeyboardWifiCaseira, 255, LV_PART_ITEMS | LV_STATE_PRESSED);

    lv_obj_add_event_cb(ui_inputWifi, ui_event_inputWifi, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_inputSenhaC, ui_event_inputSenhaC, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_caseiraEstudante, ui_event_caseiraEstudante, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_botaoConectaWifi1, ui_event_botaoConectaWifi1, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_botaoReseta1, ui_event_botaoReseta1, LV_EVENT_ALL, NULL);
    lv_keyboard_set_textarea(ui_KeyboardWifiCaseira, ui_inputWifi);
    lv_obj_add_event_cb(ui_KeyboardWifiCaseira, ui_event_KeyboardWifiCaseira, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_ConexaoWifiEspCaseira, ui_event_ConexaoWifiEspCaseira, LV_EVENT_ALL, NULL);

}

void ui_ConexaoWifiEspCaseira_screen_destroy(void)
{
    if(ui_ConexaoWifiEspCaseira) lv_obj_del(ui_ConexaoWifiEspCaseira);

    // NULL screen variables
    ui_ConexaoWifiEspCaseira = NULL;
    ui_painelIputs3 = NULL;
    ui_inputWifi = NULL;
    ui_inputSenhaC = NULL;
    ui_modoLocalRemoto2 = NULL;
    ui_labelLocalRemoto2 = NULL;
    ui_caseiraEstudante = NULL;
    ui_tituloTelaParametrosBroker3 = NULL;
    ui_botaoConectaWifi1 = NULL;
    ui_labelBotaoConecta2 = NULL;
    ui_Spinner6 = NULL;
    ui_botaoReseta1 = NULL;
    ui_Label43 = NULL;
    ui_KeyboardWifiCaseira = NULL;

}
