// This file was generated by SquareLine Studio
// SquareLine Studio version: SquareLine Studio 1.5.3
// LVGL version: 8.3.11
// Project name: espDisplayUi

#include "ui.h"

lv_obj_t * ui_telaAnimacaoLocal = NULL;
lv_obj_t * ui_barnivelagua1 = NULL;
lv_obj_t * ui_Panel2 = NULL;
lv_obj_t * ui_Label20 = NULL;
lv_obj_t * ui_Panel3 = NULL;
lv_obj_t * ui_labelValorNvel1 = NULL;
lv_obj_t * ui_Label21 = NULL;
lv_obj_t * ui_Label22 = NULL;
lv_obj_t * ui_Label23 = NULL;
lv_obj_t * ui_Label24 = NULL;
lv_obj_t * ui_Label25 = NULL;
lv_obj_t * ui_Label26 = NULL;
lv_obj_t * ui_Label27 = NULL;
lv_obj_t * ui_Label28 = NULL;
lv_obj_t * ui_Label29 = NULL;
lv_obj_t * ui_Label30 = NULL;
lv_obj_t * ui_Label31 = NULL;
lv_obj_t * ui_Label32 = NULL;
lv_obj_t * ui_Label33 = NULL;
lv_obj_t * ui_Label34 = NULL;
lv_obj_t * ui_Label35 = NULL;
lv_obj_t * ui_Label36 = NULL;
lv_obj_t * ui_atualizaReferencia = NULL;
lv_obj_t * ui_Button2 = NULL;
lv_obj_t * ui_Label37 = NULL;
lv_obj_t * ui_Spinner5 = NULL;
lv_obj_t * ui_Button7 = NULL;
lv_obj_t * ui_Label42 = NULL;
lv_obj_t * ui_Keyboard5 = NULL;
// event funtions
void ui_event_telaAnimacaoLocal(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_SCREEN_LOADED) {
        _ui_flag_modify(ui_Spinner5, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_ADD);
    }
}

void ui_event_atualizaReferencia(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_CLICKED) {
        _ui_flag_modify(ui_Keyboard5, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_REMOVE);
    }
}

void ui_event_Button2(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_CLICKED) {
        _ui_screen_change(&ui_InicialScreen, LV_SCR_LOAD_ANIM_FADE_ON, 500, 8000, &ui_InicialScreen_screen_init);
        _ui_flag_modify(ui_Spinner5, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_REMOVE);
    }
}

void ui_event_Button7(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_CLICKED) {
        _ui_screen_change(&ui_InicialScreen, LV_SCR_LOAD_ANIM_FADE_ON, 500, 0, &ui_InicialScreen_screen_init);
    }
}

void ui_event_Keyboard5(lv_event_t * e)
{
    lv_event_code_t event_code = lv_event_get_code(e);

    if(event_code == LV_EVENT_READY) {
        _ui_flag_modify(ui_Keyboard5, LV_OBJ_FLAG_HIDDEN, _UI_MODIFY_FLAG_ADD);
    }
}

// build funtions

void ui_telaAnimacaoLocal_screen_init(void)
{
    ui_telaAnimacaoLocal = lv_obj_create(NULL);
    lv_obj_clear_flag(ui_telaAnimacaoLocal,
                      LV_OBJ_FLAG_SCROLLABLE | LV_OBJ_FLAG_SCROLL_ELASTIC | LV_OBJ_FLAG_SCROLL_MOMENTUM |
                      LV_OBJ_FLAG_SCROLL_CHAIN);     /// Flags

    ui_barnivelagua1 = lv_bar_create(ui_telaAnimacaoLocal);
    lv_bar_set_range(ui_barnivelagua1, 0, 17);
    lv_bar_set_value(ui_barnivelagua1, 16, LV_ANIM_OFF);
    lv_bar_set_start_value(ui_barnivelagua1, 0, LV_ANIM_OFF);
    lv_obj_set_width(ui_barnivelagua1, 190);
    lv_obj_set_height(ui_barnivelagua1, 72);
    lv_obj_set_x(ui_barnivelagua1, -3);
    lv_obj_set_y(ui_barnivelagua1, 127);
    lv_obj_set_align(ui_barnivelagua1, LV_ALIGN_CENTER);
    lv_obj_set_style_transform_angle(ui_barnivelagua1, -900, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Panel2 = lv_obj_create(ui_telaAnimacaoLocal);
    lv_obj_set_width(ui_Panel2, 132);
    lv_obj_set_height(ui_Panel2, 28);
    lv_obj_set_x(ui_Panel2, 90);
    lv_obj_set_y(ui_Panel2, -77);
    lv_obj_set_align(ui_Panel2, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Panel2, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_Panel2, lv_color_hex(0xCCC7C7), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Panel2, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label20 = lv_label_create(ui_Panel2);
    lv_obj_set_width(ui_Label20, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label20, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_Label20, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label20, "Nivel da agua");

    ui_Panel3 = lv_obj_create(ui_telaAnimacaoLocal);
    lv_obj_set_width(ui_Panel3, 128);
    lv_obj_set_height(ui_Panel3, 50);
    lv_obj_set_x(ui_Panel3, 89);
    lv_obj_set_y(ui_Panel3, -35);
    lv_obj_set_align(ui_Panel3, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Panel3, LV_OBJ_FLAG_SCROLLABLE);      /// Flags

    ui_labelValorNvel1 = lv_label_create(ui_Panel3);
    lv_obj_set_width(ui_labelValorNvel1, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_labelValorNvel1, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_labelValorNvel1, 0);
    lv_obj_set_y(ui_labelValorNvel1, -1);
    lv_obj_set_align(ui_labelValorNvel1, LV_ALIGN_CENTER);
    lv_label_set_text(ui_labelValorNvel1, "0");

    ui_Label21 = lv_label_create(ui_telaAnimacaoLocal);
    lv_obj_set_width(ui_Label21, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label21, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label21, -48);
    lv_obj_set_y(ui_Label21, 79);
    lv_obj_set_align(ui_Label21, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label21, "-----");

    ui_Label22 = lv_label_create(ui_telaAnimacaoLocal);
    lv_obj_set_width(ui_Label22, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label22, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label22, -41);
    lv_obj_set_y(ui_Label22, 68);
    lv_obj_set_align(ui_Label22, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label22, "-----");

    ui_Label23 = lv_label_create(ui_telaAnimacaoLocal);
    lv_obj_set_width(ui_Label23, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label23, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label23, -41);
    lv_obj_set_y(ui_Label23, 58);
    lv_obj_set_align(ui_Label23, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label23, "-----");

    ui_Label24 = lv_label_create(ui_telaAnimacaoLocal);
    lv_obj_set_width(ui_Label24, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label24, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label24, -41);
    lv_obj_set_y(ui_Label24, 47);
    lv_obj_set_align(ui_Label24, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label24, "-----");

    ui_Label25 = lv_label_create(ui_telaAnimacaoLocal);
    lv_obj_set_width(ui_Label25, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label25, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label25, -41);
    lv_obj_set_y(ui_Label25, 36);
    lv_obj_set_align(ui_Label25, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label25, "-----");

    ui_Label26 = lv_label_create(ui_telaAnimacaoLocal);
    lv_obj_set_width(ui_Label26, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label26, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label26, -41);
    lv_obj_set_y(ui_Label26, 24);
    lv_obj_set_align(ui_Label26, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label26, "-----");

    ui_Label27 = lv_label_create(ui_telaAnimacaoLocal);
    lv_obj_set_width(ui_Label27, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label27, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label27, -40);
    lv_obj_set_y(ui_Label27, 13);
    lv_obj_set_align(ui_Label27, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label27, "-----");

    ui_Label28 = lv_label_create(ui_telaAnimacaoLocal);
    lv_obj_set_width(ui_Label28, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label28, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label28, -41);
    lv_obj_set_y(ui_Label28, 2);
    lv_obj_set_align(ui_Label28, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label28, "-----");

    ui_Label29 = lv_label_create(ui_telaAnimacaoLocal);
    lv_obj_set_width(ui_Label29, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label29, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label29, -41);
    lv_obj_set_y(ui_Label29, -9);
    lv_obj_set_align(ui_Label29, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label29, "-----");

    ui_Label30 = lv_label_create(ui_telaAnimacaoLocal);
    lv_obj_set_width(ui_Label30, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label30, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label30, -41);
    lv_obj_set_y(ui_Label30, -20);
    lv_obj_set_align(ui_Label30, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label30, "-----");

    ui_Label31 = lv_label_create(ui_telaAnimacaoLocal);
    lv_obj_set_width(ui_Label31, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label31, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label31, -41);
    lv_obj_set_y(ui_Label31, -31);
    lv_obj_set_align(ui_Label31, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label31, "-----");

    ui_Label32 = lv_label_create(ui_telaAnimacaoLocal);
    lv_obj_set_width(ui_Label32, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label32, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label32, -41);
    lv_obj_set_y(ui_Label32, -43);
    lv_obj_set_align(ui_Label32, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label32, "-----");

    ui_Label33 = lv_label_create(ui_telaAnimacaoLocal);
    lv_obj_set_width(ui_Label33, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label33, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label33, -41);
    lv_obj_set_y(ui_Label33, -55);
    lv_obj_set_align(ui_Label33, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label33, "-----");

    ui_Label34 = lv_label_create(ui_telaAnimacaoLocal);
    lv_obj_set_width(ui_Label34, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label34, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label34, -41);
    lv_obj_set_y(ui_Label34, -66);
    lv_obj_set_align(ui_Label34, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label34, "-----");

    ui_Label35 = lv_label_create(ui_telaAnimacaoLocal);
    lv_obj_set_width(ui_Label35, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label35, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label35, -41);
    lv_obj_set_y(ui_Label35, -77);
    lv_obj_set_align(ui_Label35, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label35, "-----");

    ui_Label36 = lv_label_create(ui_telaAnimacaoLocal);
    lv_obj_set_width(ui_Label36, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label36, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_x(ui_Label36, -49);
    lv_obj_set_y(ui_Label36, -88);
    lv_obj_set_align(ui_Label36, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label36, "-----");

    ui_atualizaReferencia = lv_textarea_create(ui_telaAnimacaoLocal);
    lv_obj_set_width(ui_atualizaReferencia, 150);
    lv_obj_set_height(ui_atualizaReferencia, 39);
    lv_obj_set_x(ui_atualizaReferencia, 75);
    lv_obj_set_y(ui_atualizaReferencia, 14);
    lv_obj_set_align(ui_atualizaReferencia, LV_ALIGN_CENTER);
    lv_textarea_set_placeholder_text(ui_atualizaReferencia, "Referencia");

    ui_Button2 = lv_btn_create(ui_telaAnimacaoLocal);
    lv_obj_set_width(ui_Button2, 100);
    lv_obj_set_height(ui_Button2, 44);
    lv_obj_set_x(ui_Button2, 94);
    lv_obj_set_y(ui_Button2, 82);
    lv_obj_set_align(ui_Button2, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Button2, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_Button2, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_Button2, lv_color_hex(0x32EC2B), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Button2, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label37 = lv_label_create(ui_Button2);
    lv_obj_set_width(ui_Label37, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label37, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_Label37, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label37, "Concluir");

    ui_Spinner5 = lv_spinner_create(ui_telaAnimacaoLocal, 1000, 90);
    lv_obj_set_width(ui_Spinner5, 33);
    lv_obj_set_height(ui_Spinner5, 26);
    lv_obj_set_x(ui_Spinner5, -128);
    lv_obj_set_y(ui_Spinner5, 97);
    lv_obj_set_align(ui_Spinner5, LV_ALIGN_CENTER);
    lv_obj_clear_flag(ui_Spinner5, LV_OBJ_FLAG_CLICKABLE);      /// Flags
    lv_obj_set_style_arc_width(ui_Spinner5, 2, LV_PART_MAIN | LV_STATE_DEFAULT);

    lv_obj_set_style_arc_color(ui_Spinner5, lv_color_hex(0x4040FF), LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_arc_opa(ui_Spinner5, 255, LV_PART_INDICATOR | LV_STATE_DEFAULT);
    lv_obj_set_style_arc_width(ui_Spinner5, 2, LV_PART_INDICATOR | LV_STATE_DEFAULT);

    ui_Button7 = lv_btn_create(ui_telaAnimacaoLocal);
    lv_obj_set_width(ui_Button7, 25);
    lv_obj_set_height(ui_Button7, 25);
    lv_obj_set_x(ui_Button7, 143);
    lv_obj_set_y(ui_Button7, -107);
    lv_obj_set_align(ui_Button7, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Button7, LV_OBJ_FLAG_SCROLL_ON_FOCUS);     /// Flags
    lv_obj_clear_flag(ui_Button7, LV_OBJ_FLAG_SCROLLABLE);      /// Flags
    lv_obj_set_style_bg_color(ui_Button7, lv_color_hex(0xEA0808), LV_PART_MAIN | LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui_Button7, 255, LV_PART_MAIN | LV_STATE_DEFAULT);

    ui_Label42 = lv_label_create(ui_Button7);
    lv_obj_set_width(ui_Label42, LV_SIZE_CONTENT);   /// 1
    lv_obj_set_height(ui_Label42, LV_SIZE_CONTENT);    /// 1
    lv_obj_set_align(ui_Label42, LV_ALIGN_CENTER);
    lv_label_set_text(ui_Label42, "x");

    ui_Keyboard5 = lv_keyboard_create(ui_telaAnimacaoLocal);
    lv_obj_set_width(ui_Keyboard5, 316);
    lv_obj_set_height(ui_Keyboard5, 120);
    lv_obj_set_x(ui_Keyboard5, 4);
    lv_obj_set_y(ui_Keyboard5, 54);
    lv_obj_set_align(ui_Keyboard5, LV_ALIGN_CENTER);
    lv_obj_add_flag(ui_Keyboard5, LV_OBJ_FLAG_HIDDEN);     /// Flags

    lv_obj_add_event_cb(ui_atualizaReferencia, ui_event_atualizaReferencia, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_Button2, ui_event_Button2, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_Button7, ui_event_Button7, LV_EVENT_ALL, NULL);
    lv_keyboard_set_textarea(ui_Keyboard5, ui_atualizaReferencia);
    lv_obj_add_event_cb(ui_Keyboard5, ui_event_Keyboard5, LV_EVENT_ALL, NULL);
    lv_obj_add_event_cb(ui_telaAnimacaoLocal, ui_event_telaAnimacaoLocal, LV_EVENT_ALL, NULL);

}

void ui_telaAnimacaoLocal_screen_destroy(void)
{
    if(ui_telaAnimacaoLocal) lv_obj_del(ui_telaAnimacaoLocal);

    // NULL screen variables
    ui_telaAnimacaoLocal = NULL;
    ui_barnivelagua1 = NULL;
    ui_Panel2 = NULL;
    ui_Label20 = NULL;
    ui_Panel3 = NULL;
    ui_labelValorNvel1 = NULL;
    ui_Label21 = NULL;
    ui_Label22 = NULL;
    ui_Label23 = NULL;
    ui_Label24 = NULL;
    ui_Label25 = NULL;
    ui_Label26 = NULL;
    ui_Label27 = NULL;
    ui_Label28 = NULL;
    ui_Label29 = NULL;
    ui_Label30 = NULL;
    ui_Label31 = NULL;
    ui_Label32 = NULL;
    ui_Label33 = NULL;
    ui_Label34 = NULL;
    ui_Label35 = NULL;
    ui_Label36 = NULL;
    ui_atualizaReferencia = NULL;
    ui_Button2 = NULL;
    ui_Label37 = NULL;
    ui_Spinner5 = NULL;
    ui_Button7 = NULL;
    ui_Label42 = NULL;
    ui_Keyboard5 = NULL;

}
