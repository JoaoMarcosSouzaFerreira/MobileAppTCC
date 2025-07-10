SET(SOURCES ui_PrimeiraTela.c
    ui_SplashScreen.c
    ui_ParametrosEspRemoto.c
    ui_ParametrosEspLocal.c
    ui_ParametrosExperimento.c
    ui_ListaRedes.c
    ui_Sucesso.c
    ui.c
    ui_comp_hook.c
    ui_helpers.c
    ui_img_logo_ufu_png.c
    ui_img_atualizar_png.c)

add_library(ui ${SOURCES})
