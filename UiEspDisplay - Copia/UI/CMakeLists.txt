SET(SOURCES ui_PrimeiraTela.c
    ui_SplashScreen.c
    ui_ParametrosEspRemoto.c
    ui_ParametrosEspLocal.c
    ui_ParametrosExperimento.c
    ui_ConexaoWifiEspCaseira.c
    ui_ConexaoWifiEspEstudante.c
    ui_Sucesso.c
    ui_Inicio.c
    ui_Reservatorios.c
    ui.c
    ui_comp_hook.c
    ui_helpers.c
    ui_img_logo_ufu_png.c)

add_library(ui ${SOURCES})
