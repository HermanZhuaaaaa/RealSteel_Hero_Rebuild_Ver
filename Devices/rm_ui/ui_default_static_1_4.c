//
// Created by RM UI Designer
//

#include "ui_default_static_1_4.h"

#define FRAME_ID 0
#define GROUP_ID 0
#define START_ID 4
#define OBJ_NUM 1
#define FRAME_OBJ_NUM 1

CAT(ui_, CAT(FRAME_OBJ_NUM, _frame_t)) ui_default_static_1_4;
ui_interface_rect_t *ui_default_static_1_supercap_box = (ui_interface_rect_t *)&(ui_default_static_1_4.data[0]);

void _ui_init_default_static_1_4() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_static_1_4.data[i].figure_name[0] = FRAME_ID;
        ui_default_static_1_4.data[i].figure_name[1] = GROUP_ID;
        ui_default_static_1_4.data[i].figure_name[2] = i + START_ID;
        ui_default_static_1_4.data[i].operate_tpyel = 1;
    }
    for (int i = OBJ_NUM; i < FRAME_OBJ_NUM; i++) {
        ui_default_static_1_4.data[i].operate_tpyel = 0;
    }

    ui_default_static_1_supercap_box->figure_tpye = 1;
    ui_default_static_1_supercap_box->layer = 0;
    ui_default_static_1_supercap_box->start_x = 720;
    ui_default_static_1_supercap_box->start_y = 100;
    ui_default_static_1_supercap_box->color = 2;
    ui_default_static_1_supercap_box->width = 5;
    ui_default_static_1_supercap_box->end_x = 1290;
    ui_default_static_1_supercap_box->end_y = 175;


    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_static_1_4);
    SEND_MESSAGE((uint8_t *) &ui_default_static_1_4, sizeof(ui_default_static_1_4));
}

void _ui_update_default_static_1_4() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_static_1_4.data[i].operate_tpyel = 2;
    }

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_static_1_4);
    SEND_MESSAGE((uint8_t *) &ui_default_static_1_4, sizeof(ui_default_static_1_4));
}

void _ui_remove_default_static_1_4() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_static_1_4.data[i].operate_tpyel = 3;
    }

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_static_1_4);
    SEND_MESSAGE((uint8_t *) &ui_default_static_1_4, sizeof(ui_default_static_1_4));
}
