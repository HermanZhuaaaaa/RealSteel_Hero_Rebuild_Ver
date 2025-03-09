//
// Created by RM UI Designer
//

#include "ui_default_static_2_0.h"

#define FRAME_ID 0
#define GROUP_ID 5
#define START_ID 0
#define OBJ_NUM 5
#define FRAME_OBJ_NUM 5

CAT(ui_, CAT(FRAME_OBJ_NUM, _frame_t)) ui_default_static_2_0;
ui_interface_line_t *ui_default_static_2_CrossHair = (ui_interface_line_t *)&(ui_default_static_2_0.data[0]);
ui_interface_line_t *ui_default_static_2_line1 = (ui_interface_line_t *)&(ui_default_static_2_0.data[1]);
ui_interface_line_t *ui_default_static_2_line2 = (ui_interface_line_t *)&(ui_default_static_2_0.data[2]);
ui_interface_line_t *ui_default_static_2_line3 = (ui_interface_line_t *)&(ui_default_static_2_0.data[3]);
ui_interface_line_t *ui_default_static_2_line4 = (ui_interface_line_t *)&(ui_default_static_2_0.data[4]);

void _ui_init_default_static_2_0() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_static_2_0.data[i].figure_name[0] = FRAME_ID;
        ui_default_static_2_0.data[i].figure_name[1] = GROUP_ID;
        ui_default_static_2_0.data[i].figure_name[2] = i + START_ID;
        ui_default_static_2_0.data[i].operate_tpyel = 1;
    }
    for (int i = OBJ_NUM; i < FRAME_OBJ_NUM; i++) {
        ui_default_static_2_0.data[i].operate_tpyel = 0;
    }

    ui_default_static_2_CrossHair->figure_tpye = 0;
    ui_default_static_2_CrossHair->layer = 0;
    ui_default_static_2_CrossHair->start_x = 960;
    ui_default_static_2_CrossHair->start_y = 282;
    ui_default_static_2_CrossHair->end_x = 960;
    ui_default_static_2_CrossHair->end_y = 879;
    ui_default_static_2_CrossHair->color = 6;
    ui_default_static_2_CrossHair->width = 1;

    ui_default_static_2_line1->figure_tpye = 0;
    ui_default_static_2_line1->layer = 0;
    ui_default_static_2_line1->start_x = 762;
    ui_default_static_2_line1->start_y = 319;
    ui_default_static_2_line1->end_x = 1162;
    ui_default_static_2_line1->end_y = 319;
    ui_default_static_2_line1->color = 0;
    ui_default_static_2_line1->width = 1;

    ui_default_static_2_line2->figure_tpye = 0;
    ui_default_static_2_line2->layer = 0;
    ui_default_static_2_line2->start_x = 797;
    ui_default_static_2_line2->start_y = 498;
    ui_default_static_2_line2->end_x = 1122;
    ui_default_static_2_line2->end_y = 498;
    ui_default_static_2_line2->color = 0;
    ui_default_static_2_line2->width = 1;

    ui_default_static_2_line3->figure_tpye = 0;
    ui_default_static_2_line3->layer = 0;
    ui_default_static_2_line3->start_x = 677;
    ui_default_static_2_line3->start_y = 615;
    ui_default_static_2_line3->end_x = 1241;
    ui_default_static_2_line3->end_y = 615;
    ui_default_static_2_line3->color = 0;
    ui_default_static_2_line3->width = 1;

    ui_default_static_2_line4->figure_tpye = 0;
    ui_default_static_2_line4->layer = 0;
    ui_default_static_2_line4->start_x = 780;
    ui_default_static_2_line4->start_y = 409;
    ui_default_static_2_line4->end_x = 1140;
    ui_default_static_2_line4->end_y = 409;
    ui_default_static_2_line4->color = 0;
    ui_default_static_2_line4->width = 1;


    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_static_2_0);
    SEND_MESSAGE((uint8_t *) &ui_default_static_2_0, sizeof(ui_default_static_2_0));
}

void _ui_update_default_static_2_0() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_static_2_0.data[i].operate_tpyel = 2;
    }

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_static_2_0);
    SEND_MESSAGE((uint8_t *) &ui_default_static_2_0, sizeof(ui_default_static_2_0));
}

void _ui_remove_default_static_2_0() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_static_2_0.data[i].operate_tpyel = 3;
    }

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_static_2_0);
    SEND_MESSAGE((uint8_t *) &ui_default_static_2_0, sizeof(ui_default_static_2_0));
}
