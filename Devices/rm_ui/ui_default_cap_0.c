//
// Created by RM UI Designer
//

#include "ui_default_cap_0.h"

#define FRAME_ID 0
#define GROUP_ID 4
#define START_ID 0
#define OBJ_NUM 5
#define FRAME_OBJ_NUM 5

CAT(ui_, CAT(FRAME_OBJ_NUM, _frame_t)) ui_default_cap_0;
ui_interface_line_t *ui_default_cap_cap_block_1 = (ui_interface_line_t *)&(ui_default_cap_0.data[0]);
ui_interface_line_t *ui_default_cap_cap_block_2 = (ui_interface_line_t *)&(ui_default_cap_0.data[1]);
ui_interface_line_t *ui_default_cap_cap_block_3 = (ui_interface_line_t *)&(ui_default_cap_0.data[2]);
ui_interface_line_t *ui_default_cap_cap_block_4 = (ui_interface_line_t *)&(ui_default_cap_0.data[3]);
ui_interface_line_t *ui_default_cap_cap_block_5 = (ui_interface_line_t *)&(ui_default_cap_0.data[4]);

void _ui_init_default_cap_0() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_cap_0.data[i].figure_name[0] = FRAME_ID;
        ui_default_cap_0.data[i].figure_name[1] = GROUP_ID;
        ui_default_cap_0.data[i].figure_name[2] = i + START_ID;
        ui_default_cap_0.data[i].operate_tpyel = 1;
    }
    for (int i = OBJ_NUM; i < FRAME_OBJ_NUM; i++) {
        ui_default_cap_0.data[i].operate_tpyel = 0;
    }

    ui_default_cap_cap_block_1->figure_tpye = 0;
    ui_default_cap_cap_block_1->layer = 2;
    ui_default_cap_cap_block_1->start_x = 722;
    ui_default_cap_cap_block_1->start_y = 110;
    ui_default_cap_cap_block_1->end_x = 1290;
    ui_default_cap_cap_block_1->end_y = 110;
    ui_default_cap_cap_block_1->color = 6;
    ui_default_cap_cap_block_1->width = 75;

    ui_default_cap_cap_block_2->figure_tpye = 0;
    ui_default_cap_cap_block_2->layer = 2;
    ui_default_cap_cap_block_2->start_x = 722;
    ui_default_cap_cap_block_2->start_y = 110;
    ui_default_cap_cap_block_2->end_x = 1150;
    ui_default_cap_cap_block_2->end_y = 110;
    ui_default_cap_cap_block_2->color = 6;
    ui_default_cap_cap_block_2->width = 75;

    ui_default_cap_cap_block_3->figure_tpye = 0;
    ui_default_cap_cap_block_3->layer = 2;
    ui_default_cap_cap_block_3->start_x = 722;
    ui_default_cap_cap_block_3->start_y = 110;
    ui_default_cap_cap_block_3->end_x = 1000;
    ui_default_cap_cap_block_3->end_y = 110;
    ui_default_cap_cap_block_3->color = 1;
    ui_default_cap_cap_block_3->width = 75;

    ui_default_cap_cap_block_4->figure_tpye = 0;
    ui_default_cap_cap_block_4->layer = 2;
    ui_default_cap_cap_block_4->start_x = 722;
    ui_default_cap_cap_block_4->start_y = 103;
    ui_default_cap_cap_block_4->end_x = 859;
    ui_default_cap_cap_block_4->end_y = 103;
    ui_default_cap_cap_block_4->color = 4;
    ui_default_cap_cap_block_4->width = 75;

    ui_default_cap_cap_block_5->figure_tpye = 0;
    ui_default_cap_cap_block_5->layer = 2;
    ui_default_cap_cap_block_5->start_x = 722;
    ui_default_cap_cap_block_5->start_y = 103;
    ui_default_cap_cap_block_5->end_x = 724;
    ui_default_cap_cap_block_5->end_y = 103;
    ui_default_cap_cap_block_5->color = 8;
    ui_default_cap_cap_block_5->width = 75;


    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_cap_0);
    SEND_MESSAGE((uint8_t *) &ui_default_cap_0, sizeof(ui_default_cap_0));
}

void _ui_update_default_cap_0() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_cap_0.data[i].operate_tpyel = 2;
    }

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_cap_0);
    SEND_MESSAGE((uint8_t *) &ui_default_cap_0, sizeof(ui_default_cap_0));
}

void _ui_remove_default_cap_0() {
    for (int i = 0; i < OBJ_NUM; i++) {
        ui_default_cap_0.data[i].operate_tpyel = 3;
    }

    CAT(ui_proc_, CAT(FRAME_OBJ_NUM, _frame))(&ui_default_cap_0);
    SEND_MESSAGE((uint8_t *) &ui_default_cap_0, sizeof(ui_default_cap_0));
}
