//
// Created by RM UI Designer
//

#include "ui_default_chassis_1_1.h"
#include "string.h"

#define FRAME_ID 0
#define GROUP_ID 1
#define START_ID 1

ui_string_frame_t ui_default_chassis_1_1;

ui_interface_string_t* ui_default_chassis_1_chassis_mode_2 = &ui_default_chassis_1_1.option;

void _ui_init_default_chassis_1_1() {
    ui_default_chassis_1_1.option.figure_name[0] = FRAME_ID;
    ui_default_chassis_1_1.option.figure_name[1] = GROUP_ID;
    ui_default_chassis_1_1.option.figure_name[2] = START_ID;
    ui_default_chassis_1_1.option.operate_tpyel = 1;
    ui_default_chassis_1_1.option.figure_tpye = 7;
    ui_default_chassis_1_1.option.layer = 1;
    ui_default_chassis_1_1.option.font_size = 40;
    ui_default_chassis_1_1.option.start_x = 300;
    ui_default_chassis_1_1.option.start_y = 890;
    ui_default_chassis_1_1.option.color = 6;
    ui_default_chassis_1_1.option.str_length = 7;
    ui_default_chassis_1_1.option.width = 4;
    strcpy(ui_default_chassis_1_chassis_mode_2->string, "LIT-TOP");

    ui_proc_string_frame(&ui_default_chassis_1_1);
    SEND_MESSAGE((uint8_t *) &ui_default_chassis_1_1, sizeof(ui_default_chassis_1_1));
}

void _ui_update_default_chassis_1_1() {
    ui_default_chassis_1_1.option.operate_tpyel = 2;

    ui_proc_string_frame(&ui_default_chassis_1_1);
    SEND_MESSAGE((uint8_t *) &ui_default_chassis_1_1, sizeof(ui_default_chassis_1_1));
}

void _ui_remove_default_chassis_1_1() {
    ui_default_chassis_1_1.option.operate_tpyel = 3;

    ui_proc_string_frame(&ui_default_chassis_1_1);
    SEND_MESSAGE((uint8_t *) &ui_default_chassis_1_1, sizeof(ui_default_chassis_1_1));
}