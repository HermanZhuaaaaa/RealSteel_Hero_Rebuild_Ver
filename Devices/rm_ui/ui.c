#include "ui.h"
#include "cmsis_os.h"
referee_ui_data_t referee_ui_data;

void referee_ui_init()
{
    for (uint8_t i = 0; i < 30; i++)
    {
        ui_init_default_static_1();
        ui_init_default_chassis_1();
        ui_init_default_fric();
        ui_init_default_clip();
        ui_init_default_cap();
        ui_init_default_static_2();
        osDelay(100);
    }
}

void referee_ui_change(referee_ui_data_t *referee_ui_data)
{
    switch (referee_ui_data->chassis_mode)
    {
    case 0:
        ui_remove_default_chassis_1();
        break;

    case 1:
        //follow
        ui_remove_default_chassis_1();
        _ui_init_default_chassis_1_0();
        _ui_update_default_chassis_1_0();
        break;

    case 2:
        //little-top
        ui_remove_default_chassis_1();
        _ui_init_default_chassis_1_1();
        _ui_update_default_chassis_1_1();
        break;

    default:
        break;
    }

    switch (referee_ui_data->friction_on)
    {
    case 0:
        //friction_on
        ui_remove_default_fric();
        _ui_init_default_fric_0();
        _ui_update_default_fric_0();
        break;

    case 1:
        //friction_off
        ui_remove_default_fric();
        _ui_init_default_fric_1();
        _ui_update_default_fric_1();
        break;

    default:
        break;
    }

    switch (referee_ui_data->clip_on)
    {
    case 0:
        //clip_on
        ui_remove_default_clip();
        _ui_init_default_clip_0();
        _ui_update_default_clip_0();
        break;

    case 1:
        //clip_off
        ui_remove_default_clip();
        _ui_init_default_clip_1();
        _ui_update_default_clip_1();
        break;

    default:
        break;
    }

    switch (referee_ui_data->cap_capacity)
    {
    case 0:
        //cap_capacity_1
        ui_remove_default_cap();
        break;

    case 1:
        //cap_capacity_3/4
        ui_remove_default_cap();
        break;

    case 2:
        //cap_capacity_1/2
        ui_remove_default_cap();
        break;

    case 3:
        //cap_capacity_1/4
        ui_remove_default_cap();
        break;

    case 4:
        //cap_capacity_0
        ui_remove_default_cap();
        break;

    default:
        break;
    }
}

void referee_ui_test_app(uint32_t cnt)
{
    if (cnt < 10)
    {
        referee_ui_data.cap_capacity = 0;
        referee_ui_data.chassis_mode = 0;
        referee_ui_data.clip_on = 0;
        referee_ui_data.friction_on = 0;
    }
    else if (cnt > 10 && cnt < 20)
    {
        referee_ui_data.cap_capacity = 1;
        referee_ui_data.chassis_mode = 1;
        referee_ui_data.clip_on = 1;
        referee_ui_data.friction_on = 1;
    }
    else if (cnt > 20 && cnt < 30)
    {
        referee_ui_data.cap_capacity = 2;
        referee_ui_data.chassis_mode = 2;
        referee_ui_data.clip_on = 0;
        referee_ui_data.friction_on = 0;
    }
    else if (cnt > 30 && cnt < 40)
    {
        referee_ui_data.cap_capacity = 3;
        referee_ui_data.chassis_mode = 0;
        referee_ui_data.clip_on = 1;
        referee_ui_data.friction_on = 1;
    }
    else if (cnt > 40 && cnt < 50)
    {
        referee_ui_data.cap_capacity = 0;
        referee_ui_data.chassis_mode = 1;
        referee_ui_data.clip_on = 0;
        referee_ui_data.friction_on = 0;
    }
}