//
// Created by RM UI Designer
//

#ifndef UI_H
#define UI_H
#ifdef __cplusplus
extern "C" {
#endif

#include "ui_interface.h"

#include "ui_default_static_1_0.h"
#include "ui_default_static_1_1.h"
#include "ui_default_static_1_2.h"
#include "ui_default_static_1_3.h"
#include "ui_default_static_1_4.h"

#define ui_init_default_static_1() \
_ui_init_default_static_1_0(); \
_ui_init_default_static_1_1(); \
_ui_init_default_static_1_2(); \
_ui_init_default_static_1_3(); \
_ui_init_default_static_1_4()

#define ui_update_default_static_1() \
_ui_update_default_static_1_0(); \
_ui_update_default_static_1_1(); \
_ui_update_default_static_1_2(); \
_ui_update_default_static_1_3(); \
_ui_update_default_static_1_4()

#define ui_remove_default_static_1() \
_ui_remove_default_static_1_0(); \
_ui_remove_default_static_1_1(); \
_ui_remove_default_static_1_2(); \
_ui_remove_default_static_1_3(); \
_ui_remove_default_static_1_4()

#include "ui_default_chassis_1_0.h"
#include "ui_default_chassis_1_1.h"

#define ui_init_default_chassis_1() \
_ui_init_default_chassis_1_0(); \
_ui_init_default_chassis_1_1()

#define ui_update_default_chassis_1() \
_ui_update_default_chassis_1_0(); \
_ui_update_default_chassis_1_1()

#define ui_remove_default_chassis_1() \
_ui_remove_default_chassis_1_0(); \
_ui_remove_default_chassis_1_1()
    

#include "ui_default_fric_0.h"
#include "ui_default_fric_1.h"

#define ui_init_default_fric() \
_ui_init_default_fric_0(); \
_ui_init_default_fric_1()

#define ui_update_default_fric() \
_ui_update_default_fric_0(); \
_ui_update_default_fric_1()

#define ui_remove_default_fric() \
_ui_remove_default_fric_0(); \
_ui_remove_default_fric_1()
    

#include "ui_default_clip_0.h"
#include "ui_default_clip_1.h"

#define ui_init_default_clip() \
_ui_init_default_clip_0(); \
_ui_init_default_clip_1()

#define ui_update_default_clip() \
_ui_update_default_clip_0(); \
_ui_update_default_clip_1()

#define ui_remove_default_clip() \
_ui_remove_default_clip_0(); \
_ui_remove_default_clip_1()
    

#include "ui_default_cap_0.h"

#define ui_init_default_cap() \
_ui_init_default_cap_0()

#define ui_update_default_cap() \
_ui_update_default_cap_0()

#define ui_remove_default_cap() \
_ui_remove_default_cap_0()
    

#include "ui_default_static_2_0.h"

#define ui_init_default_static_2() \
_ui_init_default_static_2_0()

#define ui_update_default_static_2() \
_ui_update_default_static_2_0()

#define ui_remove_default_static_2() \
_ui_remove_default_static_2_0()
    
typedef struct 
{
	uint8_t chassis_mode;
	uint8_t friction_on;
	uint8_t clip_on;
	uint8_t cap_capacity;
}referee_ui_data_t;


void referee_ui_init();
void referee_ui_change(referee_ui_data_t *referee_ui_data);
void referee_ui_test_app(uint32_t cnt);


#ifdef __cplusplus
}
#endif

#endif //UI_H
