//
// Created by RM UI Designer
// Static Edition
//

#include <string.h>

#include "ui_interface.h"


ui_string_frame_t ui_temp_Ungroup_0;
ui_interface_string_t* ui_temp_Ungroup_reload = &(ui_temp_Ungroup_0.option);

void _ui_init_temp_Ungroup_0() {
    ui_temp_Ungroup_0.option.figure_name[0] = 1;
    ui_temp_Ungroup_0.option.figure_name[1] = 0;
    ui_temp_Ungroup_0.option.figure_name[2] = 0;
    ui_temp_Ungroup_0.option.operate_type = 1;

    ui_temp_Ungroup_reload->figure_type = 7;
    ui_temp_Ungroup_reload->operate_type = 1;
    ui_temp_Ungroup_reload->layer = 0;
    ui_temp_Ungroup_reload->color = 7;
    ui_temp_Ungroup_reload->start_x = 60;
    ui_temp_Ungroup_reload->start_y = 300;
    ui_temp_Ungroup_reload->width = 2;
    ui_temp_Ungroup_reload->font_size = 20;
    ui_temp_Ungroup_reload->str_length = 9;
    strcpy(ui_temp_Ungroup_reload->string, "拨弹盘");


    ui_proc_string_frame(&ui_temp_Ungroup_0);
    SEND_MESSAGE((uint8_t *) &ui_temp_Ungroup_0, sizeof(ui_temp_Ungroup_0));
}

void _ui_update_temp_Ungroup_0() {
    ui_temp_Ungroup_0.option.operate_type = 2;

    ui_proc_string_frame(&ui_temp_Ungroup_0);
    SEND_MESSAGE((uint8_t *) &ui_temp_Ungroup_0, sizeof(ui_temp_Ungroup_0));
}

void _ui_remove_temp_Ungroup_0() {
    ui_temp_Ungroup_0.option.operate_type = 3;

    ui_proc_string_frame(&ui_temp_Ungroup_0);
    SEND_MESSAGE((uint8_t *) &ui_temp_Ungroup_0, sizeof(ui_temp_Ungroup_0));
}
ui_string_frame_t ui_temp_Ungroup_1;
ui_interface_string_t* ui_temp_Ungroup_shoot = &(ui_temp_Ungroup_1.option);

void _ui_init_temp_Ungroup_1() {
    ui_temp_Ungroup_1.option.figure_name[0] = 1;
    ui_temp_Ungroup_1.option.figure_name[1] = 0;
    ui_temp_Ungroup_1.option.figure_name[2] = 1;
    ui_temp_Ungroup_1.option.operate_type = 1;

    ui_temp_Ungroup_shoot->figure_type = 7;
    ui_temp_Ungroup_shoot->operate_type = 1;
    ui_temp_Ungroup_shoot->layer = 0;
    ui_temp_Ungroup_shoot->color = 7;
    ui_temp_Ungroup_shoot->start_x = 60;
    ui_temp_Ungroup_shoot->start_y = 260;
    ui_temp_Ungroup_shoot->width = 2;
    ui_temp_Ungroup_shoot->font_size = 20;
    ui_temp_Ungroup_shoot->str_length = 9;
    strcpy(ui_temp_Ungroup_shoot->string, "摩擦轮");


    ui_proc_string_frame(&ui_temp_Ungroup_1);
    SEND_MESSAGE((uint8_t *) &ui_temp_Ungroup_1, sizeof(ui_temp_Ungroup_1));
}

void _ui_update_temp_Ungroup_1() {
    ui_temp_Ungroup_1.option.operate_type = 2;

    ui_proc_string_frame(&ui_temp_Ungroup_1);
    SEND_MESSAGE((uint8_t *) &ui_temp_Ungroup_1, sizeof(ui_temp_Ungroup_1));
}

void _ui_remove_temp_Ungroup_1() {
    ui_temp_Ungroup_1.option.operate_type = 3;

    ui_proc_string_frame(&ui_temp_Ungroup_1);
    SEND_MESSAGE((uint8_t *) &ui_temp_Ungroup_1, sizeof(ui_temp_Ungroup_1));
}
ui_string_frame_t ui_temp_Ungroup_2;
ui_interface_string_t* ui_temp_Ungroup_reload_flag = &(ui_temp_Ungroup_2.option);

void _ui_init_temp_Ungroup_2() {
    ui_temp_Ungroup_2.option.figure_name[0] = 1;
    ui_temp_Ungroup_2.option.figure_name[1] = 0;
    ui_temp_Ungroup_2.option.figure_name[2] = 2;
    ui_temp_Ungroup_2.option.operate_type = 1;

    ui_temp_Ungroup_reload_flag->figure_type = 7;
    ui_temp_Ungroup_reload_flag->operate_type = 1;
    ui_temp_Ungroup_reload_flag->layer = 0;
    ui_temp_Ungroup_reload_flag->color = 0;
    ui_temp_Ungroup_reload_flag->start_x = 150;
    ui_temp_Ungroup_reload_flag->start_y = 300;
    ui_temp_Ungroup_reload_flag->width = 2;
    ui_temp_Ungroup_reload_flag->font_size = 20;
    ui_temp_Ungroup_reload_flag->str_length = 1;
    strcpy(ui_temp_Ungroup_reload_flag->string, "0");


    ui_proc_string_frame(&ui_temp_Ungroup_2);
    SEND_MESSAGE((uint8_t *) &ui_temp_Ungroup_2, sizeof(ui_temp_Ungroup_2));
}

void _ui_update_temp_Ungroup_2() {
    ui_temp_Ungroup_2.option.operate_type = 2;

    ui_proc_string_frame(&ui_temp_Ungroup_2);
    SEND_MESSAGE((uint8_t *) &ui_temp_Ungroup_2, sizeof(ui_temp_Ungroup_2));
}

void _ui_remove_temp_Ungroup_2() {
    ui_temp_Ungroup_2.option.operate_type = 3;

    ui_proc_string_frame(&ui_temp_Ungroup_2);
    SEND_MESSAGE((uint8_t *) &ui_temp_Ungroup_2, sizeof(ui_temp_Ungroup_2));
}
ui_string_frame_t ui_temp_Ungroup_3;
ui_interface_string_t* ui_temp_Ungroup_shoot_flag = &(ui_temp_Ungroup_3.option);

void _ui_init_temp_Ungroup_3() {
    ui_temp_Ungroup_3.option.figure_name[0] = 1;
    ui_temp_Ungroup_3.option.figure_name[1] = 0;
    ui_temp_Ungroup_3.option.figure_name[2] = 3;
    ui_temp_Ungroup_3.option.operate_type = 1;

    ui_temp_Ungroup_shoot_flag->figure_type = 7;
    ui_temp_Ungroup_shoot_flag->operate_type = 1;
    ui_temp_Ungroup_shoot_flag->layer = 0;
    ui_temp_Ungroup_shoot_flag->color = 0;
    ui_temp_Ungroup_shoot_flag->start_x = 150;
    ui_temp_Ungroup_shoot_flag->start_y = 260;
    ui_temp_Ungroup_shoot_flag->width = 2;
    ui_temp_Ungroup_shoot_flag->font_size = 20;
    ui_temp_Ungroup_shoot_flag->str_length = 1;
    strcpy(ui_temp_Ungroup_shoot_flag->string, "1");


    ui_proc_string_frame(&ui_temp_Ungroup_3);
    SEND_MESSAGE((uint8_t *) &ui_temp_Ungroup_3, sizeof(ui_temp_Ungroup_3));
}

void _ui_update_temp_Ungroup_3() {
    ui_temp_Ungroup_3.option.operate_type = 2;

    ui_proc_string_frame(&ui_temp_Ungroup_3);
    SEND_MESSAGE((uint8_t *) &ui_temp_Ungroup_3, sizeof(ui_temp_Ungroup_3));
}

void _ui_remove_temp_Ungroup_3() {
    ui_temp_Ungroup_3.option.operate_type = 3;

    ui_proc_string_frame(&ui_temp_Ungroup_3);
    SEND_MESSAGE((uint8_t *) &ui_temp_Ungroup_3, sizeof(ui_temp_Ungroup_3));
}
ui_string_frame_t ui_temp_Ungroup_4;
ui_interface_string_t* ui_temp_Ungroup_chassis_mode = &(ui_temp_Ungroup_4.option);

void _ui_init_temp_Ungroup_4() {
    ui_temp_Ungroup_4.option.figure_name[0] = 1;
    ui_temp_Ungroup_4.option.figure_name[1] = 0;
    ui_temp_Ungroup_4.option.figure_name[2] = 4;
    ui_temp_Ungroup_4.option.operate_type = 1;

    ui_temp_Ungroup_chassis_mode->figure_type = 7;
    ui_temp_Ungroup_chassis_mode->operate_type = 1;
    ui_temp_Ungroup_chassis_mode->layer = 0;
    ui_temp_Ungroup_chassis_mode->color = 7;
    ui_temp_Ungroup_chassis_mode->start_x = 60;
    ui_temp_Ungroup_chassis_mode->start_y = 340;
    ui_temp_Ungroup_chassis_mode->width = 2;
    ui_temp_Ungroup_chassis_mode->font_size = 20;
    ui_temp_Ungroup_chassis_mode->str_length = 12;
    strcpy(ui_temp_Ungroup_chassis_mode->string, "底盘模式");


    ui_proc_string_frame(&ui_temp_Ungroup_4);
    SEND_MESSAGE((uint8_t *) &ui_temp_Ungroup_4, sizeof(ui_temp_Ungroup_4));
}

void _ui_update_temp_Ungroup_4() {
    ui_temp_Ungroup_4.option.operate_type = 2;

    ui_proc_string_frame(&ui_temp_Ungroup_4);
    SEND_MESSAGE((uint8_t *) &ui_temp_Ungroup_4, sizeof(ui_temp_Ungroup_4));
}

void _ui_remove_temp_Ungroup_4() {
    ui_temp_Ungroup_4.option.operate_type = 3;

    ui_proc_string_frame(&ui_temp_Ungroup_4);
    SEND_MESSAGE((uint8_t *) &ui_temp_Ungroup_4, sizeof(ui_temp_Ungroup_4));
}
ui_string_frame_t ui_temp_Ungroup_5;
ui_interface_string_t* ui_temp_Ungroup_chassis_mode_flag = &(ui_temp_Ungroup_5.option);

void _ui_init_temp_Ungroup_5() {
    ui_temp_Ungroup_5.option.figure_name[0] = 1;
    ui_temp_Ungroup_5.option.figure_name[1] = 0;
    ui_temp_Ungroup_5.option.figure_name[2] = 5;
    ui_temp_Ungroup_5.option.operate_type = 1;

    ui_temp_Ungroup_chassis_mode_flag->figure_type = 7;
    ui_temp_Ungroup_chassis_mode_flag->operate_type = 1;
    ui_temp_Ungroup_chassis_mode_flag->layer = 0;
    ui_temp_Ungroup_chassis_mode_flag->color = 0;
    ui_temp_Ungroup_chassis_mode_flag->start_x = 175;
    ui_temp_Ungroup_chassis_mode_flag->start_y = 340;
    ui_temp_Ungroup_chassis_mode_flag->width = 2;
    ui_temp_Ungroup_chassis_mode_flag->font_size = 20;
    ui_temp_Ungroup_chassis_mode_flag->str_length = 6;
    strcpy(ui_temp_Ungroup_chassis_mode_flag->string, "normal");


    ui_proc_string_frame(&ui_temp_Ungroup_5);
    SEND_MESSAGE((uint8_t *) &ui_temp_Ungroup_5, sizeof(ui_temp_Ungroup_5));
}

void _ui_update_temp_Ungroup_5() {
    ui_temp_Ungroup_5.option.operate_type = 2;

    ui_proc_string_frame(&ui_temp_Ungroup_5);
    SEND_MESSAGE((uint8_t *) &ui_temp_Ungroup_5, sizeof(ui_temp_Ungroup_5));
}

void _ui_remove_temp_Ungroup_5() {
    ui_temp_Ungroup_5.option.operate_type = 3;

    ui_proc_string_frame(&ui_temp_Ungroup_5);
    SEND_MESSAGE((uint8_t *) &ui_temp_Ungroup_5, sizeof(ui_temp_Ungroup_5));
}

void ui_init_temp_Ungroup() 
{
    _ui_init_temp_Ungroup_0();
    _ui_init_temp_Ungroup_1();
    _ui_init_temp_Ungroup_2();
    _ui_init_temp_Ungroup_3();
    _ui_init_temp_Ungroup_4();
    _ui_init_temp_Ungroup_5();
}

void ui_update_temp_Ungroup() {
    _ui_update_temp_Ungroup_0();
    _ui_update_temp_Ungroup_1();
    _ui_update_temp_Ungroup_2();
    _ui_update_temp_Ungroup_3();
    _ui_update_temp_Ungroup_4();
    _ui_update_temp_Ungroup_5();
}

void ui_remove_temp_Ungroup() {
    _ui_remove_temp_Ungroup_0();
    _ui_remove_temp_Ungroup_1();
    _ui_remove_temp_Ungroup_2();
    _ui_remove_temp_Ungroup_3();
    _ui_remove_temp_Ungroup_4();
    _ui_remove_temp_Ungroup_5();
}

