//
// Created by RM UI Designer
// Static Edition
//

#include <string.h>

#include "ui_interface.h"

ui_2_frame_t ui_g_Ungroup_0;

ui_interface_line_t *ui_g_Ungroup_chassis_l = (ui_interface_line_t*)&(ui_g_Ungroup_0.data[0]);
ui_interface_line_t *ui_g_Ungroup_chassis_r = (ui_interface_line_t*)&(ui_g_Ungroup_0.data[1]);

void _ui_init_g_Ungroup_0() {
    for (int i = 0; i < 2; i++) {
        ui_g_Ungroup_0.data[i].figure_name[0] = 0;
        ui_g_Ungroup_0.data[i].figure_name[1] = 0;
        ui_g_Ungroup_0.data[i].figure_name[2] = i + 0;
        ui_g_Ungroup_0.data[i].operate_type = 1;
    }
    for (int i = 2; i < 2; i++) {
        ui_g_Ungroup_0.data[i].operate_type = 0;
    }

    ui_g_Ungroup_chassis_l->figure_type = 0;
    ui_g_Ungroup_chassis_l->operate_type = 1;
    ui_g_Ungroup_chassis_l->layer = 0;
    ui_g_Ungroup_chassis_l->color = 4;
    ui_g_Ungroup_chassis_l->start_x = 0;
    ui_g_Ungroup_chassis_l->start_y = 499;
    ui_g_Ungroup_chassis_l->width = 3;
    ui_g_Ungroup_chassis_l->end_x = 820;
    ui_g_Ungroup_chassis_l->end_y = 646;

    ui_g_Ungroup_chassis_r->figure_type = 0;
    ui_g_Ungroup_chassis_r->operate_type = 1;
    ui_g_Ungroup_chassis_r->layer = 0;
    ui_g_Ungroup_chassis_r->color = 4;
    ui_g_Ungroup_chassis_r->start_x = 1916;
    ui_g_Ungroup_chassis_r->start_y = 500;
    ui_g_Ungroup_chassis_r->width = 3;
    ui_g_Ungroup_chassis_r->end_x = 1100;
    ui_g_Ungroup_chassis_r->end_y = 645;


    ui_proc_2_frame(&ui_g_Ungroup_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_0, sizeof(ui_g_Ungroup_0));
}

void _ui_update_g_Ungroup_0() {
    for (int i = 0; i < 2; i++) {
        ui_g_Ungroup_0.data[i].operate_type = 2;
    }

    ui_proc_2_frame(&ui_g_Ungroup_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_0, sizeof(ui_g_Ungroup_0));
}

void _ui_remove_g_Ungroup_0() {
    for (int i = 0; i < 2; i++) {
        ui_g_Ungroup_0.data[i].operate_type = 3;
    }

    ui_proc_2_frame(&ui_g_Ungroup_0);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_0, sizeof(ui_g_Ungroup_0));
}

ui_string_frame_t ui_g_Ungroup_1;
ui_interface_string_t* ui_g_Ungroup_reload = &(ui_g_Ungroup_1.option);

void _ui_init_g_Ungroup_1() {
    ui_g_Ungroup_1.option.figure_name[0] = 0;
    ui_g_Ungroup_1.option.figure_name[1] = 0;
    ui_g_Ungroup_1.option.figure_name[2] = 2;
    ui_g_Ungroup_1.option.operate_type = 1;

    ui_g_Ungroup_reload->figure_type = 7;
    ui_g_Ungroup_reload->operate_type = 1;
    ui_g_Ungroup_reload->layer = 0;
    ui_g_Ungroup_reload->color = 7;
    ui_g_Ungroup_reload->start_x = 110;
    ui_g_Ungroup_reload->start_y = 760;
    ui_g_Ungroup_reload->width = 2;
    ui_g_Ungroup_reload->font_size = 20;
    ui_g_Ungroup_reload->str_length = 1;
    strcpy(ui_g_Ungroup_reload->string, "R");


    ui_proc_string_frame(&ui_g_Ungroup_1);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_1, sizeof(ui_g_Ungroup_1));
}

void _ui_update_g_Ungroup_1() {
    ui_g_Ungroup_1.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_Ungroup_1);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_1, sizeof(ui_g_Ungroup_1));
}

void _ui_remove_g_Ungroup_1() {
    ui_g_Ungroup_1.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_Ungroup_1);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_1, sizeof(ui_g_Ungroup_1));
}
ui_string_frame_t ui_g_Ungroup_2;
ui_interface_string_t* ui_g_Ungroup_shoot = &(ui_g_Ungroup_2.option);

void _ui_init_g_Ungroup_2() {
    ui_g_Ungroup_2.option.figure_name[0] = 0;
    ui_g_Ungroup_2.option.figure_name[1] = 0;
    ui_g_Ungroup_2.option.figure_name[2] = 3;
    ui_g_Ungroup_2.option.operate_type = 1;

    ui_g_Ungroup_shoot->figure_type = 7;
    ui_g_Ungroup_shoot->operate_type = 1;
    ui_g_Ungroup_shoot->layer = 0;
    ui_g_Ungroup_shoot->color = 7;
    ui_g_Ungroup_shoot->start_x = 110;
    ui_g_Ungroup_shoot->start_y = 720;
    ui_g_Ungroup_shoot->width = 2;
    ui_g_Ungroup_shoot->font_size = 20;
    ui_g_Ungroup_shoot->str_length = 1;
    strcpy(ui_g_Ungroup_shoot->string, "S");


    ui_proc_string_frame(&ui_g_Ungroup_2);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_2, sizeof(ui_g_Ungroup_2));
}

void _ui_update_g_Ungroup_2() {
    ui_g_Ungroup_2.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_Ungroup_2);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_2, sizeof(ui_g_Ungroup_2));
}

void _ui_remove_g_Ungroup_2() {
    ui_g_Ungroup_2.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_Ungroup_2);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_2, sizeof(ui_g_Ungroup_2));
}
ui_string_frame_t ui_g_Ungroup_3;
ui_interface_string_t* ui_g_Ungroup_reload_flag = &(ui_g_Ungroup_3.option);

void _ui_init_g_Ungroup_3() {
    ui_g_Ungroup_3.option.figure_name[0] = 0;
    ui_g_Ungroup_3.option.figure_name[1] = 0;
    ui_g_Ungroup_3.option.figure_name[2] = 4;
    ui_g_Ungroup_3.option.operate_type = 1;

    ui_g_Ungroup_reload_flag->figure_type = 7;
    ui_g_Ungroup_reload_flag->operate_type = 1;
    ui_g_Ungroup_reload_flag->layer = 0;
    ui_g_Ungroup_reload_flag->color = 3;
    ui_g_Ungroup_reload_flag->start_x = 150;
    ui_g_Ungroup_reload_flag->start_y = 760;
    ui_g_Ungroup_reload_flag->width = 2;
    ui_g_Ungroup_reload_flag->font_size = 20;
    ui_g_Ungroup_reload_flag->str_length = 1;

    strcpy(ui_g_Ungroup_reload_flag->string, "0");

    ui_proc_string_frame(&ui_g_Ungroup_3);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_3, sizeof(ui_g_Ungroup_3));
}

void _ui_update_g_Ungroup_3() {
    ui_g_Ungroup_3.option.operate_type = 2;

    if (ui_all_flag.reload) {
        strcpy(ui_g_Ungroup_reload_flag->string, "1");
    } else {
        strcpy(ui_g_Ungroup_reload_flag->string, "0");
    }

    ui_proc_string_frame(&ui_g_Ungroup_3);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_3, sizeof(ui_g_Ungroup_3));
}

void _ui_remove_g_Ungroup_3() {
    ui_g_Ungroup_3.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_Ungroup_3);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_3, sizeof(ui_g_Ungroup_3));
}
ui_string_frame_t ui_g_Ungroup_4;
ui_interface_string_t* ui_g_Ungroup_shoot_flag = &(ui_g_Ungroup_4.option);

void _ui_init_g_Ungroup_4() {
    ui_g_Ungroup_4.option.figure_name[0] = 0;
    ui_g_Ungroup_4.option.figure_name[1] = 0;
    ui_g_Ungroup_4.option.figure_name[2] = 5;
    ui_g_Ungroup_4.option.operate_type = 1;

    ui_g_Ungroup_shoot_flag->figure_type = 7;
    ui_g_Ungroup_shoot_flag->operate_type = 1;
    ui_g_Ungroup_shoot_flag->layer = 0;
    ui_g_Ungroup_shoot_flag->color = 3;
    ui_g_Ungroup_shoot_flag->start_x = 150;
    ui_g_Ungroup_shoot_flag->start_y = 720;
    ui_g_Ungroup_shoot_flag->width = 2;
    ui_g_Ungroup_shoot_flag->font_size = 20;
    ui_g_Ungroup_shoot_flag->str_length = 1;

    strcpy(ui_g_Ungroup_shoot_flag->string, "0");

    ui_proc_string_frame(&ui_g_Ungroup_4);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_4, sizeof(ui_g_Ungroup_4));
}

void _ui_update_g_Ungroup_4() {
    ui_g_Ungroup_4.option.operate_type = 2;

    if (ui_all_flag.shoot) {
        strcpy(ui_g_Ungroup_shoot_flag->string, "1");
    } else {
        strcpy(ui_g_Ungroup_shoot_flag->string, "0");
    }

    ui_proc_string_frame(&ui_g_Ungroup_4);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_4, sizeof(ui_g_Ungroup_4));
}

void _ui_remove_g_Ungroup_4() {
    ui_g_Ungroup_4.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_Ungroup_4);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_4, sizeof(ui_g_Ungroup_4));
}
ui_string_frame_t ui_g_Ungroup_5;
ui_interface_string_t* ui_g_Ungroup_chassis_mode = &(ui_g_Ungroup_5.option);

void _ui_init_g_Ungroup_5() {
    ui_g_Ungroup_5.option.figure_name[0] = 0;
    ui_g_Ungroup_5.option.figure_name[1] = 0;
    ui_g_Ungroup_5.option.figure_name[2] = 6;
    ui_g_Ungroup_5.option.operate_type = 1;

    ui_g_Ungroup_chassis_mode->figure_type = 7;
    ui_g_Ungroup_chassis_mode->operate_type = 1;
    ui_g_Ungroup_chassis_mode->layer = 0;
    ui_g_Ungroup_chassis_mode->color = 7;
    ui_g_Ungroup_chassis_mode->start_x = 110;
    ui_g_Ungroup_chassis_mode->start_y = 800;
    ui_g_Ungroup_chassis_mode->width = 2;
    ui_g_Ungroup_chassis_mode->font_size = 20;
    ui_g_Ungroup_chassis_mode->str_length = 1;
    strcpy(ui_g_Ungroup_chassis_mode->string, "C");


    ui_proc_string_frame(&ui_g_Ungroup_5);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_5, sizeof(ui_g_Ungroup_5));
}

void _ui_update_g_Ungroup_5() {
    ui_g_Ungroup_5.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_Ungroup_5);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_5, sizeof(ui_g_Ungroup_5));
}

void _ui_remove_g_Ungroup_5() {
    ui_g_Ungroup_5.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_Ungroup_5);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_5, sizeof(ui_g_Ungroup_5));
}
ui_string_frame_t ui_g_Ungroup_6;
ui_interface_string_t* ui_g_Ungroup_chassis_mode_flag = &(ui_g_Ungroup_6.option);

void _ui_init_g_Ungroup_6() {
    ui_g_Ungroup_6.option.figure_name[0] = 0;
    ui_g_Ungroup_6.option.figure_name[1] = 0;
    ui_g_Ungroup_6.option.figure_name[2] = 7;
    ui_g_Ungroup_6.option.operate_type = 1;

    ui_g_Ungroup_chassis_mode_flag->figure_type = 7;
    ui_g_Ungroup_chassis_mode_flag->operate_type = 1;
    ui_g_Ungroup_chassis_mode_flag->layer = 0;
    ui_g_Ungroup_chassis_mode_flag->color = 3;
    ui_g_Ungroup_chassis_mode_flag->start_x = 150;
    ui_g_Ungroup_chassis_mode_flag->start_y = 800;
    ui_g_Ungroup_chassis_mode_flag->width = 2;
    ui_g_Ungroup_chassis_mode_flag->font_size = 20;
    ui_g_Ungroup_chassis_mode_flag->str_length = 6;

    strcpy(ui_g_Ungroup_chassis_mode_flag->string, "normal");

    ui_proc_string_frame(&ui_g_Ungroup_6);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_6, sizeof(ui_g_Ungroup_6));
}

void _ui_update_g_Ungroup_6() {
    ui_g_Ungroup_6.option.operate_type = 2;

    char chassis_flag[8] = "normal";

    switch (ui_all_flag.chassis)
    {
        case 1:
        {
            strcpy(chassis_flag, "spin   ");
            break;
        }
        case 2:
        {
            strcpy(chassis_flag, "normal");
            break;
        }
        case 3:
        {
            strcpy(chassis_flag, "followh");
            break;
        }
        case 4:
        {
            strcpy(chassis_flag, "followb");
            break;
        }
    }
    strcpy(ui_g_Ungroup_chassis_mode_flag->string, chassis_flag);

    ui_proc_string_frame(&ui_g_Ungroup_6);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_6, sizeof(ui_g_Ungroup_6));
}

void _ui_remove_g_Ungroup_6() {
    ui_g_Ungroup_6.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_Ungroup_6);
    SEND_MESSAGE((uint8_t *) &ui_g_Ungroup_6, sizeof(ui_g_Ungroup_6));
}

void ui_init_g_Ungroup() {
    _ui_init_g_Ungroup_0();
    _ui_init_g_Ungroup_1();
    _ui_init_g_Ungroup_2();
    _ui_init_g_Ungroup_3();
    _ui_init_g_Ungroup_4();
    _ui_init_g_Ungroup_5();
    _ui_init_g_Ungroup_6();
}

void ui_update_g_Ungroup() {
    _ui_update_g_Ungroup_0();
    _ui_update_g_Ungroup_1();
    _ui_update_g_Ungroup_2();
    _ui_update_g_Ungroup_3();
    _ui_update_g_Ungroup_4();
    _ui_update_g_Ungroup_5();
    _ui_update_g_Ungroup_6();
}

void ui_remove_g_Ungroup() {
    _ui_remove_g_Ungroup_0();
    _ui_remove_g_Ungroup_1();
    _ui_remove_g_Ungroup_2();
    _ui_remove_g_Ungroup_3();
    _ui_remove_g_Ungroup_4();
    _ui_remove_g_Ungroup_5();
    _ui_remove_g_Ungroup_6();
}

