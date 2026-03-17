//
// Created by RM UI Designer
// Static Edition
//

#ifndef UI_g_H
#define UI_g_H

#include "ui_interface.h"

extern ui_interface_line_t *ui_g_Ungroup_chassis_l;
extern ui_interface_line_t *ui_g_Ungroup_chassis_r;
extern ui_interface_string_t *ui_g_Ungroup_reload;
extern ui_interface_string_t *ui_g_Ungroup_shoot;
extern ui_interface_string_t *ui_g_Ungroup_reload_flag;
extern ui_interface_string_t *ui_g_Ungroup_shoot_flag;
extern ui_interface_string_t *ui_g_Ungroup_chassis_mode;
extern ui_interface_string_t *ui_g_Ungroup_chassis_mode_flag;

void ui_init_g_Ungroup();
void ui_update_g_Ungroup();
void ui_remove_g_Ungroup();


#endif // UI_g_H
