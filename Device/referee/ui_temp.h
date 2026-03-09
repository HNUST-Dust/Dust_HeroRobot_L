//
// Created by RM UI Designer
// Static Edition
//

#ifndef UI_temp_H
#define UI_temp_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ui_interface.h"

extern ui_interface_string_t *ui_temp_Ungroup_reload;
extern ui_interface_string_t *ui_temp_Ungroup_shoot;
extern ui_interface_string_t *ui_temp_Ungroup_reload_flag;
extern ui_interface_string_t *ui_temp_Ungroup_shoot_flag;
extern ui_interface_string_t *ui_temp_Ungroup_chassis_mode;
extern ui_interface_string_t *ui_temp_Ungroup_chassis_mode_flag;

void ui_init_temp_Ungroup();
void ui_update_temp_Ungroup();
void ui_remove_temp_Ungroup();

#ifdef __cplusplus
}
#endif

#endif // UI_temp_H
