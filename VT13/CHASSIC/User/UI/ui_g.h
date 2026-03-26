//
// Created by RM UI Designer
// Static Edition
//

#ifndef UI_g_H
#define UI_g_H

#include "ui_interface.h"

extern ui_interface_number_t *ui_g_1_pitch_num;
extern ui_interface_number_t *ui_g_1_fire_state;
extern ui_interface_number_t *ui_g_1_shoot_state;
extern ui_interface_number_t *ui_g_1_vision_state;
extern ui_interface_number_t *ui_g_1_max_heat;
extern ui_interface_number_t *ui_g_1_now_heat;
extern ui_interface_line_t *ui_g_1_heng;
extern ui_interface_line_t *ui_g_1_shu;
extern ui_interface_round_t *ui_g_1_zhongxin;
extern ui_interface_rect_t *ui_g_1_kuang;
extern ui_interface_string_t *ui_g_1_vision;
extern ui_interface_string_t *ui_g_1_heat;
extern ui_interface_string_t *ui_g_1_NewText;
extern ui_interface_string_t *ui_g_1_fire_wheel;
extern ui_interface_string_t *ui_g_1_shoot;
extern ui_interface_string_t *ui_g_1_Pitch;
extern ui_interface_string_t *ui_g_1_xiaoxiaobu;

void ui_init_g_1();
void ui_update_g_1();
void ui_remove_g_1();


#endif // UI_g_H
