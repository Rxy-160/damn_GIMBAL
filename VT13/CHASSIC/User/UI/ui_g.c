//
// Created by RM UI Designer
// Static Edition
//

#include <string.h>

#include "ui_interface.h"

ui_7_frame_t ui_g_1_0;

ui_interface_number_t *ui_g_1_pitch_num = (ui_interface_number_t*)&(ui_g_1_0.data[0]);
ui_interface_number_t *ui_g_1_fire_state = (ui_interface_number_t*)&(ui_g_1_0.data[1]);
ui_interface_number_t *ui_g_1_shoot_state = (ui_interface_number_t*)&(ui_g_1_0.data[2]);
ui_interface_number_t *ui_g_1_vision_state = (ui_interface_number_t*)&(ui_g_1_0.data[3]);
ui_interface_number_t *ui_g_1_max_heat = (ui_interface_number_t*)&(ui_g_1_0.data[4]);
ui_interface_number_t *ui_g_1_now_heat = (ui_interface_number_t*)&(ui_g_1_0.data[5]);
ui_interface_line_t *ui_g_1_heng = (ui_interface_line_t*)&(ui_g_1_0.data[6]);

void _ui_init_g_1_0() {
    for (int i = 0; i < 7; i++) {
        ui_g_1_0.data[i].figure_name[0] = 0;
        ui_g_1_0.data[i].figure_name[1] = 0;
        ui_g_1_0.data[i].figure_name[2] = i + 0;
        ui_g_1_0.data[i].operate_type = 1;
    }
    for (int i = 7; i < 7; i++) {
        ui_g_1_0.data[i].operate_type = 0;
    }

    ui_g_1_pitch_num->figure_type = 5;
    ui_g_1_pitch_num->operate_type = 1;
    ui_g_1_pitch_num->layer = 0;
    ui_g_1_pitch_num->color = 4;
    ui_g_1_pitch_num->start_x = 58;
    ui_g_1_pitch_num->start_y = 750;
    ui_g_1_pitch_num->width = 4;
    ui_g_1_pitch_num->font_size = 40;
    ui_g_1_pitch_num->number = pitch_state;

    ui_g_1_fire_state->figure_type = 6;
    ui_g_1_fire_state->operate_type = 1;
    ui_g_1_fire_state->layer = 0;
    ui_g_1_fire_state->color = 1;
    ui_g_1_fire_state->start_x = 398;
    ui_g_1_fire_state->start_y = 498;
    ui_g_1_fire_state->width = 4;
    ui_g_1_fire_state->font_size = 35;
    ui_g_1_fire_state->number = fire_wheel_state;

    ui_g_1_shoot_state->figure_type = 6;
    ui_g_1_shoot_state->operate_type = 1;
    ui_g_1_shoot_state->layer = 0;
    ui_g_1_shoot_state->color = 6;
    ui_g_1_shoot_state->start_x = 240;
    ui_g_1_shoot_state->start_y = 402;
    ui_g_1_shoot_state->width = 4;
    ui_g_1_shoot_state->font_size = 35;
    ui_g_1_shoot_state->number = shoot_sta;

    ui_g_1_vision_state->figure_type = 6;
    ui_g_1_vision_state->operate_type = 1;
    ui_g_1_vision_state->layer = 0;
    ui_g_1_vision_state->color = 2;
    ui_g_1_vision_state->start_x = 268;
    ui_g_1_vision_state->start_y = 302;
    ui_g_1_vision_state->width = 4;
    ui_g_1_vision_state->font_size = 35;
    ui_g_1_vision_state->number = vision_state;

    ui_g_1_max_heat->figure_type = 6;
    ui_g_1_max_heat->operate_type = 1;
    ui_g_1_max_heat->layer = 0;
    ui_g_1_max_heat->color = 7;
    ui_g_1_max_heat->start_x = 276;
    ui_g_1_max_heat->start_y = 584;
    ui_g_1_max_heat->width = 3;
    ui_g_1_max_heat->font_size = 30;
    ui_g_1_max_heat->number =User_data.robot_status . shooter_barrel_heat_limit;

    ui_g_1_now_heat->figure_type = 5;
    ui_g_1_now_heat->operate_type = 1;
    ui_g_1_now_heat->layer = 0;
    ui_g_1_now_heat->color = 7;
    ui_g_1_now_heat->start_x = 9;
    ui_g_1_now_heat->start_y = 584;
    ui_g_1_now_heat->width = 3;
    ui_g_1_now_heat->font_size = 30;
    ui_g_1_now_heat->number = User_data.power_heat_data.shooter_17mm_barrel_heat;

    ui_g_1_heng->figure_type = 0;
    ui_g_1_heng->operate_type = 1;
    ui_g_1_heng->layer = 0;
    ui_g_1_heng->color = 5;
    ui_g_1_heng->start_x = 914;
    ui_g_1_heng->start_y = 538;
    ui_g_1_heng->width = 7;
    ui_g_1_heng->end_x = 1004;
    ui_g_1_heng->end_y = 538;


    ui_proc_7_frame(&ui_g_1_0);
    SEND_MESSAGE((uint8_t *) &ui_g_1_0, sizeof(ui_g_1_0));
}

void _ui_update_g_1_0() {
    for (int i = 0; i < 7; i++) {
        ui_g_1_0.data[i].operate_type = 2;
    }

    ui_proc_7_frame(&ui_g_1_0);
    SEND_MESSAGE((uint8_t *) &ui_g_1_0, sizeof(ui_g_1_0));
}

void _ui_remove_g_1_0() {
    for (int i = 0; i < 7; i++) {
        ui_g_1_0.data[i].operate_type = 3;
    }

    ui_proc_7_frame(&ui_g_1_0);
    SEND_MESSAGE((uint8_t *) &ui_g_1_0, sizeof(ui_g_1_0));
}
ui_5_frame_t ui_g_1_1;

ui_interface_line_t *ui_g_1_shu = (ui_interface_line_t*)&(ui_g_1_1.data[0]);
ui_interface_round_t *ui_g_1_zhongxin = (ui_interface_round_t*)&(ui_g_1_1.data[1]);
ui_interface_rect_t *ui_g_1_kuang = (ui_interface_rect_t*)&(ui_g_1_1.data[2]);

void _ui_init_g_1_1() {
    for (int i = 0; i < 3; i++) {
        ui_g_1_1.data[i].figure_name[0] = 0;
        ui_g_1_1.data[i].figure_name[1] = 0;
        ui_g_1_1.data[i].figure_name[2] = i + 7;
        ui_g_1_1.data[i].operate_type = 1;
    }
    for (int i = 3; i < 5; i++) {
        ui_g_1_1.data[i].operate_type = 0;
    }

    ui_g_1_shu->figure_type = 0;
    ui_g_1_shu->operate_type = 1;
    ui_g_1_shu->layer = 0;
    ui_g_1_shu->color = 5;
    ui_g_1_shu->start_x = 957;
    ui_g_1_shu->start_y = 501;
    ui_g_1_shu->width = 7;
    ui_g_1_shu->end_x = 957;
    ui_g_1_shu->end_y = 580;

    ui_g_1_zhongxin->figure_type = 2;
    ui_g_1_zhongxin->operate_type = 1;
    ui_g_1_zhongxin->layer = 0;
    ui_g_1_zhongxin->color = 5;
    ui_g_1_zhongxin->start_x = 959;
    ui_g_1_zhongxin->start_y = 538;
    ui_g_1_zhongxin->width = 2;
    ui_g_1_zhongxin->r = 76;

    ui_g_1_kuang->figure_type = 1;
    ui_g_1_kuang->operate_type = 1;
    ui_g_1_kuang->layer = 0;
    ui_g_1_kuang->color = 5;
    ui_g_1_kuang->start_x = 553;
    ui_g_1_kuang->start_y = 264;
    ui_g_1_kuang->width = 7;
    ui_g_1_kuang->end_x = 1362;
    ui_g_1_kuang->end_y = 821;


    ui_proc_5_frame(&ui_g_1_1);
    SEND_MESSAGE((uint8_t *) &ui_g_1_1, sizeof(ui_g_1_1));
}

void _ui_update_g_1_1() {
    for (int i = 0; i < 3; i++) {
        ui_g_1_1.data[i].operate_type = 2;
    }

    ui_proc_5_frame(&ui_g_1_1);
    SEND_MESSAGE((uint8_t *) &ui_g_1_1, sizeof(ui_g_1_1));
}

void _ui_remove_g_1_1() {
    for (int i = 0; i < 3; i++) {
        ui_g_1_1.data[i].operate_type = 3;
    }

    ui_proc_5_frame(&ui_g_1_1);
    SEND_MESSAGE((uint8_t *) &ui_g_1_1, sizeof(ui_g_1_1));
}

ui_string_frame_t ui_g_1_2;
ui_interface_string_t* ui_g_1_vision = &(ui_g_1_2.option);

void _ui_init_g_1_2() {
    ui_g_1_2.option.figure_name[0] = 0;
    ui_g_1_2.option.figure_name[1] = 0;
    ui_g_1_2.option.figure_name[2] = 10;
    ui_g_1_2.option.operate_type = 1;

    ui_g_1_vision->figure_type = 7;
    ui_g_1_vision->operate_type = 1;
    ui_g_1_vision->layer = 0;
    ui_g_1_vision->color = 2;
    ui_g_1_vision->start_x = 19;
    ui_g_1_vision->start_y = 301;
    ui_g_1_vision->width = 4;
    ui_g_1_vision->font_size = 35;
    ui_g_1_vision->str_length = 7;
    strcpy(ui_g_1_vision->string, "vision:");


    ui_proc_string_frame(&ui_g_1_2);
    SEND_MESSAGE((uint8_t *) &ui_g_1_2, sizeof(ui_g_1_2));
}

void _ui_update_g_1_2() {
    ui_g_1_2.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_1_2);
    SEND_MESSAGE((uint8_t *) &ui_g_1_2, sizeof(ui_g_1_2));
}

void _ui_remove_g_1_2() {
    ui_g_1_2.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_1_2);
    SEND_MESSAGE((uint8_t *) &ui_g_1_2, sizeof(ui_g_1_2));
}
ui_string_frame_t ui_g_1_3;
ui_interface_string_t* ui_g_1_heat = &(ui_g_1_3.option);

void _ui_init_g_1_3() {
    ui_g_1_3.option.figure_name[0] = 0;
    ui_g_1_3.option.figure_name[1] = 0;
    ui_g_1_3.option.figure_name[2] = 11;
    ui_g_1_3.option.operate_type = 1;

    ui_g_1_heat->figure_type = 7;
    ui_g_1_heat->operate_type = 1;
    ui_g_1_heat->layer = 0;
    ui_g_1_heat->color = 7;
    ui_g_1_heat->start_x = 71;
    ui_g_1_heat->start_y = 671;
    ui_g_1_heat->width = 4;
    ui_g_1_heat->font_size = 40;
    ui_g_1_heat->str_length = 5;
    strcpy(ui_g_1_heat->string, "heat:");


    ui_proc_string_frame(&ui_g_1_3);
    SEND_MESSAGE((uint8_t *) &ui_g_1_3, sizeof(ui_g_1_3));
}

void _ui_update_g_1_3() {
    ui_g_1_3.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_1_3);
    SEND_MESSAGE((uint8_t *) &ui_g_1_3, sizeof(ui_g_1_3));
}

void _ui_remove_g_1_3() {
    ui_g_1_3.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_1_3);
    SEND_MESSAGE((uint8_t *) &ui_g_1_3, sizeof(ui_g_1_3));
}
ui_string_frame_t ui_g_1_4;
ui_interface_string_t* ui_g_1_NewText = &(ui_g_1_4.option);

void _ui_init_g_1_4() {
    ui_g_1_4.option.figure_name[0] = 0;
    ui_g_1_4.option.figure_name[1] = 0;
    ui_g_1_4.option.figure_name[2] = 12;
    ui_g_1_4.option.operate_type = 1;

    ui_g_1_NewText->figure_type = 7;
    ui_g_1_NewText->operate_type = 1;
    ui_g_1_NewText->layer = 0;
    ui_g_1_NewText->color = 7;
    ui_g_1_NewText->start_x = 216;
    ui_g_1_NewText->start_y = 590;
    ui_g_1_NewText->width = 4;
    ui_g_1_NewText->font_size = 40;
    ui_g_1_NewText->str_length = 1;
    strcpy(ui_g_1_NewText->string, "/");


    ui_proc_string_frame(&ui_g_1_4);
    SEND_MESSAGE((uint8_t *) &ui_g_1_4, sizeof(ui_g_1_4));
}

void _ui_update_g_1_4() {
    ui_g_1_4.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_1_4);
    SEND_MESSAGE((uint8_t *) &ui_g_1_4, sizeof(ui_g_1_4));
}

void _ui_remove_g_1_4() {
    ui_g_1_4.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_1_4);
    SEND_MESSAGE((uint8_t *) &ui_g_1_4, sizeof(ui_g_1_4));
}
ui_string_frame_t ui_g_1_5;
ui_interface_string_t* ui_g_1_fire_wheel = &(ui_g_1_5.option);

void _ui_init_g_1_5() {
    ui_g_1_5.option.figure_name[0] = 0;
    ui_g_1_5.option.figure_name[1] = 0;
    ui_g_1_5.option.figure_name[2] = 13;
    ui_g_1_5.option.operate_type = 1;

    ui_g_1_fire_wheel->figure_type = 7;
    ui_g_1_fire_wheel->operate_type = 1;
    ui_g_1_fire_wheel->layer = 0;
    ui_g_1_fire_wheel->color = 1;
    ui_g_1_fire_wheel->start_x = 14;
    ui_g_1_fire_wheel->start_y = 499;
    ui_g_1_fire_wheel->width = 4;
    ui_g_1_fire_wheel->font_size = 35;
    ui_g_1_fire_wheel->str_length = 11;
    strcpy(ui_g_1_fire_wheel->string, "fire_wheel:");


    ui_proc_string_frame(&ui_g_1_5);
    SEND_MESSAGE((uint8_t *) &ui_g_1_5, sizeof(ui_g_1_5));
}

void _ui_update_g_1_5() {
    ui_g_1_5.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_1_5);
    SEND_MESSAGE((uint8_t *) &ui_g_1_5, sizeof(ui_g_1_5));
}

void _ui_remove_g_1_5() {
    ui_g_1_5.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_1_5);
    SEND_MESSAGE((uint8_t *) &ui_g_1_5, sizeof(ui_g_1_5));
}
ui_string_frame_t ui_g_1_6;
ui_interface_string_t* ui_g_1_shoot = &(ui_g_1_6.option);

void _ui_init_g_1_6() {
    ui_g_1_6.option.figure_name[0] = 0;
    ui_g_1_6.option.figure_name[1] = 0;
    ui_g_1_6.option.figure_name[2] = 14;
    ui_g_1_6.option.operate_type = 1;

    ui_g_1_shoot->figure_type = 7;
    ui_g_1_shoot->operate_type = 1;
    ui_g_1_shoot->layer = 0;
    ui_g_1_shoot->color = 6;
    ui_g_1_shoot->start_x = 14;
    ui_g_1_shoot->start_y = 400;
    ui_g_1_shoot->width = 4;
    ui_g_1_shoot->font_size = 35;
    ui_g_1_shoot->str_length = 6;
    strcpy(ui_g_1_shoot->string, "shoot:");


    ui_proc_string_frame(&ui_g_1_6);
    SEND_MESSAGE((uint8_t *) &ui_g_1_6, sizeof(ui_g_1_6));
}

void _ui_update_g_1_6() {
    ui_g_1_6.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_1_6);
    SEND_MESSAGE((uint8_t *) &ui_g_1_6, sizeof(ui_g_1_6));
}

void _ui_remove_g_1_6() {
    ui_g_1_6.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_1_6);
    SEND_MESSAGE((uint8_t *) &ui_g_1_6, sizeof(ui_g_1_6));
}
ui_string_frame_t ui_g_1_7;
ui_interface_string_t* ui_g_1_Pitch = &(ui_g_1_7.option);

void _ui_init_g_1_7() {
    ui_g_1_7.option.figure_name[0] = 0;
    ui_g_1_7.option.figure_name[1] = 0;
    ui_g_1_7.option.figure_name[2] = 15;
    ui_g_1_7.option.operate_type = 1;

    ui_g_1_Pitch->figure_type = 7;
    ui_g_1_Pitch->operate_type = 1;
    ui_g_1_Pitch->layer = 0;
    ui_g_1_Pitch->color = 4;
    ui_g_1_Pitch->start_x = 50;
    ui_g_1_Pitch->start_y = 835;
    ui_g_1_Pitch->width = 5;
    ui_g_1_Pitch->font_size = 50;
    ui_g_1_Pitch->str_length = 6;
    strcpy(ui_g_1_Pitch->string, "Pitch:");


    ui_proc_string_frame(&ui_g_1_7);
    SEND_MESSAGE((uint8_t *) &ui_g_1_7, sizeof(ui_g_1_7));
}

void _ui_update_g_1_7() {
    ui_g_1_7.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_1_7);
    SEND_MESSAGE((uint8_t *) &ui_g_1_7, sizeof(ui_g_1_7));
}

void _ui_remove_g_1_7() {
    ui_g_1_7.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_1_7);
    SEND_MESSAGE((uint8_t *) &ui_g_1_7, sizeof(ui_g_1_7));
}
ui_string_frame_t ui_g_1_8;
ui_interface_string_t* ui_g_1_xiaoxiaobu = &(ui_g_1_8.option);

void _ui_init_g_1_8() {
    ui_g_1_8.option.figure_name[0] = 0;
    ui_g_1_8.option.figure_name[1] = 0;
    ui_g_1_8.option.figure_name[2] = 16;
    ui_g_1_8.option.operate_type = 1;

    ui_g_1_xiaoxiaobu->figure_type = 7;
    ui_g_1_xiaoxiaobu->operate_type = 1;
    ui_g_1_xiaoxiaobu->layer = 0;
    ui_g_1_xiaoxiaobu->color = 2;
    ui_g_1_xiaoxiaobu->start_x = 810;
    ui_g_1_xiaoxiaobu->start_y = 71;
    ui_g_1_xiaoxiaobu->width = 4;
    ui_g_1_xiaoxiaobu->font_size = 40;
    ui_g_1_xiaoxiaobu->str_length = 23;
    strcpy(ui_g_1_xiaoxiaobu->string, "(๑•̀ㅂ•́)و✧");


    ui_proc_string_frame(&ui_g_1_8);
    SEND_MESSAGE((uint8_t *) &ui_g_1_8, sizeof(ui_g_1_8));
}

void _ui_update_g_1_8() {
    ui_g_1_8.option.operate_type = 2;

    ui_proc_string_frame(&ui_g_1_8);
    SEND_MESSAGE((uint8_t *) &ui_g_1_8, sizeof(ui_g_1_8));
}

void _ui_remove_g_1_8() {
    ui_g_1_8.option.operate_type = 3;

    ui_proc_string_frame(&ui_g_1_8);
    SEND_MESSAGE((uint8_t *) &ui_g_1_8, sizeof(ui_g_1_8));
}

void ui_init_g_1() {
    _ui_init_g_1_0();
    _ui_init_g_1_1();
    _ui_init_g_1_2();
//    _ui_init_g_1_3();
//    _ui_init_g_1_4();
//    _ui_init_g_1_5();
//    _ui_init_g_1_6();
//    _ui_init_g_1_7();
//    _ui_init_g_1_8();
}

void ui_update_g_1() {
    _ui_update_g_1_0();
    _ui_update_g_1_1();
    _ui_update_g_1_2();
//    _ui_update_g_1_3();
//    _ui_update_g_1_4();
//    _ui_update_g_1_5();
//    _ui_update_g_1_6();
//    _ui_update_g_1_7();
//    _ui_update_g_1_8();
}

void ui_remove_g_1() {
    _ui_remove_g_1_0();
    _ui_remove_g_1_1();
    _ui_remove_g_1_2();
//    _ui_remove_g_1_3();
//    _ui_remove_g_1_4();
//    _ui_remove_g_1_5();
//    _ui_remove_g_1_6();
//    _ui_remove_g_1_7();
//    _ui_remove_g_1_8();
}

