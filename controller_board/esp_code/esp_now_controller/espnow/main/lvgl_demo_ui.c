/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 */

#include "lvgl.h"

#include <stdlib.h>
#include <stdio.h>


double distance = 34.4;

void example_lvgl_demo_ui(lv_disp_t *disp)
{
    char flt_buffer[10];
    char str_buffer[20] = "Dist: ";

    snprintf(flt_buffer, sizeof(flt_buffer), "%.2f", distance);
    strcat(str_buffer, flt_buffer);
    strcat(str_buffer, " cm");
    lv_obj_t *scr = lv_disp_get_scr_act(disp);
    lv_obj_t *label = lv_label_create(scr);
    //lv_label_set_long_mode(label, LV_LABEL_LONG_SCROLL_CIRCULAR); /* Circular scroll */
    lv_label_set_text(label, str_buffer);
    /* Size of the screen (if you use rotation 90 or 270, please set disp->driver->ver_res) */
    lv_obj_set_width(label, disp->driver->hor_res);
    lv_obj_align(label, LV_ALIGN_TOP_LEFT, 0, 0);

    lv_obj_t *metal_dec_label = lv_label_create(scr);
    if (metal_detected == 1){
        lv_label_set_text(metal_dec_label, "Detected: YES");
    } else{
        lv_label_set_text(metal_dec_label, "Detected: NO");
    }
    
    lv_obj_align(metal_dec_label, LV_ALIGN_TOP_LEFT, 0, 32); 
}
