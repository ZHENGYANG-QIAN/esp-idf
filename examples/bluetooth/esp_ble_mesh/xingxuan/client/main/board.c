/* board.c - Board-specific hooks */

/*
 * SPDX-FileCopyrightText: 2017 Intel Corporation
 * SPDX-FileContributor: 2018-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>

#include "driver/gpio.h"
#include "esp_log.h"

#include "iot_button.h"
#include "board.h"
#include "esp_timer.h"

#define TAG "BOARD"

#define BUTTON_ACTIVE_LEVEL     0

extern void example_ble_mesh_start_example_configuration(void);
extern void example_ble_mesh_send_df_vendor_message(bool by_df);
extern uint8_t in_configuration_phase;

static void button_tap_cb(void* arg)
{
    ESP_LOGI(TAG, "tap cb (%s)", (char *)arg);
    example_ble_mesh_send_df_vendor_message(true);
    // static bool use_df = true;
    // if (in_configuration_phase) {
    //     example_ble_mesh_start_example_configuration();
    // } else {
    //     example_ble_mesh_send_df_vendor_message(use_df);
    //     use_df = !use_df;
    // }
}

static void board_button_init(void)
{
    button_handle_t btn_handle = iot_button_create(BUTTON_IO_NUM, BUTTON_ACTIVE_LEVEL);
    if (btn_handle) {
        iot_button_set_evt_cb(btn_handle, BUTTON_CB_RELEASE, button_tap_cb, "RELEASE");
    }
}

void board_init(void)
{
    board_button_init();
}
