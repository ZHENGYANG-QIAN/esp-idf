/* main.c - Application main entry point */

/*
 * SPDX-FileCopyrightText: 2017 Intel Corporation
 * SPDX-FileContributor: 2018-2021 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
#include <inttypes.h>

#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"

#include "mesh/adapter.h"

#include "esp_ble_mesh_defs.h"
#include "esp_ble_mesh_common_api.h"
#include "esp_ble_mesh_networking_api.h"
#include "esp_ble_mesh_provisioning_api.h"
#include "esp_ble_mesh_config_model_api.h"
#include "esp_ble_mesh_generic_model_api.h"
#include "esp_ble_mesh_local_data_operation_api.h"
#include "esp_ble_mesh_df_model_api.h"

#include "board.h"
#include "ble_mesh_example_init.h"

#define TAG "EXAMPLE"

#define CID_ESP 0x02E5
#define APP_KEY_IDX         0x0000
#define APP_KEY_OCTET       0x12

#define ESP_BLE_MESH_VND_MODEL_ID_CLIENT    0x0000
#define ESP_BLE_MESH_VND_MODEL_ID_SERVER    0x0001

#define ESP_BLE_MESH_VND_MODEL_OP_SET      ESP_BLE_MESH_MODEL_OP_3(0x00, CID_ESP)
#define ESP_BLE_MESH_VND_MODEL_OP_STATUS    ESP_BLE_MESH_MODEL_OP_3(0x01, CID_ESP)

extern int bt_mesh_test_update_white_list(struct bt_mesh_white_list *wl);
extern int bt_mesh_test_stop_scanning(void);
extern int bt_mesh_test_start_scanning(bool wl_en);

static uint8_t dev_uuid[16] = { 0xaa, 0x55 };
static uint8_t app_key[16] = {0};

static esp_ble_mesh_cfg_srv_t config_server = {
    /* 3 transmissions with 20ms interval */
    .net_transmit = ESP_BLE_MESH_TRANSMIT(2, 20),
    .relay = ESP_BLE_MESH_RELAY_ENABLED,
    .relay_retransmit = ESP_BLE_MESH_TRANSMIT(2, 20),
    .beacon = ESP_BLE_MESH_BEACON_ENABLED,
#if defined(CONFIG_BLE_MESH_GATT_PROXY_SERVER)
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_ENABLED,
#else
    .gatt_proxy = ESP_BLE_MESH_GATT_PROXY_NOT_SUPPORTED,
#endif
#if defined(CONFIG_BLE_MESH_FRIEND)
    .friend_state = ESP_BLE_MESH_FRIEND_ENABLED,
#else
    .friend_state = ESP_BLE_MESH_FRIEND_NOT_SUPPORTED,
#endif
    .default_ttl = 7,
};

#if CONFIG_BLE_MESH_DF_SRV
static esp_ble_mesh_df_srv_t directed_forwarding_server = {
    .directed_net_transmit = ESP_BLE_MESH_TRANSMIT(1, 100),
    .directed_relay_retransmit = ESP_BLE_MESH_TRANSMIT(2, 100),
    .default_rssi_threshold = (-80),
    .rssi_margin = 20,
    .directed_node_paths = 20,
    .directed_relay_paths = 20,
#if defined(CONFIG_BLE_MESH_GATT_PROXY_SERVER)
    .directed_proxy_paths = 20,
#else
    .directed_proxy_paths = 0,
#endif
#if defined(CONFIG_BLE_MESH_FRIEND)
    .directed_friend_paths = 20,
#else
    .directed_friend_paths = 0,
#endif
    .path_monitor_interval = 120,
    .path_disc_retry_interval = 300,
    .path_disc_interval = ESP_BLE_MESH_PATH_DISC_INTERVAL_30_SEC,
    .lane_disc_guard_interval = ESP_BLE_MESH_LANE_DISC_GUARD_INTERVAL_10_SEC,
    .directed_ctl_net_transmit = ESP_BLE_MESH_TRANSMIT(1, 100),
    .directed_ctl_relay_retransmit = ESP_BLE_MESH_TRANSMIT(2, 100),
};
#endif

// ESP_BLE_MESH_MODEL_PUB_DEFINE(onoff_pub_0, 2 + 3, ROLE_NODE);
// static esp_ble_mesh_gen_onoff_srv_t onoff_server_0 = {
//     .rsp_ctrl ={
//         .get_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
//         .set_auto_rsp = ESP_BLE_MESH_SERVER_AUTO_RSP,
//     },
// };

static esp_ble_mesh_model_t root_models[] = {
    ESP_BLE_MESH_MODEL_CFG_SRV(&config_server),
#if CONFIG_BLE_MESH_DF_SRV
    ESP_BLE_MESH_MODEL_DF_SRV(&directed_forwarding_server),
#endif
    // ESP_BLE_MESH_MODEL_GEN_ONOFF_SRV(&onoff_pub_0, &onoff_server_0),
};

static esp_ble_mesh_model_op_t vnd_op[] = {
    ESP_BLE_MESH_MODEL_OP(ESP_BLE_MESH_VND_MODEL_OP_SET, 2),
    ESP_BLE_MESH_MODEL_OP_END,
};

static esp_ble_mesh_model_t vnd_models[] = {
    ESP_BLE_MESH_VENDOR_MODEL(CID_ESP, ESP_BLE_MESH_VND_MODEL_ID_SERVER,
    vnd_op, NULL, NULL),
};

static esp_ble_mesh_elem_t elements[] = {
    ESP_BLE_MESH_ELEMENT(0, root_models, vnd_models),
};

static esp_ble_mesh_comp_t composition = {
    .cid = CID_ESP,
    .element_count = ARRAY_SIZE(elements),
    .elements = elements,
};

/* Disable OOB security for SILabs Android app */
static esp_ble_mesh_prov_t provision = {
    .uuid = dev_uuid,
#if 0
    .output_size = 4,
    .output_actions = ESP_BLE_MESH_DISPLAY_NUMBER,
    .input_size = 4,
    .input_actions = ESP_BLE_MESH_PUSH,
#else
    .output_size = 0,
    .output_actions = 0,
#endif
};

static void prov_complete(uint16_t net_idx, uint16_t addr, uint8_t flags, uint32_t iv_index)
{
    ESP_LOGI(TAG, "net_idx: 0x%04x, addr: 0x%04x", net_idx, addr);
    ESP_LOGI(TAG, "flags: 0x%02x, iv_index: 0x%08" PRIx32, flags, iv_index);
    esp_ble_mesh_node_add_local_app_key(app_key, net_idx, APP_KEY_IDX);
}

static void example_ble_mesh_provisioning_cb(esp_ble_mesh_prov_cb_event_t event,
                                             esp_ble_mesh_prov_cb_param_t *param)
{
    switch (event) {
    case ESP_BLE_MESH_PROV_REGISTER_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_PROV_REGISTER_COMP_EVT, err_code %d", param->prov_register_comp.err_code);
        break;
    case ESP_BLE_MESH_NODE_PROV_ENABLE_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_ENABLE_COMP_EVT, err_code %d", param->node_prov_enable_comp.err_code);
        break;
    case ESP_BLE_MESH_NODE_PROV_LINK_OPEN_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_LINK_OPEN_EVT, bearer %s",
            param->node_prov_link_open.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
        break;
    case ESP_BLE_MESH_NODE_PROV_LINK_CLOSE_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_LINK_CLOSE_EVT, bearer %s",
            param->node_prov_link_close.bearer == ESP_BLE_MESH_PROV_ADV ? "PB-ADV" : "PB-GATT");
        break;
    case ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_COMPLETE_EVT");
        prov_complete(param->node_prov_complete.net_idx, param->node_prov_complete.addr,
            param->node_prov_complete.flags, param->node_prov_complete.iv_index);
        break;
    case ESP_BLE_MESH_NODE_PROV_RESET_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_PROV_RESET_EVT");
        break;
    case ESP_BLE_MESH_NODE_SET_UNPROV_DEV_NAME_COMP_EVT:
        ESP_LOGI(TAG, "ESP_BLE_MESH_NODE_SET_UNPROV_DEV_NAME_COMP_EVT, err_code %d", param->node_set_unprov_dev_name_comp.err_code);
        break;
    case ESP_BLE_MESH_NODE_ADD_LOCAL_APP_KEY_COMP_EVT:{
        ESP_LOGI(TAG, "ESP_BLE_MESH_PROVISIONER_ADD_LOCAL_APP_KEY_COMP_EVT, err_code %d", param->node_add_app_key_comp.err_code);
        if (param->node_add_app_key_comp.err_code == ESP_OK) {
            esp_err_t err = 0;
            uint8_t app_idx = param->node_add_app_key_comp.app_idx;
            if (app_idx == APP_KEY_IDX) {
                err = esp_ble_mesh_node_bind_app_key_to_local_model(esp_ble_mesh_get_primary_element_address(), CID_ESP,
                        ESP_BLE_MESH_VND_MODEL_ID_SERVER, APP_KEY_IDX);
                if (err != ESP_OK) {
                    ESP_LOGE(TAG, "Provisioner bind local model appkey failed %d", err);
                    return;
                }
            }
        }
        break;
    }

    default:
        break;
    }
}

static void example_ble_mesh_config_server_cb(esp_ble_mesh_cfg_server_cb_event_t event,
                                              esp_ble_mesh_cfg_server_cb_param_t *param)
{
    if (event == ESP_BLE_MESH_CFG_SERVER_STATE_CHANGE_EVT) {
        switch (param->ctx.recv_op) {
        case ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_APP_KEY_ADD");
            ESP_LOGI(TAG, "net_idx 0x%04x, app_idx 0x%04x",
                param->value.state_change.appkey_add.net_idx,
                param->value.state_change.appkey_add.app_idx);
            ESP_LOG_BUFFER_HEX("AppKey", param->value.state_change.appkey_add.app_key, 16);
            break;
        case ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_MODEL_APP_BIND");
            ESP_LOGI(TAG, "elem_addr 0x%04x, app_idx 0x%04x, cid 0x%04x, mod_id 0x%04x",
                param->value.state_change.mod_app_bind.element_addr,
                param->value.state_change.mod_app_bind.app_idx,
                param->value.state_change.mod_app_bind.company_id,
                param->value.state_change.mod_app_bind.model_id);
            break;
        case ESP_BLE_MESH_MODEL_OP_MODEL_SUB_ADD:
            ESP_LOGI(TAG, "ESP_BLE_MESH_MODEL_OP_MODEL_SUB_ADD");
            ESP_LOGI(TAG, "elem_addr 0x%04x, sub_addr 0x%04x, cid 0x%04x, mod_id 0x%04x",
                param->value.state_change.mod_sub_add.element_addr,
                param->value.state_change.mod_sub_add.sub_addr,
                param->value.state_change.mod_sub_add.company_id,
                param->value.state_change.mod_sub_add.model_id);
            break;
        default:
            break;
        }
    }
}

static void example_ble_mesh_directed_forwarding_server_cb(esp_ble_mesh_df_server_cb_event_t event,
                                                           esp_ble_mesh_df_server_cb_param_t *param)
{
    esp_ble_mesh_df_server_table_change_t change;
    esp_ble_mesh_uar_t path_origin;
    esp_ble_mesh_uar_t path_target;
    memset(&change, 0, sizeof(esp_ble_mesh_df_server_table_change_t));

    if (event == ESP_BLE_MESH_DF_SERVER_TABLE_CHANGE_EVT) {
        memcpy(&change, &param->value.table_change, sizeof(esp_ble_mesh_df_server_table_change_t));

        switch (change.action) {
            case ESP_BLE_MESH_DF_TABLE_ADD: {
                memcpy(&path_origin, &change.df_table_info.df_table_entry_add_remove.path_origin, sizeof(path_origin));
                memcpy(&path_target, &change.df_table_info.df_table_entry_add_remove.path_target, sizeof(path_target));
                ESP_LOGI(TAG, "Established a path from 0x%04x to 0x%04x", path_origin.range_start, path_target.range_start);

            }
                break;
            case ESP_BLE_MESH_DF_TABLE_REMOVE: {
                memcpy(&path_origin, &change.df_table_info.df_table_entry_add_remove.path_origin, sizeof(path_origin));
                memcpy(&path_target, &change.df_table_info.df_table_entry_add_remove.path_target, sizeof(path_target));
                ESP_LOGI(TAG, "Remove a path from 0x%04x to 0x%04x", path_origin.range_start, path_target.range_start);
            }
                break;
            default:
                ESP_LOGW(TAG, "Unknown action %d", change.action);
        }
    }

    return;
}

static void example_ble_mesh_custom_model_cb(esp_ble_mesh_model_cb_event_t event,
                                             esp_ble_mesh_model_cb_param_t *param)
{
    // ESP_LOGI(TAG, "event: %04x opcode %06x", event, param->model_operation.opcode);
    switch (event) {
    case ESP_BLE_MESH_MODEL_OPERATION_EVT:
        if (param->model_operation.opcode == ESP_BLE_MESH_VND_MODEL_OP_SET) {
            uint8_t *msg = (uint8_t *)param->model_operation.msg;
            uint8_t tid = msg[0];
            uint8_t len = msg[1];
            ESP_LOGI("Server", "Recv 0x%06" PRIx32 ", tid 0x%02x, len 0x%02x", param->model_operation.opcode, tid, len);
            ESP_LOG_BUFFER_HEX("Msg", msg + 2, len);
            esp_err_t err = esp_ble_mesh_server_model_send_msg(&vnd_models[0],
                    param->model_operation.ctx, ESP_BLE_MESH_VND_MODEL_OP_STATUS,
                    sizeof(msg), (uint8_t *)msg);
            if (err) {
                ESP_LOGE("Server", "Failed to send message 0x%06x", ESP_BLE_MESH_VND_MODEL_OP_STATUS);
            }
        }
        break;
    case ESP_BLE_MESH_MODEL_SEND_COMP_EVT:
        if (param->model_send_comp.err_code) {
            ESP_LOGE("Server", "Failed to send message 0x%06" PRIx32, param->model_send_comp.opcode);
            break;
        }
        ESP_LOGI("Server", "Send 0x%06" PRIx32, param->model_send_comp.opcode);
        break;
    default:
        break;
    }
}

static esp_err_t ble_mesh_init(void)
{
    esp_err_t err = ESP_OK;

    memset(app_key, APP_KEY_OCTET, sizeof(app_key));

    esp_ble_mesh_register_prov_callback(example_ble_mesh_provisioning_cb);
    esp_ble_mesh_register_config_server_callback(example_ble_mesh_config_server_cb);
    esp_ble_mesh_register_custom_model_callback(example_ble_mesh_custom_model_cb);
    esp_ble_mesh_register_df_server_callback(example_ble_mesh_directed_forwarding_server_cb);

    err = esp_ble_mesh_init(&provision, &composition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize mesh stack (err %d)", err);
        return err;
    }

    err = esp_ble_mesh_node_prov_enable((esp_ble_mesh_prov_bearer_t)(ESP_BLE_MESH_PROV_ADV | ESP_BLE_MESH_PROV_GATT));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable mesh node (err %d)", err);
        return err;
    }

    ESP_LOGI(TAG, "BLE Mesh Node initialized");

    return err;
}

void app_main(void)
{
    esp_err_t err;

    ESP_LOGI(TAG, "Initializing...");

    board_init();

    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);

    err = bluetooth_init();
    if (err) {
        ESP_LOGE(TAG, "esp32_bluetooth_init failed (err %d)", err);
        return;
    }

    ble_mesh_get_dev_uuid(dev_uuid);

    ESP_LOG_BUFFER_HEX(TAG":Provisioner bd address: ", dev_uuid + 2, BD_ADDR_LEN);
    /* Initialize the Bluetooth Mesh Subsystem */
    err = ble_mesh_init();
    if (err) {
        ESP_LOGE(TAG, "Bluetooth mesh init failed (err %d)", err);
    }
}
