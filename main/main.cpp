#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include <cstring>
#include "esp_gatts_api.h"
#include "esp_gatt_common_api.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

constexpr int NUM_CHARS { CONFIG_BLE_CHARS };

constexpr char DEVICE_NAME[] { "TESTER" };

static uint8_t base_uuid[16] {
	// base UUID: 367ec074-9a6c-11ea-8ad0-377f1627427f
	0x7f, 0x42, 0x27, 0x16, 0x7f, 0x37, 0xd0, 0x8a, 0xea, 0x11,
	0x6c, 0x9a, 0x74, 0xc0, 0x7e, 0x36
};

static uint8_t current_char_uuid[16];

constexpr char TAG[] { "app" };

static void le_gap_event_handler(
	esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param
) {
	ESP_LOGW(TAG, "unknown BLE GAP event %d", event);
}

constexpr int NUM_HANDLES { NUM_CHARS * 3 + 1 };
static int cur_char { 0 };

uint16_t char_handle(int idx) { return idx * 3 + 0x2a; }
uint16_t desc_handle(int idx) { return idx * 3 + 0x2b; }

void inc_uuid(uint8_t *uuid) {
	assert(uuid);
	uint8_t carry { 1 };
	uint8_t *cur { uuid + 12 };
	for (; carry && cur < uuid + 16; ++cur) {
		*cur += 1;
		carry = ! *cur;
	}
	if (carry) {
		ESP_LOGW(TAG, "uuid overflow");
	}
}

static void gatts_event_handler(
	esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if,
	esp_ble_gatts_cb_param_t *param
) {
	switch (event) {
	case ESP_GATTS_REG_EVT: {
		if (param->reg.status != ESP_GATT_OK) {
			ESP_LOGE(TAG, "event %d failed, app_id 0x%04x, status %d",
				event, param->reg.app_id, param->reg.status
			);
			return;
		}
		ESP_LOGI(TAG, "%d, status %d, app_id 0x%04x",
			event,param->reg.status, param->reg.app_id
		);
		ESP_ERROR_CHECK(esp_ble_gap_set_device_name(DEVICE_NAME));
		esp_ble_adv_data_t adv_data { };
		adv_data.set_scan_rsp = false;
		adv_data.include_name = false;
		adv_data.include_txpower = false;
		adv_data.min_interval = 0x0006;
		adv_data.max_interval = 0x0010;
		adv_data.appearance = 0x00;
		adv_data.manufacturer_len = 0;
		adv_data.p_manufacturer_data = nullptr;
		adv_data.service_data_len = 0;
		adv_data.p_service_data = nullptr;
		adv_data.service_uuid_len = sizeof(base_uuid);
		adv_data.p_service_uuid = base_uuid;
		adv_data.flag = ESP_BLE_ADV_FLAG_GEN_DISC |
			ESP_BLE_ADV_FLAG_BREDR_NOT_SPT;
		ESP_ERROR_CHECK(esp_ble_gap_config_adv_data(&adv_data));
		esp_gatt_srvc_id_t service_id { };
		service_id.is_primary = true;
		service_id.id.inst_id = 0;
		service_id.id.uuid.len = ESP_UUID_LEN_128;
		static_assert(ESP_UUID_LEN_128 <= sizeof(base_uuid));
		memcpy(service_id.id.uuid.uuid.uuid128, base_uuid,
			ESP_UUID_LEN_128
		);
		ESP_ERROR_CHECK(esp_ble_gatts_create_service(
			gatts_if, &service_id, NUM_HANDLES
		));
		break;
	}
	case ESP_GATTS_CREATE_EVT: {
		auto handle { param->create.service_handle };
		ESP_LOGI(TAG,
			"event %d, status %d, service_handle 0x%02x",
			event, param->create.status, handle
		);
		ESP_ERROR_CHECK(esp_ble_gatts_start_service(handle));
		break;
	}
	case ESP_GATTS_ADD_CHAR_EVT: {
		auto service { param->add_char.service_handle };
		auto attr { param->add_char.attr_handle };
		if (param->add_char.status) ESP_LOGW(TAG,
			"event %d, status %d, attr_handle 0x%02x, "
			"service_handle 0x%02x", event, param->add_char.status,
			attr, service
		);
		if (attr != char_handle(cur_char)) {
			ESP_LOGE(TAG, "wrong char handle %02x, %02x, %02x",
				cur_char, attr, char_handle(cur_char)
			);
		}
		esp_bt_uuid_t desc_id;
		desc_id.len = ESP_UUID_LEN_16;
		desc_id.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

		ESP_ERROR_CHECK(esp_ble_gatts_add_char_descr(
			service, &desc_id,
			ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
			nullptr, nullptr
		));
		break;
	}
	case ESP_GATTS_ADD_CHAR_DESCR_EVT: {
		auto service { param->add_char.service_handle };
		auto attr { param->add_char.attr_handle };
		if (param->add_char_descr.status) ESP_LOGW(TAG,
			"event %d, status %d, attr_handle 0x%0x, "
			"service_handle 0x%02x",
			event, param->add_char_descr.status, attr, service
		);
		if (desc_handle(cur_char) != attr) {
			ESP_LOGE(TAG, "wrong desc handle %02x, %02x, %02x",
				cur_char, attr, desc_handle(cur_char)
			);
		}
		if (++cur_char < NUM_CHARS) {
			esp_bt_uuid_t char_id;
			char_id.len = ESP_UUID_LEN_128;
			inc_uuid(current_char_uuid);
			memcpy(
				char_id.uuid.uuid128, current_char_uuid,
				ESP_UUID_LEN_128
			);
			ESP_ERROR_CHECK(esp_ble_gatts_add_char(
				service, &char_id,
				ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
				ESP_GATT_CHAR_PROP_BIT_READ |
					ESP_GATT_CHAR_PROP_BIT_WRITE |
					ESP_GATT_CHAR_PROP_BIT_WRITE_NR |
					ESP_GATT_CHAR_PROP_BIT_NOTIFY,
				nullptr, nullptr
			));
		} else {
			ESP_LOGI(TAG, "registered %d characteristics", NUM_CHARS);
		}
		break;
	}
	case ESP_GATTS_START_EVT: {
		auto service { param->start.service_handle };
		ESP_LOGI(TAG, "event %d, status %d, service_handle 0x%02x",
			event, param->start.status, service
		);
		esp_bt_uuid_t char_id { };
		char_id.len = ESP_UUID_LEN_128;
		memcpy(current_char_uuid, base_uuid,
			sizeof(current_char_uuid)
		);
		inc_uuid(current_char_uuid);
		memcpy(char_id.uuid.uuid128, current_char_uuid,
			ESP_UUID_LEN_128
		);
		ESP_ERROR_CHECK(esp_ble_gatts_add_char(
			service, &char_id,
			ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
			ESP_GATT_CHAR_PROP_BIT_READ |
				ESP_GATT_CHAR_PROP_BIT_WRITE |
				ESP_GATT_CHAR_PROP_BIT_WRITE_NR |
				ESP_GATT_CHAR_PROP_BIT_NOTIFY,
			nullptr, nullptr
		));
		break;
	}
	default:
		ESP_LOGW(TAG, "unhandled event %d", event);
		break;
	}
}

extern "C" void app_main() {
	esp_err_t ret { nvs_flash_init() };
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
		ret == ESP_ERR_NVS_NEW_VERSION_FOUND
	) {
		ESP_ERROR_CHECK(nvs_flash_erase());
		ret = nvs_flash_init();
	}
	ESP_ERROR_CHECK(ret);

	esp_bt_controller_config_t bt_cfg =
		BT_CONTROLLER_INIT_CONFIG_DEFAULT();
	ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
	ESP_ERROR_CHECK(esp_bt_controller_enable( ESP_BT_MODE_BLE));
	ESP_ERROR_CHECK(esp_bluedroid_init());
	ESP_ERROR_CHECK(esp_bluedroid_enable());
	ESP_ERROR_CHECK(esp_ble_gap_register_callback(
		le_gap_event_handler
	));
	ESP_ERROR_CHECK(esp_ble_gatts_register_callback(
		gatts_event_handler
	));

	constexpr int APP_ID { 0 };
	ESP_ERROR_CHECK(esp_ble_gatts_app_register(APP_ID));
}
