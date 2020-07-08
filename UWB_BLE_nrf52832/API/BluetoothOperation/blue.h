#ifndef __BLUE_H__
#define __BLUE_H__

#include "ble_nus.h"
#include "ble_conn_params.h"
#include "ble_advertising.h"
#include "nrf_ble_gatt.h"
#include "bsp.h"
#include "nrf_ble_qwr.h"
#include "nrf_sdh_ble.h"
#include "nrf_pwr_mgmt.h"
#include "bsp_btn_ble.h"
#include "nrf_sdh.h"
#include "app_util_platform.h"
#include "app_timer.h"
#include "app_uart.h"
#include "nrf_sdh_soc.h"

extern bool flag_ble_connected;
extern bool flag_last_tx_need_retry;

extern void log_init(void);
extern void uart_init(void);
extern void timers_init(void);
extern void ble_init(void);
extern void send_info_to_ErgoLAB(void);
extern void send_data_to_ergolab(void);
extern void ble_nus_send_data(uint8_t *p_data, uint16_t length);
extern void update_scan_uuid_by_group_id(uint8_t group_id);
extern uint8_t UartData_UWB_Pkg[BLE_NUS_MAX_DATA_LEN];
extern uint8_t uart_data_ready;
#endif  // __BLUE_H__

