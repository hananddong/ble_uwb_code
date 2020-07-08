#include "dev_test_max_speed.h"
#include "dev.h"
#include "ble_nus.h"
#include "battery.h"
#include "blue.h"
#include "user_log.h"
    
/*
    NRF_SDH_BLE_GATT_MAX_MTU_SIZE 最大 247
    nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    
    #define BLE_NUS_MAX_DATA_LEN (NRF_SDH_BLE_GATT_MAX_MTU_SIZE - OPCODE_LENGTH - HANDLE_LENGTH)
    
    BLE_GATT_ATT_MTU_DEFAULT
*/

uint32_t Pkg_Index = 0;
uint32_t Pkg_Index_Max = 0;

void dev_init_test_max_speed(void)
{
    set_battery_percent_for_test();
}

void dev_send_64hz_in_rtc_test_max_speed(void)
{
    uint16_t tmp_uint16 = 0;
    uint32_t tmp_uint32 = 0;
    
    if(flag_ble_connected)
    {
        MsgToMaster[0] = 0xFF;
        MsgToMaster[1] = 0x02;
    
        MsgToMaster[2] = _rssi_;                        // 2020-01-13 两个字节的 ID 改为 RSSI 和 ID
        MsgToMaster[3] = UserFlashCfg.Pkg.DevID;
        
        Pkg_Index++;
        tmp_uint32 = ++pDevInfo->FrameIndex;            // FrameIndex
        MsgToMaster[4] = tmp_uint32 >> 24;
        MsgToMaster[5] = tmp_uint32 >> 16;
        MsgToMaster[6] = tmp_uint32 >> 8;
        MsgToMaster[7] = tmp_uint32 >> 0;
        
        MsgToMaster[8] = 250 / 2;
            
        tmp_uint16 = Pkg_Index_Max;                     
        MsgToMaster[9] = tmp_uint16 >> 8;   // PPG_0
        MsgToMaster[10] = tmp_uint16;
        
        MsgToMaster[11] = tmp_uint16 >> 8;  // PPG_1
        MsgToMaster[12] = tmp_uint16;
        
        MsgToMaster[13] = tmp_uint16 >> 8;  // PPG_2
        MsgToMaster[14] = tmp_uint16;
        
        MsgToMaster[15] = tmp_uint16 >> 8;  // PPG_3
        MsgToMaster[16] = tmp_uint16;
        
        tmp_uint16 = Pkg_Index_Max >> 16;                     
        MsgToMaster[17] = tmp_uint16 >> 8;  // BPM_0
        MsgToMaster[18] = tmp_uint16;
        
        MsgToMaster[19] = tmp_uint16 >> 8;  // BPM_1
        MsgToMaster[20] = tmp_uint16;
        
        MsgToMaster[21] = tmp_uint16 >> 8;  // BPM_2
        MsgToMaster[22] = tmp_uint16;
        
        MsgToMaster[23] = tmp_uint16 >> 8;  // BPM_3
        MsgToMaster[24] = tmp_uint16;
        
        send_data_to_chen();
        //ble_nus_send_data(MsgToMaster, BLE_NUS_MAX_DATA_LEN);
        //ble_nus_send_data(MsgToMaster, BLE_GATT_ATT_MTU_DEFAULT - 3);
        
        log_dbg_speed("%d\r\n", pDevInfo->FrameIndex);
    }
}

void test_max_speed_index_reset(void)
{
    Pkg_Index_Max = Pkg_Index;
    Pkg_Index = 0;
}

