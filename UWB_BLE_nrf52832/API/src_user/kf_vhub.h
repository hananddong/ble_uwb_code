#ifndef	__KF_VHUB_H__
#define __KF_VHUB_H__

#include "stdint.h"
#include "stdbool.h"

#define BleCmd_01_Info                  0x01
#define BleCmd_02_Data                  0x02
#define BleCmd_03_Stop                  0x03
#define BleCmd_04_Config                0x04
#define BleCmd_05_SetGroupID            0x05
#define BleCmd_0A_SetDevID              0x0A
#define BleCmd_0B_SetNameIndex          0x0B

// Slave ble report type
typedef enum
{
    Report_Stop,    // 停止上报
    Report_Info,    // 上报 Info
    Report_Data,    // 上报 Data
}ReportType_t;

extern ReportType_t ReportType;
extern uint32_t msCnt_ModifyGroupID;
extern bool flag_SysReset;

extern char * pAdvName;

extern void update_user_cfg_by_flash_record(void);
extern void save_cfg_update_adv_name(void);

#endif	// __KF_VHUB_H__

