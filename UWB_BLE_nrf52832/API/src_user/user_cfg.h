#ifndef __USER_CFG_H__
#define __USER_CFG_H__

#include "stdint.h"

#include "data_fitting.h"




typedef struct
{
    uint32_t RecordCnt;     // 通过这个值 可知被配置过多少次
    uint8_t GroupID;        // 组编号
    uint8_t NameIndex;      // 设备名称编号
    uint8_t DevID;          // 设备 ID
    
    Least_square_polynomial_coefficient polynomial_coeff_ant_0;
    Least_square_polynomial_coefficient polynomial_coeff_ant_1;
    Least_square_polynomial_coefficient polynomial_coeff_ant_2;
    
    
}UserFlashCfg_Pkg_t;

typedef union
{
    UserFlashCfg_Pkg_t Pkg;
    uint8_t Buf_U8[sizeof(UserFlashCfg_Pkg_t) + ((sizeof(UserFlashCfg_Pkg_t)%4)?(4 - sizeof(UserFlashCfg_Pkg_t)%4):(0))];
    uint8_t Buf_U32[sizeof(UserFlashCfg_Pkg_t) + ((sizeof(UserFlashCfg_Pkg_t)%4)?1:0)];
    
}UserFlashCfg_t;

#define Dev_Chest                   (0)
#define Dev_Wrist                   (Dev_Chest          + 1)
#define Dev_Vehicle                 (Dev_Wrist          + 1)
#define Dev_GPS                     (Dev_Vehicle        + 1)
#define Dev_IMU                     (Dev_GPS            + 1)
#define Dev_OBD                     (Dev_IMU            + 1)
#define Dev_Finger                  (Dev_OBD            + 1)
#define Dev_Ear                     (Dev_Finger         + 1)
#define Dev_Wrist_V3                (Dev_Ear            + 1)
#define Dev_Ear_V2                  (Dev_Wrist_V3       + 1)
#define Dev_Test_MaxSpeed           (Dev_Ear_V2         + 1)
#define Dev_Indoor_Localization     (Dev_Test_MaxSpeed  + 1)

#define DevID_Min                   (0)
#define DevID_Chest                 (DevID_Min)
#define DevID_Wrist                 (DevID_Chest        + 1)
#define DevID_Finger                (DevID_Wrist        + 1)
#define DevID_Ear_001               (DevID_Finger       + 1)
#define DevID_Ear_002               (DevID_Ear_001      + 1)
#define DevID_Wrist_V3              (DevID_Ear_002      + 1)
#define DevID_Ear_V2_001            (DevID_Wrist_V3     + 1)
#define DevID_Ear_V2_002            (DevID_Ear_V2_001   + 1)
#define DevID_Test_MaxSpeed         (DevID_Ear_V2_002   + 1)
#define DevID_Indoor_Localization   (DevID_Test_MaxSpeed + 1)
#define DevID_Max                   (15)

#define DevID_Check(x)              (((x) <= DevID_Max) ? true : false) //((((x) >= DevID_Min) && ((x) <= DevID_Max)) ? true : false)

#define GroupID_Min                 (1)
#define GroupID_Max                 (6)

#define GroupID_Check(x)            ((((x) >= GroupID_Min) && ((x) <= GroupID_Max)) ? true : false)


#define EN_TEST_MAX_SPEED           (0)

#if EN_TEST_MAX_SPEED
    #define CFG_DevType             (Dev_Test_MaxSpeed)
    #define CFG_DevID               (DevID_Test_MaxSpeed)
    
    #define MAX_SPEED_TEST_PKG_LEN  247
#else
    
    //#define CFG_DevType                 (Dev_Wrist)
    //#define CFG_DevID                   (DevID_Wrist)

    //#define CFG_DevType                 (Dev_Ear)
    //#define CFG_DevID                   (DevID_Ear_001)

//#define CFG_DevType                 (Dev_Ear)
//#define CFG_DevID                   (DevID_Ear_002)

//    #define CFG_DevType                 (Dev_Finger)
//    #define CFG_DevID                   (DevID_Finger)

    //#define CFG_DevType                 (Dev_Ear_V2)
    //#define CFG_DevID                   (DevID_Ear_V2_001)

    //#define CFG_DevType                 (Dev_Ear_V2)
    //#define CFG_DevID                   (DevID_Ear_V2_002)

    //#define CFG_DevType                 (Dev_Wrist_V3)
    //#define CFG_DevID                   (DevID_Wrist_V3)

    #define CFG_DevType                 (Dev_Indoor_Localization)
    #define CFG_DevID                   (DevID_Indoor_Localization)
    
    
#endif  // EN_TEST_MAX_SPEED

extern volatile UserFlashCfg_t UserFlashCfg;

extern void user_cfg_fs_init(void);
extern void read_user_cfg_in_flash(void);
extern void user_cfg_write(void);
extern void user_cfg_write_service(void);

#endif	// __USER_CFG_H__

