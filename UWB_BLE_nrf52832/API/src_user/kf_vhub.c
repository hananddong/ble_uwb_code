#include "kf_vhub.h"
#include "user_cfg.h"
#include "string.h"
#include "user_log.h"
#include "blue.h"
#include "dev.h"

ReportType_t ReportType = Report_Stop;
uint32_t msCnt_ModifyGroupID = 0;
bool flag_SysReset = false;

static char Adv_Prefix[]    = {"EGL"};
static char Adv_GroupID[]   = {"_000"};
static char Adv_DevID[]     = {"_000"};

#if(Dev_Chest == CFG_DevType)
    static char AdvName_Type[]      = {"_Chest"};

#elif(Dev_Wrist == CFG_DevType)
    static char AdvName_Type[]      = {"_Wrist"};

#elif(Dev_Finger == CFG_DevType)
    static char AdvName_Type[]      = {"_Finger"};

#elif(Dev_Ear == CFG_DevType)
    static char AdvName_Type[]      = {"_Ear"};

#elif(Dev_Wrist_V3 == CFG_DevType)
    static char AdvName_Type[]      = {"_Wrist_V3"};

#elif(Dev_Ear_V2 == CFG_DevType)
    static char AdvName_Type[]      = {"_Ear_V3"};

#elif (Dev_Test_MaxSpeed == CFG_DevType)
    static char AdvName_Type[]      = {"_TestMaxSpeed"};
    
#elif (Dev_Indoor_Localization == CFG_DevType)
    static char AdvName_Type[]      = {"_IndoorLocal"};
    
#else
    #error "please check CFG_DevType and CFG_DevID"
#endif

#define StrLen(x)   (sizeof(x) - 1)     // 去掉 '\0' 所占长度

static char AdvName[StrLen(Adv_Prefix) + StrLen(Adv_GroupID) + StrLen(AdvName_Type) + StrLen(Adv_DevID) + 1];

char * pAdvName = 0;

void update_device_adv_name(void);
void target_device_name_log(void);

void save_cfg_update_adv_name(void)
{
    /*
        运行 BLE 时 如下操作 Flash 会导致出错 使用指针读取同样会出错
        
        //nrf_nvmc_page_erase(USER_CFG_ADDR_ST);
        
        // nrf_nvmc_write_words(uint32_t address, const uint32_t *src, uint32_t num_words)
        //nrf_nvmc_write_words(USER_CFG_ADDR_ST, (const uint32_t*)UserFlashCfg.Buf_U32, sizeof(UserFlashCfg)/4);
    */
    
    UserFlashCfg.Pkg.RecordCnt += 1;
    
    update_device_adv_name();
    
    user_cfg_write();
    
    user_cfg_write_service();                      // 放置在这期间 
}

void update_device_adv_name(void)
{
    uint16_t i, j;
    
    // EGL
    for(i=0; i<strlen(Adv_Prefix); i++)
        AdvName[i] = Adv_Prefix[i];
    
    // DevName
    for(j=0; j<strlen(AdvName_Type); j++)
        AdvName[i++] = AdvName_Type[j];
    
    // GroupID
    if(0xFF != UserFlashCfg.Pkg.GroupID)
    {
        if(UserFlashCfg.Pkg.GroupID / 100)         // 判断位数，然后转换为字符串 
            sprintf((char*)Adv_GroupID, "_%3d", UserFlashCfg.Pkg.GroupID);
        else if(UserFlashCfg.Pkg.GroupID / 10)
            sprintf((char*)Adv_GroupID, "_%2d", UserFlashCfg.Pkg.GroupID);
        else
            sprintf((char*)Adv_GroupID, "_%d", UserFlashCfg.Pkg.GroupID);
        
        for(j=0; j<strlen(Adv_GroupID); j++)
            AdvName[i++] = Adv_GroupID[j];
    }
    
    // DevID
    if(UserFlashCfg.Pkg.DevID / 100)               // 判断位数，然后转换为字符串             
        sprintf((char*)Adv_DevID, "_%3d", UserFlashCfg.Pkg.DevID);
    else if(UserFlashCfg.Pkg.DevID / 10)
        sprintf((char*)Adv_DevID, "_%2d", UserFlashCfg.Pkg.DevID);
    else
        sprintf((char*)Adv_DevID, "_%d", UserFlashCfg.Pkg.DevID);   // CFG_DevID);
    
    for(j=0; j<strlen(Adv_DevID); j++)
        AdvName[i++] = Adv_DevID[j];
    
    AdvName[i] = '\0';
}

void update_user_cfg_by_flash_record(void)
{
    #define BLE_MASTER_TEST_DEBUG
    #ifdef BLE_MASTER_TEST_DEBUG
        #define  DEFAULT_DevGroupID  0x0a   // 调试使用
    #else
        #define  DEFAULT_DevGroupID  0x01   // 
    #endif
    static bool Flash_Cfg_Need_Save = false;
    
    // // 读取 Flash 配置
    // read_user_cfg_in_flash();
    
    log_cfg("flash_record cnt=%d(0x%08X) group_id=%d dev_id=%d name_id=%d\r\n",
            UserFlashCfg.Pkg.RecordCnt,
            UserFlashCfg.Pkg.RecordCnt,
            UserFlashCfg.Pkg.GroupID,
            UserFlashCfg.Pkg.DevID,
            UserFlashCfg.Pkg.NameIndex);
    
    // DevID check
    if( !DevID_Check(UserFlashCfg.Pkg.DevID))              // 判断设备的ID 是否在合理范围之内
    {
        UserFlashCfg.Pkg.DevID = CFG_DevID;
        Flash_Cfg_Need_Save = true;
    }
    
    // GroupID check
    if( !GroupID_Check(UserFlashCfg.Pkg.GroupID))          // 判断分组信息是否是合理范围之内  
    {
        UserFlashCfg.Pkg.GroupID = DEFAULT_DevGroupID;     // 默认(比如 Flash 刚被擦除而造成没有记录)归为组 1
        Flash_Cfg_Need_Save = true;
    }
    
    update_scan_uuid_by_group_id(UserFlashCfg.Pkg.GroupID);// 配置广播信息中用于组别区分的 UUID 值
    
    update_device_adv_name();
    
    pAdvName = AdvName;
    
    target_device_name_log();
    
    if(Flash_Cfg_Need_Save)
    {
        log_msg("save_cfg\r\n");
        save_cfg_update_adv_name();
    }
}

extern void target_device_name_log(void)
{
    log_msg("AdvName: %s\r\n", (char*)pAdvName);
}

