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

#define StrLen(x)   (sizeof(x) - 1)     // ȥ�� '\0' ��ռ����

static char AdvName[StrLen(Adv_Prefix) + StrLen(Adv_GroupID) + StrLen(AdvName_Type) + StrLen(Adv_DevID) + 1];

char * pAdvName = 0;

void update_device_adv_name(void);
void target_device_name_log(void);

void save_cfg_update_adv_name(void)
{
    /*
        ���� BLE ʱ ���²��� Flash �ᵼ�³��� ʹ��ָ���ȡͬ�������
        
        //nrf_nvmc_page_erase(USER_CFG_ADDR_ST);
        
        // nrf_nvmc_write_words(uint32_t address, const uint32_t *src, uint32_t num_words)
        //nrf_nvmc_write_words(USER_CFG_ADDR_ST, (const uint32_t*)UserFlashCfg.Buf_U32, sizeof(UserFlashCfg)/4);
    */
    
    UserFlashCfg.Pkg.RecordCnt += 1;
    
    update_device_adv_name();
    
    user_cfg_write();
    
    user_cfg_write_service();                      // ���������ڼ� 
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
        if(UserFlashCfg.Pkg.GroupID / 100)         // �ж�λ����Ȼ��ת��Ϊ�ַ��� 
            sprintf((char*)Adv_GroupID, "_%3d", UserFlashCfg.Pkg.GroupID);
        else if(UserFlashCfg.Pkg.GroupID / 10)
            sprintf((char*)Adv_GroupID, "_%2d", UserFlashCfg.Pkg.GroupID);
        else
            sprintf((char*)Adv_GroupID, "_%d", UserFlashCfg.Pkg.GroupID);
        
        for(j=0; j<strlen(Adv_GroupID); j++)
            AdvName[i++] = Adv_GroupID[j];
    }
    
    // DevID
    if(UserFlashCfg.Pkg.DevID / 100)               // �ж�λ����Ȼ��ת��Ϊ�ַ���             
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
        #define  DEFAULT_DevGroupID  0x0a   // ����ʹ��
    #else
        #define  DEFAULT_DevGroupID  0x01   // 
    #endif
    static bool Flash_Cfg_Need_Save = false;
    
    // // ��ȡ Flash ����
    // read_user_cfg_in_flash();
    
    log_cfg("flash_record cnt=%d(0x%08X) group_id=%d dev_id=%d name_id=%d\r\n",
            UserFlashCfg.Pkg.RecordCnt,
            UserFlashCfg.Pkg.RecordCnt,
            UserFlashCfg.Pkg.GroupID,
            UserFlashCfg.Pkg.DevID,
            UserFlashCfg.Pkg.NameIndex);
    
    // DevID check
    if( !DevID_Check(UserFlashCfg.Pkg.DevID))              // �ж��豸��ID �Ƿ��ں���Χ֮��
    {
        UserFlashCfg.Pkg.DevID = CFG_DevID;
        Flash_Cfg_Need_Save = true;
    }
    
    // GroupID check
    if( !GroupID_Check(UserFlashCfg.Pkg.GroupID))          // �жϷ�����Ϣ�Ƿ��Ǻ���Χ֮��  
    {
        UserFlashCfg.Pkg.GroupID = DEFAULT_DevGroupID;     // Ĭ��(���� Flash �ձ����������û�м�¼)��Ϊ�� 1
        Flash_Cfg_Need_Save = true;
    }
    
    update_scan_uuid_by_group_id(UserFlashCfg.Pkg.GroupID);// ���ù㲥��Ϣ������������ֵ� UUID ֵ
    
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

