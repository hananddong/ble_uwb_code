#include "user_cfg.h"
#include "nrf_nvmc.h"
#include "user_log.h"
#include "nrf_log.h"
#include "stdbool.h"
#include "nrf_fstorage.h"
#include "nrf_fstorage_sd.h"

#define USER_CFG_ADDR_ST       (127 * 4 * 1024)
#define USER_CFG_ADDR_SP       (128 * 4 * 1024)

UserFlashCfg_t volatile UserFlashCfg =
{
    .Pkg.RecordCnt = 0xFFFFFFFF,
    .Pkg.GroupID = 0xFF,
    .Pkg.polynomial_coeff_ant_0.g_parak = {0},
    .Pkg.polynomial_coeff_ant_1.g_parak = {0},
    .Pkg.polynomial_coeff_ant_2.g_parak = {0},
};

typedef struct
{
    bool need_write;                            //写标志
    bool need_read;                             //读标志
    bool busy;                                  //FS忙标志
}my_fs_info_t;

my_fs_info_t my_fs_info =
{
    .need_write = false,
    .need_read = false,
    .busy = false,
};

void fs_evt_handle(nrf_fstorage_evt_t *p_evt);

NRF_FSTORAGE_DEF(nrf_fstorage_t my_fs) =
{
    //.p_api = NULL,
    //.p_flash_info = NULL,
    
    .evt_handler = fs_evt_handle,
    .start_addr = USER_CFG_ADDR_ST,
    .end_addr   = USER_CFG_ADDR_SP,
};

void fs_evt_handle(nrf_fstorage_evt_t *p_evt)
{
	if (p_evt->result != NRF_SUCCESS)
    {
        NRF_LOG_INFO("--> Event received: ERROR while executing an fstorage operation.");
        return;
    }
    
    switch (p_evt->id)
    {
        case NRF_FSTORAGE_EVT_WRITE_RESULT:     // FS 写完成事件
        {
            my_fs_info.busy = false;  
        } break;
        
        case NRF_FSTORAGE_EVT_ERASE_RESULT:     // 擦除完成事件
        {
            my_fs_info.busy = false; 
        } break;

        default:
            break;
    }
}

void user_cfg_fs_init(void)                     // FLASH存储初始化  
{
    ret_code_t rc;
    
	nrf_fstorage_api_t * p_fs_api;
	p_fs_api = &nrf_fstorage_sd;
    
	//初始化FS
	rc = nrf_fstorage_init(&my_fs, p_fs_api, NULL);
    APP_ERROR_CHECK(rc);
}

void read_user_cfg_in_flash(void)               // 读用户参数  
{
    nrf_fstorage_read(&my_fs, USER_CFG_ADDR_ST, (void*)UserFlashCfg.Buf_U8, sizeof(UserFlashCfg_t));
    #ifdef  CFG_LOG_TEST
    log_msg("0 %lf \r\n",UserFlashCfg.Pkg.polynomial_coeff_ant_0.g_parak[0]);
    log_msg("1 %lf \r\n",UserFlashCfg.Pkg.polynomial_coeff_ant_0.g_parak[1]);
    log_msg("2 %lf \r\n",UserFlashCfg.Pkg.polynomial_coeff_ant_0.g_parak[2]);
    log_msg("3 %lf \r\n",UserFlashCfg.Pkg.polynomial_coeff_ant_0.g_parak[3]);
    log_msg("4 %lf \r\n",UserFlashCfg.Pkg.polynomial_coeff_ant_0.g_parak[4]);
    log_msg("5 %lf \r\n",UserFlashCfg.Pkg.polynomial_coeff_ant_0.g_parak[5]);
    log_msg("a0 %lf \r\n",UserFlashCfg.Pkg.polynomial_coeff_ant_1.g_parak[0]);
    log_msg("a1 %lf \r\n",UserFlashCfg.Pkg.polynomial_coeff_ant_1.g_parak[1]);
    log_msg("a2 %lf \r\n",UserFlashCfg.Pkg.polynomial_coeff_ant_1.g_parak[2]);
    log_msg("a3 %lf \r\n",UserFlashCfg.Pkg.polynomial_coeff_ant_1.g_parak[3]);
    log_msg("a4 %lf \r\n",UserFlashCfg.Pkg.polynomial_coeff_ant_1.g_parak[4]);
    log_msg("a5 %lf \r\n",UserFlashCfg.Pkg.polynomial_coeff_ant_1.g_parak[5]);
    log_msg("b0 %lf \r\n",UserFlashCfg.Pkg.polynomial_coeff_ant_2.g_parak[0]);
    log_msg("b1 %lf \r\n",UserFlashCfg.Pkg.polynomial_coeff_ant_2.g_parak[1]);
    log_msg("b2 %lf \r\n",UserFlashCfg.Pkg.polynomial_coeff_ant_2.g_parak[2]);
    log_msg("b3 %lf \r\n",UserFlashCfg.Pkg.polynomial_coeff_ant_2.g_parak[3]);
    log_msg("b4 %lf \r\n",UserFlashCfg.Pkg.polynomial_coeff_ant_2.g_parak[4]);
    log_msg("b5 %lf \r\n",UserFlashCfg.Pkg.polynomial_coeff_ant_2.g_parak[5]);
    #endif
    get_eepprom_parameters();
    
}





void user_cfg_write(void)                        // 写用户参数  
{
    ret_code_t rc;
    
    // erase page
    if(!my_fs_info.busy)
    {
        my_fs_info.busy = true;
        
        // nrf_fstorage_erase(nrf_fstorage_t const *p_fs, uint32_t page_addr, uint32_t len, void *p_context)
        rc = nrf_fstorage_erase(&my_fs, USER_CFG_ADDR_ST, 1, NULL);
        APP_ERROR_CHECK(rc);
        
        log_cfg("erase_cfg\r\n");
    }
    else
        log_cfg("erase_busy\r\n");
    
    // fill write parameter
    my_fs_info.need_write = true;
}

void user_cfg_write_service(void)
{
    ret_code_t rc;
    
    if(!my_fs_info.busy)
    {
        // 写入
        if(my_fs_info.need_write)
        {
            my_fs_info.busy = true;
            my_fs_info.need_write = false;
            
            rc = nrf_fstorage_write(&my_fs, USER_CFG_ADDR_ST, (void*)UserFlashCfg.Buf_U8, sizeof(UserFlashCfg_t), NULL);
            
            if(NRF_SUCCESS != rc)
                log_cfg("write_fail: %d(0x%02X)\r\n", rc, rc);
            else
            {
                log_cfg("save_cfg_success cnt=%d(0x%08X) group_id=%d dev_id=%d name_id=%d\r\n",
                        UserFlashCfg.Pkg.RecordCnt,
                        UserFlashCfg.Pkg.RecordCnt,
                        UserFlashCfg.Pkg.GroupID,
                        UserFlashCfg.Pkg.DevID,
                        UserFlashCfg.Pkg.NameIndex);
            }
        }
        
        // 读取
        else if(my_fs_info.need_read)
        {
            my_fs_info.need_read = false;
            
            rc = nrf_fstorage_read(&my_fs, USER_CFG_ADDR_ST, (void*)UserFlashCfg.Buf_U8, sizeof(UserFlashCfg_t));
            
            if(NRF_SUCCESS != rc)
                log_cfg("read_fail: %d(0x%02X)\r\n", rc, rc);
            else
            {
                log_cfg("get_cfg_success cnt=%d(0x%08X) group_id=%d(0x%02X)\r\n",
                        UserFlashCfg.Pkg.RecordCnt,
                        UserFlashCfg.Pkg.RecordCnt,
                        UserFlashCfg.Pkg.GroupID,
                        UserFlashCfg.Pkg.GroupID);
            }
        }
    }
}

