#include "blue.h"
#include "nrf_log.h"
#include "nrf_uart.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "data_fitting.h"
#include "kf_vhub.h"
#include "user_log.h"
#include "user_cfg.h"
#include "dev.h"

#include "ble_conn_params.h"

#define APP_BLE_CONN_CFG_TAG            1                                   // A tag identifying the SoftDevice BLE configuration

#define NUS_SERVICE_UUID_TYPE           BLE_UUID_TYPE_BLE                   // UUID type for the Nordic UART Service (vendor specific)

#define APP_BLE_OBSERVER_PRIO           3                                   // Application's BLE observer priority. You shouldn't need to modify this value
#define APP_ADV_INTERVAL                64                                  // The advertising interval (in units of 0.625 ms. This value corresponds to 40 ms)
#define APP_ADV_DURATION                18000                               // The advertising duration (180 seconds) in units of 10 milliseconds

#define MIN_CONN_INTERVAL               MSEC_TO_UNITS( 7.500, UNIT_1_25_MS) // Minimum acceptable connection interval (20 ms), Connection interval uses 1.25 ms units
#define MAX_CONN_INTERVAL               MSEC_TO_UNITS(15.625, UNIT_1_25_MS) // Maximum acceptable connection interval (75 ms), Connection interval uses 1.25 ms units
#define SLAVE_LATENCY                   0                                   // Slave latency
#define CONN_SUP_TIMEOUT                MSEC_TO_UNITS(4000, UNIT_10_MS)     // Connection supervisory timeout (4 seconds), Supervision Timeout uses 10 ms units
#define FIRST_CONN_PARAMS_UPDATE_DELAY  APP_TIMER_TICKS(5000)               // Time from initiating event (connect or start of notification) to first time sd_ble_gap_conn_param_update is called (5 seconds)
#define NEXT_CONN_PARAMS_UPDATE_DELAY   APP_TIMER_TICKS(30000 )             // Time between each call to sd_ble_gap_conn_param_update after the first call (30 seconds)
#define MAX_CONN_PARAMS_UPDATE_COUNT    3                                   // Number of attempts before giving up the connection parameter negotiation

#define UART_TX_BUF_SIZE                256                                 // UART TX buffer size
#define UART_RX_BUF_SIZE                256                                 // UART RX buffer size
#define TX_POWER_LEVEL                  (3)                                 // set sending power

BLE_NUS_DEF(m_nus, NRF_SDH_BLE_TOTAL_LINK_COUNT);                           // BLE NUS service instance
NRF_BLE_GATT_DEF(m_gatt);                                                   // GATT module instance
NRF_BLE_QWR_DEF(m_qwr);                                                     // Context for the Queued Write module
BLE_ADVERTISING_DEF(m_advertising);                                         // Advertising module instance

bool flag_ble_connected = false;
bool flag_last_tx_need_retry = false;

static uint16_t   m_conn_handle          = BLE_CONN_HANDLE_INVALID;         // Handle of the current connection
static uint16_t   m_ble_nus_max_data_len = BLE_GATT_ATT_MTU_DEFAULT - 3;    // Maximum length of data (in bytes) that can be transmitted to the peer by the Nordic UART service module

// Universally unique service identifier
static ble_uuid_t m_adv_uuids[] =
{
    // The UUID of the Nordic UART Service
    {
        BLE_UUID_NUS_SERVICE,       // NUS_BASE_UUID
        NUS_SERVICE_UUID_TYPE
    },
    
    // 扫描时组别身份(自定义)
    {
        BLE_UUID_KF_EGL_SCAN,       // KF_EGL_SCAN_BASE_UUID
        BLE_UUID_KF_EGL_SCAN_TYPE
    }
};

//void assert_nrf_callback(uint16_t line_num, const uint8_t *p_file_name);
void timers_init(void);
void gap_params_init(void);
void nrf_qwr_error_handler(uint32_t nrf_error);
void nus_data_handler(ble_nus_evt_t *p_evt);
void services_init(void);
void on_conn_params_evt(ble_conn_params_evt_t *p_evt);
void conn_params_error_handler(uint32_t nrf_error);
void conn_params_init(void);
void sleep_mode_enter(void);
void on_adv_evt(ble_adv_evt_t ble_adv_evt);

void ble_stack_init(void);
void gatt_evt_handler(nrf_ble_gatt_t *p_gatt, nrf_ble_gatt_evt_t const *p_evt);
void gatt_init(void);
void bsp_event_handler(bsp_event_t event);
void uart_event_handle(app_uart_evt_t *p_event);
void advertising_init(void);
void buttons_leds_init(bool *p_erase_bonds);
void power_management_init(void);
void idle_state_handle(void);
void advertising_start(void);
void send_sensor_info(void);

//static void ble_evt_handler(ble_evt_t const *p_ble_evt, void *p_context);

// @brief Function for initializing the timer module.
void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}

/*
    @brief Function for the GAP initialization.
    @details This function will set up all the necessary GAP (Generic Access Profile) parameters of
            the device. It also sets the permissions and appearance.
*/
void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;
    
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    err_code = sd_ble_gap_device_name_set(&sec_mode, (const uint8_t *) pAdvName, strlen(pAdvName));
    APP_ERROR_CHECK(err_code);
    memset(&gap_conn_params, 0, sizeof(gap_conn_params));
    
    #if EN_TEST_MAX_SPEED
        gap_conn_params.min_conn_interval = MSEC_TO_UNITS( 7.500, UNIT_1_25_MS);
        gap_conn_params.max_conn_interval = MSEC_TO_UNITS( 7.500, UNIT_1_25_MS);
    #else
        gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
        gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    #endif  // EN_TEST_MAX_SPEED
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;
    
    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);       //Set GAP Peripheral Preferred Connection Parameters.
    APP_ERROR_CHECK(err_code);
}

/*
    @brief Function for handling Queued Write Module errors.
    @details A pointer to this function will be passed to each service which may need to inform the
            application about an error.
    @param[in]   nrf_error   Error code containing information about what went wrong.
*/
void nrf_qwr_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/*
    @brief Function for handling the data from the Nordic UART Service.
    @details This function will process the data received from the Nordic UART BLE Service and send
            it to the UART module.
    @param[in] p_evt       Nordic UART Service event.
*/
// @snippet [Handling the data received over BLE] */
void nus_data_handler(ble_nus_evt_t *p_evt)
{
    uint8_t i;
    uint8_t* pData = 0;
    uint8_t setting_group_id;
    uint8_t setting_dev_id;
    uint8_t setting_name_index;
    uint8_t Data_Len = 0;
    
    if(p_evt->type == BLE_NUS_EVT_RX_DATA)
    {
        pData = (uint8_t*)p_evt->params.rx_data.p_data;
        Data_Len = p_evt->params.rx_data.length;
        
        log_msg("Data_Len=%d:[", Data_Len);
        
        for(i=0; i<Data_Len; i++)
        {
            log_msg("%02X ", pData[i]);
        }
        
        log_msg("]\r\n");
        
        if(1 == Data_Len)
        {
            switch(pData[0])
            {
                case BleCmd_01_Info:
                {
                    log_connect_pc("cmd 01\r\n");
                    msg_to_master_clear();
                    ReportType = Report_Info;
                }
                break;
                
                case BleCmd_02_Data:
                {
                    log_connect_pc("cmd 02\r\n");
                    msg_to_master_clear();
                    ReportType = Report_Data;
                    pDevInfo->FrameIndex = 0;
                }
                break;
                
                case BleCmd_03_Stop:
                {
                    log_connect_pc("cmd 03\r\n");
                    ReportType = Report_Stop;
                }
                break;
                
                case BleCmd_04_Config:
                {
                    log_connect_pc("cmd 04 %02X %02X\r\n", pData[1], pData[2]);
                    ReportType = Report_Stop;
                    
                    if(     (0x01 == pData[1])
                        &&  (0 == msCnt_ModifyGroupID))
                    {
                        msCnt_ModifyGroupID = 500;
                        UserFlashCfg.Pkg.GroupID = pData[2];
                        save_cfg_update_adv_name();
                    }
                }
                break;
                
                default:
                    break;
            }
        }
        else if(2 == Data_Len)
        {
            switch(pData[0])
            {
                case BleCmd_05_SetGroupID:
                {
                    log_connect_pc("cmd 05 %02X(%d)\r\n", pData[1], pData[1]);
                    
                    setting_group_id = pData[1];
                    
                    if(     (setting_group_id > 0)
                        &&  (setting_group_id <=6)    )
                    {
                        if(UserFlashCfg.Pkg.GroupID == setting_group_id)
                            ;
                        else
                        {
                            ReportType = Report_Stop;
                            
                            msCnt_ModifyGroupID = 500; 
                            UserFlashCfg.Pkg.GroupID = setting_group_id;
                            save_cfg_update_adv_name();
                        }
                    }
                    else
                    {
                        log_msg("err: new_group_id=%d (1~6)\r\n", setting_group_id);
                    }
                }
                break;
                
                case BleCmd_0A_SetDevID:
                {
                    log_connect_pc("cmd 0A %02X(%d)\r\n", pData[1], pData[1]);
                    
                    setting_dev_id = pData[1];
                    
                    if(UserFlashCfg.Pkg.DevID == setting_dev_id)
                        ;
                    else
                    {
                        ReportType = Report_Stop;
                        
                        msCnt_ModifyGroupID = 500;
                        UserFlashCfg.Pkg.DevID = setting_dev_id;
                        save_cfg_update_adv_name();
                    }
                }
                break;
                
                case BleCmd_0B_SetNameIndex:
                {
                    log_connect_pc("cmd 0B %02X(%d)\r\n", pData[1], pData[1]);
                    
                    setting_name_index = pData[1];
                    
                    if(UserFlashCfg.Pkg.NameIndex == setting_name_index)
                        ;
                    else
                    {
                        ReportType = Report_Stop;
                        
                        msCnt_ModifyGroupID = 500;
                        UserFlashCfg.Pkg.NameIndex = setting_name_index;
                        save_cfg_update_adv_name();
                    }
                }
                break;
                
                default:
                    break;
            }
        }
    }
}

// @snippet [Handling the data received over BLE]
// @brief Function for initializing services that will be used by the application.
void services_init(void)
{
    uint32_t           err_code;
    ble_nus_init_t     nus_init;
    nrf_ble_qwr_init_t qwr_init = {0};
    
    // Initialize Queued Write Module.
    qwr_init.error_handler = nrf_qwr_error_handler;
    
    err_code = nrf_ble_qwr_init(&m_qwr, &qwr_init);
    APP_ERROR_CHECK(err_code);
    
    // Initialize NUS.
    memset(&nus_init, 0, sizeof(nus_init));
    
    nus_init.data_handler = nus_data_handler;     //为处理接受到的数据  而调用的时间处理程序
    
    err_code = ble_nus_init(&m_nus, &nus_init);
    APP_ERROR_CHECK(err_code);
}

/*
    @brief Function for handling an event from the Connection Parameters Module.
    @details This function will be called for all events in the Connection Parameters Module
            which are passed to the application.
    @note All this function does is to disconnect. This could have been done by simply setting
         the disconnect_on_fail config parameter, but instead we use the event handler
         mechanism to demonstrate its use.
    @param[in] p_evt  Event received from the Connection Parameters Module.
*/
void on_conn_params_evt(ble_conn_params_evt_t *p_evt)
{
    uint32_t err_code;
    
    if(p_evt->evt_type == BLE_CONN_PARAMS_EVT_FAILED)
    {
        err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_CONN_INTERVAL_UNACCEPTABLE);
        APP_ERROR_CHECK(err_code);
    }
}

/*
    @brief Function for handling errors from the Connection Parameters module.
    @param[in] nrf_error  Error code containing information about what went wrong.
*/
void conn_params_error_handler(uint32_t nrf_error)
{
    APP_ERROR_HANDLER(nrf_error);
}

/*
    @brief Function for initializing the Connection Parameters module.
*/
static void conn_params_init(void)
{
    uint32_t               err_code;
    ble_conn_params_init_t cp_init;
    
    memset(&cp_init, 0, sizeof(cp_init));
    
    cp_init.p_conn_params                  = NULL;
    cp_init.first_conn_params_update_delay = FIRST_CONN_PARAMS_UPDATE_DELAY;
    cp_init.next_conn_params_update_delay  = NEXT_CONN_PARAMS_UPDATE_DELAY;
    cp_init.max_conn_params_update_count   = MAX_CONN_PARAMS_UPDATE_COUNT;
    cp_init.start_on_notify_cccd_handle    = BLE_GATT_HANDLE_INVALID;
    cp_init.disconnect_on_fail             = false;
    cp_init.evt_handler                    = on_conn_params_evt;
    cp_init.error_handler                  = conn_params_error_handler;
    
    err_code = ble_conn_params_init(&cp_init);
    APP_ERROR_CHECK(err_code);
}

/*
    @brief Function for putting the chip into sleep mode.
    @note This function will not return.
*/
void sleep_mode_enter(void)
{
    uint32_t err_code = bsp_indication_set(BSP_INDICATE_IDLE);
    APP_ERROR_CHECK(err_code);
    
    // Prepare wakeup buttons.
    err_code = bsp_btn_ble_sleep_mode_prepare();
    APP_ERROR_CHECK(err_code);
    
    // Go to system-off mode (this function will not return; wakeup will cause a reset).
    err_code = sd_power_system_off();
    APP_ERROR_CHECK(err_code);
}

/*
    @brief Function for handling advertising events.
    @details This function will be called for advertising events which are passed to the application.
    @param[in] ble_adv_evt  Advertising event.
*/
void on_adv_evt(ble_adv_evt_t ble_adv_evt)
{
    uint32_t err_code;
    
    switch(ble_adv_evt)
    {
        case BLE_ADV_EVT_FAST:
            err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
            APP_ERROR_CHECK(err_code);
            break;
            
        case BLE_ADV_EVT_IDLE:
            sleep_mode_enter();
            break;
            
        default:
            break;
    }
}

/*
    @brief Function for handling BLE events.
    @param[in]   p_ble_evt   Bluetooth stack event.
    @param[in]   p_context   Unused.
*/
static void ble_event_handler(ble_evt_t const *p_ble_evt, void *p_context)
{
    uint32_t err_code;
    
//    #if (defined(log_ble_evt))
//    ble_gap_evt_t const *p_gap_evt = &p_ble_evt->evt.gap_evt;
//    #endif  // log_ble_evt
    
    switch(p_ble_evt->header.evt_id)
    {
        case BLE_GAP_EVT_CONNECTED:
        {
            NRF_LOG_INFO("Connected");
            log_msg("connect\r\n");
            flag_ble_connected = true;
            err_code = bsp_indication_set(BSP_INDICATE_CONNECTED);
            APP_ERROR_CHECK(err_code);
            m_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
            err_code = nrf_ble_qwr_conn_handle_assign(&m_qwr, m_conn_handle);
            APP_ERROR_CHECK(err_code);
        }
        break;
            
        case BLE_GAP_EVT_DISCONNECTED:
        {
            NRF_LOG_INFO("Disconnected");
            log_msg("disconnect\r\n");
            flag_ble_connected = false;
            // LED indication will be changed when advertising starts.
            m_conn_handle = BLE_CONN_HANDLE_INVALID;
        }
        break;
        
        case BLE_GAP_EVT_CONN_PARAM_UPDATE:
        {
            //on_conn_params_update(p_ble_evt);
            
//            log_ble_evt("update_param:i_min[%d] i_max[%d] latency[%d] timeout[%d]\r\n",
//                        p_gap_evt->params.conn_param_update_request.conn_params.min_conn_interval,
//                        p_gap_evt->params.conn_param_update_request.conn_params.max_conn_interval,
//                        p_gap_evt->params.conn_param_update_request.conn_params.slave_latency,
//                        p_gap_evt->params.conn_param_update_request.conn_params.conn_sup_timeout);
        }
            
        case BLE_GAP_EVT_DATA_LENGTH_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("DLE update request.");
            
            log_dbg_dis_cnct("BLE_GAP_EVT_DATA_LENGTH_UPDATE_REQUEST\r\n");
            
            ble_gap_data_length_params_t dle_param;
            memset(&dle_param, 0, sizeof(ble_gap_data_length_params_t));   //0 means auto select DLE
            err_code = sd_ble_gap_data_length_update(p_ble_evt->evt.gap_evt.conn_handle, &dle_param, NULL);
            APP_ERROR_CHECK(err_code);
        }
        break;
        
        case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
        {
            NRF_LOG_DEBUG("PHY update request.");
            
            log_dbg_dis_cnct("BLE_GAP_EVT_PHY_UPDATE_REQUEST\r\n");
            
            ble_gap_phys_t const phys =
            {
                .rx_phys = BLE_GAP_PHY_AUTO,
                .tx_phys = BLE_GAP_PHY_AUTO,
            };
            err_code = sd_ble_gap_phy_update(p_ble_evt->evt.gap_evt.conn_handle, &phys);
            APP_ERROR_CHECK(err_code);
        }
        break;
        
        case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
            log_dbg_dis_cnct("BLE_GAP_EVT_SEC_PARAMS_REQUEST\r\n");
            // Pairing not supported
            err_code = sd_ble_gap_sec_params_reply(m_conn_handle, BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
            APP_ERROR_CHECK(err_code);
            break;
            
        case BLE_GATTS_EVT_SYS_ATTR_MISSING:
            log_dbg_dis_cnct("BLE_GATTS_EVT_SYS_ATTR_MISSING\r\n");
            // No system attributes have been stored.
            err_code = sd_ble_gatts_sys_attr_set(m_conn_handle, NULL, 0, 0);
            APP_ERROR_CHECK(err_code);
            break;
            
        case BLE_GATTC_EVT_TIMEOUT:
            log_dbg_dis_cnct("BLE_GATTC_EVT_TIMEOUT\r\n");
            // Disconnect on GATT Client timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gattc_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;
            
        case BLE_GATTS_EVT_TIMEOUT:
            log_dbg_dis_cnct("BLE_GATTS_EVT_TIMEOUT\r\n");
            // Disconnect on GATT Server timeout event.
            err_code = sd_ble_gap_disconnect(p_ble_evt->evt.gatts_evt.conn_handle,
                                             BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            APP_ERROR_CHECK(err_code);
            break;
            
        default:
        {
            // No implementation needed.
            if(0x57 != p_ble_evt->header.evt_id)
                ;   //log_dbg_dis_cnct("ble_evt_id = %d (0x%02X)\r\n", p_ble_evt->header.evt_id, p_ble_evt->header.evt_id);
        }
            break;
    }
}

/*
    @brief Function for the SoftDevice initialization.
    @details This function initializes the SoftDevice and the BLE event interrupt.
*/
void ble_stack_init(void)
{
    ret_code_t err_code;
    
    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);
    
    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);
    
    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);
    
    // Register a handler for BLE events.
    NRF_SDH_BLE_OBSERVER(m_ble_observer, APP_BLE_OBSERVER_PRIO, ble_event_handler, NULL);
}

// @brief Function for handling events from the GATT library
void gatt_evt_handler(nrf_ble_gatt_t *p_gatt, nrf_ble_gatt_evt_t const *p_evt)
{

    if((m_conn_handle == p_evt->conn_handle) && (p_evt->evt_id == NRF_BLE_GATT_EVT_ATT_MTU_UPDATED))
    {
        m_ble_nus_max_data_len = p_evt->params.att_mtu_effective - OPCODE_LENGTH - HANDLE_LENGTH;
        NRF_LOG_INFO("Data len is set to 0x%X(%d)", m_ble_nus_max_data_len, m_ble_nus_max_data_len);
    }
    
    NRF_LOG_DEBUG("ATT MTU exchange completed. central 0x%x peripheral 0x%x",
                  p_gatt->att_mtu_desired_central,
                  p_gatt->att_mtu_desired_periph);
}

// @brief Function for initializing the GATT library
void gatt_init(void)
{
    ret_code_t err_code;
    
    err_code = nrf_ble_gatt_init(&m_gatt, gatt_evt_handler);
    APP_ERROR_CHECK(err_code);
    
    err_code = nrf_ble_gatt_att_mtu_periph_set(&m_gatt, NRF_SDH_BLE_GATT_MAX_MTU_SIZE);
    APP_ERROR_CHECK(err_code);
    
    err_code = nrf_ble_gatt_data_length_set(&m_gatt, BLE_CONN_HANDLE_INVALID, 251);
    APP_ERROR_CHECK(err_code);
}

/*
    @brief Function for handling events from the BSP module.
    @param[in]   event   Event generated by button press.
*/
void bsp_event_handler(bsp_event_t event)
{
    uint32_t err_code;
    
    switch(event)
    {
        case BSP_EVENT_SLEEP:
            sleep_mode_enter();
            break;
            
        case BSP_EVENT_DISCONNECT:
            err_code = sd_ble_gap_disconnect(m_conn_handle, BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
            
            if(err_code != NRF_ERROR_INVALID_STATE)
                APP_ERROR_CHECK(err_code);
                
            break;
            
        case BSP_EVENT_WHITELIST_OFF:
            if(m_conn_handle == BLE_CONN_HANDLE_INVALID)
            {
                err_code = ble_advertising_restart_without_whitelist(&m_advertising);
                
                if(err_code != NRF_ERROR_INVALID_STATE)
                    APP_ERROR_CHECK(err_code);
            }
            
            break;
            
        default:
            break;
    }
}

/*
    @brief   Function for handling app_uart events.
    @details This function will receive a single character from the app_uart module and append it to
            a string. The string will be be sent over BLE when the last character received was a
            'new line' '\n' (hex 0x0A) or if the string has reached the maximum data length.
*/
/**@snippet [Handling the data received over UART] */

uint8_t UartData_UWB_Pkg[BLE_NUS_MAX_DATA_LEN];

uint8_t uart_data_ready=0;

void uart_event_handle(app_uart_evt_t *p_event)
{
    //static uint8_t RxData[BLE_NUS_MAX_DATA_LEN];
	static uint8_t RxData[100];
    static uint8_t index = 0;
    
    switch(p_event->evt_type)
    {
		uint8_t i;
        
        case APP_UART_DATA_READY:
        {
            /*
                IDL UWB Pkg
                00 F0
                01 TagID
                02 PkgID_L
                03 PkgID_H
                04 D0_cm_L
                05 D0_cm_H
                06 D1_cm_L
                07 D1_cm_H
                08 D2_cm_L
                09 D2_cm_H
            */
            UNUSED_VARIABLE(app_uart_get(&RxData[index]));
            
            //app_uart_put(RxData[index]);
            cb_data_fitting_uart_handler(RxData[index]);
            
            #ifdef BLE_SENSOR
            if( (0 == index) && (0xF0 != RxData[index]) )
            {
                index = 0;
            }
            else
            {
                index++;
                if(index >= 16)
                {
                    for(i=0; i<index; i++)
                    {
                        UartData_UWB_Pkg[i] = RxData[i];
                        log_dbg_uwb_uart("%02X ", RxData[i]);
                    }
                    log_dbg_uwb_uart("\r\n");
                    uart_data_ready=1;
                    index = 0;
                }
            }
            #endif
        }
        break;
        
        case APP_UART_COMMUNICATION_ERROR:
        {
            APP_ERROR_HANDLER(p_event->data.error_communication);
        }
        break;
            
        case APP_UART_FIFO_ERROR:
        {
            APP_ERROR_HANDLER(p_event->data.error_code);
        }
        break;
            
        default:
            break;
    }
}

void uart_init(void)
{
    uint32_t err_code;
    
    app_uart_comm_params_t const comm_params =
    {
        .rx_pin_no    = UART_PIN_RX,
        .tx_pin_no    = UART_PIN_TX,
        .rts_pin_no   = RTS_PIN_NUMBER,
        .cts_pin_no   = CTS_PIN_NUMBER,
        .flow_control = APP_UART_FLOW_CONTROL_DISABLED,
        .use_parity   = false,
        #if defined (UART_PRESENT)
        .baud_rate    = UART_BAUD
        #else
        .baud_rate    = NRF_UART_BAUDRATE_57600
        #endif
    };
    
    APP_UART_FIFO_INIT(&comm_params,
                       UART_RX_BUF_SIZE,
                       UART_TX_BUF_SIZE,
                       uart_event_handle,
                       APP_IRQ_PRIORITY_LOWEST,
                       err_code);
    APP_ERROR_CHECK(err_code);
}

void advertising_init(void)
{
    uint32_t               err_code;
    
    //ble_advdata_t           advdata;       // Advertising data: name, appearance, discovery flags, and more
    //ble_adv_modes_config_t  config;        // Select which advertising modes and intervals will be utilized.*/
    
    ble_advertising_init_t init;
    
    memset(&init, 0, sizeof(init));
    
    init.advdata.name_type          = BLE_ADVDATA_FULL_NAME;                                //Include full device name in advertising data
    init.advdata.include_appearance = false;
    init.advdata.flags              = BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE;          // LE Limited Discoverable Mode, BR/EDR not supported
    //init.advdata.p_tx_power_level   = &tx_power_level;                                    // set sending power
    
    
    init.srdata.uuids_complete.uuid_cnt = sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0]);     // Number of UUID entries
    init.srdata.uuids_complete.p_uuids  = m_adv_uuids;                                      // Pointer to UUID array entries
    
    init.config.ble_adv_fast_enabled  = true;                                               // Enable or disable fast advertising mode
    init.config.ble_adv_fast_interval = APP_ADV_INTERVAL;                                   // Advertising interval for fast advertising
    init.config.ble_adv_fast_timeout  = APP_ADV_DURATION;                                   // Time-out (in units of 10ms) for fast advertising
    init.evt_handler = on_adv_evt;
    
    err_code = ble_advertising_init(&m_advertising, &init);                                 //Function for initializing the Advertising Module
    APP_ERROR_CHECK(err_code);
    
    ble_advertising_conn_cfg_tag_set(&m_advertising, APP_BLE_CONN_CFG_TAG);                 //Function for changing the connection settings tag that will be used for upcoming connections
}

void buttons_leds_init(bool *p_erase_bonds)
{
    bsp_event_t startup_event;
    
    uint32_t err_code = bsp_init(BSP_INIT_LEDS | BSP_INIT_BUTTONS, bsp_event_handler);
    APP_ERROR_CHECK(err_code);
    
    err_code = bsp_btn_ble_init(NULL, &startup_event);
    APP_ERROR_CHECK(err_code);
    
    *p_erase_bonds = (startup_event == BSP_EVENT_CLEAR_BONDING_DATA);
}

// @brief Function for initializing the nrf log module
void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
    log_msg("\r\n");
}

void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}

// @brief Function for handling the idle state (main loop)
// @details If there is no pending log operation, then sleep until next the next event occurs

void idle_state_handle(void)
{
    UNUSED_RETURN_VALUE(NRF_LOG_PROCESS());
    nrf_pwr_mgmt_run();
}

// @brief Function for starting advertising
void advertising_start(void)
{
    uint32_t err_code = ble_advertising_start(&m_advertising, BLE_ADV_MODE_FAST); // Fast advertising will connect to any peer device, or filter with a whitelist if one exists
    APP_ERROR_CHECK(err_code);
}

static void tx_power_set(void)
{
    ret_code_t err_code = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, m_advertising.adv_handle, TX_POWER_LEVEL);
    APP_ERROR_CHECK(err_code);
}

//static void throughput_test()
//{
//    ret_code_t err_code;
//    
//    //app_timer_create(&m_timer_speed, APP_TIMER_MODE_REPEATED, throughput_timer_handler);
//    ble_opt_t  opt;
//    memset(&opt, 0x00, sizeof(opt));
//    opt.common_opt.conn_evt_ext.enable = true;
//    err_code = sd_ble_opt_set(BLE_COMMON_OPT_CONN_EVT_EXT, &opt);
//    APP_ERROR_CHECK(err_code);
//}

static void pa_lna_assist(uint32_t gpio_pa_pin, uint32_t gpio_lna_pin)
{
    static const uint32_t gpio_toggle_ch = 0;
    static const uint32_t ppi_set_ch = 0;
    static const uint32_t ppi_clr_ch = 1;

    // Configure SoftDevice PA/LNA assist
    static ble_opt_t opt;
    
    ret_code_t err_code;
    
    nrf_gpio_cfg_output(gpio_pa_pin);
    nrf_gpio_pin_clear(gpio_pa_pin);
    nrf_gpio_cfg_output(gpio_lna_pin);
    nrf_gpio_pin_clear(gpio_lna_pin);
    
    memset(&opt, 0, sizeof(ble_opt_t));
    
    // Common PA/LNA config
    opt.common_opt.pa_lna.gpiote_ch_id  = gpio_toggle_ch;       // GPIOTE channel
    opt.common_opt.pa_lna.ppi_ch_id_clr = ppi_clr_ch;           // PPI channel for pin clearing
    opt.common_opt.pa_lna.ppi_ch_id_set = ppi_set_ch;           // PPI channel for pin setting
    
    // PA config
    opt.common_opt.pa_lna.pa_cfg.active_high    = 1;            // Set the pin to be active high
    opt.common_opt.pa_lna.pa_cfg.enable         = 1;            // Enable toggling
    opt.common_opt.pa_lna.pa_cfg.gpio_pin       = gpio_pa_pin;  // The GPIO pin to toggle

    // LNA config
    opt.common_opt.pa_lna.lna_cfg.active_high   = 1;            // Set the pin to be active high
    opt.common_opt.pa_lna.lna_cfg.enable        = 1;            // Enable toggling
    opt.common_opt.pa_lna.lna_cfg.gpio_pin      = gpio_lna_pin; // The GPIO pin to toggle

    err_code = sd_ble_opt_set(BLE_COMMON_OPT_PA_LNA, &opt);
    APP_ERROR_CHECK(err_code);
}


void ble_init(void)
{
    //bool erase_bonds;
    timers_init();
    //buttons_leds_init(&erase_bonds);
    //power_management_init();                  // 电源管理 如果不用 进入休眠
    ble_stack_init();                           // ble协议栈初始化 
    pa_lna_assist(23,22);
	log_dwm1000_init("stack_init!!\r\n");
    gap_params_init();                          // gap参数初始化部分
    gatt_init();                                // gatt初始化部分
    services_init();                            // 添加蓝牙service和characteristic
    advertising_init();                         // 广播初始化
    conn_params_init();                         // 连接参数初始化
    #ifdef TAG                                  // 不让主机连接上ble功能
    advertising_start();                        // 启动ble广播  
    #endif // TAG
    //throughput_test();
    tx_power_set();                             // 设置ble发送功率
}

void send_data_to_ergolab(void)
{
    uint32_t err_code;
    uint16_t length;
    static uint8_t send_counte = 0;
    
    length = m_ble_nus_max_data_len;
    
    do
    {
        send_counte++;
        
        if(send_counte > 6)
        {
            send_counte = 0;
            break;
        }
        
        err_code = ble_nus_data_send(&m_nus, (uint8_t *)MsgToMaster, &length, m_conn_handle);
        
        if(     (err_code != NRF_ERROR_INVALID_STATE)
            &&  (err_code != NRF_ERROR_RESOURCES)
            &&  (err_code != NRF_ERROR_NOT_FOUND))
        {
            APP_ERROR_CHECK(err_code);
        }
        
        if(NRF_ERROR_RESOURCES == err_code)
        {
            flag_last_tx_need_retry = true;
        }
            
        if(err_code == NRF_SUCCESS)
            send_counte = 0;
            
    } while(err_code != NRF_SUCCESS);
}

void ble_nus_send_data(uint8_t *p_data, uint16_t length)
{
    uint16_t *p_length;
    
    *p_length = length;
    
    ble_nus_data_send(&m_nus, p_data, p_length, m_conn_handle);
}

void update_scan_uuid_by_group_id(uint8_t group_id)
{
    uint16_t last_id = (sizeof(m_adv_uuids) / sizeof(m_adv_uuids[0])) - 1;
    
    m_adv_uuids[last_id].uuid = group_id;
    m_adv_uuids[last_id].uuid <<= 8;
    m_adv_uuids[last_id].uuid |= group_id;
}

void send_sensor_info(void)
{
    /*
        00 01 02         03        04    05   06  07  08
        FF 01 SensorType NameIndex DevID Freq Bat Amp GroupID
    */
    uint16_t length = 32;
    
    MsgToMaster[0] = 0xFF;
    MsgToMaster[1] = 0x01;
    
    MsgToMaster[2] = pDevInfo->DevType;
    
    MsgToMaster[3] = UserFlashCfg.Pkg.NameIndex;
    MsgToMaster[4] = UserFlashCfg.Pkg.DevID;
    
    MsgToMaster[5] = pDevInfo->RptFreq;             // frequency
    MsgToMaster[6] = pDevInfo->Battery;             // BatteryPct
    MsgToMaster[7] = pDevInfo->Amp;                 // Amp
    
    MsgToMaster[8] = UserFlashCfg.Pkg.GroupID;      // GroupID
    
    //err_code = ble_nus_data_send(&m_nus, MsgToMaster, &length, m_conn_handle);
    ble_nus_data_send(&m_nus, (uint8_t *)MsgToMaster, &length, m_conn_handle);
    
    log_cfg("slv_rpt_DevID=%d\r\n", UserFlashCfg.Pkg.DevID);
}

void send_info_to_ErgoLAB(void)
{
    if(Report_Info == ReportType)
    {
        ReportType = Report_Stop;
        send_sensor_info();
    }
}

