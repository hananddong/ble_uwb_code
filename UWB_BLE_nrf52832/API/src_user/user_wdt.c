#include "user_wdt.h"
#include "nrfx_wdt.h"

nrfx_wdt_channel_id m_channel_id;

void wdt_event_handler(void);

void wdt_event_handler(void)
{
    //NOTE: The max amount of time we can spend in WDT interrupt is two cycles of 32768[Hz] clock - after that, reset occurs
}

void user_wdt_init(void)
{
    uint32_t err_code = NRF_SUCCESS;
    
    //Configure WDT.
    nrfx_wdt_config_t config = NRFX_WDT_DEAFULT_CONFIG;
    err_code = nrfx_wdt_init(&config, wdt_event_handler);
    APP_ERROR_CHECK(err_code);
    err_code = nrfx_wdt_channel_alloc(&m_channel_id);
    APP_ERROR_CHECK(err_code);
    nrfx_wdt_enable();
}

void user_wdt_feed(void)
{
    nrfx_wdt_feed();
}

