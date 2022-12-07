/*
 * nbiot_poll_mgmt.c
 *
 * Created: 9/08/2018 9:11:21 AM
 *  Author: fook
 */

#include <stdio.h>
#include <string.h>
#include "nbiot_radio_priv.h"
#include "rtc_api.h"
#include "nbiot_poll_mgmt.h"
#include "nbiot_util.h"
#include "nbiot_lgup_report.h"
#include "plat-time.h"
#include "op-mode-params.h"
#include "schedule.h"
#include "battery-gauge.h"
#include "report_sync.h"
#include "cfgmgt.h"
#undef  MODULE_NAME
#define MODULE_NAME     "pllmgr"
#include "debug.h"

#include "x2m_hal_utils.h"

#undef NBIOT_CROWBAR
#define NBIOT_CROWBAR
#ifdef NBIOT_CROWBAR
extern void nbiotUpdateCrowbarTimer(void);
#endif // NBIOT_CROWBAR

// Module level debug configuration - Must be defined before including debug.h
// 8 Character tag used to identify source of console log message.
#undef  MODULE_NAME
#define MODULE_NAME     "pollmgr"
// local override of global threshold for console log messages.
#undef  PRINT_LEVEL
#define PRINT_LEVEL     PLAT_DEBUG
#include "debug.h"
//-------------------------------------------------------------------------------------------------
process_event_t                     nwk_statistics_ready_event = PROCESS_EVENT_NONE;
#define NWK_STATISTICS_READY_EVENT  (nwk_statistics_ready_event ? nwk_statistics_ready_event : 0)
#define DEFAULT_POLL_REQUEST_INTERVAL	(10 * 60)
struct time_value                   poll_timer_interval = {.s = DEFAULT_POLL_REQUEST_INTERVAL, .ms = 0, .us = 0};
struct rtc_timer                    nbiot_poll_timer;
static uint32_t                     poll_timer_offset = 0;
//-------------------------------------------------------------------------------------------------
process_event_t                     lgup_quality_report_event  = PROCESS_EVENT_NONE;
#define LGUP_QUALITY_REPORT_EVENT   (lgup_quality_report_event ? lgup_quality_report_event : 0)
#define LGUP_QREPORT_INTERVAL	    (24 * 60 * 60)
//#define LGUP_QREPORT_INTERVAL	    (10 * 60)
struct time_value                   lgup_qreport_timer_interval = {.s = LGUP_QREPORT_INTERVAL, .ms = 0, .us = 0};
static struct rtc_timer             lgup_qreport_poll_timer;

#define NBIOT_POLL_PROC_NAME	"Poll Request Mgr"
PROCESS (nbiot_poll_mgr, NBIOT_POLL_PROC_NAME);

//-------------------------------------------------------------------------------------------------
static void pollmgr_init(void)
{
    x2m_printf(PLAT_DEBUG, MODULE_NAME, "Starting poll request manager...\n");
    if (nwk_statistics_ready_event == PROCESS_EVENT_NONE)
    {
        nwk_statistics_ready_event = process_alloc_event();
    }

    const op_mode_param_t* param    = op_mode_hdlr.op_mode_params_get_current();
    poll_timer_interval.s           = (uint32_t)param->poll_interval;
    poll_timer_offset               = (uint32_t)param->poll_offset;
    report_sync_update_timer("DReq", &nbiot_poll_timer, poll_timer_interval.s, poll_timer_offset, RS_TMR_REALIGN);

    if (lgup_quality_report_event == PROCESS_EVENT_NONE)
    {
        lgup_quality_report_event = process_alloc_event();
    }

    //Removal of LGU+ Signal Quality reports
    //report_sync_update_timer("LGU+", &lgup_qreport_poll_timer, lgup_qreport_timer_interval.s, 0, RS_TMR_REALIGN);
}

//-------------------------------------------------------------------------------------------------
static bool pollmgr_check_for_poll_event(void* pdata)
{
    bool ret = false;
    if ((pdata == &nbiot_poll_timer) && rtc_timer_expired(&nbiot_poll_timer))
    {
        ret = true;
    }
    return ret;
}

//-------------------------------------------------------------------------------------------------
void pollmgr_nwk_statistics_request(void)
{
    MacMetadata_t md;

    memset((void *)&md, 0, sizeof(MacMetadata_t));
    md.endpoint = EP_MAC;
    md.lifetime = 3600;
    md.priority = 0;
    x2m_printf(PLAT_INFO, MODULE_NAME, "Requesting network statistics information\n");
    MacRequest(MAC_STATISTICS_GET, (MacMetadata_t*)&md, NULL, NULL);
}

//-------------------------------------------------------------------------------------------------
// Send a data request complete with some statistics
static void nb_send_dr_and_stats(process_data_t *evtData)
{
    MacResult_t     result;
    uint8_t         NBIOT_DR_MSG_LEN = (11 + 2 + 2 + 2 + 2 + 4);
    size_t          len = NBIOT_DR_MSG_LEN;
    MacMetadataTx_t reqTx;
    memset(&reqTx, 0, sizeof(MacMetadataTx_t));
    reqTx.endpoint = EP_MAC;

    uint8_t data[SIZE_MAX_CLIENT_PAYLOAD];
    uint8_t *pointer = data;

    *pointer++ = NBIOT_DR_COMMAND_ID;
    *pointer++ = LORA_METER_STATUS_SUCCESS;
    uint32_t rtc_timestamp =  plat_get_time_epoch();
    memcpy((void *)pointer, (void *)&rtc_timestamp, sizeof(uint32_t));
    pointer += sizeof(uint32_t);

    *pointer++ = battery_get_percentage();

    // Get radio statistics information.
    nuestats_t stats;
    memset(&stats, 0, sizeof(stats));
    if (evtData)
    {
        memcpy((void *)&stats, (void *)evtData, sizeof(nuestats_t));
    }

    // Convert rsrq value to signed 16 bit.
    int16_t rssi = (int16_t)((stats.cell_rssi) & 0x0000FFFF);
    memcpy((void *)pointer, (void *)&rssi, sizeof(int16_t));
    pointer += sizeof(int16_t);

    // Convert snr value to unsigned 16 bit.
    uint16_t snr = (uint16_t)((stats.cell_snr) & 0x0000FFFF);
    memcpy((void *)pointer, (void *)&snr, sizeof(uint16_t));
    pointer += sizeof(uint16_t);

    uint16_t signal_power = (uint16_t)((stats.radio_signal_power) & 0x0000FFFF);
    memcpy((void *)pointer, (void *)&signal_power, sizeof(uint16_t));
    pointer += sizeof(uint16_t);

    uint16_t total_power = (uint16_t)((stats.radio_total_power) & 0x0000FFFF);
    memcpy((void *)pointer, (void *)&total_power, sizeof(uint16_t));
    pointer += sizeof(uint16_t);

    uint16_t tx_power = (uint16_t)((stats.radio_tx_power) & 0x0000FFFF);
    memcpy((void *)pointer, (void *)&tx_power, sizeof(uint16_t));
    pointer += sizeof(uint16_t);


    uint16_t physical_cell_id = (uint16_t)(stats.physical_cell_id);
    memcpy((void *)pointer, (void *)&physical_cell_id, sizeof(uint16_t));
    pointer += sizeof(uint16_t);

    uint64_t cell_id = (uint64_t)(stats.cell_id);
    memcpy((void *)pointer, (void *)&cell_id, sizeof(uint64_t));
    pointer += sizeof(uint64_t);

    result = MacRequest(MAC_DATA_TX, (MacMetadataTx_t*)&reqTx, (void*)data, &len);
    if (result == RC_INT_SUCCESS)
    {
        x2m_printf(PLAT_INFO, MODULE_NAME, "Sending DATA_REQUEST\n");
    }
    else
    {
        x2m_printf(PLAT_ERR, MODULE_NAME, "DATA_REQUEST submission error: %d\n", result);
    }
}

//-------------------------------------------------------------------------------------------------
void nbiot_pollmgr_start(void)
{
    process_start(&nbiot_poll_mgr, NULL);
}

//-------------------------------------------------------------------------------------------------
void nbiot_pollmgr_stop(void)
{
    struct process *proc;
    x2m_printf(PLAT_DEBUG, MODULE_NAME, "Stop poll request manager...\n");
    for(proc = PROCESS_LIST(); proc != NULL; proc = proc->next)
    {
        char namebuf[50];
        strncpy(namebuf, PROCESS_NAME_STRING(proc), sizeof(namebuf));
        if (!strncmp(NBIOT_POLL_PROC_NAME,  namebuf, strlen(NBIOT_POLL_PROC_NAME)))
        {
            x2m_printf(PLAT_DEBUG, MODULE_NAME, "Stopping %s process...\n", namebuf);
            process_exit(proc);
        }
    }
}

//-------------------------------------------------------------------------------------------------
static void pollmgr_handle_poll_event(void* pdata)
{
    // Timer event to trigger periodic data request messages upstream
    if (pollmgr_check_for_poll_event(pdata))
    {
        report_sync_update_timer("DReq", &nbiot_poll_timer, poll_timer_interval.s, poll_timer_offset, RS_TMR_ALIGNED);

		if (isNwkUp())
		{
		    if(isStartUpProcessOngoing())
			{
			    x2m_printf(PLAT_INFO, MODULE_NAME,"Skipping NW statsuery (for DATA_REQ) message as start-up procedure is ongoing\n");
			}
			else
			{
			    // Request network statistics information to include in the data request.
			    pollmgr_nwk_statistics_request();
			}
		}
        else
        {
            #ifdef NBIOT_CROWBAR
                // Prevent crowbar rebooting NIC while network is down.
                nbiotUpdateCrowbarTimer();
            #endif // NBIOT_CROWBAR
        }
    }
}

//-------------------------------------------------------------------------------------------------
static bool pollmgr_check_for_lgup_qrep_event(void* pdata)
{
    return ( (pdata == &lgup_qreport_poll_timer) && rtc_timer_expired(&lgup_qreport_poll_timer) );
}

//-------------------------------------------------------------------------------------------------
static void pollmgr_handle_lgup_qrep_event(void* pdata)
{
    // Timer event to trigger periodic data request messages upstream
    if (pollmgr_check_for_lgup_qrep_event(pdata))
    {
        //Removal of LGU+ Signal Quality reports
        //send_lgu_quality_report(true);
        //report_sync_update_timer("LGU+", &lgup_qreport_poll_timer, lgup_qreport_timer_interval.s, 0, RS_TMR_ALIGNED);
    }
}

//-------------------------------------------------------------------------------------------------
// PROCESS
//-------------------------------------------------------------------------------------------------
PROCESS_THREAD (nbiot_poll_mgr, ev, data)
{
    PROCESS_BEGIN ();

    // Start default poll request timer
    pollmgr_init();

    x2m_printf(PLAT_DEBUG, MODULE_NAME, "sending AT+NUESTAT to provide RSSI & SNR information for ATE test\n"); //[NN-729]
    pollmgr_nwk_statistics_request();

    while(1)
    {
        PROCESS_WAIT_EVENT ();
        if (ev == RTC_TIMER_EVENT)
        {
            pollmgr_handle_poll_event(data);
            pollmgr_handle_lgup_qrep_event(data);
        }

        if (ev == NWK_STATISTICS_READY_EVENT)
        {
            //-------------------------------------------------------------------------------------
            // @testpoint: data request, landing point after attach complete. Test reboot after attach here
            //-------------------------------------------------------------------------------------

            nb_send_dr_and_stats(data);                 // Send out a data request
            //x2m_delay_s(3);
            send_lgu_quality_report(false);             // LGU+ only
            x2m_delay_s(1);
            bc95_query_cereg_network_registration();    // poll manager
        }

        if ((ev == NIC_OP_MODE_CHANGED_EVENT)  || (ev == NIC_PROV_MODE_END_EVENT))
        {
            if (ev == NIC_OP_MODE_CHANGED_EVENT)
            {
                x2m_printf(PLAT_INFO, MODULE_NAME, "Event received:OP_MODE_CHANGED_EVENT\n");
            }
            else
            {
                x2m_printf(PLAT_INFO, MODULE_NAME, "Event received:PROV_MODE_END_EVENT\n");
            }

            const op_mode_param_t* param = op_mode_hdlr.op_mode_params_get_current();
            poll_timer_interval.s = param->poll_interval;
            poll_timer_offset = param->poll_offset;
			x2m_printf(PLAT_INFO, MODULE_NAME, "Poll Interval = %d seconds\n", poll_timer_interval.s);
			x2m_printf(PLAT_INFO, MODULE_NAME, "Poll Offset = %d seconds\n", poll_timer_offset);

            report_sync_update_timer("DReq", &nbiot_poll_timer, poll_timer_interval.s, poll_timer_offset, RS_TMR_REALIGN);

            #ifdef NBIOT_CROWBAR
                nbiotUpdateCrowbarTimer();
            #endif // NBIOT_CROWBAR
        }

        if (ev == REPORT_RESYNC_NOTIFICATION_EVENT)
        {
            x2m_printf(PLAT_INFO, MODULE_NAME, "Event received:REPORT_RESYNC_NOTIFICATION_EVENT\n");
            report_sync_update_timer("DReq", &nbiot_poll_timer, poll_timer_interval.s, poll_timer_offset, RS_TMR_REALIGN);
        }

        if (ev == EVT_NWK_RECONNECT)
        {
            //x2m_printf(PLAT_INFO, MODULE_NAME, "Event received: EVT_NWK_RECONNECT\n");
            x2m_printf(PLAT_INFO, MODULE_NAME, "Poll Interval = %d seconds\n", poll_timer_interval.s);
            x2m_printf(PLAT_INFO, MODULE_NAME, "Poll Offset = %d seconds\n", poll_timer_offset);
            report_sync_update_timer("DReq", &nbiot_poll_timer, poll_timer_interval.s, poll_timer_offset, RS_TMR_REALIGN);
        }
    }
    PROCESS_END ();
}

