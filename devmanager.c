//-------------------------------------------------------------------------------------------------------------------------------------------
// Module            Contiki device manager.
// Description       branch off
// Project           Freestyle NBIOT
// Copyright         Freestyle Technology 2016.
//-------------------------------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------------------------------
// Includes
//-------------------------------------------------------------------------------------------------------------------------------------------
#include <string.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdint.h>
#include "rtc_api.h"
#include "devmanager.h"
#include "button-sensor.h"
#include "serial-line.h"
//#include "platform-driver.h"  // For platform specific actions.
#include "cfgmgt.h"
#include "secom.h"
#include "energmon.h"
#include "mem_addresses.h"
#include "debug.h"
#include "lpm-manager.h"

#include "x2m_hal_init.h"
#include "x2m_hal_debug.h"
#include "x2m_hal_utils.h"
#include "tamper_sensor/tamper_sensor.h"

//-------------------------------------------------------------------------------------------------------------------------------------------
// Structures
//-------------------------------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------------------------------
// Definitions and Macros
//-------------------------------------------------------------------------------------------------------------------------------------------
#define MODULE_NAME "devmgr"

// Watchdog feed interval.
#define DEVMGR_WATCHDOG_FEED_INTERVAL_SEC   (60*5)

// Command line configuration macros.
#define DEVMGT_CONSOLE_KEEPAWAKE_MS         20000
#define DEVMGT_CONSOLEMSGTOKEN_COUNT        4         // qualifier<delim>cmd<delim>params<delim>seqnum
#define DEVMGT_CONSOLEMSGTOKEN_QULI_POS     0
#define DEVMGT_CONSOLEMSGTOKEN_CMD_POS      1
#define DEVMGT_CONSOLEMSGTOKEN_PARAM_POS    2

#define DEVMGT_CONSOLEMSGQULI_RADIOMOD      "RM"
#define DEVMGT_CONSOLEMSGQULI_METERMOD      "MM"

#define RESET_DELAY_SEC						10

PROCESS (devmanager_process, "NIC device manager process");

//-------------------------------------------------------------------------------------------------------------------------------------------
// Global Variables
//-------------------------------------------------------------------------------------------------------------------------------------------
struct rtc_timer watchdog_timer;

//-------------------------------------------------------------------------------------------------------------------------------------------
// External Function Prototypes
//-------------------------------------------------------------------------------------------------------------------------------------------
extern void secom_init (void); // This is not exposed to users through secom.h, hence the extern here.

//-------------------------------------------------------------------------------------------------------------------------------------------
// Function Prototypes
//-------------------------------------------------------------------------------------------------------------------------------------------
static int  devmgt_consolemsg_exec (char *data);
static void system_reboot(void);

//-------------------------------------------------------------------------------------------------------------------------------------------
// Functions
//-------------------------------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------------------------------
static void
devmgt_feedwd_periodic (void *ptr)
{
    struct time_value t = {.s = DEVMGR_WATCHDOG_FEED_INTERVAL_SEC, .ms = 0, .us = 0};

    // Reset watchdog.
    // We do not need to do anything here, the main scheduler loop will take care of watchdog reset for us.
    // We just need to wake the system up from LPM.

    // Set up next wakeup.
    rtc_timer_stop (&watchdog_timer);

	bool ret = rtc_timer_set_sc (&watchdog_timer, t, devmgt_feedwd_periodic, NULL);
	//x2m_printf(PLAT_INFO, MODULE_NAME, "watchdog_timer set, ret = %d\n",ret);

    RTC_HW_VERIFY();

	tamper_sensor_detect();

    energmon_data_save();
}

//-------------------------------------------------------------------------------------------------------------------------------------------
//  Tentative EUID64 format
//  +------+-------+------+-------+------+-------+------+-------+
//  |      FST OUI        |  CNST |            CRC32            |
//  +------+-------+------+-------+------+-------+------+-------+
//  |<-------- 24 ------->|<- 08->|<------------ 32 ----------->|
//  +------+-------+------+-------+------+-------+------+-------+
void devmgt_boardeui64 (addr64_t *eui64)
{
    static volatile uint8_t *unique_id = (volatile uint8_t*) FST_CPU_SN_ADDR;
    static uint8_t temp[15];
    static uint32_t c32;

    memcpy ((void *)temp, (void *)unique_id, 15);

    //static uint32_t i;
    //x2m_printf(PLAT_INFO, MODULE_NAME, "CPU Serial Number = 0x");
    //for (i=0; i<15; i++) {
    //printf("%02X", temp[i]);
    //}
    //printf("\n");

    c32 = crc32 (temp, 15);
    //x2m_printf(PLAT_INFO, MODULE_NAME, "Generated CRC32   = 0x%lX\n", c32);

    static uint64_t byteorderswapped;
    byteorderswapped = Swap64 ((uint64_t)(FST_OUI_24) << 40 | (uint64_t)(FST_LORA_RESERVED_8) << 32 | c32);
    memcpy (eui64, &byteorderswapped, 8);

    //x2m_printf(PLAT_INFO, MODULE_NAME, "Tentative EUI64   = 0x");
    //addr64_print (*eui64);

    return;
}

//-------------------------------------------------------------------------------------------------------------------------------------------
/**
 * \brief Configure UART console.
 */
void
devmgt_console_configure (void)
{
    x2m_console_uart_init();
}

//-------------------------------------------------------------------------------------------------------------------------------------------
static int
devmgt_consolemsg_exec (char *data)
{
    char datacopy[100] = {0};
    const char s[2] = "|";
    static char params[DEVMGT_CONSOLEMSGTOKEN_COUNT][20] = {{0}};
    char *token;
    int i = 0;

    // Copy data to a local array.
    memset (datacopy, 0, 100);
    strncpy (datacopy, data, 100);

    // Get the first token.
    token = strtok (datacopy, s);

    // Walk through parameters.
    while ((token != NULL) && (i < DEVMGT_CONSOLEMSGTOKEN_COUNT))
    {
        strncpy (params[i], token, 19);
        token = strtok (NULL, s);
        //x2m_printf(PLAT_INFO, MODULE_NAME, "********* params[%d] = [%s].\n", i, params[i]);
        ++i;
    }

    // Validate qualifier.
    if (NULL == params[DEVMGT_CONSOLEMSGTOKEN_QULI_POS])
    {
        x2m_printf(PLAT_INFO, MODULE_NAME, "Console message qualifier not found. Expect format is [quli%scmd%sparams].\n", s, s);
        return -1;
    }

    // Validate command.
    if (NULL == params[DEVMGT_CONSOLEMSGTOKEN_CMD_POS])
    {
        x2m_printf(PLAT_INFO, MODULE_NAME, "Console message command not found. Expect format is [quli%scmd%sparams].\n", s, s);
        return -1;
    }
    if (false == is_sanestr(params[DEVMGT_CONSOLEMSGTOKEN_CMD_POS]))
    {
        x2m_printf(PLAT_INFO, MODULE_NAME, "Console message command falied sanity check. Check the command again.\n");
        return -1;
    }

    // We received the radio module test command.
    if (NULL != strstr (params[DEVMGT_CONSOLEMSGTOKEN_QULI_POS], DEVMGT_CONSOLEMSGQULI_RADIOMOD))
    {
        x2m_printf(PLAT_INFO, MODULE_NAME, "Radio module test message received.\n");
        x2m_printf(PLAT_INFO, MODULE_NAME, "Command> %s.\n", params[DEVMGT_CONSOLEMSGTOKEN_CMD_POS]);
    }

    // We received the meter module test command.
    if (0 != strstr (params[DEVMGT_CONSOLEMSGTOKEN_QULI_POS], DEVMGT_CONSOLEMSGQULI_METERMOD))
    {
        x2m_printf(PLAT_INFO, MODULE_NAME, "Meter module test message received.\n");
        x2m_printf(PLAT_INFO, MODULE_NAME, "Command> %s, Params> %s.\n", params[DEVMGT_CONSOLEMSGTOKEN_CMD_POS], params[DEVMGT_CONSOLEMSGTOKEN_PARAM_POS]);
    }

    return 0;
}

//-------------------------------------------------------------------------------------------------------------------------------------------
static void
devmgt_reboot_do (void *data)
{
    x2m_printf(PLAT_INFO, MODULE_NAME, "Reboot alarm fired. Rebooting now...\n");
    system_reboot ();
    return;
}

//-------------------------------------------------------------------------------------------------------------------------------------------
static void
system_reboot(void)
{
    while (1)
    {
        x2m_system_reset();
    }
}

//-------------------------------------------------------------------------------------------------------------------------------------------
void
devmgt_reset (const char *fmt, ...)
{
    va_list vl;
    static char buf[50];

    // Print use supplied message.
    va_start (vl, fmt);
    vsnprintf (buf, 50, fmt, vl);
    va_end (vl);
    x2m_printf(PLAT_INFO, MODULE_NAME, "%s", buf);
    fflush (stdout);
    x2m_delay_ms (1000);

    // What do we do now?

    // Wait here?
    x2m_printf(PLAT_INFO, MODULE_NAME, "Endless loop here.\n");
    while (1) {};

    // OR reboot.
    //x2m_printf(PLAT_INFO, MODULE_NAME, "Rebooting system.\n");
    //system_reboot ();

    return;
}

//-------------------------------------------------------------------------------------------------------------------------------------------
// Processes
//-------------------------------------------------------------------------------------------------------------------------------------------

//-------------------------------------------------------------------------------------------------------------------------------------------
PROCESS_THREAD (devmanager_process, ev, data)
{
    static struct rtc_timer reboot_timer;    // For reboot delay.
    char databuf[30];

    PROCESS_BEGIN ();

    x2m_printf(PLAT_INFO, MODULE_NAME, "Device manager started.\n");

    // Set up a periodic watchdog reset callback.
    // Even if the system is not doing anything useful, we need to wakeup every
    // DEVMGR_WATCHDOG_FEED_INTERVAL_SEC seconds to reset the hardware watchdog.
    devmgt_feedwd_periodic (NULL);

    // Set up the reboot event.
    reboot_event = process_alloc_event ();

    // Initialize serial port comms module.
    secom_init ();

    // Perform any platform specific initializations.
    PLATFORM_DRIVER.init ();
    x2m_printf(PLAT_INFO, MODULE_NAME, "Platform extension %s initialized.\n", PLATFORM_DRIVER.name);

    while (1)
    {

        // Wait for any event.
        PROCESS_WAIT_EVENT ();

        // Allow any platform specific actions to take place.
        PLATFORM_DRIVER.eventhandler (ev, data);

        //------------------------------------------------------------
        // reboot event.
        //------------------------------------------------------------
        if (ev == reboot_event)
        {
            x2m_printf(PLAT_INFO, MODULE_NAME, "Reboot request received.\n");

            // Set timer to reboot.
            // LORATODO: Validate delay.
            uint8_t rd = RESET_DELAY_SEC;
            struct time_value reboot_val = {.s = rd, .ms = 0, .us = 0};
            rtc_timer_set_sc (&reboot_timer, reboot_val, devmgt_reboot_do, NULL);

            // Notify all processes.
            process_post (PROCESS_BROADCAST, reboot_event, &rd);

            // Wait until all processes have handled the reboot event.
            if(PROCESS_ERR_OK ==
                    process_post (PROCESS_CURRENT(), PROCESS_EVENT_CONTINUE, NULL))
            {
                PROCESS_WAIT_EVENT_UNTIL(ev == PROCESS_EVENT_CONTINUE);
            }
        }

        //------------------------------------------------------------
        // Console input event.
        //------------------------------------------------------------
        else if (ev == serial_line_event_message)
        {
            x2m_printf(PLAT_INFO, MODULE_NAME, "Received command through console. Wakelock for %dms.\n", DEVMGT_CONSOLE_KEEPAWAKE_MS);
            memset (databuf, 0, 30);
            memcpy (databuf, (char*)data, 30);
            lpm_wakelock_request (DEVMGT_CONSOLE_KEEPAWAKE_MS);
            devmgt_consolemsg_exec (databuf); // LORATODO: Check for the return value.
        }

        //------------------------------------------------------------
        // Network host incoming message event.
        //------------------------------------------------------------
        //else if (ev == eventxxx) {
        //}

        //------------------------------------------------------------
        // Sensor event.
        //------------------------------------------------------------
        else if (ev == sensors_event)
        {
            //x2m_printf(PLAT_INFO, MODULE_NAME, "Sensor event received.\n");
        }

    }

    PROCESS_END ();
}

// End if file.

