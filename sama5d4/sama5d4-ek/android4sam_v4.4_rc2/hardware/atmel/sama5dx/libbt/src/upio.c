/******************************************************************************
 *
 *  Copyright (C) 2009-2012 Broadcom Corporation
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 *  you may not use this file except in compliance with the License.
 *  You may obtain a copy of the License at:
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS,
 *  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 *
 ******************************************************************************/

/******************************************************************************
 *
 *  Filename:      upio.c
 *
 *  Description:   Contains I/O functions, like
 *                      rfkill control
 *                      BT_WAKE/HOST_WAKE control
 *
 ******************************************************************************/

#define LOG_TAG "bt_upio"

#include <utils/Log.h>
#include <fcntl.h>
#include <errno.h>
#include <cutils/properties.h>
#include "bt_vendor.h"
#include "upio.h"
#include "userial_vendor.h"

/******************************************************************************
**  Constants & Macros
******************************************************************************/

#ifndef UPIO_DBG
#define UPIO_DBG FALSE
#endif

#if (UPIO_DBG == TRUE)
#define UPIODBG(param, ...) {ALOGD(param, ## __VA_ARGS__);}
#else
#define UPIODBG(param, ...) {}
#endif

/******************************************************************************
**  Local type definitions
******************************************************************************/

#if (BT_WAKE_VIA_PROC == TRUE)

/* proc fs node for toggling BT_WAKE gpio */
#ifndef VENDOR_LPM_PROC_NODE
#define VENDOR_LPM_PROC_NODE "/proc/bluetooth/sleep/lpm"
#endif
#endif

#define AT_CMD_POWER_UP		"BT_POWER_UP"
#define AT_CMD_DOWNLOAD_FW	"BT_DOWNLOAD_FW"
#define AT_CMD_POWER_DOWN	"BT_POWER_DOWN"


/******************************************************************************
**  Static variables
******************************************************************************/

static uint8_t upio_state[UPIO_MAX_COUNT];
static int rfkill_id = -1;
static int bt_emul_enable = 0;
static char power_dev_path[] = "/dev/at_bt_pwr";

/******************************************************************************
**  Static functions
******************************************************************************/

static char *lpm_state[] = {
    "UNKNOWN",
    "de-asserted",
    "asserted"
};

/*****************************************************************************
**   Bluetooth On/Off Static Functions
*****************************************************************************/
static int is_emulator_context(void)
{
    char value[PROPERTY_VALUE_MAX];

    property_get("ro.kernel.qemu", value, "0");
    UPIODBG("is_emulator_context : %s", value);
    if (strcmp(value, "1") == 0) {
        return 1;
    }
    return 0;
}

static int is_rfkill_disabled(void)
{
    char value[PROPERTY_VALUE_MAX];

    property_get("ro.rfkilldisabled", value, "0");
    UPIODBG("is_rfkill_disabled ? [%s]", value);

    if (strcmp(value, "1") == 0) {
        return UPIO_BT_POWER_ON;
    }

    return UPIO_BT_POWER_OFF;
}


/*****************************************************************************
**   LPM Static Functions
*****************************************************************************/

/*****************************************************************************
**   UPIO Interface Functions
*****************************************************************************/

/*******************************************************************************
**
** Function        upio_init
**
** Description     Initialization
**
** Returns         None
**
*******************************************************************************/
void upio_init(void)
{
	/* Init to Assert, otherwise, wake assert will stop the break which isn't set
	 in the first place, and the tablet will hang*/
    memset(upio_state, UPIO_ASSERT, UPIO_MAX_COUNT);
}

/*******************************************************************************
**
** Function        upio_cleanup
**
** Description     Clean up
**
** Returns         None
**
*******************************************************************************/
void upio_cleanup(void)
{
}

/*******************************************************************************
**
** Function        upio_set_bluetooth_power
**
** Description     Interact with low layer driver to set Bluetooth power
**                 on/off.
**
** Returns         0  : SUCCESS or Not-Applicable
**                 <0 : ERROR
**
*******************************************************************************/
int upio_set_bluetooth_power(int on)
{
    int sz;
    int fd = -1;
    int ret = -1;
    char buffer = '0';

	ALOGI("<Atmel: [%s] (%d) new power state: %d>", __FUNCTION__, __LINE__, on);

	fd = open(power_dev_path, O_WRONLY);
	if (fd < 0)
    {
        ALOGE("set_bluetooth_power : open(%s) for write failed: %s (%d)",
            power_dev_path, strerror(errno), errno);
        return ret;
    }
	
    switch(on)
    {
        case UPIO_BT_POWER_ON:
		{
			sz = write(fd, AT_CMD_POWER_UP, strlen(AT_CMD_POWER_UP));

		    if (sz < 0) {
		        ALOGE("set_bluetooth_power : write(%s) failed: %s (%d)",
		            AT_CMD_POWER_UP, strerror(errno),errno);
		    }
		    else
			{
				ret = 0;
			}

		}
		break;

        case UPIO_BT_POWER_OFF:
		{
			sz = write(fd, AT_CMD_POWER_DOWN, strlen(AT_CMD_POWER_DOWN));

		    if (sz < 0) {
		        ALOGE("set_bluetooth_power : write(%s) failed: %s (%d)",
		            AT_CMD_POWER_DOWN, strerror(errno),errno);
		    }
		    else
			{
				ret = 0;
			}
		}
		break;
		case UPIO_BT_FIRMWARE_DOWNLOAD:
		{
			sz = write(fd, AT_CMD_DOWNLOAD_FW, strlen(AT_CMD_DOWNLOAD_FW));
			if (sz < 0)
			{
				ALOGE("set_bluetooth_power : write(%s) failed: %s (%d)",
					AT_CMD_DOWNLOAD_FW, strerror(errno),errno);
		    }
			ret = 0;
		}
		break;
    }

    if (fd >= 0)
        close(fd);

    return ret;
}


/*******************************************************************************
**
** Function        upio_set
**
** Description     Set i/o based on polarity
**
** Returns         None
**
*******************************************************************************/
void upio_set(uint8_t pio, uint8_t action, uint8_t polarity)
{
    int rc;
#if (BT_WAKE_VIA_PROC == TRUE)
    int fd = -1;
    char buffer;
#endif

    switch (pio)
    {
        case UPIO_BT_WAKE:
            if (upio_state[UPIO_BT_WAKE] == action)
            {
                UPIODBG("BT_WAKE is %s already", action);
                return;
            }

            upio_state[UPIO_BT_WAKE] = action;

#if (BT_WAKE_VIA_PROC == TRUE)
            fd = open(VENDOR_LPM_PROC_NODE, O_WRONLY);

            if (fd < 0)
            {
                ALOGE("upio_set : open(%s) for write failed: %s (%d)",
                        VENDOR_LPM_PROC_NODE, strerror(errno), errno);
                break;
            }

            if (action == UPIO_ASSERT)
                buffer = '1';
            else if (action == UPIO_DEASSERT)
                buffer = '0';
            else
            {
                close(fd);
                break;
            }

            if (write(fd, &buffer, 1) < 0)
                ALOGE("upio_set : write(%s) failed: %s (%d)",
                        VENDOR_LPM_PROC_NODE, strerror(errno), errno);
            close(fd);
#endif

#if (BT_WAKE_VIA_USERIAL_IOCTL == TRUE)
			ALOGD("<Atmel> %s %d, action %s", __FUNCTION__, __LINE__, (action==UPIO_ASSERT)?"UPIO_ASSERT": \
											((action==UPIO_DEASSERT) ? "UPIO_DEASSERT": "UNKNOWN"));
            userial_vendor_ioctl( ( (action==UPIO_ASSERT) ? \
                      USERIAL_OP_ASSERT_BT_WAKE : USERIAL_OP_DEASSERT_BT_WAKE),\
                      NULL);
#endif

            break;

        case UPIO_HOST_WAKE:
            UPIODBG("upio_set: UPIO_HOST_WAKE");
            break;
    }
}
