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
 *  Filename:      bt_vendor.c
 *
 *  Description:   Broadcom vendor specific library implementation
 *
 ******************************************************************************/

#define LOG_TAG "bt_vendor"

#include <utils/Log.h>
#include "bt_vendor.h"
#include "upio.h"
#include "userial_vendor.h"

#ifndef BTVND_DBG
#define BTVND_DBG FALSE
#endif

#if (BTVND_DBG == TRUE)
#define BTVNDDBG(param, ...) {ALOGD(param, ## __VA_ARGS__);}
#else
#define BTVNDDBG(param, ...) {}
#endif

/******************************************************************************
**  Externs
******************************************************************************/
extern vnd_userial_cb_t vnd_userial;

void hw_config_start(void);
uint8_t hw_lpm_enable(uint8_t turn_on);
uint32_t hw_lpm_get_idle_timeout(void);
void hw_lpm_set_wake_state(uint8_t wake_assert);
#if (SCO_CFG_INCLUDED == TRUE)
void hw_sco_config(void);
#endif
void vnd_load_conf(const char *p_path);

/******************************************************************************
**  Variables
******************************************************************************/

bt_vendor_callbacks_t *bt_vendor_cbacks = NULL;
uint8_t vnd_local_bd_addr[6]={0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

/******************************************************************************
**  Local type definitions
******************************************************************************/

/******************************************************************************
**  Static Variables
******************************************************************************/

/*
* if no baud rate in userial_vendor.h, set USERIAL_BAUD_AUTO
*/

static const tUSERIAL_CFG userial_init_cfg =
 {
    (USERIAL_DATABITS_8 | USERIAL_PARITY_NONE | USERIAL_STOPBITS_1),
	USERIAL_BAUD_AUTO	
 };

/******************************************************************************
**  Functions
******************************************************************************/

/*****************************************************************************
**
**   BLUETOOTH VENDOR INTERFACE LIBRARY FUNCTIONS
**
*****************************************************************************/

static int init(const bt_vendor_callbacks_t* p_cb, unsigned char *local_bdaddr)
{
    ALOGI("init");

    if (p_cb == NULL)
    {
        ALOGE("init failed with no user callbacks!");
        return -1;
    }

#if (VENDOR_LIB_RUNTIME_TUNING_ENABLED == TRUE)
    ALOGW("*****************************************************************");
    ALOGW("*****************************************************************");
    ALOGW("** Warning - BT Vendor Lib is loaded in debug tuning mode!");
    ALOGW("**");
    ALOGW("** If this is not intentional, rebuild libbt-vendor.so ");
    ALOGW("** with VENDOR_LIB_RUNTIME_TUNING_ENABLED=FALSE and ");
    ALOGW("** check if any run-time tuning parameters needed to be");
    ALOGW("** carried to the build-time configuration accordingly.");
    ALOGW("*****************************************************************");
    ALOGW("*****************************************************************");
#endif

    userial_vendor_init();
    upio_init();

    vnd_load_conf(VENDOR_LIB_CONF_FILE);

    /* store reference to user callbacks */
    bt_vendor_cbacks = (bt_vendor_callbacks_t *) p_cb;

    /* This is handed over from the stack */
    memcpy(vnd_local_bd_addr, local_bdaddr, 6);

    return 0;
}


/** Requested operations */
static int op(bt_vendor_opcode_t opcode, void *param)
{
    int retval = 0;

    BTVNDDBG("op for %d", opcode);

    switch(opcode)
    {
        case BT_VND_OP_POWER_CTRL:
            {
		/*  [operation]
		 *      Power on or off the BT Controller.
		 *  [input param]
		 *      A pointer to int type with content of bt_vendor_power_state_t.
		 *      Typecasting conversion: (int *) param.
		 *  [return]
		 *      0 - default, don't care.
		 *  [callback]
		 *      None.
		 */
                int *state = (int *) param;
				
                if (*state == BT_VND_PWR_OFF) {
					ALOGI("________BT_VND_OP_POWER_CTRL: UPIO_BT_POWER_OFF");
                    upio_set_bluetooth_power(UPIO_BT_POWER_OFF);
                }
                else if (*state == BT_VND_PWR_ON)
                {
                	ALOGI("________BT_VND_OP_POWER_CTRL: UPIO_BT_POWER_ON");
	                upio_set_bluetooth_power(UPIO_BT_POWER_ON);
					if(vnd_userial.android_bt_fw_download != 0)	/* we need this flag ????, tony */
		 			{	
			 			ms_delay(50);
						upio_set_bluetooth_power(UPIO_BT_FIRMWARE_DOWNLOAD);
						ms_delay(500);
                	}
                }
            }
            break;

        case BT_VND_OP_FW_CFG:
            {
		/*  [operation]
		 *      Perform any vendor specific initialization or configuration
		 *      on the BT Controller. This is called before stack initialization.
		 *  [input param]
		 *      None.
		 *  [return]
		 *      0 - default, don't care.
		 *  [callback]
		 *      Must call fwcfg_cb to notify the stack of the completion of vendor
		 *      specific initialization once it has been done.
		 */
				ALOGI("________BT_VND_OP_FW_CFG");
				//ALOGI("Download bt firmware:Will config hw!!");
				//hw_config_start();
				bt_vendor_cbacks->fwcfg_cb(BT_VND_OP_RESULT_SUCCESS);
            }
            break;

        case BT_VND_OP_SCO_CFG:
            {
		/*  [operation]
		 *      Perform any vendor specific SCO/PCM configuration on the BT Controller.
		 *      This is called after stack initialization.
		 *  [input param]
		 *      None.
		 *  [return]
		 *      0 - default, don't care.
		 *  [callback]
		 *      Must call scocfg_cb to notify the stack of the completion of vendor
		 *      specific SCO configuration once it has been done.
		 */
		 		bt_vendor_cbacks->scocfg_cb(BT_VND_OP_RESULT_SUCCESS); //dummy
            }
            break;

        case BT_VND_OP_USERIAL_OPEN:
            {
		/*  [operation]
		 *      Open UART port on where the BT Controller is attached.
		 *      This is called before stack initialization.
		 *  [input param]
		 *      A pointer to int array type for open file descriptors.
		 *      The mapping of HCI channel to fd slot in the int array is given in
		 *      bt_vendor_hci_channels_t.
		 *      And, it requires the vendor lib to fill up the content before returning
		 *      the call.
		 *      Typecasting conversion: (int (*)[]) param.
		 *  [return]
		 *      Numbers of opened file descriptors.
		 *      Valid number:
		 *          1 - CMD/EVT/ACL-In/ACL-Out via the same fd (e.g. UART)
		 *          2 - CMD/EVT on one fd, and ACL-In/ACL-Out on the other fd
		 *          4 - CMD, EVT, ACL-In, ACL-Out are on their individual fd
		 *  [callback]
		 *      None.
		 */
		 
                int (*fd_array)[] = (int (*)[]) param;
                int fd, idx;
				ALOGI("<Atmel> :open uart");
                fd = userial_vendor_open((tUSERIAL_CFG *) &userial_init_cfg);
                if (fd != -1)
                {
                    for (idx=0; idx < CH_MAX; idx++)
                        (*fd_array)[idx] = fd;

                    retval = 1;
                }
                /* retval contains numbers of open fd of HCI channels */
            }
            break;

        case BT_VND_OP_USERIAL_CLOSE:
            {
		/*  [operation]
		 *      Close the previously opened UART port.
		 *  [input param]
		 *      None.
		 *  [return]
		 *      0 - default, don't care.
		 *  [callback]
		 *      None.
		 */
				ALOGI("<Atmel> :close uart");
                userial_vendor_close();
            }
            break;

        case BT_VND_OP_GET_LPM_IDLE_TIMEOUT:
            {
		/*  [operation]
		 *      Get the LPM idle timeout in milliseconds.
		 *      The stack uses this information to launch a timer delay before it
		 *      attempts to de-assert LPM WAKE signal once downstream HCI packet
		 *      has been delivered.
		 *  [input param]
		 *      A pointer to uint32_t type which is passed in by the stack. And, it
		 *      requires the vendor lib to fill up the content before returning
		 *      the call.
		 *      Typecasting conversion: (uint32_t *) param.
		 *  [return]
		 *      0 - default, don't care.
		 *  [callback]
		 *      None.
		 */
		 
                uint32_t *timeout_ms = (uint32_t *) param;
                *timeout_ms = hw_lpm_get_idle_timeout();
				ALOGI("<Atmel> :returning timeout of %d ms", *timeout_ms);
            }
            break;

        case BT_VND_OP_LPM_SET_MODE:
            {
		/*  [operation]
		 *      Enable or disable LPM mode on BT Controller.
		 *  [input param]
		 *      A pointer to uint8_t type with content of bt_vendor_lpm_mode_t.
		 *      Typecasting conversion: (uint8_t *) param.
		 *  [return]
		 *      0 - default, don't care.
		 *  [callback]
		 *      Must call lpm_cb to notify the stack of the completion of LPM
		 *      disable/enable process once it has been done.
		 */
                bt_vendor_cbacks->lpm_cb(BT_VND_OP_RESULT_SUCCESS); 
            }
            break;

        case BT_VND_OP_LPM_WAKE_SET_STATE:
            break;

		case BT_VND_OP_EPILOG:
		{
			bt_vendor_cbacks->epilog_cb(BT_VND_OP_RESULT_SUCCESS);
		}
		break;
				
    }

    return retval;
}

/** Closes the interface */
static void cleanup( void )
{
    BTVNDDBG("cleanup");

    upio_cleanup();

    bt_vendor_cbacks = NULL;
}

// Entry point of DLib
const bt_vendor_interface_t BLUETOOTH_VENDOR_LIB_INTERFACE = {
    sizeof(bt_vendor_interface_t),
    init,
    op,
    cleanup
};
