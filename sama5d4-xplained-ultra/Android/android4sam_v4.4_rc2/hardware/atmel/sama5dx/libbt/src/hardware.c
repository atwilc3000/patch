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
 *  Filename:      hardware.c
 *
 *  Description:   Contains controller-specific functions, like
 *                      firmware patch download
 *                      low power mode operations
 *
 ******************************************************************************/

#define LOG_TAG "bt_hwcfg"


#include <utils/Log.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <signal.h>
#include <time.h>
#include <errno.h>
#include <fcntl.h>
#include <dirent.h>
#include <ctype.h>
#include <cutils/properties.h>
#include <stdlib.h>
#include "bt_hci_bdroid.h"
#include "bt_vendor.h"
#include "userial.h"
#include "userial_vendor.h"
#include "upio.h"
#include "at_log.h"

/******************************************************************************
**  Constants & Macros
******************************************************************************/

#ifndef BTHW_DBG
#define BTHW_DBG FALSE
#endif

#if (BTHW_DBG == TRUE)
#define BTHWDBG(param, ...) {ALOGD(param, ## __VA_ARGS__);}
#else
#define BTHWDBG(param, ...) {}
#endif

#define FW_PATCHFILE_EXTENSION      ".hcd"
#define FW_PATCHFILE_EXTENSION_LEN  4
#define FW_PATCHFILE_PATH_MAXLEN    248 /* Local_Name length of return of
                                           HCI_Read_Local_Name */

#define HCI_CMD_MAX_LEN             258

#define HCI_RESET									0x0C03
//#define HCI_VSC_WRITE_UART_CLOCK_SETTING			0xFC45
//#define HCI_VSC_UPDATE_BAUDRATE					0xFC18
#define HCI_VSC_UPDATE_BAUDRATE						0xFC53	//atmel: HCI for UART baudrate update
#define HCI_READ_LOCAL_NAME							0x0C14
//#define HCI_VSC_DOWNLOAD_MINIDRV					0xFC2E
#define HCI_VSC_WRITE_BD_ADDR						0xFC54
//#define HCI_VSC_WRITE_SLEEP_MODE					0xFC27
//#define HCI_VSC_WRITE_SCO_PCM_INT_PARAM			0xFC1C
//#define HCI_VSC_WRITE_PCM_DATA_FORMAT_PARAM		0xFC1E
//#define HCI_VSC_WRITE_I2SPCM_INTERFACE_PARAM		0xFC6D
//#define HCI_VSC_LAUNCH_RAM 						0xFC4E
#define HCI_READ_LOCAL_BDADDR						0x1009
#define HCI_READ_MEM								0xFC01
#define HCI_VSC_WRITE_MEM							0xFC52
//#define HCI_VSC_MOVE_MEM							0xFC50
#define HCI_CPU_RESET								0xFC55
#define HCI_READ_LOCAL_VERSION						0x1001

#define HCI_EVT_CMD_CMPL_STATUS_RET_BYTE        5
#define HCI_EVT_CMD_CMPL_LOCAL_NAME_STRING      6
#define HCI_EVT_CMD_CMPL_LOCAL_BDADDR_ARRAY     6
#define HCI_EVT_CMD_CMPL_LOCAL_VER_HCI_VER      6
#define HCI_EVT_CMD_CMPL_OPCODE                 3
#define LPM_CMD_PARAM_SIZE                      12
//#define UPDATE_BAUDRATE_CMD_PARAM_SIZE          6
#define UPDATE_BAUDRATE_CMD_PARAM_SIZE          5
#define READ_MEMORY_CMD_PARAM_SIZE 				6
#define HCI_CMD_PREAMBLE_SIZE                   3
#define HCD_REC_PAYLOAD_LEN_BYTE                2
#define BD_ADDR_LEN                             6
#define LOCAL_NAME_BUFFER_LEN                   32
#define LOCAL_BDADDR_PATH_BUFFER_LEN            256
#define WRITE_MEM_CMD_PARAM_SIZE				8
#define MOVE_MEM_CMD_PARAM_SIZE					13

#define FW_DOWNLOAD_CHUNK_SIZE (4*1024)
#define DRAM_TEST_REGION_1_BASE   0x00002000
#define DRAM_TEST_REGION_1_LENGTH 0x00001000
#define IRAM_FW_DOWNLAOD_BASE 0x80000000
#define BT_CLK_SRC_REG	0x40001028


#define STREAM_TO_UINT16(u16, p) {u16 = ((uint16_t)(*(p)) + (((uint16_t)(*((p) + 1))) << 8)); (p) += 2;}
#define UINT16_TO_STREAM(p, u16) {*(p)++ = (uint8_t)(u16); *(p)++ = (uint8_t)((u16) >> 8);}
#define UINT32_TO_STREAM(p, u32) {*(p)++ = (uint8_t)(u32); *(p)++ = (uint8_t)((u32) >> 8); *(p)++ = (uint8_t)((u32) >> 16); *(p)++ = (uint8_t)((u32) >> 24);}

#define AT_CHIP_NAME "AT-WILC3000"

/******************************************************************************
**  Local type definitions
******************************************************************************/

/* Hardware Configuration State */
enum {
    HW_CFG_START = 1,
    HW_CFG_SET_UART_CLOCK,
    HW_CFG_SET_UART_BAUD_1,
    HW_CFG_READ_LOCAL_NAME,
    HW_CFG_DL_MINIDRIVER,
    HW_CFG_DL_FW_PATCH,
    HW_CFG_SET_UART_BAUD_2,
	HW_CFG_SET_BD_ADDR,
    HW_CFG_CHECK_BOOTROM,
    HW_CFG_CHECK_BT_CLK_SRC,
	HW_CFG_SET_BAUD_RATE
#if (USE_CONTROLLER_BDADDR == TRUE)
    , HW_CFG_READ_BD_ADDR
#endif
};

/* h/w config control block */
typedef struct
{
    uint8_t state;                          /* Hardware configuration state */
    int     fw_fd;                          /* FW patch file fd */
    uint8_t f_set_baud_2;                   /* Baud rate switch state */
    char    local_chip_name[LOCAL_NAME_BUFFER_LEN];
	uint32_t fw_dl_progress;				/* Keeps track of the firmware download progress */
	uint32_t fw_last_dl_size;				/* Keeps track of the last firmware download size */
} bt_hw_cfg_cb_t;

/* low power mode parameters */
typedef struct
{
    uint8_t sleep_mode;                     /* 0(disable),1(UART),9(H5) */
    uint8_t host_stack_idle_threshold;      /* Unit scale 300ms/25ms */
    uint8_t host_controller_idle_threshold; /* Unit scale 300ms/25ms */
    uint8_t bt_wake_polarity;               /* 0=Active Low, 1= Active High */
    uint8_t host_wake_polarity;             /* 0=Active Low, 1= Active High */
    uint8_t allow_host_sleep_during_sco;
    uint8_t combine_sleep_mode_and_lpm;
    uint8_t enable_uart_txd_tri_state;      /* UART_TXD Tri-State */
    uint8_t sleep_guard_time;               /* sleep guard time in 12.5ms */
    uint8_t wakeup_guard_time;              /* wakeup guard time in 12.5ms */
    uint8_t txd_config;                     /* TXD is high in sleep state */
    uint8_t pulsed_host_wake;               /* pulsed host wake if mode = 1 */
} bt_lpm_param_t;

/* Firmware re-launch settlement time */
typedef struct {
    const char *chipset_name;
    const uint32_t delay_time;
} fw_settlement_entry_t;


/******************************************************************************
**  Externs
******************************************************************************/

void hw_config_cback(void *p_evt_buf);
uint8_t hw_config_update_ctrl_baud_rate(int baud , uint8_t flow_control);
extern uint8_t vnd_local_bd_addr[BD_ADDR_LEN];
extern vnd_userial_cb_t vnd_userial;
int get_closest_baud_rate(uint32_t target_baud_rate ,uint32_t* baud);

/******************************************************************************
**  Static variables
******************************************************************************/

static char fw_patchfile_path[256] = FW_PATCHFILE_LOCATION;
static char fw_patchfile_name[128] = { 0 };
#if (VENDOR_LIB_RUNTIME_TUNING_ENABLED == TRUE)
static int fw_patch_settlement_delay = -1;
#endif

static bt_hw_cfg_cb_t hw_cfg_cb;

static bt_lpm_param_t lpm_param =
{
    LPM_SLEEP_MODE,
    LPM_IDLE_THRESHOLD,
    LPM_HC_IDLE_THRESHOLD,
    LPM_BT_WAKE_POLARITY,
    LPM_HOST_WAKE_POLARITY,
    LPM_ALLOW_HOST_SLEEP_DURING_SCO,
    LPM_COMBINE_SLEEP_MODE_AND_LPM,
    LPM_ENABLE_UART_TXD_TRI_STATE,
    0,  /* not applicable */
    0,  /* not applicable */
    0,  /* not applicable */
    LPM_PULSED_HOST_WAKE
};

#if (!defined(SCO_USE_I2S_INTERFACE) || (SCO_USE_I2S_INTERFACE == FALSE))
static uint8_t bt_sco_param[SCO_PCM_PARAM_SIZE] =
{
    SCO_PCM_ROUTING,
    SCO_PCM_IF_CLOCK_RATE,
    SCO_PCM_IF_FRAME_TYPE,
    SCO_PCM_IF_SYNC_MODE,
    SCO_PCM_IF_CLOCK_MODE
};

static uint8_t bt_pcm_data_fmt_param[PCM_DATA_FORMAT_PARAM_SIZE] =
{
    PCM_DATA_FMT_SHIFT_MODE,
    PCM_DATA_FMT_FILL_BITS,
    PCM_DATA_FMT_FILL_METHOD,
    PCM_DATA_FMT_FILL_NUM,
    PCM_DATA_FMT_JUSTIFY_MODE
};
#else
static uint8_t bt_sco_param[SCO_I2SPCM_PARAM_SIZE] =
{
    SCO_I2SPCM_IF_MODE,
    SCO_I2SPCM_IF_ROLE,
    SCO_I2SPCM_IF_SAMPLE_RATE,
    SCO_I2SPCM_IF_CLOCK_RATE
};
#endif

/*
 * The look-up table of recommended firmware settlement delay (milliseconds) on
 * known chipsets.
 */
static const fw_settlement_entry_t fw_settlement_table[] = {
    {"BCM43241", 200},
	{AT_CHIP_NAME, 200},
    {(const char *) NULL, 100}  // Giving the generic fw settlement delay setting.
};

/******************************************************************************
**  Static functions
******************************************************************************/
static uint8_t hw_config_set_bdaddr(char * bd_addr);
static uint8_t hw_config_reset(void);

/******************************************************************************
**  Controller Initialization Static Functions
******************************************************************************/

/*******************************************************************************
**
** Function        look_up_fw_settlement_delay
**
** Description     If FW_PATCH_SETTLEMENT_DELAY_MS has not been explicitly
**                 re-defined in the platform specific build-time configuration
**                 file, we will search into the look-up table for a
**                 recommended firmware settlement delay value.
**
**                 Although the settlement time might be also related to board
**                 configurations such as the crystal clocking speed.
**
** Returns         Firmware settlement delay
**
*******************************************************************************/
uint32_t look_up_fw_settlement_delay (void)
{
    uint32_t ret_value;
    fw_settlement_entry_t *p_entry;

    if (FW_PATCH_SETTLEMENT_DELAY_MS > 0)
    {
        ret_value = FW_PATCH_SETTLEMENT_DELAY_MS;
    }
#if (VENDOR_LIB_RUNTIME_TUNING_ENABLED == TRUE)
    else if (fw_patch_settlement_delay >= 0)
    {
        ret_value = fw_patch_settlement_delay;
    }
#endif
    else
    {
        p_entry = (fw_settlement_entry_t *)fw_settlement_table;

        while (p_entry->chipset_name != NULL)
        {
            if (strstr(hw_cfg_cb.local_chip_name, p_entry->chipset_name)!=NULL)
            {
                break;
            }

            p_entry++;
        }

        ret_value = p_entry->delay_time;
    }

    BTHWDBG( "Settlement delay -- %d ms", ret_value);

    return (ret_value);
}

/*******************************************************************************
**
** Function        ms_delay
**
** Description     sleep unconditionally for timeout milliseconds
**
** Returns         None
**
*******************************************************************************/
void ms_delay (uint32_t timeout)
{
    struct timespec delay;
    int err;

    if (timeout == 0)
        return;

    delay.tv_sec = timeout / 1000;
    delay.tv_nsec = 1000 * 1000 * (timeout%1000);

    /* [u]sleep can't be used because it uses SIGALRM */
    do {
        err = nanosleep(&delay, &delay);
    } while (err < 0 && errno ==EINTR);
}

/*******************************************************************************
**
** Function         hw_strncmp
**
** Description      Used to compare two strings in caseless
**
** Returns          0: match, otherwise: not match
**
*******************************************************************************/
static int hw_strncmp (const char *p_str1, const char *p_str2, const int len)
{
    int i;

    if (!p_str1 || !p_str2)
        return (1);

    for (i = 0; i < len; i++)
    {
        if (toupper(p_str1[i]) != toupper(p_str2[i]))
            return (i+1);
    }

    return 0;
}

/*******************************************************************************
**
** Function         hw_config_findpatch
**
** Description      Search for a proper firmware patch file
**                  The selected firmware patch file name with full path
**                  will be stored in the input string parameter, i.e.
**                  p_chip_id_str, when returns.
**
** Returns          TRUE when found the target patch file, otherwise FALSE
**
*******************************************************************************/
static uint8_t hw_config_findpatch(char *p_chip_id_str)
{
    DIR *dirp;
    struct dirent *dp;
    int filenamelen;
    uint8_t retval = FALSE;

    BTHWDBG("Target name = [%s]", p_chip_id_str);

    if (strlen(fw_patchfile_name)> 0)
    {
        /* If specific filepath and filename have been given in run-time
         * configuration /etc/bluetooth/bt_vendor.conf file, we will use them
         * to concatenate the filename to open rather than searching a file
         * matching to chipset name in the fw_patchfile_path folder.
         */
        sprintf(p_chip_id_str, "%s", fw_patchfile_path);
        if (fw_patchfile_path[strlen(fw_patchfile_path)- 1] != '/')
        {
            strcat(p_chip_id_str, "/");
        }
        strcat(p_chip_id_str, fw_patchfile_name);

        ALOGI("FW patchfile: %s", p_chip_id_str);
        return TRUE;
    }

    if ((dirp = opendir(fw_patchfile_path)) != NULL)
    {
        /* Fetch next filename in patchfile directory */
        while ((dp = readdir(dirp)) != NULL)
        {
            /* Check if filename starts with chip-id name */
            if ((hw_strncmp(dp->d_name, p_chip_id_str, strlen(p_chip_id_str)) \
                ) == 0)
            {
                /* Check if it has .hcd extenstion */
                filenamelen = strlen(dp->d_name);
                if ((filenamelen >= FW_PATCHFILE_EXTENSION_LEN) &&
                    ((hw_strncmp(
                          &dp->d_name[filenamelen-FW_PATCHFILE_EXTENSION_LEN], \
                          FW_PATCHFILE_EXTENSION, \
                          FW_PATCHFILE_EXTENSION_LEN) \
                     ) == 0))
                {
                    ALOGI("Found patchfile: %s/%s", \
                        fw_patchfile_path, dp->d_name);

                    /* Make sure length does not exceed maximum */
                    if ((filenamelen + strlen(fw_patchfile_path)) > \
                         FW_PATCHFILE_PATH_MAXLEN)
                    {
                        ALOGE("Invalid patchfile name (too long)");
                    }
                    else
                    {
                        memset(p_chip_id_str, 0, FW_PATCHFILE_PATH_MAXLEN);
                        /* Found patchfile. Store location and name */
                        strcpy(p_chip_id_str, fw_patchfile_path);
                        if (fw_patchfile_path[ \
                            strlen(fw_patchfile_path)- 1 \
                            ] != '/')
                        {
                            strcat(p_chip_id_str, "/");
                        }
                        strcat(p_chip_id_str, dp->d_name);
                        retval = TRUE;
                    }
                    break;
                }
            }
        }

        closedir(dirp);

        if (retval == FALSE)
        {
            /* Try again chip name without revision info */

            int len = strlen(p_chip_id_str);
            char *p = p_chip_id_str + len - 1;

            /* Scan backward and look for the first alphabet
               which is not M or m
            */
            while (len > 3) // BCM****
            {
                if ((isdigit(*p)==0) && (*p != 'M') && (*p != 'm'))
                    break;

                p--;
                len--;
            }

            if (len > 3)
            {
                *p = 0;
                retval = hw_config_findpatch(p_chip_id_str);
            }
        }
    }
    else
    {
        ALOGE("Could not open %s", fw_patchfile_path);
    }

    return (retval);
}

/*******************************************************************************
**
** Function         hw_config_set_bdaddr
**
** Description      Program controller's Bluetooth Device Address
**
** Returns          TRUE, if valid address is sent
**                  FALSE, otherwise
**
*******************************************************************************/

static uint8_t hw_config_set_bdaddr(char * bd_addr)
{
	uint8_t retval = FALSE;
	HC_BT_HDR  *p_buf = NULL;
    uint8_t     *p;
   	uint8_t     is_proceeding=0;
	
	ALOGI("Atmel: Set bd address ");
	if(vnd_userial.fd == -1)
	{
		ALOGE("vendor lib fw conf aborted. serial port not open");
		pwr_dev_deliver_event(CHIP_ALLOW_SLEP_AFTER_BT_FW_DOWNLOAD);
		bt_vendor_cbacks->fwcfg_cb(BT_VND_OP_RESULT_FAIL);
	}
	else
	{
		if (bt_vendor_cbacks)		
			p_buf = (HC_BT_HDR  *)bt_vendor_cbacks->alloc(BT_HC_HDR_SIZE + \
	                                                       HCI_CMD_MAX_LEN );
		if (p_buf)
		{
			p_buf->event = MSG_STACK_TO_HC_HCI_CMD;
			p_buf->offset = 0;
			p_buf->layer_specific = 0;
			p_buf->len = HCI_CMD_PREAMBLE_SIZE;
			
			p = (uint8_t *) (p_buf + 1);
			UINT16_TO_STREAM(p, HCI_VSC_WRITE_BD_ADDR);
			*p++ =6; /* parameter length */
					
			*p++ =bd_addr[0];
			*p++ =bd_addr[1];
			*p++ =bd_addr[2];
			*p++ =bd_addr[3];
			*p++ =bd_addr[4];
			*p++ =bd_addr[5];

	        p_buf->len = HCI_CMD_PREAMBLE_SIZE + 6;
			hw_cfg_cb.state = HW_CFG_SET_BD_ADDR;
						
			is_proceeding= bt_vendor_cbacks->xmit_cb(HCI_VSC_WRITE_BD_ADDR, p_buf, hw_config_cback);
		}
		else
		{
			pwr_dev_deliver_event(CHIP_ALLOW_SLEP_AFTER_BT_FW_DOWNLOAD);
			if(bt_vendor_cbacks)
		    {
		    	ALOGE("vendor lib fw conf aborted [no buffer]");
				bt_vendor_cbacks->fwcfg_cb(BT_VND_OP_RESULT_FAIL);
		    }
		}
	}
	return is_proceeding;
}

#if (USE_CONTROLLER_BDADDR == TRUE)
/*******************************************************************************
**
** Function         hw_config_read_bdaddr
**
** Description      Read controller's Bluetooth Device Address
**
** Returns          TRUE, if valid address is sent
**                  FALSE, otherwise
**
*******************************************************************************/
static uint8_t hw_config_read_bdaddr(HC_BT_HDR *p_buf)
{
    uint8_t retval = FALSE;
    uint8_t *p = (uint8_t *) (p_buf + 1);

    UINT16_TO_STREAM(p, HCI_READ_LOCAL_BDADDR);
    *p = 0; /* parameter length */

    p_buf->len = HCI_CMD_PREAMBLE_SIZE;
    hw_cfg_cb.state = HW_CFG_READ_BD_ADDR;

    retval = bt_vendor_cbacks->xmit_cb(HCI_READ_LOCAL_BDADDR, p_buf, \
                                 hw_config_cback);

    return (retval);
}
#endif // (USE_CONTROLLER_BDADDR == TRUE)
/*******************************************************************************
**
** Function        isBTFirmwareDownloading
**
** Description    Check if BT firmware is being downloaded
**
** Returns         True: If downloading process is in progress
			    False:If downloading is done or did not start
**
*******************************************************************************/
int bt_fw_downloading=0;
int is_bt_fw_downloading(void)
{
	return bt_fw_downloading;
}
/*******************************************************************************
**
** Function        isBTFirmwareDownloadingSetValue
**
** Description    Change value fo BT firmware download flag
**
**
*******************************************************************************/
void set_bt_fw_downlading(int val)
{
	bt_fw_downloading=val;
}
/*******************************************************************************
**
** Function         hw_config_cback
**
** Description      Callback function for controller configuration
**
** Returns          None
**
*******************************************************************************/
int fw_size=0;
extern int uart_close;
void hw_config_cback(void *p_mem)
{
	HC_BT_HDR *p_evt_buf = (HC_BT_HDR *) p_mem;
	uint8_t     read_buf[256];
	char		*p_name, *p_tmp;
	uint8_t 	*p, status;
	uint16_t	opcode;
	HC_BT_HDR  *p_buf=NULL;
	uint8_t 	is_proceeding = FALSE;
	int 		i;
	int bytes_read;
	FILE* fw;
	uint32_t baud_rate;

	#if (USE_CONTROLLER_BDADDR == TRUE)
	const uint8_t null_bdaddr[BD_ADDR_LEN] = {0,0,0,0,0,0};
	#endif

    status = *((uint8_t *)(p_evt_buf + 1) + HCI_EVT_CMD_CMPL_STATUS_RET_BYTE);
    p = (uint8_t *)(p_evt_buf + 1) + HCI_EVT_CMD_CMPL_OPCODE;
    STREAM_TO_UINT16(opcode,p);

    /* Ask a new buffer big enough to hold any HCI commands sent in here */
    if ((status == 0) && bt_vendor_cbacks)
        p_buf = (HC_BT_HDR *) bt_vendor_cbacks->alloc(BT_HC_HDR_SIZE + \
                                                       HCI_CMD_MAX_LEN);

	ALOGI("[Atmel %s] rcvd cb with state: %d!!,status:%d,op_code:%x",__func__, hw_cfg_cb.state,status,opcode);
	if (p_buf != NULL)
	{
		p_buf->event = MSG_STACK_TO_HC_HCI_CMD;
		p_buf->offset = 0;
		p_buf->len = 0;
		p_buf->layer_specific = 0;

        p = (uint8_t *) (p_buf + 1);

        switch (hw_cfg_cb.state)
        {
        	case HW_CFG_CHECK_BOOTROM:
			{
				uint8_t *hci_version;
				uint8_t *ret_buffer;
					
				ALOGI("[Atmel] bt vendor lib config callback (HW_CFG_CHECK_BOOTROM)");

				ret_buffer = (uint8_t *)(p_evt_buf + 1) ;
				hci_version = (uint8_t *) (p_evt_buf + 1) + \
						 HCI_EVT_CMD_CMPL_LOCAL_VER_HCI_VER;

			//	for(i=0;i<30;i++)
					ALOGI("read[%d]=%x",6,ret_buffer[6]);
			
					if((hci_version[0]==6))/*this indicates that there has been a baud rate mismatch between host and controller*/ 
					{
						ALOGI("Firmware already downloaded");
						
						ms_delay(10);
						if(vnd_userial.fw_op_baudrate != vnd_userial.bootrom_baudrate)
						{
							ALOGI("Raise host and controller baud rates to %d",vnd_userial.fw_op_baudrate);
							baud_rate=get_closest_baud_rate(vnd_userial.fw_op_baudrate,&vnd_userial.actual_baud);
							is_proceeding=hw_config_update_ctrl_baud_rate(baud_rate,vnd_userial.flow_control);
							
							
						}
						else if (vnd_userial.enable_bdaddress_change != 0)
						{
						
							is_proceeding=hw_config_set_bdaddr(vnd_userial.bd_addr);
								
						}
						else
						{
							is_proceeding=1;
							pwr_dev_deliver_event(CHIP_ALLOW_SLEP_AFTER_BT_FW_DOWNLOAD);
							bt_vendor_cbacks->fwcfg_cb(BT_VND_OP_RESULT_SUCCESS);
						}
						
					}		
					else	
					{	
						ALOGI("No firmware downloaded");
					
						p_buf->event = MSG_STACK_TO_HC_HCI_CMD;
			       		p_buf->offset = 0;
			       		p_buf->layer_specific = 0;
			       		p_buf->len = HCI_CMD_PREAMBLE_SIZE;
			
			       		p = (uint8_t *) (p_buf + 1);
			       		UINT16_TO_STREAM(p, HCI_RESET);
			      			*p = 0; /* parameter length */

						hw_cfg_cb.state = HW_CFG_START;
						//hw_cfg_cb.state = HW_CFG_SET_UART_BAUD_1;
					
						ALOGI("[Atmel %s] sending RESET command. UART_TARGET_BAUD_RATE is %d",__func__,UART_TARGET_BAUD_RATE);
			       		is_proceeding=bt_vendor_cbacks->xmit_cb(HCI_RESET, p_buf, hw_config_cback);
					}
					
	        		
		
        		}
			break;
            case HW_CFG_SET_UART_BAUD_1:
                /* update baud rate of host's UART port */
                ALOGI("bt vendor lib: set UART baud %i", UART_TARGET_BAUD_RATE);
           
            	get_closest_baud_rate(vnd_userial.fw_dwnld_baudrate,&vnd_userial.actual_baud);
              	userial_vendor_set_baud(vnd_userial.actual_baud);

				#if 0 // NMI We don't support HCI_READ_LOCAL_NAME, although it might be useful to support in the future
                /* read local name */
                UINT16_TO_STREAM(p, HCI_READ_LOCAL_NAME);
                *p = 0; /* parameter length */

                p_buf->len = HCI_CMD_PREAMBLE_SIZE;
                hw_cfg_cb.state = HW_CFG_READ_LOCAL_NAME;

                is_proceeding = bt_vendor_cbacks->xmit_cb(HCI_READ_LOCAL_NAME, \
                                                    p_buf, hw_config_cback);
                break;
				#endif // NMI fall through intentionaly cause we won't send HCI_READ_LOCAL_NAME
			case HW_CFG_READ_LOCAL_NAME:
				ALOGI("[Atmel] bt vendor lib config callback (HW_CFG_READ_LOCAL_NAME)");
		  		#if 0 //NMI: we don't support local chip name for now
				p_tmp = p_name = (char *) (p_evt_buf + 1) + \
						 HCI_EVT_CMD_CMPL_LOCAL_NAME_STRING;

                for (i=0; (i < LOCAL_NAME_BUFFER_LEN)||(*(p_name+i) != 0); i++)
                    *(p_name+i) = toupper(*(p_name+i));

                if ((p_name = strstr(p_name, "BCM")) != NULL)
                {
                    strncpy(hw_cfg_cb.local_chip_name, p_name, \
                            LOCAL_NAME_BUFFER_LEN-1);
                }
                else
                {
                    strncpy(hw_cfg_cb.local_chip_name, "UNKNOWN", \
                            LOCAL_NAME_BUFFER_LEN-1);
                    p_name = p_tmp;
                }

                hw_cfg_cb.local_chip_name[LOCAL_NAME_BUFFER_LEN-1] = 0;

                BTHWDBG("Chipset %s", hw_cfg_cb.local_chip_name);

                if ((status = hw_config_findpatch(p_name)) == TRUE)
                {
                    if ((hw_cfg_cb.fw_fd = open(p_name, O_RDONLY)) == -1)
                    {
                        ALOGE("vendor lib preload failed to open [%s]", p_name);
                    }
                    else
                    {
                        /* vsc_download_minidriver */
                        UINT16_TO_STREAM(p, HCI_VSC_DOWNLOAD_MINIDRV);
                        *p = 0; /* parameter length */

                        p_buf->len = HCI_CMD_PREAMBLE_SIZE;
                        hw_cfg_cb.state = HW_CFG_DL_MINIDRIVER;

                        is_proceeding = bt_vendor_cbacks->xmit_cb( \
                                            HCI_VSC_DOWNLOAD_MINIDRV, p_buf, \
                                            hw_config_cback);
                    }
                }
                else
                {
                    ALOGE( \
                    "vendor lib preload failed to locate firmware patch file" \
                    );
                }

				if (is_proceeding == FALSE)
				{
					is_proceeding = hw_config_set_bdaddr(p_buf);
				}
				break;
				#else
				char p_nmi_path[] = "/system/etc/firmware/bt_firmware.bin";
				p_name = p_nmi_path;
				if ((hw_cfg_cb.fw_fd = open(p_name, O_RDONLY)) == -1)
				{
					ALOGE("vendor lib preload failed to open [%s]", p_name);
					is_proceeding=FALSE;
					break;
				}
				/* get firmware file size to use it for final check*/	
				fw=fdopen(hw_cfg_cb.fw_fd, "rb");
		
				if (fw )
				{
					fseek(fw,0L,SEEK_END);

					fw_size=ftell(fw);

					fseek(fw,0L,SEEK_SET);
					ALOGI("[Atmel]Firmware size= %d",fw_size);
		
				}
				if((fw_size == 0) || (fw_size < 0))
				{
					ALOGI("Couldn't get firmware file size");
					is_proceeding=FALSE;
					break;
				}
		
				hw_cfg_cb.fw_dl_progress = 0;
				hw_cfg_cb.fw_last_dl_size = 0;
				#endif // NMI:
				/* NMI: fall through intentionally cause we won't send any commands  from this state*/
			case HW_CFG_DL_MINIDRIVER:
				ALOGI("bt vendor lib config callback (HW_CFG_DL_MINIDRIVER)");
				/* give time for placing firmware in download mode */
				ms_delay(50);
				hw_cfg_cb.state = HW_CFG_DL_FW_PATCH;

				/* fall through intentionally */
			case HW_CFG_DL_FW_PATCH:
				//ALOGI("bt vendor lib config callback (HW_CFG_DL_FW_PATCH)");
				#if 0 // NMI:
				p_buf->len = read(hw_cfg_cb.fw_fd, p, HCI_CMD_PREAMBLE_SIZE);
				if (p_buf->len > 0)
				{
					if ((p_buf->len < HCI_CMD_PREAMBLE_SIZE) || \
						(opcode == HCI_VSC_LAUNCH_RAM))
					{
						ALOGW("firmware patch file might be altered!");
					}
					else
					{
						p_buf->len += read(hw_cfg_cb.fw_fd, \
										   p+HCI_CMD_PREAMBLE_SIZE,\
										   *(p+HCD_REC_PAYLOAD_LEN_BYTE));
						STREAM_TO_UINT16(opcode,p);
						is_proceeding = bt_vendor_cbacks->xmit_cb(opcode, \
												p_buf, hw_config_cback);
						break;
					}
				}
				#else // NMI: Do our firmware download
				/* Need bigger buffer to carry the write memory HCI command and the actual data*/
				bt_vendor_cbacks->dealloc(p_buf);
				p_buf = (HC_BT_HDR	*)bt_vendor_cbacks->alloc(BT_HC_HDR_SIZE + HCI_CMD_MAX_LEN + FW_DOWNLOAD_CHUNK_SIZE);

				if(p_buf != NULL)
				{
			
				
					p_buf->event = MSG_STACK_TO_HC_HCI_CMD;
					p_buf->offset = 0;
					p_buf->layer_specific = 0;
					p_buf->len = HCI_CMD_PREAMBLE_SIZE;
					p = (uint8_t *) (p_buf + 1);
			
					hw_cfg_cb.fw_last_dl_size = read(hw_cfg_cb.fw_fd,p+WRITE_MEM_CMD_PARAM_SIZE+HCI_CMD_PREAMBLE_SIZE,FW_DOWNLOAD_CHUNK_SIZE);

					if(hw_cfg_cb.fw_last_dl_size)
					{
						UINT16_TO_STREAM(p, HCI_VSC_WRITE_MEM);
						*p++ = WRITE_MEM_CMD_PARAM_SIZE;				/* parameter length */
						UINT32_TO_STREAM(p,  IRAM_FW_DOWNLAOD_BASE+hw_cfg_cb.fw_dl_progress); /* Traget Address */
						UINT32_TO_STREAM(p, hw_cfg_cb.fw_last_dl_size);	/* Size */
							
		
						p_buf->len = HCI_CMD_PREAMBLE_SIZE+WRITE_MEM_CMD_PARAM_SIZE+hw_cfg_cb.fw_last_dl_size;
								
						hw_cfg_cb.state = HW_CFG_DL_FW_PATCH;
						hw_cfg_cb.fw_dl_progress += hw_cfg_cb.fw_last_dl_size;

						is_proceeding=bt_vendor_cbacks->xmit_cb(HCI_VSC_WRITE_MEM, p_buf, hw_config_cback);
							  
						break;
					}
						
				}		
				else
				{
					ALOGE("Failed to allocate memory to download firmware");
					is_proceeding=FALSE;
					break;
				}
						
				close(hw_cfg_cb.fw_fd);
				hw_cfg_cb.fw_fd = -1;
				if(hw_cfg_cb.fw_dl_progress == (uint32_t)fw_size)
				{		
					ALOGI("vendor lib fwcfg completed");


					/*Reset the controller so that it starts executing from the downloaded firmware*/

					p = (uint8_t *) (p_buf );
					*p++=1;
					UINT16_TO_STREAM(p, HCI_CPU_RESET);
					*p++ =0;				/* parameter length */
					p = (uint8_t *) (p_buf );
				
					is_proceeding=write(vnd_userial.fd,p,4);
					
				}
				else
				{
					ALOGI("Downladed size %d not equal to firmware file size %d",hw_cfg_cb.fw_dl_progress,fw_size);
					is_proceeding=FALSE;
					break;
				}	
				#endif // NMI	
 				/* Normally the firmware patch configuration file
				 * sets the new starting baud rate at 115200.
				 * So, we need update host's baud rate accordingly.
				 */
				ms_delay(300);// make sure the FW is completely downloaded before changing the host baudrate

				get_closest_baud_rate(vnd_userial.bootrom_baudrate,&vnd_userial.actual_baud);
				userial_vendor_set_baud(vnd_userial.actual_baud);
				if(vnd_userial.fw_op_baudrate != vnd_userial.bootrom_baudrate)
				{
					baud_rate=get_closest_baud_rate(vnd_userial.fw_op_baudrate,&vnd_userial.actual_baud);
					is_proceeding=hw_config_update_ctrl_baud_rate(baud_rate,vnd_userial.flow_control);
					
				}

				else if(vnd_userial.enable_bdaddress_change != 0)
				{
					is_proceeding=hw_config_set_bdaddr(vnd_userial.bd_addr);
				}
				else
				{
					
					/* Next, we would like to boost baud rate up again
					* to desired working speed.
					*/
					hw_cfg_cb.f_set_baud_2 = TRUE;

					/* Check if we need to pause a few hundred milliseconds
					 * before sending down any HCI command.
					 */
					ms_delay(look_up_fw_settlement_delay());
					pwr_dev_deliver_event(CHIP_ALLOW_SLEP_AFTER_BT_FW_DOWNLOAD);
					bt_vendor_cbacks->fwcfg_cb(BT_VND_OP_RESULT_SUCCESS);

					hw_cfg_cb.fw_dl_progress = 0;
					hw_cfg_cb.fw_last_dl_size = 0;
					hw_cfg_cb.state = 0;
					is_proceeding=1;
				
					// In case we will go on with changing FW's baudrate using 
					// HCI command, remove this break and continue with fall through
				}
			break;

	
	
			case HW_CFG_SET_BAUD_RATE:
			ALOGI("bt vendor lib config callback (HW_CFG_SET_BAUD_RATE)");
			if(uart_close)
			{
				
				bt_vendor_cbacks->epilog_cb(BT_VND_OP_RESULT_SUCCESS);
				is_proceeding=1;
				uart_close=0;
				break;
			}
			userial_vendor_set_baud_ctsrts(vnd_userial.actual_baud, vnd_userial.flow_control);
			if(vnd_userial.enable_bdaddress_change != 0)
			{
				is_proceeding=hw_config_set_bdaddr(vnd_userial.bd_addr);
								
			}
			else
			{
				pwr_dev_deliver_event(CHIP_ALLOW_SLEP_AFTER_BT_FW_DOWNLOAD);
				bt_vendor_cbacks->fwcfg_cb(BT_VND_OP_RESULT_SUCCESS);
				is_proceeding=1;
			}
			break;
	

			case HW_CFG_START:
				ALOGI("bt vendor lib config callback (HW_CFG_START)");
				#if 0 // NMI: Not using 3M speed
				if (UART_TARGET_BAUD_RATE > 3000000)
				{
					/* set UART clock to 48MHz */
					UINT16_TO_STREAM(p, HCI_VSC_WRITE_UART_CLOCK_SETTING);
					*p++ = 1; /* parameter length */
					*p = 1; /* (1,"UART CLOCK 48 MHz")(2,"UART CLOCK 24 MHz") */

                    p_buf->len = HCI_CMD_PREAMBLE_SIZE + 1;
                    hw_cfg_cb.state = HW_CFG_SET_UART_CLOCK;

					is_proceeding = bt_vendor_cbacks->xmit_cb( \
										HCI_VSC_WRITE_UART_CLOCK_SETTING, \
										p_buf, hw_config_cback);
					break;
				}
				#endif
				/* fall through intentionally */

			case HW_CFG_CHECK_BT_CLK_SRC:
			
				/*read clock select register to check if cpll is done*/
				p_buf->event = MSG_STACK_TO_HC_HCI_CMD;
			    p_buf->offset = 0;
			    p_buf->layer_specific = 0;
			    p_buf->len = HCI_CMD_PREAMBLE_SIZE;
			
				p = (uint8_t *) (p_buf + 1);
			    UINT16_TO_STREAM(p, HCI_READ_MEM);
			    *p++ = 6; /* parameter length */
				UINT32_TO_STREAM(p, BT_CLK_SRC_REG);
				*p++=0x20;
				*p++=1;

				hw_cfg_cb.state = HW_CFG_SET_UART_CLOCK;
				p_buf->len = HCI_CMD_PREAMBLE_SIZE + \
	                         READ_MEMORY_CMD_PARAM_SIZE ;
				is_proceeding = bt_vendor_cbacks->xmit_cb(HCI_READ_MEM, \
	                                                p_buf, hw_config_cback);
				
				break;
				
		
			case HW_CFG_SET_UART_CLOCK:
	
			{
				uint8_t *clk_src;
				uint8_t *ret_buffer;
				ret_buffer = (uint8_t *)(p_evt_buf + 1) ;
				clk_src = (uint8_t *) (p_evt_buf + 1) + 7;
			//	for(i=0;i<30;i++)
				ALOGI("clk src=%x,read[%d]=%x",*clk_src,7,ret_buffer[7]);
				p_buf->event = MSG_STACK_TO_HC_HCI_CMD;
				p_buf->offset = 0;
				p_buf->len = 0;
				p_buf->layer_specific = 0;

		        p = (uint8_t *) (p_buf + 1);
				/* set controller's UART baud rate to 3M */
				ALOGI("bt vendor lib config callback (HW_CFG_SET_UART_CLOCK): set controller UART baud %i",  UART_TARGET_BAUD_RATE);
				UINT16_TO_STREAM(p, HCI_VSC_UPDATE_BAUDRATE);
				*p++ = UPDATE_BAUDRATE_CMD_PARAM_SIZE; /* parameter length */
				// NMI: We don't use encoded baud rate or encoded form
				//*p++ = 0; /* encoded baud rate */
				//*p++ = 0; /* use encoded form */
				//if(ret_buffer[7]==3)
					if(*clk_src==3)
				{
					ALOGI("BT Clock source is 3 ,Pll update si done");
					UINT32_TO_STREAM(p, vnd_userial.fw_dwnld_baudrate/2);
				}
				else
				{
					ALOGI("BT Clock source is 2 ,no PLL ");
					UINT32_TO_STREAM(p, vnd_userial.fw_dwnld_baudrate);
				}
				
			}
		
				*p++ = 0; /* NMI: No Flow control */

	            p_buf->len = HCI_CMD_PREAMBLE_SIZE + \
	                         UPDATE_BAUDRATE_CMD_PARAM_SIZE;
	            hw_cfg_cb.state = (hw_cfg_cb.f_set_baud_2) ? \
	                        HW_CFG_SET_UART_BAUD_2 : HW_CFG_SET_UART_BAUD_1;

	            is_proceeding = bt_vendor_cbacks->xmit_cb(HCI_VSC_UPDATE_BAUDRATE, \
	                                                p_buf, hw_config_cback);
			
			break;

			case HW_CFG_SET_UART_BAUD_2:
				/* update baud rate of host's UART port */
				ALOGI("bt vendor lib config callback (HW_CFG_SET_UART_BAUD_2): set host UART baud %i",  vnd_userial.fw_dwnld_baudrate);
			
				get_closest_baud_rate(vnd_userial.fw_dwnld_baudrate,&vnd_userial.actual_baud);
				userial_vendor_set_baud(vnd_userial.actual_baud);

				#if (USE_CONTROLLER_BDADDR == TRUE)
				if ((is_proceeding = hw_config_read_bdaddr(p_buf)) == TRUE)
					break;
				#else
				//if ((is_proceeding = hw_config_set_bdaddr(p_buf)) == TRUE)
				//	break;
				#endif
				/* fall through intentionally */
			case HW_CFG_SET_BD_ADDR:
				
				ALOGI("bt vendor lib config callback (HW_CFG_SET_BD_ADDR)");
				bt_vendor_cbacks->dealloc(p_buf);
				pwr_dev_deliver_event(CHIP_ALLOW_SLEP_AFTER_BT_FW_DOWNLOAD);
				bt_vendor_cbacks->fwcfg_cb(BT_VND_OP_RESULT_SUCCESS);

                hw_cfg_cb.state = 0;

                if (hw_cfg_cb.fw_fd != -1)
                {
                    close(hw_cfg_cb.fw_fd);
                    hw_cfg_cb.fw_fd = -1;
                }

                is_proceeding = TRUE;
                break;

			#if (USE_CONTROLLER_BDADDR == TRUE)
			case HW_CFG_READ_BD_ADDR:
				 ALOGI("bt vendor lib config callback (HW_CFG_READ_BD_ADDR)");
				p_tmp = (char *) (p_evt_buf + 1) + \
						 HCI_EVT_CMD_CMPL_LOCAL_BDADDR_ARRAY;

                if (memcmp(p_tmp, null_bdaddr, BD_ADDR_LEN) == 0)
                {
                    // Controller does not have a valid OTP BDADDR!
                    // Set the BTIF initial BDADDR instead.
                   // if ((is_proceeding = hw_config_set_bdaddr(p_buf)) == TRUE)
                   //     break;
                }
                else
                {
                    ALOGI("Controller OTP bdaddr %02X:%02X:%02X:%02X:%02X:%02X",
                        *(p_tmp+5), *(p_tmp+4), *(p_tmp+3),
                        *(p_tmp+2), *(p_tmp+1), *p_tmp);
                }

                ALOGI("vendor lib fwcfg completed");
                bt_vendor_cbacks->dealloc(p_buf);
                bt_vendor_cbacks->fwcfg_cb(BT_VND_OP_RESULT_SUCCESS);

                hw_cfg_cb.fw_dl_progress = 0;
				hw_cfg_cb.fw_last_dl_size = 0;
				hw_cfg_cb.state = 0;

                if (hw_cfg_cb.fw_fd != -1)
                {
                    close(hw_cfg_cb.fw_fd);
                    hw_cfg_cb.fw_fd = -1;
                }

                is_proceeding = TRUE;
                break;
#endif // (USE_CONTROLLER_BDADDR == TRUE)
        } // switch(hw_cfg_cb.state)
    } // if (p_buf != NULL)

    /* Free the RX event buffer */
    if (bt_vendor_cbacks)
        bt_vendor_cbacks->dealloc(p_evt_buf);

    if (is_proceeding == FALSE)
    {
        ALOGE("vendor lib fwcfg aborted!!!");
		
		set_bt_fw_downlading(0);

        if (bt_vendor_cbacks)
        {
            if (p_buf != NULL)
                bt_vendor_cbacks->dealloc(p_buf);

			pwr_dev_deliver_event(CHIP_ALLOW_SLEP_AFTER_BT_FW_DOWNLOAD);		

            bt_vendor_cbacks->fwcfg_cb(BT_VND_OP_RESULT_FAIL);
        }

        if (hw_cfg_cb.fw_fd != -1)
        {
            close(hw_cfg_cb.fw_fd);
            hw_cfg_cb.fw_fd = -1;
        }

        hw_cfg_cb.state = 0;
    }
}

/******************************************************************************
**   LPM Static Functions
******************************************************************************/

/*******************************************************************************
**
** Function         hw_lpm_ctrl_cback
**
** Description      Callback function for lpm enable/disable rquest
**
** Returns          None
**
*******************************************************************************/
void hw_lpm_ctrl_cback(void *p_mem)
{
    HC_BT_HDR *p_evt_buf = (HC_BT_HDR *) p_mem;
    bt_vendor_op_result_t status = BT_VND_OP_RESULT_FAIL;

    if (*((uint8_t *)(p_evt_buf + 1) + HCI_EVT_CMD_CMPL_STATUS_RET_BYTE) == 0)
    {
        status = BT_VND_OP_RESULT_SUCCESS;
    }

    if (bt_vendor_cbacks)
    {
        bt_vendor_cbacks->lpm_cb(status);
        bt_vendor_cbacks->dealloc(p_evt_buf);
    }
}


#if (SCO_CFG_INCLUDED == TRUE)
/*****************************************************************************
**   SCO Configuration Static Functions
*****************************************************************************/

/*******************************************************************************
**
** Function         hw_sco_cfg_cback
**
** Description      Callback function for SCO configuration rquest
**
** Returns          None
**
*******************************************************************************/
void hw_sco_cfg_cback(void *p_mem)
{
#if 0
    HC_BT_HDR *p_evt_buf = (HC_BT_HDR *) p_mem;
    uint8_t     *p;
    uint16_t    opcode;
    HC_BT_HDR  *p_buf=NULL;

    p = (uint8_t *)(p_evt_buf + 1) + HCI_EVT_CMD_CMPL_OPCODE;
    STREAM_TO_UINT16(opcode,p);

    /* Free the RX event buffer */
    if (bt_vendor_cbacks)
        bt_vendor_cbacks->dealloc(p_evt_buf);

#if (!defined(SCO_USE_I2S_INTERFACE) || (SCO_USE_I2S_INTERFACE == FALSE))
    if (opcode == HCI_VSC_WRITE_SCO_PCM_INT_PARAM)
    {
        uint8_t ret = FALSE;

        /* Ask a new buffer to hold WRITE_PCM_DATA_FORMAT_PARAM command */
        if (bt_vendor_cbacks)
            p_buf = (HC_BT_HDR *) bt_vendor_cbacks->alloc(BT_HC_HDR_SIZE + \
                                                HCI_CMD_PREAMBLE_SIZE + \
                                                PCM_DATA_FORMAT_PARAM_SIZE);
        if (p_buf)
        {
            p_buf->event = MSG_STACK_TO_HC_HCI_CMD;
            p_buf->offset = 0;
            p_buf->layer_specific = 0;
            p_buf->len = HCI_CMD_PREAMBLE_SIZE + PCM_DATA_FORMAT_PARAM_SIZE;

            p = (uint8_t *) (p_buf + 1);
            UINT16_TO_STREAM(p, HCI_VSC_WRITE_PCM_DATA_FORMAT_PARAM);
            *p++ = PCM_DATA_FORMAT_PARAM_SIZE;
            memcpy(p, &bt_pcm_data_fmt_param, PCM_DATA_FORMAT_PARAM_SIZE);

            if ((ret = bt_vendor_cbacks->xmit_cb(HCI_VSC_WRITE_PCM_DATA_FORMAT_PARAM,\
                                           p_buf, hw_sco_cfg_cback)) == FALSE)
            {
                bt_vendor_cbacks->dealloc(p_buf);
            }
            else
                return;
        }
    }
#endif  // !SCO_USE_I2S_INTERFACE

if (bt_vendor_cbacks)
    bt_vendor_cbacks->scocfg_cb(BT_VND_OP_RESULT_SUCCESS);
#endif
}
#endif // SCO_CFG_INCLUDED

/*****************************************************************************
**   Hardware Configuration Interface Functions
*****************************************************************************/


/*******************************************************************************
**
** Function        hw_config_start
**
** Description     Kick off controller initialization process
**
** Returns         None
**
*******************************************************************************/
void hw_config_start(void)
{
    HC_BT_HDR  *p_buf = NULL;
    uint8_t     *p;

   	uint8_t     is_proceeding=0,read_buf[256],bytes_read,i;
	ALOGI("hw_config_start");
    hw_cfg_cb.state = 0;
    hw_cfg_cb.fw_fd = -1;
    hw_cfg_cb.f_set_baud_2 = FALSE;
	uart_close=0;
	uint32_t baud_rate;

	strcpy(hw_cfg_cb.local_chip_name, AT_CHIP_NAME);
	if(vnd_userial.android_bt_fw_download_uart !=0)
	{

		if(!pwr_dev_deliver_event(CHIP_WAKEUP_FOR_BT_FW_DOWNLOAD))
		{	
		
			/* Must make sure serial port is open because /external/bluetooth/bluedroid/hci/src/bt_hci_bdroid.c 
			 don't check for userial_vendor_open() return, so if open failed, it will still call hw_config_start() */ 
			if(vnd_userial.fd == -1)
			{
				ALOGE("vendor lib fw conf aborted. serial port not open");
				pwr_dev_deliver_event(CHIP_ALLOW_SLEP_AFTER_BT_FW_DOWNLOAD);
				bt_vendor_cbacks->fwcfg_cb(BT_VND_OP_RESULT_FAIL);
			}
			else if(hw_cfg_cb.state != 0)
			{
				// Download in progress... don't interrupt
				AT_DBG("Download in progress, don't interrupt!");
				return;
			}
			else
			{
				if (bt_vendor_cbacks)		
					p_buf = (HC_BT_HDR  *)bt_vendor_cbacks->alloc(BT_HC_HDR_SIZE + \
		                                                       HCI_CMD_MAX_LEN );
				if (p_buf)
				{
					/*send read_local_version command to check if firmware is already downloaded*/
					p_buf->event = MSG_STACK_TO_HC_HCI_CMD;
		       		p_buf->offset = 0;
		       		p_buf->layer_specific = 0;
		       		p_buf->len = HCI_CMD_PREAMBLE_SIZE;
		
		       		p = (uint8_t *) (p_buf + 1);
		       		UINT16_TO_STREAM(p, HCI_READ_LOCAL_VERSION);
		      			*p = 0; /* parameter length */

					hw_cfg_cb.state = HW_CFG_CHECK_BOOTROM;
					set_bt_fw_downlading(1);
				
					ALOGI("[Atmel %s] sending read local version command",__func__);
		       		bt_vendor_cbacks->xmit_cb(HCI_READ_LOCAL_VERSION, p_buf, hw_config_cback);
				}
				else
				{
					pwr_dev_deliver_event(CHIP_ALLOW_SLEP_AFTER_BT_FW_DOWNLOAD);
					if(bt_vendor_cbacks)
		        	{
		        		ALOGE("vendor lib fw conf aborted [no buffer]");
						bt_vendor_cbacks->fwcfg_cb(BT_VND_OP_RESULT_FAIL);
		        	}
				}
			}
		}
		else
		{
			ALOGE("Chip wake up failed");
			bt_vendor_cbacks->fwcfg_cb(BT_VND_OP_RESULT_FAIL);
		}
	}
	else
	{
		ms_delay(5); //"BUG 1028" Delay required to avoid BT hang after callback sending
	
	
		ALOGI("vnd_userial.android_bt_fw_download= 0");
	   	ALOGI("Bluetooth Firmware and smd is initialized");
		ALOGI("Atmel: local bd address: %02x:%02x:%02x:%02x:%02x:%02x:", 
			vnd_local_bd_addr[0],vnd_local_bd_addr[1],vnd_local_bd_addr[2],
			vnd_local_bd_addr[3],vnd_local_bd_addr[4],vnd_local_bd_addr[5]);
		if(vnd_userial.fw_op_baudrate != FW_DEFAULT_BAUD_RATE || vnd_userial.flow_control == 1)
		{
			ALOGI("Raise host and controller baud rates to %d",vnd_userial.fw_op_baudrate);
			baud_rate=get_closest_baud_rate(vnd_userial.fw_op_baudrate,&vnd_userial.actual_baud);
			is_proceeding=hw_config_update_ctrl_baud_rate(baud_rate,vnd_userial.flow_control);
		}
		else if(vnd_userial.enable_bdaddress_change != 0)
		{
			is_proceeding=hw_config_set_bdaddr(vnd_userial.bd_addr);
		}
		else
			bt_vendor_cbacks->fwcfg_cb(BT_VND_OP_RESULT_SUCCESS);
			
	}

}

static uint8_t hw_config_reset(void)
{
	HC_BT_HDR  *p_buf = NULL;
    uint8_t     *p;
   	uint8_t     is_proceeding=0;
	uint8_t retval = FALSE;
	
 	ALOGI("atmel: bt_reset ");

	if(vnd_userial.fd == -1)
	{
		ALOGE("vendor lib fw conf aborted. serial port not open");
		bt_vendor_cbacks->fwcfg_cb(BT_VND_OP_RESULT_FAIL);
	}
	else
	{
		if (bt_vendor_cbacks)		
			p_buf = (HC_BT_HDR  *)bt_vendor_cbacks->alloc(BT_HC_HDR_SIZE + \
	                                                       HCI_CMD_MAX_LEN );
		if (p_buf)
		{
			p = (uint8_t *) (p_buf );
		
			*p++=1;//hci commmand
			
			UINT16_TO_STREAM(p, HCI_RESET);
			*p++ =0; /* parameter length */
					
			p = (uint8_t *) (p_buf );
	
			is_proceeding=write(vnd_userial.fd,p,4);

			retval = TRUE;
		}
	}
	return retval;
}


 uint8_t hw_config_update_ctrl_baud_rate(int baud , uint8_t flow_control )
 {
  	HC_BT_HDR  *p_buf = NULL;
    uint8_t     *p;
   	uint8_t     is_proceeding=0;
	uint8_t retval = FALSE;
	
	ALOGI("atmel: adjust controller baudrate  to %d",baud);
 	
	if(vnd_userial.fd == -1)
	{
		ALOGE("vendor lib fw conf aborted. serial port not open");
		pwr_dev_deliver_event(CHIP_ALLOW_SLEP_AFTER_BT_FW_DOWNLOAD);
		bt_vendor_cbacks->fwcfg_cb(BT_VND_OP_RESULT_FAIL);
	}
	else
	{
		if (bt_vendor_cbacks)		
			p_buf = (HC_BT_HDR  *)bt_vendor_cbacks->alloc(BT_HC_HDR_SIZE + \
	                                                       HCI_CMD_MAX_LEN );
		if (p_buf)
		{
	
			p_buf->event = MSG_STACK_TO_HC_HCI_CMD;
			p_buf->offset = 0;
			p_buf->layer_specific = 0;
			p_buf->len = HCI_CMD_PREAMBLE_SIZE;
			
			p = (uint8_t *) (p_buf + 1);
			UINT16_TO_STREAM(p, HCI_VSC_UPDATE_BAUDRATE);
			*p++ = UPDATE_BAUDRATE_CMD_PARAM_SIZE; /* parameter length */
			
			UINT32_TO_STREAM(p, baud);
			*p++ = flow_control; /* NMI: No Flow control */

			hw_cfg_cb.state = HW_CFG_SET_BAUD_RATE;
			p_buf->len = HCI_CMD_PREAMBLE_SIZE + \
	                         UPDATE_BAUDRATE_CMD_PARAM_SIZE ;
			is_proceeding = bt_vendor_cbacks->xmit_cb(HCI_VSC_UPDATE_BAUDRATE, \
	                                                p_buf, hw_config_cback);
				
		}
		else
		{
			pwr_dev_deliver_event(CHIP_ALLOW_SLEP_AFTER_BT_FW_DOWNLOAD);
			if(bt_vendor_cbacks)
		    {
		    	ALOGE("vendor lib fw conf aborted [no buffer]");
				bt_vendor_cbacks->fwcfg_cb(BT_VND_OP_RESULT_FAIL);
		    }
		}
	}

	return is_proceeding;
 }
 
/*******************************************************************************
**
** Function        hw_lpm_enable
**
** Description     Enalbe/Disable LPM
**
** Returns         TRUE/FALSE
**
*******************************************************************************/
uint8_t hw_lpm_enable(uint8_t turn_on)
{
    HC_BT_HDR  *p_buf = NULL;
    uint8_t     *p;
    uint8_t     ret = FALSE;

	ALOGI("<Atmel> [%s] (%d) turn_on: %d", __FUNCTION__, __LINE__, turn_on);

    if (bt_vendor_cbacks)
	{
		bt_vendor_cbacks->lpm_cb(BT_VND_OP_RESULT_SUCCESS);
		ret = TRUE;
    }
#if 0
    if (bt_vendor_cbacks)
        p_buf = (HC_BT_HDR *) bt_vendor_cbacks->alloc(BT_HC_HDR_SIZE + \
                                                       HCI_CMD_PREAMBLE_SIZE + \
                                                       LPM_CMD_PARAM_SIZE);

    if (p_buf)
    {
        p_buf->event = MSG_STACK_TO_HC_HCI_CMD;
        p_buf->offset = 0;
        p_buf->layer_specific = 0;
        p_buf->len = HCI_CMD_PREAMBLE_SIZE + LPM_CMD_PARAM_SIZE;

        p = (uint8_t *) (p_buf + 1);
        UINT16_TO_STREAM(p, HCI_VSC_WRITE_SLEEP_MODE);
        *p++ = LPM_CMD_PARAM_SIZE; /* parameter length */

        if (turn_on)
        {
            memcpy(p, &lpm_param, LPM_CMD_PARAM_SIZE);
        }
        else
        {
            memset(p, 0, LPM_CMD_PARAM_SIZE);
        }

        if ((ret = bt_vendor_cbacks->xmit_cb(HCI_VSC_WRITE_SLEEP_MODE, p_buf, \
                                        hw_lpm_ctrl_cback)) == FALSE)
        {
            bt_vendor_cbacks->dealloc(p_buf);
        }
    }

    if ((ret == FALSE) && bt_vendor_cbacks)
        bt_vendor_cbacks->lpm_cb(BT_VND_OP_RESULT_FAIL);
#endif
    return ret;
}

/*******************************************************************************
**
** Function        hw_lpm_get_idle_timeout
**
** Description     Calculate idle time based on host stack idle threshold
**
** Returns         idle timeout value
**
*******************************************************************************/
uint32_t hw_lpm_get_idle_timeout(void)
{
    uint32_t timeout_ms;

    /* set idle time to be LPM_IDLE_TIMEOUT_MULTIPLE times of
     * host stack idle threshold (in 300ms/25ms)
     */
    timeout_ms = (uint32_t)lpm_param.host_stack_idle_threshold \
                            * LPM_IDLE_TIMEOUT_MULTIPLE;

   	ALOGI("<Atmel> hw_lpm_get_idle_timeout chip name is %s", hw_cfg_cb.local_chip_name);

	/* This path is called from lpm_init(), by then, local_chip_name won't be set yet */
    if (strstr(hw_cfg_cb.local_chip_name, AT_CHIP_NAME) != NULL)
    {
    	ALOGI("<Atmel> hw_lpm_get_idle_timeout correct chip name, will get powersave timeout from userial");
    	timeout_ms = userial_get_powersave_timeout(); // Originaly was (12.5 or 25 ?) * LPM_IDLE_TIMEOUT_MULTIPLE (10)
    }
    else
        timeout_ms = userial_get_powersave_timeout(); // Originaly was 300 * LPM_IDLE_TIMEOUT_MULTIPLE
    return timeout_ms;
}

/*******************************************************************************
**
** Function        hw_lpm_set_wake_state
**
** Description     Assert/Deassert BT_WAKE
**
** Returns         None
**
*******************************************************************************/
void hw_lpm_set_wake_state(uint8_t wake_assert)
{
    uint8_t state = (wake_assert) ? UPIO_ASSERT : UPIO_DEASSERT;

    upio_set(UPIO_BT_WAKE, state, lpm_param.bt_wake_polarity);
}

#if (SCO_CFG_INCLUDED == TRUE)
/*******************************************************************************
**
** Function         hw_sco_config
**
** Description      Configure SCO related hardware settings
**
** Returns          None
**
*******************************************************************************/
void hw_sco_config(void)
{
#if 0
    HC_BT_HDR  *p_buf = NULL;
    uint8_t     *p, ret;

#if (!defined(SCO_USE_I2S_INTERFACE) || (SCO_USE_I2S_INTERFACE == FALSE))
    uint16_t cmd_u16 = HCI_CMD_PREAMBLE_SIZE + SCO_PCM_PARAM_SIZE;
#else
    uint16_t cmd_u16 = HCI_CMD_PREAMBLE_SIZE + SCO_I2SPCM_PARAM_SIZE;
#endif

    if (bt_vendor_cbacks)
        p_buf = (HC_BT_HDR *) bt_vendor_cbacks->alloc(BT_HC_HDR_SIZE+cmd_u16);

    if (p_buf)
    {
        p_buf->event = MSG_STACK_TO_HC_HCI_CMD;
        p_buf->offset = 0;
        p_buf->layer_specific = 0;
        p_buf->len = cmd_u16;

        p = (uint8_t *) (p_buf + 1);
#if (!defined(SCO_USE_I2S_INTERFACE) || (SCO_USE_I2S_INTERFACE == FALSE))
        UINT16_TO_STREAM(p, HCI_VSC_WRITE_SCO_PCM_INT_PARAM);
        *p++ = SCO_PCM_PARAM_SIZE;
        memcpy(p, &bt_sco_param, SCO_PCM_PARAM_SIZE);
        cmd_u16 = HCI_VSC_WRITE_SCO_PCM_INT_PARAM;
        ALOGI("SCO PCM configure {%d, %d, %d, %d, %d}",
           bt_sco_param[0], bt_sco_param[1], bt_sco_param[2], bt_sco_param[3], \
           bt_sco_param[4]);

#else
        UINT16_TO_STREAM(p, HCI_VSC_WRITE_I2SPCM_INTERFACE_PARAM);
        *p++ = SCO_I2SPCM_PARAM_SIZE;
        memcpy(p, &bt_sco_param, SCO_I2SPCM_PARAM_SIZE);
        cmd_u16 = HCI_VSC_WRITE_I2SPCM_INTERFACE_PARAM;
        ALOGI("SCO over I2SPCM interface {%d, %d, %d, %d}",
           bt_sco_param[0], bt_sco_param[1], bt_sco_param[2], bt_sco_param[3]);
#endif

        if ((ret=bt_vendor_cbacks->xmit_cb(cmd_u16, p_buf, hw_sco_cfg_cback)) \
             == FALSE)
        {
            bt_vendor_cbacks->dealloc(p_buf);
        }
        else
            return;
    }

    if (bt_vendor_cbacks)
    {
        ALOGE("vendor lib scocfg aborted");
        bt_vendor_cbacks->scocfg_cb(BT_VND_OP_RESULT_FAIL);
    }
	#endif
}
#endif  // SCO_CFG_INCLUDED

/*******************************************************************************
**
** Function        hw_set_patch_file_path
**
** Description     Set the location of firmware patch file
**
** Returns         0 : Success
**                 Otherwise : Fail
**
*******************************************************************************/
int hw_set_patch_file_path(char *p_conf_name, char *p_conf_value, int param)
{
    strcpy(fw_patchfile_path, p_conf_value);
	ALOGI("loaded patch file path: %s", fw_patchfile_path);

    return 0;
}

/*******************************************************************************
**
** Function        hw_set_patch_file_name
**
** Description     Give the specific firmware patch filename
**
** Returns         0 : Success
**                 Otherwise : Fail
**
*******************************************************************************/
int hw_set_patch_file_name(char *p_conf_name, char *p_conf_value, int param)
{
    strcpy(fw_patchfile_name, p_conf_value);
	ALOGI("loaded patch file name: %s", fw_patchfile_name);

    return 0;
}

#if (VENDOR_LIB_RUNTIME_TUNING_ENABLED == TRUE)
/*******************************************************************************
**
** Function        hw_set_patch_settlement_delay
**
** Description     Give the specific firmware patch settlement time in milliseconds
**
** Returns         0 : Success
**                 Otherwise : Fail
**
*******************************************************************************/
int hw_set_patch_settlement_delay(char *p_conf_name, char *p_conf_value, int param)
{
    fw_patch_settlement_delay = atoi(p_conf_value);
	ALOGI("loaded patch settlement delay: %d", fw_patch_settlement_delay);

    return 0;
}
#endif  //VENDOR_LIB_RUNTIME_TUNING_ENABLED

