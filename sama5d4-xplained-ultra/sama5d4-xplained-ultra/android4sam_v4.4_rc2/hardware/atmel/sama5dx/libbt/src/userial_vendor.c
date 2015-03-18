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
 *  Filename:      userial_vendor.c
 *
 *  Description:   Contains vendor-specific userial functions
 *
 ******************************************************************************/

#define LOG_TAG "bt_userial_vendor"

#include <utils/Log.h>
#include <termios.h>
#include <fcntl.h>
#include <errno.h>
#include <stdio.h>
#include "bt_vendor.h"
#include "userial.h"
#include "userial_vendor.h"

/******************************************************************************
**  Constants & Macros
******************************************************************************/

#ifndef VNDUSERIAL_DBG
#define VNDUSERIAL_DBG FALSE
#endif

#if (VNDUSERIAL_DBG == TRUE)
#define VNDUSERIALDBG(param, ...) {ALOGD(param, ## __VA_ARGS__);}
#else
#define VNDUSERIALDBG(param, ...) {}
#endif



/******************************************************************************
**  Local type definitions
******************************************************************************/



/******************************************************************************
**  Static variables
******************************************************************************/

vnd_userial_cb_t vnd_userial;

/*****************************************************************************
**   Helper Functions
*****************************************************************************/

/*******************************************************************************
**
** Function        userial_to_tcio_baud
**
** Description     helper function converts USERIAL baud rates into TCIO
**                  conforming baud rates
**
** Returns         TRUE/FALSE
**
*******************************************************************************/
uint8_t userial_to_tcio_baud(uint8_t cfg_baud, uint32_t *baud)
{
    if (cfg_baud == USERIAL_BAUD_115200)
        *baud = B115200;
    else if (cfg_baud == USERIAL_BAUD_4M)
        *baud = B4000000;
    else if (cfg_baud == USERIAL_BAUD_3M)
        *baud = B3000000;
    else if (cfg_baud == USERIAL_BAUD_2M)
        *baud = B2000000;
	else if (cfg_baud == USERIAL_BAUD_1_5M)
        *baud = B1500000;
    else if (cfg_baud == USERIAL_BAUD_1M)
        *baud = B1000000;
    else if (cfg_baud == USERIAL_BAUD_921600)
        *baud = B921600;
    else if (cfg_baud == USERIAL_BAUD_460800)
        *baud = B460800;
    else if (cfg_baud == USERIAL_BAUD_230400)
        *baud = B230400;
    else if (cfg_baud == USERIAL_BAUD_57600)
        *baud = B57600;
    else if (cfg_baud == USERIAL_BAUD_19200)
        *baud = B19200;
    else if (cfg_baud == USERIAL_BAUD_9600)
        *baud = B9600;
    else if (cfg_baud == USERIAL_BAUD_1200)
        *baud = B1200;
    else if (cfg_baud == USERIAL_BAUD_600)
        *baud = B600;
    else
    {
        ALOGE( "userial vendor open: unsupported baud idx %i", cfg_baud);
        *baud = B115200;
        return FALSE;
    }

    return TRUE;
}

/*******************************************************************************
**
** Function        line_speed_to_userial_baud
**
** Description     helper function converts line speed number into USERIAL baud
**                 rate symbol
**
** Returns         unit8_t (USERIAL baud symbol)
**
*******************************************************************************/
uint8_t line_speed_to_userial_baud(uint32_t line_speed)
{
    uint8_t baud;

    if (line_speed == 4000000)
        baud = USERIAL_BAUD_4M;
    else if (line_speed == 3000000)
        baud = USERIAL_BAUD_3M;
    else if (line_speed == 2000000)
        baud = USERIAL_BAUD_2M;
	else if (line_speed == 1500000)
        baud = USERIAL_BAUD_1_5M;
    else if (line_speed == 1000000)
        baud = USERIAL_BAUD_1M;
    else if (line_speed == 921600)
        baud = USERIAL_BAUD_921600;
    else if (line_speed == 460800)
        baud = USERIAL_BAUD_460800;
    else if (line_speed == 230400)
        baud = USERIAL_BAUD_230400;
    else if (line_speed == 115200)
        baud = USERIAL_BAUD_115200;
    else if (line_speed == 57600)
        baud = USERIAL_BAUD_57600;
    else if (line_speed == 19200)
        baud = USERIAL_BAUD_19200;
    else if (line_speed == 9600)
        baud = USERIAL_BAUD_9600;
    else if (line_speed == 1200)
        baud = USERIAL_BAUD_1200;
    else if (line_speed == 600)
        baud = USERIAL_BAUD_600;
    else
    {
        ALOGE( "userial vendor: unsupported baud speed %d", line_speed);
        baud = USERIAL_BAUD_115200;
    }

    return baud;
}
#if (BT_WAKE_VIA_USERIAL_IOCTL==TRUE)
/*******************************************************************************
**
** Function        userial_ioctl_init_bt_wake
**
** Description     helper function to set the open state of the bt_wake if ioctl
**                  is used. it should not hurt in the rfkill case but it might
**                  be better to compile it out.
**
** Returns         none
**
*******************************************************************************/
void userial_ioctl_init_bt_wake(int fd)
{
    uint32_t bt_wake_state;
	ALOGD("<Atmel> %s %d", __FUNCTION__, __LINE__);
    /* assert BT_WAKE through ioctl */
   // ioctl(fd, USERIAL_IOCTL_BT_WAKE_ASSERT, NULL);
//    ioctl(fd, USERIAL_IOCTL_BT_WAKE_GET_ST, &bt_wake_state);
    VNDUSERIALDBG("userial_ioctl_init_bt_wake read back BT_WAKE state=%i", \
               bt_wake_state);
}
#endif // (BT_WAKE_VIA_USERIAL_IOCTL==TRUE)


/*****************************************************************************
**   Userial Vendor API Functions
*****************************************************************************/

/*******************************************************************************
**
** Function        userial_vendor_init
**
** Description     Initialize userial vendor-specific control block
**
** Returns         None
**
*******************************************************************************/
void userial_vendor_init(void)
{
    vnd_userial.fd = -1;
    //snprintf(vnd_userial.port_name, VND_PORT_NAME_MAXLEN, "%s", \
    //        BLUETOOTH_UART_DEVICE_PORT);
	vnd_userial.android_bt_fw_download= 0;
    vnd_userial.flow_control = 0;
	vnd_userial.fw_op_baudrate = 921600;
	strcpy(vnd_userial.port_name, "/dev/ttyS2");
	vnd_userial.bd_addr[0]=0x1;
	vnd_userial.bd_addr[1]=0x23;
	vnd_userial.bd_addr[2]=0x45;
	vnd_userial.bd_addr[3]=0x67;
	vnd_userial.bd_addr[4]=0x89;
	vnd_userial.bd_addr[5]=0xAB;
	vnd_userial.enable_bdaddress_change = 0;
	
	vnd_userial.is_powersave_enabled = 0;
	vnd_userial.powersave_timeout = 50;
}

#define ASYNCB_SPD_HI		 4 /* Use 56000 instead of 38400 bps */
#define ASYNCB_SPD_VHI		 5 /* Use 115200 instead of 38400 bps */
#define ASYNCB_SPD_SHI		12 /* Use 230400 instead of 38400 bps */
#define ASYNC_SPD_SHI		(1U << ASYNCB_SPD_SHI)
#define ASYNC_SPD_HI		(1U << ASYNCB_SPD_HI)
#define ASYNC_SPD_VHI		(1U << ASYNCB_SPD_VHI)

#define ASYNC_SPD_CUST		(ASYNC_SPD_HI|ASYNC_SPD_VHI)
#define ASYNC_SPD_WARP		(ASYNC_SPD_HI|ASYNC_SPD_SHI)
#define ASYNC_SPD_MASK		(ASYNC_SPD_HI|ASYNC_SPD_VHI|ASYNC_SPD_SHI)

struct serial_struct
{
	int	type;	
	int	line;
	unsigned int	port;
	int	irq;
	int	flags;
	int	xmit_fifo_size;
	int	custom_divisor;
	int	baud_base;
	unsigned short	close_delay;
	char	io_type;
	char	reserved_char[1];
	int	hub6;
	unsigned short	closing_wait; /* time to wait before closing */
	unsigned short	closing_wait2; /* no longer used... */
	unsigned char	*iomem_base;
	unsigned short	iomem_reg_shift;
	unsigned int	port_high;
	unsigned long	iomap_base;	/* cookie passed into ioremap */
};

/*******************************************************************************
**
** Function        userial_vendor_open
**
** Description     Open the serial port with the given configuration
**
** Returns         device fd
**
*******************************************************************************/
int userial_vendor_open(tUSERIAL_CFG *p_cfg)
{
    uint32_t baud;
    uint8_t data_bits;
    uint16_t parity;
    uint8_t stop_bits;

    vnd_userial.fd = -1;

    if(p_cfg->fmt & USERIAL_DATABITS_8)
        data_bits = CS8;
    else if(p_cfg->fmt & USERIAL_DATABITS_7)
        data_bits = CS7;
    else if(p_cfg->fmt & USERIAL_DATABITS_6)
        data_bits = CS6;
    else if(p_cfg->fmt & USERIAL_DATABITS_5)
        data_bits = CS5;
    else
    {
        ALOGE("userial vendor open: unsupported data bits");
        return -1;
    }

    if(p_cfg->fmt & USERIAL_PARITY_NONE)
        parity = 0;
    else if(p_cfg->fmt & USERIAL_PARITY_EVEN)
        parity = PARENB;
    else if(p_cfg->fmt & USERIAL_PARITY_ODD)
        parity = (PARENB | PARODD);
    else
    {
        ALOGE("userial vendor open: unsupported parity bit mode");
        return -1;
    }

    if(p_cfg->fmt & USERIAL_STOPBITS_1)
        stop_bits = 0;
    else if(p_cfg->fmt & USERIAL_STOPBITS_2)
        stop_bits = CSTOPB;
    else
    {
        ALOGE("userial vendor open: unsupported stop bits");
        return -1;
    }

	ALOGI("userial vendor open: opening %s", vnd_userial.port_name);
	
    if ((vnd_userial.fd = open(vnd_userial.port_name, O_RDWR)) == -1)
    {
        ALOGE("userial vendor open: unable to open %s", vnd_userial.port_name);
        return -1;
    }

    tcflush(vnd_userial.fd, TCIOFLUSH);

	if (!userial_to_tcio_baud(p_cfg->baud, &baud))
	{
		uint32_t expectedbaudrate;
		uint32_t actualbaudrate;
		struct serial_struct ss;
		float error;
		
	    // configure port to use custom speed instead of 38400
	    if(ioctl(vnd_userial.fd, TIOCGSERIAL, &ss) < 0) {
			ALOGE("<Atmel> Cannot get serial port");
			return -1;
		}
		expectedbaudrate = vnd_userial.fw_op_baudrate;
		ss.flags = (ss.flags & ~ASYNC_SPD_MASK) | ASYNC_SPD_CUST;
		ss.custom_divisor = (ss.baud_base + (expectedbaudrate / 2)) / expectedbaudrate; 
		actualbaudrate = ss.baud_base / ss.custom_divisor;

		if(ioctl(vnd_userial.fd, TIOCSSERIAL, &ss) < 0) {
			ALOGE("<Atmel> Cannot set serial port");
			return -1;
		}

		baud = B38400;
		error = 1 - (expectedbaudrate/actualbaudrate);
		ALOGI("Expected baud rate: %d, Actual baud rate: %d, custom_divisor: %d, baud_base: %d, error: %f", 
			expectedbaudrate, actualbaudrate, ss.custom_divisor, ss.baud_base, actualbaudrate);
	}
	
    tcgetattr(vnd_userial.fd, &vnd_userial.termios);
    cfmakeraw(&vnd_userial.termios);

    if(vnd_userial.flow_control == 0)
    {
    	vnd_userial.termios.c_cflag |= (stop_bits);
		vnd_userial.termios.c_cflag &= ~CRTSCTS;
    }
    else
    {
        vnd_userial.termios.c_cflag |= (CRTSCTS | stop_bits);
    }
    tcsetattr(vnd_userial.fd, TCSANOW, &vnd_userial.termios);
    tcflush(vnd_userial.fd, TCIOFLUSH);

    tcsetattr(vnd_userial.fd, TCSANOW, &vnd_userial.termios);
    tcflush(vnd_userial.fd, TCIOFLUSH);
    tcflush(vnd_userial.fd, TCIOFLUSH);

    /* set input/output baudrate */
    cfsetospeed(&vnd_userial.termios, baud);
    cfsetispeed(&vnd_userial.termios, baud);
    tcsetattr(vnd_userial.fd, TCSANOW, &vnd_userial.termios);

#if (BT_WAKE_VIA_USERIAL_IOCTL==TRUE)
    userial_ioctl_init_bt_wake(vnd_userial.fd);
#endif

    ALOGI("device fd = %d open", vnd_userial.fd);

    return vnd_userial.fd;
}

/*******************************************************************************
**
** Function        userial_vendor_close
**
** Description     Conduct vendor-specific close work
**
** Returns         None
**
*******************************************************************************/
void userial_vendor_close(void)
{
    int result;

    if (vnd_userial.fd == -1)
        return;

#if (BT_WAKE_VIA_USERIAL_IOCTL==TRUE)
    /* de-assert bt_wake BEFORE closing port */
	//    ioctl(vnd_userial.fd, USERIAL_IOCTL_BT_WAKE_DEASSERT, NULL);
	// TODO: let the chip sleep at close!Then wake it up at startup
#endif

    ALOGI("device fd = %d close", vnd_userial.fd);

    if ((result = close(vnd_userial.fd)) < 0)
        ALOGE( "close(fd:%d) FAILED result:%d", vnd_userial.fd, result);

    vnd_userial.fd = -1;
}

/*******************************************************************************
**
** Function        userial_vendor_set_baud
**
** Description     Set new baud rate
**
** Returns         None
**
*******************************************************************************/
void userial_vendor_set_baud(uint8_t userial_baud)
{
    uint32_t tcio_baud;

    userial_to_tcio_baud(userial_baud, &tcio_baud);
	//AT_DBG("Setting baud rate to %d", userial_baud);

    cfsetospeed(&vnd_userial.termios, tcio_baud);
    cfsetispeed(&vnd_userial.termios, tcio_baud);
    tcsetattr(vnd_userial.fd, TCSADRAIN, &vnd_userial.termios);//TCSADRAIN to wait until everything has been transmitted
}

/*******************************************************************************
**
** Function        userial_vendor_ioctl
**
** Description     ioctl inteface
**
** Returns         None
**
*******************************************************************************/
void userial_vendor_ioctl(userial_vendor_ioctl_op_t op, void *p_data)
{
	ALOGI("<Atmel: [%s] (%d) op: %d>", __FUNCTION__, __LINE__, op);
	if(vnd_userial.is_powersave_enabled == TRUE)
	{
	    switch(op)
	    {
	#if (BT_WAKE_VIA_USERIAL_IOCTL==TRUE)
	        case USERIAL_OP_ASSERT_BT_WAKE:
	            VNDUSERIALDBG("## userial_vendor_ioctl: Asserting BT_Wake ##");
	            //ioctl(vnd_userial.fd, USERIAL_IOCTL_BT_WAKE_ASSERT, NULL);
	            ioctl(vnd_userial.fd, TIOCCBRK, 0);
	            break;

	        case USERIAL_OP_DEASSERT_BT_WAKE:
	            VNDUSERIALDBG("## userial_vendor_ioctl: De-asserting BT_Wake ##");
	            //ioctl(vnd_userial.fd, USERIAL_IOCTL_BT_WAKE_DEASSERT, NULL);
	            ioctl(vnd_userial.fd, TIOCSBRK, NULL); // break again
	            break;

	        case USERIAL_OP_GET_BT_WAKE_STATE:
	           // ioctl(vnd_userial.fd, USERIAL_IOCTL_BT_WAKE_GET_ST, p_data);
	           // TODO: do we need to get the status?
	            break;
	#endif  //  (BT_WAKE_VIA_USERIAL_IOCTL==TRUE)

	        default:
	            break;
	    }
	}
}

/*******************************************************************************
**
** Function        userial_get_powersave_timeout
**
** Description     Returns the powersave timeout value in ms
**
** Returns         0 : Success
**                 Otherwise : Fail
**
*******************************************************************************/
int userial_get_powersave_timeout(void)
{
	return vnd_userial.powersave_timeout;
}

/*******************************************************************************
**
** Function        userial_set_port
**
** Description     Configure UART port name
**
** Returns         0 : Success
**                 Otherwise : Fail
**
*******************************************************************************/
int userial_set_port(char *p_conf_name, char *p_conf_value, int param)
{
    //p_conf_value = "/dev/ttyUSB0";
    strcpy(vnd_userial.port_name, p_conf_value);
	ALOGI("loaded port name: %s", vnd_userial.port_name);
		
    return 0;
}

/*******************************************************************************
**
** Function        userial_set_fw_op_baudrate
**
** Description     Configure Firmware UART baudrate
**
** Returns         0 : Success
**                 Otherwise : Fail
**
*******************************************************************************/
int userial_set_fw_op_baudrate(char *p_conf_name, char *p_conf_value, int param)
{
    vnd_userial.fw_op_baudrate = (atoi(p_conf_value)<4000000)?(atoi(p_conf_value)):(UART_TARGET_BAUD_RATE);
	ALOGI("loaded operational baud rate: %d", vnd_userial.fw_op_baudrate);
	
    return 0;
}

/*******************************************************************************
**
** Function        userial_set_flow_control
**
** Description     set flow control
**
** Returns         0 : Success
**                 Otherwise : Fail
**
*******************************************************************************/
int userial_set_flow_control(char *p_conf_name, char *p_conf_value, int param)
{
    vnd_userial.flow_control= atoi(p_conf_value);
	ALOGI("loaded flow control: %d", vnd_userial.flow_control);
	
    return 0;
}

/*******************************************************************************
**
** Function        userial_set_bt_fw_download_flag
**
** Description     Control downloading bt firmware from android
**
** Returns         0 : Success
**                 Otherwise : Fail
**
*******************************************************************************/
int userial_set_bt_fw_download_flag(char *p_conf_name, char *p_conf_value, int param)
{
    vnd_userial.android_bt_fw_download= atoi(p_conf_value);
	ALOGI("AndroidBtFwDownload: %d", vnd_userial.android_bt_fw_download);
	
    return 0;
}

int userial_set_bt_bd_addr(char *p_conf_name, char *p_conf_value, int param)
{
	char bd_addr_str[13],bd_addr_hex[6];
	uint8_t i,j;
	strcpy(bd_addr_str, p_conf_value);
	ALOGI("loaded bd address %s ", bd_addr_str);
	
	
	
	for(i=0;i<12;i++)
	{
	
		if(bd_addr_str[i]<='9'&&bd_addr_str[i]>='0')
		{
			bd_addr_str[i]-='0';
			
		}
		else if(bd_addr_str[i]<='F'&&bd_addr_str[i]>='A')
		{
			
			bd_addr_str[i]=bd_addr_str[i]-'A'+10;
			
		}
		else if( bd_addr_str[i]<='f'&& bd_addr_str[i]>='a')
		{
			
			bd_addr_str[i]=bd_addr_str[i]-'a'+10;
			
		}
		else
		{
			ALOGI("Wrong bd address");
			return -1;
			
		}
		
	}
	
	for(i=0,j=0; i < 6 ; i++,j+=2)
    {		
        
		vnd_userial.bd_addr[5-i] =( (bd_addr_str[j])* 16) + bd_addr_str[j+1] ;
		
    }
	

    return 0;
}

int userial_set_enable_bd_address_change(char *p_conf_name, char *p_conf_value, int param)
{
	vnd_userial.enable_bdaddress_change = atoi(p_conf_value);
	ALOGI("EnableBDAddressChange: %d", vnd_userial.enable_bdaddress_change);
	
    return 0;
}

/*******************************************************************************
**
** Function        userial_set_bluetooth_powersave
**
** Description     Configure UART powersave enable
**
** Returns         0 : Success
**                 Otherwise : Fail
**
*******************************************************************************/
int userial_set_bluetooth_powersave(char *p_conf_name, char *p_conf_value, int param)
{
    vnd_userial.is_powersave_enabled = (atoi(p_conf_value) == 0)? FALSE:TRUE;
	ALOGI("loaded Power save mode: %d", vnd_userial.is_powersave_enabled);
	
    return 0;
}

/*******************************************************************************
**
** Function        userial_set_flow_control
**
** Description     Configure UART powersave enable
**
** Returns         0 : Success
**                 Otherwise : Fail
**
*******************************************************************************/
int userial_set_bluetooth_timeout(char *p_conf_name, char *p_conf_value, int param)
{
    vnd_userial.powersave_timeout = atoi(p_conf_value);
	ALOGI("loaded Power save timeout: %d", vnd_userial.powersave_timeout);
	
    return 0;
}


