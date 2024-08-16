/*
 * usbsid.c - Generic usbsid abstraction layer.
 *
 * Written by
 *  Andreas Boose <viceteam@t-online.de>
 *  usbsid Support <support@usbsid.com>
 *  Marco van den Heuvel <blackystardust68@yahoo.com>
 *  LouDnl
 *
 * This file is part of VICE, the Versatile Commodore Emulator.
 * See README for copyright notice.
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 *  02111-1307  USA.
 *
 */



#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <errno.h>
#include <libusb.h>
#include <string.h>
#include "vice.h"

#ifdef UNIX_COMPILE
#ifdef HAVE_USBSID


#include "usbsid.h"
#include "sid-snapshot.h"
#include "vicetypes.h"
#include "sid-resources.h"
#include "log.h"
#include "usbsid-macros.h"

/* #define DEBUG_USBSID */
#if defined(DEBUG_USBSID)
#define DBG(x)  printf x
#else
#define DBG(x)
#endif

#define DEBUG_USBSID_C
#ifdef DEBUG_USBSID_C
#define UDBG(...) fprintf(stdout, __VA_ARGS__)
#else
#define UDBG(...)
#endif

/* #define DEBUG_USBSIDMEM */
#ifdef DEBUG_USBSIDMEM
#define MDBG(...) printf(__VA_ARGS__)
uint8_t memory[65536];
#else
#define MDBG(...)
#endif

#define VENDOR_ID      0xcafe
#define PRODUCT_ID     0x4011
#define ACM_CTRL_DTR   0x01
#define ACM_CTRL_RTS   0x02
static int ep_out_addr = 0x02;
static int ep_in_addr  = 0x82;
static struct libusb_device_handle *devh = NULL;

/* set line encoding: here 9600 8N1
 * 9600 = 0x2580 -> 0x80, 0x25 in little endian
 * 115200 = 0x1C200 -> 0x00, 0xC2, 0x01 in little endian
 * 921600 = 0xE1000 -> 0x00, 0x10, 0x0E in little endian
 */
static unsigned char encoding[] = { 0x00, 0x10, 0x0E, 0x00, 0x00, 0x00, 0x08 };
static int rc;
static int actual_length;

static int sids_found = -1;
static int usid_dev = -1;
static int usbsid_is_open = -1;

/* buffer containing current register state of SIDs */
static uint8_t sidbuf[0x20 * US_MAXSID];

int usbsid_open(void)
{
    DBG(("usbsid_open, usbsid_is_open: %d\n", usbsid_is_open));
    if (usbsid_is_open) {
        usbsid_is_open = us_device_open();
        memset(sidbuf, 0, sizeof(sidbuf));
    }
    if (usbsid_is_open == -1)
        fprintf(stderr, "Failed to open USBSID\r\n");
    DBG(("usbsid_open usbsid_is_open=%d\r\n", usbsid_is_open));
    return usbsid_is_open;
}

int usbsid_close(void)
{
    if (!usbsid_is_open) {
        us_device_close();
        usbsid_is_open = -1;
    }
    DBG(("usbsid_close usbsid_is_open=%d\r\n", usbsid_is_open));
    return usbsid_is_open;
}

void usbsid_reset(void)
{
    if (!usbsid_is_open) {
        us_device_reset();
    }
}

int usbsid_read(uint16_t addr, int chipno)
{
    if (!usbsid_is_open && chipno < US_MAXSID) {
        int val = us_device_read(addr, chipno);
        sidbuf[(chipno * 0x20) + addr] = (uint8_t)val;
        DBG(("[R]@0x%04x [S]%d [?]%d\r\n", addr, chipno, addr + (chipno * 0x20)));
        return val;
    }

    return usbsid_is_open;
}

void usbsid_store(uint16_t addr, uint8_t val, int chipno)
{
    if (!usbsid_is_open && chipno < US_MAXSID) {
        /* write to sidbuf[] for write-only registers */
        sidbuf[(chipno * 0x20) + addr] = val;
        DBG(("[W]@0x%04x [S]%d [?]%d\r\n", addr, chipno, addr + (chipno * 0x20)));
        us_device_store(addr, val, chipno);
    }
}

int usbsid_available(void)
{
    DBG(("usbsid_available, usbsid_is_open: %d\r\n", usbsid_is_open));
    if (usbsid_is_open)
    {
        usbsid_open();
    }

    if (!usbsid_is_open) {
        return us_device_available();
    }
    return usbsid_is_open;
}

/* ------------------------- */

static int usbsid_init(void)
{

    /* Already open */
    if (usid_dev >= 0) {
        return -1;
    }
    // rc = 0;
    if (devh != NULL) {
        libusb_close(devh);
        // devh = -1;
    }

    /* Initialize libusb */
    rc = libusb_init(NULL);
	if (rc != 0) {
        fprintf(stderr, "Error initializing libusb: %s: %s\r\n",
        libusb_error_name(rc), libusb_strerror(rc));
        goto out;
    }

	/* Set debugging output to max level */
	// libusb_set_option(NULL, LIBUSB_OPTION_LOG_LEVEL, 0);
	// libusb_set_option(NULL, LIBUSB_OPTION_LOG_LEVEL, 3);

	/* Look for a specific device and open it */
	devh = libusb_open_device_with_vid_pid(NULL, VENDOR_ID, PRODUCT_ID);
    if (!devh) {
        fprintf(stderr, "Error opening USB device with VID & PID: %d\r\n", rc);
        rc = -1;
        goto out;
    }

	/* As we are dealing with a CDC-ACM device, it's highly probable that
     * Linux already attached the cdc-acm driver to this device.
     * We need to detach the drivers from all the USB interfaces. The CDC-ACM
     * Class defines two interfaces: the Control interface and the
     * Data interface.
     */
    for (int if_num = 0; if_num < 2; if_num++) {
        if (libusb_kernel_driver_active(devh, if_num)) {
            libusb_detach_kernel_driver(devh, if_num);
        }
        rc = libusb_claim_interface(devh, if_num);
        if (rc < 0) {
            fprintf(stderr, "Error claiming interface: %d, %s: %s\r\n",
            rc, libusb_error_name(rc), libusb_strerror(rc));
            goto out;
        }
    }

    /* Start configuring the device:
     * - set line state */
    rc = libusb_control_transfer(devh, 0x21, 0x22, ACM_CTRL_DTR | ACM_CTRL_RTS, 0, NULL, 0, 0);
    if (rc != 0 && rc != 7) {
        fprintf(stderr, "?Error configuring line state during control transfer: %d, %s: %s\r\n",
            rc, libusb_error_name(rc), libusb_strerror(rc));
        goto out;
    }

    /* - set line encoding here */
    rc = libusb_control_transfer(devh, 0x21, 0x20, 0, 0, encoding, sizeof(encoding), 0);
    if (rc != 0 && rc != 7) {
        fprintf(stderr, "Error configuring line encoding during control transfer: %d, %s: %s\r\n",
            rc, libusb_error_name(rc), libusb_strerror(rc));
        goto out;
    }

    usid_dev = (rc == 0 || rc == 7) ? 0 : -1;

    if (usid_dev < 0)
    {
        fprintf(stderr, "Could not open SID device USBSID\r\n");
        goto out;
    }

	return usid_dev;
out:
    us_device_close();
    return rc;
}

void us_device_reset(void)
{
    if (sids_found > 0) {
        unsigned char data[4] = {0x3, 0x0, 0x0, 0x0};
        int size = sizeof(data);
        if (libusb_bulk_transfer(devh, ep_out_addr, data, size, &actual_length, 0) < 0)
        {
            fprintf(stderr, "Error while sending reset to sid\r\n");
        }
        UDBG("USBSID reset!\r\n");
    }
}

int us_device_open(void)
{
    if (!sids_found) {
        return -1;
    }

    if (sids_found > 0) {
        return sids_found;
    }

    sids_found = 0;

    fprintf(stdout, "Detecting Linux usbsid boards\r\n");

    if (usid_dev != 0) {
        rc = usbsid_init();
        if (rc != 0) {
            return -1;
        }
    }

    fprintf(stdout, "Linux usbsid boards detected [rc]%d [usid_dev]%d [sids_found]%d\r\n", rc, usid_dev, sids_found);

    sids_found = 1;
    usbsid_reset(); /* eventually calls us_device_reset  */

    /* zero length read to clear any lingering data */
    int transferred = 0;
    unsigned char buffer[1];
    // const int res = libusb_bulk_transfer(devh, 0x82, buffer, 0, &transferred, 1);
    libusb_bulk_transfer(devh, ep_out_addr, buffer, 0, &transferred, 1);

    fprintf(stdout, "Linux usbsid: opened\r\n");

    #ifdef DEBUG_USBSIDMEM
	for (unsigned int i = 0; i < 65536; i++)
	{
		if (i >= 0xD400 || i <= 0xD430 ) {
			/* do nothing */
		} else {
			memory[i] = 0x00; // fill with NOPs
		}
	}
    #endif

    return 0;
}

int us_device_close(void)
{
    /* Driver cleans up after itself */
    if (devh != NULL) {
        for (int if_num = 0; if_num < 2; if_num++) {
            libusb_release_interface(devh, if_num);
            if (libusb_kernel_driver_active(devh, if_num)) {
                libusb_detach_kernel_driver(devh, if_num);
            }
        }
        libusb_close(devh);
        libusb_exit(NULL);
    }
    /* Clean up vars */
    sids_found = -1;
    usid_dev = -1;
    rc = -1;
    devh = NULL;
    fprintf(stdout, "Linux usbsid: closed\r\n");
    return 0;
}

int us_device_read(uint16_t addr, int chipno)
{
    if (chipno < US_MAXSID && usid_dev >= 0)
    {
        unsigned char data[4];  /* Read buffer where each byte should be the same value */
        memset(data, 0, sizeof data);
        addr &= 0x1F;                                 /* remove address limitation */
        addr = (addr + (chipno * 0x20));
        unsigned char wdata[4] = {0x1, 0xD4, addr, 0x0};  /* set addr write data for read */  /* NOTICE: addr is limited to $D400 range */
        int actual_lengthw;  /* Stores the actual length of the read data */
        if (libusb_bulk_transfer(devh, ep_out_addr, wdata, sizeof(wdata), &actual_lengthw, 0) < 0)
        {
            fprintf(stderr, "Error while sending char\r\n");
        }
        UDBG("[S#]%d RW@[0x%02x] (0b"PRINTF_BINARY_PATTERN_INT8") ", chipno, addr, PRINTF_BYTE_TO_BINARY_INT8(addr));
        int actual_lengthr;  /* Stores the actual length of the read data */
        rc = 0;
        rc = libusb_bulk_transfer(devh, ep_in_addr, data, sizeof(data), &actual_lengthr, 1000);
        if (rc == LIBUSB_ERROR_TIMEOUT) {
            fprintf(stdout, "Timeout (%d)\r\n", actual_lengthr);
            return -1;
        } else if (rc < 0) {
            fprintf(stdout, "Error while waiting for char\r\n");
            return -1;
        }
        UDBG("[S#]%d RCV[0x%02x] (0b"PRINTF_BINARY_PATTERN_INT8")\r\n", chipno, data[0], PRINTF_BYTE_TO_BINARY_INT8(data[0]));
        return data[0];
    }
    return 0;
}

void us_device_store(uint16_t addr, uint8_t val, int chipno) /* max chipno = 1 */
{
    if (chipno < US_MAXSID && usid_dev >= 0) {  /* remove 0x20 address limitation */
        #ifdef DEBUG_USBSIDMEM
        memory[(0xD400 | addr)] = val;
        #endif
        addr &= 0x1F;
        addr = (addr + (chipno * 0x20));
        unsigned char data[4] = {0x0, 0xD4, addr, val}; /* NOTICE: addr is limited to $D400 range */
        int size = sizeof(data);
        if (libusb_bulk_transfer(devh, ep_out_addr, data, size, &actual_length, 0) < 0)
        {
            fprintf(stdout, "Error while sending char\r\n");
        }
        UDBG("[S#]%d WR@[0x%02x] (0b"PRINTF_BINARY_PATTERN_INT8") DAT[0x%02x] (0b"PRINTF_BINARY_PATTERN_INT8")\r\n", chipno, addr, PRINTF_BYTE_TO_BINARY_INT8(addr), val, PRINTF_BYTE_TO_BINARY_INT8(val));
        #ifdef DEBUG_USBSIDMEM
        MDBG("one single memwrite ~ addr: %04x byte: %04x phyaddr: %04x | Synth 1: $%02X%02X %02X%02X %02X %02X %02X | Synth 2: $%02X%02X %02X%02X %02X %02X %02X | Synth 3: $%02X%02X %02X%02X %02X %02X %02X\n",
		addr, val, laddr,
		memory[0xD400], memory[0xD401], memory[0xD402], memory[0xD403], memory[0xD404], memory[0xD405], memory[0xD406],
		memory[0xD407], memory[0xD408], memory[0xD409], memory[0xD40A], memory[0xD40B], memory[0xD40C], memory[0xD40D],
		memory[0xD40E], memory[0xD40F], memory[0xD410], memory[0xD411], memory[0xD412], memory[0xD413], memory[0xD414]);
        #endif
    }
}

unsigned int us_device_available(void)
{
    return sids_found;
}



#else
int usbsid_available(void)
{
    return 0;
}
#endif /* HAVE_USBSID */
#endif /* UNIX_COMPILE */
