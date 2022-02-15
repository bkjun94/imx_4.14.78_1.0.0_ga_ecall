/*
 * STMicroelectronics st_ais2120 sensor driver
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/types.h>

#include "st_ais2120.h"

/*
 * 32-bit communication protocol
 *
 * The communication between slave and master is transmitted by 32-bit data word, MSB first.
 * An off-frame protocol is used meaning that each transfer is completed through a sequence of 2 phases.
 * The answer of a given request is sent within the very next frame.
 */

const static u8 crc8table[256] =
{
	0x00, 0x2F, 0x5E, 0x71, 0xBC, 0x93, 0xE2, 0xCD, 0x57, 0x78, 0x09, 0x26, 0xEB, 0xC4, 0xB5, 0x9A,
	0xAE, 0x81, 0xF0, 0xDF, 0x12, 0x3D, 0x4C, 0x63, 0xF9, 0xD6, 0xA7, 0x88, 0x45, 0x6A, 0x1B, 0x34,
	0x73, 0x5C, 0x2D, 0x02, 0xCF, 0xE0, 0x91, 0xBE, 0x24, 0x0B, 0x7A, 0x55, 0x98, 0xB7, 0xC6, 0xE9,
	0xDD, 0xF2, 0x83, 0xAC, 0x61, 0x4E, 0x3F, 0x10, 0x8A, 0xA5, 0xD4, 0xFB, 0x36, 0x19, 0x68, 0x47,
	0xE6, 0xC9, 0xB8, 0x97, 0x5A, 0x75, 0x04, 0x2B, 0xB1, 0x9E, 0xEF, 0xC0, 0x0D, 0x22, 0x53, 0x7C,
	0x48, 0x67, 0x16, 0x39, 0xF4, 0xDB, 0xAA, 0x85, 0x1F, 0x30, 0x41, 0x6E, 0xA3, 0x8C, 0xFD, 0xD2,
	0x95, 0xBA, 0xCB, 0xE4, 0x29, 0x06, 0x77, 0x58, 0xC2, 0xED, 0x9C, 0xB3, 0x7E, 0x51, 0x20, 0x0F,
	0x3B, 0x14, 0x65, 0x4A, 0x87, 0xA8, 0xD9, 0xF6, 0x6C, 0x43, 0x32, 0x1D, 0xD0, 0xFF, 0x8E, 0xA1,
	0xE3, 0xCC, 0xBD, 0x92, 0x5F, 0x70, 0x01, 0x2E, 0xB4, 0x9B, 0xEA, 0xC5, 0x08, 0x27, 0x56, 0x79,
	0x4D, 0x62, 0x13, 0x3C, 0xF1, 0xDE, 0xAF, 0x80, 0x1A, 0x35, 0x44, 0x6B, 0xA6, 0x89, 0xF8, 0xD7,
	0x90, 0xBF, 0xCE, 0xE1, 0x2C, 0x03, 0x72, 0x5D, 0xC7, 0xE8, 0x99, 0xB6, 0x7B, 0x54, 0x25, 0x0A,
	0x3E, 0x11, 0x60, 0x4F, 0x82, 0xAD, 0xDC, 0xF3, 0x69, 0x46, 0x37, 0x18, 0xD5, 0xFA, 0x8B, 0xA4,
	0x05, 0x2A, 0x5B, 0x74, 0xB9, 0x96, 0xE7, 0xC8, 0x52, 0x7D, 0x0C, 0x23, 0xEE, 0xC1, 0xB0, 0x9F,
	0xAB, 0x84, 0xF5, 0xDA, 0x17, 0x38, 0x49, 0x66, 0xFC, 0xD3, 0xA2, 0x8D, 0x40, 0x6F, 0x1E, 0x31,
	0x76, 0x59, 0x28, 0x07, 0xCA, 0xE5, 0x94, 0xBB, 0x21, 0x0E, 0x7F, 0x50, 0x9D, 0xB2, 0xC3, 0xEC,
	0xD8, 0xF7, 0x86, 0xA9, 0x64, 0x4B, 0x3A, 0x15, 0x8F, 0xA0, 0xD1, 0xFE, 0x33, 0x1C, 0x6D, 0x42
};

/* ais2120 POLY(0x2F) CRC Table Generator Application */
#if 0
/*
   Generate a table for a byte-wise 8-bit CRC calculation on the polynomial:
   The ais2120 applied polynomial is X^8 + X^5 + X^3 + X^2 + X^1 + x^0 */
*/
#include <stdint.h>
#include <stdio.h>
void make_crc_table( void ) {
    int i, j;
    unsigned long poly, c;
    /* terms of polynomial defining this crc (except x^8): */
    static const uint8_t p[] = {0, 1, 2, 3, 5};

    /* make exclusive-or pattern from polynomial (0x2F) */
    poly = 0L;
    for ( i = 0; i < sizeof( p ) / sizeof( uint8_t ); i++ ) {
        poly |= 1L << p[i];
    }

    for ( i = 0; i < 256; i++ ) {
        c = i;
        for ( j = 0; j < 8; j++ ) {
            c = ( c & 0x80 ) ? poly ^ ( c << 1 ) : ( c << 1 );
        }
        crc8table[i] = (uint8_t) c;
    }
}

void main()
{
    make_crc_table();

    for (int i; i<256 ; i++){
        printf("%02X, ", crc8table[i]);
        if ((i+1)%16 == 0) {
            printf("\n");
        }
    }
}
#endif

static u8 calculate_crc8(u8 crc, u8 *buf, int buflen)
{
	int i;
	for (i=0;i<buflen;i++)
	{
		crc = crc ^ buf[i];			 /* apply byte */
		crc = crc8table[crc & 0xff]; /* one round of 8-bits */
	}
	return crc;
}

static u8 check_odd_parity(u32 cmd)
{
	int i, counts = 0;
	for (i=8;i<32;i++)
		if (cmd & (1 << i)) counts++;
	return (counts & 1) ? 0 : 1;
}

void ais2120_reg_make_cmd(u8 *buf, u8 readop, u8 addr, u8 data)
{
	u32 cmd = 0;
	cmd |= (readop ? 3 : 1) << 30;
	cmd |= (0 << 29); // non-sensor data
	cmd |= (addr & 0x1f) << 21; // register address for R/W operations
	if (readop == 0) {
        cmd |= data << 13;
    }
	cmd |= check_odd_parity(cmd) << 28; // set parity (odd parity, cover bit 31-8)
	*(u32 *)buf = __swap32__(cmd);
	buf[3] = calculate_crc8(0, buf, 3);
	//printk("[gschoi] cmd: 0x%08x\n", cmd);
}

int ais2120_reg_decode_data(u8 *buf, u8* pdata)
{
	u32 res = __swap32__(*(u32 *)buf);
	u8 sen = (res >> 31) & 1;
	u8 op = (res >> 29) & 3;
	u8 parity = (res >> 28) & 1;
	u8 addr = (res >> 21) & 0x1f;
	u8 data = (res >> 13) & 0xff;
	u8 status = (res >> 8) & 0x0f;
	u8 crc = res & 0xff;
	u8 ccrc = calculate_crc8(0, buf, 3);
    if (crc != ccrc) {
        printk("[ais2120] reg rx data crc error (%02x:%02x)\n", crc, ccrc);
    }else {
        if (pdata)
		    *pdata = data;
    }
	//printk("[gschoi] response[0x%08X] sen=%d,op=%d,par=%d,addr=%d,data=0x%02x,sf=%d,crc(%02X:%02X)\n", res, sen, op, parity, addr, data, status, crc, ccrc);
	return status;
}

void ais2120_sensor_make_cmd(u8 *buf, u8 channel)
{
	u32 cmd = 0;
	cmd |= (channel & 3) << 30;
	cmd |= (1 << 29); // sensor data
	cmd |= check_odd_parity(cmd) << 28; // set parity (odd parity, cover bit 31-8)
	*(u32 *)buf = __swap32__(cmd);
	buf[3] = calculate_crc8(0, buf, 3);
	//printk("[gschoi] sensor_cmd: 0x%08x\n", cmd);
}

int ais2120_sensor_decode_data(u8 *buf, u16* pdata)
{
	u32 res = __swap32__(*(u32 *)buf);
	u8 sen = (res >> 31) & 1;
	u8 ch = (res >> 29) & 3;
	u8 parity = (res >> 28) & 1;
	u8 st = (res >> 26) & 3;
	u16 data = (res >> 12) & 0x3fff; // 14-bit data
	u8 status = (res >> 8) & 0x0f;
	u8 crc = res & 0xff;
	u8 ccrc = calculate_crc8(0, buf, 3);
    if (crc != ccrc) {
        printk("[ais2120] sensor rx data crc error (%02x:%02x)\n", crc, ccrc);
    }else {
    	if (pdata)
    		*pdata = data;
    }
	//printk("[gschoi] response[0x%08X] sen=%d,ch=%d,par=%d,st=%d,data=0x%04x,sf=%d,crc(%02X:%02X)\n", res, sen, ch, parity, st, data, status, crc, ccrc);
	return status;
}

