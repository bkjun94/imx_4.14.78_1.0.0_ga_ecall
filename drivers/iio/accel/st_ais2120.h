/*
 * STMicroelectronics st_ais2120 sensor driver
 */

#ifndef ST_AIS2120_H
#define ST_AIS2120_H

#include <linux/iio/iio.h>

#define __swap32__(x)	( (((x) & 0xff000000) >> 24) |	\
						  (((x) & 0x00ff0000) >>  8) |	\
						  (((x) & 0x0000ff00) <<  8) |	\
  						  (((x) & 0x000000ff) << 24) )

// AIS2120 32bit SPI Communication
#define ST_AIS2120_DATA_SIZE		4
#define ST_AIS2120_RX_MAX_LENGTH	4
#define ST_AIS2120_TX_MAX_LENGTH	4

struct st_ais2120_transfer_buffer {
	u8 rx_buf[ST_AIS2120_RX_MAX_LENGTH];
	u8 tx_buf[ST_AIS2120_TX_MAX_LENGTH] ____cacheline_aligned;
};

struct st_ais2120_hw {
	struct device *dev;
	const char *name;
	int irq;
    int fir;
    int self_test_status;
	struct mutex lock;
	struct st_ais2120_transfer_buffer tb;
};

void ais2120_reg_make_cmd(u8 *buf, u8 readop, u8 addr, u8 data);
int ais2120_reg_decode_data(u8 *buf, u8* pdata);
void ais2120_sensor_make_cmd(u8 *buf, u8 channel);
int ais2120_sensor_decode_data(u8 *buf, u16* pdata);

int st_ais2120_write_with_mask(struct device *dev, u8 addr, u8 mask, u8 val);
int st_ais2120_spi_read(struct device *dev, u8 addr, int len, u8 *data);
//int st_ais2120_set_enable(struct st_ais2120_hw *hw, bool enable);


#endif /* ST_AIS2120_H */
