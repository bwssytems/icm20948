#include <stdio.h>
#include <unistd.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

#include "esp_log.h"
#include "esp_system.h"
#include "driver/i2c_master.h"

#include "icm20948.h"
#include "icm20948_i2c.h"

#define TAG "i2c_agmt"

/* ICM 20948 configuration */
icm0948_config_i2c_t icm_config = {
	.bus_handle = NULL,
	.dev_handle = NULL,
	.i2c_addr = ICM_20948_I2C_ADDR_AD0
};


void print_agmt(icm20948_agmt_t agmt)
{
  	ESP_LOGI(TAG, "Acc: [ %d, %d, %d ] Gyr: [ %d, %d, %d ] Mag: [ %d, %d, %d ] Tmp: [ %d ]", 
		agmt.acc.axes.x, agmt.acc.axes.y, agmt.acc.axes.z,
		agmt.gyr.axes.x, agmt.gyr.axes.y, agmt.gyr.axes.z,
		agmt.mag.axes.x, agmt.mag.axes.y, agmt.mag.axes.z,
		agmt.tmp.val
	);
}

void app_main(void)
{
	icm20948_device_t icm;

	/* setup i2c bus */
	i2c_master_bus_config_t bus_config = {
		.i2c_port = I2C_NUM_0,
		.sda_io_num = CONFIG_I2C_MASTER_SDA,
		.scl_io_num = CONFIG_I2C_MASTER_SCL,
		.clk_source = I2C_CLK_SRC_DEFAULT,
		.glitch_ignore_cnt = 7,
		.flags.enable_internal_pullup = true,
	};
	ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, &icm_config.bus_handle));

	/* setup i2c device */
	i2c_device_config_t dev_config = {
		.dev_addr_length = I2C_ADDR_BIT_LEN_7,
		.device_address = icm_config.i2c_addr,
		.scl_speed_hz = 400000,
	};
	ESP_ERROR_CHECK(i2c_master_bus_add_device(icm_config.bus_handle, &dev_config, &icm_config.dev_handle));
	vTaskDelay(100 / portTICK_PERIOD_MS);
	
	/* setup ICM20948 device */
	icm20948_init_i2c(&icm, &icm_config);
	
	/* wake up device first */
	icm20948_sleep(&icm, false);
	vTaskDelay(50 / portTICK_PERIOD_MS);
		
	/* check ID */
	uint8_t whoami = 0x00;
	icm20948_status_e stat = icm20948_get_who_am_i(&icm, &whoami);
	ESP_LOGI(TAG, "WHO_AM_I read status: %d, value: 0x%02X (expected: 0xEA)", stat, whoami);
	
    while (icm20948_check_id(&icm) != ICM_20948_STAT_OK)
	{
		stat = icm20948_get_who_am_i(&icm, &whoami);
		ESP_LOGE(TAG, "check id failed - status: %d, WHO_AM_I: 0x%02X", stat, whoami);
		vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
	ESP_LOGI(TAG, "check id passed");

	/* Here we are doing a SW reset to make sure the device starts in a known state */
	icm20948_sw_reset(&icm);
	vTaskDelay(250 / portTICK_PERIOD_MS);

	icm20948_internal_sensor_id_bm sensors = (icm20948_internal_sensor_id_bm)(ICM_20948_INTERNAL_ACC | ICM_20948_INTERNAL_GYR);

	// Set Gyro and Accelerometer to a particular sample mode
	// optiona: SAMPLE_MODE_CONTINUOUS. SAMPLE_MODE_CYCLED
	icm20948_set_sample_mode(&icm, sensors, SAMPLE_MODE_CONTINUOUS); 

	// Set full scale ranges for both acc and gyr
	icm20948_fss_t myfss;
	myfss.a = GPM_2;   // (icm20948_accel_config_fs_sel_e)
	myfss.g = DPS_250; // (icm20948_gyro_config_1_fs_sel_e)
	icm20948_set_full_scale(&icm, sensors, myfss);

	// Set up DLPF configuration
	icm20948_dlpcfg_t myDLPcfg;
	myDLPcfg.a = ACC_D473BW_N499BW;
	myDLPcfg.g = GYR_D361BW4_N376BW5;
	icm20948_set_dlpf_cfg(&icm, sensors, myDLPcfg);

	// Choose whether or not to use DLPF
	icm20948_enable_dlpf(&icm, ICM_20948_INTERNAL_ACC, false);
	icm20948_enable_dlpf(&icm, ICM_20948_INTERNAL_GYR, false);

	// Now wake the sensor up
	icm20948_sleep(&icm, false);
	icm20948_low_power(&icm, false);

    /* loop */
    while(1)
	{
		vTaskDelay(100 / portTICK_PERIOD_MS);

		icm20948_agmt_t agmt; // = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}, {0}};
		if (icm20948_get_agmt(&icm, &agmt) == ICM_20948_STAT_OK) {
			print_agmt(agmt);
		} else {
			ESP_LOGE(TAG, "Uh oh");
		}        
    }
}
