#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

#include "esp_log.h"
#include "esp_system.h"
#include "driver/i2c_master.h"

#include "icm20948.h"
#include "icm20948_i2c.h"

#define TAG "i2c_dmp_quat9_orientation"

/* ICM 20948 configuration */
icm0948_config_i2c_t icm_config = {
	.bus_handle = NULL,
	.dev_handle = NULL,
	.i2c_addr = ICM_20948_I2C_ADDR_AD0
};


void init_dmp(icm20948_device_t *icm)
{
  	bool success = true; // Use success to show if the DMP configuration was successful

  	// Initialize the DMP with defaults (includes magnetometer setup)
  	success &= (icm20948_init_dmp_sensor_with_defaults(icm) == ICM_20948_STAT_OK);
	
	// Enable the DMP orientation sensor (9-axis with magnetometer)
	success &= (inv_icm20948_enable_dmp_sensor(icm, INV_ICM20948_SENSOR_ORIENTATION, 1) == ICM_20948_STAT_OK);
	
	// Enable magnetometer for 9-axis fusion
	success &= (inv_icm20948_enable_dmp_sensor(icm, INV_ICM20948_SENSOR_MAGNETIC_FIELD_UNCALIBRATED, 1) == ICM_20948_STAT_OK);

	// Set ODR for Quat9 and magnetometer
	success &= (inv_icm20948_set_dmp_sensor_period(icm, DMP_ODR_Reg_Quat9, 0) == ICM_20948_STAT_OK);
	success &= (inv_icm20948_set_dmp_sensor_period(icm, DMP_ODR_Reg_Cpass, 0) == ICM_20948_STAT_OK);
	
	// Enable the FIFO
	success &= (icm20948_enable_fifo(icm, true) == ICM_20948_STAT_OK);
	// Enable the DMP
	success &= (icm20948_enable_dmp(icm, 1) == ICM_20948_STAT_OK);
	// Reset DMP
	success &= (icm20948_reset_dmp(icm) == ICM_20948_STAT_OK);
	// Reset FIFO
	success &= (icm20948_reset_fifo(icm) == ICM_20948_STAT_OK);

	// Check success
	if (success)
	{
		ESP_LOGI(TAG, "DMP enabled!");
	} else {
		ESP_LOGE(TAG, "Enable DMP failed!");
		while (1)
		; // Do nothing more
	}	
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
		.scl_speed_hz = 100000,
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

	/* now the fun with DMP starts */
	init_dmp(&icm);

    while(1)
	{
		// Read any DMP data waiting in the FIFO
		// Note:
		//    readDMPdataFromFIFO will return ICM_20948_STAT_FIFO_NO_DATA_AVAIL if no data is available.
		//    If data is available, readDMPdataFromFIFO will attempt to read _one_ frame of DMP data.
		//    readDMPdataFromFIFO will return ICM_20948_STAT_FIFO_INCOMPLETE_DATA if a frame was present but was incomplete
		//    readDMPdataFromFIFO will return ICM_20948_STAT_OK if a valid frame was read.
		//    readDMPdataFromFIFO will return ICM_20948_STAT_FIFO_MORE_DATA_AVAIL if a valid frame was read _and_ the FIFO contains more (unread) data.
		icm_20948_DMP_data_t data;
		icm20948_status_e status = inv_icm20948_read_dmp_data(&icm, &data);
		/* Was valid data available? */
  		if ((status == ICM_20948_STAT_OK) || (status == ICM_20948_STAT_FIFO_MORE_DATA_AVAIL)) 
		{
			/* We have asked for orientation data so we should receive Quat9 */
			if ((data.header & DMP_header_bitmap_Quat9) > 0) 
			{
				// Q0 value is computed from this equation: Q0^2 + Q1^2 + Q2^2 + Q3^2 = 1.
				// In case of drift, the sum will not add to 1, therefore, quaternion data need to be corrected with right bias values.
				// The quaternion data is scaled by 2^30.
				// Scale to +/- 1
				double q1 = ((double)data.Quat9.Data.Q1) / 1073741824.0; // Convert to double. Divide by 2^30
				double q2 = ((double)data.Quat9.Data.Q2) / 1073741824.0; // Convert to double. Divide by 2^30
				double q3 = ((double)data.Quat9.Data.Q3) / 1073741824.0; // Convert to double. Divide by 2^30
				double q0 = sqrt(1.0 - ((q1 * q1) + (q2 * q2) + (q3 * q3)));
				
				// Convert quaternion to Euler angles (roll, pitch, yaw)
				double roll = atan2(2.0 * (q0 * q1 + q2 * q3), 1.0 - 2.0 * (q1 * q1 + q2 * q2)) * 180.0 / M_PI;
				double sinp = 2.0 * (q0 * q2 - q3 * q1);
				double pitch = (fabs(sinp) >= 1) ? copysign(90.0, sinp) : asin(sinp) * 180.0 / M_PI;
				double yaw = atan2(2.0 * (q0 * q3 + q1 * q2), 1.0 - 2.0 * (q2 * q2 + q3 * q3)) * 180.0 / M_PI;
				
				ESP_LOGI(TAG, "Roll: %.2f° Pitch: %.2f° Yaw: %.2f° (Accuracy: %d)", roll, pitch, yaw, data.Quat9.Data.Accuracy);
			}
		}
		if(status != ICM_20948_STAT_FIFO_MORE_DATA_AVAIL) {
			vTaskDelay(10 / portTICK_PERIOD_MS);
		}
	}	
}
