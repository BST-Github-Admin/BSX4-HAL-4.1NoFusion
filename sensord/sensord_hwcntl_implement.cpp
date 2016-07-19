/*!
 * @section LICENSE
 *
 * (C) Copyright 2011~2015 Bosch Sensortec GmbH All Rights Reserved
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 *------------------------------------------------------------------------------
 *  Disclaimer
 *
 * Common: Bosch Sensortec products are developed for the consumer goods
 * industry. They may only be used within the parameters of the respective valid
 * product data sheet.  Bosch Sensortec products are provided with the express
 * understanding that there is no warranty of fitness for a particular purpose.
 * They are not fit for use in life-sustaining, safety or security sensitive
 * systems or any system or device that may lead to bodily harm or property
 * damage if the system or device malfunctions. In addition, Bosch Sensortec
 * products are not fit for use in products which interact with motor vehicle
 * systems.  The resale and/or use of products are at the purchaser's own risk
 * and his own responsibility. The examination of fitness for the intended use
 * is the sole responsibility of the Purchaser.
 *
 * The purchaser shall indemnify Bosch Sensortec from all third party claims,
 * including any claims for incidental, or consequential damages, arising from
 * any product use not covered by the parameters of the respective valid product
 * data sheet or not approved by Bosch Sensortec and reimburse Bosch Sensortec
 * for all costs in connection with such claims.
 *
 * The purchaser must monitor the market for the purchased products,
 * particularly with regard to product safety and inform Bosch Sensortec without
 * delay of all security relevant incidents.
 *
 * Engineering Samples are marked with an asterisk (*) or (e). Samples may vary
 * from the valid technical specifications of the product series. They are
 * therefore not intended or fit for resale to third parties or for use in end
 * products. Their sole purpose is internal client testing. The testing of an
 * engineering sample may in no way replace the testing of a product series.
 * Bosch Sensortec assumes no liability for the use of engineering samples. By
 * accepting the engineering samples, the Purchaser agrees to indemnify Bosch
 * Sensortec from all claims arising from the use of engineering samples.
 *
 * Special: This software module (hereinafter called "Software") and any
 * information on application-sheets (hereinafter called "Information") is
 * provided free of charge for the sole purpose to support your application
 * work. The Software and Information is subject to the following terms and
 * conditions:
 *
 * The Software is specifically designed for the exclusive use for Bosch
 * Sensortec products by personnel who have special experience and training. Do
 * not use this Software if you do not have the proper experience or training.
 *
 * This Software package is provided `` as is `` and without any expressed or
 * implied warranties, including without limitation, the implied warranties of
 * merchantability and fitness for a particular purpose.
 *
 * Bosch Sensortec and their representatives and agents deny any liability for
 * the functional impairment of this Software in terms of fitness, performance
 * and safety. Bosch Sensortec and their representatives and agents shall not be
 * liable for any direct or indirect damages or injury, except as otherwise
 * stipulated in mandatory applicable law.
 *
 * The Information provided is believed to be accurate and reliable. Bosch
 * Sensortec assumes no responsibility for the consequences of use of such
 * Information nor for any infringement of patents or other rights of third
 * parties which may result from its use.
 *
 * @file         sensord_hwcntl_implement.cpp
 * @date         "Thu Jul 14 17:05:20 2016 +0800"
 * @commit       "27a99f5"
 *
 * @brief
 *
 * @detail
 *
 */

#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/cdefs.h>
#include <sys/stat.h>
#include <linux/input.h>
#include <sys/timerfd.h>
#include <poll.h>
#include <fcntl.h>
#include <dirent.h>

#if !defined(UNIT_TEST_ACTIVE)
#include<android/log.h>
/*Android utils headers*/
#include <utils/SystemClock.h>
#include <utils/Timers.h>
#endif

#include "BstSensor.h"

#include "axis_remap.h"
#include "sensord_hwcntl.h"
#include "sensord_pltf.h"
#include "sensord_cfg.h"
#include "sensord_algo.h"
#include "util_misc.h"
#include "sensord_hwcntl_util.h"

/* input event definition
struct input_event {
    struct timeval time;
    __u16 type;
    __u16 code;
    __s32 value;
};

EV_SYN            0x00 EV_KEY            0x01 EV_REL            0x02
EV_ABS            0x03 EV_MSC            0x04 EV_SW            0x05 EV_LED            0x11
EV_SND            0x12 EV_REP            0x14 EV_FF            0x15 EV_PWR            0x16
EV_FF_STATUS        0x17 EV_MAX            0x1f EV_CNT            (EV_MAX+1)
*/


#define BMI160_ACCEL_ODR_RESERVED       0x00
#define BMI160_ACCEL_ODR_0_78HZ         0x01
#define BMI160_ACCEL_ODR_1_56HZ         0x02
#define BMI160_ACCEL_ODR_3_12HZ         0x03
#define BMI160_ACCEL_ODR_6_25HZ         0x04
#define BMI160_ACCEL_ODR_12_5HZ         0x05
#define BMI160_ACCEL_ODR_25HZ           0x06
#define BMI160_ACCEL_ODR_50HZ           0x07
#define BMI160_ACCEL_ODR_100HZ          0x08
#define BMI160_ACCEL_ODR_200HZ          0x09
#define BMI160_ACCEL_ODR_400HZ          0x0A
#define BMI160_ACCEL_ODR_800HZ          0x0B
#define BMI160_ACCEL_ODR_1600HZ         0x0C

/* BMI160 Gyro ODR */
#define BMI160_GYRO_ODR_RESERVED   0x00
#define BMI160_GYRO_ODR_25HZ   0x06
#define BMI160_GYRO_ODR_50HZ   0x07
#define BMI160_GYRO_ODR_100HZ   0x08
#define BMI160_GYRO_ODR_200HZ   0x09
#define BMI160_GYRO_ODR_400HZ   0x0A
#define BMI160_GYRO_ODR_800HZ   0x0B
#define BMI160_GYRO_ODR_1600HZ   0x0C
#define BMI160_GYRO_ODR_3200HZ   0x0D

/* BMI160 Mag ODR */
#define BMI160_MAG_ODR_RESERVED       0x00
#define BMI160_MAG_ODR_0_78HZ         0x01
#define BMI160_MAG_ODR_1_56HZ         0x02
#define BMI160_MAG_ODR_3_12HZ         0x03
#define BMI160_MAG_ODR_6_25HZ         0x04
#define BMI160_MAG_ODR_12_5HZ         0x05
#define BMI160_MAG_ODR_25HZ           0x06
#define BMI160_MAG_ODR_50HZ           0x07
#define BMI160_MAG_ODR_100HZ          0x08
#define BMI160_MAG_ODR_200HZ          0x09
#define BMI160_MAG_ODR_400HZ          0x0A
#define BMI160_MAG_ODR_800HZ          0x0B
#define BMI160_MAG_ODR_1600HZ         0x0C

/**for BMA2x2, ODR = Filter Bandwidth x 2*/
#define BMA2X2_BW_7_81HZ        0x08
#define BMA2X2_BW_15_63HZ       0x09
#define BMA2X2_BW_31_25HZ       0x0A
#define BMA2X2_BW_62_50HZ       0x0B
#define BMA2X2_BW_125HZ         0x0C
#define BMA2X2_BW_250HZ         0x0D
#define BMA2X2_BW_500HZ         0x0E
#define BMA2X2_BW_1000HZ        0x0F
#define BMA2X2_ODR_15_63HZ       BMA2X2_BW_7_81HZ
#define BMA2X2_ODR_31_25HZ       BMA2X2_BW_15_63HZ
#define BMA2X2_ODR_62_50HZ       BMA2X2_BW_31_25HZ
#define BMA2X2_ODR_125HZ         BMA2X2_BW_62_50HZ
#define BMA2X2_ODR_250HZ         BMA2X2_BW_125HZ
#define BMA2X2_ODR_500HZ         BMA2X2_BW_250HZ
#define BMA2X2_ODR_1000HZ        BMA2X2_BW_500HZ
#define BMA2X2_ODR_2000HZ        BMA2X2_BW_1000HZ

/**for BMG160, ODR mapped to Filter Bandwidth according to this table
 * ODR_Hz      Bandwidth
 *  100         32Hz
 *  200         64Hz
 *  100         12Hz    --- optimal for 100Hz
 *  200         23Hz    --- optimal for 200Hz
 *  400         47Hz
 *  1000        116Hz
 *  2000        230Hz   --- optimal for 2000Hz
 *  2000        523Hz(Unfiltered)*/
#define BMG160_BW_12HZ          0x05
#define BMG160_BW_23HZ          0x04
#define BMG160_BW_32HZ          0x07
#define BMG160_BW_47HZ          0x03
#define BMG160_BW_64HZ          0x06
#define BMG160_BW_116HZ         0x02
#define BMG160_BW_230HZ         0x01
#define BMG160_BW_523HZ         0x00
#define BMG160_ODR_100HZ        BMG160_BW_12HZ
#define BMG160_ODR_200HZ        BMG160_BW_23HZ
#define BMG160_ODR_400HZ        BMG160_BW_47HZ
#define BMG160_ODR_1000HZ       BMG160_BW_116HZ
#define BMG160_ODR_2000HZ       BMG160_BW_230HZ

#define BMI160_ACCEL_RANGE_2G   3
#define BMI160_ACCEL_RANGE_4G   5
#define BMI160_ACCEL_RANGE_8G   8
#define BMI160_ACCEL_RANGE_16G  12

#define BMA2X2_RANGE_2G     3
#define BMA2X2_RANGE_4G     5
#define BMA2X2_RANGE_8G     8
#define BMA2X2_RANGE_16G    12

#define BMA2x2_FIFO_PASSBY  0
#define BMA2x2_FIFO_STREAM  2
#define BMG160_FIFO_PASSBY  0
#define BMG160_FIFO_STREAM  2

#define BMA222E_ADC_BITS    8
#define BMA250E_ADC_BITS    10
#define BMA255_ADC_BITS     12
#define BMA280_ADC_BITS     14
#define BMM_COMPVAL_TO_uT (16) //bmm compensation value need divide 16 to become uT
#define AKM09912_COMPVAL_TO_uT (0.15) //AKM09912 driver output is in unit of 0.15uT
#define AKM09911_COMPVAL_TO_uT (0.6) //AKM09911 driver output is in unit of 0.6uT
#define AKM09915_COMPVAL_TO_uT (0.15) //AKM09915 driver output is in unit of 0.15uT
#define AKM09916_COMPVAL_TO_uT (0.15) //AKM09916 driver output is in unit of 0.15uT
#define YAS5xx_COMPVAL_TO_uT (0.001) //YAS5xx driver output is in unit of 0.001uT
#define MMC_COMPVAL_TO_mG (0.9765625) //driver output uses 1024 counter to represent 1000mG(1000 * 0.1uT)

#define EVENT_CODE_HW_SIGMOTION    3
#define EVENT_CODE_HW_STEPD        5
#define EVENT_CODE_HW_FOC          6

#if !defined(UNIT_TEST_ACTIVE)
static char mag_input_dir_name[128] = {0};
static char acc_input_dir_name[128] = {0};
static char gyr_input_dir_name[128] = {0};
static char bmi_input_dir_name[128] = {0};

static float BMI160_acc_resl = 0.061; //16bit ADC, default range +-2000 mg. algorithm input requires "mg"
static float BMA255_acc_resl = 0.97656; //12bit ADC, default range +-2000 mg. algorithm input requires "mg"
static float BMA280_acc_resl = 0.244; //14bit ADC, default range +-2000 mg. algorithm input requires "mg"
#endif
/**
 *
 * @param p_sSensorList
 * @return
 */
uint32_t ap_get_sensorlist(struct sensor_t const** p_sSensorList)
{
    uint64_t avail_sens_regval = 0;
    uint32_t sensor_amount = 0;
    int32_t i;
    int32_t j;

    if (0 == bst_sensorlist.list_len)
    {
        switch(accl_range){
            case ACC_CHIP_RANGCONF_2G:
            case ACC_CHIP_RANGCONF_4G:
            case ACC_CHIP_RANGCONF_8G:
            case ACC_CHIP_RANGCONF_16G:
                bst_all_sensors[SENSORLIST_INX_ACCELEROMETER].maxRange = accl_range * GRAVITY_EARTH;
                bst_all_sensors[SENSORLIST_INX_LINEAR_ACCELERATION].maxRange = accl_range * GRAVITY_EARTH;
                bst_all_sensors[SENSORLIST_INX_GRAVITY].maxRange = accl_range * GRAVITY_EARTH;
                bst_all_sensors[SENSORLIST_INX_WAKEUP_ACCELEROMETER].maxRange = accl_range * GRAVITY_EARTH;
                bst_all_sensors[SENSORLIST_INX_WAKEUP_LINEAR_ACCELERATION].maxRange = accl_range * GRAVITY_EARTH;
                bst_all_sensors[SENSORLIST_INX_WAKEUP_GRAVITY].maxRange = accl_range * GRAVITY_EARTH;
                break;
            default:
                PWARN("Invalid accl_range: %d", accl_range);
                break;
        }

        if(MAG_CHIP_AKM09912 ==  magn_chip || MAG_CHIP_AKM09911 ==  magn_chip ||
                MAG_CHIP_AKM09915 ==  magn_chip || MAG_CHIP_AKM09916 ==  magn_chip)
        {
            bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD].name = "AKM Magnetic Field Sensor";
            bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD].vendor = "AKM";
            bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD].maxRange = 4912.0f;
            bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD_UNCALIBRATED].name = "AKM Magnetic Field Uncalibrated Sensor";
            bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD_UNCALIBRATED].vendor = "AKM";
            bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD_UNCALIBRATED].maxRange = 4912.0f;
            bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_FIELD].name = "AKM Magnetic Field (Wakeup) Sensor";
            bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_FIELD].vendor = "AKM";
            bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_FIELD].maxRange = 4912.0f;
            bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_FIELD_UNCALIBRATED].name = "AKM Magnetic Field Uncalibrated (Wakeup) Sensor";
            bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_FIELD_UNCALIBRATED].vendor = "AKM";
            bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_FIELD_UNCALIBRATED].maxRange = 4912.0f;
            switch (magn_chip) {
                case MAG_CHIP_AKM09912:
                    bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD].resolution = 0.15;
                    bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD].power = 1.0f;
                    bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD_UNCALIBRATED].resolution = 0.15;
                    bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD_UNCALIBRATED].power = 1.0f;
                    bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_FIELD].resolution = 0.15;
                    bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_FIELD].power = 1.0f;
                    bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_FIELD_UNCALIBRATED].resolution = 0.15;
                    bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_FIELD_UNCALIBRATED].power = 1.0f;
                    break;
                case MAG_CHIP_AKM09911:
                    bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD].resolution = 0.6;
                    bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD].power = 2.4f;
                    bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD_UNCALIBRATED].resolution = 0.6;
                    bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD_UNCALIBRATED].power = 2.4f;
                    bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_FIELD].resolution = 0.6;
                    bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_FIELD].power = 2.4f;
                    bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_FIELD_UNCALIBRATED].resolution = 0.6;
                    bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_FIELD_UNCALIBRATED].power = 2.4f;
                    break;
                case MAG_CHIP_AKM09915:
                    bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD].resolution = 0.15;
                    bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD].power = 3.0f;
                    bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD_UNCALIBRATED].resolution = 0.15;
                    bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD_UNCALIBRATED].power = 3.0f;
                    bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_FIELD].resolution = 0.15;
                    bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_FIELD].power = 3.0f;
                    bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_FIELD_UNCALIBRATED].resolution = 0.15;
                    bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_FIELD_UNCALIBRATED].power = 3.0f;
                    break;
                case MAG_CHIP_AKM09916:
                    bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD].resolution = 0.15;
                    bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD].power = 1.5f;
                    bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD_UNCALIBRATED].resolution = 0.15;
                    bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD_UNCALIBRATED].power = 1.5f;
                    bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_FIELD].resolution = 0.15;
                    bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_FIELD].power = 1.5f;
                    bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_FIELD_UNCALIBRATED].resolution = 0.15;
                    bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_FIELD_UNCALIBRATED].power = 1.5f;
                    break;
                default:
                    break;
            }
        }

        if(MAG_CHIP_YAS537 == magn_chip || MAG_CHIP_YAS532 == magn_chip)
        {
            bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD].name = "YAS Magnetic Field Sensor";
            bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD].vendor = "YAS";
            bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD_UNCALIBRATED].name = "YAS Magnetic Field Uncalibrated Sensor";
            bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD_UNCALIBRATED].vendor = "YAS";
            bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_FIELD].name = "YAS Magnetic Field (Wakeup) Sensor";
            bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_FIELD].vendor = "YAS";
            bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_FIELD_UNCALIBRATED].name = "YAS Magnetic Field Uncalibrated (Wakeup) Sensor";
            bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_FIELD_UNCALIBRATED].vendor = "YAS";

            switch (magn_chip) {
                case MAG_CHIP_YAS537:
                    bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD].maxRange = 2000.0f;
                    bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD].resolution = 0.3f;
                    bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD].power = 1.8f;
                    bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD_UNCALIBRATED].maxRange = 2000.0f;
                    bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD_UNCALIBRATED].resolution = 0.3f;
                    bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD_UNCALIBRATED].power = 1.8f;
                    bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_FIELD].maxRange = 2000.0f;
                    bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_FIELD].resolution = 0.3f;
                    bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_FIELD].power = 1.8f;
                    bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_FIELD_UNCALIBRATED].maxRange = 2000.0f;
                    bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_FIELD_UNCALIBRATED].resolution = 0.3f;
                    bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_FIELD_UNCALIBRATED].power = 1.8f;
                    break;
                case MAG_CHIP_YAS532:
                    bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD].maxRange = 1200.0f;
                    bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD].resolution = 0.15f;
                    bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD].power = 2.6f;
                    bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD_UNCALIBRATED].maxRange = 1200.0f;
                    bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD_UNCALIBRATED].resolution = 0.15f;
                    bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD_UNCALIBRATED].power = 2.6f;
                    bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_FIELD].maxRange = 1200.0f;
                    bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_FIELD].resolution = 0.15f;
                    bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_FIELD].power = 2.6f;
                    bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_FIELD_UNCALIBRATED].maxRange = 1200.0f;
                    bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_FIELD_UNCALIBRATED].resolution = 0.15f;
                    bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_FIELD_UNCALIBRATED].power = 2.6f;
                    break;
                default:
                    break;
            }
        }

        if(MAG_CHIP_MMC == magn_chip)
        {
            bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD].name = "MMC Magnetic Field Sensor";
            bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD].vendor = "MMC";
            bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD].maxRange = 3000.0f;
            bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD].resolution = 0.097656f;
            bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD].power = 1.8f;
            bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD_UNCALIBRATED].name = "MMC Magnetic Field Uncalibrated Sensor";
            bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD_UNCALIBRATED].vendor = "MMC";
            bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD_UNCALIBRATED].maxRange = 3000.0f;
            bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD_UNCALIBRATED].resolution = 0.097656f;
            bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD_UNCALIBRATED].power = 1.8f;
            bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_FIELD].name = "MMC Magnetic Field (Wakeup) Sensor";
            bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_FIELD].vendor = "MMC";
            bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_FIELD].maxRange = 3000.0f;
            bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_FIELD].resolution = 0.097656f;
            bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_FIELD].power = 1.8f;
            bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_FIELD_UNCALIBRATED].name = "MMC Magnetic Field Uncalibrated (Wakeup) Sensor";
            bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_FIELD_UNCALIBRATED].vendor = "MMC";
            bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_FIELD_UNCALIBRATED].maxRange = 3000.0f;
            bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_FIELD_UNCALIBRATED].resolution = 0.097656f;
            bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_FIELD_UNCALIBRATED].power = 1.8f;
        }

        /*supports Mag relevant sensors run at 3.125 - 50Hz*/
        bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD].minDelay = 20000;
        bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD].maxDelay = 320000;
        bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD_UNCALIBRATED].minDelay = 20000;
        bst_all_sensors[SENSORLIST_INX_MAGNETIC_FIELD_UNCALIBRATED].maxDelay = 320000;
        bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_FIELD].minDelay = 20000;
        bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_FIELD].maxDelay = 320000;
        bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_FIELD_UNCALIBRATED].minDelay = 20000;
        bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_FIELD_UNCALIBRATED].maxDelay = 320000;
        bst_all_sensors[SENSORLIST_INX_MAGNETIC_ROTATION_VECTOR].minDelay = 20000;
        bst_all_sensors[SENSORLIST_INX_MAGNETIC_ROTATION_VECTOR].maxDelay = 320000;
        bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_ROTATION_VECTOR].minDelay = 20000;
        bst_all_sensors[SENSORLIST_INX_WAKEUP_MAGNETIC_ROTATION_VECTOR].maxDelay = 320000;

        {
            avail_sens_regval = 0x14002;
        }

        sensor_amount = sensord_popcount_64(avail_sens_regval);

        bst_sensorlist.list = (struct sensor_t *) malloc(sensor_amount * sizeof(struct sensor_t));
        if (NULL == bst_sensorlist.list)
        {

            PERR("fail to malloc %d * sizeof(struct sensor_t)(=%d)",
                    sensor_amount, sizeof(struct sensor_t));
            return 0;
        }

        bst_sensorlist.bsx_list_index = (int32_t *) malloc(sensor_amount * sizeof(int32_t));
        if (NULL == bst_sensorlist.bsx_list_index)
        {

            PERR("fail to malloc %d * 4", sensor_amount);
            free(bst_sensorlist.list);
            return 0;
        }

        for (i = 0, j = 0; i < SENSORLIST_INX_END; i++, avail_sens_regval >>= 1)
        {
            if (0x0 == avail_sens_regval)
            {
                break;
            }

            if (avail_sens_regval & 0x1)
            {
                memcpy(&(bst_sensorlist.list[j]), &(bst_all_sensors[i]), sizeof(struct sensor_t));
                bst_sensorlist.bsx_list_index[j] = i;
                j++;
            }
        }

        bst_sensorlist.list_len = sensor_amount;
    }

    *p_sSensorList = bst_sensorlist.list;
    return bst_sensorlist.list_len;
}

static inline float BMI160_convert_ODR(int32_t bsx_input_id, float Hz, int32_t *odrCode)
{
    if (BSX_INPUT_ID_ACCELERATION == bsx_input_id)
    {
        if (Hz > 800)
        {
            *odrCode = BMI160_ACCEL_ODR_1600HZ;
            return 1600;
        }
        if (Hz > 400 && Hz <= 800)
        {
            *odrCode = BMI160_ACCEL_ODR_800HZ;
            return 800;
        }
        if (Hz > 200 && Hz <= 400)
        {
            *odrCode = BMI160_ACCEL_ODR_400HZ;
            return 400;
        }
        if (Hz > 100 && Hz <= 200)
        {
            *odrCode = BMI160_ACCEL_ODR_200HZ;
            return 200;
        }
        if (Hz > 50 && Hz <= 100)
        {
            *odrCode = BMI160_ACCEL_ODR_100HZ;
            return 100;
        }
        if (Hz > 25 && Hz <= 50)
        {
            *odrCode = BMI160_ACCEL_ODR_50HZ;
            return 50;
        }
        if (Hz > 12.5 && Hz <= 25)
        {
            *odrCode = BMI160_ACCEL_ODR_25HZ;
            return 25;
        }
        if (Hz > 6.25 && Hz <= 12.5)
        {
            *odrCode = BMI160_ACCEL_ODR_12_5HZ;
            return 12.5;
        }
        if (Hz > 3.125 && Hz <= 6.25)
        {
            *odrCode = BMI160_ACCEL_ODR_6_25HZ;
            return 6.25;
        }
        if (Hz > 1.5625 && Hz <= 3.125)
        {
            *odrCode = BMI160_ACCEL_ODR_3_12HZ;
            return 3.125;
        }
        if (Hz > 0.78125 && Hz <= 1.5625)
        {
            *odrCode = BMI160_ACCEL_ODR_1_56HZ;
            return 1.5625;
        }
        if (Hz > 0 && Hz <= 0.78125)
        {
            *odrCode = BMI160_ACCEL_ODR_0_78HZ;
            return 0.78125;
        }

        return BMI160_ACCEL_ODR_RESERVED;
    }
    else if (BSX_INPUT_ID_ANGULARRATE == bsx_input_id)
    {
        if (Hz > 1600)
        {
            *odrCode = BMI160_GYRO_ODR_3200HZ;
            return 3200;
        }
        if (Hz > 800 && Hz <= 1600)
        {
            *odrCode = BMI160_GYRO_ODR_1600HZ;
            return 1600;
        }
        if (Hz > 400 && Hz <= 800)
        {
            *odrCode = BMI160_GYRO_ODR_800HZ;
            return 800;
        }
        if (Hz > 200 && Hz <= 400)
        {
            *odrCode = BMI160_GYRO_ODR_400HZ;
            return 400;
        }
        if (Hz > 100 && Hz <= 200)
        {
            *odrCode = BMI160_GYRO_ODR_200HZ;
            return 200;
        }
        if (Hz > 50 && Hz <= 100)
        {
            *odrCode = BMI160_GYRO_ODR_100HZ;
            return 100;
        }
        if (Hz > 25 && Hz <= 50)
        {
            *odrCode = BMI160_GYRO_ODR_50HZ;
            return 50;
        }
        if (Hz > 0 && Hz <= 25)
        {
            *odrCode = BMI160_GYRO_ODR_25HZ;
            return 25;
        }

        return BMI160_GYRO_ODR_RESERVED;
    }
    else if (BSX_INPUT_ID_MAGNETICFIELD == bsx_input_id)
    {
        if (Hz > 200)
        {
            *odrCode = BMI160_MAG_ODR_400HZ;
            return 400;
        }
        if (Hz > 100 && Hz <= 200)
        {
            *odrCode = BMI160_MAG_ODR_200HZ;
            return 200;
        }
        if (Hz > 50 && Hz <= 100)
        {
            *odrCode = BMI160_MAG_ODR_100HZ;
            return 100;
        }
        if (Hz > 25 && Hz <= 50)
        {
            *odrCode = BMI160_MAG_ODR_50HZ;
            return 50;
        }
        if (Hz > 12.5 && Hz <= 25)
        {
            *odrCode = BMI160_MAG_ODR_25HZ;
            return 25;
        }
        if (Hz > 6.25 && Hz <= 12.5)
        {
            *odrCode = BMI160_MAG_ODR_12_5HZ;
            return 12.5;
        }
        if (Hz > 3.125 && Hz <= 6.25)
        {
            *odrCode = BMI160_MAG_ODR_6_25HZ;
            return 6.25;
        }
        if (Hz > 1.5625 && Hz <= 3.125)
        {
            *odrCode = BMI160_MAG_ODR_3_12HZ;
            return 3.125;
        }
        if (Hz > 0.78125 && Hz <= 1.5625)
        {
            *odrCode = BMI160_MAG_ODR_1_56HZ;
            return 1.5625;
        }
        if (Hz > 0 && Hz <= 0.78125)
        {
            *odrCode = BMI160_MAG_ODR_0_78HZ;
            return 0.78125;
        }

        return BMI160_MAG_ODR_RESERVED;
    }

    return 0;
}

/**
 * @param Hz
 * @param p_bandwith
 * @return physically support Hz
 */
static inline float BMA2x2_convert_ODR(float Hz, int32_t *p_bandwith)
{
    if (Hz > 800)
    {
        *p_bandwith = BMA2X2_ODR_2000HZ;
        return 2000.f;
    }
    if (Hz > 400 && Hz <= 800)
    {
        *p_bandwith = BMA2X2_ODR_1000HZ;
        return 1000.f;
    }
    if (Hz > 200 && Hz <= 400)
    {
        *p_bandwith = BMA2X2_ODR_500HZ;
        return 500.f;
    }
    if (Hz > 100 && Hz <= 200)
    {
        *p_bandwith = BMA2X2_ODR_250HZ;
        return 250.f;
    }
    if (Hz > 50 && Hz <= 100)
    {
        *p_bandwith = BMA2X2_ODR_125HZ;
        return 125.f;
    }
    if (Hz > 25 && Hz <= 50)
    {
        *p_bandwith = BMA2X2_ODR_62_50HZ;
        return 62.5f;
    }
    if (Hz > 12.5 && Hz <= 25)
    {
        *p_bandwith = BMA2X2_ODR_31_25HZ;
        return 31.25f;
    }
    if (Hz > 6.25 && Hz <= 12.5)
    {
        *p_bandwith = BMA2X2_ODR_15_63HZ;
        return 15.63f;
    }
    if (Hz > 1 && Hz <= 6.25)
    {
        *p_bandwith = BMA2X2_ODR_15_63HZ;
        return 15.63f;
    }

    return 0;

}

/**
 * @param Hz
 * @param p_bandwith
 * @return physical support Hz
 */
static inline float BMG160_convert_ODR(float Hz, int32_t *p_bandwith)
{
    if (Hz > 1000)
    {
        *p_bandwith = BMG160_ODR_2000HZ;
        return 2000.f;
    }
    if (Hz > 400 && Hz <= 1000)
    {
        *p_bandwith = BMG160_ODR_1000HZ;
        return 1000.f;
    }
    if (Hz > 200 && Hz <= 400)
    {
        *p_bandwith = BMG160_ODR_400HZ;
        return 400.f;
    }
    if (Hz > 100 && Hz <= 200)
    {
        *p_bandwith = BMG160_ODR_200HZ;
        return 200.f;
    }
    if (Hz > 1 && Hz <= 100)
    {
        *p_bandwith = BMG160_ODR_100HZ;
        return 100.f;
    }

    return 0;

}

#if !defined(UNIT_TEST_ACTIVE)
static int32_t is_acc_open = 0;
static int32_t is_gyr_open = 0;
static int32_t is_mag_open = 0;
static int32_t is_hwstc_open = 0;
static float bma_odr_set = 0;
static float bmg_odr_set = 0;
static float mag_odr_set = 0;

static int poll_timer_fd = -1;
static int acc_poll_mulp = 0;
static int gyr_poll_mulp = 0;
static int mag_poll_mulp = 0;
static int hwstc_poll_mulp = 0;
static int acc_poll_cnt = 0;
static int gyr_poll_cnt = 0;
static int mag_poll_cnt = 0;
static int hwstc_poll_cnt = 0;
#define MIN_TIMER_INTERVAL_ms 20
#define SET_POLL_PERIOD(period_ms, poll_mulp, poll_cnt) \
{\
    poll_mulp = period_ms / MIN_TIMER_INTERVAL_ms;\
    poll_cnt = 1;\
}

static int32_t bma_if_start_up = 0;
static float bma_odr_tmp = 0;
#define BMA_START_RESET(odr_set) \
{\
    bma_if_start_up = 0;\
    bma_odr_tmp = odr_set;\
}

static int32_t bmg_if_start_up = 0;
static float bmg_odr_tmp = 0;
#define BMG_START_RESET(odr_set) \
{\
    bmg_if_start_up = 0;\
    bmg_odr_tmp = odr_set;\
}

static int32_t mag_if_start_up = 0;
static float mag_odr_tmp = 0;
#define MAG_START_RESET(odr_set) \
{\
    mag_if_start_up = 0;\
    mag_odr_tmp = odr_set;\
}

static int hw_event_fd = -1;
static int32_t stepc_internal = 0;

/**depends on coordination of is_acc_open, is_mag_open, is_gyr_open.
 * Please note that this function should be called before assignment to is_acc_open, is_mag_open, is_gyr_open*/
static int32_t ap_activate_poll_timer(int32_t if_enable)
{
    struct itimerspec timerspec;

    if(if_enable){

        if(0 == is_acc_open + is_mag_open + is_gyr_open ){
            timerspec.it_value.tv_sec = 0;
            timerspec.it_value.tv_nsec = MIN_TIMER_INTERVAL_ms * 1000000;
            timerspec.it_interval.tv_sec = timerspec.it_value.tv_sec;
            timerspec.it_interval.tv_nsec = timerspec.it_value.tv_nsec;

            return timerfd_settime(poll_timer_fd, 0, &timerspec, NULL);
        }

    }else{

        if(1 == is_acc_open + is_mag_open + is_gyr_open ){
            timerspec.it_value.tv_sec = 0;
            timerspec.it_value.tv_nsec = 0;
            timerspec.it_interval.tv_sec = timerspec.it_value.tv_sec;
            timerspec.it_interval.tv_nsec = timerspec.it_value.tv_nsec;

            return timerfd_settime(poll_timer_fd, 0, &timerspec, NULL);
        }
    }

    return 0;
}

static void ap_config_phyACC(bsx_f32_t sample_rate)
{
    int32_t ret = 0;
    int32_t odr_code = 0;
    int32_t bandwidth = 0;
    int32_t fifo_data_sel_regval;
    float physical_Hz = 0;
    uint32_t period_ms = 0;
    const int32_t bma2x2_expected_frms = 5;


    PINFO("set physical ACC rate %f", sample_rate);

    if(ACC_CHIP_BMI160 == accl_chip)
    {
        ret = rd_sysfs_oneint("fifo_data_sel", bmi_input_dir_name, &fifo_data_sel_regval);
        if (0 != ret)
        {
            PERR("read fifo_data_sel fail, ret = %d, set fifo_data_sel_regval = 0", ret);
            /*Keep on trying*/
            fifo_data_sel_regval = 0;
        }

        if (SAMPLE_RATE_DISABLED == sample_rate)
        {
            if (1 == is_acc_open)
            {
                PDEBUG("shutdown acc");

                ap_activate_poll_timer(0);
                /** when closing in BMI160, set op mode firstly. */
                ret = wr_sysfs_oneint("acc_op_mode", bmi_input_dir_name, SENSOR_PM_SUSPEND);
                fifo_data_sel_regval &= ~(1 << 0);
                wr_sysfs_oneint("fifo_data_sel", bmi_input_dir_name, fifo_data_sel_regval);

                is_acc_open = 0;
            }
        }else
        {
            PDEBUG("set acc odr: %f", sample_rate);
            physical_Hz = BMI160_convert_ODR(BSX_INPUT_ID_ACCELERATION, sample_rate, &odr_code);
            ret = wr_sysfs_oneint("acc_odr", bmi_input_dir_name, odr_code);
            BMA_START_RESET(physical_Hz);

            /*activate is included*/
            if (0 == is_acc_open)
            {
                /** when opening in BMI160, set op mode at last. */
                fifo_data_sel_regval |= (1 << 0);
                wr_sysfs_oneint("fifo_data_sel", bmi_input_dir_name, fifo_data_sel_regval);
                ret = wr_sysfs_oneint("acc_op_mode", bmi_input_dir_name, SENSOR_PM_NORMAL);
                ap_activate_poll_timer(1);

                is_acc_open = 1;
            }
        }
    }else if(ACC_CHIP_BMA2x2 == accl_chip)
    {
        if (SAMPLE_RATE_DISABLED == sample_rate)
        {
            if (1 == is_acc_open)
            {
                PDEBUG("shutdown acc");

                ap_activate_poll_timer(0);
                ret = wr_sysfs_oneint("fifo_mode", acc_input_dir_name, BMA2x2_FIFO_PASSBY);
                ret = wr_sysfs_oneint("op_mode", acc_input_dir_name, SENSOR_PM_SUSPEND);

                is_acc_open = 0;
            }
        }else
        {
            PDEBUG("set acc odr: %f", sample_rate);

            physical_Hz = BMA2x2_convert_ODR(sample_rate, &bandwidth);
            ret = wr_sysfs_oneint("bandwidth", acc_input_dir_name, bandwidth);
            BMA_START_RESET(physical_Hz);
            period_ms = (uint32_t)(bma2x2_expected_frms * (1000.0f / physical_Hz));
            SET_POLL_PERIOD(period_ms, acc_poll_mulp, acc_poll_cnt);

            /*activate is included*/
            if (0 == is_acc_open)
            {
                ret = wr_sysfs_oneint("op_mode", acc_input_dir_name, SENSOR_PM_NORMAL);
                ret = wr_sysfs_oneint("fifo_mode", acc_input_dir_name, BMA2x2_FIFO_STREAM);
                ap_activate_poll_timer(1);

                is_acc_open = 1;
            }
        }
    }

    return;
}

static void ap_config_phyGYR(bsx_f32_t sample_rate)
{
    int32_t ret = 0;
    int32_t odr_code = 0;
    int32_t bandwidth = 0;
    int32_t fifo_data_sel_regval;
    float physical_Hz = 0;
    uint32_t period_ms = 0;
    const int32_t bmg160_expected_frms = 4;

    PINFO("set physical GYRO rate %f", sample_rate);

    if(GYR_CHIP_BMI160 == gyro_chip)
    {
        ret = rd_sysfs_oneint("fifo_data_sel", bmi_input_dir_name, &fifo_data_sel_regval);
        if (0 != ret)
        {
            PERR("read fifo_data_sel fail, ret = %d, set fifo_data_sel_regval = 0", ret);
            /*Keep on trying*/
            fifo_data_sel_regval = 0;
        }

        if (SAMPLE_RATE_DISABLED == sample_rate)
        {
            if (1 == is_gyr_open)
            {
                PDEBUG("shutdown gyro");

                ap_activate_poll_timer(0);
                /** when closing in BMI160, set op mode firstly. */
                ret = wr_sysfs_oneint("gyro_op_mode", bmi_input_dir_name, SENSOR_PM_SUSPEND);
                fifo_data_sel_regval &= ~(1 << 1);
                wr_sysfs_oneint("fifo_data_sel", bmi_input_dir_name, fifo_data_sel_regval);

                is_gyr_open = 0;
            }
        }else
        {
            PDEBUG("set gyr odr: %f", sample_rate);
            physical_Hz = BMI160_convert_ODR(BSX_INPUT_ID_ANGULARRATE, sample_rate, &odr_code);
            ret = wr_sysfs_oneint("gyro_odr", bmi_input_dir_name, odr_code);
            BMG_START_RESET(physical_Hz);

            /*activate is included*/
            if (0 == is_gyr_open)
            {
                /** when opening in BMI160, set op mode at last. */
                fifo_data_sel_regval |= (1 << 1);
                wr_sysfs_oneint("fifo_data_sel", bmi_input_dir_name, fifo_data_sel_regval);
                ret = wr_sysfs_oneint("gyro_op_mode", bmi_input_dir_name, SENSOR_PM_NORMAL);
                ap_activate_poll_timer(1);

                is_gyr_open = 1;
            }
        }
    }else if(GYR_CHIP_BMG160 == gyro_chip)
    {
        if (SAMPLE_RATE_DISABLED == sample_rate)
        {
            if (1 == is_gyr_open)
            {
                PDEBUG("shutdown gyro");

                ap_activate_poll_timer(0);
                ret = wr_sysfs_oneint("fifo_mode", gyr_input_dir_name, BMG160_FIFO_PASSBY);
                ret = wr_sysfs_oneint("op_mode", gyr_input_dir_name, SENSOR_PM_SUSPEND);

                is_gyr_open = 0;
            }
        }else
        {
            PDEBUG("set gyr odr: %f", sample_rate);

            physical_Hz = BMG160_convert_ODR(sample_rate, &bandwidth);
            ret = wr_sysfs_oneint("bandwidth", gyr_input_dir_name, bandwidth);
            BMG_START_RESET(physical_Hz);
            period_ms = (uint32_t)(bmg160_expected_frms * (1000.0f / physical_Hz));
            SET_POLL_PERIOD(period_ms, gyr_poll_mulp, gyr_poll_cnt);

            /*activate is included*/
            if (0 == is_gyr_open)
            {
                ret = wr_sysfs_oneint("op_mode", gyr_input_dir_name, SENSOR_PM_NORMAL);
                ret = wr_sysfs_oneint("fifo_mode", gyr_input_dir_name, BMG160_FIFO_STREAM);
                ap_activate_poll_timer(1);

                is_gyr_open = 1;
            }
        }
    }

    return;
}

static void ap_config_phyMAG(bsx_f32_t sample_rate)
{
    int32_t ret = 0;
    int32_t odr_code = 0;
    int32_t fifo_data_sel_regval;
    uint32_t Hz_to_delay_ms = 0;
    float physical_Hz = 0;

    PINFO("set physical MAG rate %f", sample_rate);

    if(MAG_CHIP_BMI160 == magn_chip)
    {
        ret = rd_sysfs_oneint("fifo_data_sel", bmi_input_dir_name, &fifo_data_sel_regval);
        if (0 != ret)
        {
            PERR("read fifo_data_sel fail, ret = %d, set fifo_data_sel_regval = 0", ret);
            /*Keep on trying*/
            fifo_data_sel_regval = 0;
        }

        if (SAMPLE_RATE_DISABLED == sample_rate)
        {
            if (1 == is_mag_open)
            {
                PDEBUG("shutdown magn");

                ap_activate_poll_timer(0);
                /** when closing in BMI160, set op mode firstly. */
                ret = wr_sysfs_oneint("mag_op_mode", bmi_input_dir_name, SENSOR_PM_SUSPEND);
                fifo_data_sel_regval &= ~(1 << 2);
                wr_sysfs_oneint("fifo_data_sel", bmi_input_dir_name, fifo_data_sel_regval);

                is_mag_open = 0;
            }
        }else
        {
            PDEBUG("set magn odr: %f", sample_rate);
            physical_Hz = BMI160_convert_ODR(BSX_INPUT_ID_MAGNETICFIELD, sample_rate, &odr_code);
            ret = wr_sysfs_oneint("mag_odr", bmi_input_dir_name, odr_code);
            MAG_START_RESET(physical_Hz);

            /*activate is included*/
            if (0 == is_mag_open)
            {
                /** when opening in BMI160, set op mode at last. */
                fifo_data_sel_regval |= (1 << 2);
                wr_sysfs_oneint("fifo_data_sel", bmi_input_dir_name, fifo_data_sel_regval);
                ret = wr_sysfs_oneint("mag_op_mode", bmi_input_dir_name, SENSOR_PM_NORMAL);
                ap_activate_poll_timer(1);

                is_mag_open = 1;
            }
        }
    }else if(MAG_CHIP_BMM150 == magn_chip ||
            MAG_CHIP_AKM09912 ==  magn_chip || MAG_CHIP_AKM09911 ==  magn_chip ||
            MAG_CHIP_AKM09915 ==  magn_chip || MAG_CHIP_AKM09916 ==  magn_chip ||
            MAG_CHIP_YAS537 ==  magn_chip || MAG_CHIP_YAS532 ==  magn_chip || MAG_CHIP_MMC == magn_chip)
    {
        if (SAMPLE_RATE_DISABLED == sample_rate)
        {
            if (1 == is_mag_open)
            {
                PDEBUG("shutdown magn");

                ap_activate_poll_timer(0);
                ret = wr_sysfs_oneint("op_mode", mag_input_dir_name, SENSOR_PM_SUSPEND);

                is_mag_open = 0;
            }
        }else
        {
            PDEBUG("set magn odr: %f", sample_rate);

            Hz_to_delay_ms = (uint32_t)(1000.0f / sample_rate);
            SET_POLL_PERIOD(Hz_to_delay_ms, mag_poll_mulp, mag_poll_cnt);
            MAG_START_RESET(sample_rate);

            /*activate is included*/
            if (0 == is_mag_open)
            {
                ret = wr_sysfs_oneint("op_mode", mag_input_dir_name, SENSOR_PM_NORMAL);

                if(MAG_CHIP_BMM150 == magn_chip){
                    /*set filter parameter to reduce noise.
                     * only can do on our BMM but open this to customer on AKM*/
                    ret = wr_sysfs_oneint("rept_xy", mag_input_dir_name, 4);
                    ret = wr_sysfs_oneint("rept_z", mag_input_dir_name, 15);
                }
                ap_activate_poll_timer(1);

                is_mag_open = 1;
            }
        }
    }

    return;
}

#endif

static void ap_config_physensor(bsx_u32_t input_id, bsx_f32_t sample_rate)
{
#if !defined(UNIT_TEST_ACTIVE)
    int32_t ret = 0;

    if (BSX_PHYSICAL_SENSOR_ID_INVALID == input_id)
    { //TODO: library may has bug
        return;
    }

    switch (input_id)
    {
        case BSX_INPUT_ID_ACCELERATION:
            ap_config_phyACC(sample_rate);
            break;
        case BSX_INPUT_ID_MAGNETICFIELD:
            ap_config_phyMAG(sample_rate);
            break;
        case BSX_INPUT_ID_ANGULARRATE:
            ap_config_phyGYR(sample_rate);
            break;
        default:
            PWARN("unknown input id: %d", input_id);
            ret = 0;
            break;
    }

    if (ret < 0)
    {
        PERR("write_sysfs() fail");
    }
#endif

    return;
}


/**
 *
 */
static void ap_send_config(int32_t bsx_list_inx)
{
    BSX_SENSOR_CONFIG *p_config;
    int32_t list_inx_base;
    int32_t ret = 0;
    uint32_t Hz_to_delay_ms = 0;
    bsx_u32_t input_id;

    if (bsx_list_inx <= SENSORLIST_INX_AMBIENT_IAQ)
    {
        list_inx_base = SENSORLIST_INX_GAS_RESIST;
        p_config = BSX_sensor_config_nonwk;
    }
    else
    {
        list_inx_base = SENSORLIST_INX_WAKEUP_SIGNI_PRESSURE;
        p_config = BSX_sensor_config_wk;
    }

#if !defined(UNIT_TEST_ACTIVE)
    if(1 == hardware_motion && ACC_CHIP_BMI160 == accl_chip)
    {
        switch (bsx_list_inx) {
            case SENSORLIST_INX_WAKEUP_SIGNIFICANT_MOTION:
                PDEBUG("enable hardware SIGNIFICANT_MOTION");
                ret += wr_sysfs_oneint("sig_en", bmi_input_dir_name, 1);
                return;
            case SENSORLIST_INX_WAKEUP_STEP_DETECTOR:
                PDEBUG("enable hardware STEP_DETECTOR");
                ret += wr_sysfs_oneint("std_en", bmi_input_dir_name, 1);
                return;
            case SENSORLIST_INX_WAKEUP_STEP_COUNTER:
                PDEBUG("enable hardware STEP_COUNTER, rate = %f", p_config[bsx_list_inx - list_inx_base].rate_Hz);
                Hz_to_delay_ms = (uint32_t)(1000.0f / p_config[bsx_list_inx - list_inx_base].rate_Hz);
                SET_POLL_PERIOD(Hz_to_delay_ms, hwstc_poll_mulp, hwstc_poll_cnt);
                if(0 == is_hwstc_open)
                {
                    ret += wr_sysfs_oneint("stc_enable", bmi_input_dir_name, 1);
                    is_hwstc_open  = 1;
                }
                return;
            default:
                break;
        }
    }
#endif

    {
        switch (bsx_list_inx) {
            case SENSORLIST_INX_ACCELEROMETER:
                input_id = BSX_INPUT_ID_ACCELERATION;
                break;
            case SENSORLIST_INX_MAGNETIC_FIELD_UNCALIBRATED:
                input_id = BSX_INPUT_ID_MAGNETICFIELD;
                break;
            case SENSORLIST_INX_GYROSCOPE_UNCALIBRATED:
                input_id = BSX_INPUT_ID_ANGULARRATE;
                break;
            default:
                PERR("wrong list index: %d", bsx_list_inx);
                return;
        }

        ap_config_physensor(input_id, p_config[bsx_list_inx - list_inx_base].rate_Hz);
    }

    return;
}

/**
 *
 * @param bsx_list_inx
 */
static void ap_send_disable_config(int32_t bsx_list_inx)
{
    int32_t ret = 0;
    bsx_u32_t input_id;

#if !defined(UNIT_TEST_ACTIVE)
    if(1 == hardware_motion && ACC_CHIP_BMI160 == accl_chip)
    {
        switch (bsx_list_inx) {
            case SENSORLIST_INX_WAKEUP_SIGNIFICANT_MOTION:
                PDEBUG("close hardware SIGNIFICANT_MOTION");
                ret += wr_sysfs_oneint("sig_en", bmi_input_dir_name, 0);
                return;
            case SENSORLIST_INX_WAKEUP_STEP_DETECTOR:
                PDEBUG("close hardware STEP_DETECTOR");
                ret += wr_sysfs_oneint("std_en", bmi_input_dir_name, 0);
                return;
            case SENSORLIST_INX_WAKEUP_STEP_COUNTER:
                if(1 == is_hwstc_open)
                {
                    PDEBUG("close hardware STEP_COUNTER");
                    stepc_internal = 0;
                    ret += wr_sysfs_oneint("stc_enable", bmi_input_dir_name, 0);
                    is_hwstc_open  = 0;
                }
                return;
            default:
                break;
        }
    }
#endif

    {
        switch (bsx_list_inx) {
            case SENSORLIST_INX_ACCELEROMETER:
                input_id = BSX_INPUT_ID_ACCELERATION;
                break;
            case SENSORLIST_INX_MAGNETIC_FIELD_UNCALIBRATED:
                input_id = BSX_INPUT_ID_MAGNETICFIELD;
                break;
            case SENSORLIST_INX_GYROSCOPE_UNCALIBRATED:
                input_id = BSX_INPUT_ID_ANGULARRATE;
                break;
            default:
                PERR("wrong list index: %d", bsx_list_inx);
                return;
        }

        ap_config_physensor(input_id, SAMPLE_RATE_DISABLED);
    }
    return;
}

int32_t ap_activate(int32_t handle, int32_t enabled)
{
    struct sensor_t *p_sensor;
    int32_t bsx_list_inx;
    int32_t ret;

    if (BSX_SENSOR_ID_INVALID == handle)
    {
        /*private sensors which not supported in library and Android yet*/
        return 0;
    }

    PDEBUG("ap_activate(handle: %d, enabled: %d)", handle, enabled);

    get_sensor_t(handle, &p_sensor, &bsx_list_inx);
    if (NULL == p_sensor)
    {
        PWARN("invalid id: %d", handle);
        return -EINVAL;
    }

    /*To adapt BSX4 algorithm's way of configuration string, activate_configref_resort() is employed*/
    ret = activate_configref_resort(bsx_list_inx, enabled);
    if (ret)
    {
        if (enabled)
        {
            ap_send_config(bsx_list_inx);
        }
        else
        {
            ap_send_disable_config(bsx_list_inx);
        }
    }

    return 0;
}

int32_t ap_batch(int32_t handle, int32_t flags, int64_t sampling_period_ns, int64_t max_report_latency_ns)
{

    (void) flags; //Deprecated in SENSORS_DEVICE_API_VERSION_1_3
    struct sensor_t *p_sensor;
    int32_t bsx_list_inx;
    int32_t ret;

    //Sensors whoes fifoMaxEventCount==0 can be called batching according to spec
    get_sensor_t(handle, &p_sensor, &bsx_list_inx);
    if (NULL == p_sensor)
    {
        PWARN("invalid id: %d", handle);
        /*If maxReportingLatency is set to 0, this function must succeed*/
        if (0 == max_report_latency_ns)
        {
            return 0;
        }
        else
        {
            return -EINVAL;
        }
    }

    /*One-shot sensors are sometimes referred to as trigger sensors.
     The sampling_period_ns and max_report_latency_ns parameters
     passed to the batch function is ignored.
     Events from one-shot events cannot be stored in hardware FIFOs:
     the events must be reported as soon as they are generated.
     */
    if (SENSOR_FLAG_ONE_SHOT_MODE == (p_sensor->flags & REPORTING_MODE_MASK))
    {
        return 0;
    }

    /*For special reporting-mode sensors, process each one according to spec*/
    if (SENSOR_FLAG_SPECIAL_REPORTING_MODE == (p_sensor->flags & REPORTING_MODE_MASK))
    {
        //...
        return 0;
    }

    PDEBUG("batch(handle: %d, sampling_period_ns = %lld)", handle, sampling_period_ns);

    /*
     For continuous and on-change sensors
     1. if sampling_period_ns is less than sensor_t.minDelay,
     then the HAL implementation must silently clamp it to max(sensor_t.minDelay, 1ms).
     Android does not support the generation of events at more than 1000Hz.
     2. if sampling_period_ns is greater than sensor_t.maxDelay,
     then the HAL implementation must silently truncate it to sensor_t.maxDelay.
     */
    if (sampling_period_ns < (int64_t)(p_sensor->minDelay) * 1000LL)
    {
        sampling_period_ns = (int64_t)(p_sensor->minDelay) * 1000LL;
    }
    else if (sampling_period_ns > (int64_t)(p_sensor->maxDelay) * 1000LL)
    {
        sampling_period_ns = (int64_t)(p_sensor->maxDelay) * 1000LL;
    }

    /*In Android's perspective, a sensor can be configured no matter if it's active.
     To adapt BSX4 algorithm's way of configuration string, batch_configref_resort() is employed*/
    ret = batch_configref_resort(bsx_list_inx, sampling_period_ns, max_report_latency_ns);
    if (ret)
    {
        ap_send_config(bsx_list_inx);
    }

    return 0;
}

int32_t ap_flush(BstSensor *bstsensor, int32_t handle)
{
    struct sensor_t *p_sensor;
    int32_t bsx_list_inx;

    get_sensor_t(handle, &p_sensor, &bsx_list_inx);
    if (NULL == p_sensor)
    {
        PWARN("invalid id: %d", handle);
        return -EINVAL;
    }

    /*flush does not apply to one-shot sensors:
     if sensor_handle refers to a one-shot sensor,
     flush must return -EINVAL and not generate any flush complete metadata event.
     */
    if (SENSOR_FLAG_ONE_SHOT_MODE == (p_sensor->flags & REPORTING_MODE_MASK))
    {
        PWARN("invalid flags for id: %d", handle);
        return -EINVAL;
    }

    /*call driver interface, flush event is sent by driver when complete flushing.
     The flush happens asynchronously (i.e.: this function must return immediately)
     */
#if !defined(UNIT_TEST_ACTIVE)
    // TODO:
#endif

    //Now the driver can not support this specification, so has to work around
    (void) bstsensor->send_flush_event(handle);

    /*If the specified sensor has no FIFO (no buffering possible),
     or if the FIFO was empty at the time of the call,
     flush must still succeed and send a flush complete event for that sensor.
     This applies to all sensors
     (except one-shot sensors, and already filted in former code).
     */

    return 0;
}





#if !defined(UNIT_TEST_ACTIVE)
#define TM_DEVIATE_PERCENT 0.4

static void ap_hw_poll_bma2x2(BstSimpleList *dest_list)
{
    static char fname_buf[MAX_FILENAME_LEN+1] = {0};
    static int64_t bma_poll_start_tm = 0;
    static int64_t bma_last_frm_tm = 0;
    static uint32_t bma_total_frm = 0;
    static int32_t bma2x2_raw_cnt = 0;
    static int32_t bma2x2_pre_raw[3] = {0};
    static int64_t bma2x2_pre_tm = 0;
#define BMA2x2_RESET_RESAMPLE() \
{\
    bma2x2_raw_cnt = 0;\
    bma2x2_pre_raw[0] = 0;\
    bma2x2_pre_raw[1] = 0;\
    bma2x2_pre_raw[2] = 0;\
    bma2x2_pre_tm = 0;\
}
    int32_t ret;
    int32_t fifo_fd = -1;
    uint8_t fifo_data_buf[32*3*2] = {0}; //32 frames, each frame contains 2Bytes X, 2Bytes Y and 2Bytes Z
    HW_DATA_UNION *p_hwdata;
    int16_t X_tmp = 0;
    int16_t Y_tmp = 0;
    int16_t Z_tmp = 0;
    int64_t sys_tm = 0;
    int32_t frames = 0;
    int32_t i;
    int64_t average_interval;
    int8_t resample_valid = 0;
    int32_t to_resample_raw[3] = {0};
    int64_t to_resample_tm = 0;


    sys_tm = android::elapsedRealtimeNano();

    if(0 == fname_buf[0]){
        snprintf(fname_buf, MAX_FILENAME_LEN, "%s/%s", acc_input_dir_name, "fifo_data_frame");
    }

    fifo_fd = open(fname_buf, O_RDONLY | O_NONBLOCK);
    if(-1 == fifo_fd){
        PERR("can not open %s, errno = %d(%s)", fname_buf, errno, strerror(errno));
        return;
    }

    ret = read(fifo_fd, fifo_data_buf, sizeof(fifo_data_buf));
    close(fifo_fd);
    if(ret < 0 ){
        PERR("read %d fail: ret = %d, errno = %d(%s)", fifo_fd, ret, errno, strerror(errno));
        return;
    }

    frames = ret/6;/*2BytesX 2BytesY 2BytesZ*/
    if(0 == frames)
    {
        return;
    }

    /*there's a start up time needed for BMA2x2*/
    if(0 == bma_if_start_up)
    {
        /*update time stamp estimation base line*/
        bma_poll_start_tm = sys_tm;
        bma_last_frm_tm = bma_poll_start_tm;
        bma_total_frm = 0;
        bma_odr_set = bma_odr_tmp * 0.8; /*BMA2x2 is resamples as 5 to 4*/
        BMA2x2_RESET_RESAMPLE();

        bma_if_start_up = 1;
        return;
    }

    bma_total_frm += frames; //no matter deliver successfully or not
    average_interval = (sys_tm - bma_poll_start_tm) / bma_total_frm;

    for (i = 0; i < frames; ++i)
    {
        /*2 Bytes output from all BMA ASIC, valid bits are located high*/
        X_tmp = fifo_data_buf[i*6+0] | (fifo_data_buf[i*6+1] << 8);
        X_tmp >>= (16 - BMA255_ADC_BITS);
        Y_tmp = fifo_data_buf[i*6+2] | (fifo_data_buf[i*6+3] << 8);
        Y_tmp >>= (16 - BMA255_ADC_BITS);
        Z_tmp = fifo_data_buf[i*6+4] | (fifo_data_buf[i*6+5] << 8);
        Z_tmp >>= (16 - BMA255_ADC_BITS);

        p_hwdata = (HW_DATA_UNION *) calloc(1, sizeof(HW_DATA_UNION));
        if (NULL == p_hwdata)
        {
            PERR("malloc fail");
            continue;
        }

        p_hwdata->id = SENSOR_TYPE_ACCELEROMETER;
        p_hwdata->x = X_tmp * BMA255_acc_resl;
        p_hwdata->y = Y_tmp * BMA255_acc_resl;
        p_hwdata->z = Z_tmp * BMA255_acc_resl;
        p_hwdata->timestamp = bma_last_frm_tm + average_interval;
        bma_last_frm_tm += average_interval;

        if(p_hwdata->timestamp > sys_tm){
            p_hwdata->timestamp = sys_tm;
            bma_last_frm_tm = p_hwdata->timestamp;
        }

        /*resample and deliver*/
        to_resample_raw[0] = (int32_t)p_hwdata->x;
        to_resample_raw[1] = (int32_t)p_hwdata->y;
        to_resample_raw[2] = (int32_t)p_hwdata->z;
        to_resample_tm = p_hwdata->timestamp;

        resample_valid = sensord_resample5to4(to_resample_raw, &to_resample_tm,
                                                bma2x2_pre_raw, &bma2x2_pre_tm,
                                                bma2x2_raw_cnt++);
        if(resample_valid && 0 != to_resample_tm)
        {
            /*if(0 == to_resample_tm), it's the first raw data, omit*/
            p_hwdata->x = (float)to_resample_raw[0];
            p_hwdata->y = (float)to_resample_raw[1];
            p_hwdata->z = (float)to_resample_raw[2];
            p_hwdata->timestamp = to_resample_tm;

            hw_remap_sensor_data(&(p_hwdata->x), &(p_hwdata->y), &(p_hwdata->z), g_place_a);

            ret = dest_list->list_add_rear((void *) p_hwdata);
            if (ret)
            {
                PERR("list_add_rear() fail, ret = %d", ret);
                if(-1 == ret){
                    free(p_hwdata);
                }
            }
        }else
        {
            free(p_hwdata);
        }
    }

    return;
}

#define BMG160_ABNORMAL_CEILING 10
static void ap_hw_poll_bmg160(BstSimpleList *dest_list)
{
    int32_t ret;
    int32_t fifo_fd = -1;
    static char fname_buf[MAX_FILENAME_LEN+1] = {0};
    static int64_t bmg_poll_start_tm = 0;
    static int64_t bmg_last_frm_tm = 0;
    static uint32_t bmg_total_frm = 0;
    uint8_t fifo_data_buf[100*3*2] = {0}; //100 frames, each frame contains 2Bytes X, 2Bytes Y and 2Bytes Z
    HW_DATA_UNION *p_hwdata;
    int16_t X_tmp = 0;
    int16_t Y_tmp = 0;
    int16_t Z_tmp = 0;
    int64_t sys_tm = 0;
    int32_t frames = 0;
    int32_t i;
    int64_t average_interval;
    float deviate_percent = 1;

    sys_tm = android::elapsedRealtimeNano();

    if(0 == fname_buf[0]){
        snprintf(fname_buf, MAX_FILENAME_LEN, "%s/%s", gyr_input_dir_name, "fifo_data_frame");
    }

    fifo_fd = open(fname_buf, O_RDONLY | O_NONBLOCK);
    if(-1 == fifo_fd){
        PERR("can not open %s, errno = %d(%s)", fname_buf, errno, strerror(errno));
        return;
    }

    ret = read(fifo_fd, fifo_data_buf, sizeof(fifo_data_buf));
    close(fifo_fd);
    if(ret < 0 ){
        PERR("read %d fail: ret = %d, errno = %d(%s)", fifo_fd, ret, errno, strerror(errno));
        return;
    }

    frames = ret/6;/*2BytesX 2BytesY 2BytesZ*/
    if(0 == frames)
    {
        return;
    }

    /*there's a start up time needed for BMG160*/
    if(0 == bmg_if_start_up)
    {
        /*When changing op mode (opening), BMG160 produces several invalid frames in the
         * beginning. They have to be dropped otherwise Gyroscope sample's standard deviation
         * will be impacted a lot*/
        X_tmp = fifo_data_buf[0] | (fifo_data_buf[1] << 8);
        Y_tmp = fifo_data_buf[2] | (fifo_data_buf[3] << 8);
        Z_tmp = fifo_data_buf[4] | (fifo_data_buf[5] << 8);
        if(X_tmp < BMG160_ABNORMAL_CEILING && Y_tmp < BMG160_ABNORMAL_CEILING && Z_tmp < BMG160_ABNORMAL_CEILING)
        {
            /*update time stamp estimation base line*/
            bmg_poll_start_tm = sys_tm;
            bmg_last_frm_tm = bmg_poll_start_tm;
            bmg_total_frm = 0;
            bmg_odr_set = bmg_odr_tmp;

            bmg_if_start_up = 1;
        }

        return;
    }


    bmg_total_frm += frames; //no matter deliver successfully or not
    average_interval = (sys_tm - bmg_poll_start_tm) / bmg_total_frm;

    for (i = 0; i < frames; ++i)
    {
        X_tmp = fifo_data_buf[i*6+0] | (fifo_data_buf[i*6+1] << 8);
        Y_tmp = fifo_data_buf[i*6+2] | (fifo_data_buf[i*6+3] << 8);
        Z_tmp = fifo_data_buf[i*6+4] | (fifo_data_buf[i*6+5] << 8);

        p_hwdata = (HW_DATA_UNION *) calloc(1, sizeof(HW_DATA_UNION));
        if (NULL == p_hwdata)
        {
            PERR("malloc fail");
            continue;
        }

        p_hwdata->id = SENSOR_TYPE_GYROSCOPE_UNCALIBRATED;
        p_hwdata->x = X_tmp;
        p_hwdata->y = Y_tmp;
        p_hwdata->z = Z_tmp;
        p_hwdata->timestamp = bmg_last_frm_tm + average_interval;
        bmg_last_frm_tm += average_interval;

        if(p_hwdata->timestamp > sys_tm)
        {
            /*BMG160 generates more samples than required.*/
            deviate_percent = (float)(p_hwdata->timestamp - sys_tm) / (float)average_interval;
            if(deviate_percent < TM_DEVIATE_PERCENT)
            {
                p_hwdata->timestamp = sys_tm;
                hw_remap_sensor_data(&(p_hwdata->x), &(p_hwdata->y), &(p_hwdata->z), g_place_g);

                 ret = dest_list->list_add_rear((void *) p_hwdata);
                 if (ret)
                 {
                     PERR("list_add_rear() fail, ret = %d", ret);
                     if(-1 == ret){
                         free(p_hwdata);
                     }
                 }

                 bmg_last_frm_tm = sys_tm;

            }else{

                free(p_hwdata);
                bmg_last_frm_tm -= average_interval;
            }

            break;

        }else
        {
            hw_remap_sensor_data(&(p_hwdata->x), &(p_hwdata->y), &(p_hwdata->z), g_place_g);

            ret = dest_list->list_add_rear((void *) p_hwdata);
            if (ret)
            {
                PERR("list_add_rear() fail, ret = %d", ret);
                if(-1 == ret){
                    free(p_hwdata);
                }
            }
        }
    }

    return;
}

/**
 * @param cur_tm, the latest times tamp got from system API
 * @return = tm_base + (average interval of historical time stamps)
 * tm_base update in every calculation.
 */
static int64_t mag_poll_tmfilt(int64_t cur_tm)
{
    int64_t total_interval = 0;
    int i;
    static int64_t mag_polltm_hist_buf[5] = {0};
    static int32_t mag_polltm_hist_cnt = 0;
    static int64_t mag_poll_starttm = 0;
#define MAG_POLLTM_HIST_CLN() \
{\
    memset(mag_polltm_hist_buf, 0, sizeof(mag_polltm_hist_buf));\
    mag_polltm_hist_cnt = 0;\
    mag_poll_starttm = 0;\
}

    if(0 == mag_if_start_up)
    {
        MAG_POLLTM_HIST_CLN();
        mag_odr_set = mag_odr_tmp;
        mag_if_start_up = 1;
    }

    if( ARRAY_ELEMENTS(mag_polltm_hist_buf) != mag_polltm_hist_cnt)
    {
        mag_polltm_hist_buf[mag_polltm_hist_cnt] = cur_tm;
        mag_polltm_hist_cnt++;
    }else
    {
        memmove(mag_polltm_hist_buf, &mag_polltm_hist_buf[1], sizeof(mag_polltm_hist_buf[0]) * (ARRAY_ELEMENTS(mag_polltm_hist_buf)-1));
        mag_polltm_hist_buf[mag_polltm_hist_cnt - 1] = cur_tm;
    }

    if(1 == mag_polltm_hist_cnt)
    {
        mag_poll_starttm = cur_tm;

    }else
    {
        for (i = 1; i < mag_polltm_hist_cnt; ++i) {
            total_interval += mag_polltm_hist_buf[i] - mag_polltm_hist_buf[i-1];
        }

        mag_poll_starttm += total_interval/(mag_polltm_hist_cnt-1);
    }

    if(mag_poll_starttm > cur_tm){
        /*reduce the drift between estimation time and system time*/
        mag_poll_starttm = cur_tm;
    }

    return mag_poll_starttm;
}

static void ap_hw_poll_bmm150(BstSimpleList *dest_list)
{
    int32_t ret;
    FILE *value_fp = NULL;
    static char fname_buf[MAX_FILENAME_LEN+1] = {0};
    HW_DATA_UNION *p_hwdata;
    int32_t X_tmp = 0;
    int32_t Y_tmp = 0;
    int32_t Z_tmp = 0;
    int64_t sys_tm = 0;

    sys_tm = android::elapsedRealtimeNano();

    //force update mag sample, otherwise it update only in 10Hz
    ret = wr_sysfs_oneint("op_mode", mag_input_dir_name, SENSOR_PM_LP1);

    if(0 == fname_buf[0]){
        snprintf(fname_buf, MAX_FILENAME_LEN, "%s/%s", mag_input_dir_name, "value");
    }

    value_fp = fopen(fname_buf, "r");
    if (NULL == value_fp)
    {
        PERR("can not open %s, errno = %d(%s)", fname_buf, errno, strerror(errno));
        return;
    }

    ret = fscanf(value_fp, "%8d %8d %8d", &X_tmp, &Y_tmp, &Z_tmp);
    fclose(value_fp);
    if(ret <= 0){
        PERR("fscanf fail, errno = %d(%s)", errno, strerror(errno));
        return;
    }

    p_hwdata = (HW_DATA_UNION *) calloc(1, sizeof(HW_DATA_UNION));
    if (NULL == p_hwdata)
    {
        PERR("malloc fail");
        return;
    }

    p_hwdata->id = SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED;
    p_hwdata->x = X_tmp * 10 / BMM_COMPVAL_TO_uT; // library requires input is in 0.1uT
    p_hwdata->y = Y_tmp * 10 / BMM_COMPVAL_TO_uT;
    p_hwdata->z = Z_tmp * 10 / BMM_COMPVAL_TO_uT;
    p_hwdata->timestamp = mag_poll_tmfilt(sys_tm);

    hw_remap_sensor_data(&(p_hwdata->x), &(p_hwdata->y), &(p_hwdata->z), g_place_m);

    ret = dest_list->list_add_rear((void *) p_hwdata);
    if (ret)
    {
        PERR("list_add_rear() fail, ret = %d", ret);
        if(-1 == ret){
            free(p_hwdata);
        }
    }

    return;
}

static void ap_hw_poll_akm099xx(BstSimpleList *dest_list)
{
    int32_t ret;
    FILE *value_fp = NULL;
    static char fname_buf[MAX_FILENAME_LEN+1] = {0};
    HW_DATA_UNION *p_hwdata;
    int32_t X_tmp = 0;
    int32_t Y_tmp = 0;
    int32_t Z_tmp = 0;
    int32_t S1_tmp = 0;
    int32_t S2_tmp = 0;
    int64_t sys_tm = 0;

    if(0 == fname_buf[0]){
        snprintf(fname_buf, MAX_FILENAME_LEN, "%s/%s", mag_input_dir_name, "value");
    }

    value_fp = fopen(fname_buf, "r");
    if (NULL == value_fp)
    {
        PERR("can not open %s, errno = %d(%s)", fname_buf, errno, strerror(errno));
        return;
    }

    sys_tm = android::elapsedRealtimeNano();

    /*AKM need reset power mode to let it update data register*/
    ret = wr_sysfs_oneint("op_mode", mag_input_dir_name, SENSOR_PM_NORMAL);

    ret = fscanf(value_fp, "%8d %8d %8d %8d %8d", &X_tmp, &Y_tmp, &Z_tmp, &S1_tmp, &S2_tmp);
    fclose(value_fp);
    if(ret <= 0){
        PERR("fscanf fail, errno = %d(%s)", errno, strerror(errno));
        return;
    }

    p_hwdata = (HW_DATA_UNION *) calloc(1, sizeof(HW_DATA_UNION));
    if (NULL == p_hwdata)
    {
        PERR("malloc fail");
        return;
    }

    p_hwdata->id = SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED;
    switch (magn_chip) {
        case MAG_CHIP_AKM09912:
            p_hwdata->x = X_tmp * 10 * AKM09912_COMPVAL_TO_uT;
            p_hwdata->y = Y_tmp * 10 * AKM09912_COMPVAL_TO_uT;
            p_hwdata->z = Z_tmp * 10 * AKM09912_COMPVAL_TO_uT;
            break;
        case MAG_CHIP_AKM09911:
            p_hwdata->x = X_tmp * 10 * AKM09911_COMPVAL_TO_uT;
            p_hwdata->y = Y_tmp * 10 * AKM09911_COMPVAL_TO_uT;
            p_hwdata->z = Z_tmp * 10 * AKM09911_COMPVAL_TO_uT;
            break;
        case MAG_CHIP_AKM09915:
            p_hwdata->x = X_tmp * 10 * AKM09915_COMPVAL_TO_uT;
            p_hwdata->y = Y_tmp * 10 * AKM09915_COMPVAL_TO_uT;
            p_hwdata->z = Z_tmp * 10 * AKM09915_COMPVAL_TO_uT;
            break;
        case MAG_CHIP_AKM09916:
            p_hwdata->x = X_tmp * 10 * AKM09916_COMPVAL_TO_uT;
            p_hwdata->y = Y_tmp * 10 * AKM09916_COMPVAL_TO_uT;
            p_hwdata->z = Z_tmp * 10 * AKM09916_COMPVAL_TO_uT;
            break;
        default:
            break;
    }
    p_hwdata->timestamp = mag_poll_tmfilt(sys_tm);

    hw_remap_sensor_data(&(p_hwdata->x), &(p_hwdata->y), &(p_hwdata->z), g_place_m);

    ret = dest_list->list_add_rear((void *) p_hwdata);
    if (ret)
    {
        PERR("list_add_rear() fail, ret = %d", ret);
        if(-1 == ret){
            free(p_hwdata);
        }
    }

    return;
}

static void ap_hw_poll_yas5xx(BstSimpleList *dest_list)
{
    int32_t ret;
    FILE *value_fp = NULL;
    static char fname_buf[MAX_FILENAME_LEN+1] = {0};
    HW_DATA_UNION *p_hwdata;
    int32_t X_tmp = 0;
    int32_t Y_tmp = 0;
    int32_t Z_tmp = 0;
    int64_t sys_tm = 0;

    if(0 == fname_buf[0]){
        snprintf(fname_buf, MAX_FILENAME_LEN, "%s/%s", mag_input_dir_name, "value");
    }

    value_fp = fopen(fname_buf, "r");
    if (NULL == value_fp)
    {
        PERR("can not open %s, errno = %d(%s)", fname_buf, errno, strerror(errno));
        return;
    }

    sys_tm = android::elapsedRealtimeNano();

    /*YAS need reset power mode to let it update data register*/
    ret = wr_sysfs_oneint("op_mode", mag_input_dir_name, SENSOR_PM_NORMAL);

    ret = fscanf(value_fp, "%8d %8d %8d", &X_tmp, &Y_tmp, &Z_tmp);
    fclose(value_fp);
    if(ret <= 0){
        PERR("fscanf fail, errno = %d(%s)", errno, strerror(errno));
        return;
    }

    p_hwdata = (HW_DATA_UNION *) calloc(1, sizeof(HW_DATA_UNION));
    if (NULL == p_hwdata)
    {
        PERR("malloc fail");
        return;
    }

    p_hwdata->id = SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED;
    p_hwdata->x = X_tmp * 10 * YAS5xx_COMPVAL_TO_uT;
    p_hwdata->y = Y_tmp * 10 * YAS5xx_COMPVAL_TO_uT;
    p_hwdata->z = Z_tmp * 10 * YAS5xx_COMPVAL_TO_uT;
    p_hwdata->timestamp = mag_poll_tmfilt(sys_tm);

    hw_remap_sensor_data(&(p_hwdata->x), &(p_hwdata->y), &(p_hwdata->z), g_place_m);

    ret = dest_list->list_add_rear((void *) p_hwdata);
    if (ret)
    {
        PERR("list_add_rear() fail, ret = %d", ret);
        if(-1 == ret){
            free(p_hwdata);
        }
    }

    return;
}

static void ap_hw_poll_mmc(BstSimpleList *dest_list)
{
    int32_t ret;
    FILE *value_fp = NULL;
    static char fname_buf[MAX_FILENAME_LEN+1] = {0};
    HW_DATA_UNION *p_hwdata;
    int32_t X_tmp = 0;
    int32_t Y_tmp = 0;
    int32_t Z_tmp = 0;
    int64_t sys_tm = 0;

    if(0 == fname_buf[0]){
        snprintf(fname_buf, MAX_FILENAME_LEN, "%s/%s", mag_input_dir_name, "value");
    }

    value_fp = fopen(fname_buf, "r");
    if (NULL == value_fp)
    {
        PERR("can not open %s, errno = %d(%s)", fname_buf, errno, strerror(errno));
        return;
    }

    sys_tm = android::elapsedRealtimeNano();

    /*No need to triger measure process*/
    //ret = wr_sysfs_oneint("op_mode", mag_input_dir_name, SENSOR_PM_NORMAL);

    ret = fscanf(value_fp, "%8d %8d %8d", &X_tmp, &Y_tmp, &Z_tmp);
    fclose(value_fp);
    if(ret <= 0){
        PERR("fscanf fail, errno = %d(%s)", errno, strerror(errno));
        return;
    }

    p_hwdata = (HW_DATA_UNION *) calloc(1, sizeof(HW_DATA_UNION));
    if (NULL == p_hwdata)
    {
        PERR("malloc fail");
        return;
    }

    p_hwdata->id = SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED;
    p_hwdata->x = X_tmp * MMC_COMPVAL_TO_mG;
    p_hwdata->y = Y_tmp * MMC_COMPVAL_TO_mG;
    p_hwdata->z = Z_tmp * MMC_COMPVAL_TO_mG;
    p_hwdata->timestamp = mag_poll_tmfilt(sys_tm);

    hw_remap_sensor_data(&(p_hwdata->x), &(p_hwdata->y), &(p_hwdata->z), g_place_m);

    ret = dest_list->list_add_rear((void *) p_hwdata);
    if (ret)
    {
        PERR("list_add_rear() fail, ret = %d", ret);
        if(-1 == ret){
            free(p_hwdata);
        }
    }

    return;
}

static void ap_hw_poll_IICmag(int32_t chip_type, BstSimpleList *dest_list)
{
    if(MAG_CHIP_BMM150 == chip_type){
        ap_hw_poll_bmm150(dest_list);
    }else if(MAG_CHIP_AKM09912 == chip_type || MAG_CHIP_AKM09911 == chip_type ||
            MAG_CHIP_AKM09915 == chip_type || MAG_CHIP_AKM09916 == chip_type){
        ap_hw_poll_akm099xx(dest_list);
    }else if(MAG_CHIP_YAS537 == chip_type || MAG_CHIP_YAS532 == chip_type){
        ap_hw_poll_yas5xx(dest_list);
    }else if(MAG_CHIP_MMC == chip_type){
        ap_hw_poll_mmc(dest_list);
    }

    return;
}


static void ap_hw_poll_bmi055(BstSimpleList *acc_dest_list, BstSimpleList *mag_dest_list, BstSimpleList *gyr_dest_list)
{
    if(ACC_CHIP_BMA2x2 == accl_chip && is_acc_open)
    {
        /* add acc_poll_cnt and gyr_poll_cnt for BMI055 to eliminate the dependency on expected restrict
         * behavior of driver and chip: they should return len of 0 if there's no new samples */
        if(acc_poll_cnt == acc_poll_mulp)
        {
            ap_hw_poll_bma2x2(acc_dest_list);
            acc_poll_cnt = 1;
        }else{
            acc_poll_cnt++;
        }
    }

    if(GYR_CHIP_BMG160 == gyro_chip && is_gyr_open)
    {
        if(gyr_poll_cnt == gyr_poll_mulp)
        {
            ap_hw_poll_bmg160(gyr_dest_list);
            gyr_poll_cnt = 1;
        }else{
            gyr_poll_cnt++;
        }
    }

    if((MAG_CHIP_BMM150 == magn_chip ||
            MAG_CHIP_AKM09912 == magn_chip || MAG_CHIP_AKM09911 == magn_chip ||
            MAG_CHIP_AKM09915 == magn_chip || MAG_CHIP_AKM09916 == magn_chip ||
            MAG_CHIP_YAS537 ==  magn_chip || MAG_CHIP_YAS532 ==  magn_chip || MAG_CHIP_MMC == magn_chip) &&
            is_mag_open)
    {
        if(mag_poll_cnt == mag_poll_mulp)
        {
            ap_hw_poll_IICmag(magn_chip, mag_dest_list);
            mag_poll_cnt = 1;
        }else{
            mag_poll_cnt++;
        }
    }

    return;
}

#define BOUNDARY_I2C_QUP 256
/*! FIFO Head definition*/
#define FIFO_HEAD_DATAMOD 0x80
#define FIFO_HEAD_CTRLMOD 0x40
#define FIFO_HEAD_DATA_BITM 4
#define FIFO_HEAD_DATA_BITG 3
#define FIFO_HEAD_DATA_BITA 2
#define FIFO_HEAD_DATA_BITINT1 1
#define FIFO_HEAD_DATA_BITINT0 0
#define FIFO_HEAD_CTRL_BITTM  2
#define MAG_FRM_LEN 8 //2BytesX + 2BytesY + 2BytesZ + 2RHALL
#define GYR_FRM_LEN 6 //2BytesX + 2BytesY + 2BytesZ
#define ACC_FRM_LEN 6 //2BytesX + 2BytesY + 2BytesZ
#define FIFO_HEAD_SKIP_FRAME            0x40
#define FIFO_HEAD_SENSOR_TIME           0x44
#define FIFO_HEAD_INPUT_CONFIG          0x48
#define FIFO_HEAD_READ_OVER             0x00
#define SKIP_FRM_LEN  1
#define TM_FRM_LEN  3
#define INCONF_FRM_LEN  1

typedef struct
{
    uint8_t head;
    uint8_t payload[0];
}BMI160_FIFO_FRAME;

static uint8_t acc_frm_buf[1024] = {0};
static uint8_t mag_frm_buf[1024] = {0};
static uint8_t gyr_frm_buf[1024] = {0};
/**
 * @param p_frame
 * @return
 * If meets any unknown head, neglect it and continue to match head.
 * In practise maybe there's some unknown bytes.
 */
static int32_t bmi160_fifoframe_datalen(BMI160_FIFO_FRAME *p_frame)
{
    int32_t data_len = 0;

    if(p_frame->head & FIFO_HEAD_DATAMOD)
    {
        if(p_frame->head & (1 << FIFO_HEAD_DATA_BITM)){
            data_len += MAG_FRM_LEN;
        }
        if(p_frame->head & (1 << FIFO_HEAD_DATA_BITG)){
            data_len += GYR_FRM_LEN;
        }
        if(p_frame->head & (1 << FIFO_HEAD_DATA_BITA)){
            data_len += ACC_FRM_LEN;
        }
    }else if(p_frame->head & FIFO_HEAD_CTRLMOD)
    {
        switch(p_frame->head){
            case FIFO_HEAD_SKIP_FRAME:
                data_len = SKIP_FRM_LEN;
                break;
            case FIFO_HEAD_SENSOR_TIME:
                data_len = TM_FRM_LEN;
                break;
            case FIFO_HEAD_INPUT_CONFIG:
                data_len = INCONF_FRM_LEN;
                break;
            default:
                PWARN("UNKOWN CONTROL CODE: 0x%.2x", p_frame->head);
                break;
        }
    }else if(FIFO_HEAD_READ_OVER == p_frame->head)
    {
        /*no data, neglect*/
        return 0;
    }else
    {
        PWARN("UNKOWN HEAD MODE: 0x%.2x", p_frame->head);
        return 0;
    }

    return data_len;
}

/**
    NOTE: On Nexus 5, i2c driver=i2c_qup, I2C is implemented with 256 Bytes boundary.
    So the out data need a special handle. And BMI160 FIFO can resend the uncompleted frame(refer data sheet).

    0                  255
    ____________________
    |               |***|
    ---------------------
    |                 |*|
    ---------------------
    |              |****|
    ---------------------
    |                |**|
    ---------------------

"***" means the data sent again by BMI160 FIFO

@return extra offset to add when move to next frame in fifo_data.

*/
uint32_t bmi160_fifo_extra_offset(uint8_t *p_fifodata, BMI160_FIFO_FRAME *p_frame)
{
    uint32_t offset;
    uint32_t multiple;
    uint32_t ret_val = 0;
    int32_t data_len;

    offset = (uint8_t *)p_frame - p_fifodata;
    multiple = offset / BOUNDARY_I2C_QUP;
    data_len = bmi160_fifoframe_datalen(p_frame);
    if(data_len < 0)
    {
        return 0;
    }

    if(offset + sizeof(BMI160_FIFO_FRAME) + data_len > (multiple + 1) * BOUNDARY_I2C_QUP){
        ret_val = (multiple + 1) * BOUNDARY_I2C_QUP - offset;
    }

    return ret_val;
}


static void bmi160_fifo_parse(uint8_t *p_fifodata, uint32_t fifodata_len,
        uint32_t *p_acc_frmcnt, uint32_t *p_mag_frmcnt, uint32_t *p_gyr_frmcnt)
{
    uint32_t acc_frm_cnt = 0;
    uint32_t mag_frm_cnt = 0;
    uint32_t gyr_frm_cnt = 0;
    BMI160_FIFO_FRAME *p_frame;
    uint8_t *p_fifoSample;
    int32_t frame_datalen = 0;
    uint32_t extra_offset = 0;

    *p_acc_frmcnt = 0;
    *p_mag_frmcnt = 0;
    *p_gyr_frmcnt = 0;

    if(0 == fifodata_len){
        return;
    }

    p_frame = (BMI160_FIFO_FRAME *)p_fifodata;
    while (1)
    {
        p_fifoSample = p_frame->payload;
        frame_datalen = bmi160_fifoframe_datalen(p_frame);

        if( (p_fifoSample + frame_datalen) > (p_fifodata+fifodata_len))
        {
            /*Exception: if the last frame is not complete, neglect it.*/
            break;
        }

        if(p_frame->head & FIFO_HEAD_DATAMOD)
        {
            if(p_frame->head & (1 << FIFO_HEAD_DATA_BITM)){
                memcpy(mag_frm_buf + (mag_frm_cnt * MAG_FRM_LEN), p_fifoSample, MAG_FRM_LEN);
                p_fifoSample += MAG_FRM_LEN;
                mag_frm_cnt++;
            }
            if(p_frame->head & (1 << FIFO_HEAD_DATA_BITG)){
                memcpy(gyr_frm_buf + (gyr_frm_cnt * GYR_FRM_LEN), p_fifoSample, GYR_FRM_LEN);
                p_fifoSample += GYR_FRM_LEN;
                gyr_frm_cnt++;
            }
            if(p_frame->head & (1 << FIFO_HEAD_DATA_BITA)){
                memcpy(acc_frm_buf + (acc_frm_cnt * ACC_FRM_LEN), p_fifoSample, ACC_FRM_LEN);
                p_fifoSample += ACC_FRM_LEN;
                acc_frm_cnt++;
            }
        }else if(p_frame->head & FIFO_HEAD_CTRLMOD)
        {
            /*neglect*/
            p_fifoSample += frame_datalen;
        }else if(FIFO_HEAD_READ_OVER != p_frame->head)
        {
            /* If meets any unknown head, neglect it and continue to match head.
             * In practise maybe there's some unknown bytes.
             */
            PWARN("UNKOWN HEAD MODE: 0x%.2x, offset=: %u", p_frame->head, (uint8_t *)p_frame-(uint8_t *)p_fifodata);
        }

        if( p_fifoSample >= (p_fifodata + fifodata_len))
        {
            /*No data to handle*/
            break;
        }else{
            /*move to next frame*/
            p_frame = (BMI160_FIFO_FRAME *)p_fifoSample;
            extra_offset = bmi160_fifo_extra_offset(p_fifodata, p_frame);
            p_frame += extra_offset;
        }
    }

    *p_acc_frmcnt = acc_frm_cnt;
    *p_mag_frmcnt = mag_frm_cnt;
    *p_gyr_frmcnt = gyr_frm_cnt;
    return;
}


static void bmi160_mag_compensate(int16_t *p_X, int16_t *p_Y, int16_t *p_Z, int16_t RHALL)
{
    int32_t fd = -1;
    char fname_buf[MAX_FILENAME_LEN+1] = {0};
    int32_t ret = 0;
    uint8_t compensation_buf[MAG_FRM_LEN] = {0};

    snprintf(fname_buf, MAX_FILENAME_LEN, "%s/%s", bmi_input_dir_name, "mag_compensate");

    fd = open(fname_buf, O_WRONLY | O_NONBLOCK);
    if(-1 == fd){
        PERR("can not open %s, errno = %d(%s)", fname_buf, errno, strerror(errno));
        return;
    }

    compensation_buf[0] = (*p_X) & 0xFF;
    compensation_buf[1] = (*p_X) >> 8;
    compensation_buf[2] = (*p_Y) & 0xFF;
    compensation_buf[3] = (*p_Y) >> 8;
    compensation_buf[4] = (*p_Z) & 0xFF;
    compensation_buf[5] = (*p_Z) >> 8;
    compensation_buf[6] = (RHALL) & 0xFF;
    compensation_buf[7] = (RHALL) >> 8;

    ret = write(fd, compensation_buf, sizeof(compensation_buf));
    close(fd);
    if(ret < 0 ){
        PERR("write %d fail: ret = %d, errno = %d(%s)", fd, ret, errno, strerror(errno));
        return;
    }

    fd = open(fname_buf, O_RDONLY | O_NONBLOCK);
    if(-1 == fd){
        PERR("can not open %s, errno = %d(%s)", fname_buf, errno, strerror(errno));
        return;
    }

    ret = read(fd, compensation_buf, sizeof(compensation_buf));
    close(fd);
    if(ret < 0 ){
        PERR("read %d fail: ret = %d, errno = %d(%s)", fd, ret, errno, strerror(errno));
        return;
    }

    memcpy(p_X, &compensation_buf[0], sizeof(int16_t));
    memcpy(p_Y, &compensation_buf[2], sizeof(int16_t));
    memcpy(p_Z, &compensation_buf[4], sizeof(int16_t));

    return;
}

static void bmi160_esttm_and_deliver(uint32_t physical_sensor_id, int64_t sys_tm,
        uint32_t frmcnt, BstSimpleList *dest_list)
{
    int32_t ret = 0;
    HW_DATA_UNION *p_hwdata = NULL;
    int16_t X_tmp = 0;
    int16_t Y_tmp = 0;
    int16_t Z_tmp = 0;
    int16_t RHALL_tmp = 0;
    int64_t avg_interval_ns = 0;
    uint32_t i = 0;
    float deviate_percent = 1;
    static int64_t bma_poll_start_tm = 0;
    static int64_t bma_last_frm_tm = 0;
    static uint32_t bma_total_frm = 0;
    static int64_t bmg_poll_start_tm = 0;
    static int64_t bmg_last_frm_tm = 0;
    static uint32_t bmg_total_frm = 0;
    static int64_t mag_poll_start_tm = 0;
    static int64_t mag_last_frm_tm = 0;
    static int32_t mag_total_frm = 0;

    if(0 == frmcnt)
    {
        return;
    }

    if(BSX_INPUT_ID_ACCELERATION == physical_sensor_id)
    {
        if(0 == bma_if_start_up)
        {
            /*update time stamp estimation base line*/
            bma_poll_start_tm = sys_tm;
            bma_last_frm_tm = bma_poll_start_tm;
            bma_total_frm = 0;
            bma_odr_set = bma_odr_tmp;

            bma_if_start_up = 1;
            return;
        }


        bma_total_frm += frmcnt;
        avg_interval_ns = (sys_tm - bma_poll_start_tm) / bma_total_frm;

        for (i = 0; i < frmcnt; ++i)
        {
            X_tmp = acc_frm_buf[i*ACC_FRM_LEN+0] | (acc_frm_buf[i*ACC_FRM_LEN+1] << 8);
            Y_tmp = acc_frm_buf[i*ACC_FRM_LEN+2] | (acc_frm_buf[i*ACC_FRM_LEN+3] << 8);
            Z_tmp = acc_frm_buf[i*ACC_FRM_LEN+4] | (acc_frm_buf[i*ACC_FRM_LEN+5] << 8);

            p_hwdata = (HW_DATA_UNION *) calloc(1, sizeof(HW_DATA_UNION));
            if (NULL == p_hwdata)
            {
                PERR("malloc fail");
                continue;
            }

            p_hwdata->id = SENSOR_TYPE_ACCELEROMETER;
            p_hwdata->x = X_tmp * BMI160_acc_resl;
            p_hwdata->y = Y_tmp * BMI160_acc_resl;
            p_hwdata->z = Z_tmp * BMI160_acc_resl;
            p_hwdata->timestamp = bma_last_frm_tm + avg_interval_ns;
            bma_last_frm_tm += avg_interval_ns;

            if(p_hwdata->timestamp > sys_tm)
            {
                deviate_percent = (float)(p_hwdata->timestamp - sys_tm) / (float)avg_interval_ns;
                if(deviate_percent < TM_DEVIATE_PERCENT)
                {
                    p_hwdata->timestamp = sys_tm;
                    bma_last_frm_tm = p_hwdata->timestamp;

                    hw_remap_sensor_data(&(p_hwdata->x), &(p_hwdata->y), &(p_hwdata->z), g_place_a);

                    ret = dest_list->list_add_rear((void *) p_hwdata);
                    if (ret)
                    {
                        PERR("list_add_rear() fail, ret = %d", ret);
                        if(-1 == ret){
                            free(p_hwdata);
                        }
                    }

                }else{

                    free(p_hwdata);
                    bma_last_frm_tm -= avg_interval_ns;
                }

                break;

            }else
            {
                hw_remap_sensor_data(&(p_hwdata->x), &(p_hwdata->y), &(p_hwdata->z), g_place_a);

                ret = dest_list->list_add_rear((void *) p_hwdata);
                if (ret)
                {
                    PERR("list_add_rear() fail, ret = %d", ret);
                    if(-1 == ret){
                        free(p_hwdata);
                    }
                }
            }
        }

    }else if(BSX_INPUT_ID_MAGNETICFIELD == physical_sensor_id)
    {
        if(0 == mag_if_start_up)
        {
            /*update time stamp estimation base line*/
            mag_poll_start_tm = sys_tm;
            mag_last_frm_tm = mag_poll_start_tm;
            mag_total_frm = 0;
            mag_odr_set = mag_odr_tmp;

            mag_if_start_up = 1;
            return;
        }


        mag_total_frm += frmcnt;
        avg_interval_ns = (sys_tm - mag_poll_start_tm) / mag_total_frm;

        for (i = 0; i < frmcnt; ++i)
        {
            X_tmp = mag_frm_buf[i*MAG_FRM_LEN+0] | (mag_frm_buf[i*MAG_FRM_LEN+1] << 8);
            Y_tmp = mag_frm_buf[i*MAG_FRM_LEN+2] | (mag_frm_buf[i*MAG_FRM_LEN+3] << 8);
            Z_tmp = mag_frm_buf[i*MAG_FRM_LEN+4] | (mag_frm_buf[i*MAG_FRM_LEN+5] << 8);
            RHALL_tmp = mag_frm_buf[i*MAG_FRM_LEN+6] | (mag_frm_buf[i*MAG_FRM_LEN+7] << 8);

            bmi160_mag_compensate(&X_tmp, &Y_tmp, &Z_tmp, RHALL_tmp);

            p_hwdata = (HW_DATA_UNION *) calloc(1, sizeof(HW_DATA_UNION));
            if (NULL == p_hwdata)
            {
                PERR("malloc fail");
                continue;
            }

            p_hwdata->id = SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED;
            p_hwdata->x = X_tmp * 10 / BMM_COMPVAL_TO_uT;
            p_hwdata->y = Y_tmp * 10 / BMM_COMPVAL_TO_uT;
            p_hwdata->z = Z_tmp * 10 / BMM_COMPVAL_TO_uT;
            p_hwdata->timestamp = mag_last_frm_tm + avg_interval_ns;
            mag_last_frm_tm += avg_interval_ns;

            if(p_hwdata->timestamp > sys_tm)
            {
                deviate_percent = (float)(p_hwdata->timestamp - sys_tm) / (float)avg_interval_ns;
                if(deviate_percent < TM_DEVIATE_PERCENT)
                {
                    p_hwdata->timestamp = sys_tm;
                    mag_last_frm_tm = p_hwdata->timestamp;

                    hw_remap_sensor_data(&(p_hwdata->x), &(p_hwdata->y), &(p_hwdata->z), g_place_m);

                    ret = dest_list->list_add_rear((void *) p_hwdata);
                    if (ret)
                    {
                        PERR("list_add_rear() fail, ret = %d", ret);
                        if(-1 == ret){
                            free(p_hwdata);
                        }
                    }

                }else{

                    free(p_hwdata);
                    mag_last_frm_tm -= avg_interval_ns;
                }

                break;

            }else
            {
                hw_remap_sensor_data(&(p_hwdata->x), &(p_hwdata->y), &(p_hwdata->z), g_place_m);

                ret = dest_list->list_add_rear((void *) p_hwdata);
                if (ret)
                {
                    PERR("list_add_rear() fail, ret = %d", ret);
                    if(-1 == ret){
                        free(p_hwdata);
                    }
                }
            }
        }

    }else if(BSX_INPUT_ID_ANGULARRATE == physical_sensor_id)
    {
        if(0 == bmg_if_start_up)
        {
            /*update time stamp estimation base line*/
            bmg_poll_start_tm = sys_tm;
            bmg_last_frm_tm = bmg_poll_start_tm;
            bmg_total_frm = 0;
            bmg_odr_set = bmg_odr_tmp;

            bmg_if_start_up = 1;
            return;
        }


        bmg_total_frm += frmcnt;
        avg_interval_ns = (sys_tm - bmg_poll_start_tm) / bmg_total_frm;

        for (i = 0; i < frmcnt; ++i)
        {
            X_tmp = gyr_frm_buf[i*GYR_FRM_LEN+0] | (gyr_frm_buf[i*GYR_FRM_LEN+1] << 8);
            Y_tmp = gyr_frm_buf[i*GYR_FRM_LEN+2] | (gyr_frm_buf[i*GYR_FRM_LEN+3] << 8);
            Z_tmp = gyr_frm_buf[i*GYR_FRM_LEN+4] | (gyr_frm_buf[i*GYR_FRM_LEN+5] << 8);

            p_hwdata = (HW_DATA_UNION *) calloc(1, sizeof(HW_DATA_UNION));
            if (NULL == p_hwdata)
            {
                PERR("malloc fail");
                continue;
            }

            p_hwdata->id = SENSOR_TYPE_GYROSCOPE_UNCALIBRATED;
            p_hwdata->x = X_tmp;
            p_hwdata->y = Y_tmp;
            p_hwdata->z = Z_tmp;
            p_hwdata->timestamp = bmg_last_frm_tm + avg_interval_ns;
            bmg_last_frm_tm += avg_interval_ns;

            if(p_hwdata->timestamp > sys_tm)
            {
                deviate_percent = (float)(p_hwdata->timestamp - sys_tm) / (float)avg_interval_ns;
                if(deviate_percent < TM_DEVIATE_PERCENT)
                {
                    p_hwdata->timestamp = sys_tm;
                    bmg_last_frm_tm = p_hwdata->timestamp;

                    hw_remap_sensor_data(&(p_hwdata->x), &(p_hwdata->y), &(p_hwdata->z), g_place_g);

                    ret = dest_list->list_add_rear((void *) p_hwdata);
                    if (ret)
                    {
                       PERR("list_add_rear() fail, ret = %d", ret);
                       if(-1 == ret){
                           free(p_hwdata);
                       }
                    }

                }else{

                    free(p_hwdata);
                    bmg_last_frm_tm -= avg_interval_ns;
                }

                break;

            }else
            {
                hw_remap_sensor_data(&(p_hwdata->x), &(p_hwdata->y), &(p_hwdata->z), g_place_g);

                ret = dest_list->list_add_rear((void *) p_hwdata);
                if (ret)
                {
                    PERR("list_add_rear() fail, ret = %d", ret);
                    if(-1 == ret){
                        free(p_hwdata);
                    }
                }
            }
        }

    }

    return;
}


static void ap_hw_poll_bmi160(BstSimpleList *acc_dest_list, BstSimpleList *mag_dest_list, BstSimpleList *gyr_dest_list)
{
    if(is_acc_open || is_gyr_open || (MAG_CHIP_BMI160 ==  magn_chip && is_mag_open))
    {
        int32_t ret;
        int32_t fifo_fd = -1;
        static char fname_buf[MAX_FILENAME_LEN+1] = {0};
        static uint8_t fifo_data_buf[1024] = {0}; //max size supported by BMI160
        uint32_t acc_frmcnt = 0;
        uint32_t mag_frmcnt = 0;
        uint32_t gyr_frmcnt = 0;
        int64_t sys_tm = 0;

        sys_tm = android::elapsedRealtimeNano();

        if(0 == fname_buf[0]){
            snprintf(fname_buf, MAX_FILENAME_LEN, "%s/%s", bmi_input_dir_name, "fifo_data_frame");
        }

        fifo_fd = open(fname_buf, O_RDONLY | O_NONBLOCK);
        if(-1 == fifo_fd){
            PERR("can not open %s, errno = %d(%s)", fname_buf, errno, strerror(errno));
            return;
        }

        ret = read(fifo_fd, fifo_data_buf, sizeof(fifo_data_buf));
        close(fifo_fd);
        if(ret < 0 ){
            PERR("read %d fail: ret = %d, errno = %d(%s)", fifo_fd, ret, errno, strerror(errno));
            return;
        }

        bmi160_fifo_parse(fifo_data_buf, ret, &acc_frmcnt, &mag_frmcnt, &gyr_frmcnt);

        /*assign timestamp*/
        bmi160_esttm_and_deliver(BSX_INPUT_ID_ACCELERATION, sys_tm, acc_frmcnt, acc_dest_list);
        bmi160_esttm_and_deliver(BSX_INPUT_ID_MAGNETICFIELD, sys_tm, mag_frmcnt, mag_dest_list);
        bmi160_esttm_and_deliver(BSX_INPUT_ID_ANGULARRATE, sys_tm, gyr_frmcnt, gyr_dest_list);
    }

    if((MAG_CHIP_BMM150 == magn_chip ||
            MAG_CHIP_AKM09912 == magn_chip || MAG_CHIP_AKM09911 == magn_chip ||
            MAG_CHIP_AKM09915 == magn_chip || MAG_CHIP_AKM09916 == magn_chip ||
            MAG_CHIP_YAS537 ==  magn_chip || MAG_CHIP_YAS532 ==  magn_chip || MAG_CHIP_MMC == magn_chip) &&
            is_mag_open)
    {
        if(mag_poll_cnt == mag_poll_mulp)
        {
            ap_hw_poll_IICmag(magn_chip, mag_dest_list);
            mag_poll_cnt = 1;
        }else{
            mag_poll_cnt++;
        }
    }

    return;
}

static void ap_hw_process_hwevent(BstSensor *bstsensor)
{
    int32_t ret;
    struct input_event event[2];
    int64_t sys_tm = 0;
    sensors_event_t *p_event = NULL;

    while( (ret = read(hw_event_fd, event, sizeof(event))) > 0)
    {
        if(EV_MSC != event[0].type || EV_SYN != event[1].type)
        {
            PWARN("Unexpected events: %d, %d; %d;",
                    event[0].type, event[0].code,
                    event[1].type);
            continue;
        }

        sys_tm = android::elapsedRealtimeNano();

        switch (event[0].code) {
            case EVENT_CODE_HW_SIGMOTION:
                p_event = (sensors_event_t *) calloc(1, sizeof(sensors_event_t));
                if (NULL == p_event){
                    PWARN("calloc fail");
                    break;
                }

                p_event->version = sizeof(sensors_event_t);
                p_event->sensor = BSX_SENSOR_ID_SIGNIFICANT_MOTION_WAKEUP;
                p_event->type = SENSOR_TYPE_SIGNIFICANT_MOTION;
                p_event->data[0] = 1;
                p_event->timestamp = sys_tm;
                bstsensor->sensord_deliver_event(p_event);

                break;
            case EVENT_CODE_HW_STEPD:
                p_event = (sensors_event_t *) calloc(1, sizeof(sensors_event_t));
                if (NULL == p_event){
                    PWARN("calloc fail");
                    break;
                }

                p_event->version = sizeof(sensors_event_t);
                p_event->sensor = BSX_SENSOR_ID_STEP_DETECTOR_WAKEUP;
                p_event->type = SENSOR_TYPE_STEP_DETECTOR;
                p_event->data[0] = 1;
                p_event->timestamp = sys_tm;
                bstsensor->sensord_deliver_event(p_event);

                break;
            case EVENT_CODE_HW_FOC:
                //TODO:
                //PNOTE("receive FOC offset: 0x%x", event[0].value);

                break;
            default:
                break;
        }

    }

    return;
}

static void ap_hw_poll_hwstc(BstSensor *bstsensor)
{
    int32_t ret;
    sensors_event_t *p_event = NULL;
    int32_t stepcnt_tmp = 0;
    int64_t sys_tm = 0;

    if(is_hwstc_open)
    {
        if(hwstc_poll_cnt == hwstc_poll_mulp)
        {
            sys_tm = android::elapsedRealtimeNano();

            ret = rd_sysfs_oneint("stc_value", bmi_input_dir_name, &stepcnt_tmp);
            if (0 != ret)
            {
                PERR("read fail, ret = %d, errno = %d(%s)", ret, errno, strerror(errno));
                return;
            }

            if(stepcnt_tmp == stepc_internal){
                /*now it stands still, don't report*/
                return;
            }

            p_event = (sensors_event_t *) calloc(1, sizeof(sensors_event_t));
            if (NULL == p_event){
                PWARN("calloc fail");
                return;
            }

            p_event->version = sizeof(sensors_event_t);
            p_event->sensor = BSX_SENSOR_ID_STEP_COUNTER_WAKEUP;
            p_event->type = SENSOR_TYPE_STEP_COUNTER;
            p_event->u64.step_counter = (uint64_t)stepcnt_tmp;
            p_event->timestamp = sys_tm;

            bstsensor->sensord_deliver_event(p_event);

            stepc_internal = stepcnt_tmp;


            hwstc_poll_cnt = 1;
        }else{
            hwstc_poll_cnt++;
        }
    }

    return;
}

/*
static void dump_samples(BstSimpleList *al, BstSimpleList *gl, BstSimpleList *ml)
{
    HW_DATA_UNION *p_hwdata;
    struct list_node *pnode;
    uint32_t i;

    pnode = al->head;
    for (i = 0; i < al->list_len; ++i) {
        p_hwdata = (HW_DATA_UNION *)(pnode->p_data);
        PNOTE("**ACCL tm = %lld", p_hwdata->timestamp);
        pnode = pnode->next;
    }

    pnode = gl->head;
    for (i = 0; i < gl->list_len; ++i) {
        p_hwdata = (HW_DATA_UNION *)(pnode->p_data);
        PNOTE("--GYRO tm = %lld", p_hwdata->timestamp);
        pnode = pnode->next;
    }

    pnode = ml->head;
    for (i = 0; i < ml->list_len; ++i) {
        p_hwdata = (HW_DATA_UNION *)(pnode->p_data);
        PNOTE("##MAGN tm = %lld", p_hwdata->timestamp);
        pnode = pnode->next;
    }

    PNOTE("==============================");

    return;
}
*/


static uint32_t find_s_ind(HW_DATA_UNION *interpol_buf, uint32_t interpol_len, HW_DATA_UNION **in_buf, uint32_t in_len)
{
    uint32_t i;
    int32_t left_cnt;
    int32_t interval_cnt;
    int32_t ind_tmp;

    if(interpol_len <= 1 || 0 == in_len){
        PWARN("invalid parameter %d, %d", interpol_len, in_len);
        return 0;
    }

    if(in_buf[in_len - 1]->timestamp < interpol_buf[0].timestamp)
    {
        return 0;
    }

    if(in_buf[0]->timestamp > interpol_buf[interpol_len - 1].timestamp)
    {
        interval_cnt = interpol_len / in_len;
        left_cnt = in_len;
        ind_tmp = interpol_len - 1;
        while(left_cnt && (ind_tmp - interval_cnt) >= 0)
        {
            ind_tmp -= interval_cnt;
            left_cnt--;
        }

        return ind_tmp;
    }

    for (i = 0; i < interpol_len; ++i)
    {
        if(interpol_buf[i].timestamp >= in_buf[0]->timestamp)
        {
            if(0 == i)
            {
                break;
            }else
            {
                if(in_buf[0]->timestamp - interpol_buf[i-1].timestamp <=
                        interpol_buf[i].timestamp - in_buf[0]->timestamp)
                {
                    i -= 1;
                    break;
                }else{
                    break;
                }
            }
        }
    }

    interval_cnt = interpol_len / in_len;
    left_cnt = in_len;
    ind_tmp = i;

    while(left_cnt && (uint32_t)(ind_tmp + interval_cnt) <= (interpol_len - 1))
    {
        ind_tmp += interval_cnt;
        left_cnt--;
    }

    if(0 == left_cnt)
    {
        return i;
    }else
    {
        ind_tmp = i;
        while(left_cnt && (ind_tmp - interval_cnt) >= 0)
        {
            ind_tmp -= interval_cnt;
            left_cnt--;
        }

        return ind_tmp;
    }
}


#define MAX_TMBUF_LEN 16
#define MAX_MAGTMBUF_LEN 4

static void interpolation_align(HW_DATA_UNION **in_buf, uint32_t *p_in_len, float in_hz,
        int64_t *in_stdtm_buf, uint32_t in_stdtm_len, float in_std_hz)
{
    uint32_t i;
    uint32_t j;
    HW_DATA_UNION interpol_buf[MAX_TMBUF_LEN];
    uint32_t interval_cnt;
    uint32_t first_ind;
    HW_DATA_UNION *p_hwdata = NULL;
    uint32_t sensor_id;

    /*interpolation all points*/
    for (i = 0; i < in_stdtm_len; ++i)
    {
        if(in_stdtm_buf[i] < in_buf[0]->timestamp)
        {
            sensord_resample_at_tm(in_buf[0]->data, in_buf[0]->timestamp,
                    in_buf[1]->data, in_buf[1]->timestamp,
                    interpol_buf[i].data, in_stdtm_buf[i]);
            interpol_buf[i].timestamp = in_stdtm_buf[i];

        }else if(in_stdtm_buf[i] > in_buf[(*p_in_len) - 1]->timestamp)
        {
            sensord_resample_at_tm(in_buf[(*p_in_len) - 2]->data, in_buf[(*p_in_len) - 2]->timestamp,
                    in_buf[(*p_in_len) - 1]->data, in_buf[(*p_in_len) - 1]->timestamp,
                    interpol_buf[i].data, in_stdtm_buf[i]);
            interpol_buf[i].timestamp = in_stdtm_buf[i];

        }else
        {
            for (j = 0; j < ((*p_in_len) - 1); ++j)
            {
                if(in_stdtm_buf[i] >= in_buf[j]->timestamp && in_stdtm_buf[i] <= in_buf[j+1]->timestamp)
                {
                    sensord_resample_at_tm(in_buf[j]->data, in_buf[j]->timestamp,
                            in_buf[j+1]->data, in_buf[j+1]->timestamp,
                            interpol_buf[i].data, in_stdtm_buf[i]);
                    interpol_buf[i].timestamp = in_stdtm_buf[i];
                    break;
                }
            }
        }

    }

    /*select interpolation points*/
    interval_cnt = in_std_hz / in_hz;
    sensor_id = in_buf[0]->id;
    if(1 == interval_cnt)
    {
        /*clean old samples and generate new ones*/
        for (j = 0; j < (*p_in_len); ++j)
        {
            free(in_buf[j]);
            in_buf[j] = NULL;
        }
        (*p_in_len) = 0;

        for (j = 0; j < in_stdtm_len; ++j)
        {
            p_hwdata = (HW_DATA_UNION *)calloc(1, sizeof(HW_DATA_UNION));
            if(NULL != p_hwdata){
                p_hwdata->id = sensor_id;
                memcpy(p_hwdata->data, interpol_buf[j].data, sizeof(interpol_buf[j].data));
                p_hwdata->timestamp = interpol_buf[j].timestamp;
                in_buf[(*p_in_len)] = p_hwdata;
                (*p_in_len)++;
            }else{
                PWARN("malloc fail!");
            }
        }
    }else
    {
        first_ind = find_s_ind(interpol_buf, in_stdtm_len, in_buf, (*p_in_len));

        /*clean old samples and generate new ones*/
        for (j = 0; j < (*p_in_len); ++j)
        {
            free(in_buf[j]);
            in_buf[j] = NULL;
        }
        (*p_in_len) = 0;

        while(first_ind <= in_stdtm_len - 1)
        {
            p_hwdata = (HW_DATA_UNION *)calloc(1, sizeof(HW_DATA_UNION));
            if(NULL != p_hwdata){
                p_hwdata->id = sensor_id;
                memcpy(p_hwdata->data, interpol_buf[first_ind].data, sizeof(interpol_buf[first_ind].data));
                p_hwdata->timestamp = interpol_buf[first_ind].timestamp;
                in_buf[(*p_in_len)] = p_hwdata;
                (*p_in_len)++;
            }else{
                PWARN("malloc fail!");
            }
            first_ind += interval_cnt;
        }
    }

    return;
}


static void approximate_align(HW_DATA_UNION **in_buf, int64_t *in_stdtm_buf, uint32_t in_stdtm_len)
{
    uint32_t i;

    if(in_buf[0]->timestamp <= in_stdtm_buf[0])
    {
        in_buf[0]->timestamp = in_stdtm_buf[0];
    }else if(in_buf[0]->timestamp >= in_stdtm_buf[in_stdtm_len-1]){
        in_buf[0]->timestamp = in_stdtm_buf[in_stdtm_len-1];
    }else{
        for (i = 1; i < in_stdtm_len; ++i)
        {
            if(in_buf[0]->timestamp >= in_stdtm_buf[i-1] &&
                    in_buf[0]->timestamp <= in_stdtm_buf[i])
            {
                if(in_buf[0]->timestamp - in_stdtm_buf[i-1] < in_stdtm_buf[i] - in_buf[0]->timestamp)
                {
                    in_buf[0]->timestamp = in_stdtm_buf[i-1];
                }else
                {
                    in_buf[0]->timestamp = in_stdtm_buf[i];
                }
                break;
            }

        }
    }

    return;
}

static void sample_align_and_reload(HW_DATA_UNION **in_buf, uint32_t *p_in_len, float in_hz,
        int64_t *in_stdtm_buf, uint32_t in_stdtm_len, float in_std_hz, BstSimpleList *dl)
{
    uint32_t i;

    if(0 == (*p_in_len))
    {
        return;
    }else if(1 == (*p_in_len))
    {
        approximate_align(in_buf, in_stdtm_buf, in_stdtm_len);
    }else
    {
        interpolation_align(in_buf, p_in_len, in_hz, in_stdtm_buf, in_stdtm_len, in_std_hz);
    }

    for (i = 0; i < (*p_in_len); ++i) {
        dl->list_add_rear(in_buf[i]);
    }

    return;
}

static void syncTM_BMI(BstSimpleList *al, BstSimpleList *gl, BstSimpleList *ml, float ahz, float ghz, float mhz)
{
    uint32_t i;
    HW_DATA_UNION *p_hwdata = NULL;
    HW_DATA_UNION *in_acc_buf[MAX_TMBUF_LEN] = {0};
    uint32_t in_acc_len = 0;
    HW_DATA_UNION *in_gyr_buf[MAX_TMBUF_LEN] = {0};
    uint32_t in_gyr_len = 0;
    HW_DATA_UNION *in_mag_buf[MAX_MAGTMBUF_LEN] = {0};
    uint32_t in_mag_len = 0;
    int64_t in_stdtm_buf[MAX_TMBUF_LEN] = {0};
    uint32_t in_stdtm_len = 0;
    float in_std_hz;

    if(0 == gl->list_len + al->list_len ||
            0 == gl->list_len + ml->list_len ||
            0 == al->list_len + ml->list_len){
        /*nothing to do*/;
        return;
    }

    if(al->list_len > MAX_TMBUF_LEN)
    {
        /*This should not happen*/
        PWARN("acc samples amount %d exceed assumed %d", al->list_len, MAX_TMBUF_LEN);
        while(MAX_TMBUF_LEN < al->list_len){
            al->list_get_headdata((void **)&p_hwdata);
            free(p_hwdata);
        }
    }

    if(gl->list_len > MAX_TMBUF_LEN)
    {
        /*This should not happen*/
        PWARN("gyr samples amount %d exceed assumed %d", gl->list_len, MAX_TMBUF_LEN);
        while(MAX_TMBUF_LEN < gl->list_len){
            gl->list_get_headdata((void **)&p_hwdata);
            free(p_hwdata);
        }
    }

    if(ml->list_len > MAX_MAGTMBUF_LEN)
    {
        /*This should not happen*/
        PWARN("mag samples amount %d exceed assumed %d", ml->list_len, MAX_MAGTMBUF_LEN);
        while(MAX_MAGTMBUF_LEN < ml->list_len){
            ml->list_get_headdata((void **)&p_hwdata);
            free(p_hwdata);
        }
    }

    in_acc_len = al->list_len;
    for (i = 0; i < in_acc_len; ++i) {
        al->list_get_headdata((void **)&in_acc_buf[i]);
    }

    in_gyr_len = gl->list_len;
    for (i = 0; i < in_gyr_len; ++i) {
        gl->list_get_headdata((void **)&in_gyr_buf[i]);
    }

    in_mag_len = ml->list_len;
    for (i = 0; i < in_mag_len; ++i) {
        ml->list_get_headdata((void **)&in_mag_buf[i]);
    }

    /*magnetometer cannot run fater than acclerometer or gyroscope so far*/
    if(in_gyr_len && in_acc_len)
    {
        if(ghz >= ahz)
        {
            for (i = 0; i < in_gyr_len; ++i) {
                in_stdtm_buf[i] = in_gyr_buf[i]->timestamp;
            }
            in_stdtm_len = in_gyr_len;
            in_std_hz = ghz;
        }else
        {
            for (i = 0; i < in_acc_len; ++i) {
                in_stdtm_buf[i] = in_acc_buf[i]->timestamp;
            }
            in_stdtm_len = in_acc_len;
            in_std_hz = ahz;
        }
    }else
    {
        if(in_gyr_len)
        {
            for (i = 0; i < in_gyr_len; ++i) {
                in_stdtm_buf[i] = in_gyr_buf[i]->timestamp;
            }
            in_stdtm_len = in_gyr_len;
            in_std_hz = ghz;
        }else
        {
            for (i = 0; i < in_acc_len; ++i) {
                in_stdtm_buf[i] = in_acc_buf[i]->timestamp;
            }
            in_stdtm_len = in_acc_len;
            in_std_hz = ahz;
        }
    }


    if(1 == in_stdtm_len)
    {
        if(in_acc_len){
            in_acc_buf[0]->timestamp = in_stdtm_buf[0];
            al->list_add_rear(in_acc_buf[0]);
        }
        if(in_gyr_len){
            in_gyr_buf[0]->timestamp = in_stdtm_buf[0];
            gl->list_add_rear(in_gyr_buf[0]);
        }
        if(in_mag_len){
            in_mag_buf[0]->timestamp = in_stdtm_buf[0];
            ml->list_add_rear(in_mag_buf[0]);
        }

    }else{

        sample_align_and_reload(in_acc_buf, &in_acc_len, ahz, in_stdtm_buf, in_stdtm_len, in_std_hz, al);
        sample_align_and_reload(in_gyr_buf, &in_gyr_len, ghz, in_stdtm_buf, in_stdtm_len, in_std_hz, gl);
        sample_align_and_reload(in_mag_buf, &in_mag_len, mhz, in_stdtm_buf, in_stdtm_len, in_std_hz, ml);
    }

    return;
}



static uint32_t hwcntl_deliver_sensordata(BstSensor *bstsensor)
{
    int32_t ret;
    uint32_t i;
    struct pollfd poll_fds[2];
    uint64_t over_run = 0;

    poll_fds[0].fd = poll_timer_fd;
    poll_fds[0].events = POLLIN;
    poll_fds[1].fd = hw_event_fd;
    poll_fds[1].events = POLLIN;

    ret = poll(poll_fds, ARRAY_ELEMENTS(poll_fds), -1);
    if (ret <= 0)
    {
        PERR("poll in error: ret=%d", ret);
        return 0;
    }

    for (i = 0; i < ARRAY_ELEMENTS(poll_fds); i++)
    {
        if (POLLIN != poll_fds[i].revents)
        {
            continue;
        }

        switch (i)
        {
            case 0:
                ret = read(poll_timer_fd, &over_run, sizeof(uint64_t));
                if(sizeof(uint64_t) == ret){
                    if(ACC_CHIP_BMI160 == accl_chip)
                    {
                        ap_hw_poll_bmi160(bstsensor->tmplist_hwcntl_acclraw,
                                bstsensor->tmplist_hwcntl_magnraw,
                                bstsensor->tmplist_hwcntl_gyroraw);
                        if(1 == hardware_motion){
                            ap_hw_poll_hwstc(bstsensor);
                        }
                    }else if(ACC_CHIP_BMA2x2 == accl_chip)
                    {
                        ap_hw_poll_bmi055(bstsensor->tmplist_hwcntl_acclraw,
                                bstsensor->tmplist_hwcntl_magnraw,
                                bstsensor->tmplist_hwcntl_gyroraw);
                    }
                }

                break;

            case 1:
                if(1 == hardware_motion && ACC_CHIP_BMI160 == accl_chip)
                {
                    ap_hw_process_hwevent(bstsensor);
                }else
                {
                    PWARN("unexpected hardware event");
                }

                break;
        }

    }

#if 1
    if (0 == bstsensor->tmplist_hwcntl_acclraw->list_len
            + bstsensor->tmplist_hwcntl_gyroraw->list_len
            + bstsensor->tmplist_hwcntl_magnraw->list_len)
    {
        return 0;
    }

    //dump_samples(bstsensor->tmplist_hwcntl_acclraw, bstsensor->tmplist_hwcntl_gyroraw, bstsensor->tmplist_hwcntl_magnraw);
    syncTM_BMI(bstsensor->tmplist_hwcntl_acclraw,
            bstsensor->tmplist_hwcntl_gyroraw,
            bstsensor->tmplist_hwcntl_magnraw,
            bma_odr_set, bmg_odr_set, mag_odr_set);
    //dump_samples(bstsensor->tmplist_hwcntl_acclraw, bstsensor->tmplist_hwcntl_gyroraw, bstsensor->tmplist_hwcntl_magnraw);

    pthread_mutex_lock(&(bstsensor->shmem_hwcntl.mutex));

    ret = bstsensor->shmem_hwcntl.p_list->list_mount_rear(bstsensor->tmplist_hwcntl_acclraw);
    if(ret){
        PWARN("list mount fail");
    }

    ret = bstsensor->shmem_hwcntl.p_list->list_mount_rear(bstsensor->tmplist_hwcntl_gyroraw);
    if(ret){
        PWARN("list mount fail");
    }

    ret = bstsensor->shmem_hwcntl.p_list->list_mount_rear(bstsensor->tmplist_hwcntl_magnraw);
    if(ret){
        PWARN("list mount fail");
    }

    pthread_cond_signal(&(bstsensor->shmem_hwcntl.cond));
    pthread_mutex_unlock(&(bstsensor->shmem_hwcntl.mutex));
#endif
    return 0;

}

#endif

uint32_t UNITTEST_hw_deliver_sensordata(BstSensor *bstsensor)
{
    static int64_t cur_time = 0x100;
    static int32_t stub_index = 0;
    int32_t sim_interval_ns = 5000000;
    HW_DATA_UNION *p_hwdata;
    int32_t ret;
    static int32_t if_end_deliver = 0;

    float acc_stub[] = {
            52, -41, 927,
            28, 658, 616,
            25, 666, 622,
            49, -39, 927,
            52, -41, 927,
            28, 658, 616,
            25, 666, 622,
            49, -39, 927,
    };
    float gyro_stub[] = {
            0, 9, 0,
            2, 8, -1,
            1, 9, 1,
            1, 8, 1,
            0, 9, 0,
            2, 8, -1,
            1, 9, 1,
            1, 8, 1,
    };
    float mag_stub[] = {
            257, 6825, 6380,
            536, -424, 9504,
            257, 6825, 6380,
            536, -424, 9504,
            257, 6825, 6380,
            536, -424, 9504,
            257, 6825, 6380,
            536, -424, 9504,
    };

    usleep(100000); //delay the display speed

    if(if_end_deliver){
        return 0;
    }


    cur_time += sim_interval_ns;//forged timestamp

    switch (stub_index) {
        case 0:
        case 1:
        case 2:
        case 4:
        case 5:
        case 6:
            p_hwdata = (HW_DATA_UNION *) calloc(1, sizeof(HW_DATA_UNION));
            if (NULL == p_hwdata)
            {
                PERR("malloc fail");
                return 0;
            }

            p_hwdata->id = SENSOR_TYPE_ACCELEROMETER;
            p_hwdata->x = acc_stub[stub_index*3];
            p_hwdata->y = acc_stub[stub_index*3 + 1];
            p_hwdata->z = acc_stub[stub_index*3 + 2];
            p_hwdata->timestamp = cur_time;

            hw_remap_sensor_data(&(p_hwdata->x), &(p_hwdata->y), &(p_hwdata->z), g_place_a);

            ret = bstsensor->tmplist_hwcntl_acclraw->list_add_rear((void *) p_hwdata);
            if (ret)
            {
                PERR("list_add_rear() fail, ret = %d", ret);
                if(-1 == ret){
                    free(p_hwdata);
                }
            }

            p_hwdata = (HW_DATA_UNION *) calloc(1, sizeof(HW_DATA_UNION));
            if (NULL == p_hwdata)
            {
                PERR("malloc fail");
                return 0;
            }

            p_hwdata->id = SENSOR_TYPE_GYROSCOPE_UNCALIBRATED;
            p_hwdata->x = gyro_stub[stub_index*3];
            p_hwdata->y = gyro_stub[stub_index*3 + 1];
            p_hwdata->z = gyro_stub[stub_index*3 + 2];
            p_hwdata->timestamp = cur_time;

            hw_remap_sensor_data(&(p_hwdata->x), &(p_hwdata->y), &(p_hwdata->z), g_place_g);

            ret = bstsensor->tmplist_hwcntl_gyroraw->list_add_rear((void *) p_hwdata);
            if (ret)
            {
                PERR("list_add_rear() fail, ret = %d", ret);
                if(-1 == ret){
                    free(p_hwdata);
                }
            }

            break;

        case 3:
        case 7:
            p_hwdata = (HW_DATA_UNION *) calloc(1, sizeof(HW_DATA_UNION));
            if (NULL == p_hwdata)
            {
                PERR("malloc fail");
                return 0;
            }

            p_hwdata->id = SENSOR_TYPE_ACCELEROMETER;
            p_hwdata->x = acc_stub[stub_index*3];
            p_hwdata->y = acc_stub[stub_index*3 + 1];
            p_hwdata->z = acc_stub[stub_index*3 + 2];
            p_hwdata->timestamp = cur_time;

            hw_remap_sensor_data(&(p_hwdata->x), &(p_hwdata->y), &(p_hwdata->z), g_place_a);

            ret = bstsensor->tmplist_hwcntl_acclraw->list_add_rear((void *) p_hwdata);
            if (ret)
            {
                PERR("list_add_rear() fail, ret = %d", ret);
                if(-1 == ret){
                    free(p_hwdata);
                }
            }

            p_hwdata = (HW_DATA_UNION *) calloc(1, sizeof(HW_DATA_UNION));
            if (NULL == p_hwdata)
            {
                PERR("malloc fail");
                return 0;
            }

            p_hwdata->id = SENSOR_TYPE_GYROSCOPE_UNCALIBRATED;
            p_hwdata->x = gyro_stub[stub_index*3];
            p_hwdata->y = gyro_stub[stub_index*3 + 1];
            p_hwdata->z = gyro_stub[stub_index*3 + 2];
            p_hwdata->timestamp = cur_time;

            hw_remap_sensor_data(&(p_hwdata->x), &(p_hwdata->y), &(p_hwdata->z), g_place_g);

            ret = bstsensor->tmplist_hwcntl_gyroraw->list_add_rear((void *) p_hwdata);
            if (ret)
            {
                PERR("list_add_rear() fail, ret = %d", ret);
                if(-1 == ret){
                    free(p_hwdata);
                }
            }

            p_hwdata = (HW_DATA_UNION *) calloc(1, sizeof(HW_DATA_UNION));
            if (NULL == p_hwdata)
            {
                PERR("malloc fail");
                return 0;
            }

            p_hwdata->id = SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED;
            p_hwdata->x = mag_stub[stub_index*3];
            p_hwdata->y = mag_stub[stub_index*3 + 1];
            p_hwdata->z = mag_stub[stub_index*3 + 2];
            p_hwdata->timestamp = cur_time;

            hw_remap_sensor_data(&(p_hwdata->x), &(p_hwdata->y), &(p_hwdata->z), g_place_m);

            ret = bstsensor->tmplist_hwcntl_magnraw->list_add_rear((void *) p_hwdata);
            if (ret)
            {
                PERR("list_add_rear() fail, ret = %d", ret);
                if(-1 == ret){
                    free(p_hwdata);
                }
            }

            break;
        default:
            break;
    }

    stub_index++;

#if 1
    if (bstsensor->tmplist_hwcntl_acclraw->list_len
            + bstsensor->tmplist_hwcntl_gyroraw->list_len
            + bstsensor->tmplist_hwcntl_magnraw->list_len)
    {
        pthread_mutex_lock(&(bstsensor->shmem_hwcntl.mutex));

        ret = bstsensor->shmem_hwcntl.p_list->list_mount_rear(bstsensor->tmplist_hwcntl_acclraw);
        if(ret)
        {
            PWARN("list mount fail, ret = %d", ret);
        }
        ret = bstsensor->shmem_hwcntl.p_list->list_mount_rear(bstsensor->tmplist_hwcntl_gyroraw);
        if(ret)
        {
            PWARN("list mount fail, ret = %d", ret);
        }
        ret = bstsensor->shmem_hwcntl.p_list->list_mount_rear(bstsensor->tmplist_hwcntl_magnraw);
        if(ret)
        {
            PWARN("list mount fail, ret = %d", ret);
        }

        pthread_cond_signal(&(bstsensor->shmem_hwcntl.cond));
        pthread_mutex_unlock(&(bstsensor->shmem_hwcntl.mutex));
    }
#endif

    if(8 == stub_index)
    {
        /**Stop delivering stub raw data, send a special flush event to notify main thread to end
         * but before that wait a moment to let main thread can handle all events*/
        usleep(500000);
        (void) bstsensor->send_flush_event(0xFFFFFFFF);
        if_end_deliver = 1;
    }

    return 0;
}



static void ap_show_ver()
{
    const char* accl_chip_name = NULL;
    const char* gyro_chip_name = NULL;
    const char* magn_chip_name = NULL;
    const char* solution_name = NULL;
    char data_log_buf[256] = { 0 };

    switch (accl_chip) {
        case ACC_CHIP_BMI160:
            accl_chip_name = "BMI160";
            break;
        case ACC_CHIP_BMA2x2:
            accl_chip_name = "BMA2x2";
            break;
    }

    switch (gyro_chip) {
        case GYR_CHIP_BMI160:
            gyro_chip_name = "BMI160";
            break;
        case GYR_CHIP_BMG160:
            gyro_chip_name = "BMG160";
            break;
    }

    switch (magn_chip) {
        case MAG_CHIP_BMI160:
            magn_chip_name = "BMI160_Aux";
            break;
        case MAG_CHIP_BMM150:
            magn_chip_name = "BMM150";
            break;
        case MAG_CHIP_AKM09911:
            magn_chip_name = "AKM09911";
            break;
        case MAG_CHIP_AKM09912:
            magn_chip_name = "AKM09912";
            break;
        case MAG_CHIP_AKM09915:
            magn_chip_name = "AKM09915";
            break;
        case MAG_CHIP_AKM09916:
            magn_chip_name = "AKM09916";
            break;
        case MAG_CHIP_YAS532:
            magn_chip_name = "YAS532";
            break;
        case MAG_CHIP_YAS537:
            magn_chip_name = "YAS537";
            break;
        case MAG_CHIP_MMC:
            magn_chip_name = "MMC";
            break;
    }

    solution_name = "NO_FUSION";

    PNOTE("\n **************************\n \
* HAL version: %d.%d.%d.%d\n \
* HAL commit id: %s\n \
* build time: %s, %s\n \
* solution type: %s\n \
* accl_chip: %s\n \
* magn_chip: %s\n \
* gyro_chip: %s\n \
**************************",
            HAL_ver[0], HAL_ver[1], HAL_ver[2], HAL_ver[3], HAL_commit_id,
            __DATE__, __TIME__, solution_name, accl_chip_name, magn_chip_name, gyro_chip_name);

    if(bsx_datalog){
        sprintf(data_log_buf, "<component>HAL version: %d.%d.%d.%d</component>\n",
                HAL_ver[0], HAL_ver[1], HAL_ver[2], HAL_ver[3]);
        bsx_datalog_algo(data_log_buf);
        sprintf(data_log_buf, "<component>commit id: %s</component>\n", HAL_commit_id);
        bsx_datalog_algo(data_log_buf);
        sprintf(data_log_buf, "<component>build time: %s, %s</component>\n",
                __DATE__, __TIME__);
        bsx_datalog_algo(data_log_buf);
        sprintf(data_log_buf, "<component>solution type: %s</component>\n\n", solution_name);
        bsx_datalog_algo(data_log_buf);

        if(accl_chip_name){
            sprintf(data_log_buf, "<device name=\"%s\" bus=\"IIC\"> </device>\n", accl_chip_name);
            bsx_datalog_algo(data_log_buf);
        }

        if(magn_chip_name){
            sprintf(data_log_buf, "<device name=\"%s\" bus=\"\" > </device>\n", magn_chip_name);
            bsx_datalog_algo(data_log_buf);
        }

        if(gyro_chip_name){
            sprintf(data_log_buf, "<device name=\"%s\" bus=\"IIC\"> </device>\n", gyro_chip_name);
            bsx_datalog_algo(data_log_buf);
        }

        sprintf(data_log_buf, "\n");
        bsx_datalog_algo(data_log_buf);
    }

    return;
}

#if !defined(UNIT_TEST_ACTIVE)
static void driver_show_ver(const char* base_path)
{
    char path[64] = {0};
    char driver_info[64] = {0};
    FILE *fp;
    int ret;

    if(NULL == base_path)
    {
        PWARN("base_path is invalid");
        return;
    }

    sprintf(path, "%s/driver_version", base_path);

    fp = fopen(path, "r");
    if (NULL == fp)
    {
        PWARN("read driver version failed");
        return;
    }

    ret = fread(driver_info, 1, sizeof(driver_info), fp);
    fclose(fp);
    if(ret <= 0)
    {
        PERR("fread fail, errno = %d(%s)", errno, strerror(errno));
    }

    PNOTE("\n **************************\n \
* %s \
**************************", driver_info);

    return;
}


static int32_t ap_hwcntl_init_ACC()
{
    int32_t ret = 0;
    const char *accl_device_name = NULL;
    int acc_input_num = 0;

    if(ACC_CHIP_BMI160 == accl_chip)
    {
        accl_device_name = "bmi160";
    }else if(ACC_CHIP_BMA2x2 == accl_chip)
    {
        accl_device_name = "bma2x2";
    }

    open_input_by_name(accl_device_name, &hw_event_fd, &acc_input_num);
    if (-1 == hw_event_fd)
    {
        PERR("Failed to open input event\n");
        return -ENODEV;
    }
    if(0 == hardware_motion || ACC_CHIP_BMI160 != accl_chip)
    {
        close(hw_event_fd);
        hw_event_fd = -1;
    }

    PDEBUG("acc input_num = %d", acc_input_num);

    if(ACC_CHIP_BMI160 == accl_chip)
    {
        sprintf(bmi_input_dir_name, "/sys/class/input/input%d", acc_input_num);
        driver_show_ver(bmi_input_dir_name);

    }else if(ACC_CHIP_BMA2x2 == accl_chip)
    {
        sprintf(acc_input_dir_name, "/sys/class/input/input%d", acc_input_num);
        driver_show_ver(acc_input_dir_name);
    }


    if(ACC_CHIP_BMI160 == accl_chip)
    {
        ret = 0;
        /*set BMI160 RANGE*/
        switch(accl_range){
            case ACC_CHIP_RANGCONF_4G:
                ret += wr_sysfs_oneint("acc_range", bmi_input_dir_name, BMI160_ACCEL_RANGE_4G);
                BMI160_acc_resl *= 2;
                break;
            case ACC_CHIP_RANGCONF_8G:
                ret += wr_sysfs_oneint("acc_range", bmi_input_dir_name, BMI160_ACCEL_RANGE_8G);
                BMI160_acc_resl *= 4;
                break;
            case ACC_CHIP_RANGCONF_16G:
                ret += wr_sysfs_oneint("acc_range", bmi_input_dir_name, BMI160_ACCEL_RANGE_16G);
                BMI160_acc_resl *= 8;
                break;
            default:
                break;
        }
    }else if(ACC_CHIP_BMA2x2 == accl_chip)
    {
        switch(accl_range){
            case ACC_CHIP_RANGCONF_4G:
                ret += wr_sysfs_oneint("range", acc_input_dir_name, BMA2X2_RANGE_4G);
                BMA255_acc_resl *= 2;
                break;
            case ACC_CHIP_RANGCONF_8G:
                ret += wr_sysfs_oneint("range", acc_input_dir_name, BMA2X2_RANGE_8G);
                BMA255_acc_resl *= 4;
                break;
            case ACC_CHIP_RANGCONF_16G:
                ret += wr_sysfs_oneint("range", acc_input_dir_name, BMA2X2_RANGE_16G);
                BMA255_acc_resl *= 8;
                break;
            default:
                break;
        }
    }

    return 0;
}


static int32_t ap_hwcntl_init_GYRO()
{
    const char *gyro_device_name = NULL;
    int gyr_input_fd = -1;
    int gyr_input_num = 0;

    if(GYR_CHIP_BMI160 == gyro_chip)
    {
        /*Nothing to do*/
    }else if(GYR_CHIP_BMG160 ==  gyro_chip)
    {
        gyro_device_name = "bmg160";

        open_input_by_name(gyro_device_name, &gyr_input_fd, &gyr_input_num);
        if (-1 == gyr_input_fd)
        {
            PERR("Failed to open input event\n");
            return -ENODEV;
        }
        close(gyr_input_fd);

        PDEBUG("gyr input_num = %d", gyr_input_num);
        sprintf(gyr_input_dir_name, "/sys/class/input/input%d", gyr_input_num);

    }

    return 0;

}


static int32_t ap_hwcntl_init_MAGN()
{
    const char *magn_device_name = NULL;
    int magn_input_fd = -1;
    int magn_input_num = 0;

    /* Find the device requested */
    if(MAG_CHIP_BMI160 == magn_chip)
    {
        /*Nothing to do*/
    }else if(MAG_CHIP_BMM150 == magn_chip ||
            MAG_CHIP_AKM09912 == magn_chip || MAG_CHIP_AKM09911 == magn_chip ||
            MAG_CHIP_AKM09915 == magn_chip || MAG_CHIP_AKM09916 == magn_chip ||
            MAG_CHIP_YAS537 ==  magn_chip || MAG_CHIP_YAS532 ==  magn_chip || MAG_CHIP_MMC == magn_chip)
    {
        if(MAG_CHIP_BMM150 == magn_chip){
            magn_device_name = "bmm050";
        }else if(MAG_CHIP_AKM09912 == magn_chip){
            magn_device_name = "akm09912";
        }else if(MAG_CHIP_AKM09911 == magn_chip){
            magn_device_name = "akm09911";
        }else if(MAG_CHIP_AKM09915 == magn_chip){
            magn_device_name = "akm09915";
        }else if(MAG_CHIP_AKM09916 == magn_chip){
            magn_device_name = "akm09916";
        }else if(MAG_CHIP_YAS537 ==  magn_chip){
            magn_device_name = "yas537";
        }else if( MAG_CHIP_YAS532 ==  magn_chip){
            magn_device_name = "yas532";
        }else if(MAG_CHIP_MMC == magn_chip){
            magn_device_name = "mmc3524x";
        }

        open_input_by_name(magn_device_name, &magn_input_fd, &magn_input_num);
        if (-1 == magn_input_fd)
        {
            PERR("Failed to open input event\n");
            return -ENODEV;
        }
        close(magn_input_fd);

        PDEBUG("magn_input_num = %d", magn_input_num);
        sprintf(mag_input_dir_name, "/sys/class/input/input%d", magn_input_num);
    }

    return 0;

}
#endif


int32_t hwcntl_init(BstSensor *bstsensor)
{
    int32_t ret = 0;

    ap_show_ver();

    bstsensor->pfun_get_sensorlist = ap_get_sensorlist;
    bstsensor->pfun_activate = ap_activate;
    bstsensor->pfun_batch = ap_batch;
    bstsensor->pfun_flush = ap_flush;

#if !defined(UNIT_TEST_ACTIVE)
    bstsensor->pfun_hw_deliver_sensordata = hwcntl_deliver_sensordata;

    ret = ap_hwcntl_init_ACC();
    ret = ap_hwcntl_init_GYRO();
    ret = ap_hwcntl_init_MAGN();

    poll_timer_fd = timerfd_create(CLOCK_MONOTONIC, TFD_NONBLOCK);
    if(-1 == poll_timer_fd)
    {
        PERR("Failed to create timer\n");
        return -ENODEV;
    }

#else
    bstsensor->pfun_hw_deliver_sensordata = UNITTEST_hw_deliver_sensordata;
#endif


    return ret;
}

