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
 * @file         sensord_cfg.cpp
 * @date         "Tue Jan 26 11:14:07 2016 +0800"
 * @commit       "c6abe62"
 *
 * @brief
 *
 * @detail
 *
 */

#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <fcntl.h>

#include "BstSensor.h"
#include "sensord_cfg.h"
#include "sensord_pltf.h"
#include "sensord_hwcntl.h"
#include "util_misc.h"

/* Maximum line length for any line in INI file. */
#define INI_MAX_LINE 200

#define MAX_SECTION 50

#define BSTHAL_CFG_FILE  (PATH_DIR_SENSOR_STORAGE "/bst_hal_cfg.txt")

#define SOLUTION_SEC		"SOLUTION"
#define SOLUTION_NAME		"solution"

#define DRIVER_SEC         "DRIVER"
#define ACC_CHIP_NAME      "acceleration_chip"
#define GYR_CHIP_NAME      "gyroscope_chip"
#define MAG_CHIP_NAME      "magnetic_chip"
#define ACC_RANGE_NAME      "acceleration_range"

#define AXIS_REMAP_SEC			"AXIS_REMAP"
#define AXIS_REMAP_A_NAME		"accl"
#define AXIS_REMAP_G_NAME		"gyro"
#define AXIS_REMAP_M_NAME		"magn"

#define DEBUG_SEC	"DEBUG"
#define TRACE_LEVEL_NAME   "trace_level"
#define LOGCAT_NAME        "logcat"
#define ALGOPASS_NAME       "algo_pass"
#define DATA_LOG_NAME		"data_log_enable"
#define BSX_DATALOG_NAME   "bsx_data_log"
#define SENSORS_MASK_NAME  "sensors_mask"
#define AMSH_INTR_PIN_NAME	"amsh_intr_pin"
#define AMSH_CALIB_NAME	"amsh_calibration"

int g_place_a = 0;
int g_place_m = 0;
int g_place_g = 0;
int solution_type = SOLUTION_MDOF;
int accl_chip = ACC_CHIP_BMI160;
int gyro_chip = GYR_CHIP_BMI160;
int magn_chip = MAG_CHIP_BMI160;
int accl_range = ACC_CHIP_RANGCONF_2G;
int algo_pass = 0;
int amsh_intr_pin = 0;
int amsh_calibration = 0;
int data_log = 0;
int bsx_datalog = 0;
int trace_level = 0x1C; //NOTE + ERR + WARN
int trace_to_logcat = 0;
long long unsigned int sensors_mask = 0;

/* Strip right whitespaces. */
static char* rstrip(char *str)
{

    char *end = str + strlen(str) - 1;

    while (end >= str && isspace((int) (*end)))
    {
        *end = 0;
        end--;
    }

    return str;
}

/* Strip left whitespaces. */
static char* lskip(char *str)
{

    while (*str && isspace((int) (*str)))
    {
        str++;
    }

    return str;
}

static void parse_file(FILE* file,
        void (*handler)(const char*, const char*, const char*))
{

    char line[INI_MAX_LINE + 1];
    char section[MAX_SECTION + 1] = { 0 };
    char* start;
    char* end;
    char* name;
    char* value;
    int lineno = 0;

    /* Scan through file line by line */
    while (NULL != fgets(line, INI_MAX_LINE, file))
    {

        start = lskip(rstrip(line));

        /* blank line or comments */
        if (0 == *start || ';' == *start || '#' == *start)
        {
            lineno++;
            continue;
        }

        /* [section] */
        if (*start == '[')
        {

            end = strchr(start, ']');
            if (end)
            {
                *end = '\0';
                strncpy(section, start + 1, MAX_SECTION);
            }
            else
            {
                PERR("line: %d: No ']' found on section line", lineno);
            }

            lineno++;
            continue;
        }

        /* Except that, should be a "name = value" pair */
        end = strchr(start, '=');
        if (end)
        {

            *end = '\0';
            name = rstrip(start);
            value = lskip(end + 1);
            handler(section, name, value);
        }
        else
        {
            PERR("invalid in line %d", lineno);
        }

        lineno++;
    }

    return;
}

static void parse_handler(const char* section,
        const char* name,
        const char* value)
{
#define MATCH(s, n) strcmp(section, s) == 0 && strcmp(name, n) == 0

    if (MATCH(SOLUTION_SEC, SOLUTION_NAME))
    {
        solution_type = atoi(value);
    }
    else if (MATCH(DRIVER_SEC, ACC_CHIP_NAME))
    {
        accl_chip = atoi(value);
    }else if (MATCH(DRIVER_SEC, GYR_CHIP_NAME))
    {
        gyro_chip = atoi(value);
    }
    else if (MATCH(DRIVER_SEC, MAG_CHIP_NAME))
    {
        magn_chip = atoi(value);
    }
    else if (MATCH(DRIVER_SEC, ACC_RANGE_NAME))
    {
        accl_range = atoi(value);
    }else if (MATCH(AXIS_REMAP_SEC, AXIS_REMAP_A_NAME))
    {
        g_place_a = atoi(value);
    }
    else if (MATCH(AXIS_REMAP_SEC, AXIS_REMAP_G_NAME))
    {
        g_place_g = atoi(value);
    }
    else if (MATCH(AXIS_REMAP_SEC, AXIS_REMAP_M_NAME))
    {
        g_place_m = atoi(value);
    }
    else if (MATCH(DEBUG_SEC, ALGOPASS_NAME))
    {
        algo_pass = atoi(value);
    }
    else if (MATCH(DEBUG_SEC, DATA_LOG_NAME))
    {
        data_log = atoi(value);
    }else if(MATCH(DEBUG_SEC, BSX_DATALOG_NAME))
    {
        bsx_datalog = atoi(value);
    }
    else if(MATCH(DEBUG_SEC, TRACE_LEVEL_NAME))
    {
        sscanf(value, "0x%8x", &trace_level);
    }
    else if(MATCH(DEBUG_SEC, LOGCAT_NAME))
    {
        trace_to_logcat = atoi(value);
    }
    else if(MATCH(DEBUG_SEC, SENSORS_MASK_NAME))
    {
        sscanf(value, "0x%16llx", &sensors_mask);
    }
    else if (MATCH(DEBUG_SEC, AMSH_INTR_PIN_NAME))
    {
        amsh_intr_pin = atoi(value);
    }
    else if (MATCH(DEBUG_SEC, AMSH_CALIB_NAME))
    {
        amsh_calibration = atoi(value);
    }

    return;
}

void BstSensor::sensord_cfg_init()
{

    FILE* file;

    file = fopen(BSTHAL_CFG_FILE, "r");
    if (file)
    {
        parse_file(file, parse_handler);
        fclose(file);
    }
    else
    {
        PERR("config file %s does not exist, assuming default config", BSTHAL_CFG_FILE);
    }

    return;
}

