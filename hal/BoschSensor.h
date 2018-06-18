/*!
 * @section LICENSE
 *
 * (C) Copyright 2011~2015 Bosch Sensortec GmbH All Rights Reserved
 *
 * (C) Modification Copyright 2018 Robert Bosch Kft  All Rights Reserved
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
 * Disclaimer
 *
 * Common:
 *
 * Assessment of Products Returned from Field
 *
 * Returned products are considered good if they fulfill the specifications / 
 * test data for 0-mileage and field listed in this document.
 *
 * Engineering Samples
 * 
 * Engineering samples are marked with (e) or (E). Samples may vary from the
 * valid technical specifications of the series product contained in this
 * data sheet. Therefore, they are not intended or fit for resale to
 * third parties or for use in end products. Their sole purpose is internal
 * client testing. The testing of an engineering sample may in no way replace
 * the testing of a series product. Bosch assumes no liability for the use
 * of engineering samples. The purchaser shall indemnify Bosch from all claims
 * arising from the use of engineering samples.
 *
 * Intended use
 *
 * Provided that SMI130 is used within the conditions (environment, application,
 * installation, loads) as described in this TCD and the corresponding
 * agreed upon documents, Bosch ensures that the product complies with
 * the agreed properties. Agreements beyond this require
 * the written approval by Bosch. The product is considered fit for the intended
 * use when the product successfully has passed the tests
 * in accordance with the TCD and agreed upon documents.
 *
 * It is the responsibility of the customer to ensure the proper application
 * of the product in the overall system/vehicle.
 *
 * Bosch does not assume any responsibility for changes to the environment
 * of the product that deviate from the TCD and the agreed upon documents 
 * as well as all applications not released by Bosch
  *
 * The resale and/or use of products are at the purchaser’s own risk and 
 * responsibility. The examination and testing of the SMI130 
 * is the sole responsibility of the purchaser.
 *
 * The purchaser shall indemnify Bosch from all third party claims 
 * arising from any product use not covered by the parameters of 
 * this product data sheet or not approved by Bosch and reimburse Bosch 
 * for all costs and damages in connection with such claims.
 *
 * The purchaser must monitor the market for the purchased products,
 * particularly with regard to product safety, and inform Bosch without delay
 * of all security relevant incidents.
 *
 * Application Examples and Hints
 *
 * With respect to any application examples, advice, normal values
 * and/or any information regarding the application of the device,
 * Bosch hereby disclaims any and all warranties and liabilities of any kind,
 * including without limitation warranties of
 * non-infringement of intellectual property rights or copyrights
 * of any third party.
 * The information given in this document shall in no event be regarded 
 * as a guarantee of conditions or characteristics. They are provided
 * for illustrative purposes only and no evaluation regarding infringement
 * of intellectual property rights or copyrights or regarding functionality,
 * performance or error has been made.
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
 * @file         BoschSensor.h
 * @date         "Fri Feb 5 15:40:38 2016 +0800"
 * @commit       "666efb6"
 *
 * @modification date         "Thu May 3 12:23:56 2018 +0100"
 *
 * @brief
 *
 * @detail
 *
 */

#ifndef ANDROID_BST_SENSOR_H
#define ANDROID_BST_SENSOR_H

#include <stdio.h>
#include <stdint.h>
#include <sys/cdefs.h>
#include <signal.h>
#include <pthread.h>
#if !defined(PLTF_LINUX_ENABLED)
#include <hardware/sensors.h>
#else
#include "sensors.h"
#endif
#include "sensord_def.h"
#include "boschsimple_list.h"

typedef struct
{
    pthread_mutex_t mutex;
    pthread_cond_t cond;
    BoschSimpleList *p_list;
} SENSORD_SHARED_MEM;

class BoschSensor
{
public:
    static BoschSensor *getInstance();
    static void destroy();

    int activate(int handle, int enabled);
    int setDelay(int handle, int64_t ns);
    int read_events(sensors_event_t* data, int count);
    int batch(int handle, int flags, int64_t sampling_period_ns, int64_t max_report_latency_ns);
    int flush(int handle);
#if defined(SENSORS_DEVICE_API_VERSION_1_4)
    int inject_sensor_data(const sensors_event_t *data);
#endif
    uint32_t get_sensorlist(struct sensor_t const** p_sSensorList);
    void sensord_read_rawdata();
    void sensord_deliver_event(sensors_event_t *p_event);

    int send_flush_event(int32_t sensor_id);

    int (*pfun_activate)(int handle, int enabled);
    int (*pfun_batch)(int handle, int flags, int64_t sampling_period_ns, int64_t max_report_latency_ns);
    int (*pfun_flush)(BoschSensor *boschsensor, int handle);
    uint32_t (*pfun_get_sensorlist)(struct sensor_t const** p_sSensorList);
    uint32_t (*pfun_hw_deliver_sensordata)(BoschSensor *boschsensor);

    BoschSimpleList *tmplist_hwcntl_acclraw;
    BoschSimpleList *tmplist_hwcntl_gyroraw;
    BoschSimpleList *tmplist_hwcntl_magnraw;

    BoschSimpleList *tmplist_sensord_acclraw;
    BoschSimpleList *tmplist_sensord_gyroraw;
    BoschSimpleList *tmplist_sensord_magnraw;

    SENSORD_SHARED_MEM shmem_hwcntl;
    int HALpipe_fd[2];

private:
    BoschSensor();
    BoschSensor(const BoschSensor & other); //for cppcheck "noCopyConstructor"
    ~BoschSensor();

    static BoschSensor *instance;
    void sensord_cfg_init();

    pthread_t thread_sensord;
    pthread_t thread_hwcntl;
};

#endif  // ANDROID_BST_SENSOR_H
