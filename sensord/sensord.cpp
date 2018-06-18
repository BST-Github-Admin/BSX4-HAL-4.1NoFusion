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
 * @file         sensord.cpp
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

#include <unistd.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <errno.h>
#include <sys/types.h>
#include <signal.h>

#include "BoschSensor.h"
#include "sensord.h"

#include "sensord_pltf.h"
#include "sensord_cfg.h"
#include "sensord_algo.h"
#include "sensord_hwcntl.h"
#include "axis_remap.h"

#include "util_misc.h"


void BoschSensor::sensord_read_rawdata()
{
    int ret;
    HW_DATA_UNION *p_hwdata;

    pthread_mutex_lock(&(shmem_hwcntl.mutex));

    if(0 == shmem_hwcntl.p_list->list_len)
    {
        ret = pthread_cond_wait(&(shmem_hwcntl.cond), &(shmem_hwcntl.mutex));
        if (ret)
        {
            pthread_mutex_unlock(&(shmem_hwcntl.mutex));
            return;
        }
    }
    /*	testing result and code investigation shows that even time out,
     pthread_cond_timedwait() will wait on calling pthread_mutex_lock(),
     so when return from pthread_cond_timedwait(), hwcntl may have sent
     the signal and unlock the pthread mutex at the same time occasionally,
     so check on share memory is still needed.
     When runs here, it means:
     1. 0 == ret, thread is woke up
     2. ETIMEDOUT == ret, but meanwhile hwcntl have sent the signal
     */

    while (shmem_hwcntl.p_list->list_len)
    {
        shmem_hwcntl.p_list->list_get_headdata((void **) &p_hwdata);

        switch (p_hwdata->id)
        {

            case SENSOR_TYPE_ACCELEROMETER:
                ret = tmplist_sensord_acclraw->list_add_rear((void *) p_hwdata);
                if (ret)
                {
                    PERR("list_add_rear() fail, ret = %d", ret);
                    if(-1 == ret){
                        free(p_hwdata);
                    }
                }
                break;
            case SENSOR_TYPE_GYROSCOPE_UNCALIBRATED:
                ret = tmplist_sensord_gyroraw->list_add_rear((void *) p_hwdata);
                if (ret)
                {
                    PERR("list_add_rear() fail, ret = %d", ret);
                    if(-1 == ret){
                        free(p_hwdata);
                    }
                }
                break;
            case SENSOR_TYPE_MAGNETIC_FIELD_UNCALIBRATED:
                ret = tmplist_sensord_magnraw->list_add_rear((void *) p_hwdata);
                if (ret)
                {
                    PERR("list_add_rear() fail, ret = %d", ret);
                    if(-1 == ret){
                        free(p_hwdata);
                    }
                }
                break;
        }
    }

    pthread_mutex_unlock(&(shmem_hwcntl.mutex));

    return;
}

void BoschSensor::sensord_deliver_event(sensors_event_t *p_event)
{
    int32_t ret;
    /*deliver up*/
    ret = write(HALpipe_fd[1], p_event, sizeof(sensors_event_t));
    if(ret < 0){
        PERR("deliver event fail, errno = %d(%s)", errno, strerror(errno));
    }

    free(p_event);

    return;
}


/**
 *
 * @param sensor_id
 * @return
 */
int BoschSensor::send_flush_event(int32_t sensor_id)
{
    sensors_meta_data_event_t *p_event;
    int32_t ret;

    p_event = (sensors_meta_data_event_t *) calloc(1, sizeof(sensors_meta_data_event_t));
    if (NULL == p_event)
    {
        PWARN("calloc fail");
        return -1;
    }

    p_event->version = META_DATA_VERSION;
    p_event->type = SENSOR_TYPE_META_DATA;
    p_event->meta_data.what = META_DATA_FLUSH_COMPLETE;
    p_event->meta_data.sensor = sensor_id;

    ret = write(HALpipe_fd[1], p_event, sizeof(sensors_meta_data_event_t));
    if(ret < 0){
        PERR("send flush echo fail, errno = %d(%s)", errno, strerror(errno));
    }

    free(p_event);

    return 0;
}


/**
 * Only used in AP solution
 */
void *sensord_main(void *arg)
{
    BoschSensor *bosch_sensor = reinterpret_cast<BoschSensor *>(arg);
    int ret = 0;

    while (1)
    {
        bosch_sensor->sensord_read_rawdata();
        sensord_algo_process(bosch_sensor);
    }

    //should not run here
    pthread_exit((void *) &ret);

    return NULL;
}

void sensord_sighandler(int signo, siginfo_t *sig_info, void *ctx)
{
    (void) sig_info;
    (void) ctx;
    int ret = 0;

    if (SIGTERM == signo)
    {
        pthread_exit((void *) &ret);
    }

    return;
}

