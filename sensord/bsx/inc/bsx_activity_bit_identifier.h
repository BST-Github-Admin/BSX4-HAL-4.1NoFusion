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
 */

/*!@addtogroup bsx
 * @{*/

#ifndef __BSX_ACTIVITY_BIT_IDENTIFIER_H__
#define __BSX_ACTIVITY_BIT_IDENTIFIER_H__

#ifdef __cplusplus
extern "C"
{
#endif

/*! @brief BSX activity identifiers */
typedef enum
{
    BSX_ID_ACTIVITY_STILL           = (1), /*!< Still, similar to a static state */
    BSX_ID_ACTIVITY_WALK            = (2), /*!< Walking */
    BSX_ID_ACTIVITY_RUN             = (3), /*!< Running */
    BSX_ID_ACTIVITY_BIKE            = (4), /*!< On the bicycle */
    BSX_ID_ACTIVITY_INVEHICLE       = (5), /*!< In a vehicle or on a motor-bike */
    BSX_ID_ACTIVITY_TILTING         = (6), /*!< Tilt gesture */
    BSX_ID_ACTIVITY_VEHICLESTILL    = (7), /*!< In a non-moving vehicle */
    BSX_ID_ACTIVITY_MAX             = (16) /*!< Maximum possible value for an activity */
} bsx_activity_identifier_t;

/*! @brief convert activity identifier to bit mask used to encode its begin */
#define BSX_ID_ACTIVITY_START(x) (1 << ((x)-1))

/*! @brief convert activity identifier to bit mask used to encode its end */
#define BSX_ID_ACTIVITY_STOP(x) (1 << ((x)+15))

/*! @brief BSX activity bit identifiers */
typedef enum
{
    BSX_ID_ACTIVITY_BIT_NOCHANGE            = (0x00000000U),
    BSX_ID_ACTIVITY_BIT_STILL_START         = BSX_ID_ACTIVITY_START(BSX_ID_ACTIVITY_STILL),
    BSX_ID_ACTIVITY_BIT_WALK_START          = BSX_ID_ACTIVITY_START(BSX_ID_ACTIVITY_WALK),
    BSX_ID_ACTIVITY_BIT_RUN_START           = BSX_ID_ACTIVITY_START(BSX_ID_ACTIVITY_RUN),
    BSX_ID_ACTIVITY_BIT_BIKE_START          = BSX_ID_ACTIVITY_START(BSX_ID_ACTIVITY_BIKE),
    BSX_ID_ACTIVITY_BIT_INVEHICLE_START     = BSX_ID_ACTIVITY_START(BSX_ID_ACTIVITY_INVEHICLE),
    BSX_ID_ACTIVITY_BIT_TILTING_START       = BSX_ID_ACTIVITY_START(BSX_ID_ACTIVITY_TILTING),
    BSX_ID_ACTIVITY_BIT_VEHICLESTILL_START  = BSX_ID_ACTIVITY_START(BSX_ID_ACTIVITY_VEHICLESTILL),
    BSX_ID_ACTIVITY_BIT_STILL_STOP          = BSX_ID_ACTIVITY_STOP(BSX_ID_ACTIVITY_STILL),
    BSX_ID_ACTIVITY_BIT_WALK_STOP           = BSX_ID_ACTIVITY_STOP(BSX_ID_ACTIVITY_WALK),
    BSX_ID_ACTIVITY_BIT_RUN_STOP            = BSX_ID_ACTIVITY_STOP(BSX_ID_ACTIVITY_RUN),
    BSX_ID_ACTIVITY_BIT_BIKE_STOP           = BSX_ID_ACTIVITY_STOP(BSX_ID_ACTIVITY_BIKE),
    BSX_ID_ACTIVITY_BIT_INVEHICLE_STOP      = BSX_ID_ACTIVITY_STOP(BSX_ID_ACTIVITY_INVEHICLE),
    BSX_ID_ACTIVITY_BIT_TILTING_STOP        = BSX_ID_ACTIVITY_STOP(BSX_ID_ACTIVITY_TILTING),
    BSX_ID_ACTIVITY_BIT_VEHICLESTILL_STOP   = BSX_ID_ACTIVITY_STOP(BSX_ID_ACTIVITY_VEHICLESTILL)
} bsx_activity_bit_identifier_t;

#ifdef __cplusplus
}
#endif

#endif /*__BSX_ACTIVITY_BIT_IDENTIFIER_H__*/
/*! @}*/
