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

/*! @addtogroup bsx
 * @{
 */

#ifndef BSX_VIRTUAL_SENSOR_IDENTIFIER_H_
#define BSX_VIRTUAL_SENSOR_IDENTIFIER_H_

/*! @brief Provides the enumeration containing virtual sensor identifier 
 *
 * @{
 */ 
#define  BSX_VIRTUAL_SENSOR_ID_INVALID   (0)
#define BSX_OUTPUT_ID_ACCELERATION_RAW   (2)
#define BSX_OUTPUT_ID_ACCELERATION_CORRECTED   (4)
#define BSX_OUTPUT_ID_ACCELERATION_OFFSET   (6)
#define BSX_WAKEUP_ID_ACCELERATION_CORRECTED   (8)
#define BSX_WAKEUP_ID_ACCELERATION_RAW   (10)
#define BSX_OUTPUT_ID_ANGULARRATE_RAW   (12)
#define BSX_OUTPUT_ID_ANGULARRATE_CORRECTED   (14)
#define BSX_OUTPUT_ID_ANGULARRATE_OFFSET   (16)
#define BSX_WAKEUP_ID_ANGULARRATE_CORRECTED   (18)
#define BSX_WAKEUP_ID_ANGULARRATE_RAW   (20)
#define BSX_OUTPUT_ID_MAGNETICFIELD_RAW   (22)
#define BSX_OUTPUT_ID_MAGNETICFIELD_CORRECTED   (24)
#define BSX_OUTPUT_ID_MAGNETICFIELD_OFFSET   (26)
#define BSX_WAKEUP_ID_MAGNETICFIELD_CORRECTED   (28)
#define BSX_WAKEUP_ID_MAGNETICFIELD_RAW   (30)
#define BSX_OUTPUT_ID_GRAVITY   (32)
#define BSX_WAKEUP_ID_GRAVITY   (34)
#define BSX_OUTPUT_ID_LINEARACCELERATION   (36)
#define BSX_WAKEUP_ID_LINEARACCELERATION   (38)
/** @brief NDOF, e-Compass or M4G */
#define BSX_OUTPUT_ID_ROTATION   (40)
/** @brief NDOF, e-Compass or M4G */
#define BSX_WAKEUP_ID_ROTATION   (42)
/** @brief no magnetic field */
#define BSX_OUTPUT_ID_ROTATION_GAME   (44)
/** @brief no magnetic field */
#define BSX_WAKEUP_ID_ROTATION_GAME   (46)
/** @brief no gyroscope, either e-Compass or M4G */
#define BSX_OUTPUT_ID_ROTATION_GEOMAGNETIC   (48)
/** @brief no gyroscope, either e-Compass or M4G */
#define BSX_WAKEUP_ID_ROTATION_GEOMAGNETIC   (50)
/** @brief deprecated: NDOF, e-Compass or M4G computed from OUTPUT_ROTATION */
#define BSX_OUTPUT_ID_ORIENTATION   (52)
/** @brief deprecated: NDOF, e-Compass or M4G computed from WAKEUP_ROTATION */
#define BSX_WAKEUP_ID_ORIENTATION   (54)
#define BSX_OUTPUT_ID_FLIP_STATUS   (62)
#define BSX_OUTPUT_ID_TILT_STATUS   (64)
#define BSX_OUTPUT_ID_STEPDETECTOR   (66)
#define BSX_OUTPUT_ID_STEPCOUNTER   (68)
#define BSX_WAKEUP_ID_STEPCOUNTER   (70)
#define BSX_OUTPUT_ID_SIGNIFICANTMOTION_STATUS   (72)
#define BSX_OUTPUT_ID_WAKE_STATUS   (74)
#define BSX_OUTPUT_ID_GLANCE_STATUS   (76)
#define BSX_OUTPUT_ID_PICKUP_STATUS   (78)
#define BSX_OUTPUT_ID_ACTIVITY   (80)
#define BSX_OUTPUT_ID_PROPAGATION   (82)
#define BSX_OUTPUT_ID_POSITION_STEPS   (84)
#define BSX_OUTPUT_ID_STANDBY_STATUS   (86)
#define BSX_OUTPUT_ID_ACCELERATION_STATUS   (90)
#define BSX_OUTPUT_ID_ACCELERATION_DYNAMIC   (92)
#define BSX_OUTPUT_ID_ANGULARRATE_STATUS   (94)
#define BSX_OUTPUT_ID_MAGNETICFIELD_STATUS   (96)
/** @brief dedicated debug output for the angular rate from M4G */
#define BSX_OUTPUT_ID_ANGULARRATE_M4G   (98)
/** @brief dedicated debug output for the angular rate from M4G */
#define BSX_WAKEUP_ID_ANGULARRATE_M4G   (100)
/*!
 * @}
 */ 

#endif /* BSX_VIRTUAL_SENSOR_IDENTIFIER_H_ */

/*! @}
 */
