/* 
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
#define BSX_OUTPUT_ID_ACCELERATION_PASSTHROUGH   (2)
#define BSX_CUSTOM_ID_ACCELERATION_PASSTHROUGH   (4)
#define BSX_OUTPUT_ID_ACCELERATION_RAW   (6)
#define BSX_OUTPUT_ID_ACCELERATION_CORRECTED   (8)
#define BSX_OUTPUT_ID_ACCELERATION_OFFSET   (10)
#define BSX_WAKEUP_ID_ACCELERATION_OFFSET   (182)
#define BSX_WAKEUP_ID_ACCELERATION_CORRECTED   (12)
#define BSX_WAKEUP_ID_ACCELERATION_RAW   (14)
#define BSX_CUSTOM_ID_ACCELERATION_CORRECTED   (16)
#define BSX_CUSTOM_ID_ACCELERATION_RAW   (18)
#define BSX_OUTPUT_ID_ANGULARRATE_PASSTHROUGH   (20)
#define BSX_CUSTOM_ID_ANGULARRATE_PASSTHROUGH   (22)
#define BSX_OUTPUT_ID_ANGULARRATE_RAW   (24)
#define BSX_OUTPUT_ID_ANGULARRATE_CORRECTED   (26)
#define BSX_OUTPUT_ID_ANGULARRATE_OFFSET   (28)
#define BSX_WAKEUP_ID_ANGULARRATE_OFFSET   (184)
#define BSX_WAKEUP_ID_ANGULARRATE_CORRECTED   (30)
#define BSX_WAKEUP_ID_ANGULARRATE_RAW   (32)
#define BSX_CUSTOM_ID_ANGULARRATE_CORRECTED   (34)
#define BSX_CUSTOM_ID_ANGULARRATE_RAW   (36)
#define BSX_OUTPUT_ID_MAGNETICFIELD_PASSTHROUGH   (38)
#define BSX_CUSTOM_ID_MAGNETICFIELD_PASSTHROUGH   (40)
#define BSX_OUTPUT_ID_MAGNETICFIELD_RAW   (42)
#define BSX_OUTPUT_ID_MAGNETICFIELD_CORRECTED   (44)
#define BSX_OUTPUT_ID_MAGNETICFIELD_OFFSET   (46)
#define BSX_WAKEUP_ID_MAGNETICFIELD_OFFSET   (186)
#define BSX_WAKEUP_ID_MAGNETICFIELD_CORRECTED   (48)
#define BSX_WAKEUP_ID_MAGNETICFIELD_RAW   (50)
#define BSX_CUSTOM_ID_MAGNETICFIELD_CORRECTED   (52)
#define BSX_CUSTOM_ID_MAGNETICFIELD_RAW   (54)
#define BSX_OUTPUT_ID_GRAVITY   (56)
#define BSX_WAKEUP_ID_GRAVITY   (58)
#define BSX_CUSTOM_ID_GRAVITY   (60)
#define BSX_OUTPUT_ID_LINEARACCELERATION   (62)
#define BSX_WAKEUP_ID_LINEARACCELERATION   (64)
#define BSX_CUSTOM_ID_LINEARACCELERATION   (66)
/** @brief NDOF or e-Compass */
#define BSX_OUTPUT_ID_ROTATION   (68)
/** @brief NDOF or e-Compass */
#define BSX_WAKEUP_ID_ROTATION   (70)
#define BSX_CUSTOM_ID_ROTATION   (72)
/** @brief no magnetic field */
#define BSX_OUTPUT_ID_ROTATION_GAME   (74)
/** @brief no magnetic field */
#define BSX_WAKEUP_ID_ROTATION_GAME   (76)
#define BSX_CUSTOM_ID_ROTATION_GAME   (78)
/** @brief no gyroscope, only e-Compass */
#define BSX_OUTPUT_ID_ROTATION_GEOMAGNETIC   (80)
/** @brief no gyroscope, only e-Compass */
#define BSX_WAKEUP_ID_ROTATION_GEOMAGNETIC   (82)
#define BSX_CUSTOM_ID_ROTATION_GEOMAGNETIC   (84)
/** @brief deprecated: NDOF or e-Compass computed from OUTPUT_ROTATION */
#define BSX_OUTPUT_ID_ORIENTATION   (86)
/** @brief deprecated: NDOF or e-Compass computed from WAKEUP_ROTATION */
#define BSX_WAKEUP_ID_ORIENTATION   (88)
#define BSX_CUSTOM_ID_ORIENTATION   (90)
#define BSX_OUTPUT_ID_FLIP_STATUS   (92)
#define BSX_CUSTOM_ID_FLIP_STATUS   (94)
#define BSX_OUTPUT_ID_TILT_STATUS   (96)
#define BSX_CUSTOM_ID_TILT_STATUS   (98)
#define BSX_OUTPUT_ID_STEPDETECTOR   (100)
#define BSX_WAKEUP_ID_STEPDETECTOR   (188)
#define BSX_CUSTOM_ID_STEPDETECTOR   (102)
#define BSX_OUTPUT_ID_STEPCOUNTER   (104)
#define BSX_WAKEUP_ID_STEPCOUNTER   (106)
#define BSX_CUSTOM_ID_STEPCOUNTER   (108)
#define BSX_OUTPUT_ID_SIGNIFICANTMOTION_STATUS   (110)
#define BSX_CUSTOM_ID_SIGNIFICANTMOTION_STATUS   (112)
#define BSX_OUTPUT_ID_WAKE_STATUS   (114)
#define BSX_CUSTOM_ID_WAKE_STATUS   (116)
#define BSX_OUTPUT_ID_GLANCE_STATUS   (118)
#define BSX_CUSTOM_ID_GLANCE_STATUS   (120)
#define BSX_OUTPUT_ID_PICKUP_STATUS   (122)
#define BSX_CUSTOM_ID_PICKUP_STATUS   (124)
#define BSX_OUTPUT_ID_ACTIVITY   (126)
#define BSX_CUSTOM_ID_ACTIVITY   (128)
#define BSX_OUTPUT_ID_PROPAGATION   (130)
#define BSX_OUTPUT_ID_POSITION_STEPS   (132)
#define BSX_OUTPUT_ID_WRIST_TILT_STATUS   (134)
#define BSX_CUSTOM_ID_WRIST_TILT_STATUS   (136)
#define BSX_OUTPUT_ID_DEVICE_ORIENTATION   (138)
#define BSX_WAKEUP_ID_DEVICE_ORIENTATION   (140)
#define BSX_CUSTOM_ID_DEVICE_ORIENTATION   (142)
#define BSX_OUTPUT_ID_POSE_6DOF   (144)
#define BSX_WAKEUP_ID_POSE_6DOF   (146)
#define BSX_CUSTOM_ID_POSE_6DOF   (148)
#define BSX_OUTPUT_ID_STATIONARY_DETECT   (150)
#define BSX_CUSTOM_ID_STATIONARY_DETECT   (152)
#define BSX_OUTPUT_ID_MOTION_DETECT   (154)
#define BSX_CUSTOM_ID_MOTION_DETECT   (156)
#define BSX_OUTPUT_ID_STANDBY_STATUS   (158)
#define BSX_OUTPUT_ID_ACCELERATION_STATUS   (160)
#define BSX_OUTPUT_ID_ACCELERATION_DYNAMIC   (162)
#define BSX_OUTPUT_ID_ANGULARRATE_STATUS   (164)
#define BSX_OUTPUT_ID_MAGNETICFIELD_STATUS   (166)
/** @brief dedicated debug output for the angular rate from M4G */
#define BSX_OUTPUT_ID_ANGULARRATE_M4G   (168)
/** @brief dedicated debug output for the angular rate from M4G */
#define BSX_WAKEUP_ID_ANGULARRATE_M4G   (170)
/*!
 * @}
 */ 

#endif /* BSX_VIRTUAL_SENSOR_IDENTIFIER_H_ */

/*! @}
 */
