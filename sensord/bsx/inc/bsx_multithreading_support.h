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

/*!
 * @defgroup bsx_multithreading_support
 * @brief
 * @{*/


#ifndef BSX_MULTITHREADING_SUPPORT_H_
#define BSX_MULTITHREADING_SUPPORT_H_


/********************************************************/
/* header includes */
#include "bsx_datatypes.h"


/********************************************************/
/* function prototype declarations */


/* forward definition of the documentation */
/*! @def BSX_MT_TRIGGER_THREAD(x)
 * @brief Place holder macro for a call-back function to push a frame to a thread message queue
 *
 * @param[in] x Identifier of the thread
 */

/*! @def BSX_ENTER_CRITICAL_SECTION()
 * @brief Place holder macro for call-back function to enter a critical code section
 */

/*! @def BSX_LEAVE_CRITICAL_SECTION()
 * @brief Place holder macro for call-back function to leave a critical code section
 */


#ifdef ENABLE_MULTITHREADING_CALIBRATION

// TODO remove this ugly definition for multi-threading emulation and use/implement the generic Ladon MT concept
/*! @brief Maximum size of the buffer to store samples for emulation of queues */
#define BSX_MULTITHREADING_BUFFER_MAX (100)
/*! @brief Enter critical code section by disabling all interrupts
 *
 * Callback function to integrate BSX with an operating system to avoid race conditions when accessing
 * common memory among threads, especially memory accesses from different threads.
 */
void bsx_enter_critical_section(void);

/*! @brief Leave critical code section by re-enabling all previous interrupts
 *
 * Callback function to integrate BSX with an operating system to avoid race conditions when accessing
 * common memory among threads, especially memory accesses from different threads.
 */
void bsx_leave_critical_section(void);

/*! @brief Signals to the OS that a thread shall be executed
 *
 * Callback function to integrate with an operating system to signal that an extra thread needs to be run
 *
 * @param[in] x Identifier of the thread
 */
void bsx_mt_trigger_thread(uint8_t x);


#define BSX_ENTER_CRITICAL_SECTION()    bsx_enter_critical_section()
#define BSX_LEAVE_CRITICAL_SECTION()    bsx_leave_critical_section()

#define BSX_MT_TRIGGER_THREAD(x)        bsx_mt_trigger_thread(x)

#else

#define BSX_ENTER_CRITICAL_SECTION()
#define BSX_LEAVE_CRITICAL_SECTION()

#define BSX_MT_TRIGGER_THREAD(x)

#endif


#endif /* BSX_MULTITHREADING_SUPPORT_H_ */

/** @}*/


