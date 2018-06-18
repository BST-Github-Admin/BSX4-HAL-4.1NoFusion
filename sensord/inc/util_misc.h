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
 * @file         util_misc.h
 * @date         "Wed Dec 23 14:29:46 2015 +0800"
 * @commit       "bbf5b88"
 *
 * @modification date         "Thu May 3 12:23:56 2018 +0100"
 *
 * @brief
 *
 * @detail
 *
 */

#ifndef __UTIL_MISC_H
#define __UTIL_MISC_H

#ifdef __cplusplus
extern "C"
{
#endif

#define ABS(x) ((x) > 0 ? (x) : -(x))

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(arr) ((int)(sizeof(arr) / sizeof((arr)[0])))
#endif

#ifndef OFFSET_OF
#define OFFSET_OF(type, member)\
	((size_t) &((type *)0)->member)
#endif

#ifndef CONTAINER_OF
#define CONTAINER_OF(ptr, type, member)\
	(type *)((char *)(ptr) - OFFSET_OF(type, member))
#endif

#define ARRAY_ELEMENTS(array)  (sizeof(array) / sizeof(array[0]))

#define FLOAT_EQU(x, y)\
	((x > y) ? ((x - y) < 0.000001) : ((y - x) < 0.000001))

#define EMIT_BUG() \
	do {\
		fprintf(stderr, "bug at file: %s line: %d\n", __FILE__, __LINE__);\
		*((unsigned char *)0) = 0;\
	} while (0);

#define SET_VALUE_BITS(x,val,start,bits)\
	(((x) & ~(((1 << (bits)) - 1) << (start))) | ((val) << (start)))

#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) < (b) ? (a) : (b))

#define SWAP(type, a, b)\
	do {\
		type t;\
		t = a;\
		a = b;\
		b = t;\
	} while (0);

#define IS_BIT_SET(value, bit)  (value & (1 << bit))

#define UNUSED_PARAM(param) ((void)(param))
#define __unused__ __attribute__((unused))

    void get_token_pairs(const char *psz_line, char seperator,
            char *ptoken_l, char *ptoken_r);
    int get_gcd(int a, int b);
    int32_t get_nearest_divisible_int(uint32_t a, uint32_t b);
    /* get number of 1s (set bits) in a u32 integer */
    int get_num_set_bits_u32(uint32_t n);

    void debug_mem(unsigned char *addr, uint32_t len);
    int bosch_pthread_mutex_timedlock(pthread_mutex_t *mutex, uint32_t msecs);
    int bosch_pthread_cond_timedwait(pthread_cond_t *cond, pthread_mutex_t *mutex, uint32_t usecs);
//int bosch_pthread_sigqueue(pthread_t thread, int sig, const union sigval value);
    uint32_t sensord_popcount_32(uint32_t x);
    uint32_t sensord_popcount_64(uint64_t x);
    uint32_t sensord_popcount_less1(uint64_t x);

#ifdef __cplusplus
}
#endif

#endif
