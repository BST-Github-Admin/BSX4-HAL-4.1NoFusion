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
 * @file         util_misc.c
 * @date         "Mon Nov 9 15:19:30 2015 +0800"
 * @commit       "d4de1ba"
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
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/time.h>
#include <pthread.h>

#include "util_misc.h"

void get_token_pairs(const char *psz_line, char seperator, char *ptoken_l, char *ptoken_r)
{
    char ch;
    int i;
    int pos = -1;

    i = 0;
    while ('\0' != (ch = psz_line[i]))
    {
        if (ch == seperator)
        {
            pos = i;
            break;
        }
        else
        {
            ptoken_l[i] = psz_line[i];
        }
        i++;
    }
    ptoken_l[i] = '\0';

    if (-1 != pos)
    {
        i = pos + 1;
        while ('\0' != psz_line[i])
        {
            ptoken_r[i - pos - 1] = psz_line[i];
            i++;
        }
        ptoken_r[i - pos - 1] = '\0';
    }
    else
    {
        ptoken_r[0] = '\0';
    }
}

/*! get greatest common dividor
 */
int get_gcd(int a, int b)
{
    if (0 == b)
        return a;
    return get_gcd(b, a % b);
}

int32_t get_nearest_divisible_int(uint32_t a, uint32_t b)
{
    int32_t ret;

    if (0 == b)
    {
        /* error */
        return -1;
    }

    if (0 == a)
    {
        return (int32_t) b;
    }

    if (a <= b)
    {
        return (int32_t) a;
    }

    for (ret = b; ret > 0; ret--)
    {
        if (0 == (a % ret))
        {
            break;
        }
    }

    return ret;
}


void debug_mem(uint8_t *addr, uint32_t len)
{
    unsigned int i;
    unsigned int BYTES_PER_LINE = 16;

    for (i = 0; i < len; i++)
    {
        printf("%02x%c", addr[i], ((i + 1) % BYTES_PER_LINE) ? ' ' : '\n');
    }

    printf("\n+++++++++++++++++++++++++++++++++\n");
}


/*msecs is assumed to be in milliseconds */
int bosch_pthread_mutex_timedlock(pthread_mutex_t *mutex, uint32_t msecs)
{
#if 1
#define INTERVAL_USEC 1000
    useconds_t usecs;
    int ret = 0;

    usecs = msecs * 1000;
    while (usecs)
    {
        ret = pthread_mutex_trylock(mutex);
        if (0 == ret)
        {
            return 0;
        }

        usecs -= INTERVAL_USEC;
        usleep(INTERVAL_USEC);
    }

    return ret;

#else //So far pthread_mutex_timedlock is not provided in AOSP version
    struct timespec t_set;
    struct timeval time_now;
    time_t s_set;
    long us_set;

    gettimeofday(&time_now, NULL);
    s_set = time_now.tv_sec + msecs / 1000;
    us_set = time_now.tv_usec + (msecs%1000)*1000;
    //carry up
    s_set += us_set/1000000;
    us_set %= 1000000;

    t_set.tv_sec = s_set;
    t_set.tv_nsec = us_set * 1000;

    return pthread_mutex_timedlock(mutex, &t_set);
#endif
}

uint32_t sensord_popcount_32(uint32_t x)
{
    const uint32_t m1 = 0x55555555; //binary: 0101...
    const uint32_t m2 = 0x33333333; //binary: 00110011..
    const uint32_t m4 = 0x0f0f0f0f; //binary:  4 zeros,  4 ones ...
    const uint32_t m8 = 0x00ff00ff; //binary:  8 zeros,  8 ones ...
    const uint32_t m16 = 0x0000ffff; //binary: 16 zeros, 16 ones ...
    //const uint32_t hff = 0xffffffff; //binary: all ones
    //const uint32_t h01 = 0x01010101; //the sum of 256 to the power of 0,1,2,3...

    x = (x & m1) + ((x >> 1) & m1); //put count of each  2 bits into those  2 bits
    x = (x & m2) + ((x >> 2) & m2); //put count of each  4 bits into those  4 bits
    x = (x & m4) + ((x >> 4) & m4); //put count of each  8 bits into those  8 bits
    x = (x & m8) + ((x >> 8) & m8); //put count of each 16 bits into those 16 bits
    x = (x & m16) + ((x >> 16) & m16); //put count of each 32 bits into those 32 bits

    return x;
}

uint32_t sensord_popcount_64(uint64_t x)
{
    const uint64_t m1 = 0x5555555555555555; //binary: 0101...
    const uint64_t m2 = 0x3333333333333333; //binary: 00110011..
    const uint64_t m4 = 0x0f0f0f0f0f0f0f0f; //binary:	4 zeros,  4 ones ...
    const uint64_t m8 = 0x00ff00ff00ff00ff; //binary:	8 zeros,  8 ones ...
    const uint64_t m16 = 0x0000ffff0000ffff; //binary: 16 zeros, 16 ones ...
    const uint64_t m32 = 0x00000000ffffffff; //binary: 32 zeros, 32 ones
    //const uint64_t hff = 0xffffffffffffffff; //binary: all ones
    //const uint64_t h01 = 0x0101010101010101; //the sum of 256 to the power of 0,1,2,3...

    x = (x & m1) + ((x >> 1) & m1); //put count of each  2 bits into those  2 bits
    x = (x & m2) + ((x >> 2) & m2); //put count of each  4 bits into those  4 bits
    x = (x & m4) + ((x >> 4) & m4); //put count of each  8 bits into those  8 bits
    x = (x & m8) + ((x >> 8) & m8); //put count of each 16 bits into those 16 bits
    x = (x & m16) + ((x >> 16) & m16); //put count of each 32 bits into those 32 bits
    x = (x & m32) + ((x >> 32) & m32); //put count of each 64 bits into those 64 bits

    return x;
}

/* As Wegner (1960) described, the bitwise and of x with x-1 differs
 from x only in zeroing out the least significant nonzero bit: subtracting 1
 changes the rightmost string of 0s to 1s, and changes the rightmost 1 to a 0.
 If x originally had n bits that were 1, then after only n iterations of this operation,
 x will be reduced to zero */
uint32_t sensord_popcount_less1(uint64_t x)
{
    uint32_t count;

    for (count = 0; x; count++)
        x &= x - 1;

    return count;
}

