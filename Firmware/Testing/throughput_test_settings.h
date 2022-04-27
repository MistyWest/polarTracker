/*
 * througphut_test_settings.h
 *
 *  Created on: Mar 11, 2019
 *      Author: klockwood
 */

/**@file
 * @defgroup mw_test_settings Throughput Test Settings
 * @ingroup  mw_test
 * @brief    Throughput test settings
 */

#ifndef TESTING_THROUGPHUT_TEST_SETTINGS_H_
#define TESTING_THROUGPHUT_TEST_SETTINGS_H_


//******************************************************
//**** Data Throughput Items

#define DATA_SEND_INTERVAL                      400
#define DATA_SEND_INTERVAL_BUFFER               3         /**< should be atleast 1/2 connection interval when PHY = 1Mbps */
#define DATA_SEND_INTERVAL_BUFFER_LOW_DL        15
#define DATA_SEND_INTERVAL_DIVISOR_2MBPS        5
#define DATA_SEND_INTERVAL_DIVISOR_1MBPS        4
#define DATA_SEND_INTERVAL_DIVISOR_1MBPS_LOW_DL 1


#define DATA_SEND_INTERVAL_NON_ADAPTIVE         1
//******************************************************

#endif /* TESTING_THROUGPHUT_TEST_SETTINGS_H_ */
