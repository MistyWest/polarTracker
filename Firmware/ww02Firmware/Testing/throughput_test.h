/*
 * throughput_test.h
 *
 *  Created on: Nov 28, 2018
 *      Author: KLockwood
 */

#ifndef THROUGHPUT_TEST_H_
#define THROUGHPUT_TEST_H_

#define STOP_THROUGHPUT_TEST				0
#define START_THROUGHPUT_TEST				1
#define ENABLE_2MBPS_TEST						2

void throughput_test_update_ble_connection_params( bool irq_context, uint16_t interval_update );
void throughput_test_update_ble_throughput_settings( uint16_t mtu_size_update, uint16_t gap_length_update );
void throughput_test_update_ble_phy_setting( uint8_t phy );
void throughput_test_check_throughput_test( uint8_t value );

void start_data_throughput_test();
void stop_data_throughput_test();

void throughput_test_init( uint16_t * conn_handle, ble_nus_t * p_nus );

#endif /* THROUGHPUT_TEST_H_ */
