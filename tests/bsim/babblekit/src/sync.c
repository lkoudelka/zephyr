/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include "argparse.h"
#include "bs_types.h"
#include "bs_tracing.h"
#include "time_machine.h"
#include "bs_pc_backchannel.h"

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(sync, CONFIG_LOG_DEFAULT_LEVEL);

#define CHANNEL_ID 0
#define MSG_SIZE 1

int bk_sync_init(void)
{
	uint device_number = get_device_nbr();
	uint peer_number = device_number ^ 1;
	uint device_numbers[] = { peer_number };
	uint channel_numbers[] = { CHANNEL_ID };
	uint *ch;

	ch = bs_open_back_channel(device_number, device_numbers, channel_numbers,
				  ARRAY_SIZE(channel_numbers));
	if (!ch) {
		return -1;
	}

	LOG_DBG("Sync initialized");

	return 0;
}

void bk_sync_send(void)
{
	uint8_t sync_msg[MSG_SIZE] = { get_device_nbr() };

	LOG_DBG("Sending sync");
	bs_bc_send_msg(CHANNEL_ID, sync_msg, ARRAY_SIZE(sync_msg));
}

void bk_sync_wait(void)
{
	uint8_t sync_msg[MSG_SIZE];

	LOG_DBG("Waiting for sync");

	while (true) {
		if (bs_bc_is_msg_received(CHANNEL_ID) > 0) {
			bs_bc_receive_msg(CHANNEL_ID, sync_msg, ARRAY_SIZE(sync_msg));
			if (sync_msg[0] != get_device_nbr()) {
				/* Received a message from another device, exit */
				break;
			}
		}

		k_sleep(K_MSEC(1));
	}

	LOG_DBG("Sync received");
}
