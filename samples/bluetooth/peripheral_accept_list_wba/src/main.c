/*
 * Copyright (c) 2022 Michal Morsisko
 * Copyright (c) 2015-2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <zephyr/sys/printk.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/settings/settings.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/bluetooth/hci.h>

/* Custom Service Variables */
static void remove_addr_from_filter_list(const bt_addr_le_t *addr);

#define BT_UUID_CUSTOM_SERVICE_VAL \
	BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef0)

static const struct bt_uuid_128 primary_service_uuid = BT_UUID_INIT_128(
	BT_UUID_CUSTOM_SERVICE_VAL);

static const struct bt_uuid_128 read_characteristic_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef1));

static const struct bt_uuid_128 write_characteristic_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x12345678, 0x1234, 0x5678, 0x1234, 0x56789abcdef2));

static int signed_value;
static struct bt_le_adv_param adv_param;
static struct bt_le_adv_param adv_param_privacy;
static int bond_count;
static volatile int is_connected=0;
static K_SEM_DEFINE(sem_security_changed, 0, 1);


static ssize_t read_signed(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			   void *buf, uint16_t len, uint16_t offset)
{
	int *value = &signed_value;

	return bt_gatt_attr_read(conn, attr, buf, len, offset, value,
				 sizeof(signed_value));
}

static ssize_t write_signed(struct bt_conn *conn, const struct bt_gatt_attr *attr,
			    const void *buf, uint16_t len, uint16_t offset,
			    uint8_t flags)
{
	int *value = &signed_value;

	if (offset + len > sizeof(signed_value)) {
		return BT_GATT_ERR(BT_ATT_ERR_INVALID_OFFSET);
	}

	memcpy(value + offset, buf, len);

	return len;
}

/* Vendor Primary Service Declaration */
BT_GATT_SERVICE_DEFINE(primary_service,
	BT_GATT_PRIMARY_SERVICE(&primary_service_uuid),
	BT_GATT_CHARACTERISTIC(&read_characteristic_uuid.uuid,
			       BT_GATT_CHRC_READ,
			       BT_GATT_PERM_READ,
			       read_signed, NULL, NULL),
	BT_GATT_CHARACTERISTIC(&write_characteristic_uuid.uuid,
			       BT_GATT_CHRC_WRITE,
			       BT_GATT_PERM_WRITE_ENCRYPT,
			       NULL, write_signed, NULL),
);

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR))
};

static const struct bt_data sd[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_CUSTOM_SERVICE_VAL),
	BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static const struct bt_data sd_privacy[] = {
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_CUSTOM_SERVICE_VAL),
	BT_DATA(BT_DATA_NAME_COMPLETE, "Private adv", sizeof("Private adv") - 1),
};

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		printk("Connection failed, err 0x%02x %s\n", err, bt_hci_err_to_str(err));
		is_connected=0;
	} else {
		printk("Connected\n");
		is_connected=1;
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	is_connected=0;
	printk("Disconnected, reason 0x%02x %s\n", reason, bt_hci_err_to_str(reason));
}

static void security_changed_cb(struct bt_conn *conn, bt_security_t level,
				enum bt_security_err sec_err)
{
	if (sec_err == 0) {
		printk("Security changed: %u", level);
		// k_sem_give(&sem_security_changed);
	} else {
		printk("Failed to set security level: %s(%d)",
			bt_security_err_to_str(sec_err), sec_err);
			
			int err;

			printk("Removing old key");
			err = bt_unpair(BT_ID_DEFAULT, bt_conn_get_dst(conn));
			if (err != 0) {
				printk("Failed to remove old key: %d", err);
			}else{
				remove_addr_from_filter_list(bt_conn_get_dst(conn));
			}
	}
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
	.connected = connected,
	.disconnected = disconnected,
	.security_changed = security_changed_cb,
};

static void add_bonded_addr_to_filter_list(const struct bt_bond_info *info, void *data)
{
	char addr_str[BT_ADDR_LE_STR_LEN];

	bt_le_filter_accept_list_add(&info->addr);
	bt_addr_le_to_str(&info->addr, addr_str, sizeof(addr_str));
	printk("Added %s to advertising accept filter list\n", addr_str);
	bond_count++;
}

static void remove_addr_from_filter_list(const bt_addr_le_t *addr)
{
	char addr_str[BT_ADDR_LE_STR_LEN];

	bt_le_filter_accept_list_remove(addr);
	bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));
	printk("Remove %s from advertising accept filter list\n", addr_str);
	bond_count--;
}


static void bt_ready(void)
{
	int err;

	printk("Bluetooth initialized\n");

	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}

	bond_count = 0;
	bt_foreach_bond(BT_ID_DEFAULT, add_bonded_addr_to_filter_list, NULL);

	adv_param = *BT_LE_ADV_CONN_FAST_1;

	/* If we have got at least one bond, activate the filter */
	if (bond_count) {
		/* BT_LE_ADV_OPT_FILTER_CONN is required to activate accept filter list,
		 * BT_LE_ADV_OPT_FILTER_SCAN_REQ will prevent sending scan response data to
		 * devices, that are not on the accept filter list
		 */
		adv_param.options |= BT_LE_ADV_OPT_FILTER_CONN | BT_LE_ADV_OPT_FILTER_SCAN_REQ;
	}

	err = bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));

	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
	} else {
		printk("Advertising successfully started\n");
	}
}

static void start_advertising(void)
{
	int err;
	adv_param = *BT_LE_ADV_CONN_FAST_1;
	err = bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));

	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
	} else {
		printk("Advertising successfully started\n");
	}
}

static void start_advertising_privacy(void)
{
	int err;
	// err = bt_le_filter_accept_list_clear();
 
    // if (err) 
    // {
    //     printk("Cannot clear accept list (err: %d)\n", err);
    //     return err;
    // }
	bt_foreach_bond(BT_ID_DEFAULT, add_bonded_addr_to_filter_list, NULL);
	adv_param_privacy = *BT_LE_ADV_CONN_FAST_1;
	adv_param_privacy.options |= BT_LE_ADV_OPT_FILTER_CONN | BT_LE_ADV_OPT_FILTER_SCAN_REQ;
	err = bt_le_adv_start(&adv_param_privacy, ad, ARRAY_SIZE(ad), sd_privacy, ARRAY_SIZE(sd_privacy));

	if (err) {
		printk("Advertising privacy failed to start (err %d)\n", err);
	} else {
		printk("Advertising privacy successfully started\n");
	}
}

static void stop_advertising(void)
{
	int err;

	err = bt_le_adv_stop();

	if (err) {
		printk("Stop advertising failed (err %d)\n", err);
	} else {
		printk("Stop advertising successfull\n");
	}
}

void pairing_complete(struct bt_conn *conn, bool bonded)
{
	printk("Pairing completed. Rebooting in 5 seconds...\n");

	k_sleep(K_SECONDS(5));
	sys_reboot(SYS_REBOOT_WARM);
}

static struct bt_conn_auth_info_cb bt_conn_auth_info = {
	.pairing_complete = pairing_complete
};




int main(void)
{
	int err;

	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}
	if (IS_ENABLED(CONFIG_SETTINGS)) {
		settings_load();
	}
	// bt_ready();
	  bt_conn_auth_info_cb_register(&bt_conn_auth_info);
	 while (1) {
		// bt_foreach_bond(BT_ID_DEFAULT, add_bonded_addr_to_filter_list, NULL);
		start_advertising();
		bt_conn_auth_info_cb_register(&bt_conn_auth_info);
	 	k_msleep(20000);
	 	 while(is_connected==1){
	 		k_msleep(1000);
	 	 }
	 	stop_advertising();
		
	 	start_advertising_privacy();
	 	bt_conn_auth_info_cb_register(&bt_conn_auth_info);
	 	k_msleep(20000);
	 	while(is_connected==1){
	 		k_msleep(1000);
	 	}
	 	stop_advertising();
	 }
	while (1) {
		k_sleep(K_FOREVER);
	}
	return 0;
}
