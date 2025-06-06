/*
 * Copyright (c) 2023 Nuvoton Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ZEPHYR_DRIVERS_ETHERNET_ETH_NUMAKER_PRIV_H_
#define ZEPHYR_DRIVERS_ETHERNET_ETH_NUMAKER_PRIV_H_

#include <zephyr/types.h>

#define NU_ETH_MAX_FLEN             (1518)

#define NU_HWADDR_SIZE              (6)

#define NU_ETH_MTU_SIZE              1500

/* NuMaker chip's OUI*/
#define NUMAKER_OUI_B0 0xDA
#define NUMAKER_OUI_B1 0x00
#define NUMAKER_OUI_B2 0x53

#endif /* ZEPHYR_DRIVERS_ETHERNET_ETH_NUMAKER_PRIV_H_ */
