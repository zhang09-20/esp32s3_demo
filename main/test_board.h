/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef _TEST_BOARD_H_
#define _TEST_BOARD_H_

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Codec configuration by ESP32S3_KORVO2_V3
 */
#define TEST_BOARD_I2C_SDA_PIN      (1)
#define TEST_BOARD_I2C_SCL_PIN      (2)

#define TEST_BOARD_I2S_BCK_PIN      (40)
#define TEST_BOARD_I2S_MCK_PIN      (39)
#define TEST_BOARD_I2S_DATA_IN_PIN  (42)
#define TEST_BOARD_I2S_DATA_OUT_PIN (38)
#define TEST_BOARD_I2S_DATA_WS_PIN  (41)

#define TEST_BOARD_PA_PIN           (21)

#ifdef __cplusplus
}
#endif

#endif
