/*
 * Copyright (c) 2025, MDLZCOOL
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#if __has_include("stm32f1xx_hal.h")
#include "stm32f1xx_hal.h"
#elif __has_include("stm32f2xx_hal.h")
#include "stm32f2xx_hal.h"
#elif __has_include("stm32f4xx_hal.h")
#include "stm32f4xx_hal.h"
#elif __has_include("stm32f7xx_hal.h")
#include "stm32f7xx_hal.h"
#elif __has_include("stm32h7xx_hal.h")
#include "stm32h7xx_hal.h"
#elif __has_include("stm32h7rsxx_hal.h")
#include "stm32h7rsxx_hal.h"
#elif __has_include("stm32l4xx_hal.h")
#include "stm32l4xx_hal.h"
#endif

extern __IO uint32_t uwTick;

class Bind_CherryUSB
{
public:
    void init();
    int read();
    void write(uint8_t* dat, int len);
    unsigned long time();
};
