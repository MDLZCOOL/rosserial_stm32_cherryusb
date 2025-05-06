/*
 * Copyright (c) 2025, MDLZCOOL
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

#include "ros/node_handle.h"
#include "binding_cherryusb.h"

namespace ros
{
  typedef NodeHandle_<Bind_CherryUSB> NodeHandle; // default 25, 25, 512, 512
}

