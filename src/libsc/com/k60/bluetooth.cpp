/*
 * bluetooth.h
 * Bluetooth abstraction. For compatibility only, consider using UartDevice
 *
 * Author: Ming Tsang
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#include <mini_common.h>

#include <MK60_uart.h>

#include "libsc/com/config.h"
#include "libsc/com/uart_device.h"
#include "libsc/com/bluetooth.h"

namespace libsc
{

#ifdef LIBSC_BT_UART

Bluetooth::Bluetooth()
		: UartDevice(LIBSC_BT_UART)
{}

#endif

}
