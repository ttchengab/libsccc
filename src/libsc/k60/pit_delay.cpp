/*
 * pit_delay.cpp
 *
 * Author: Ming Tsang
 * Copyright (c) 2014-2015 HKUST SmartCar Team
 * Refer to LICENSE for details
 */

#include <cstdint>

#include "libbase/k60/clock_utils.h"
#include "libbase/k60/pit.h"

#include "libsc/k60/pit_delay.h"

using namespace libbase::k60;

namespace libsc
{
namespace k60
{

namespace
{

Pit::Config GetPitConfig(const uint8_t channel)
{
	Pit::Config pc;
	pc.channel = channel;
	pc.is_enable = false;
	return pc;
}

}

PitDelay::PitDelay(const uint8_t channel)
		: m_pit(GetPitConfig(channel))
{}

inline void PitDelay::DelayCount(const uint32_t count)
{
	m_pit.SetCount(count);
	m_pit.SetEnable(true);
	m_pit.ConsumeInterrupt();
	while (!m_pit.IsInterruptRequested())
	{}
	m_pit.SetEnable(false);
}

void PitDelay::DelayUs(const uint16_t us)
{
	DelayCount(ClockUtils::GetBusTickPerUs(us));
}

void PitDelay::DelayMs(const uint16_t ms)
{
	DelayCount(ClockUtils::GetBusTickPerMs(ms));
}

}
}
