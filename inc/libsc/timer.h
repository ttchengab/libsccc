/*
 * timer.h
 *
 * Author: Ming Tsang
 * Copyright (c) 2014-2015 HKUST SmartCar Team
 * Refer to LICENSE for details
 */

#pragma once

#include <cstdint>

namespace libsc
{

class Timer
{
public:
	typedef uint32_t TimerInt;

	virtual ~Timer()
	{}

	/**
	 * Return the time elapsed, in ms
	 *
	 * @return
	 */
	virtual TimerInt Time() = 0;

	/**
	 * Return how @a a is apart from @a b
	 *
	 * @param a Some time in the future, in terms of @a b
	 * @param b Some time in the past, in terms of @a a
	 * @return
	 */
	static TimerInt TimeDiff(const TimerInt a, const TimerInt b)
	{
		return (TimerInt)(a - b);
	}
};

}
