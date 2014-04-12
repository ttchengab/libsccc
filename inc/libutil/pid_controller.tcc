/*
 * pid_controller.tcc
 * Generic PID controller
 *
 * Author: Ming Tsang
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#include <cstdint>
#include <cstdio>

#include <log.h>

#include "libutil/clock.h"

#include "libutil/pid_controller.h"

namespace libutil
{

template<typename T, typename U>
PidController<T, U>::PidController(const InputType setpoint, const float kp,
		const float ki, const float kd)
		: m_setpoint(setpoint), m_kp(kp), m_ki(ki), m_kd(kd),
		  m_accumulated_error(0), m_prev_error(0), m_prev_time(Clock::Time())
{}

template<typename T, typename U>
typename PidController<T, U>::OutputType PidController<T, U>::Calc(
		const Clock::ClockInt time, const InputType current_val)
{
	const Clock::ClockInt time_diff = Clock::TimeDiff(time, m_prev_time);
	const SignedInputType error = m_setpoint - current_val;

	const float p = m_kp * error;
	m_accumulated_error += error;
	const float i = m_ki * m_accumulated_error;
	const float slope = (error - m_prev_error) / (float)time_diff;
	const float d = m_kd * slope;

	m_prev_error = error;
	m_prev_time = time;
	LOG_D("P: %f I: %f D: %f", p, i, d);
	return p + i - d;
}

template<typename T, typename U>
inline void PidController<T, U>::PrintError(const int)
{
	printf("Accumulated Error: %d\n", m_accumulated_error);
}

template<typename T, typename U>
inline void PidController<T, U>::PrintError(const double)
{
	printf("Accumulated Error: %f\n", m_accumulated_error);
}

template<typename T, typename U>
void PidController<T, U>::Print(const char *label)
{
	if (label)
	{
		iprintf("=== PID State for %s ===\n", label);
	}
	else
	{
		iprintf("=== PID State ===\n");
	}
	printf("KP: %f KI: %f KD: %f\n", m_kp, m_ki, m_kd);
	PrintError(m_accumulated_error);
}

}
