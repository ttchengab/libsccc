/*
 * mma8451q.cpp
 *
 * Author: Peter
 * Copyright (c) 2014-2015 HKUST SmartCar Team
 * Refer to LICENSE for details
 */

#include <cassert>
#include <cmath>
#include <cstdint>

#include <array>
#include <vector>

#include "libbase/log.h"

#include "libsc/config.h"
#include "libsc/device_h/mma8451q.h"
#include "libsc/k60/mma8451q.h"
#include "libutil/math.h"
#include "libutil/misc.h"

using namespace libbase::k60;
using namespace libutil;
using namespace std;

#define HALF_PI 1.57079633f

namespace libsc
{
namespace k60
{

#ifdef LIBSC_USE_MMA8451Q

namespace
{

#if LIBSC_USE_MMA8451Q == 1
inline Pin::Name GetSclPin(const uint8_t id)
{
	if (id != 0)
	{
		assert(false);
	}
	return LIBSC_MMA8451Q0_SCL;
}

inline Pin::Name GetSdaPin(const uint8_t id)
{
	if (id != 0)
	{
		assert(false);
	}
	return LIBSC_MMA8451Q0_SDA;
}

#endif

SoftI2cMaster::Config GetI2cMasterConfig(const Mma8451q::Config &config)
{
	SoftI2cMaster::Config product;
	product.scl_pin = GetSclPin(config.id);
	product.sda_pin = GetSdaPin(config.id);
	return product;
}

}

Mma8451q::Mma8451q(const Mma8451q::Config &config)
		: m_i2c_master(GetI2cMasterConfig(config)),
		  m_sensitivity(config.sensitivity),
		  m_scale_factor((float)(1 << ((Byte)m_sensitivity + 0x0C)))
{
	SetActive(false);

	WriteRegByte(MMA8451Q_RA_XYZ_DATA_CFG, 2 - (Byte)config.sensitivity);

//	WriteRegByte(MMA8451Q_RA_CTRL_REG2, (Byte)config.power_mode);
//
//	WriteRegByte(MMA8451Q_RA_CTRL_REG1, (Byte)config.output_data_rate << 3 | MMA8451Q_CR1_LNOISE);
	WriteRegByte(MMA8451Q_RA_CTRL_REG1, (Byte)config.output_data_rate << 3);

	SetActive(true);
}

bool Mma8451q::IsConnected()
{
	Byte devId = 0;
	devId = ReadRegByte(MMA8451Q_RA_REG_WHO_AM_I);
	return (devId == 0x1A);
}

bool Mma8451q::Update()
{
	if (ReadRegByte(MMA8451Q_RA_REG_STATUS) & MMA8451Q_S_ZYXDR)
	{
		GetAllAccel();
		GetAllAngle();
		return true;
	}
	return false;
}

void Mma8451q::GetAllAccel()
{
	const vector<Byte> &bytes = ReadRegBytes(MMA8451Q_RA_REG_OUT_ALL, 0x06);

	for (int i = 0, j = 0; i < 3; i++, j += 2)
		m_last_accel[i] = (int16_t)(bytes[j] << 8 | bytes[j + 1]) / m_scale_factor;
}

void Mma8451q::GetAllAngle()
{
	for (int j = 0; j < 3; j++)
	{
		const float factor = m_last_accel[(j + 1) % 3] * m_last_accel[(j + 1) % 3]
				+ m_last_accel[(j + 2) % 3] * m_last_accel[(j + 2) % 3];
		m_last_angle[j] = Math::ArcTan(m_last_accel[j] / Math::Sqrt2(factor));
		if (!isfinite(m_last_angle[j]))
			m_last_angle[j] = HALF_PI;
	}
	m_last_angle[2] -= HALF_PI * ((m_last_accel[2] < 0)? -1 : 1);
}

void Mma8451q::SetActive(const bool flag)
{
	uint8_t reg = ReadRegByte(MMA8451Q_RA_CTRL_REG1);
	if (flag)
	{
		CLEAR_BIT(reg, MMA8451Q_CR1_F_ACTIVE_SHIFT);
	}
	else
	{
		SET_BIT(reg, MMA8451Q_CR1_F_ACTIVE_SHIFT);
	}

	WriteRegByte(MMA8451Q_RA_CTRL_REG1, reg);
}

vector<Byte> Mma8451q::ReadRegBytes(const Byte reg, const Byte length)
{
	vector<Byte> data;
	data = m_i2c_master.GetBytes(MMA8451Q_DEFAULT_ADDRESS, reg, length);
	if (data.empty())
	{
		LOG_W("MMA8451Q Failed reading 0x%X", reg);
	}
	return data;
}

Byte Mma8451q::ReadRegByte(const Byte reg)
{
	Byte data = 0;
	if (!m_i2c_master.GetByte(MMA8451Q_DEFAULT_ADDRESS, reg, &data))
	{
		LOG_W("MMA8451Q Failed reading 0x%X", reg);
	}
	return data;
}

void Mma8451q::WriteRegByte(const Byte reg, const Byte data)
{
	if (!m_i2c_master.SendByte(MMA8451Q_DEFAULT_ADDRESS, reg, data))
	{
		LOG_W("MMA8451Q Failed sending 0x%X", reg);
	}
}

#else
Mma8451q::Mma8451q(const Config&)
		: m_i2c_master(nullptr), m_sensitivity(Config::Sensitivity::kHigh),
		  m_scale_factor(0.0f)
{
	LOG_DL("Configured not to use Mma8451q");
}
bool Mma8451q::IsConnected() { return false; }
bool Mma8451q::Update() { return false; }

#endif

}
}