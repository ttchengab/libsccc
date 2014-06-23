/*
 * uart.h
 *
 * Author: Ming Tsang
 * Copyright (c) 2014 HKUST SmartCar Team
 */

#ifndef LIBBASE_K60_UART_H_
#define LIBBASE_K60_UART_H_

#include <vectors.h>

#include <cstdint>

#include <bitset>
#include <functional>
#include <vector>

#include "libbase/k60/pin.h"

namespace libbase
{
namespace k60
{

struct UartConfig
{
	enum struct BaudRate
	{
		BR_2400,
		BR_4800,
		BR_9600,
		BR_14400,
		BR_19200,
		BR_28800,
		BR_38400,
		BR_57600,
		BR_76800,
		BR_115200,
		BR_230400,
		BR_460800,
	};

	enum ConfigBit
	{
		// Connect TX to RX
		LOOP_MODE,
		// Enable parity bit or not, if enabled, it would be even parity
		ENABLE_EVEN_PARITY,
		FIFO,

		SIZE,
	};

	PinConfig::Name tx_pin;
	PinConfig::Name rx_pin;
	BaudRate baud_rate;
	std::bitset<ConfigBit::SIZE> config;
	// The # bytes in the Tx buffer needed to fire the interrupt
	uint8_t tx_irq_threshold = 0;
	// To treat tx_irq_threshold as a percentage of Tx buffer size
	bool is_tx_irq_threshold_percentage = false;
	// The # bytes in the Tx buffer needed to fire the interrupt
	uint8_t rx_irq_threshold = 1;
	// To treat rx_irq_threshold as a percentage of Rx buffer size
	bool is_rx_irq_threshold_percentage = false;
};

class Uart
{
public:
	typedef std::function<void(Uart *uart)> OnTxEmptyListener;
	typedef std::function<void(Uart *uart)> OnRxFullListener;

	explicit Uart(const UartConfig &config);
	~Uart();

	void SetLoopMode(const bool flag);

	uint8_t GetAvailableBytes() const;
	Byte GetByte() const;
	bool PeekByte(Byte *out_byte) const;
	std::vector<Byte> GetBytes() const;
	bool PeekBytes(std::vector<Byte> *out_bytes) const;
	void SendByte(const Byte byte);
	size_t SendBytes(const Byte *bytes, const size_t size);
	size_t SendBytes(const std::vector<Byte> &bytes)
	{
		return SendBytes(bytes.data(), bytes.size());
	}

	uint8_t GetTxFifoSize() const
	{
		return m_tx_fifo_size;
	}

	uint8_t GetRxFifoSize() const
	{
		return m_rx_fifo_size;
	}

	void SetTxIsr(OnTxEmptyListener isr);
	void SetRxIsr(OnRxFullListener isr);

private:
	enum Module
	{
		UART0 = 0,
		UART1,
		UART2,
		UART3,
		UART4,
		UART5,
	};

	void SetEnableClockGate(const bool flag);
	bool SetModule(const PinConfig::Name tx_pin, const PinConfig::Name rx_pin);
	void SetBaudRate(const UartConfig::BaudRate br);
	void InitPin(const PinConfig::Name tx_pin, const PinConfig::Name rx_pin);
	void InitC1Reg(const UartConfig &config);
	void InitFifo(const UartConfig &config);
	void SetInterrupt();

	static __ISR void IrqHandler();

	Module m_module;
	bool m_is_fifo;

	Pin m_tx;
	uint8_t m_tx_fifo_size;
	OnTxEmptyListener m_tx_isr;

	Pin m_rx;
	uint8_t m_rx_fifo_size;
	OnRxFullListener m_rx_isr;
};

}
}

#endif /* LIBBASE_K60_UART_H_ */