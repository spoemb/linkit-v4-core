#pragma once

#include "kineis_device.hpp"
#include "CppUTestExt/MockSupport.h"

class MockKineisDevice : public KineisDevice {
public:
	MockKineisDevice() {}
	virtual ~MockKineisDevice() {}

	void send(const KineisModulation mode, const KineisPacket& /*packet*/, const unsigned int size_bits) override {
		mock().actualCall("send")
			.onObject(this)
			.withUnsignedIntParameter("mode", (unsigned int)mode)
			.withUnsignedIntParameter("size_bits", size_bits);
	}

	void stop_send() override {
		mock().actualCall("stop_send")
			.onObject(this);
	}

	void power_off_immediate() override {
		mock().actualCall("power_off_immediate")
			.onObject(this);
	}

	void start_receive(const KineisModulation mode) override {
		mock().actualCall("start_receive")
			.onObject(this)
			.withUnsignedIntParameter("mode", (unsigned int)mode);
	}

	bool stop_receive() override {
		mock().actualCall("stop_receive")
			.onObject(this);
		return true;
	}

	void set_frequency(double freq_mhz) override {
		mock().actualCall("set_frequency")
			.onObject(this)
			.withDoubleParameter("freq", freq_mhz);
	}

	void set_tcxo_warmup_time(unsigned int ms) override {
		mock().actualCall("set_tcxo_warmup_time")
			.onObject(this)
			.withUnsignedIntParameter("time", ms);
	}

	bool switch_modulation(KineisModulation mode, const std::string& rconf_hex) override {
		mock().actualCall("switch_modulation")
			.onObject(this)
			.withUnsignedIntParameter("mode", (unsigned int)mode)
			.withStringParameter("rconf", rconf_hex.c_str());
		m_current_mod = mode;
		return true;
	}

	KineisModulation get_current_modulation() const override {
		return m_current_mod;
	}

	void set_lpm_mode(uint8_t lpm_bitmap) override {
		(void)lpm_bitmap;  // LPM is SMD-specific, not verified in generic TX tests
	}

private:
	KineisModulation m_current_mod = KineisModulation::LDA2;
};
