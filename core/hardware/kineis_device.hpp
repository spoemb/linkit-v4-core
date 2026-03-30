#pragma once

#include <list>
#include <string>

using KineisPacket = std::string;

struct KineisEventPowerOn {};
struct KineisEventPowerOff {};
struct KineisEventTxStarted {};
struct KineisEventTxComplete {};
struct KineisEventRxStarted {};
struct KineisEventRxStopped {
	unsigned int rx_time;
};
struct KineisEventRxPacket {
	KineisPacket packet;
	unsigned int size_bits;
};
struct KineisEventDeviceError {};

enum KineisModulation {
	LDK,
	LDA2,
	VLDA4
};

class KineisEventListener {
public:
	virtual void react(KineisEventPowerOn const& ) {}
	virtual void react(KineisEventPowerOff const& ) {}
	virtual void react(KineisEventTxStarted const& ) {}
	virtual void react(KineisEventTxComplete const& ) {}
	virtual void react(KineisEventRxStarted const& ) {}
	virtual void react(KineisEventRxStopped const& ) {}
	virtual void react(KineisEventRxPacket const& ) {}
	virtual void react(KineisEventDeviceError const& ) {}
};


class KineisDevice {
private:
	std::list<KineisEventListener*> m_listeners;

protected:
	unsigned int m_idle_timeout_ms = 1000;

public:
	virtual ~KineisDevice() {}
	void subscribe(KineisEventListener& m) {
		m_listeners.push_back(&m);
	}
	void unsubscribe(KineisEventListener& m) {
		m_listeners.remove(&m);
	}
	template<typename E> void notify(E const& e) {
		for (auto m : m_listeners) {
			m->react(e);
		}
	}
	void set_idle_timeout(unsigned int ms) { m_idle_timeout_ms = ms; }
	virtual void send(const KineisModulation mode, const KineisPacket& packet, const unsigned int size_bits) = 0;
	virtual void stop_send() = 0;
	virtual void start_receive(const KineisModulation mode) = 0;
	virtual bool stop_receive() = 0;
	virtual void set_frequency(double freq_mhz) = 0;
	virtual void set_tcxo_warmup_time(unsigned int ms) = 0;
	virtual void set_tx_power(unsigned int power) { (void)power; }
	virtual void set_lpm_mode(uint8_t lpm_bitmap) { (void)lpm_bitmap; }
	virtual void set_credentials(unsigned int dec_id, unsigned int address,
	                             const std::string& seckey, const std::string& radioconf) {
		(void)dec_id; (void)address; (void)seckey; (void)radioconf;
	}
	virtual void read_credentials(unsigned int *dec_id, unsigned int *address,
	                              std::string *seckey, std::string *radioconf) {
		(void)dec_id; (void)address; (void)seckey; (void)radioconf;
	}
	// Switch radio modulation at runtime: write RCONF, save to flash, reload KMAC.
	// rconf_hex: 32-char hex string for the target modulation's radio configuration.
	// Returns true on success.
	virtual bool switch_modulation(KineisModulation mode, const std::string& rconf_hex) {
		(void)mode; (void)rconf_hex; return true;
	}
	virtual KineisModulation get_current_modulation() const { return KineisModulation::LDA2; }
};