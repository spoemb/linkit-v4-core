#pragma once

#include <list>
#include <string>

using KineisPacket = std::string;

struct KineisEventTxStarted {};
struct KineisEventTxComplete {};
struct KineisEventDeviceError {};

enum KineisMode {
	LDK,
	LDA2,
	VLDA4
};

class KineisEventListener {
public:
	// virtual void react(KineisEventPowerOn const& ) {}
	// virtual void react(KineisEventPowerOff const& ) {}
	virtual void react(KineisEventTxStarted const& ) {}
	virtual void react(KineisEventTxComplete const& ) {}
	// virtual void react(KineisEventRxStarted const& ) {}
	// virtual void react(KineisEventRxStopped const& ) {}
	// virtual void react(KineisEventRxPacket const& ) {}
	// virtual void react(KineisEventDeviceIdle const& ) {}
	// virtual void react(KineisEventDeviceReady const& ) {}
	virtual void react(KineisEventDeviceError const& ) {}
};


class KineisDevice {
private:
	std::list<KineisEventListener*> m_listeners;

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
	virtual void send(const KineisMode mode, const KineisPacket& packet, const unsigned int size_bits) = 0;
	virtual void stop_send() = 0;
};