/**
 * @file ledsm.hpp
 * @brief LED state machine — maps tracker states to RGB LED patterns (solid, flash, alternate).
 */

#pragma once

#include "tinyfsm.hpp"
#include "timer.hpp"

/// @name LED state events (dispatched by GenTracker FSM)
/// @{
struct SetLEDOff : tinyfsm::Event { };
struct SetLEDMagnetEngaged : tinyfsm::Event { };
struct SetLEDMagnetDisengaged : tinyfsm::Event { };
struct SetLEDBoot : tinyfsm::Event { };
struct SetLEDPowerDown : tinyfsm::Event { };
struct SetLEDError : tinyfsm::Event { };
struct SetLEDPreOperationalPending : tinyfsm::Event { };
struct SetLEDPreOperationalError : tinyfsm::Event { };
struct SetLEDPreOperationalBatteryNominal : tinyfsm::Event { };
struct SetLEDPreOperationalBatteryLow : tinyfsm::Event { };
struct SetLEDConfigPending : tinyfsm::Event { };
struct SetLEDConfigNotConnected : tinyfsm::Event { };
struct SetLEDConfigConnected : tinyfsm::Event { };
struct SetLEDGNSSOn : tinyfsm::Event { };
struct SetLEDGNSSOffWithFix : tinyfsm::Event { };
struct SetLEDGNSSOffWithoutFix : tinyfsm::Event { };
// 2026-05 deep-idle refactor FAST3c: visual marker when the M10Q has captured
// its first raw CloudLocate measurement mid-session. Double-blink CYAN
// distinguishes from LEDGNSSOn (steady CYAN flash) so bench operators can see
// when raw measurements are ready without waiting for full PVT.
struct SetLEDGNSSCloudLocateReady : tinyfsm::Event { };
struct SetLEDArgosTX : tinyfsm::Event { };
struct SetLEDArgosTXComplete : tinyfsm::Event { };
struct SetLEDBatteryCritical : tinyfsm::Event { };
struct SetLEDDFUUpdate : tinyfsm::Event { };
struct SetLEDOTASuccess : tinyfsm::Event { };
struct SetLEDOTAFailed : tinyfsm::Event { };
struct SetLEDFirmwareApplied : tinyfsm::Event { };
struct SetLEDConfirmConfig : tinyfsm::Event { };
struct SetLEDConfirmExitConfig : tinyfsm::Event { };
struct SetLEDConfirmPowerOff : tinyfsm::Event { };
struct SetLEDSurfaceDetected : tinyfsm::Event { };
struct SetLEDDiveDetected : tinyfsm::Event { };

class LEDOff;
class LEDBoot;
class LEDPowerDown;
class LEDError;
class LEDPreOperationalPending;
class LEDPreOperationalBatteryNominal;
class LEDPreOperationalBatteryLow;
class LEDPreOperationalError;
class LEDConfigPending;
class LEDConfigNotConnected;
class LEDConfigConnected;
class LEDGNSSOn;
class LEDGNSSOffWithFix;
class LEDGNSSOffWithoutFix;
class LEDGNSSCloudLocateReady;   // 2026-05 deep-idle refactor FAST3c
class LEDArgosTX;
class LEDArgosTXComplete;
class LEDBatteryCritical;
class LEDDFUUpdate;
class LEDOTASuccess;
class LEDOTAFailed;
class LEDFirmwareApplied;
class LEDConfirmConfig;
class LEDConfirmExitConfig;
class LEDConfirmPowerOff;
class LEDSurfaceDetected;
class LEDDiveDetected;


/// @}

/// @brief LED FSM base — dispatches events to LED state subclasses.
class LEDState : public tinyfsm::Fsm<LEDState> {
protected:
	static inline bool m_is_battery_critical = false;
	static inline bool m_is_gnss_on = false;
	static inline bool m_is_magnet_engaged = false;
public:
	void react(SetLEDOff const &) { transit<LEDOff>(); }
	void react(SetLEDMagnetEngaged const &) { if (!m_is_magnet_engaged) { m_is_magnet_engaged = true; enter(); } }
	void react(SetLEDMagnetDisengaged const &) { if (m_is_magnet_engaged) { m_is_magnet_engaged = false; enter(); } }
	void react(SetLEDBoot const &) { transit<LEDBoot>(); }
	void react(SetLEDPowerDown const &) { transit<LEDPowerDown>(); }
	void react(SetLEDError const &) { transit<LEDError>(); }
	void react(SetLEDPreOperationalPending const &) { transit<LEDPreOperationalPending>(); }
	void react(SetLEDPreOperationalError const &) { transit<LEDPreOperationalError>(); }
	void react(SetLEDPreOperationalBatteryNominal const &) { transit<LEDPreOperationalBatteryNominal>(); }
	void react(SetLEDPreOperationalBatteryLow const &) { transit<LEDPreOperationalBatteryLow>(); }
	void react(SetLEDConfigPending const &) { transit<LEDConfigPending>(); }
	void react(SetLEDConfigNotConnected const &) { transit<LEDConfigNotConnected>(); }
	void react(SetLEDConfigConnected const &) { transit<LEDConfigConnected>(); }
	void react(SetLEDGNSSOn const &) { transit<LEDGNSSOn>(); }
	void react(SetLEDGNSSOffWithFix const &) { transit<LEDGNSSOffWithFix>(); }
	void react(SetLEDGNSSOffWithoutFix const &) { transit<LEDGNSSOffWithoutFix>(); }
	void react(SetLEDGNSSCloudLocateReady const &) { transit<LEDGNSSCloudLocateReady>(); }
	void react(SetLEDArgosTX const &) { transit<LEDArgosTX>(); }
	void react(SetLEDArgosTXComplete const &) { transit<LEDArgosTXComplete>(); }
	void react(SetLEDBatteryCritical const &) { transit<LEDBatteryCritical>(); }
	void react(SetLEDDFUUpdate const &) { transit<LEDDFUUpdate>(); }
	void react(SetLEDOTASuccess const &) { transit<LEDOTASuccess>(); }
	void react(SetLEDOTAFailed const &) { transit<LEDOTAFailed>(); }
	void react(SetLEDFirmwareApplied const &) { transit<LEDFirmwareApplied>(); }
	void react(SetLEDConfirmConfig const &) { transit<LEDConfirmConfig>(); }
	void react(SetLEDConfirmExitConfig const &) { transit<LEDConfirmExitConfig>(); }
	void react(SetLEDConfirmPowerOff const &) { transit<LEDConfirmPowerOff>(); }
	void react(SetLEDSurfaceDetected const &) { transit<LEDSurfaceDetected>(); }
	void react(SetLEDDiveDetected const &) { transit<LEDDiveDetected>(); }

	virtual void entry(void) {}
	virtual void exit(void) {}
};


class LEDOff : public LEDState
{
public:
	void entry() override;
	void exit() override {};
};


class LEDBoot : public LEDState
{
public:
	void entry() override;
	void exit() override {};
};

class LEDPowerDown : public LEDState
{
public:
	void entry() override;
	void exit() override {};
};

class LEDError : public LEDState
{
public:
	void entry() override;
	void exit() override {};
};

class LEDPreOperationalPending : public LEDState
{
public:
	void entry() override;
	void exit() override {};
};

class LEDPreOperationalError : public LEDState
{
public:
	void entry() override;
	void exit() override {};
};

class LEDPreOperationalBatteryNominal : public LEDState
{
public:
	void entry() override;
	void exit() override {};
};

class LEDPreOperationalBatteryLow : public LEDState
{
public:
	void entry() override;
	void exit() override {};
};

class LEDConfigPending : public LEDState
{
public:
	void entry() override;
	void exit() override {};
};

class LEDConfigNotConnected : public LEDState
{
public:
	void entry() override;
	void exit() override {};
};

class LEDConfigConnected : public LEDState
{
public:
	void entry() override;
	void exit() override {};
};

class LEDGNSSOn : public LEDState
{
public:
	void entry() override;
	void exit() override {};
};

class LEDGNSSOffWithFix : public LEDState
{
public:
	void entry() override;
	void exit() override {};
};

class LEDGNSSOffWithoutFix : public LEDState
{
public:
	void entry() override;
	void exit() override {};
};

// 2026-05 deep-idle refactor FAST3c: distinct visual pattern when the first
// CloudLocate raw measurement arrives mid-session. Double-blink CYAN to
// distinguish from the steady CYAN flash of LEDGNSSOn — bench operator
// instantly knows raw measurements are ready (which can be uploaded via
// Argos for cloud-side position resolution even without a full PVT fix).
// After the double-blink, the state machine transitions back to LEDGNSSOn
// (if GPS still active) or LEDOff (if GNSS_CLOUDLOCATE_ONLY terminated the
// session). Transition handled inside the entry() via a scheduled task.
class LEDGNSSCloudLocateReady : public LEDState
{
public:
	void entry() override;
	void exit() override {};
};

class LEDArgosTX : public LEDState
{
public:
	void entry() override;
	void exit() override {};
};

class LEDArgosTXComplete : public LEDState
{
public:
	void entry() override;
	void exit() override {};
};

class LEDBatteryCritical : public LEDState
{
public:
	void entry() override;
	void exit() override {};
};

class LEDDFUUpdate : public LEDState
{
public:
	void entry() override;
	void exit() override {};
};

class LEDOTASuccess : public LEDState
{
public:
	void entry() override;
	void exit() override {};
};

class LEDOTAFailed : public LEDState
{
public:
	void entry() override;
	void exit() override {};
};

class LEDFirmwareApplied : public LEDState
{
public:
	void entry() override;
	void exit() override {};
};

class LEDConfirmConfig : public LEDState
{
public:
	void entry() override;
	void exit() override {};
};

class LEDConfirmExitConfig : public LEDState
{
public:
	void entry() override;
	void exit() override {};
};

class LEDConfirmPowerOff : public LEDState
{
public:
	void entry() override;
	void exit() override {};
};

class LEDSurfaceDetected : public LEDState
{
public:
	void entry() override;
	void exit() override {};
};

class LEDDiveDetected : public LEDState
{
public:
	void entry() override;
	void exit() override {};
};
