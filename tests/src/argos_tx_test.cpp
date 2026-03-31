#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"

#include "mock_kineis_device.hpp"
#include "fake_rtc.hpp"
#include "fake_config_store.hpp"
#include "fake_timer.hpp"
#include "fake_battery_mon.hpp"
#include "dte_protocol.hpp"
#include "binascii.hpp"
#include "timeutils.hpp"
#include "scheduler.hpp"
#include "argos_tx_service.hpp"

using namespace std::string_literals;

extern Timer *system_timer;
extern ConfigurationStore *configuration_store;
extern Scheduler *system_scheduler;
extern RTC *rtc;
extern BatteryMonitor *battery_monitor;


TEST_GROUP(ArgosTxService)
{
	FakeBatteryMonitor *fake_battery_monitor;
	FakeConfigurationStore *fake_config_store;
	MockKineisDevice *mock_kineis;
	FakeRTC *fake_rtc;
	FakeTimer *fake_timer;
	unsigned int txco_warmup = 5U;

	void setup() {
		fake_battery_monitor = new FakeBatteryMonitor;
		battery_monitor = fake_battery_monitor;
		mock_kineis = new MockKineisDevice;
		fake_config_store = new FakeConfigurationStore;
		configuration_store = fake_config_store;
		configuration_store->init();
		fake_rtc = new FakeRTC;
		rtc = fake_rtc;
		// Set RTC time so service_is_time_known() returns true
		fake_rtc->settime(1580083200); // 27/01/2020 00:00:00
		fake_timer = new FakeTimer;
		system_timer = fake_timer; // linux_timer;
		system_scheduler = new Scheduler(system_timer);
		fake_timer->start();

		// Setup PP_MIN_ELEVATION for 5.0
		double min_elevation = 5.0;
		fake_config_store->write_param(ParamID::PP_MIN_ELEVATION, min_elevation);

		// Disable cooldown by default in tests (individual tests can override)
		unsigned int no_cooldown = 0U;
		fake_config_store->write_param(ParamID::MIN_SURFACE_CYCLE_INTERVAL_S, no_cooldown);
	}

	void teardown() {
		delete system_scheduler;
		delete fake_timer;
		delete fake_rtc;
		delete fake_config_store;
		delete mock_kineis;
		delete fake_battery_monitor;
	}

	GPSLogEntry make_gps_location(bool is_valid=true, double longitude=0, double latitude=0, std::time_t t=0, bool is_3d_fix = false, int32_t hMSL=0, int32_t gSpeed=0, uint16_t batt=4200) {
		GPSLogEntry log;
		log.info.valid = is_valid;
		log.info.lon = longitude;
		log.info.lat = latitude;
		log.info.schedTime = t;
		log.info.fixType = is_3d_fix ? 3 : 2;
		log.info.hMSL = hMSL;
		log.info.batt_voltage = batt;
		log.info.gSpeed = gSpeed;

		if (t == 0)
			t = rtc->gettime();

		uint16_t year;
		uint8_t month, day, hour, min, sec;
		convert_datetime_to_epoch(t, year, month, day, hour, min, sec);
		log.header.year = log.info.year = year;
		log.header.month = log.info.month = month;
		log.header.day = log.info.day = day;
		log.header.hours = log.info.hour = hour;
		log.header.minutes = log.info.min = min;
		log.header.seconds = log.info.sec = sec;
		log.info.schedTime = t;
		return log;
	}

	void inject_gps_active() {
		ServiceEvent e;
		e.event_type = ServiceEventType::SERVICE_ACTIVE;
		e.event_source = ServiceIdentifier::GNSS_SENSOR;
		e.event_originator_unique_id = 0x12345678;
		ServiceManager::notify_peer_event(e);
	}

	void inject_gps_inactive() {
		ServiceEvent e;
		e.event_type = ServiceEventType::SERVICE_INACTIVE;
		e.event_source = ServiceIdentifier::GNSS_SENSOR;
		e.event_originator_unique_id = 0x12345678;
		ServiceManager::notify_peer_event(e);
	}

	void inject_sensor_data(ServiceSensorData data, ServiceIdentifier service) {
		ServiceEvent e;
		e.event_source = service;
		e.event_type = ServiceEventType::SERVICE_LOG_UPDATED;
		e.event_data = data;
		e.event_originator_unique_id = 0x12345678;
		ServiceManager::notify_peer_event(e);
	}

	void inject_gps_location(bool is_valid=true, double longitude=0, double latitude=0, std::time_t t=0, bool is_3d_fix = false, int32_t hMSL=0, int32_t gSpeed=0, uint16_t batt=4200) {
		ServiceEvent e;
		GPSLogEntry log = make_gps_location(is_valid, longitude, latitude, t, is_3d_fix, hMSL, gSpeed, batt);

		e.event_source = ServiceIdentifier::GNSS_SENSOR;
		e.event_type = ServiceEventType::SERVICE_LOG_UPDATED;
		e.event_data = log;
		e.event_originator_unique_id = 0x12345678;
		ServiceManager::notify_peer_event(e);
		configuration_store->notify_gps_location(log);
	}

	void notify_underwater_state(bool state) {
		ServiceEvent e;
		e.event_type = ServiceEventType::SERVICE_LOG_UPDATED;
		e.event_data = state;
		e.event_source = ServiceIdentifier::UW_SENSOR;
		e.event_originator_unique_id = 0x12345678;
		ServiceManager::notify_peer_event(e);
	}
};

TEST(ArgosTxService, DepthPileFillsAndEmpties)
{
	DepthPile<GPSLogEntry> dp;
	std::vector<GPSLogEntry*> v;
	GPSLogEntry e;

	// Load up full depth pile with burst count of 1
	for (unsigned int i = 0; i < 24; i++) {
		e.info.day = i;
		dp.store(e, 1);
	}

	// Should have 24 eligible entries
	CHECK_EQUAL(24, dp.eligible());

	// Retrieving the latest entry should not decrement burst counter
	// for time sync burst case
	v = dp.retrieve_latest();
	CHECK_EQUAL(1, v.size());
	CHECK_EQUAL(24, dp.eligible());
	CHECK_EQUAL(23, v.at(0)->info.day); // Should be most recent

	// Retrieve entire depth pile (in blocks of 4) ensuring
	// eligible count is decremented
	for (unsigned int i = 0; i < 6; i++) {
		//CHECK_EQUAL(24-i, dp.eligible());
		v = dp.retrieve(24);
		CHECK_EQUAL(4, v.size());
		CHECK_EQUAL(20-(i*4), dp.eligible());
		for (unsigned j = 0; j < 4; j++) {
			CHECK_EQUAL(24-((i+1)*4)+j, v.at(j)->info.day);
		}
	}

	// Check depth pile returns empty vector
	v = dp.retrieve_latest();
	CHECK_EQUAL(0, v.size());
	v = dp.retrieve(24);
	CHECK_EQUAL(0, v.size());
}

TEST(ArgosTxService, DepthPile1)
{
	DepthPile<GPSLogEntry> dp;
	std::vector<GPSLogEntry*> v;
	GPSLogEntry e;

	// Load up full depth pile with burst count of 1
	for (unsigned int i = 0; i < 24; i++) {
		e.info.day = i;
		dp.store(e, 1);
	}

	// Should have 24 eligible entries
	CHECK_EQUAL(24, dp.eligible());

	// Retrieving the latest entry should not decrement burst counter
	// for time sync burst case
	v = dp.retrieve_latest();
	CHECK_EQUAL(1, v.size());
	CHECK_EQUAL(24, dp.eligible());
	CHECK_EQUAL(23, v.at(0)->info.day); // Should be most recent

	// Retrieve depth pile and should be most recent
	v = dp.retrieve(1);
	CHECK_EQUAL(1, v.size());
	CHECK_EQUAL(23, v.at(0)->info.day);

	// Check depth pile returns empty vector
	v = dp.retrieve_latest();
	CHECK_EQUAL(0, v.size());
	v = dp.retrieve(1);
	CHECK_EQUAL(0, v.size());
}

TEST(ArgosTxService, DutyCycleCalculation)
{
	unsigned int mask;
	mask = 0xFFFFFF;
	for (unsigned int i = 0; i < 24; i++)
		CHECK_EQUAL((bool)((mask >> (23-i))&1), ArgosTxScheduler::is_in_duty_cycle(i*3600*1000, mask));
	mask = 0xEEEEEE;
	for (unsigned int i = 0; i < 24; i++)
		CHECK_EQUAL((bool)((mask >> (23-i))&1), ArgosTxScheduler::is_in_duty_cycle(i*3600*1000, mask));
	mask = 0;
	for (unsigned int i = 0; i < 24; i++)
		CHECK_EQUAL((bool)((mask >> (23-i))&1), ArgosTxScheduler::is_in_duty_cycle(i*3600*1000, mask));
	mask = 0x123456;
	for (unsigned int i = 0; i < 24; i++)
		CHECK_EQUAL((bool)((mask >> (23-i))&1), ArgosTxScheduler::is_in_duty_cycle(i*3600*1000, mask));
}

TEST(ArgosTxService, SchedulerLegacyNoJitter)
{
	ArgosTxScheduler sched;
	ArgosConfig config;

	config.argos_tx_jitter_en = false;
	config.tx_interval_s = 10;
	unsigned int result;

	result = sched.schedule_legacy(config, 0);
	CHECK_EQUAL(0U, result);
	sched.notify_tx_complete();
	result = sched.schedule_legacy(config, 10);
	CHECK_EQUAL(0U, result);
	sched.notify_tx_complete();
	result = sched.schedule_legacy(config, 20);
	CHECK_EQUAL(0U, result);
	sched.notify_tx_complete();
	result = sched.schedule_legacy(config, 30);
	CHECK_EQUAL(0U, result);
	sched.notify_tx_complete();
	result = sched.schedule_legacy(config, 35);
	CHECK_EQUAL(5000U, result);
	sched.notify_tx_complete();
	result = sched.schedule_legacy(config, 59);
	CHECK_EQUAL(1000U, result);
	sched.notify_tx_complete();
}

TEST(ArgosTxService, SchedulerLegacyNoJitterWithEarliestTxSet)
{
	ArgosTxScheduler sched;
	ArgosConfig config;

	config.argos_tx_jitter_en = false;
	config.tx_interval_s = 10;
	unsigned int result;

	result = sched.schedule_legacy(config, 0);
	CHECK_EQUAL(0U, result);
	sched.notify_tx_complete();
	result = sched.schedule_legacy(config, 10);
	CHECK_EQUAL(0U, result);
	sched.notify_tx_complete();
	result = sched.schedule_legacy(config, 20);
	CHECK_EQUAL(0U, result);
	sched.notify_tx_complete();
	result = sched.schedule_legacy(config, 30);
	CHECK_EQUAL(0U, result);
	sched.notify_tx_complete();
	sched.set_earliest_schedule(41);
	result = sched.schedule_legacy(config, 35);
	CHECK_EQUAL(6000U, result);
	sched.notify_tx_complete();
}

TEST(ArgosTxService, SchedulerLegacyWithJitter)
{
	ArgosTxScheduler sched;
	ArgosConfig config;

	config.argos_tx_jitter_en = true;
	config.tx_interval_s = 10;
	unsigned int result;

	result = sched.schedule_legacy(config, 0);
	CHECK_TRUE(result <= 5000);
	sched.notify_tx_complete();
	result = sched.schedule_legacy(config, 0);
	CHECK_TRUE(result <= 15000 && result >= 10000);
	sched.notify_tx_complete();
}

TEST(ArgosTxService, SchedulerDutyCycleNoJitter)
{
	ArgosTxScheduler sched;
	ArgosConfig config;

	config.argos_tx_jitter_en = false;
	config.tx_interval_s = 10;
	config.duty_cycle = 0x1;

	unsigned int result = sched.schedule_duty_cycle(config, 0);
	CHECK_EQUAL(23*3600*1000, result);
	sched.notify_tx_complete();
	result = sched.schedule_duty_cycle(config, 23*3600);
	CHECK_EQUAL(10000U, result);
	sched.notify_tx_complete();
}

TEST(ArgosTxService, BuildLongCertificationPacket)
{
	unsigned int size_bits;
	std::string x = ArgosPacketBuilder::build_certification_packet("FFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFFF", size_bits);
	CHECK_EQUAL(224, size_bits);
	CHECK_EQUAL("\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF"s, x);
}

TEST(ArgosTxService, BuildShortCertificationPacket)
{
	unsigned int size_bits;
	// SHORT_PACKET_BYTES = 12, so input must be ≤12 bytes (24 hex chars)
	std::string x = ArgosPacketBuilder::build_certification_packet("FFFFFFFFFFFFFFFFFFFF", size_bits); // 10 bytes
	CHECK_EQUAL(96, size_bits); // SHORT_PACKET_BITS = 96
	CHECK_EQUAL("\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\xFF\x00\x00"s, x); // Padded to 12 bytes
}

TEST(ArgosTxService, BuildShortGNSSPacket)
{
	unsigned int size_bits;
	GPSLogEntry e = make_gps_location(1, 12.3, 44.4, 1652105502);
	std::vector<GPSLogEntry*> v({&e});
	std::string x = ArgosPacketBuilder::build_gnss_packet(v, false, false, BaseDeltaTimeLoc::DELTA_T_10MIN, size_bits);
	CHECK_EQUAL("097166C6600781E00003FE58"s, Binascii::hexlify(x));

	x = ArgosPacketBuilder::build_gnss_packet(v, true, false, BaseDeltaTimeLoc::DELTA_T_10MIN, size_bits);
	CHECK_EQUAL("097166C6600781E00403FE58"s, Binascii::hexlify(x));

	x = ArgosPacketBuilder::build_gnss_packet(v, false, true, BaseDeltaTimeLoc::DELTA_T_10MIN, size_bits);
	CHECK_EQUAL("097166C6600781E00003FE5C"s, Binascii::hexlify(x));

	x = ArgosPacketBuilder::build_gnss_packet(v, true, true, BaseDeltaTimeLoc::DELTA_T_10MIN, size_bits);
	CHECK_EQUAL("097166C6600781E00403FE5C"s, Binascii::hexlify(x));
}


TEST(ArgosTxService, BuildLongGNSSPacket)
{
	unsigned int size_bits;
	GPSLogEntry e = make_gps_location(1, 12.3, 44.4, 1652105502);
	std::vector<GPSLogEntry*> v({&e, &e});
	// CRC8 and BCH are now handled by the satellite module — payload only
	std::string x = ArgosPacketBuilder::build_gnss_packet(v, false, false, BaseDeltaTimeLoc::DELTA_T_10MIN, size_bits);
	CHECK_EQUAL("4B8B3633003C0F0012C26C6600781E3FFFFFFFFFFFFFFFFFFFFF"s, Binascii::hexlify(x));
	v = {&e, &e, &e};
	x = ArgosPacketBuilder::build_gnss_packet(v, false, false, BaseDeltaTimeLoc::DELTA_T_10MIN, size_bits);
	CHECK_EQUAL("4B8B3633003C0F0012C26C6600781E0D8CC00F03C7FFFFFFFFFF"s, Binascii::hexlify(x));
	v = {&e, &e, &e, &e};
	x = ArgosPacketBuilder::build_gnss_packet(v, false, false, BaseDeltaTimeLoc::DELTA_T_10MIN, size_bits);
	CHECK_EQUAL("4B8B3633003C0F0012C26C6600781E0D8CC00F03C1B19801E078"s, Binascii::hexlify(x));
	x = ArgosPacketBuilder::build_gnss_packet(v, true, false, BaseDeltaTimeLoc::DELTA_T_10MIN, size_bits);
	CHECK_EQUAL("4B8B3633003C0F0032C26C6600781E0D8CC00F03C1B19801E078"s, Binascii::hexlify(x));
	x = ArgosPacketBuilder::build_gnss_packet(v, false, true, BaseDeltaTimeLoc::DELTA_T_10MIN, size_bits);
	CHECK_EQUAL("4B8B3633003C0F0012E26C6600781E0D8CC00F03C1B19801E078"s, Binascii::hexlify(x));
	x = ArgosPacketBuilder::build_gnss_packet(v, true, true, BaseDeltaTimeLoc::DELTA_T_10MIN, size_bits);
	CHECK_EQUAL("4B8B3633003C0F0032E26C6600781E0D8CC00F03C1B19801E078"s, Binascii::hexlify(x));
	x = ArgosPacketBuilder::build_gnss_packet(v, true, true, BaseDeltaTimeLoc::DELTA_T_30MIN, size_bits);
	CHECK_EQUAL("4B8B3633003C0F0032E66C6600781E0D8CC00F03C1B19801E078"s, Binascii::hexlify(x));
}


TEST(ArgosTxService, TxCounterIncrements)
{
	ArgosTxService serv(*mock_kineis);

	unsigned int counter;
	counter = configuration_store->read_param<unsigned int>(ParamID::TX_COUNTER);
	CHECK_EQUAL(0U, counter);

	double frequency = 900.22;
	fake_config_store->write_param(ParamID::ARGOS_FREQ, frequency);
	BaseArgosPower power = BaseArgosPower::POWER_1000_MW;
	fake_config_store->write_param(ParamID::ARGOS_POWER, power);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();

	mock_kineis->notify(KineisEventTxComplete({}));

	counter = configuration_store->read_param<unsigned int>(ParamID::TX_COUNTER);
	CHECK_EQUAL(1, counter);

	mock_kineis->notify(KineisEventTxComplete({}));

	counter = configuration_store->read_param<unsigned int>(ParamID::TX_COUNTER);
	CHECK_EQUAL(2, counter);
}

TEST(ArgosTxService, TimeSyncBurstPosFix)
{
	double frequency = 900.22;
	BaseArgosMode mode = BaseArgosMode::LEGACY;
	BaseArgosPower power = BaseArgosPower::POWER_1000_MW;
	BaseDepthPile depth_pile = BaseDepthPile::DEPTH_PILE_1;
	unsigned int argos_hexid = 0x01234567U;
	unsigned int lb_threshold = 0U;
	bool lb_en = false;
	unsigned int tr_nom = 10;

	fake_config_store->write_param(ParamID::ARGOS_DEPTH_PILE, depth_pile);
	fake_config_store->write_param(ParamID::ARGOS_FREQ, frequency);
	fake_config_store->write_param(ParamID::ARGOS_MODE, mode);
	fake_config_store->write_param(ParamID::ARGOS_HEXID, argos_hexid);
	fake_config_store->write_param(ParamID::LB_EN, lb_en);
	fake_config_store->write_param(ParamID::LB_THRESHOLD, lb_threshold);
	fake_config_store->write_param(ParamID::ARGOS_POWER, power);
	fake_config_store->write_param(ParamID::TR_NOM, tr_nom);

	ArgosTxService serv(*mock_kineis);

	std::time_t t = 1652105502000;
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();

	// First TX is time sync burst
	mock().expectOneCall("send").onObject(mock_kineis).withUnsignedIntParameter("mode", (unsigned int)KineisModulation::LDA2).
			withUnsignedIntParameter("size_bits", 96);

	inject_gps_location(1, 11.8768, -33.8232, t);
	system_scheduler->run();

	mock_kineis->notify(KineisEventTxComplete({}));

	mock().expectOneCall("stop_send").onObject(mock_kineis);
	serv.stop();

	// No time sync should be scheduled now
	bool time_sync_en = false;
	fake_config_store->write_param(ParamID::ARGOS_TIME_SYNC_BURST_EN, time_sync_en);
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();
	inject_gps_location(1, 11.8768, -33.8232, t);
	system_scheduler->run();
}

TEST(ArgosTxService, TimeSyncBurstNoPosFix)
{
	double frequency = 900.22;
	BaseArgosMode mode = BaseArgosMode::LEGACY;
	BaseArgosPower power = BaseArgosPower::POWER_1000_MW;
	BaseDepthPile depth_pile = BaseDepthPile::DEPTH_PILE_1;
	unsigned int argos_hexid = 0x01234567U;
	unsigned int lb_threshold = 0U;
	bool lb_en = false;
	unsigned int tr_nom = 10;

	fake_config_store->write_param(ParamID::ARGOS_DEPTH_PILE, depth_pile);
	fake_config_store->write_param(ParamID::ARGOS_FREQ, frequency);
	fake_config_store->write_param(ParamID::ARGOS_MODE, mode);
	fake_config_store->write_param(ParamID::ARGOS_HEXID, argos_hexid);
	fake_config_store->write_param(ParamID::LB_EN, lb_en);
	fake_config_store->write_param(ParamID::LB_THRESHOLD, lb_threshold);
	fake_config_store->write_param(ParamID::ARGOS_POWER, power);
	fake_config_store->write_param(ParamID::TR_NOM, tr_nom);

	ArgosTxService serv(*mock_kineis);

	std::time_t t = 1652105502000;
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();

	// Time sync burst with invalid GPS: should still send (with invalid position encoded as all-1s)
	mock().expectOneCall("send").onObject(mock_kineis).withUnsignedIntParameter("mode", (unsigned int)KineisModulation::LDA2).
			withUnsignedIntParameter("size_bits", 96);
	inject_gps_location(0, 11.8768, -33.8232, t);
	system_scheduler->run();

	mock_kineis->notify(KineisEventTxComplete({}));

	mock().expectOneCall("stop_send").onObject(mock_kineis);
	serv.stop();

	// No time sync should be scheduled now
	bool time_sync_en = false;
	fake_config_store->write_param(ParamID::ARGOS_TIME_SYNC_BURST_EN, time_sync_en);
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();
	inject_gps_location(1, 11.8768, -33.8232, t);
	system_scheduler->run();
}

TEST(ArgosTxService, TimeSyncBurstNoPosOrTimeFix)
{
	double frequency = 900.22;
	BaseArgosMode mode = BaseArgosMode::LEGACY;
	BaseArgosPower power = BaseArgosPower::POWER_1000_MW;
	BaseDepthPile depth_pile = BaseDepthPile::DEPTH_PILE_1;
	unsigned int argos_hexid = 0x01234567U;
	unsigned int lb_threshold = 0U;
	bool lb_en = false;
	unsigned int tr_nom = 10;

	fake_config_store->write_param(ParamID::ARGOS_DEPTH_PILE, depth_pile);
	fake_config_store->write_param(ParamID::ARGOS_FREQ, frequency);
	fake_config_store->write_param(ParamID::ARGOS_MODE, mode);
	fake_config_store->write_param(ParamID::ARGOS_HEXID, argos_hexid);
	fake_config_store->write_param(ParamID::LB_EN, lb_en);
	fake_config_store->write_param(ParamID::LB_THRESHOLD, lb_threshold);
	fake_config_store->write_param(ParamID::ARGOS_POWER, power);
	fake_config_store->write_param(ParamID::TR_NOM, tr_nom);

	ArgosTxService serv(*mock_kineis);

	std::time_t t = 0;

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();

	// Even without valid GPS or time fix, should still send (invalid position encoded as all-1s)
	mock().expectOneCall("send").onObject(mock_kineis).withUnsignedIntParameter("mode", (unsigned int)KineisModulation::LDA2).
			withUnsignedIntParameter("size_bits", 96);
	inject_gps_location(0, 11.8768, -33.8232, t);
	system_scheduler->run();
}

TEST(ArgosTxService, LegacyTxServiceInv)
{
	double frequency = 900.22;
	BaseArgosMode mode = BaseArgosMode::LEGACY;
	BaseArgosPower power = BaseArgosPower::POWER_1000_MW;
	BaseDepthPile depth_pile = BaseDepthPile::DEPTH_PILE_4;
	unsigned int argos_hexid = 0x01234567U;
	unsigned int lb_threshold = 0U;
	bool lb_en = false;
	unsigned int tr_nom = 10;
	bool time_sync_en = false;

	fake_config_store->write_param(ParamID::ARGOS_DEPTH_PILE, depth_pile);
	fake_config_store->write_param(ParamID::ARGOS_FREQ, frequency);
	fake_config_store->write_param(ParamID::ARGOS_MODE, mode);
	fake_config_store->write_param(ParamID::ARGOS_HEXID, argos_hexid);
	fake_config_store->write_param(ParamID::LB_EN, lb_en);
	fake_config_store->write_param(ParamID::LB_THRESHOLD, lb_threshold);
	fake_config_store->write_param(ParamID::ARGOS_POWER, power);
	fake_config_store->write_param(ParamID::TR_NOM, tr_nom);
	fake_config_store->write_param(ParamID::ARGOS_TIME_SYNC_BURST_EN, time_sync_en);

	ArgosTxService serv(*mock_kineis);

	std::time_t t = 1665137726000;
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();

	inject_gps_location(1, -2.117964, 51.376382, t/1000, false, 0, 162, 4424);
	//inject_gps_location(1, 11.8768, -33.8232, t);
	//inject_gps_location(1, 11.8768, -33.8232, t);
	//inject_gps_location(1, 11.8768, -33.8232, t);
	system_scheduler->run();

	// Subsequent TX will be long packets
	for (unsigned int i = 0; i < 1; i++) {
		mock().expectOneCall("send").onObject(mock_kineis).withUnsignedIntParameter("mode", (unsigned int)KineisModulation::LDA2).
				withUnsignedIntParameter("size_bits", 96);

		t += serv.get_last_schedule();
		fake_rtc->settime(t/1000);
		fake_timer->set_counter(t);
		system_scheduler->run();

		mock_kineis->notify(KineisEventTxComplete({}));
	}
}


TEST(ArgosTxService, LegacyTxLowBattery)
{
	double frequency = 900.22;
	BaseArgosMode mode = BaseArgosMode::LEGACY;
	BaseArgosPower power = BaseArgosPower::POWER_1000_MW;
	BaseDepthPile depth_pile = BaseDepthPile::DEPTH_PILE_4;
	unsigned int argos_hexid = 0x01234567U;
	unsigned int lb_threshold = 20U;
	bool lb_en = true;
	unsigned int tr_nom = 10;
	bool time_sync_en = false;

	fake_config_store->write_param(ParamID::ARGOS_DEPTH_PILE, depth_pile);
	fake_config_store->write_param(ParamID::ARGOS_FREQ, frequency);
	fake_config_store->write_param(ParamID::ARGOS_MODE, mode);
	fake_config_store->write_param(ParamID::ARGOS_HEXID, argos_hexid);
	fake_config_store->write_param(ParamID::LB_EN, lb_en);
	fake_config_store->write_param(ParamID::LB_THRESHOLD, lb_threshold);
	fake_config_store->write_param(ParamID::ARGOS_POWER, power);
	fake_config_store->write_param(ParamID::TR_NOM, tr_nom);
	fake_config_store->write_param(ParamID::ARGOS_TIME_SYNC_BURST_EN, time_sync_en);

	ArgosTxService serv(*mock_kineis);

	std::time_t t = 1652105502000;
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();

	// Force LB state
	fake_config_store->set_battery_level(10U);
	fake_config_store->set_is_battery_level_low(true);
	fake_battery_monitor->set_values(10, 4200, true, false);

	inject_gps_location(1, 11.8768, -33.8232, t);
	inject_gps_location(1, 11.8768, -33.8232, t);
	inject_gps_location(1, 11.8768, -33.8232, t);
	inject_gps_location(1, 11.8768, -33.8232, t);
	system_scheduler->run();

	// Subsequent TX will be short packets (depth pile 1 in LB mode)
	for (unsigned int i = 0; i < 10; i++) {
		mock().expectOneCall("send").onObject(mock_kineis).withUnsignedIntParameter("mode", (unsigned int)KineisModulation::LDA2).
				withUnsignedIntParameter("size_bits", 96);

		t += serv.get_last_schedule();
		fake_rtc->settime(t/1000);
		fake_timer->set_counter(t);
		system_scheduler->run();

		mock_kineis->notify(KineisEventTxComplete({}));
	}
}

TEST(ArgosTxService, LegacyTxOutOfZone)
{
	double frequency = 900.22;
	BaseArgosMode mode = BaseArgosMode::LEGACY;
	BaseArgosPower power = BaseArgosPower::POWER_1000_MW;
	BaseDepthPile depth_pile = BaseDepthPile::DEPTH_PILE_4;
	unsigned int argos_hexid = 0x01234567U;
	unsigned int lb_threshold = 0U;
	bool lb_en = false;
	unsigned int tr_nom = 10;
	bool time_sync_en = false;
	bool ooz_en = true;

	fake_config_store->write_param(ParamID::ARGOS_DEPTH_PILE, depth_pile);
	fake_config_store->write_param(ParamID::ARGOS_FREQ, frequency);
	fake_config_store->write_param(ParamID::ARGOS_MODE, mode);
	fake_config_store->write_param(ParamID::ARGOS_HEXID, argos_hexid);
	fake_config_store->write_param(ParamID::LB_EN, lb_en);
	fake_config_store->write_param(ParamID::LB_THRESHOLD, lb_threshold);
	fake_config_store->write_param(ParamID::ARGOS_POWER, power);
	fake_config_store->write_param(ParamID::TR_NOM, tr_nom);
	fake_config_store->write_param(ParamID::ARGOS_TIME_SYNC_BURST_EN, time_sync_en);
	fake_config_store->write_param(ParamID::ZONE_ENABLE_OUT_OF_ZONE_DETECTION_MODE, ooz_en);

	ArgosTxService serv(*mock_kineis);

	std::time_t t = 1652105502000;
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();

	inject_gps_location(1, 11.8768, -33.8232);
	inject_gps_location(1, 11.8768, -33.8232);
	inject_gps_location(1, 11.8768, -33.8232);
	inject_gps_location(1, 11.8768, -33.8232);
	system_scheduler->run();

	// Subsequent TX will be short packets (depth pile 1 in OOZ mode)
	for (unsigned int i = 0; i < 10; i++) {
		mock().expectOneCall("send").onObject(mock_kineis).withUnsignedIntParameter("mode", (unsigned int)KineisModulation::LDA2).
				withUnsignedIntParameter("size_bits", 96);

		t += serv.get_last_schedule();
		fake_rtc->settime(t/1000);
		fake_timer->set_counter(t);
		system_scheduler->run();

		mock_kineis->notify(KineisEventTxComplete({}));
	}
}

TEST(ArgosTxService, TxServiceCancelledByUnderwaterBeforeTx)
{
	double frequency = 900.22;
	BaseArgosMode mode = BaseArgosMode::LEGACY;
	BaseArgosPower power = BaseArgosPower::POWER_1000_MW;
	BaseDepthPile depth_pile = BaseDepthPile::DEPTH_PILE_4;
	unsigned int argos_hexid = 0x01234567U;
	unsigned int lb_threshold = 0U;
	bool lb_en = false;
	unsigned int tr_nom = 10;
	bool time_sync_en = false;

	fake_config_store->write_param(ParamID::ARGOS_DEPTH_PILE, depth_pile);
	fake_config_store->write_param(ParamID::ARGOS_FREQ, frequency);
	fake_config_store->write_param(ParamID::ARGOS_MODE, mode);
	fake_config_store->write_param(ParamID::ARGOS_HEXID, argos_hexid);
	fake_config_store->write_param(ParamID::LB_EN, lb_en);
	fake_config_store->write_param(ParamID::LB_THRESHOLD, lb_threshold);
	fake_config_store->write_param(ParamID::ARGOS_POWER, power);
	fake_config_store->write_param(ParamID::TR_NOM, tr_nom);
	fake_config_store->write_param(ParamID::ARGOS_TIME_SYNC_BURST_EN, time_sync_en);

	ArgosTxService serv(*mock_kineis);

	std::time_t t = 1652105502000;
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();

	inject_gps_location(1, 11.8768, -33.8232, t);
	inject_gps_location(1, 11.8768, -33.8232, t);
	inject_gps_location(1, 11.8768, -33.8232, t);
	inject_gps_location(1, 11.8768, -33.8232, t);
	system_scheduler->run();

	CHECK_FALSE(Service::SCHEDULE_DISABLED == serv.get_last_schedule());

	// Inject UW event
	mock().expectOneCall("stop_send").onObject(mock_kineis);
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 0);
	notify_underwater_state(true);

	CHECK_TRUE(Service::SCHEDULE_DISABLED == serv.get_last_schedule());

	// Inject surfaced event
	notify_underwater_state(false);

	CHECK_FALSE(Service::SCHEDULE_DISABLED == serv.get_last_schedule());

	// Next schedule should be equal to dry time before TX since the last TX was deferred
	unsigned int dry_time_before_tx = fake_config_store->read_param<unsigned int>(ParamID::DRY_TIME_BEFORE_TX);
	CHECK_EQUAL(dry_time_before_tx*1000, serv.get_last_schedule());

	// Should now transmit (TCXO warmup skipped on first TX after submerge)
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 0);
	mock().expectOneCall("send").onObject(mock_kineis).withUnsignedIntParameter("mode", (unsigned int)KineisModulation::LDA2).
			withUnsignedIntParameter("size_bits", 224);
	t += serv.get_last_schedule();
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);
	system_scheduler->run();
}

TEST(ArgosTxService, TxServiceCancelledDuringTx)
{
	double frequency = 900.22;
	BaseArgosMode mode = BaseArgosMode::LEGACY;
	BaseArgosPower power = BaseArgosPower::POWER_1000_MW;
	BaseDepthPile depth_pile = BaseDepthPile::DEPTH_PILE_4;
	unsigned int argos_hexid = 0x01234567U;
	unsigned int lb_threshold = 0U;
	bool lb_en = false;
	unsigned int tr_nom = 10;
	bool time_sync_en = false;

	fake_config_store->write_param(ParamID::ARGOS_DEPTH_PILE, depth_pile);
	fake_config_store->write_param(ParamID::ARGOS_FREQ, frequency);
	fake_config_store->write_param(ParamID::ARGOS_MODE, mode);
	fake_config_store->write_param(ParamID::ARGOS_HEXID, argos_hexid);
	fake_config_store->write_param(ParamID::LB_EN, lb_en);
	fake_config_store->write_param(ParamID::LB_THRESHOLD, lb_threshold);
	fake_config_store->write_param(ParamID::ARGOS_POWER, power);
	fake_config_store->write_param(ParamID::TR_NOM, tr_nom);
	fake_config_store->write_param(ParamID::ARGOS_TIME_SYNC_BURST_EN, time_sync_en);

	ArgosTxService serv(*mock_kineis);

	std::time_t t = 1652105502000;
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();

	inject_gps_location(1, 11.8768, -33.8232, t);
	inject_gps_location(1, 11.8768, -33.8232, t);
	inject_gps_location(1, 11.8768, -33.8232, t);
	inject_gps_location(1, 11.8768, -33.8232, t);
	system_scheduler->run();

	mock().expectOneCall("send").onObject(mock_kineis).withUnsignedIntParameter("mode", (unsigned int)KineisModulation::LDA2).
			withUnsignedIntParameter("size_bits", 224);

	// TX should start
	t += serv.get_last_schedule();
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);
	system_scheduler->run();

	// Inject UW event
	mock().expectOneCall("stop_send").onObject(mock_kineis);
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 0);
	notify_underwater_state(true);

	// Inject surfaced event
	notify_underwater_state(false);

	// Next schedule should be equal to dry time before TX since the last TX was deferred
	unsigned int dry_time_before_tx = fake_config_store->read_param<unsigned int>(ParamID::DRY_TIME_BEFORE_TX);
	CHECK_EQUAL(dry_time_before_tx*1000, serv.get_last_schedule());

	// Should now transmit (TCXO warmup skipped on first TX after submerge)
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 0);
	mock().expectOneCall("send").onObject(mock_kineis).withUnsignedIntParameter("mode", (unsigned int)KineisModulation::LDA2).
			withUnsignedIntParameter("size_bits", 224);
	t += serv.get_last_schedule();
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);
	system_scheduler->run();
	// TX complete restores TCXO warmup
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	mock_kineis->notify(KineisEventTxComplete({}));
}


TEST(ArgosTxService, LegacyTxServiceDepthPile1)
{
	double frequency = 900.22;
	BaseArgosMode mode = BaseArgosMode::LEGACY;
	BaseArgosPower power = BaseArgosPower::POWER_1000_MW;
	BaseDepthPile depth_pile = BaseDepthPile::DEPTH_PILE_1;
	unsigned int argos_hexid = 0x01234567U;
	unsigned int lb_threshold = 0U;
	bool lb_en = false;
	unsigned int tr_nom = 10;
	bool time_sync_en = false;

	fake_config_store->write_param(ParamID::ARGOS_DEPTH_PILE, depth_pile);
	fake_config_store->write_param(ParamID::ARGOS_FREQ, frequency);
	fake_config_store->write_param(ParamID::ARGOS_MODE, mode);
	fake_config_store->write_param(ParamID::ARGOS_HEXID, argos_hexid);
	fake_config_store->write_param(ParamID::LB_EN, lb_en);
	fake_config_store->write_param(ParamID::LB_THRESHOLD, lb_threshold);
	fake_config_store->write_param(ParamID::ARGOS_POWER, power);
	fake_config_store->write_param(ParamID::TR_NOM, tr_nom);
	fake_config_store->write_param(ParamID::ARGOS_TIME_SYNC_BURST_EN, time_sync_en);

	ArgosTxService serv(*mock_kineis);

	std::time_t t = 1652105502000;
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();

	inject_gps_location(1, 11.8768, -33.8232, t);
	inject_gps_location(1, 11.8768, -33.8232, t);
	inject_gps_location(1, 11.8768, -33.8232, t);
	inject_gps_location(1, 11.8768, -33.8232, t);
	system_scheduler->run();

	// Subsequent TX will be short packets
	for (unsigned int i = 0; i < 10; i++) {
		mock().expectOneCall("send").onObject(mock_kineis).withUnsignedIntParameter("mode", (unsigned int)KineisModulation::LDA2).
				withUnsignedIntParameter("size_bits", 96);

		t += serv.get_last_schedule();
		fake_rtc->settime(t/1000);
		fake_timer->set_counter(t);
		system_scheduler->run();

		mock_kineis->notify(KineisEventTxComplete({}));
	}
}

TEST(ArgosTxService, UnderwaterFor24HoursBeforeTx)
{
	double frequency = 900.22;
	BaseArgosMode mode = BaseArgosMode::LEGACY;
	BaseArgosPower power = BaseArgosPower::POWER_1000_MW;
	BaseDepthPile depth_pile = BaseDepthPile::DEPTH_PILE_4;
	unsigned int argos_hexid = 0x01234567U;
	unsigned int lb_threshold = 0U;
	bool lb_en = false;
	unsigned int tr_nom = 10;
	bool time_sync_en = false;

	fake_config_store->write_param(ParamID::ARGOS_DEPTH_PILE, depth_pile);
	fake_config_store->write_param(ParamID::ARGOS_FREQ, frequency);
	fake_config_store->write_param(ParamID::ARGOS_MODE, mode);
	fake_config_store->write_param(ParamID::ARGOS_HEXID, argos_hexid);
	fake_config_store->write_param(ParamID::LB_EN, lb_en);
	fake_config_store->write_param(ParamID::LB_THRESHOLD, lb_threshold);
	fake_config_store->write_param(ParamID::ARGOS_POWER, power);
	fake_config_store->write_param(ParamID::TR_NOM, tr_nom);
	fake_config_store->write_param(ParamID::ARGOS_TIME_SYNC_BURST_EN, time_sync_en);

	ArgosTxService serv(*mock_kineis);

	std::time_t t = 1652105502000;
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();

	inject_gps_location(1, 11.8768, -33.8232, t);
	inject_gps_location(1, 11.8768, -33.8232, t);
	inject_gps_location(1, 11.8768, -33.8232, t);
	inject_gps_location(1, 11.8768, -33.8232, t);
	system_scheduler->run();

	// Inject UW event
	mock().expectOneCall("stop_send").onObject(mock_kineis);
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 0);
	notify_underwater_state(true);

	// Keep UW for 25 hours
	t += 50 * 3600 * 1000;
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);
	system_scheduler->run();

	// Inject surfaced event
	notify_underwater_state(false);

	// Next schedule should be equal to dry time before TX since the last TX was deferred
	unsigned int dry_time_before_tx = fake_config_store->read_param<unsigned int>(ParamID::DRY_TIME_BEFORE_TX);
	CHECK_EQUAL(dry_time_before_tx*1000, serv.get_last_schedule());

	// Should now transmit (TCXO warmup skipped on first TX after submerge)
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 0);
	mock().expectOneCall("send").onObject(mock_kineis).withUnsignedIntParameter("mode", (unsigned int)KineisModulation::LDA2).
			withUnsignedIntParameter("size_bits", 224);
	t += serv.get_last_schedule();
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);
	system_scheduler->run();
}

TEST(ArgosTxService, LastTxIsUpdated)
{
	double frequency = 900.22;
	BaseArgosMode mode = BaseArgosMode::LEGACY;
	BaseArgosPower power = BaseArgosPower::POWER_1000_MW;
	BaseDepthPile depth_pile = BaseDepthPile::DEPTH_PILE_1;
	unsigned int argos_hexid = 0x01234567U;
	unsigned int lb_threshold = 0U;
	bool lb_en = false;
	unsigned int tr_nom = 10;

	fake_config_store->write_param(ParamID::ARGOS_DEPTH_PILE, depth_pile);
	fake_config_store->write_param(ParamID::ARGOS_FREQ, frequency);
	fake_config_store->write_param(ParamID::ARGOS_MODE, mode);
	fake_config_store->write_param(ParamID::ARGOS_HEXID, argos_hexid);
	fake_config_store->write_param(ParamID::LB_EN, lb_en);
	fake_config_store->write_param(ParamID::LB_THRESHOLD, lb_threshold);
	fake_config_store->write_param(ParamID::ARGOS_POWER, power);
	fake_config_store->write_param(ParamID::TR_NOM, tr_nom);

	ArgosTxService serv(*mock_kineis);

	std::time_t t = 1652105502000;
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();

	// First TX is time sync burst
	mock().expectOneCall("send").onObject(mock_kineis).withUnsignedIntParameter("mode", (unsigned int)KineisModulation::LDA2).
			withUnsignedIntParameter("size_bits", 96);

	inject_gps_location(1, 11.8768, -33.8232, t);
	system_scheduler->run();

	mock_kineis->notify(KineisEventTxComplete({}));

	std::time_t last_tx = fake_config_store->read_param<std::time_t>(ParamID::LAST_TX);
	CHECK_EQUAL(1652105502U, (unsigned int)last_tx);

	mock().expectOneCall("stop_send").onObject(mock_kineis);
	serv.stop();
}

TEST(ArgosTxService, DPHardFaultPartialDepthPile)
{
	DepthPile<GPSLogEntry> dp;
	std::vector<GPSLogEntry*> v;
	GPSLogEntry e;

	// Load up full depth pile with burst count of 0
	for (unsigned int i = 0; i < 6; i++) {
		e.info.day = i;
		dp.store(e, 99999999);
	}

	v = dp.retrieve((unsigned int)BaseDepthPile::DEPTH_PILE_16);
	CHECK_EQUAL(4, v.size());
	v = dp.retrieve((unsigned int)BaseDepthPile::DEPTH_PILE_16);
	CHECK_EQUAL(2, v.size());
}

TEST(ArgosTxService, UnderwaterFor24HoursDryTimeZero)
{
	double frequency = 900.22;
	BaseArgosMode mode = BaseArgosMode::LEGACY;
	BaseArgosPower power = BaseArgosPower::POWER_1000_MW;
	BaseDepthPile depth_pile = BaseDepthPile::DEPTH_PILE_1;
	unsigned int argos_hexid = 0x01234567U;
	unsigned int lb_threshold = 0U;
	bool lb_en = false;
	unsigned int tr_nom = 10;
	bool time_sync_en = false;
	unsigned int dry_time_before_tx = 0;

	fake_config_store->write_param(ParamID::ARGOS_DEPTH_PILE, depth_pile);
	fake_config_store->write_param(ParamID::ARGOS_FREQ, frequency);
	fake_config_store->write_param(ParamID::ARGOS_MODE, mode);
	fake_config_store->write_param(ParamID::ARGOS_HEXID, argos_hexid);
	fake_config_store->write_param(ParamID::LB_EN, lb_en);
	fake_config_store->write_param(ParamID::LB_THRESHOLD, lb_threshold);
	fake_config_store->write_param(ParamID::ARGOS_POWER, power);
	fake_config_store->write_param(ParamID::TR_NOM, tr_nom);
	fake_config_store->write_param(ParamID::ARGOS_TIME_SYNC_BURST_EN, time_sync_en);
	fake_config_store->write_param(ParamID::DRY_TIME_BEFORE_TX, dry_time_before_tx);

	ArgosTxService serv(*mock_kineis);

	std::time_t t = 1652105502000;
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();

	inject_gps_location(1, 11.8768, -33.8232, t);

	// Do initial transmit
	mock().expectOneCall("send").onObject(mock_kineis).ignoreOtherParameters();
	t += serv.get_last_schedule();
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);
	system_scheduler->run();

	mock_kineis->notify(KineisEventTxComplete({}));

	// Inject UW event
	mock().expectOneCall("stop_send").onObject(mock_kineis);
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 0);
	notify_underwater_state(true);

	// Keep UW for 25 hours
	t += 25 * 3600 * 1000;
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);
	system_scheduler->run();

	// Inject surfaced event
	notify_underwater_state(false);

	CHECK_EQUAL(0, serv.get_last_schedule());

	// Should now transmit (TCXO warmup skipped on first TX after submerge)
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 0);
	mock().expectOneCall("send").onObject(mock_kineis).ignoreOtherParameters();
	t += serv.get_last_schedule();
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);
	system_scheduler->run();
}

TEST(ArgosTxService, SurfacingBurstDopplerPhase)
{
	// Test SURFACING_BURST mode: verify progressive Doppler intervals
	BaseArgosMode mode = BaseArgosMode::SURFACING_BURST;
	unsigned int dry_time = 1; // 1s dry time to separate scheduling from notification

	fake_config_store->write_param(ParamID::ARGOS_DEPTH_PILE, BaseDepthPile::DEPTH_PILE_1);
	fake_config_store->write_param(ParamID::ARGOS_MODE, mode);
	fake_config_store->write_param(ParamID::ARGOS_HEXID, 0x01234567U);
	fake_config_store->write_param(ParamID::TR_NOM, 60U);
	fake_config_store->write_param(ParamID::ARGOS_TIME_SYNC_BURST_EN, (bool)false);
	fake_config_store->write_param(ParamID::UNDERWATER_EN, (bool)true);
	fake_config_store->write_param(ParamID::ARGOS_TX_JITTER_EN, (bool)false);
	fake_config_store->write_param(ParamID::DRY_TIME_BEFORE_TX, dry_time);
	fake_config_store->write_param(ParamID::SURFACING_BURST_INIT_S, 5U);
	fake_config_store->write_param(ParamID::SURFACING_BURST_STEP_S, 10U);
	fake_config_store->write_param(ParamID::SURFACING_BURST_MAX_S, 60U);

	ArgosTxService serv(*mock_kineis);

	std::time_t t = 1652105502000;
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();

	// Go underwater
	mock().expectOneCall("stop_send").onObject(mock_kineis);
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 0);
	notify_underwater_state(true);

	// Advance time underwater
	t += 60000;
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);

	// Surface — first Doppler scheduled immediately (0ms)
	notify_underwater_state(false);
	CHECK_EQUAL(0U, serv.get_last_schedule());

	// Fire first Doppler TX
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 0);
	mock().expectOneCall("send").onObject(mock_kineis).withUnsignedIntParameter("mode", (unsigned int)KineisModulation::LDA2).
			withUnsignedIntParameter("size_bits", 24);
	system_scheduler->run();

	// Complete TX → reschedule with INIT interval (5s)
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	mock_kineis->notify(KineisEventTxComplete({}));
	CHECK_EQUAL(5000U, serv.get_last_schedule());

	// Advance and fire second Doppler
	mock().expectOneCall("send").onObject(mock_kineis).withUnsignedIntParameter("mode", (unsigned int)KineisModulation::LDA2).
			withUnsignedIntParameter("size_bits", 24);
	t += serv.get_last_schedule();
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);
	system_scheduler->run();

	mock_kineis->notify(KineisEventTxComplete({}));
	// Third message: init + 1*step = 5 + 10 = 15s
	CHECK_EQUAL(15000U, serv.get_last_schedule());
}

TEST(ArgosTxService, SurfacingBurstSwitchToGNSS)
{
	// Test switch from Doppler to GNSS when GPS fix arrives
	fake_config_store->write_param(ParamID::ARGOS_DEPTH_PILE, BaseDepthPile::DEPTH_PILE_1);
	fake_config_store->write_param(ParamID::ARGOS_MODE, BaseArgosMode::SURFACING_BURST);
	fake_config_store->write_param(ParamID::ARGOS_HEXID, 0x01234567U);
	fake_config_store->write_param(ParamID::TR_NOM, 60U);
	fake_config_store->write_param(ParamID::ARGOS_TIME_SYNC_BURST_EN, (bool)false);
	fake_config_store->write_param(ParamID::UNDERWATER_EN, (bool)true);
	fake_config_store->write_param(ParamID::ARGOS_TX_JITTER_EN, (bool)false);
	fake_config_store->write_param(ParamID::DRY_TIME_BEFORE_TX, 1U);
	fake_config_store->write_param(ParamID::SURFACING_BURST_INIT_S, 5U);
	fake_config_store->write_param(ParamID::SURFACING_BURST_STEP_S, 10U);
	fake_config_store->write_param(ParamID::SURFACING_BURST_MAX_S, 60U);

	ArgosTxService serv(*mock_kineis);

	std::time_t t = 1652105502000;
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();

	// Dive
	mock().expectOneCall("stop_send").onObject(mock_kineis);
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 0);
	notify_underwater_state(true);

	t += 60000;
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);

	// Surface
	notify_underwater_state(false);

	// Fire first Doppler TX
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 0);
	mock().expectOneCall("send").onObject(mock_kineis).withUnsignedIntParameter("mode", (unsigned int)KineisModulation::LDA2).
			withUnsignedIntParameter("size_bits", 24);
	system_scheduler->run();

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	mock_kineis->notify(KineisEventTxComplete({}));

	// Inject GPS fix → should switch to GNSS phase and reschedule
	inject_gps_location(true, 11.8768, -33.8232, t/1000);
	CHECK_FALSE(Service::SCHEDULE_DISABLED == serv.get_last_schedule());

	// Fire TX → should be GNSS short packet (96 bits), not Doppler (24 bits)
	mock().expectOneCall("send").onObject(mock_kineis).withUnsignedIntParameter("mode", (unsigned int)KineisModulation::LDA2).
			withUnsignedIntParameter("size_bits", 96);
	t += serv.get_last_schedule();
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);
	system_scheduler->run();
}

TEST(ArgosTxService, SurfacingBurstResetOnDive)
{
	// Test that diving resets the burst state
	fake_config_store->write_param(ParamID::ARGOS_DEPTH_PILE, BaseDepthPile::DEPTH_PILE_1);
	fake_config_store->write_param(ParamID::ARGOS_MODE, BaseArgosMode::SURFACING_BURST);
	fake_config_store->write_param(ParamID::ARGOS_HEXID, 0x01234567U);
	fake_config_store->write_param(ParamID::TR_NOM, 60U);
	fake_config_store->write_param(ParamID::ARGOS_TIME_SYNC_BURST_EN, (bool)false);
	fake_config_store->write_param(ParamID::UNDERWATER_EN, (bool)true);
	fake_config_store->write_param(ParamID::ARGOS_TX_JITTER_EN, (bool)false);
	fake_config_store->write_param(ParamID::DRY_TIME_BEFORE_TX, 1U);
	fake_config_store->write_param(ParamID::SURFACING_BURST_INIT_S, 5U);
	fake_config_store->write_param(ParamID::SURFACING_BURST_STEP_S, 10U);
	fake_config_store->write_param(ParamID::SURFACING_BURST_MAX_S, 60U);

	ArgosTxService serv(*mock_kineis);

	std::time_t t = 1652105502000;
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();

	// First dive/surface
	mock().expectOneCall("stop_send").onObject(mock_kineis);
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 0);
	notify_underwater_state(true);
	t += 60000;
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);
	notify_underwater_state(false);
	CHECK_EQUAL(0U, serv.get_last_schedule());  // First burst msg immediate

	// Fire first TX
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 0);
	mock().expectOneCall("send").onObject(mock_kineis).withUnsignedIntParameter("mode", (unsigned int)KineisModulation::LDA2).
			withUnsignedIntParameter("size_bits", 24);
	system_scheduler->run();

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	mock_kineis->notify(KineisEventTxComplete({}));
	CHECK_EQUAL(5000U, serv.get_last_schedule());

	// Dive again before second TX
	mock().expectOneCall("stop_send").onObject(mock_kineis);
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 0);
	notify_underwater_state(true);
	CHECK_TRUE(Service::SCHEDULE_DISABLED == serv.get_last_schedule());

	// Surface again → burst restarts from 0
	t += 60000;
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);
	notify_underwater_state(false);
	CHECK_EQUAL(0U, serv.get_last_schedule());  // Immediate again (reset)
}

TEST(ArgosTxService, SurfacingBurstFirstGnssTxImmediate)
{
	// Test that the first GNSS TX after fix is immediate (delay=0), not TR_NOM
	fake_config_store->write_param(ParamID::ARGOS_DEPTH_PILE, BaseDepthPile::DEPTH_PILE_1);
	fake_config_store->write_param(ParamID::ARGOS_MODE, BaseArgosMode::SURFACING_BURST);
	fake_config_store->write_param(ParamID::ARGOS_HEXID, 0x01234567U);
	fake_config_store->write_param(ParamID::TR_NOM, 60U);
	fake_config_store->write_param(ParamID::ARGOS_TIME_SYNC_BURST_EN, (bool)false);
	fake_config_store->write_param(ParamID::UNDERWATER_EN, (bool)true);
	fake_config_store->write_param(ParamID::ARGOS_TX_JITTER_EN, (bool)false);
	fake_config_store->write_param(ParamID::DRY_TIME_BEFORE_TX, 0U);
	fake_config_store->write_param(ParamID::SURFACING_BURST_INIT_S, 5U);
	fake_config_store->write_param(ParamID::SURFACING_BURST_STEP_S, 5U);
	fake_config_store->write_param(ParamID::SURFACING_BURST_MAX_S, 60U);

	ArgosTxService serv(*mock_kineis);

	std::time_t t = 1652105502000;
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();

	// Dive
	mock().expectOneCall("stop_send").onObject(mock_kineis);
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 0);
	notify_underwater_state(true);
	t += 60000;
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);

	// Surface
	notify_underwater_state(false);
	CHECK_EQUAL(0U, serv.get_last_schedule());  // Doppler #1 immediate

	// Fire Doppler TX #1
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 0);
	mock().expectOneCall("send").onObject(mock_kineis).withUnsignedIntParameter("mode", (unsigned int)KineisModulation::LDA2).
			withUnsignedIntParameter("size_bits", 24);
	system_scheduler->run();

	// TX complete
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	mock_kineis->notify(KineisEventTxComplete({}));

	// Inject GPS fix → switch to GNSS phase
	inject_gps_location(true, 11.8768, -33.8232, t/1000);

	// GNSS TX #1 should be IMMEDIATE (0 delay), not 60s
	CHECK_EQUAL(0U, serv.get_last_schedule());

	// Fire GNSS TX
	mock().expectOneCall("send").onObject(mock_kineis).withUnsignedIntParameter("mode", (unsigned int)KineisModulation::LDA2).
			withUnsignedIntParameter("size_bits", 96);
	t += serv.get_last_schedule();
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);
	system_scheduler->run();
}

TEST(ArgosTxService, SurfacingBurstDopplerMaxMsg)
{
	// Test that Doppler phase stops after SURFACING_BURST_MAX_MSG
	fake_config_store->write_param(ParamID::ARGOS_DEPTH_PILE, BaseDepthPile::DEPTH_PILE_1);
	fake_config_store->write_param(ParamID::ARGOS_MODE, BaseArgosMode::SURFACING_BURST);
	fake_config_store->write_param(ParamID::ARGOS_HEXID, 0x01234567U);
	fake_config_store->write_param(ParamID::TR_NOM, 60U);
	fake_config_store->write_param(ParamID::ARGOS_TIME_SYNC_BURST_EN, (bool)false);
	fake_config_store->write_param(ParamID::UNDERWATER_EN, (bool)true);
	fake_config_store->write_param(ParamID::ARGOS_TX_JITTER_EN, (bool)false);
	fake_config_store->write_param(ParamID::DRY_TIME_BEFORE_TX, 0U);
	fake_config_store->write_param(ParamID::SURFACING_BURST_INIT_S, 5U);
	fake_config_store->write_param(ParamID::SURFACING_BURST_STEP_S, 0U);
	fake_config_store->write_param(ParamID::SURFACING_BURST_MAX_S, 60U);
	fake_config_store->write_param(ParamID::SURFACING_BURST_MAX_MSG, 2U);  // Max 2 Doppler

	ArgosTxService serv(*mock_kineis);

	std::time_t t = 1652105502000;
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();

	// Dive and surface
	mock().expectOneCall("stop_send").onObject(mock_kineis);
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 0);
	notify_underwater_state(true);
	t += 60000;
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);
	notify_underwater_state(false);

	// Doppler #1 (immediate)
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 0);
	mock().expectOneCall("send").onObject(mock_kineis).ignoreOtherParameters();
	system_scheduler->run();
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	mock_kineis->notify(KineisEventTxComplete({}));

	// Doppler #2
	mock().expectOneCall("send").onObject(mock_kineis).ignoreOtherParameters();
	t += serv.get_last_schedule();
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);
	system_scheduler->run();
	mock_kineis->notify(KineisEventTxComplete({}));

	// After 2 Doppler, burst should stop (SCHEDULE_DISABLED)
	CHECK_TRUE(Service::SCHEDULE_DISABLED == serv.get_last_schedule());
}

TEST(ArgosTxService, SurfacingBurstAdaptiveModulationPreSwitch)
{
	// Test that after TX complete in adaptive mode, pre-switch to VLDA4 happens
	fake_config_store->write_param(ParamID::ARGOS_DEPTH_PILE, BaseDepthPile::DEPTH_PILE_1);
	fake_config_store->write_param(ParamID::ARGOS_MODE, BaseArgosMode::SURFACING_BURST);
	fake_config_store->write_param(ParamID::ARGOS_HEXID, 0x01234567U);
	fake_config_store->write_param(ParamID::TR_NOM, 60U);
	fake_config_store->write_param(ParamID::ARGOS_TIME_SYNC_BURST_EN, (bool)false);
	fake_config_store->write_param(ParamID::UNDERWATER_EN, (bool)true);
	fake_config_store->write_param(ParamID::ARGOS_TX_JITTER_EN, (bool)false);
	fake_config_store->write_param(ParamID::DRY_TIME_BEFORE_TX, 0U);
	fake_config_store->write_param(ParamID::SURFACING_BURST_INIT_S, 5U);
	fake_config_store->write_param(ParamID::SURFACING_BURST_STEP_S, 5U);
	fake_config_store->write_param(ParamID::SURFACING_BURST_MAX_S, 60U);
	fake_config_store->write_param(ParamID::ARGOS_ADAPTIVE_MODULATION, (bool)true);
	fake_config_store->write_param(ParamID::ARGOS_RADIOCONF_VLDA4, std::string("82d07f9d9ce081ee4492983672d75493"));
	fake_config_store->write_param(ParamID::ARGOS_RADIOCONF_LDK, std::string("03921fb104b92859209b18abd009de96"));

	ArgosTxService serv(*mock_kineis);

	std::time_t t = 1652105502000;
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();

	// Dive
	mock().expectOneCall("stop_send").onObject(mock_kineis);
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 0);
	notify_underwater_state(true);
	t += 60000;
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);

	// Surface → Doppler VLDA4
	notify_underwater_state(false);

	// Fire Doppler TX (VLDA4 mode)
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 0);
	mock().expectOneCall("switch_modulation").onObject(mock_kineis)
		.withUnsignedIntParameter("mode", (unsigned int)KineisModulation::VLDA4)
		.withStringParameter("rconf", "82d07f9d9ce081ee4492983672d75493");
	mock().expectOneCall("send").onObject(mock_kineis).withUnsignedIntParameter("mode", (unsigned int)KineisModulation::VLDA4).
			withUnsignedIntParameter("size_bits", 24);
	system_scheduler->run();

	// TX complete → TCXO restored, no pre-switch needed (already VLDA4)
	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	mock_kineis->notify(KineisEventTxComplete({}));

	// Inject GPS fix
	inject_gps_location(true, 11.8768, -33.8232, t/1000);

	// Fire GNSS TX → should switch to LDK
	mock().expectOneCall("switch_modulation").onObject(mock_kineis)
		.withUnsignedIntParameter("mode", (unsigned int)KineisModulation::LDK)
		.withStringParameter("rconf", "03921fb104b92859209b18abd009de96");
	mock().expectOneCall("send").onObject(mock_kineis).withUnsignedIntParameter("mode", (unsigned int)KineisModulation::LDK).
			withUnsignedIntParameter("size_bits", 96);
	t += serv.get_last_schedule();
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);
	system_scheduler->run();

	// TX complete → should pre-switch back to VLDA4
	mock().expectOneCall("switch_modulation").onObject(mock_kineis)
		.withUnsignedIntParameter("mode", (unsigned int)KineisModulation::VLDA4)
		.withStringParameter("rconf", "82d07f9d9ce081ee4492983672d75493");
	mock_kineis->notify(KineisEventTxComplete({}));

	// Verify device is back to VLDA4
	CHECK_EQUAL((unsigned int)KineisModulation::VLDA4, (unsigned int)mock_kineis->get_current_modulation());
}

TEST(ArgosTxService, DepthPileManagerSensorTimeout)
{
	bool enable = true;
	BaseSensorEnableTxMode mode = BaseSensorEnableTxMode::ONESHOT;
	fake_config_store->write_param(ParamID::ALS_SENSOR_ENABLE, enable);
	fake_config_store->write_param(ParamID::ALS_SENSOR_ENABLE_TX_MODE, mode);

	std::time_t t = 1652105502000;
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);

	DepthPileManager man;

	ServiceEvent e;
	GPSLogEntry log;
	log.info.valid = true;
	e.event_source = ServiceIdentifier::GNSS_SENSOR;
	e.event_type = ServiceEventType::SERVICE_ACTIVE;
	man.notify_peer_event(e);

	e.event_data = log;
	e.event_source = ServiceIdentifier::GNSS_SENSOR;
	e.event_type = ServiceEventType::SERVICE_LOG_UPDATED;
	man.notify_peer_event(e);

	e.event_type = ServiceEventType::SERVICE_INACTIVE;
	man.notify_peer_event(e);

	CHECK_FALSE(man.eligible());

	// Sensor timeout
	t += 2000;
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);
	system_scheduler->run();

	CHECK_TRUE(man.eligible());
}

TEST(ArgosTxService, DepthPileManagerSensorRx)
{
	bool enable = true;
	BaseSensorEnableTxMode mode = BaseSensorEnableTxMode::ONESHOT;
	fake_config_store->write_param(ParamID::ALS_SENSOR_ENABLE, enable);
	fake_config_store->write_param(ParamID::ALS_SENSOR_ENABLE_TX_MODE, mode);

	std::time_t t = 1652105502000;
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);

	DepthPileManager man;

	ServiceEvent e;
	GPSLogEntry log;
	log.info.valid = true;
	ServiceSensorData sensor;
	e.event_source = ServiceIdentifier::GNSS_SENSOR;
	e.event_type = ServiceEventType::SERVICE_ACTIVE;
	man.notify_peer_event(e);

	e.event_data = log;
	e.event_source = ServiceIdentifier::GNSS_SENSOR;
	e.event_type = ServiceEventType::SERVICE_LOG_UPDATED;
	man.notify_peer_event(e);

	e.event_data = sensor;
	e.event_source = ServiceIdentifier::ALS_SENSOR;
	e.event_type = ServiceEventType::SERVICE_LOG_UPDATED;
	man.notify_peer_event(e);

	e.event_type = ServiceEventType::SERVICE_INACTIVE;
	man.notify_peer_event(e);

	CHECK_TRUE(man.eligible());
}

TEST(ArgosTxService, DepthPileManagerTestSensorValueConversion)
{
	bool enable = true;
	BaseSensorEnableTxMode mode = BaseSensorEnableTxMode::ONESHOT;
	fake_config_store->write_param(ParamID::ALS_SENSOR_ENABLE, enable);
	fake_config_store->write_param(ParamID::ALS_SENSOR_ENABLE_TX_MODE, mode);
	fake_config_store->write_param(ParamID::PH_SENSOR_ENABLE, enable);
	fake_config_store->write_param(ParamID::PH_SENSOR_ENABLE_TX_MODE, mode);
	fake_config_store->write_param(ParamID::PRESSURE_SENSOR_ENABLE, enable);
	fake_config_store->write_param(ParamID::PRESSURE_SENSOR_ENABLE_TX_MODE, mode);
	fake_config_store->write_param(ParamID::SEA_TEMP_SENSOR_ENABLE, enable);
	fake_config_store->write_param(ParamID::SEA_TEMP_SENSOR_ENABLE_TX_MODE, mode);

	DepthPileManager man;

	ServiceEvent e;
	GPSLogEntry log;
	log.info.valid = true;
	ServiceSensorData sensor;
	e.event_source = ServiceIdentifier::GNSS_SENSOR;
	e.event_type = ServiceEventType::SERVICE_ACTIVE;
	man.notify_peer_event(e);

	e.event_data = log;
	e.event_source = ServiceIdentifier::GNSS_SENSOR;
	e.event_type = ServiceEventType::SERVICE_LOG_UPDATED;
	man.notify_peer_event(e);

	sensor.port[0] = 65535;
	e.event_data = sensor;
	e.event_source = ServiceIdentifier::ALS_SENSOR;
	e.event_type = ServiceEventType::SERVICE_LOG_UPDATED;
	man.notify_peer_event(e);

	sensor.port[0] = 12.4;
	sensor.port[1] = -38.0;
	e.event_data = sensor;
	e.event_source = ServiceIdentifier::PRESSURE_SENSOR;
	e.event_type = ServiceEventType::SERVICE_LOG_UPDATED;
	man.notify_peer_event(e);

	sensor.port[0] = 12.343;
	e.event_data = sensor;
	e.event_source = ServiceIdentifier::PH_SENSOR;
	e.event_type = ServiceEventType::SERVICE_LOG_UPDATED;
	man.notify_peer_event(e);

	sensor.port[0] = -12.343;
	e.event_data = sensor;
	e.event_source = ServiceIdentifier::SEA_TEMP_SENSOR;
	e.event_type = ServiceEventType::SERVICE_LOG_UPDATED;
	man.notify_peer_event(e);

	e.event_type = ServiceEventType::SERVICE_INACTIVE;
	man.notify_peer_event(e);

	ServiceSensorData *converted;
	converted = man.retrieve_sensor_single(1, ServiceIdentifier::ALS_SENSOR);
	CHECK_FALSE(nullptr == converted);
	CHECK_EQUAL(65535, (unsigned int)converted->port[0]);
	converted = man.retrieve_sensor_single(1, ServiceIdentifier::PH_SENSOR);
	CHECK_FALSE(nullptr == converted);
	CHECK_EQUAL(12343, (unsigned int)converted->port[0]);
	converted = man.retrieve_sensor_single(1, ServiceIdentifier::PRESSURE_SENSOR);
	CHECK_FALSE(nullptr == converted);
	CHECK_EQUAL(12400, (unsigned int)converted->port[0]);
	CHECK_EQUAL(200, (unsigned int)converted->port[1]);
	converted = man.retrieve_sensor_single(1, ServiceIdentifier::SEA_TEMP_SENSOR);
	CHECK_FALSE(nullptr == converted);
	CHECK_EQUAL(113657, (unsigned int)converted->port[0]);
}

TEST(ArgosTxService, BuildSensorPacketAll) {
	unsigned int size_bits;
	GPSLogEntry e = make_gps_location(1, 12.3, 44.4, 1652105502);
	ServiceSensorData als, ph, pressure, sea_temp;
	std::string x;

	als.port[0] = 10000; // 10000 lux
	ph.port[0] = 7000; // 7.0 ph
	pressure.port[0] = 1000; // 1.0 bar
	pressure.port[1] = 4000; // 0C
	sea_temp.port[0] = 126000; // 0C

	// CRC8 is now handled by the satellite module — no leading zero byte
	x = ArgosPacketBuilder::build_sensor_packet(&e, nullptr, nullptr, nullptr, nullptr, nullptr, false, false, size_bits);
	CHECK_EQUAL("4B8B3633003C0F0012C0"s, Binascii::hexlify(x));
	CHECK_EQUAL(75, size_bits);
	x = ArgosPacketBuilder::build_sensor_packet(&e, &als, &ph, &pressure, &sea_temp, nullptr, false, false, size_bits);
	CHECK_EQUAL("4B8B3633003C0F0012C27106D601F41F401EC300"s, Binascii::hexlify(x));
	CHECK_EQUAL(156, size_bits);
	x = ArgosPacketBuilder::build_sensor_packet(&e, nullptr, &ph, &pressure, &sea_temp, nullptr, false, false, size_bits);
	CHECK_EQUAL("4B8B3633003C0F0012CDAC03E83E803D8600"s, Binascii::hexlify(x));
	CHECK_EQUAL(139, size_bits);
	x = ArgosPacketBuilder::build_sensor_packet(&e, &als, nullptr, &pressure, &sea_temp, nullptr, false, false, size_bits);
	CHECK_EQUAL("4B8B3633003C0F0012C271007D07D007B0C0"s, Binascii::hexlify(x));
	CHECK_EQUAL(142, size_bits);
	x = ArgosPacketBuilder::build_sensor_packet(&e, &als, &ph, nullptr, &sea_temp, nullptr, false, false, size_bits);
	CHECK_EQUAL("4B8B3633003C0F0012C27106D603D860"s, Binascii::hexlify(x));
	CHECK_EQUAL(127, size_bits);
	x = ArgosPacketBuilder::build_sensor_packet(&e, &als, &ph, &pressure, nullptr, nullptr, false, false, size_bits);
	CHECK_EQUAL("4B8B3633003C0F0012C27106D601F41F40"s, Binascii::hexlify(x));
	CHECK_EQUAL(135, size_bits);
}

TEST(ArgosTxService, BuildSensorPacketSeaTemp) {
	unsigned int size_bits;
	GPSLogEntry e = make_gps_location(1, 12.3, 44.4, 1652105502);
	ServiceSensorData sea_temp;
	std::string x;

	sea_temp.port[0] = 147100; // 21.1C

	x = ArgosPacketBuilder::build_sensor_packet(&e, nullptr, nullptr, nullptr, &sea_temp, nullptr, false, false, size_bits);
	CHECK_EQUAL("4B8B3633003C0F0012C23E9C"s, Binascii::hexlify(x));
	CHECK_EQUAL(96, size_bits);
}


TEST(ArgosTxService, DepthPileManagerTestThermistorConversion)
{
	bool enable = true;
	BaseSensorEnableTxMode mode = BaseSensorEnableTxMode::ONESHOT;
	fake_config_store->write_param(ParamID::THERMISTOR_SENSOR_ENABLE, enable);
	fake_config_store->write_param(ParamID::THERMISTOR_SENSOR_ENABLE_TX_MODE, mode);

	std::time_t t = 1652105502000;
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);

	DepthPileManager man;

	ServiceEvent e;
	GPSLogEntry log;
	log.info.valid = true;
	ServiceSensorData sensor;

	e.event_source = ServiceIdentifier::GNSS_SENSOR;
	e.event_type = ServiceEventType::SERVICE_ACTIVE;
	man.notify_peer_event(e);

	e.event_data = log;
	e.event_source = ServiceIdentifier::GNSS_SENSOR;
	e.event_type = ServiceEventType::SERVICE_LOG_UPDATED;
	man.notify_peer_event(e);

	// Thermistor: 21.1°C -> (21.1 + 40.0) * 100 = 6110
	sensor.port[0] = 21.1;
	e.event_data = sensor;
	e.event_source = ServiceIdentifier::THERMISTOR_SENSOR;
	e.event_type = ServiceEventType::SERVICE_LOG_UPDATED;
	man.notify_peer_event(e);

	e.event_type = ServiceEventType::SERVICE_INACTIVE;
	man.notify_peer_event(e);

	CHECK_TRUE(man.eligible());

	// Thermistor retrieves from sea_temp depth pile
	ServiceSensorData *converted;
	converted = man.retrieve_sensor_single(1, ServiceIdentifier::THERMISTOR_SENSOR);
	CHECK_FALSE(nullptr == converted);
	CHECK_EQUAL(6110, (unsigned int)converted->port[0]);
}

TEST(ArgosTxService, BuildSensorPacketWithAXL) {
	unsigned int size_bits;
	GPSLogEntry e = make_gps_location(1, 12.3, 44.4, 1652105502);
	ServiceSensorData axl;
	std::string x;

	// AXL: temp=6500, x=16010, y=15980, z=17000, activity=5
	axl.port[0] = 6500;
	axl.port[1] = 16010;
	axl.port[2] = 15980;
	axl.port[3] = 17000;
	axl.port[4] = 5;

	x = ArgosPacketBuilder::build_sensor_packet(&e, nullptr, nullptr, nullptr, nullptr, &axl, false, false, size_bits);
	// Should include GPS + AXL data (temp 14 bits + X 15 bits + Y 15 bits + Z 15 bits + Activity 8 bits = 67 bits)
	CHECK_TRUE(size_bits > 83); // Must be larger than GPS-only (83 bits)
}

TEST(ArgosTxService, BuildSensorPacketOutOfZone) {
	unsigned int size_bits;
	GPSLogEntry e = make_gps_location(1, 12.3, 44.4, 1652105502);
	std::string x;

	// Test with out-of-zone flag set
	x = ArgosPacketBuilder::build_sensor_packet(&e, nullptr, nullptr, nullptr, nullptr, nullptr, true, false, size_bits);
	CHECK_EQUAL(75, size_bits); // GPS-only packet size should be same

	// Test with low battery flag set
	x = ArgosPacketBuilder::build_sensor_packet(&e, nullptr, nullptr, nullptr, nullptr, nullptr, false, true, size_bits);
	CHECK_EQUAL(75, size_bits);
}

TEST(ArgosTxService, BuildDopplerPacket) {
	unsigned int size_bits;
	std::string x;

	// Battery at 4200mV, not low
	x = ArgosPacketBuilder::build_doppler_packet(4200, false, size_bits);
	CHECK_TRUE(size_bits > 0);
	CHECK_FALSE(x.empty());

	// Battery at 2800mV, low battery
	x = ArgosPacketBuilder::build_doppler_packet(2800, true, size_bits);
	CHECK_TRUE(size_bits > 0);
	CHECK_FALSE(x.empty());
}

IGNORE_TEST(ArgosTxService, PassPredictWithSensorDataPayload)
{
	double frequency = 900.22;
	BaseArgosMode mode = BaseArgosMode::PASS_PREDICTION;
	BaseArgosPower power = BaseArgosPower::POWER_1000_MW;
	BaseDepthPile depth_pile = BaseDepthPile::DEPTH_PILE_4;
	unsigned int argos_hexid = 0x01234567U;
	unsigned int tr_nom = 10;
	bool time_sync_en = false;
	bool enable = true;
	BaseSensorEnableTxMode sensor_mode = BaseSensorEnableTxMode::ONESHOT;

	fake_config_store->write_param(ParamID::ARGOS_DEPTH_PILE, depth_pile);
	fake_config_store->write_param(ParamID::ARGOS_FREQ, frequency);
	fake_config_store->write_param(ParamID::ARGOS_MODE, mode);
	fake_config_store->write_param(ParamID::ARGOS_HEXID, argos_hexid);
	fake_config_store->write_param(ParamID::ARGOS_POWER, power);
	fake_config_store->write_param(ParamID::TR_NOM, tr_nom);
	fake_config_store->write_param(ParamID::ARGOS_TIME_SYNC_BURST_EN, time_sync_en);
	fake_config_store->write_param(ParamID::ALS_SENSOR_ENABLE, enable);
	fake_config_store->write_param(ParamID::ALS_SENSOR_ENABLE_TX_MODE, sensor_mode);

	BasePassPredict pass_predict = {
		/* version_code */ 0,
		7,
		{
		    { 0xA, 5, SAT_DNLK_ON_WITH_A3, SAT_UPLK_ON_WITH_A3, { 2020, 1, 26, 22, 59, 44 }, 7195.550f, 98.5444f, 327.835f, -25.341f, 101.3587f, 0.00f },
			{ 0x9, 3, SAT_DNLK_OFF, SAT_UPLK_ON_WITH_A3, { 2020, 1, 26, 22, 33, 39 }, 7195.632f, 98.7141f, 344.177f, -25.340f, 101.3600f, 0.00f },
			{ 0xB, 7, SAT_DNLK_ON_WITH_A3, SAT_UPLK_ON_WITH_A3, { 2020, 1, 26, 23, 29, 29 }, 7194.917f, 98.7183f, 330.404f, -25.336f, 101.3449f, 0.00f },
			{ 0x5, 0, SAT_DNLK_OFF, SAT_UPLK_ON_WITH_A2, { 2020, 1, 26, 23, 50, 6 }, 7180.549f, 98.7298f, 289.399f, -25.260f, 101.0419f, -1.78f },
			{ 0x8, 0, SAT_DNLK_OFF, SAT_UPLK_ON_WITH_A2, { 2020, 1, 26, 22, 12, 6 }, 7226.170f, 99.0661f, 343.180f, -25.499f, 102.0039f, -1.80f },
			{ 0xC, 6, SAT_DNLK_OFF, SAT_UPLK_ON_WITH_A3, { 2020, 1, 26, 22, 3, 52 }, 7226.509f, 99.1913f, 291.936f, -25.500f, 102.0108f, -1.98f },
			{ 0xD, 4, SAT_DNLK_ON_WITH_A3, SAT_UPLK_ON_WITH_A3, { 2020, 1, 26, 22, 3, 53 }, 7160.246f, 98.5358f, 118.029f, -25.154f, 100.6148f, 0.00f }
		}
	};

	fake_config_store->write_pass_predict(pass_predict);

	ArgosTxService serv(*mock_kineis);

	std::time_t t = 1580083200000;
	fake_rtc->settime(t/1000);
	fake_timer->set_counter(t);

	mock().expectOneCall("set_tcxo_warmup_time").onObject(mock_kineis).withUnsignedIntParameter("time", 5);
	serv.start();

	ServiceSensorData sensor_data;

	inject_gps_active();
	inject_gps_location(1, 11.8768, -33.8232, t);
	sensor_data.port[0] = 1234;
	inject_sensor_data(sensor_data, ServiceIdentifier::ALS_SENSOR);
	inject_gps_inactive();
	inject_gps_active();
	inject_gps_location(1, 11.8768, -33.8232, t);
	sensor_data.port[0] = 5678;
	inject_sensor_data(sensor_data, ServiceIdentifier::ALS_SENSOR);
	inject_gps_inactive();
	inject_gps_active();
	inject_gps_location(1, 11.8768, -33.8232, t);
	sensor_data.port[0] = 13594;
	inject_sensor_data(sensor_data, ServiceIdentifier::ALS_SENSOR);
	inject_gps_inactive();
	inject_gps_active();
	inject_gps_location(1, 11.8768, -33.8232, t);
	sensor_data.port[0] = 22782;
	inject_sensor_data(sensor_data, ServiceIdentifier::ALS_SENSOR);
	inject_gps_inactive();

	system_scheduler->run();

	// Run for 10 transmissions
	for (unsigned int i = 0; i < 10; i++) {
		mock().expectOneCall("send").onObject(mock_kineis).ignoreOtherParameters();
		t += serv.get_last_schedule();
		fake_rtc->settime(t/1000);
		fake_timer->set_counter(t);
		system_scheduler->run();
		mock_kineis->notify(KineisEventTxComplete({}));
	}
}
