#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"

#include <iostream>

#include "sws_analog_service.hpp"
#include "bsp.hpp"
#include "fake_config_store.hpp"
#include "linux_timer.hpp"

extern Timer *system_timer;
extern ConfigurationStore *configuration_store;
extern Scheduler *system_scheduler;

// Mock ADC value to be returned by the fake SAADC
static int16_t mock_adc_value = 0;

// Fake SAADC implementation for testing
nrfx_err_t nrfx_saadc_sample_convert(uint8_t channel, nrf_saadc_value_t *p_value)
{
    mock().actualCall("nrfx_saadc_sample_convert")
          .withParameter("channel", channel);

    *p_value = mock_adc_value;
    return NRFX_SUCCESS;
}

nrfx_err_t nrfx_saadc_init(nrfx_saadc_config_t const *p_config, nrfx_saadc_event_handler_t event_handler)
{
    mock().actualCall("nrfx_saadc_init");
    return NRFX_SUCCESS;
}

void nrfx_saadc_uninit(void)
{
    mock().actualCall("nrfx_saadc_uninit");
}

nrfx_err_t nrfx_saadc_channel_init(uint8_t channel, nrf_saadc_channel_config_t const *p_config)
{
    mock().actualCall("nrfx_saadc_channel_init")
          .withParameter("channel", channel);
    return NRFX_SUCCESS;
}

TEST_GROUP(SWSAnalog)
{
    FakeConfigurationStore *fake_config_store;
    LinuxTimer *linux_timer;

    void setup() {
        fake_config_store = new FakeConfigurationStore;
        configuration_store = fake_config_store;
        linux_timer = new LinuxTimer;
        system_timer = linux_timer;
        system_scheduler = new Scheduler(system_timer);
        configuration_store->init();

        // Set default configuration
        configuration_store->write_param(ParamID::UNDERWATER_EN, true);
        configuration_store->write_param(ParamID::UNDERWATER_DETECT_SOURCE, BaseUnderwaterDetectSource::SWS);
        configuration_store->write_param(ParamID::SAMPLING_UNDER_FREQ, 1U);
        configuration_store->write_param(ParamID::SAMPLING_SURF_FREQ, 1U);
        configuration_store->write_param(ParamID::UW_MAX_SAMPLES, 3U);
        configuration_store->write_param(ParamID::UW_MIN_DRY_SAMPLES, 2U);
        configuration_store->write_param(ParamID::UW_SAMPLE_GAP, 100U);
        configuration_store->write_param(ParamID::UW_PIN_SAMPLE_DELAY, 10U);
        configuration_store->write_param(ParamID::SWS_ANALOG_THRESHOLD_MIN, 100U);
        configuration_store->write_param(ParamID::SWS_ANALOG_THRESHOLD_MAX, 3000U);
        configuration_store->write_param(ParamID::SWS_ANALOG_HYSTERESIS, 10U);
        configuration_store->write_param(ParamID::SWS_ANALOG_CALIB_INTERVAL, 3600U);
        configuration_store->write_param(ParamID::UW_MAX_DIVE_TIME, 0U);  // Disabled for most tests
        configuration_store->write_param(ParamID::UW_MIN_SURFACE_TIME, 0U);

        mock_adc_value = 0;
    }

    void teardown() {
        delete system_scheduler;
        delete linux_timer;
        delete fake_config_store;
        mock().clear();
    }
};

/**
 * Test 1: Initial air calibration
 * Verifies that the service performs initial calibration when started
 */
TEST(SWSAnalog, InitialAirCalibration)
{
    SWSAnalogService s;
    system_timer->start();

    // Set mock ADC to return air value (low conductivity)
    mock_adc_value = 200;

    // Expect multiple ADC reads during calibration (10 samples)
    for (int i = 0; i < 10; i++) {
        mock().expectOneCall("set").withUnsignedIntParameter("pin", BSP::GPIO::GPIO_SLOW_SWS_SEND);
        mock().expectOneCall("delay_ms").withUnsignedIntParameter("ms", 10);
        mock().expectOneCall("nrfx_saadc_init");
        mock().expectOneCall("nrfx_saadc_channel_init").withParameter("channel", SWS_ADC);
        mock().expectOneCall("nrfx_saadc_sample_convert").withParameter("channel", SWS_ADC);
        mock().expectOneCall("nrfx_saadc_uninit");
        mock().expectOneCall("clear").withUnsignedIntParameter("pin", BSP::GPIO::GPIO_SLOW_SWS_SEND);
        mock().expectOneCall("delay_ms").withUnsignedIntParameter("ms", 100);
    }

    s.start([](ServiceEvent &event) {});

    mock().checkExpectations();
}

/**
 * Test 2: Surface detection (low ADC values)
 * Verifies that low ADC readings are correctly identified as surface
 */
TEST(SWSAnalog, SurfaceDetection)
{
    SWSAnalogService s;
    bool switch_state = true;  // Start assuming underwater
    unsigned int num_callbacks = 0;

    system_timer->start();

    // Set mock ADC to return low air value
    mock_adc_value = 150;

    s.start([&switch_state, &num_callbacks](ServiceEvent &event) {
        if (event.event_type == ServiceEventType::SERVICE_LOG_UPDATED) {
            switch_state = std::get<bool>(event.event_data);
            num_callbacks++;
        }
    });

    // Run enough samples to confirm surface state
    for (unsigned int i = 0; i < 5; i++) {
        mock().expectOneCall("set").withUnsignedIntParameter("pin", BSP::GPIO::GPIO_SLOW_SWS_SEND);
        mock().expectOneCall("delay_ms").withUnsignedIntParameter("ms", 10);
        mock().expectOneCall("nrfx_saadc_init");
        mock().expectOneCall("nrfx_saadc_channel_init").withParameter("channel", SWS_ADC);
        mock().expectOneCall("nrfx_saadc_sample_convert").withParameter("channel", SWS_ADC);
        mock().expectOneCall("nrfx_saadc_uninit");
        mock().expectOneCall("clear").withUnsignedIntParameter("pin", BSP::GPIO::GPIO_SLOW_SWS_SEND);

        while (!system_scheduler->run());
    }

    CHECK_FALSE(switch_state);  // Should be at surface
    CHECK_EQUAL(1, num_callbacks);

    s.stop();
}

/**
 * Test 3: Underwater detection (high ADC values)
 * Verifies that high ADC readings are correctly identified as underwater
 */
TEST(SWSAnalog, UnderwaterDetection)
{
    SWSAnalogService s;
    bool switch_state = false;  // Start at surface
    unsigned int num_callbacks = 0;

    system_timer->start();

    // Set mock ADC to return high water value (high conductivity)
    mock_adc_value = 2500;

    s.start([&switch_state, &num_callbacks](ServiceEvent &event) {
        if (event.event_type == ServiceEventType::SERVICE_LOG_UPDATED) {
            switch_state = std::get<bool>(event.event_data);
            num_callbacks++;
        }
    });

    // Run enough samples to confirm underwater state
    for (unsigned int i = 0; i < 5; i++) {
        mock().expectOneCall("set").withUnsignedIntParameter("pin", BSP::GPIO::GPIO_SLOW_SWS_SEND);
        mock().expectOneCall("delay_ms").withUnsignedIntParameter("ms", 10);
        mock().expectOneCall("nrfx_saadc_init");
        mock().expectOneCall("nrfx_saadc_channel_init").withParameter("channel", SWS_ADC);
        mock().expectOneCall("nrfx_saadc_sample_convert").withParameter("channel", SWS_ADC);
        mock().expectOneCall("nrfx_saadc_uninit");
        mock().expectOneCall("clear").withUnsignedIntParameter("pin", BSP::GPIO::GPIO_SLOW_SWS_SEND);

        while (!system_scheduler->run());
    }

    CHECK_TRUE(switch_state);  // Should be underwater
    CHECK_EQUAL(1, num_callbacks);

    s.stop();
}

/**
 * Test 4: Hysteresis prevents oscillation
 * Verifies that values in the hysteresis zone maintain previous state
 */
TEST(SWSAnalog, HysteresisPreventOscillation)
{
    SWSAnalogService s;
    bool switch_state = false;
    unsigned int num_callbacks = 0;

    system_timer->start();

    // Start at surface with low value
    mock_adc_value = 150;

    s.start([&switch_state, &num_callbacks](ServiceEvent &event) {
        if (event.event_type == ServiceEventType::SERVICE_LOG_UPDATED) {
            switch_state = std::get<bool>(event.event_data);
            num_callbacks++;
        }
    });

    // Confirm surface state
    for (unsigned int i = 0; i < 3; i++) {
        mock().expectOneCall("set").withUnsignedIntParameter("pin", BSP::GPIO::GPIO_SLOW_SWS_SEND);
        mock().expectOneCall("delay_ms").withUnsignedIntParameter("ms", 10);
        mock().expectOneCall("nrfx_saadc_init");
        mock().expectOneCall("nrfx_saadc_channel_init").withParameter("channel", SWS_ADC);
        mock().expectOneCall("nrfx_saadc_sample_convert").withParameter("channel", SWS_ADC);
        mock().expectOneCall("nrfx_saadc_uninit");
        mock().expectOneCall("clear").withUnsignedIntParameter("pin", BSP::GPIO::GPIO_SLOW_SWS_SEND);
        while (!system_scheduler->run());
    }

    CHECK_FALSE(switch_state);
    unsigned int callbacks_after_surface = num_callbacks;

    // Now set value to middle of hysteresis zone (should maintain surface state)
    // Assuming threshold ~350 with 10% hysteresis = ~315-385
    mock_adc_value = 350;

    for (unsigned int i = 0; i < 3; i++) {
        mock().expectOneCall("set").withUnsignedIntParameter("pin", BSP::GPIO::GPIO_SLOW_SWS_SEND);
        mock().expectOneCall("delay_ms").withUnsignedIntParameter("ms", 10);
        mock().expectOneCall("nrfx_saadc_init");
        mock().expectOneCall("nrfx_saadc_channel_init").withParameter("channel", SWS_ADC);
        mock().expectOneCall("nrfx_saadc_sample_convert").withParameter("channel", SWS_ADC);
        mock().expectOneCall("nrfx_saadc_uninit");
        mock().expectOneCall("clear").withUnsignedIntParameter("pin", BSP::GPIO::GPIO_SLOW_SWS_SEND);
        while (!system_scheduler->run());
    }

    // State should remain at surface (no new callback)
    CHECK_FALSE(switch_state);
    CHECK_EQUAL(callbacks_after_surface, num_callbacks);

    s.stop();
}

/**
 * Test 5: Salinity adaptation
 * Verifies that the system adapts to changing water salinity
 */
TEST(SWSAnalog, SalinityAdaptation)
{
    SWSAnalogService s;
    bool switch_state = false;

    system_timer->start();

    s.start([&switch_state](ServiceEvent &event) {
        if (event.event_type == ServiceEventType::SERVICE_LOG_UPDATED) {
            switch_state = std::get<bool>(event.event_data);
        }
    });

    // First submersion with moderate salinity (value = 1500)
    mock_adc_value = 1500;
    for (unsigned int i = 0; i < 5; i++) {
        mock().expectOneCall("set").withUnsignedIntParameter("pin", BSP::GPIO::GPIO_SLOW_SWS_SEND);
        mock().expectOneCall("delay_ms").withUnsignedIntParameter("ms", 10);
        mock().expectOneCall("nrfx_saadc_init");
        mock().expectOneCall("nrfx_saadc_channel_init").withParameter("channel", SWS_ADC);
        mock().expectOneCall("nrfx_saadc_sample_convert").withParameter("channel", SWS_ADC);
        mock().expectOneCall("nrfx_saadc_uninit");
        mock().expectOneCall("clear").withUnsignedIntParameter("pin", BSP::GPIO::GPIO_SLOW_SWS_SEND);
        while (!system_scheduler->run());
    }

    CHECK_TRUE(switch_state);  // Should detect underwater

    // Return to surface
    mock_adc_value = 150;
    for (unsigned int i = 0; i < 3; i++) {
        mock().expectOneCall("set").withUnsignedIntParameter("pin", BSP::GPIO::GPIO_SLOW_SWS_SEND);
        mock().expectOneCall("delay_ms").withUnsignedIntParameter("ms", 10);
        mock().expectOneCall("nrfx_saadc_init");
        mock().expectOneCall("nrfx_saadc_channel_init").withParameter("channel", SWS_ADC);
        mock().expectOneCall("nrfx_saadc_sample_convert").withParameter("channel", SWS_ADC);
        mock().expectOneCall("nrfx_saadc_uninit");
        mock().expectOneCall("clear").withUnsignedIntParameter("pin", BSP::GPIO::GPIO_SLOW_SWS_SEND);
        while (!system_scheduler->run());
    }

    CHECK_FALSE(switch_state);  // Back at surface

    // Second submersion with higher salinity (value = 2800)
    // System should still detect underwater despite salinity change
    mock_adc_value = 2800;
    for (unsigned int i = 0; i < 5; i++) {
        mock().expectOneCall("set").withUnsignedIntParameter("pin", BSP::GPIO::GPIO_SLOW_SWS_SEND);
        mock().expectOneCall("delay_ms").withUnsignedIntParameter("ms", 10);
        mock().expectOneCall("nrfx_saadc_init");
        mock().expectOneCall("nrfx_saadc_channel_init").withParameter("channel", SWS_ADC);
        mock().expectOneCall("nrfx_saadc_sample_convert").withParameter("channel", SWS_ADC);
        mock().expectOneCall("nrfx_saadc_uninit");
        mock().expectOneCall("clear").withUnsignedIntParameter("pin", BSP::GPIO::GPIO_SLOW_SWS_SEND);
        while (!system_scheduler->run());
    }

    CHECK_TRUE(switch_state);  // Should still detect underwater

    s.stop();
}

/**
 * Test 6: Max dive time safety timeout
 * Verifies that system forces surface detection after max dive time
 */
TEST(SWSAnalog, MaxDiveTimeSafety)
{
    SWSAnalogService s;
    bool switch_state = false;

    system_timer->start();

    // Enable max dive time safety (5 seconds for test)
    configuration_store->write_param(ParamID::UW_MAX_DIVE_TIME, 5U);

    s.start([&switch_state](ServiceEvent &event) {
        if (event.event_type == ServiceEventType::SERVICE_LOG_UPDATED) {
            switch_state = std::get<bool>(event.event_data);
        }
    });

    // Go underwater with high value
    mock_adc_value = 2500;
    for (unsigned int i = 0; i < 3; i++) {
        mock().expectOneCall("set").withUnsignedIntParameter("pin", BSP::GPIO::GPIO_SLOW_SWS_SEND);
        mock().expectOneCall("delay_ms").withUnsignedIntParameter("ms", 10);
        mock().expectOneCall("nrfx_saadc_init");
        mock().expectOneCall("nrfx_saadc_channel_init").withParameter("channel", SWS_ADC);
        mock().expectOneCall("nrfx_saadc_sample_convert").withParameter("channel", SWS_ADC);
        mock().expectOneCall("nrfx_saadc_uninit");
        mock().expectOneCall("clear").withUnsignedIntParameter("pin", BSP::GPIO::GPIO_SLOW_SWS_SEND);
        while (!system_scheduler->run());
    }

    CHECK_TRUE(switch_state);  // Confirmed underwater

    // Advance time by 6 seconds (past max dive time)
    linux_timer->increment(6000);

    // Take another reading (still high value indicating underwater)
    mock().expectOneCall("set").withUnsignedIntParameter("pin", BSP::GPIO::GPIO_SLOW_SWS_SEND);
    mock().expectOneCall("delay_ms").withUnsignedIntParameter("ms", 10);
    mock().expectOneCall("nrfx_saadc_init");
    mock().expectOneCall("nrfx_saadc_channel_init").withParameter("channel", SWS_ADC);
    mock().expectOneCall("nrfx_saadc_sample_convert").withParameter("channel", SWS_ADC);
    mock().expectOneCall("nrfx_saadc_uninit");
    mock().expectOneCall("clear").withUnsignedIntParameter("pin", BSP::GPIO::GPIO_SLOW_SWS_SEND);
    while (!system_scheduler->run());

    // Should force surface despite high ADC value (safety timeout)
    CHECK_FALSE(switch_state);

    s.stop();
}

/**
 * Test 7: Invalid ADC values handling
 * Verifies that invalid/saturated ADC readings are rejected
 */
TEST(SWSAnalog, InvalidADCValuesHandling)
{
    SWSAnalogService s;
    bool switch_state = false;
    unsigned int num_callbacks = 0;

    system_timer->start();

    // Start with valid low value (surface)
    mock_adc_value = 150;

    s.start([&switch_state, &num_callbacks](ServiceEvent &event) {
        if (event.event_type == ServiceEventType::SERVICE_LOG_UPDATED) {
            switch_state = std::get<bool>(event.event_data);
            num_callbacks++;
        }
    });

    // Confirm surface
    for (unsigned int i = 0; i < 3; i++) {
        mock().expectOneCall("set").withUnsignedIntParameter("pin", BSP::GPIO::GPIO_SLOW_SWS_SEND);
        mock().expectOneCall("delay_ms").withUnsignedIntParameter("ms", 10);
        mock().expectOneCall("nrfx_saadc_init");
        mock().expectOneCall("nrfx_saadc_channel_init").withParameter("channel", SWS_ADC);
        mock().expectOneCall("nrfx_saadc_sample_convert").withParameter("channel", SWS_ADC);
        mock().expectOneCall("nrfx_saadc_uninit");
        mock().expectOneCall("clear").withUnsignedIntParameter("pin", BSP::GPIO::GPIO_SLOW_SWS_SEND);
        while (!system_scheduler->run());
    }

    CHECK_FALSE(switch_state);
    unsigned int callbacks_before = num_callbacks;

    // Send invalid saturated value (should be ignored, state maintained)
    mock_adc_value = 16350;  // Near saturation

    mock().expectOneCall("set").withUnsignedIntParameter("pin", BSP::GPIO::GPIO_SLOW_SWS_SEND);
    mock().expectOneCall("delay_ms").withUnsignedIntParameter("ms", 10);
    mock().expectOneCall("nrfx_saadc_init");
    mock().expectOneCall("nrfx_saadc_channel_init").withParameter("channel", SWS_ADC);
    mock().expectOneCall("nrfx_saadc_sample_convert").withParameter("channel", SWS_ADC);
    mock().expectOneCall("nrfx_saadc_uninit");
    mock().expectOneCall("clear").withUnsignedIntParameter("pin", BSP::GPIO::GPIO_SLOW_SWS_SEND);
    while (!system_scheduler->run());

    // State should remain unchanged (no new callback due to invalid reading)
    CHECK_FALSE(switch_state);
    CHECK_EQUAL(callbacks_before, num_callbacks);

    s.stop();
}
