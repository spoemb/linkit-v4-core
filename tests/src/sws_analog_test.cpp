#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"

#include <thread>
#include <chrono>

#include "sws_analog_service.hpp"
#include "bsp.hpp"
#include "nrfx_saadc.h"
#include "fake_config_store.hpp"
#include "linux_timer.hpp"

extern Timer *system_timer;
extern ConfigurationStore *configuration_store;
extern Scheduler *system_scheduler;

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

        // Set default configuration (write_param takes T& so we need lvalues)
        bool uw_en = true;
        auto uw_src = BaseUnderwaterDetectSource::SWS;
        unsigned int val_1 = 1U, val_3 = 3U, val_2 = 2U, val_100 = 100U;
        unsigned int val_10 = 10U, val_3000 = 3000U, val_3600 = 3600U, val_0 = 0U;

        configuration_store->write_param(ParamID::UNDERWATER_EN, uw_en);
        configuration_store->write_param(ParamID::UNDERWATER_DETECT_SOURCE, uw_src);
        configuration_store->write_param(ParamID::SAMPLING_UNDER_FREQ, val_1);
        configuration_store->write_param(ParamID::SAMPLING_SURF_FREQ, val_1);
        configuration_store->write_param(ParamID::UW_MAX_SAMPLES, val_3);
        configuration_store->write_param(ParamID::UW_MIN_DRY_SAMPLES, val_2);
        configuration_store->write_param(ParamID::UW_SAMPLE_GAP, val_100);
        configuration_store->write_param(ParamID::UW_PIN_SAMPLE_DELAY, val_10);
        configuration_store->write_param(ParamID::SWS_ANALOG_THRESHOLD_MIN, val_100);
        configuration_store->write_param(ParamID::SWS_ANALOG_THRESHOLD_MAX, val_3000);
        configuration_store->write_param(ParamID::SWS_ANALOG_HYSTERESIS, val_10);
        configuration_store->write_param(ParamID::SWS_ANALOG_CALIB_INTERVAL, val_3600);
        configuration_store->write_param(ParamID::UW_MAX_DIVE_TIME, val_0);  // Disabled for most tests
        configuration_store->write_param(ParamID::UW_MIN_SURFACE_TIME, val_0);

        SAADC::set_adc_value(0);

        // Ignore GPIO/PMU/SAADC mock calls - focus on detection logic
        mock().ignoreOtherCalls();
    }

    void teardown() {
        delete system_scheduler;
        delete linux_timer;
        delete fake_config_store;
        mock().clear();
    }

    // Helper: run one scheduler cycle (one service_initiate call)
    void run_one_sample() {
        while (!system_scheduler->run());
    }
};

/**
 * Test 1: Initial air calibration
 * Verifies that the service starts and calibrates with air ADC value
 */
TEST(SWSAnalog, InitialAirCalibration)
{
    SWSAnalogService s;
    system_timer->start();

    // Set ADC to return air value (low conductivity)
    SAADC::set_adc_value(200);

    bool got_callback = false;
    s.start([&got_callback](ServiceEvent &event) {
        if (event.event_type == ServiceEventType::SERVICE_LOG_UPDATED) {
            got_callback = true;
        }
    });

    // Run a few samples - should get initial state callback
    for (int i = 0; i < 5; i++) {
        run_one_sample();
    }

    CHECK_TRUE(got_callback);
    s.stop();
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
    SAADC::set_adc_value(150);

    s.start([&switch_state, &num_callbacks](ServiceEvent &event) {
        if (event.event_type == ServiceEventType::SERVICE_LOG_UPDATED) {
            switch_state = std::get<bool>(event.event_data);
            num_callbacks++;
        }
    });

    // Run enough samples to confirm surface state
    for (unsigned int i = 0; i < 5; i++) {
        run_one_sample();
    }

    CHECK_FALSE(switch_state);  // Should be at surface
    CHECK(num_callbacks >= 1);

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

    // Calibrate with air value first
    SAADC::set_adc_value(200);

    s.start([&switch_state, &num_callbacks](ServiceEvent &event) {
        if (event.event_type == ServiceEventType::SERVICE_LOG_UPDATED) {
            switch_state = std::get<bool>(event.event_data);
            num_callbacks++;
        }
    });

    // Switch to underwater value after calibration
    SAADC::set_adc_value(2500);

    for (unsigned int i = 0; i < 10; i++) {
        run_one_sample();
    }

    CHECK_TRUE(switch_state);  // Should be underwater
    CHECK(num_callbacks >= 1);

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
    SAADC::set_adc_value(150);

    s.start([&switch_state, &num_callbacks](ServiceEvent &event) {
        if (event.event_type == ServiceEventType::SERVICE_LOG_UPDATED) {
            switch_state = std::get<bool>(event.event_data);
            num_callbacks++;
        }
    });

    // Confirm surface state
    for (unsigned int i = 0; i < 5; i++) {
        run_one_sample();
    }

    CHECK_FALSE(switch_state);
    (void)num_callbacks;

    // Set value near threshold (in hysteresis zone)
    // Air baseline ~150, water estimate ~450, threshold ~255, hysteresis ~25
    // So hysteresis zone is roughly 230-280
    SAADC::set_adc_value(260);

    for (unsigned int i = 0; i < 5; i++) {
        run_one_sample();
    }

    // State should remain at surface (no new state change callback)
    CHECK_FALSE(switch_state);

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
    SAADC::set_adc_value(150);

    s.start([&switch_state](ServiceEvent &event) {
        if (event.event_type == ServiceEventType::SERVICE_LOG_UPDATED) {
            switch_state = std::get<bool>(event.event_data);
        }
    });

    // Confirm surface
    for (unsigned int i = 0; i < 5; i++) {
        run_one_sample();
    }
    CHECK_FALSE(switch_state);

    // First submersion with moderate salinity
    SAADC::set_adc_value(2500);
    for (unsigned int i = 0; i < 10; i++) {
        run_one_sample();
    }
    CHECK_TRUE(switch_state);

    // Return to surface
    SAADC::set_adc_value(150);
    for (unsigned int i = 0; i < 10; i++) {
        run_one_sample();
    }
    CHECK_FALSE(switch_state);

    // Second submersion with higher salinity (2800)
    SAADC::set_adc_value(2800);
    for (unsigned int i = 0; i < 10; i++) {
        run_one_sample();
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
    unsigned int dive_time = 5U;
    configuration_store->write_param(ParamID::UW_MAX_DIVE_TIME, dive_time);

    SAADC::set_adc_value(200);

    s.start([&switch_state](ServiceEvent &event) {
        if (event.event_type == ServiceEventType::SERVICE_LOG_UPDATED) {
            switch_state = std::get<bool>(event.event_data);
        }
    });

    // Go underwater
    SAADC::set_adc_value(2500);
    for (unsigned int i = 0; i < 6; i++) {
        run_one_sample();
    }
    CHECK_TRUE(switch_state);  // Confirmed underwater

    // Wait for max dive time to expire (>5 seconds of real time)
    // The PMU::get_timestamp_ms() uses std::chrono, so real time passing matters
    // Sleep to simulate time passing
    std::this_thread::sleep_for(std::chrono::seconds(6));

    // Take another reading (still high ADC)
    for (unsigned int i = 0; i < 6; i++) {
        run_one_sample();
    }

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
    SAADC::set_adc_value(150);

    s.start([&switch_state, &num_callbacks](ServiceEvent &event) {
        if (event.event_type == ServiceEventType::SERVICE_LOG_UPDATED) {
            switch_state = std::get<bool>(event.event_data);
            num_callbacks++;
        }
    });

    // Confirm surface
    for (unsigned int i = 0; i < 5; i++) {
        run_one_sample();
    }
    CHECK_FALSE(switch_state);
    (void)num_callbacks;

    // Send invalid saturated value (above threshold_max=3000, should be rejected)
    SAADC::set_adc_value(3500);

    for (unsigned int i = 0; i < 3; i++) {
        run_one_sample();
    }

    // State should remain unchanged
    CHECK_FALSE(switch_state);

    s.stop();
}

// ============================================================================
// RAPID SURFACE DETECTION TESTS
// These tests validate <2s surface detection in various biofouling conditions
// ============================================================================

/**
 * Test 8: Rapid surface detection with clean sensor
 * ADC drops from 3000 to 150 (clean electrode, no biofouling)
 * Expected: Surface detected within 1 batch (3 parent samples)
 */
TEST(SWSAnalog, RapidSurfaceDetection_CleanSensor)
{
    SWSAnalogService s;
    bool switch_state = false;
    unsigned int callbacks = 0;

    system_timer->start();
    SAADC::set_adc_value(200);

    s.start([&switch_state, &callbacks](ServiceEvent &event) {
        if (event.event_type == ServiceEventType::SERVICE_LOG_UPDATED) {
            switch_state = std::get<bool>(event.event_data);
            callbacks++;
        }
    });

    // Go underwater with clear seawater ADC
    SAADC::set_adc_value(3000);
    for (int i = 0; i < 10; i++) {
        run_one_sample();
    }
    CHECK_TRUE(switch_state);
    unsigned int callbacks_before = callbacks;

    // SURFACE: ADC drops dramatically (clean sensor dries instantly)
    SAADC::set_adc_value(150);

    unsigned int samples_to_surface = 0;
    for (int i = 0; i < 10; i++) {
        run_one_sample();
        samples_to_surface++;
        if (callbacks > callbacks_before && !switch_state) {
            break;  // Surface detected!
        }
    }

    CHECK_FALSE(switch_state);
    // With parent batch of 3 (UW_MAX_SAMPLES=3), should detect within one batch
    UNSIGNED_LONGS_EQUAL_TEXT(3, samples_to_surface,
        "Clean sensor: surface should be detected within 1 batch (3 samples)");

    s.stop();
}

/**
 * Test 9: Rapid surface detection with moderate biofouling
 * ADC drops from 3000 to 1200 (salt deposits on electrodes)
 * The ADC doesn't return to baseline but the DROP is significant
 * Expected: Surface detected within 1 batch (3 parent samples)
 */
TEST(SWSAnalog, RapidSurfaceDetection_ModerateBiofouling)
{
    SWSAnalogService s;
    bool switch_state = false;
    unsigned int callbacks = 0;

    system_timer->start();
    SAADC::set_adc_value(200);

    s.start([&switch_state, &callbacks](ServiceEvent &event) {
        if (event.event_type == ServiceEventType::SERVICE_LOG_UPDATED) {
            switch_state = std::get<bool>(event.event_data);
            callbacks++;
        }
    });

    // Go underwater
    SAADC::set_adc_value(3000);
    for (int i = 0; i < 10; i++) {
        run_one_sample();
    }
    CHECK_TRUE(switch_state);
    unsigned int callbacks_before = callbacks;

    // SURFACE with biofouling: ADC drops but stays elevated
    // 3000 → 1200 = 60% raw drop, well above TIER 1 threshold (25%)
    SAADC::set_adc_value(1200);

    unsigned int samples_to_surface = 0;
    for (int i = 0; i < 10; i++) {
        run_one_sample();
        samples_to_surface++;
        if (callbacks > callbacks_before && !switch_state) {
            break;
        }
    }

    CHECK_FALSE_TEXT(switch_state,
        "Moderate biofouling: should detect surface despite elevated ADC");
    CHECK_TEXT(samples_to_surface <= 3,
        "Moderate biofouling: should detect within 1 batch (3 samples)");

    s.stop();
}

/**
 * Test 10: Rapid surface detection with heavy biofouling
 * ADC drops from 2800 to 1800 (thick biofilm + salt crystals)
 * Even though ADC stays high, the relative drop (35%) triggers rapid detection
 * Expected: Surface detected within 1 batch (3 parent samples)
 */
TEST(SWSAnalog, RapidSurfaceDetection_HeavyBiofouling)
{
    SWSAnalogService s;
    bool switch_state = false;
    unsigned int callbacks = 0;

    system_timer->start();
    SAADC::set_adc_value(200);

    s.start([&switch_state, &callbacks](ServiceEvent &event) {
        if (event.event_type == ServiceEventType::SERVICE_LOG_UPDATED) {
            switch_state = std::get<bool>(event.event_data);
            callbacks++;
        }
    });

    // Go underwater with slightly lower ADC (biofilm reduces conductivity a bit)
    SAADC::set_adc_value(2800);
    for (int i = 0; i < 10; i++) {
        run_one_sample();
    }
    CHECK_TRUE(switch_state);
    unsigned int callbacks_before = callbacks;

    // SURFACE with heavy biofouling: ADC drops from 2800 to 1800
    // Raw drop = (2800-1800)/2800 = 35.7%, TIER 1 fires
    SAADC::set_adc_value(1800);

    unsigned int samples_to_surface = 0;
    for (int i = 0; i < 10; i++) {
        run_one_sample();
        samples_to_surface++;
        if (callbacks > callbacks_before && !switch_state) {
            break;
        }
    }

    CHECK_FALSE_TEXT(switch_state,
        "Heavy biofouling: should detect surface despite high residual ADC");
    CHECK_TEXT(samples_to_surface <= 3,
        "Heavy biofouling: should detect within 1 batch (3 samples)");

    s.stop();
}

/**
 * Test 11: No false positive from underwater ADC fluctuation
 * ADC fluctuates ±10% while underwater (normal wave/turbulence variation)
 * Expected: State remains underwater (no false surface detection)
 */
TEST(SWSAnalog, NoFalsePositive_UnderwaterFluctuation)
{
    SWSAnalogService s;
    bool switch_state = false;
    unsigned int callbacks = 0;

    system_timer->start();
    SAADC::set_adc_value(200);

    s.start([&switch_state, &callbacks](ServiceEvent &event) {
        if (event.event_type == ServiceEventType::SERVICE_LOG_UPDATED) {
            switch_state = std::get<bool>(event.event_data);
            callbacks++;
        }
    });

    // Go underwater
    SAADC::set_adc_value(3000);
    for (int i = 0; i < 10; i++) {
        run_one_sample();
    }
    CHECK_TRUE(switch_state);
    (void)callbacks;

    // Simulate underwater fluctuation: ±10% variation
    // This should NOT trigger surface detection
    uint16_t fluctuation_values[] = {2700, 2850, 2600, 2900, 2750, 2800, 2650, 2950, 2700, 2850};
    for (int i = 0; i < 10; i++) {
        SAADC::set_adc_value(fluctuation_values[i]);
        run_one_sample();
    }

    // Should still be underwater - no false surface detection
    CHECK_TRUE_TEXT(switch_state,
        "Underwater fluctuation: should NOT trigger false surface detection");

    s.stop();
}

/**
 * Test 12: No false positive from gradual salinity change
 * ADC gradually decreases from 3000 to 2500 over many samples
 * (simulates swimming from high salinity to lower salinity water)
 * Expected: State remains underwater
 */
TEST(SWSAnalog, NoFalsePositive_GradualSalinityChange)
{
    SWSAnalogService s;
    bool switch_state = false;
    unsigned int callbacks = 0;

    system_timer->start();
    SAADC::set_adc_value(200);

    s.start([&switch_state, &callbacks](ServiceEvent &event) {
        if (event.event_type == ServiceEventType::SERVICE_LOG_UPDATED) {
            switch_state = std::get<bool>(event.event_data);
            callbacks++;
        }
    });

    // Go underwater at high salinity
    SAADC::set_adc_value(3000);
    for (int i = 0; i < 10; i++) {
        run_one_sample();
    }
    CHECK_TRUE(switch_state);

    // Gradually decrease ADC (salinity change, ~50 per sample over 10 samples)
    for (int i = 0; i < 10; i++) {
        uint16_t value = (uint16_t)(3000 - i * 50);
        SAADC::set_adc_value(value);
        run_one_sample();
    }

    // Should still be underwater - gradual changes are not surfacing events
    CHECK_TRUE_TEXT(switch_state,
        "Gradual salinity change: should NOT trigger false surface detection");

    s.stop();
}
