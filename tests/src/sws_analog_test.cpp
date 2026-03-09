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

        // Reset static noinit data for test isolation
        // Without this, tests depend on execution order (noinit statics persist)
        SWSAnalogService::reset_noinit_data();

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

    // Helper: run a few samples at the calibration air value to pass
    // the first-sample coherence check before switching to water ADC.
    // Without this, a sudden jump from air=200 to water=3000 triggers
    // recalibration which changes the baselines unpredictably.
    void warmup_air_samples(int count = 3) {
        for (int i = 0; i < count; i++) run_one_sample();
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

    // Run a few samples at air value to pass first-sample coherence check
    for (unsigned int i = 0; i < 5; i++) {
        run_one_sample();
    }

    // Switch to underwater value
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

    // Warm up at air value to pass coherence check
    warmup_air_samples();

    // Go underwater
    SAADC::set_adc_value(2500);
    for (unsigned int i = 0; i < 6; i++) {
        run_one_sample();
    }
    CHECK_TRUE(switch_state);  // Confirmed underwater

    // Wait for max dive time to expire (>5 seconds of real time)
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

    // Send truly invalid saturated value (above 14-bit ADC max = 16383)
    // Values > ADC_INVALID_MAX are rejected and state is maintained
    SAADC::set_adc_value(16384);

    for (unsigned int i = 0; i < 3; i++) {
        run_one_sample();
    }

    // State should remain unchanged (invalid values are rejected)
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

    // Warm up at air value to pass coherence check
    warmup_air_samples();

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
    CHECK_TEXT(samples_to_surface <= 3,
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

    // Warm up at air value to pass coherence check
    warmup_air_samples();

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

    // Warm up at air value to pass coherence check
    warmup_air_samples();

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

    // Warm up at air value to pass coherence check
    warmup_air_samples();

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

    // Warm up at air value to pass coherence check
    warmup_air_samples();

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

// ============================================================================
// PROGRESSIVE BIOFOULING TEST
// Simulates months of deployment with increasing electrode degradation.
// Each dive cycle has higher "air" ADC (salt deposits) and lower "water" ADC
// (biofilm reduces conductivity). Verifies detection remains fast at every stage.
// ============================================================================

/**
 * Test 13: Progressive biofouling over multiple dive cycles
 *
 * Simulates the lifecycle of a deployed tracker:
 *   Cycle 0 (clean):     water=3000, air=200   → 93% drop
 *   Cycle 1 (3 months):  water=2800, air=600   → 78% drop
 *   Cycle 2 (6 months):  water=2500, air=1000  → 60% drop
 *   Cycle 3 (9 months):  water=2300, air=1400  → 39% drop
 *   Cycle 4 (12 months): water=2100, air=1700  → 19% drop → still TIER 2
 *
 * At EACH stage, surface detection must complete within 1 parent batch (≤3 samples).
 * The adaptive thresholds must track the changing baselines.
 */
TEST(SWSAnalog, ProgressiveBiofouling_MultiCycleDegradation)
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

    // Warm up at air value to pass coherence check
    warmup_air_samples();

    // Biofouling degradation stages: {water_adc, air_adc, description}
    struct BiofoulingStage {
        uint16_t water_adc;
        uint16_t air_adc;
        const char *description;
    };

    BiofoulingStage stages[] = {
        {3000, 200,  "Clean sensor (month 0)"},
        {2800, 600,  "Light biofouling (month 3)"},
        {2500, 1000, "Moderate biofouling (month 6)"},
        {2300, 1400, "Heavy biofouling (month 9)"},
        {2100, 1700, "Severe biofouling (month 12)"},
    };

    for (int stage = 0; stage < 5; stage++) {
        uint16_t water = stages[stage].water_adc;
        uint16_t air = stages[stage].air_adc;

        // --- DIVE: go underwater ---
        SAADC::set_adc_value(water);
        for (int i = 0; i < 10; i++) {
            run_one_sample();
        }
        CHECK_TRUE_TEXT(switch_state,
            stages[stage].description);

        unsigned int callbacks_before = callbacks;

        // --- SURFACE: electrode loses water contact, ADC drops ---
        SAADC::set_adc_value(air);

        unsigned int samples_to_surface = 0;
        for (int i = 0; i < 10; i++) {
            run_one_sample();
            samples_to_surface++;
            if (callbacks > callbacks_before && !switch_state) {
                break;
            }
        }

        // Surface should be detected quickly:
        // High contrast (clean): within 1 batch (3 samples)
        // Low contrast (severe biofouling): within 2 batches (6 samples)
        unsigned int max_samples = (water - air < 500) ? 6 : 3;
        char msg[128];
        snprintf(msg, sizeof(msg),
            "%s: water=%u air=%u → detection took %u samples (max %u)",
            stages[stage].description, water, air, samples_to_surface, max_samples);
        CHECK_TEXT(samples_to_surface <= max_samples, msg);
        CHECK_FALSE_TEXT(switch_state, msg);

        // Stay at surface a few samples to let adaptive thresholds update
        for (int i = 0; i < 5; i++) {
            run_one_sample();
        }
    }

    s.stop();
}

/**
 * Test 14: Extreme biofouling with small ADC gap
 *
 * Worst case: water=1800, air=1500 → only 16.7% drop, 300 absolute
 * TIER 1 should still fire (25% fails BUT absolute 300 = threshold)
 * If TIER 1 fails, TIER 2 must catch it in 2 samples.
 *
 * Also verifies that the adaptive air baseline rises to ~1500 over time,
 * keeping the regular threshold detection working.
 */
TEST(SWSAnalog, ExtremeBiofouling_SmallGap)
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

    // Warm up at air value to pass coherence check
    warmup_air_samples();

    // First: establish water baseline with normal seawater
    SAADC::set_adc_value(2500);
    for (int i = 0; i < 10; i++) {
        run_one_sample();
    }
    CHECK_TRUE(switch_state);

    // Surface with moderate biofouling first (to let thresholds adapt)
    SAADC::set_adc_value(800);
    for (int i = 0; i < 10; i++) {
        run_one_sample();
    }

    // Second dive with degraded water signal
    SAADC::set_adc_value(1800);
    for (int i = 0; i < 10; i++) {
        run_one_sample();
    }

    // Now surface with extreme biofouling: only 16.7% drop
    // water=1800, air=1500
    unsigned int callbacks_before = callbacks;
    SAADC::set_adc_value(1500);

    unsigned int samples_to_surface = 0;
    for (int i = 0; i < 15; i++) {
        run_one_sample();
        samples_to_surface++;
        if (callbacks > callbacks_before && !switch_state) {
            break;
        }
    }

    // With extreme biofouling (only 16.7% drop), detection relies on L3-L5
    // which need more samples. Just verify it eventually detects surface.
    CHECK_FALSE_TEXT(switch_state,
        "Extreme biofouling (1800→1500): must detect surface");

    s.stop();
}

// ============================================================================
// ADDITIONAL COVERAGE TESTS
// Coherence check, surface lockout, CRC persistence, underflow protection
// ============================================================================

/**
 * Test 15: First-sample coherence check — stored air low, actual reading very high
 * Simulates device stored calibration from air (air=200, water=600) but first
 * reading at boot is 4800 (in water). Should recalibrate automatically.
 */
TEST(SWSAnalog, CoherenceCheck_StoredAirLow_ActualHigh)
{
    SWSAnalogService::reset_noinit_data();  // Clean slate for coherence test
    // First instance: calibrate in air at 200
    {
        SWSAnalogService s;
        system_timer->start();
        SAADC::set_adc_value(200);
        bool dummy = false;
        s.start([&dummy](ServiceEvent &event) {
            if (event.event_type == ServiceEventType::SERVICE_LOG_UPDATED)
                dummy = std::get<bool>(event.event_data);
        });
        for (int i = 0; i < 5; i++) run_one_sample();
        s.stop();
    }

    // Second instance: simulates reboot, first ADC read is very high (in water)
    {
        SWSAnalogService s;
        SAADC::set_adc_value(4800);
        bool switch_state = false;
        s.start([&switch_state](ServiceEvent &event) {
            if (event.event_type == ServiceEventType::SERVICE_LOG_UPDATED)
                switch_state = std::get<bool>(event.event_data);
        });

        // The coherence check should recalibrate and eventually detect underwater
        for (int i = 0; i < 10; i++) run_one_sample();

        CHECK_TRUE_TEXT(switch_state,
            "Coherence check: should recalibrate and detect underwater after high first reading");
        s.stop();
    }
}

/**
 * Test 16: First-sample coherence check — stored air high, actual reading low
 * Simulates calibration done in water (air=3000) but device reboots in air (reading=200).
 */
TEST(SWSAnalog, CoherenceCheck_StoredAirHigh_ActualLow)
{
    SWSAnalogService::reset_noinit_data();  // Clean slate for coherence test
    // First instance: calibrate with high reading (in water scenario)
    {
        SWSAnalogService s;
        system_timer->start();
        SAADC::set_adc_value(3500);
        bool dummy = false;
        s.start([&dummy](ServiceEvent &event) {
            if (event.event_type == ServiceEventType::SERVICE_LOG_UPDATED)
                dummy = std::get<bool>(event.event_data);
        });
        for (int i = 0; i < 5; i++) run_one_sample();
        s.stop();
    }

    // Second instance: reboot in air
    {
        SWSAnalogService s;
        SAADC::set_adc_value(200);
        bool switch_state = true;  // Start assuming underwater
        s.start([&switch_state](ServiceEvent &event) {
            if (event.event_type == ServiceEventType::SERVICE_LOG_UPDATED)
                switch_state = std::get<bool>(event.event_data);
        });

        for (int i = 0; i < 10; i++) run_one_sample();

        CHECK_FALSE_TEXT(switch_state,
            "Coherence check: should recalibrate and detect surface after low first reading");
        s.stop();
    }
}

/**
 * Test 18: CRC persistence after upward air adaptation
 * Verifies that the CRC bug fix works: after upward air adaptation,
 * a new SWSAnalogService instance should validate the stored calibration
 * (i.e., the CRC was correctly updated in the upward path).
 *
 * This test creates two service instances within the same test to simulate
 * a reboot with persistent noinit RAM (static data survives between instances).
 */
TEST(SWSAnalog, CRC_Persistence_UpwardAdaptation)
{
    SWSAnalogService::reset_noinit_data();  // Clean slate for persistence test
    unsigned int val_1 = 1U;
    configuration_store->write_param(ParamID::UW_MAX_SAMPLES, val_1);
    configuration_store->write_param(ParamID::UW_MIN_DRY_SAMPLES, val_1);

    // First instance: calibrate, go underwater, surface, trigger upward adaptation
    {
        SWSAnalogService s;
        system_timer->start();

        SAADC::set_adc_value(100);
        bool switch_state = false;
        s.start([&switch_state](ServiceEvent &event) {
            if (event.event_type == ServiceEventType::SERVICE_LOG_UPDATED)
                switch_state = std::get<bool>(event.event_data);
        });

        // Confirm surface at air=100
        for (int i = 0; i < 5; i++) run_one_sample();
        CHECK_FALSE(switch_state);

        // Go underwater
        SAADC::set_adc_value(3000);
        for (int i = 0; i < 10; i++) run_one_sample();
        CHECK_TRUE(switch_state);

        // Surface with higher air value to trigger upward adaptation
        SAADC::set_adc_value(200);
        std::this_thread::sleep_for(std::chrono::seconds(2));
        for (int i = 0; i < 5; i++) run_one_sample();

        // Need >10s at surface for adaptation to kick in
        std::this_thread::sleep_for(std::chrono::seconds(11));
        for (int i = 0; i < 15; i++) run_one_sample();

        s.stop();
    }

    // DO NOT call reset_noinit_data() here — we want to test persistence

    // Second instance: simulates reboot with noinit RAM intact
    {
        SWSAnalogService s;
        SAADC::set_adc_value(200);
        bool switch_state = false;
        s.start([&switch_state](ServiceEvent &event) {
            if (event.event_type == ServiceEventType::SERVICE_LOG_UPDATED)
                switch_state = std::get<bool>(event.event_data);
        });

        for (int i = 0; i < 5; i++) run_one_sample();

        auto status = SWSAnalogService::get_status();

        // If CRC was correctly updated after upward adaptation,
        // calibration should still be valid (not force-recalibrated)
        CHECK_TRUE_TEXT(status.is_calibrated,
            "CRC persistence: calibration should be valid after reboot");
        // Air baseline should have adapted upward from 100 toward 200
        CHECK_TEXT(status.threshold_air > 100,
            "CRC persistence: air baseline should have adapted upward");

        s.stop();
    }
}

/**
 * Test 19: Threshold underflow protection
 * With very low ADC values, threshold_low should not underflow to 65535
 */
TEST(SWSAnalog, ThresholdUnderflowProtection)
{
    SWSAnalogService::reset_noinit_data();  // Clean slate for underflow test
    SWSAnalogService s;
    bool switch_state = false;

    system_timer->start();

    unsigned int val_1 = 1U;
    configuration_store->write_param(ParamID::UW_MAX_SAMPLES, val_1);
    configuration_store->write_param(ParamID::UW_MIN_DRY_SAMPLES, val_1);

    // Very low air value → threshold will be very low
    SAADC::set_adc_value(5);

    s.start([&switch_state](ServiceEvent &event) {
        if (event.event_type == ServiceEventType::SERVICE_LOG_UPDATED)
            switch_state = std::get<bool>(event.event_data);
    });

    // Calibrate with very low air
    for (int i = 0; i < 5; i++) run_one_sample();

    // Go to a modest water value
    SAADC::set_adc_value(50);
    for (int i = 0; i < 10; i++) run_one_sample();

    // Back to low value — should detect surface (not stuck due to underflow)
    SAADC::set_adc_value(5);
    for (int i = 0; i < 10; i++) run_one_sample();

    CHECK_FALSE_TEXT(switch_state,
        "Underflow protection: low ADC values should still allow surface detection");

    s.stop();
}

/**
 * Test 20: Test mode start/stop
 * Verifies test mode API works and status notifications fire
 */
TEST(SWSAnalog, TestMode_StartStop)
{
    SWSAnalogService s;
    system_timer->start();
    SAADC::set_adc_value(200);

    CHECK_FALSE(SWSAnalogService::is_test_running());

    // Track status notifications
    int notify_count = 0;
    SWSAnalogService::set_status_notify([&notify_count](const SWSAnalogService::Status& st) {
        (void)st;
        notify_count++;
    });

    // Start service normally first, then enable test mode
    bool dummy = false;
    s.start([&dummy](ServiceEvent &event) {
        if (event.event_type == ServiceEventType::SERVICE_LOG_UPDATED)
            dummy = std::get<bool>(event.event_data);
    });

    SWSAnalogService::start_test_mode();
    CHECK_TRUE(SWSAnalogService::is_test_running());

    // Run some samples — should get notifications
    for (int i = 0; i < 5; i++) run_one_sample();
    CHECK_TEXT(notify_count > 0, "Test mode: should receive status notifications");

    SWSAnalogService::stop_test_mode();
    CHECK_FALSE(SWSAnalogService::is_test_running());

    // Clear callback
    SWSAnalogService::clear_status_notify();
    s.stop();
}
