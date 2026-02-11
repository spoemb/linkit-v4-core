#!/usr/bin/env node
/**
 * SWS Analog Parameter Optimizer
 * Recherche des paramètres optimaux par simulation Monte Carlo
 */

// ============================================
// SIMULATOR
// ============================================

class SWSSimulator {
    constructor(params) {
        this.params = {
            thresholdMin: 100,
            thresholdMax: 3000,
            hysteresis: params.hysteresis || 10,
            thresholdRatio: params.thresholdRatio || 40,
            alpha: params.alpha || 0.1,
            movingAvgSize: params.movingAvgSize || 5,
            maxSamples: params.maxSamples || 5,
            minDrySamples: params.minDrySamples || 3,
            maxDiveTime: 7200,
            minSurfaceTime: 10
        };
        this.reset();
    }

    reset() {
        this.calibration = {
            thresholdAir: 200,
            thresholdWater: 600,
            thresholdCurrent: 0,
            hysteresisValue: 0
        };
        this.state = {
            current: 0,
            previous: 0,
            timeInState: 0,
            consecutiveSamples: 0,
            movingAvgBuffer: [],
            // Biofouling detection state (matching firmware)
            minAdcDuringDive: null,
            timeInHysteresis: 0,
            surfaceReadingsBuffer: [],
            // Trend and variance detection (for biofouling surface detection)
            trendBuffer: [],
            decreasingTrendCount: 0,
            varianceValue: 0,
            // Cumulative drop tracking (more robust than consecutive-only)
            peakAdcSinceUnderwater: 0,
            cumulativeDropPercent: 0,
            // Surface lockout after max dive time (prevent immediate re-submersion)
            surfaceLockoutRemaining: 0
        };
        this.updateThreshold();
    }

    updateThreshold() {
        const ratio = this.params.thresholdRatio / 100;
        const range = this.calibration.thresholdWater - this.calibration.thresholdAir;
        this.calibration.thresholdCurrent = this.calibration.thresholdAir + ratio * range;
        this.calibration.hysteresisValue = this.calibration.thresholdCurrent * (this.params.hysteresis / 100);
    }

    movingAverage(value) {
        this.state.movingAvgBuffer.push(value);
        if (this.state.movingAvgBuffer.length > this.params.movingAvgSize) {
            this.state.movingAvgBuffer.shift();
        }
        return this.state.movingAvgBuffer.reduce((a, b) => a + b, 0) / this.state.movingAvgBuffer.length;
    }

    updateWaterBaseline(value) {
        // STRICT CONDITIONS to prevent corruption from biofouling surface readings
        // Only update water baseline if ALL conditions are met:
        // 1. Value is clearly above current threshold (truly underwater)
        // 2. Value is significantly above air baseline (at least 5x)
        // 3. Value meets ABSOLUTE minimum for seawater (> 2000 on 14-bit ADC)
        // 4. AND value is within expected water range OR higher

        const thresholdWithMargin = this.calibration.thresholdCurrent + this.calibration.hysteresisValue;
        const minExpectedWater = this.calibration.thresholdWater * 0.85;

        // CRITICAL: These absolute thresholds prevent biofouling corruption
        const ABSOLUTE_MIN_WATER = 2000;
        const MIN_WATER_AIR_RATIO = 5;

        const clearlyUnderwater = value > thresholdWithMargin;
        const aboveAbsoluteMin = value >= ABSOLUTE_MIN_WATER;
        const significantlyAboveAir = value >= this.calibration.thresholdAir * MIN_WATER_AIR_RATIO;
        const withinExpectedRange = value >= minExpectedWater;
        const salinityIncrease = value > this.calibration.thresholdWater;

        // ALL conditions must be met
        if (clearlyUnderwater && aboveAbsoluteMin && significantlyAboveAir &&
            (withinExpectedRange || salinityIncrease)) {
            this.calibration.thresholdWater =
                this.params.alpha * value + (1 - this.params.alpha) * this.calibration.thresholdWater;
            this.updateThreshold();
        }
    }

    detect(adcValue, deltaTime) {
        this.state.timeInState += deltaTime;

        if (adcValue < 50 || adcValue > 16300) {
            return this.state.current;
        }

        const filtered = this.movingAverage(adcValue);

        // === TREND DETECTION (derivative-based surface detection) ===
        // Track ADC trend to detect surface by decreasing pattern (drying)
        const TREND_BUFFER_SIZE = 8;
        const TREND_DECREASE_THRESHOLD_PERCENT = 2;  // Lowered from 5% for better sensitivity
        const TREND_DECREASE_ABSOLUTE_MIN = 10;      // Minimum 10 ADC for slow drying detection
        const TREND_CONSECUTIVE_DECREASE_MIN = 4;
        const TREND_TOTAL_DROP_PERCENT = 15;
        const CUMULATIVE_DROP_PERCENT = 20;          // Cumulative drop threshold
        const SURFACE_LOCKOUT_DURATION = 30;         // 30s lockout after max dive time

        this.state.trendBuffer.push(filtered);
        if (this.state.trendBuffer.length > TREND_BUFFER_SIZE) {
            this.state.trendBuffer.shift();
        }

        // Track peak ADC while "underwater" (for cumulative drop detection)
        if (this.state.current === 1) {
            if (filtered > this.state.peakAdcSinceUnderwater) {
                this.state.peakAdcSinceUnderwater = filtered;
            }
            // Calculate cumulative drop from peak
            if (this.state.peakAdcSinceUnderwater > 0) {
                this.state.cumulativeDropPercent = Math.round(
                    (this.state.peakAdcSinceUnderwater - filtered) * 100 / this.state.peakAdcSinceUnderwater
                );
            }
        }

        // Check if ADC is decreasing (derivative < 0) with lower threshold for slow drying
        if (this.state.trendBuffer.length >= 2) {
            const prev = this.state.trendBuffer[this.state.trendBuffer.length - 2];
            const curr = this.state.trendBuffer[this.state.trendBuffer.length - 1];
            const percentThreshold = prev * TREND_DECREASE_THRESHOLD_PERCENT / 100;
            const decreaseThreshold = Math.max(TREND_DECREASE_ABSOLUTE_MIN, percentThreshold);
            const isDecreasing = (prev > curr + decreaseThreshold);

            if (isDecreasing) {
                this.state.decreasingTrendCount++;
            } else {
                this.state.decreasingTrendCount = Math.max(0, this.state.decreasingTrendCount - 1);
            }
        }

        // Calculate total drop from start of trend buffer
        let totalDropPercent = 0;
        if (this.state.trendBuffer.length >= TREND_CONSECUTIVE_DECREASE_MIN) {
            const oldest = this.state.trendBuffer[0];
            const newest = this.state.trendBuffer[this.state.trendBuffer.length - 1];
            if (oldest > 0) {
                totalDropPercent = ((oldest - newest) / oldest) * 100;
            }
        }

        // Trend suggests surface if (consecutive decreases AND total drop) OR cumulative drop
        const consecutiveTrend = (this.state.decreasingTrendCount >= TREND_CONSECUTIVE_DECREASE_MIN &&
                                  totalDropPercent >= TREND_TOTAL_DROP_PERCENT);
        const cumulativeTrend = (this.state.cumulativeDropPercent >= CUMULATIVE_DROP_PERCENT &&
                                 this.state.timeInState > 20);  // Only after 20s
        const trendSuggestsSurface = consecutiveTrend || cumulativeTrend;

        // === VARIANCE DETECTION (high variance = unstable surface drying) ===
        const VARIANCE_HIGH_THRESHOLD = 10000;

        if (this.state.trendBuffer.length >= 4) {
            const mean = this.state.trendBuffer.reduce((a, b) => a + b, 0) / this.state.trendBuffer.length;
            const variance = this.state.trendBuffer.reduce((sum, val) => sum + Math.pow(val - mean, 2), 0) / this.state.trendBuffer.length;
            this.state.varianceValue = variance;
        }

        const varianceSuggestsSurface = (this.state.varianceValue >= VARIANCE_HIGH_THRESHOLD);
        const threshHigh = this.calibration.thresholdCurrent + this.calibration.hysteresisValue;
        const threshLow = this.calibration.thresholdCurrent - this.calibration.hysteresisValue;

        // === ADAPTIVE AIR BASELINE (Biofouling compensation) ===
        // Match firmware: When at surface for extended time with elevated readings, adapt air baseline
        if (this.state.current === 0 && this.state.timeInState > 10) {
            this.state.surfaceReadingsBuffer.push(filtered);
            if (this.state.surfaceReadingsBuffer.length > 10) {
                this.state.surfaceReadingsBuffer.shift();
            }

            // Check if readings are elevated (biofouling)
            if (this.state.surfaceReadingsBuffer.length >= 5 &&
                Math.floor(this.state.timeInState) % 10 === 0) {
                const avgSurface = this.state.surfaceReadingsBuffer.reduce((a,b) => a+b, 0) /
                                   this.state.surfaceReadingsBuffer.length;

                // If surface readings significantly above air baseline (>30%)
                if (avgSurface > this.calibration.thresholdAir * 1.3) {
                    // Gradual adaptation (10% per update)
                    this.calibration.thresholdAir = Math.round(
                        this.calibration.thresholdAir * 0.9 + avgSurface * 0.1
                    );
                    this.updateThreshold();
                }
            }
        } else if (this.state.current === 1) {
            this.state.surfaceReadingsBuffer = [];
        }

        // === TRACK MIN ADC DURING DIVE (for biofouling detection) ===
        if (this.state.current === 1) {
            if (this.state.minAdcDuringDive === null || filtered < this.state.minAdcDuringDive) {
                this.state.minAdcDuringDive = filtered;
            }

            // === EXTENDED DIVE RECALIBRATION (60s+, every 30s) ===
            if (this.state.timeInState > 60 && Math.floor(this.state.timeInState) % 30 === 0) {
                const minAdc = this.state.minAdcDuringDive;
                if (minAdc !== null &&
                    minAdc > this.calibration.thresholdAir * 3 &&
                    minAdc < this.calibration.thresholdWater * 0.7) {
                    const newAir = Math.round(minAdc * 0.85);
                    if (newAir > this.calibration.thresholdAir * 1.5) {
                        this.calibration.thresholdAir = newAir;
                        this.updateThreshold();
                        this.state.consecutiveSamples = 0;
                    }
                }
            }
        }

        let newState = this.state.current;

        if (filtered > threshHigh) {
            this.state.consecutiveSamples++;
            this.state.timeInHysteresis = 0;
            if (this.state.consecutiveSamples >= this.params.maxSamples) {
                newState = 1;
                this.updateWaterBaseline(filtered);
            }
        } else if (filtered < threshLow) {
            this.state.timeInHysteresis = 0;
            if (this.state.current === 1) {
                this.state.consecutiveSamples++;
                if (this.state.consecutiveSamples >= this.params.minDrySamples) {
                    newState = 0;
                }
            } else {
                newState = 0;
                this.state.consecutiveSamples = 0;
            }
        } else {
            // === IN HYSTERESIS ZONE ===
            this.state.consecutiveSamples = 0;
            this.state.timeInHysteresis += deltaTime;

            // === HYSTERESIS STUCK DETECTION (30s timeout) ===
            if (this.state.timeInHysteresis > 30 && Math.floor(this.state.timeInHysteresis) % 15 === 0) {
                const newAir = Math.round(filtered * 0.75);
                if (newAir > this.calibration.thresholdAir * 1.2) {
                    this.calibration.thresholdAir = newAir;
                    this.updateThreshold();
                }
            }
        }

        // === BIOFOULING SURFACE OVERRIDE ===
        // If trend or variance suggests surface AND we're currently underwater with biofouling
        // Force surface detection even if ADC is above threshold
        const ABSOLUTE_MIN_WATER = 2000;
        let biofoulingSurfaceOverride = false;

        if (this.state.current === 1 && this.state.timeInState > 30) {
            // Check for biofouling surface pattern
            if ((trendSuggestsSurface || varianceSuggestsSurface) &&
                filtered < ABSOLUTE_MIN_WATER) {
                biofoulingSurfaceOverride = true;
                // Recalibrate air baseline to current elevated level
                this.calibration.thresholdAir = Math.round(filtered * 0.9);
                this.updateThreshold();
            }
        }

        if (biofoulingSurfaceOverride) {
            newState = 0;
            this.state.consecutiveSamples = 0;
        }

        // Decrement surface lockout timer
        if (this.state.surfaceLockoutRemaining > 0) {
            this.state.surfaceLockoutRemaining = Math.max(0, this.state.surfaceLockoutRemaining - deltaTime);
        }

        // === MAX DIVE TIME with RECALIBRATION and LOCKOUT ===
        if (newState === 1 && this.state.timeInState > this.params.maxDiveTime) {
            // Recalibrate air baseline if biofouling suspected
            const minAdc = this.state.minAdcDuringDive;
            if (minAdc !== null && minAdc > this.calibration.thresholdAir * 2) {
                this.calibration.thresholdAir = Math.round(minAdc * 0.8);
                this.updateThreshold();
            }
            newState = 0;
            // CRITICAL: Set surface lockout to prevent immediate re-submersion
            this.state.surfaceLockoutRemaining = SURFACE_LOCKOUT_DURATION;
        }

        // Surface lockout: prevent return to underwater during lockout period
        if (this.state.surfaceLockoutRemaining > 0 && newState === 1) {
            newState = 0;
        }

        // Min surface time
        if (this.state.current === 0 && this.state.timeInState < this.params.minSurfaceTime && newState === 1) {
            newState = 0;
        }

        // State change handling
        if (newState !== this.state.current) {
            this.state.previous = this.state.current;
            this.state.current = newState;
            this.state.timeInState = 0;
            this.state.consecutiveSamples = 0;
            this.state.minAdcDuringDive = null;
            this.state.timeInHysteresis = 0;
            // Reset trend/variance detection on state change
            this.state.trendBuffer = [];
            this.state.decreasingTrendCount = 0;
            this.state.varianceValue = 0;
            // Reset cumulative tracking on state change
            this.state.peakAdcSinceUnderwater = 0;
            this.state.cumulativeDropPercent = 0;
            return { changed: true, state: newState };
        }

        return { changed: false, state: this.state.current };
    }
}

// ============================================
// SCENARIO RUNNER
// ============================================

class ScenarioRunner {
    generateADC(baseValue, noise = 20) {
        return Math.max(0, Math.min(16383, baseValue + (Math.random() - 0.5) * 2 * noise));
    }

    normalDive(time, context) {
        const cycle = time % 120;
        if (cycle < 30) return { adc: this.generateADC(200, 30), expectedState: 0 };
        if (cycle < 90) return { adc: this.generateADC(2500, 100), expectedState: 1 };
        return { adc: this.generateADC(200, 30), expectedState: 0 };
    }

    splashTest(time, context) {
        if (Math.random() < 0.08) {
            return { adc: this.generateADC(1200, 200), expectedState: 0, isSplash: true };
        }
        return { adc: this.generateADC(180, 25), expectedState: 0 };
    }

    salinityChange(time, context) {
        const phase = Math.floor(time / 90) % 5;
        const levels = [
            { adc: 200, state: 0 },
            { adc: 1200, state: 1 },
            { adc: 2500, state: 1 },
            { adc: 4000, state: 1 },
            { adc: 200, state: 0 }
        ];
        const level = levels[phase];
        return { adc: this.generateADC(level.adc, level.adc * 0.05), expectedState: level.state };
    }

    electrodeDrift(time, context) {
        const driftFactor = Math.max(0.5, 1 - (time / 1200) * 0.3);
        const cycle = time % 80;
        if (cycle < 30) {
            return { adc: this.generateADC(200 / driftFactor, 40), expectedState: 0 };
        }
        return { adc: this.generateADC(2500 * driftFactor, 150), expectedState: 1 };
    }

    rapidTransitions(time, context) {
        const cycle = time % 20;
        if (cycle < 8) return { adc: this.generateADC(180, 20), expectedState: 0 };
        if (cycle < 16) return { adc: this.generateADC(2800, 80), expectedState: 1 };
        return { adc: this.generateADC(180, 20), expectedState: 0 };
    }

    longDive(time, context) {
        if (time < 30) return { adc: this.generateADC(200, 30), expectedState: 0 };
        const variation = Math.sin(time / 60) * 200;
        return { adc: this.generateADC(2600 + variation, 100), expectedState: 1 };
    }

    highNoise(time, context) {
        const cycle = time % 100;
        if (cycle < 40) return { adc: this.generateADC(200, 300), expectedState: 0 };
        return { adc: this.generateADC(2500, 300), expectedState: 1 };
    }

    // ===== BIOFOULING / AGING SCENARIOS =====
    // Critical for turtle trackers after long-term deployment

    aged6m(time, context) {
        // 6 months exposure - moderate contamination
        // Residual conductivity ~300, slow ADC decay when surfacing
        const residual = 300;
        const cycle = time % 90;

        if (cycle < 25) {
            // Surface - ADC stays elevated and decays slowly
            const decayProgress = cycle / 25;
            const wetValue = 1800;
            const dryTarget = residual + 100;
            const currentAdc = wetValue - (wetValue - dryTarget) * Math.pow(decayProgress, 0.3);
            return { adc: this.generateADC(currentAdc, 60), expectedState: 0 };
        }
        return { adc: this.generateADC(2400, 100), expectedState: 1 };
    }

    aged1y(time, context) {
        // 1 year exposure - heavy contamination
        // Residual ~600, very slow decay, challenging surface detection
        const residual = 600;
        const cycle = time % 120;

        if (cycle < 40) {
            // Surface - very slow drying, ADC stays HIGH
            const decayProgress = cycle / 40;
            const wetValue = 2200;
            const dryTarget = residual + 200;
            const currentAdc = wetValue - (wetValue - dryTarget) * Math.pow(decayProgress, 0.2);
            return { adc: this.generateADC(currentAdc, 100), expectedState: 0 };
        }
        return { adc: this.generateADC(2600, 125), expectedState: 1 };
    }

    extreme(time, context) {
        // Extreme contamination - worst case
        // Surface detection very challenging
        const residual = 900;
        const cycle = time % 60;

        if (cycle < 30) {
            // ADC barely drops, stays near threshold
            const decayProgress = cycle / 30;
            const wetValue = 2500;
            const dryTarget = residual + 300;
            const currentAdc = wetValue - (wetValue - dryTarget) * Math.pow(decayProgress, 0.15);
            return { adc: this.generateADC(currentAdc, 150), expectedState: 0 };
        }
        return { adc: this.generateADC(2800, 150), expectedState: 1 };
    }

    adaptiveAir(time, context) {
        // Air baseline drifts up over time (biofouling accumulation)
        const baselineDrift = Math.min(400, time * 0.5);
        const cycle = time % 100;

        if (cycle < 35) {
            return { adc: this.generateADC(180 + baselineDrift, 40), expectedState: 0 };
        }
        return { adc: this.generateADC(2500, 100), expectedState: 1 };
    }

    run(scenarioName, duration, params) {
        const simulator = new SWSSimulator(params);
        const scenarios = {
            normal: this.normalDive.bind(this),
            splash: this.splashTest.bind(this),
            salinity: this.salinityChange.bind(this),
            drift: this.electrodeDrift.bind(this),
            rapid: this.rapidTransitions.bind(this),
            long: this.longDive.bind(this),
            noise: this.highNoise.bind(this),
            // Biofouling scenarios - critical for real-world turtle tracking
            aged6m: this.aged6m.bind(this),
            aged1y: this.aged1y.bind(this),
            extreme: this.extreme.bind(this),
            adaptiveAir: this.adaptiveAir.bind(this)
        };
        const scenario = scenarios[scenarioName];
        const context = {};

        const metrics = {
            correctDetections: 0,
            totalSamples: 0,
            falsePositives: 0,
            falseNegatives: 0,
            transitionLatencies: [],
            oscillations: 0,
            lastState: 0
        };

        let pendingTransition = null;
        let lastExpectedState = 0;
        const dt = 1;

        for (let t = 0; t < duration; t += dt) {
            const { adc, expectedState, isSplash } = scenario(t, context);
            const result = simulator.detect(adc, dt);

            if (expectedState !== lastExpectedState) {
                pendingTransition = { time: t, targetState: expectedState };
            }
            lastExpectedState = expectedState;

            metrics.totalSamples++;

            if (result.state === expectedState) {
                metrics.correctDetections++;
                if (pendingTransition && result.state === pendingTransition.targetState) {
                    metrics.transitionLatencies.push(t - pendingTransition.time);
                    pendingTransition = null;
                }
            } else {
                if (result.state === 1 && expectedState === 0 && !isSplash) {
                    metrics.falsePositives++;
                } else if (result.state === 0 && expectedState === 1) {
                    metrics.falseNegatives++;
                }
            }

            if (result.changed) {
                const timeSinceLastChange = t - (context.lastChangeTime || 0);
                if (timeSinceLastChange < 5) metrics.oscillations++;
                context.lastChangeTime = t;
                metrics.lastState = result.state;
            }
        }

        const detectionRate = metrics.correctDetections / metrics.totalSamples;
        const avgLatency = metrics.transitionLatencies.length > 0
            ? metrics.transitionLatencies.reduce((a, b) => a + b, 0) / metrics.transitionLatencies.length
            : 0;
        const stability = 1 - Math.min(1, metrics.oscillations / 20);

        return {
            detectionRate,
            falsePositiveRate: metrics.falsePositives / Math.max(1, metrics.totalSamples),
            falseNegativeRate: metrics.falseNegatives / Math.max(1, metrics.totalSamples),
            avgLatency,
            stability,
            oscillations: metrics.oscillations
        };
    }
}

// ============================================
// OPTIMIZER
// ============================================

class ParameterOptimizer {
    constructor() {
        this.runner = new ScenarioRunner();
        this.results = [];
        this.bestResult = null;
    }

    randomParams(ranges) {
        const rand = (min, max, isInt = false) => {
            const val = min + Math.random() * (max - min);
            return isInt ? Math.round(val) : val;
        };

        return {
            hysteresis: rand(ranges.hysteresis.min, ranges.hysteresis.max),
            thresholdRatio: rand(ranges.thresholdRatio.min, ranges.thresholdRatio.max),
            alpha: rand(ranges.alpha.min, ranges.alpha.max),
            movingAvgSize: rand(ranges.movingAvgSize.min, ranges.movingAvgSize.max, true),
            maxSamples: rand(ranges.maxSamples.min, ranges.maxSamples.max, true),
            minDrySamples: rand(ranges.minDrySamples.min, ranges.minDrySamples.max, true)
        };
    }

    calculateScore(scenarioResults) {
        // Scoring optimized for FAST SURFACE DETECTION even with biofouling
        // Priority: Surface latency > False negatives > Detection > Stability > False positives
        const weights = {
            detectionRate: 0.20,
            falsePositive: 0.10,
            falseNegative: 0.30,   // Missing surface is CRITICAL for turtle tracking
            latency: 0.25,         // Fast response is essential
            stability: 0.15
        };

        // Aging scenarios get higher weight (more important for real deployment)
        const agingScenarios = ['aged6m', 'aged1y', 'extreme', 'adaptiveAir'];

        let totalScore = 0;
        let totalWeight = 0;

        for (const [scenario, result] of Object.entries(scenarioResults)) {
            const detectionScore = result.detectionRate;
            const fpScore = 1 - Math.min(1, result.falsePositiveRate * 50);
            const fnScore = 1 - Math.min(1, result.falseNegativeRate * 100);  // Heavy penalty
            const latencyScore = Math.max(0, 1 - result.avgLatency / 15);     // Target <15s latency
            const stabilityScore = result.stability;

            const scenarioScore =
                weights.detectionRate * detectionScore +
                weights.falsePositive * fpScore +
                weights.falseNegative * fnScore +
                weights.latency * latencyScore +
                weights.stability * stabilityScore;

            // Biofouling scenarios are weighted higher
            const scenarioWeight = agingScenarios.includes(scenario) ? 1.5 : 1.0;
            totalScore += scenarioScore * scenarioWeight;
            totalWeight += scenarioWeight;
        }

        return totalWeight > 0 ? totalScore / totalWeight : 0;
    }

    optimize(config) {
        const {
            iterations = 1000,
            duration = 600,
            scenarios = ['normal', 'splash', 'salinity', 'drift', 'rapid', 'long', 'noise', 'aged6m', 'aged1y', 'extreme', 'adaptiveAir'],
            ranges = {
                hysteresis: { min: 5, max: 25 },
                thresholdRatio: { min: 25, max: 60 },
                alpha: { min: 0.05, max: 0.3 },
                movingAvgSize: { min: 3, max: 10 },
                maxSamples: { min: 2, max: 8 },
                minDrySamples: { min: 2, max: 6 }
            }
        } = config;

        console.log('\n╔══════════════════════════════════════════════════════════════╗');
        console.log('║         SWS ANALOG PARAMETER OPTIMIZER                       ║');
        console.log('╚══════════════════════════════════════════════════════════════╝\n');
        console.log(`Configuration:`);
        console.log(`  - Itérations: ${iterations}`);
        console.log(`  - Durée simulation: ${duration}s`);
        console.log(`  - Scénarios: ${scenarios.join(', ')}`);
        console.log('');

        const startTime = Date.now();

        for (let i = 0; i < iterations; i++) {
            const params = this.randomParams(ranges);
            const scenarioResults = {};

            for (const scenario of scenarios) {
                scenarioResults[scenario] = this.runner.run(scenario, duration, params);
            }

            const score = this.calculateScore(scenarioResults);

            const result = {
                params,
                scenarioResults,
                score,
                detectionRate: Object.values(scenarioResults).reduce((s, r) => s + r.detectionRate, 0) / scenarios.length,
                falsePositives: Object.values(scenarioResults).reduce((s, r) => s + r.falsePositiveRate, 0) / scenarios.length,
                falseNegatives: Object.values(scenarioResults).reduce((s, r) => s + r.falseNegativeRate, 0) / scenarios.length,
                avgLatency: Object.values(scenarioResults).reduce((s, r) => s + r.avgLatency, 0) / scenarios.length,
                stability: Object.values(scenarioResults).reduce((s, r) => s + r.stability, 0) / scenarios.length
            };

            this.results.push(result);

            if (!this.bestResult || score > this.bestResult.score) {
                this.bestResult = result;
                console.log(`[${i + 1}/${iterations}] Nouveau meilleur score: ${(score * 100).toFixed(2)}%`);
            }

            // Progress update
            if ((i + 1) % 100 === 0) {
                const elapsed = (Date.now() - startTime) / 1000;
                const eta = (elapsed / (i + 1)) * (iterations - i - 1);
                process.stdout.write(`\rProgression: ${i + 1}/${iterations} (${((i + 1) / iterations * 100).toFixed(1)}%) - ETA: ${Math.round(eta)}s   `);
            }
        }

        const elapsed = (Date.now() - startTime) / 1000;
        console.log(`\n\nOptimisation terminée en ${elapsed.toFixed(1)}s\n`);

        // Sort results
        this.results.sort((a, b) => b.score - a.score);

        return this.results;
    }

    printResults(topN = 10) {
        console.log('\n╔══════════════════════════════════════════════════════════════╗');
        console.log('║                    TOP RÉSULTATS                             ║');
        console.log('╚══════════════════════════════════════════════════════════════╝\n');

        const top = this.results.slice(0, topN);

        console.log('┌─────┬────────┬──────────┬─────────┬─────────┬─────────┬──────────┬───────┬───────┬───────┬────────┬────────┬────────┐');
        console.log('│ #   │ Score  │ Détect.  │ Faux+   │ Faux-   │ Latence │ Stabilité│ Hyst% │ Ratio%│ Alpha │ AvgSz  │ MaxSmp │ DrySmp │');
        console.log('├─────┼────────┼──────────┼─────────┼─────────┼─────────┼──────────┼───────┼───────┼───────┼────────┼────────┼────────┤');

        top.forEach((r, i) => {
            console.log(
                `│ ${(i + 1).toString().padStart(3)} │ ` +
                `${(r.score * 100).toFixed(1).padStart(5)}% │ ` +
                `${(r.detectionRate * 100).toFixed(1).padStart(7)}% │ ` +
                `${(r.falsePositives * 100).toFixed(2).padStart(6)}% │ ` +
                `${(r.falseNegatives * 100).toFixed(2).padStart(6)}% │ ` +
                `${r.avgLatency.toFixed(1).padStart(6)}s │ ` +
                `${(r.stability * 100).toFixed(0).padStart(7)}% │ ` +
                `${r.params.hysteresis.toFixed(1).padStart(5)} │ ` +
                `${r.params.thresholdRatio.toFixed(1).padStart(5)} │ ` +
                `${r.params.alpha.toFixed(2).padStart(5)} │ ` +
                `${r.params.movingAvgSize.toString().padStart(6)} │ ` +
                `${r.params.maxSamples.toString().padStart(6)} │ ` +
                `${r.params.minDrySamples.toString().padStart(6)} │`
            );
        });

        console.log('└─────┴────────┴──────────┴─────────┴─────────┴─────────┴──────────┴───────┴───────┴───────┴────────┴────────┴────────┘');
    }

    printBestConfig() {
        if (!this.bestResult) return;

        const p = this.bestResult.params;
        const r = this.bestResult;

        console.log('\n╔══════════════════════════════════════════════════════════════╗');
        console.log('║              CONFIGURATION OPTIMALE                          ║');
        console.log('╚══════════════════════════════════════════════════════════════╝\n');

        console.log(`Score global: ${(r.score * 100).toFixed(2)}%\n`);

        console.log('┌────────────────────────────────┬───────────────────┐');
        console.log('│ Paramètre                      │ Valeur            │');
        console.log('├────────────────────────────────┼───────────────────┤');
        console.log(`│ SWS_ANALOG_HYSTERESIS          │ ${Math.round(p.hysteresis).toString().padStart(15)}% │`);
        console.log(`│ THRESHOLD_RATIO                │ ${Math.round(p.thresholdRatio).toString().padStart(15)}% │`);
        console.log(`│ ALPHA (EMA)                    │ ${p.alpha.toFixed(2).padStart(17)} │`);
        console.log(`│ MOVING_AVG_SIZE                │ ${p.movingAvgSize.toString().padStart(17)} │`);
        console.log(`│ UW_MAX_SAMPLES                 │ ${p.maxSamples.toString().padStart(17)} │`);
        console.log(`│ UW_MIN_DRY_SAMPLES             │ ${p.minDrySamples.toString().padStart(17)} │`);
        console.log('└────────────────────────────────┴───────────────────┘');

        console.log('\nMétriques de performance:');
        console.log('┌────────────────────────────────┬───────────────────┐');
        console.log(`│ Taux de détection              │ ${(r.detectionRate * 100).toFixed(2).padStart(15)}% │`);
        console.log(`│ Taux faux positifs             │ ${(r.falsePositives * 100).toFixed(3).padStart(15)}% │`);
        console.log(`│ Taux faux négatifs             │ ${(r.falseNegatives * 100).toFixed(3).padStart(15)}% │`);
        console.log(`│ Latence moyenne                │ ${r.avgLatency.toFixed(2).padStart(15)}s │`);
        console.log(`│ Stabilité                      │ ${(r.stability * 100).toFixed(1).padStart(15)}% │`);
        console.log('└────────────────────────────────┴───────────────────┘');

        console.log('\nPerformance par scénario:');
        console.log('┌──────────────────────┬───────────┬───────────┬───────────┐');
        console.log('│ Scénario             │ Détection │ Stabilité │ Latence   │');
        console.log('├──────────────────────┼───────────┼───────────┼───────────┤');

        const scenarioNames = {
            normal: 'Plongée normale',
            splash: 'Éclaboussures',
            salinity: 'Salinité variable',
            drift: 'Dérive électrodes',
            rapid: 'Transitions rapides',
            long: 'Plongée longue',
            noise: 'Bruit élevé',
            // Biofouling scenarios
            aged6m: '🦠 6 mois biofouling',
            aged1y: '🦠 1 an biofouling',
            extreme: '💀 Extrême',
            adaptiveAir: '📈 Air adaptatif'
        };

        for (const [key, result] of Object.entries(r.scenarioResults)) {
            const name = scenarioNames[key] || key;
            console.log(
                `│ ${name.padEnd(20)} │ ` +
                `${(result.detectionRate * 100).toFixed(1).padStart(7)}% │ ` +
                `${(result.stability * 100).toFixed(0).padStart(7)}% │ ` +
                `${result.avgLatency.toFixed(1).padStart(7)}s │`
            );
        }
        console.log('└──────────────────────┴───────────┴───────────┴───────────┘');

        console.log('\n// Code à intégrer dans config_store.hpp:');
        console.log('// ========================================\n');
        console.log(`#define SWS_ANALOG_HYSTERESIS       ${Math.round(p.hysteresis)}    // % - Optimisé`);
        console.log(`#define SWS_THRESHOLD_RATIO         ${Math.round(p.thresholdRatio)}    // % - Optimisé`);
        console.log(`#define SWS_ALPHA_EMA               ${p.alpha.toFixed(2)}f  // Optimisé`);
        console.log(`#define SWS_MOVING_AVG_SIZE         ${p.movingAvgSize}     // Optimisé`);
        console.log(`#define UW_MAX_SAMPLES              ${p.maxSamples}     // Optimisé`);
        console.log(`#define UW_MIN_DRY_SAMPLES          ${p.minDrySamples}     // Optimisé`);
    }

    analyzeParameterSensitivity() {
        console.log('\n╔══════════════════════════════════════════════════════════════╗');
        console.log('║              ANALYSE DE SENSIBILITÉ                          ║');
        console.log('╚══════════════════════════════════════════════════════════════╝\n');

        const top20 = this.results.slice(0, 20);
        const params = ['hysteresis', 'thresholdRatio', 'alpha', 'movingAvgSize', 'maxSamples', 'minDrySamples'];

        console.log('Plages des 20 meilleurs résultats:\n');

        params.forEach(param => {
            const values = top20.map(r => r.params[param]);
            const min = Math.min(...values);
            const max = Math.max(...values);
            const avg = values.reduce((a, b) => a + b, 0) / values.length;
            const std = Math.sqrt(values.reduce((s, v) => s + Math.pow(v - avg, 2), 0) / values.length);

            console.log(`${param.padEnd(20)}: min=${min.toFixed(2).padStart(6)} max=${max.toFixed(2).padStart(6)} avg=${avg.toFixed(2).padStart(6)} std=${std.toFixed(2).padStart(6)}`);
        });

        // Recommendations
        console.log('\n💡 Recommandations:\n');

        const best = this.bestResult.params;

        if (best.hysteresis > 15) {
            console.log('  • Hystérésis élevée (>15%) : Bon pour environnements bruités');
            console.log('    mais peut ralentir la détection des transitions.');
        } else if (best.hysteresis < 8) {
            console.log('  • Hystérésis faible (<8%) : Très réactif mais plus sensible au bruit.');
        } else {
            console.log('  • Hystérésis modérée : Bon équilibre réactivité/stabilité.');
        }

        if (best.alpha < 0.1) {
            console.log('  • Alpha faible (<0.1) : Adaptation lente de la calibration eau,');
            console.log('    plus robuste aux variations temporaires de salinité.');
        } else if (best.alpha > 0.2) {
            console.log('  • Alpha élevé (>0.2) : Adaptation rapide aux changements de salinité,');
            console.log('    mais peut être instable avec des variations rapides.');
        }

        if (best.maxSamples > 5) {
            console.log('  • Confirmation élevée (>5 samples) : Très résistant aux faux positifs');
            console.log('    mais latence de détection plus importante.');
        }

        if (best.movingAvgSize > 7) {
            console.log('  • Filtrage important (>7) : Signal très lissé, excellent pour bruit élevé.');
        }
    }
}

// ============================================
// MAIN
// ============================================

console.log('Démarrage de l\'optimisation des paramètres SWS Analog...\n');

const optimizer = new ParameterOptimizer();

// Configuration de l'optimisation - INCLUT BIOFOULING SCENARIOS
const config = {
    iterations: 3000,
    duration: 600,
    // Include biofouling scenarios for real-world turtle tracker optimization
    scenarios: [
        'normal', 'splash', 'salinity', 'drift', 'rapid', 'long', 'noise',
        'aged6m', 'aged1y', 'extreme', 'adaptiveAir'  // Biofouling scenarios
    ],
    ranges: {
        hysteresis: { min: 8, max: 20 },       // Moderate hysteresis for stability
        thresholdRatio: { min: 35, max: 55 },  // Lower ratio = faster surface detection
        alpha: { min: 0.08, max: 0.25 },       // Higher alpha = faster adaptation
        movingAvgSize: { min: 2, max: 5 },     // Smaller = faster response
        maxSamples: { min: 1, max: 4 },        // Fewer = faster dive detection
        minDrySamples: { min: 2, max: 6 }      // Critical for surface detection speed
    }
};

optimizer.optimize(config);
optimizer.printResults(15);
optimizer.printBestConfig();
optimizer.analyzeParameterSensitivity();

console.log('\n✅ Optimisation terminée!\n');
