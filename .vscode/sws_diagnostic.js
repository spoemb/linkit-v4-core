#!/usr/bin/env node
/**
 * SWS Analog Diagnostic - Analyse détaillée du comportement
 * Simule un scénario de biofouling et trace toutes les variables internes
 */

class SWSSimulator {
    constructor() {
        this.config = {
            thresholdRatio: 37,
            hysteresis: 9,
            alpha: 0.13,
            movingAvgSize: 2,
            maxSamples: 1,
            minDrySamples: 5,
            maxDiveTime: 120,  // Réduit pour test rapide (2 min au lieu de 2h)
            minSurfaceTime: 10
        };

        this.reset();
    }

    reset() {
        this.calibration = {
            thresholdAir: 200,
            thresholdWater: 2500,
            thresholdCurrent: 0,
            hysteresisValue: 0
        };

        this.state = {
            current: 0,  // 0=surface, 1=underwater
            timeInState: 0,
            consecutiveSamples: 0,
            movingAvgBuffer: [],
            trendBuffer: [],
            decreasingTrendCount: 0,
            varianceValue: 0,
            minAdcDuringDive: null,
            surfaceReadingsBuffer: [],
            // Cumulative drop tracking
            peakAdcSinceUnderwater: 0,
            cumulativeDropPercent: 0,
            // Surface lockout
            surfaceLockoutRemaining: 0
        };

        this.updateThreshold();
    }

    updateThreshold() {
        const ratio = this.config.thresholdRatio / 100;
        const range = this.calibration.thresholdWater - this.calibration.thresholdAir;
        this.calibration.thresholdCurrent = Math.round(this.calibration.thresholdAir + ratio * range);
        this.calibration.hysteresisValue = Math.round(this.calibration.thresholdCurrent * (this.config.hysteresis / 100));
    }

    movingAverage(value) {
        this.state.movingAvgBuffer.push(value);
        if (this.state.movingAvgBuffer.length > this.config.movingAvgSize) {
            this.state.movingAvgBuffer.shift();
        }
        return Math.round(this.state.movingAvgBuffer.reduce((a, b) => a + b, 0) / this.state.movingAvgBuffer.length);
    }

    detect(adcValue) {
        this.state.timeInState++;

        const filtered = this.movingAverage(adcValue);

        // === TREND DETECTION ===
        const TREND_BUFFER_SIZE = 8;
        const TREND_DECREASE_THRESHOLD = 2;  // 2%
        const TREND_DECREASE_ABSOLUTE_MIN = 10;  // Min 10 ADC for slow drying
        const TREND_CONSECUTIVE_MIN = 4;
        const TREND_TOTAL_DROP = 15;  // 15%
        const CUMULATIVE_DROP_THRESHOLD = 20;  // 20% cumulative
        const VARIANCE_THRESHOLD = 10000;
        const ABSOLUTE_MIN_WATER = 2000;
        const SURFACE_LOCKOUT_DURATION = 30;

        // Get previous value
        const prevValue = this.state.trendBuffer.length > 0 ?
            this.state.trendBuffer[this.state.trendBuffer.length - 1] : 0;

        // Add to trend buffer
        this.state.trendBuffer.push(filtered);
        if (this.state.trendBuffer.length > TREND_BUFFER_SIZE) {
            this.state.trendBuffer.shift();
        }

        // Track peak ADC while "underwater" (for cumulative drop)
        if (this.state.current === 1) {
            if (filtered > this.state.peakAdcSinceUnderwater) {
                this.state.peakAdcSinceUnderwater = filtered;
            }
            if (this.state.peakAdcSinceUnderwater > 0) {
                this.state.cumulativeDropPercent = Math.round(
                    (this.state.peakAdcSinceUnderwater - filtered) * 100 / this.state.peakAdcSinceUnderwater
                );
            }
        }

        // Check decrease (with lower threshold for slow drying)
        let isDecreasing = false;
        if (prevValue > 0) {
            const percentThreshold = prevValue * TREND_DECREASE_THRESHOLD / 100;
            const threshold = Math.max(TREND_DECREASE_ABSOLUTE_MIN, percentThreshold);
            isDecreasing = (prevValue > filtered + threshold);
        }

        if (isDecreasing) {
            this.state.decreasingTrendCount++;
        } else {
            this.state.decreasingTrendCount = Math.max(0, this.state.decreasingTrendCount - 1);
        }

        // Calculate total drop
        const trendMax = Math.max(...this.state.trendBuffer);
        const trendMin = Math.min(...this.state.trendBuffer);
        const totalDropPercent = trendMax > 0 ? Math.round((trendMax - trendMin) * 100 / trendMax) : 0;

        // Calculate variance
        if (this.state.trendBuffer.length >= 4) {
            const mean = this.state.trendBuffer.reduce((a, b) => a + b, 0) / this.state.trendBuffer.length;
            const sumSq = this.state.trendBuffer.reduce((sum, val) => sum + Math.pow(val - mean, 2), 0);
            this.state.varianceValue = Math.round(sumSq / this.state.trendBuffer.length);
        }

        // Trend suggests surface: consecutive OR cumulative
        const consecutiveTrend = (this.state.decreasingTrendCount >= TREND_CONSECUTIVE_MIN &&
                                  totalDropPercent >= TREND_TOTAL_DROP);
        const cumulativeTrend = (this.state.cumulativeDropPercent >= CUMULATIVE_DROP_THRESHOLD &&
                                 this.state.timeInState > 20);
        const trendSuggestsSurface = consecutiveTrend || cumulativeTrend;
        const varianceSuggestsSurface = (this.state.varianceValue >= VARIANCE_THRESHOLD);

        // Thresholds
        const threshHigh = this.calibration.thresholdCurrent + this.calibration.hysteresisValue;
        const threshLow = this.calibration.thresholdCurrent - this.calibration.hysteresisValue;

        let newState = this.state.current;
        let reason = '';

        // === BIOFOULING OVERRIDE ===
        let biofoulingOverride = false;
        if (this.state.current === 1 && this.state.timeInState > 30) {
            if ((trendSuggestsSurface || varianceSuggestsSurface) && filtered < ABSOLUTE_MIN_WATER) {
                biofoulingOverride = true;
                newState = 0;
                reason = `BIOFOULING_OVERRIDE (trend=${trendSuggestsSurface}, var=${varianceSuggestsSurface})`;

                // Recalibrate
                if (trendMin > this.calibration.thresholdAir) {
                    this.calibration.thresholdAir = Math.round(trendMin * 0.9);
                    this.updateThreshold();
                }
            }
        }

        if (!biofoulingOverride) {
            if (filtered > threshHigh) {
                this.state.consecutiveSamples++;
                if (this.state.consecutiveSamples >= this.config.maxSamples) {
                    newState = 1;
                    reason = 'ABOVE_THRESHOLD';

                    // Update water baseline (with protection)
                    if (filtered >= ABSOLUTE_MIN_WATER && filtered >= this.calibration.thresholdAir * 5) {
                        this.calibration.thresholdWater = Math.round(
                            this.config.alpha * filtered + (1 - this.config.alpha) * this.calibration.thresholdWater
                        );
                        this.updateThreshold();
                    }
                }
            } else if (filtered < threshLow) {
                if (this.state.current === 1) {
                    this.state.consecutiveSamples++;
                    if (this.state.consecutiveSamples >= this.config.minDrySamples) {
                        newState = 0;
                        reason = 'BELOW_THRESHOLD';
                    }
                } else {
                    newState = 0;
                    this.state.consecutiveSamples = 0;
                }
            } else {
                this.state.consecutiveSamples = 0;
                reason = 'IN_HYSTERESIS';
            }
        }

        // Track min ADC during dive
        if (this.state.current === 1) {
            if (this.state.minAdcDuringDive === null || filtered < this.state.minAdcDuringDive) {
                this.state.minAdcDuringDive = filtered;
            }
        }

        // Decrement surface lockout
        if (this.state.surfaceLockoutRemaining > 0) {
            this.state.surfaceLockoutRemaining--;
        }

        // === MAX DIVE TIME with LOCKOUT ===
        if (newState === 1 && this.state.timeInState > this.config.maxDiveTime) {
            newState = 0;
            reason = 'MAX_DIVE_TIME';

            // Recalibrate
            const minAdc = this.state.minAdcDuringDive;
            if (minAdc && minAdc > this.calibration.thresholdAir * 2) {
                this.calibration.thresholdAir = Math.round(minAdc * 0.8);
                this.updateThreshold();
            }

            // Set surface lockout to prevent immediate re-submersion
            this.state.surfaceLockoutRemaining = SURFACE_LOCKOUT_DURATION;
        }

        // Surface lockout: prevent return to underwater during lockout
        if (this.state.surfaceLockoutRemaining > 0 && newState === 1) {
            newState = 0;
            reason = 'SURFACE_LOCKOUT';
        }

        // State change
        if (newState !== this.state.current) {
            this.state.current = newState;
            this.state.timeInState = 0;
            this.state.consecutiveSamples = 0;
            this.state.trendBuffer = [];
            this.state.decreasingTrendCount = 0;
            this.state.varianceValue = 0;
            this.state.minAdcDuringDive = null;
            // Reset cumulative tracking
            this.state.peakAdcSinceUnderwater = 0;
            this.state.cumulativeDropPercent = 0;
        }

        return {
            filtered,
            state: newState,
            reason,
            threshLow,
            threshHigh,
            trendCount: this.state.decreasingTrendCount,
            totalDrop: totalDropPercent,
            cumulativeDrop: this.state.cumulativeDropPercent,
            variance: this.state.varianceValue,
            trendSurf: trendSuggestsSurface,
            varSurf: varianceSuggestsSurface,
            timeInState: this.state.timeInState,
            air: this.calibration.thresholdAir,
            water: this.calibration.thresholdWater,
            thresh: this.calibration.thresholdCurrent,
            lockout: this.state.surfaceLockoutRemaining
        };
    }
}

// === SIMULATION ===
console.log('\n╔══════════════════════════════════════════════════════════════════════╗');
console.log('║           DIAGNOSTIC SWS ANALOG - BIOFOULING SCENARIO               ║');
console.log('╚══════════════════════════════════════════════════════════════════════╝\n');

const sim = new SWSSimulator();
const noise = 50;

function generateADC(base) {
    return Math.round(base + (Math.random() - 0.5) * 2 * noise);
}

// Scenario: 6 months biofouling
// - Surface drying is slow (exponential decay)
// - Residual conductivity ~400-800 ADC
// - True seawater ~2400

let results = [];
const phases = [
    { name: 'INITIAL_SURFACE', duration: 20, getAdc: () => generateADC(200) },
    { name: 'DIVE_1', duration: 40, getAdc: () => generateADC(2400) },
    { name: 'SURFACE_CLEAN', duration: 30, getAdc: () => generateADC(200) },
    { name: 'DIVE_2', duration: 40, getAdc: () => generateADC(2400) },
    { name: 'BIOFOULING_SURFACE', duration: 60, getAdc: (t) => {
        // Slow drying: starts at 1800, decays to 600
        const decay = Math.pow(t / 60, 0.3);  // Very slow decay
        return generateADC(1800 - (1800 - 600) * decay);
    }},
    { name: 'DIVE_3_BIOFOULED', duration: 40, getAdc: () => generateADC(2400) },
    { name: 'BIOFOULING_SURFACE_2', duration: 80, getAdc: (t) => {
        // Even slower drying due to more buildup
        const decay = Math.pow(t / 80, 0.2);
        return generateADC(2000 - (2000 - 800) * decay);
    }},
    { name: 'STUCK_UNDERWATER', duration: 150, getAdc: (t) => {
        // Simulates being at surface but detected as underwater
        // ADC slowly drops from 1500 to 900
        const decay = Math.pow(t / 150, 0.25);
        return generateADC(1500 - (1500 - 900) * decay);
    }}
];

let t = 0;
let phaseStart = 0;

console.log('Phase                  | t    | ADC  | Flt  | State | Reason              | TrendCnt | Drop% | Var    | Thr    | Air  | Water');
console.log('-'.repeat(130));

for (const phase of phases) {
    for (let i = 0; i < phase.duration; i++) {
        const adc = phase.getAdc(i);
        const result = sim.detect(adc);

        // Log every 5 seconds or on state change
        if (i % 5 === 0 || result.reason.includes('OVERRIDE') || result.reason.includes('MAX_DIVE') ||
            (i > 0 && results.length > 0 && result.state !== results[results.length - 1].state)) {
            const stateStr = result.state === 0 ? 'SURFACE' : 'UNDERWTR';
            console.log(
                `${phase.name.padEnd(22)} | ${String(t).padStart(4)} | ${String(adc).padStart(4)} | ${String(result.filtered).padStart(4)} | ${stateStr} | ` +
                `${(result.reason || '-').padEnd(19)} | ${String(result.trendCount).padStart(8)} | ${String(result.totalDrop).padStart(5)} | ${String(result.variance).padStart(6)} | ${String(result.thresh).padStart(6)} | ${String(result.air).padStart(4)} | ${String(result.water).padStart(5)}`
            );
        }

        results.push({ t, adc, ...result, phase: phase.name });
        t++;
    }
    phaseStart = t;
}

// Analysis
console.log('\n\n╔══════════════════════════════════════════════════════════════════════╗');
console.log('║                           ANALYSIS                                   ║');
console.log('╚══════════════════════════════════════════════════════════════════════╝\n');

// Count state transitions
const transitions = results.filter((r, i) => i > 0 && r.state !== results[i-1].state);
console.log(`Total transitions: ${transitions.length}`);

transitions.forEach(tr => {
    console.log(`  t=${tr.t}: ${tr.state === 0 ? 'UNDERWATER→SURFACE' : 'SURFACE→UNDERWATER'} (${tr.reason}) ADC=${tr.filtered} Thresh=${tr.thresh}`);
});

// Check biofouling detection
const biofoulingOverrides = results.filter(r => r.reason && r.reason.includes('BIOFOULING'));
console.log(`\nBiofouling overrides triggered: ${biofoulingOverrides.length}`);

// Check max dive time triggers
const maxDiveTriggers = results.filter(r => r.reason && r.reason.includes('MAX_DIVE'));
console.log(`Max dive time triggers: ${maxDiveTriggers.length}`);

// Stuck underwater analysis
const stuckPhase = results.filter(r => r.phase === 'STUCK_UNDERWATER');
const stuckUnderwater = stuckPhase.filter(r => r.state === 1).length;
const stuckSurface = stuckPhase.filter(r => r.state === 0).length;
console.log(`\nSTUCK_UNDERWATER phase: ${stuckUnderwater}s underwater, ${stuckSurface}s surface`);

// Trend detection analysis
const maxTrendCount = Math.max(...results.map(r => r.trendCount));
const maxVariance = Math.max(...results.map(r => r.variance));
const maxDrop = Math.max(...results.map(r => r.totalDrop));
console.log(`\nMax trend count reached: ${maxTrendCount} (need ≥4 for trigger)`);
console.log(`Max variance reached: ${maxVariance} (need ≥10000 for trigger)`);
console.log(`Max total drop reached: ${maxDrop}% (need ≥15% for trigger)`);

// Problem identification
console.log('\n\n╔══════════════════════════════════════════════════════════════════════╗');
console.log('║                    PROBLEM IDENTIFICATION                            ║');
console.log('╚══════════════════════════════════════════════════════════════════════╝\n');

if (maxTrendCount < 4) {
    console.log('⚠️  TREND COUNT never reaches threshold (4)');
    console.log('   → Cause: ADC not decreasing consistently enough');
    console.log('   → Solution: Lower TREND_CONSECUTIVE_MIN or use cumulative counting');
}

if (maxVariance < 10000) {
    console.log('⚠️  VARIANCE never reaches threshold (10000)');
    console.log('   → Cause: ADC values too stable during biofouling surface');
    console.log('   → Solution: Lower VARIANCE_THRESHOLD or use different metric');
}

if (maxDrop < 15) {
    console.log('⚠️  TOTAL DROP never reaches threshold (15%)');
    console.log('   → Cause: ADC values in trend buffer don\'t have enough spread');
    console.log('   → Solution: Lower TREND_TOTAL_DROP or track drop from peak differently');
}

if (stuckUnderwater > stuckSurface) {
    console.log('⚠️  STUCK_UNDERWATER: Device thinks it\'s underwater when at surface');
    console.log(`   → ${stuckUnderwater}s misdetected as underwater`);
}

console.log('\n');
