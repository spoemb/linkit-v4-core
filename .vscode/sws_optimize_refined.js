#!/usr/bin/env node
/**
 * SWS Analog Parameter Optimizer - Refined Search
 * Recherche affinée autour des meilleurs paramètres trouvés
 */

// ============================================
// SIMULATOR (same as before)
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
            movingAvgBuffer: []
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
        if (value > this.calibration.thresholdAir * 1.5) {
            this.calibration.thresholdWater =
                this.params.alpha * value + (1 - this.params.alpha) * this.calibration.thresholdWater;
            this.updateThreshold();
        }
    }

    detect(adcValue, deltaTime) {
        this.state.timeInState += deltaTime;
        if (adcValue < 50 || adcValue > 16300) return this.state.current;

        const filtered = this.movingAverage(adcValue);
        const threshHigh = this.calibration.thresholdCurrent + this.calibration.hysteresisValue;
        const threshLow = this.calibration.thresholdCurrent - this.calibration.hysteresisValue;

        let newState = this.state.current;

        if (filtered > threshHigh) {
            this.state.consecutiveSamples++;
            if (this.state.consecutiveSamples >= this.params.maxSamples) {
                newState = 1;
                this.updateWaterBaseline(filtered);
            }
        } else if (filtered < threshLow) {
            if (this.state.current === 1) {
                this.state.consecutiveSamples++;
                if (this.state.consecutiveSamples >= this.params.minDrySamples) newState = 0;
            } else {
                newState = 0;
                this.state.consecutiveSamples = 0;
            }
        } else {
            this.state.consecutiveSamples = 0;
        }

        if (newState === 1 && this.state.timeInState > this.params.maxDiveTime) newState = 0;
        if (this.state.current === 0 && this.state.timeInState < this.params.minSurfaceTime && newState === 1) newState = 0;

        if (newState !== this.state.current) {
            this.state.previous = this.state.current;
            this.state.current = newState;
            this.state.timeInState = 0;
            this.state.consecutiveSamples = 0;
            return { changed: true, state: newState };
        }
        return { changed: false, state: this.state.current };
    }
}

class ScenarioRunner {
    generateADC(baseValue, noise = 20) {
        return Math.max(0, Math.min(16383, baseValue + (Math.random() - 0.5) * 2 * noise));
    }

    normalDive(time, ctx) {
        const cycle = time % 120;
        if (cycle < 30) return { adc: this.generateADC(200, 30), expectedState: 0 };
        if (cycle < 90) return { adc: this.generateADC(2500, 100), expectedState: 1 };
        return { adc: this.generateADC(200, 30), expectedState: 0 };
    }

    splashTest(time, ctx) {
        if (Math.random() < 0.08) return { adc: this.generateADC(1200, 200), expectedState: 0, isSplash: true };
        return { adc: this.generateADC(180, 25), expectedState: 0 };
    }

    salinityChange(time, ctx) {
        const phase = Math.floor(time / 90) % 5;
        const levels = [{ adc: 200, state: 0 }, { adc: 1200, state: 1 }, { adc: 2500, state: 1 }, { adc: 4000, state: 1 }, { adc: 200, state: 0 }];
        const level = levels[phase];
        return { adc: this.generateADC(level.adc, level.adc * 0.05), expectedState: level.state };
    }

    electrodeDrift(time, ctx) {
        const driftFactor = Math.max(0.5, 1 - (time / 1200) * 0.3);
        const cycle = time % 80;
        if (cycle < 30) return { adc: this.generateADC(200 / driftFactor, 40), expectedState: 0 };
        return { adc: this.generateADC(2500 * driftFactor, 150), expectedState: 1 };
    }

    rapidTransitions(time, ctx) {
        const cycle = time % 20;
        if (cycle < 8) return { adc: this.generateADC(180, 20), expectedState: 0 };
        if (cycle < 16) return { adc: this.generateADC(2800, 80), expectedState: 1 };
        return { adc: this.generateADC(180, 20), expectedState: 0 };
    }

    longDive(time, ctx) {
        if (time < 30) return { adc: this.generateADC(200, 30), expectedState: 0 };
        const variation = Math.sin(time / 60) * 200;
        return { adc: this.generateADC(2600 + variation, 100), expectedState: 1 };
    }

    highNoise(time, ctx) {
        const cycle = time % 100;
        if (cycle < 40) return { adc: this.generateADC(200, 300), expectedState: 0 };
        return { adc: this.generateADC(2500, 300), expectedState: 1 };
    }

    // Additional realistic scenarios
    turtleBehavior(time, ctx) {
        // Realistic turtle diving pattern: short surfaces, long dives
        const pattern = [
            { start: 0, end: 20, state: 0, adc: 180 },      // Surface breathing
            { start: 20, end: 180, state: 1, adc: 2800 },   // Long dive
            { start: 180, end: 195, state: 0, adc: 190 },   // Quick surface
            { start: 195, end: 350, state: 1, adc: 2600 },  // Another dive
            { start: 350, end: 380, state: 0, adc: 200 },   // Longer surface
            { start: 380, end: 500, state: 1, adc: 3000 },  // Deep dive
            { start: 500, end: 530, state: 0, adc: 185 },   // Surface
            { start: 530, end: 600, state: 1, adc: 2700 },  // Final dive
        ];

        const cycleTime = time % 600;
        for (const p of pattern) {
            if (cycleTime >= p.start && cycleTime < p.end) {
                return { adc: this.generateADC(p.adc, 50), expectedState: p.state };
            }
        }
        return { adc: this.generateADC(200, 30), expectedState: 0 };
    }

    waveInterference(time, ctx) {
        // Surface with wave interference (periodic splashes)
        const waveFreq = 0.5; // waves every 2 seconds
        const waveAmplitude = Math.sin(time * waveFreq * Math.PI) * 300;
        const baseAdc = 200 + Math.max(0, waveAmplitude);
        return { adc: this.generateADC(baseAdc, 50), expectedState: 0 };
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
            turtle: this.turtleBehavior.bind(this),
            waves: this.waveInterference.bind(this)
        };
        const scenario = scenarios[scenarioName];
        if (!scenario) return null;

        const ctx = {};
        const metrics = { correct: 0, total: 0, fp: 0, fn: 0, latencies: [], oscillations: 0 };
        let pendingTrans = null, lastExp = 0;

        for (let t = 0; t < duration; t++) {
            const { adc, expectedState, isSplash } = scenario(t, ctx);
            const result = simulator.detect(adc, 1);

            if (expectedState !== lastExp) pendingTrans = { time: t, target: expectedState };
            lastExp = expectedState;
            metrics.total++;

            if (result.state === expectedState) {
                metrics.correct++;
                if (pendingTrans && result.state === pendingTrans.target) {
                    metrics.latencies.push(t - pendingTrans.time);
                    pendingTrans = null;
                }
            } else {
                if (result.state === 1 && expectedState === 0 && !isSplash) metrics.fp++;
                else if (result.state === 0 && expectedState === 1) metrics.fn++;
            }

            if (result.changed) {
                const since = t - (ctx.lastChange || 0);
                if (since < 5) metrics.oscillations++;
                ctx.lastChange = t;
            }
        }

        return {
            detectionRate: metrics.correct / metrics.total,
            falsePositiveRate: metrics.fp / Math.max(1, metrics.total),
            falseNegativeRate: metrics.fn / Math.max(1, metrics.total),
            avgLatency: metrics.latencies.length > 0 ? metrics.latencies.reduce((a, b) => a + b, 0) / metrics.latencies.length : 0,
            stability: 1 - Math.min(1, metrics.oscillations / 20)
        };
    }
}

// ============================================
// GRID SEARCH OPTIMIZER
// ============================================

class GridSearchOptimizer {
    constructor() {
        this.runner = new ScenarioRunner();
        this.results = [];
    }

    calculateScore(scenarioResults) {
        const weights = { det: 0.35, fp: 0.25, fn: 0.20, lat: 0.10, stab: 0.10 };
        let total = 0, count = 0;

        for (const [, r] of Object.entries(scenarioResults)) {
            total += weights.det * r.detectionRate +
                     weights.fp * (1 - Math.min(1, r.falsePositiveRate * 100)) +
                     weights.fn * (1 - Math.min(1, r.falseNegativeRate * 50)) +
                     weights.lat * Math.max(0, 1 - r.avgLatency / 30) +
                     weights.stab * r.stability;
            count++;
        }
        return count > 0 ? total / count : 0;
    }

    gridSearch(config) {
        const {
            duration = 600,
            scenarios = ['normal', 'splash', 'salinity', 'drift', 'rapid', 'long', 'noise', 'turtle', 'waves'],
            grid
        } = config;

        console.log('\n╔══════════════════════════════════════════════════════════════╗');
        console.log('║       SWS ANALOG - RECHERCHE PAR GRILLE AFFINÉE              ║');
        console.log('╚══════════════════════════════════════════════════════════════╝\n');

        const totalCombinations =
            grid.hysteresis.length * grid.thresholdRatio.length * grid.alpha.length *
            grid.movingAvgSize.length * grid.maxSamples.length * grid.minDrySamples.length;

        console.log(`Combinaisons à tester: ${totalCombinations}`);
        console.log(`Scénarios: ${scenarios.length}`);
        console.log('');

        let tested = 0;
        const startTime = Date.now();

        for (const hyst of grid.hysteresis) {
            for (const ratio of grid.thresholdRatio) {
                for (const alpha of grid.alpha) {
                    for (const avgSize of grid.movingAvgSize) {
                        for (const maxSmp of grid.maxSamples) {
                            for (const drySmp of grid.minDrySamples) {
                                const params = {
                                    hysteresis: hyst,
                                    thresholdRatio: ratio,
                                    alpha: alpha,
                                    movingAvgSize: avgSize,
                                    maxSamples: maxSmp,
                                    minDrySamples: drySmp
                                };

                                const scenarioResults = {};
                                for (const scenario of scenarios) {
                                    const result = this.runner.run(scenario, duration, params);
                                    if (result) scenarioResults[scenario] = result;
                                }

                                const score = this.calculateScore(scenarioResults);
                                const avgDet = Object.values(scenarioResults).reduce((s, r) => s + r.detectionRate, 0) / scenarios.length;
                                const avgFp = Object.values(scenarioResults).reduce((s, r) => s + r.falsePositiveRate, 0) / scenarios.length;
                                const avgFn = Object.values(scenarioResults).reduce((s, r) => s + r.falseNegativeRate, 0) / scenarios.length;
                                const avgLat = Object.values(scenarioResults).reduce((s, r) => s + r.avgLatency, 0) / scenarios.length;
                                const avgStab = Object.values(scenarioResults).reduce((s, r) => s + r.stability, 0) / scenarios.length;

                                this.results.push({
                                    params,
                                    scenarioResults,
                                    score,
                                    detectionRate: avgDet,
                                    falsePositives: avgFp,
                                    falseNegatives: avgFn,
                                    avgLatency: avgLat,
                                    stability: avgStab
                                });

                                tested++;
                                if (tested % 50 === 0) {
                                    const elapsed = (Date.now() - startTime) / 1000;
                                    const eta = (elapsed / tested) * (totalCombinations - tested);
                                    process.stdout.write(`\rProgression: ${tested}/${totalCombinations} (${(tested/totalCombinations*100).toFixed(1)}%) - ETA: ${Math.round(eta)}s   `);
                                }
                            }
                        }
                    }
                }
            }
        }

        console.log(`\n\nTerminé en ${((Date.now() - startTime) / 1000).toFixed(1)}s`);
        this.results.sort((a, b) => b.score - a.score);
        return this.results;
    }

    printResults(topN = 10) {
        console.log('\n╔══════════════════════════════════════════════════════════════╗');
        console.log('║                    RÉSULTATS OPTIMAUX                        ║');
        console.log('╚══════════════════════════════════════════════════════════════╝\n');

        const top = this.results.slice(0, topN);

        console.log('┌─────┬────────┬──────────┬─────────┬─────────┬─────────┬──────────┬───────┬───────┬───────┬────────┬────────┬────────┐');
        console.log('│ #   │ Score  │ Détect.  │ Faux+   │ Faux-   │ Latence │ Stabilité│ Hyst% │ Ratio%│ Alpha │ AvgSz  │ MaxSmp │ DrySmp │');
        console.log('├─────┼────────┼──────────┼─────────┼─────────┼─────────┼──────────┼───────┼───────┼───────┼────────┼────────┼────────┤');

        top.forEach((r, i) => {
            const p = r.params;
            console.log(
                `│ ${(i + 1).toString().padStart(3)} │ ` +
                `${(r.score * 100).toFixed(1).padStart(5)}% │ ` +
                `${(r.detectionRate * 100).toFixed(1).padStart(7)}% │ ` +
                `${(r.falsePositives * 100).toFixed(2).padStart(6)}% │ ` +
                `${(r.falseNegatives * 100).toFixed(2).padStart(6)}% │ ` +
                `${r.avgLatency.toFixed(1).padStart(6)}s │ ` +
                `${(r.stability * 100).toFixed(0).padStart(7)}% │ ` +
                `${p.hysteresis.toString().padStart(5)} │ ` +
                `${p.thresholdRatio.toString().padStart(5)} │ ` +
                `${p.alpha.toFixed(2).padStart(5)} │ ` +
                `${p.movingAvgSize.toString().padStart(6)} │ ` +
                `${p.maxSamples.toString().padStart(6)} │ ` +
                `${p.minDrySamples.toString().padStart(6)} │`
            );
        });

        console.log('└─────┴────────┴──────────┴─────────┴─────────┴─────────┴──────────┴───────┴───────┴───────┴────────┴────────┴────────┘');
    }

    printBestConfig() {
        if (this.results.length === 0) return;

        const best = this.results[0];
        const p = best.params;

        console.log('\n╔══════════════════════════════════════════════════════════════════════════════╗');
        console.log('║                     PARAMÈTRES OPTIMAUX RECOMMANDÉS                          ║');
        console.log('╚══════════════════════════════════════════════════════════════════════════════╝\n');

        console.log(`Score global: ${(best.score * 100).toFixed(2)}%\n`);

        console.log('┌─────────────────────────────────────────┬───────────────────────────────────┐');
        console.log('│ Paramètre                               │ Valeur Optimale                   │');
        console.log('├─────────────────────────────────────────┼───────────────────────────────────┤');
        console.log(`│ SWS_ANALOG_HYSTERESIS                   │ ${p.hysteresis.toString().padStart(31)}% │`);
        console.log(`│ THRESHOLD_RATIO                         │ ${p.thresholdRatio.toString().padStart(31)}% │`);
        console.log(`│ ALPHA (EMA)                             │ ${p.alpha.toFixed(2).padStart(33)} │`);
        console.log(`│ MOVING_AVG_SIZE                         │ ${p.movingAvgSize.toString().padStart(33)} │`);
        console.log(`│ UW_MAX_SAMPLES                          │ ${p.maxSamples.toString().padStart(33)} │`);
        console.log(`│ UW_MIN_DRY_SAMPLES                      │ ${p.minDrySamples.toString().padStart(33)} │`);
        console.log('└─────────────────────────────────────────┴───────────────────────────────────┘');

        console.log('\nPerformance par scénario:');
        console.log('┌──────────────────────────┬───────────┬───────────┬───────────┬───────────┐');
        console.log('│ Scénario                 │ Détection │ Faux+     │ Faux-     │ Latence   │');
        console.log('├──────────────────────────┼───────────┼───────────┼───────────┼───────────┤');

        const names = {
            normal: 'Plongée normale', splash: 'Éclaboussures', salinity: 'Salinité variable',
            drift: 'Dérive électrodes', rapid: 'Transitions rapides', long: 'Plongée longue',
            noise: 'Bruit élevé', turtle: 'Comportement tortue', waves: 'Interférence vagues'
        };

        for (const [key, r] of Object.entries(best.scenarioResults)) {
            console.log(
                `│ ${(names[key] || key).padEnd(24)} │ ` +
                `${(r.detectionRate * 100).toFixed(1).padStart(7)}% │ ` +
                `${(r.falsePositiveRate * 100).toFixed(2).padStart(7)}% │ ` +
                `${(r.falseNegativeRate * 100).toFixed(2).padStart(7)}% │ ` +
                `${r.avgLatency.toFixed(1).padStart(7)}s │`
            );
        }
        console.log('└──────────────────────────┴───────────┴───────────┴───────────┴───────────┘');

        // Generate code
        console.log('\n// ═══════════════════════════════════════════════════════════════════════════');
        console.log('// CONFIGURATION À INTÉGRER DANS LE CODE');
        console.log('// ═══════════════════════════════════════════════════════════════════════════\n');

        console.log('// Dans config_store.hpp ou sws_analog_service.cpp:');
        console.log('');
        console.log(`// Paramètres optimisés par simulation Monte Carlo`);
        console.log(`// Score: ${(best.score * 100).toFixed(1)}% | Détection: ${(best.detectionRate * 100).toFixed(1)}% | Faux+: ${(best.falsePositives * 100).toFixed(2)}%`);
        console.log('');
        console.log(`static constexpr uint8_t  SWS_ANALOG_HYSTERESIS    = ${p.hysteresis};     // % hystérésis`);
        console.log(`static constexpr uint8_t  SWS_THRESHOLD_RATIO      = ${p.thresholdRatio};     // % position du seuil`);
        console.log(`static constexpr float    SWS_ALPHA_EMA            = ${p.alpha.toFixed(2)}f;  // facteur EMA`);
        console.log(`static constexpr uint8_t  SWS_MOVING_AVG_SIZE      = ${p.movingAvgSize};      // taille moyenne mobile`);
        console.log(`static constexpr uint8_t  UW_MAX_SAMPLES           = ${p.maxSamples};      // échantillons confirmation plongée`);
        console.log(`static constexpr uint8_t  UW_MIN_DRY_SAMPLES       = ${p.minDrySamples};      // échantillons confirmation surface`);
        console.log('');
        console.log('// Paramètres secondaires (garder les valeurs par défaut):');
        console.log('static constexpr uint16_t SWS_ANALOG_THRESHOLD_MIN = 100;    // ADC min valide');
        console.log('static constexpr uint16_t SWS_ANALOG_THRESHOLD_MAX = 3000;   // ADC max eau');
        console.log('static constexpr uint16_t UW_MAX_DIVE_TIME         = 7200;   // 2h timeout plongée');
        console.log('static constexpr uint8_t  UW_MIN_SURFACE_TIME      = 10;     // anti-splash');
    }

    analyzeConsensus() {
        console.log('\n╔══════════════════════════════════════════════════════════════╗');
        console.log('║              ANALYSE DE CONSENSUS (Top 50)                   ║');
        console.log('╚══════════════════════════════════════════════════════════════╝\n');

        const top50 = this.results.slice(0, 50);
        const params = ['hysteresis', 'thresholdRatio', 'alpha', 'movingAvgSize', 'maxSamples', 'minDrySamples'];

        console.log('Statistiques des 50 meilleurs résultats:\n');
        console.log('┌──────────────────────┬─────────┬─────────┬─────────┬─────────┬─────────────────────┐');
        console.log('│ Paramètre            │   Min   │   Max   │  Moyenne│  Médiane│ Valeur recommandée  │');
        console.log('├──────────────────────┼─────────┼─────────┼─────────┼─────────┼─────────────────────┤');

        params.forEach(param => {
            const values = top50.map(r => r.params[param]).sort((a, b) => a - b);
            const min = values[0];
            const max = values[values.length - 1];
            const avg = values.reduce((a, b) => a + b, 0) / values.length;
            const median = values[Math.floor(values.length / 2)];

            // Mode (most common value)
            const freq = {};
            values.forEach(v => { freq[v] = (freq[v] || 0) + 1; });
            const mode = Object.entries(freq).sort((a, b) => b[1] - a[1])[0][0];

            const isFloat = param === 'alpha';
            const fmt = (v) => isFloat ? v.toFixed(2) : v.toString();

            console.log(
                `│ ${param.padEnd(20)} │ ` +
                `${fmt(min).padStart(7)} │ ` +
                `${fmt(max).padStart(7)} │ ` +
                `${fmt(avg).padStart(7)} │ ` +
                `${fmt(median).padStart(7)} │ ` +
                `${fmt(parseFloat(mode)).padStart(19)} │`
            );
        });

        console.log('└──────────────────────┴─────────┴─────────┴─────────┴─────────┴─────────────────────┘');
    }
}

// ============================================
// MAIN - Refined Grid Search
// ============================================

console.log('SWS Analog - Recherche affinée des paramètres optimaux\n');

const optimizer = new GridSearchOptimizer();

// Grid search around best values found in first pass
const config = {
    duration: 800,  // Longer simulation for accuracy
    scenarios: ['normal', 'splash', 'salinity', 'drift', 'rapid', 'long', 'noise', 'turtle', 'waves'],
    grid: {
        // Refined ranges based on first optimization
        hysteresis: [8, 9, 10, 11, 12, 13, 14, 15],
        thresholdRatio: [40, 42, 44, 46, 48, 50, 52, 54],
        alpha: [0.08, 0.10, 0.12, 0.14, 0.16],
        movingAvgSize: [3, 4, 5],
        maxSamples: [2, 3, 4],
        minDrySamples: [3, 4, 5, 6]
    }
};

optimizer.gridSearch(config);
optimizer.printResults(20);
optimizer.printBestConfig();
optimizer.analyzeConsensus();

console.log('\n✅ Optimisation affinée terminée!\n');
