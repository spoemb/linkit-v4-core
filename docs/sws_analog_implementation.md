# Documentation SWS Analog - Détection de Surface avec Threshold Dynamique

## 📋 Table des Matières

1. [Vue d'ensemble](#vue-densemble)
2. [Problématique résolue](#problématique-résolue)
3. [Architecture technique](#architecture-technique)
4. [Algorithme de détection](#algorithme-de-détection)
5. [Configuration matérielle](#configuration-matérielle)
6. [Paramètres de configuration](#paramètres-de-configuration)
7. [Utilisation](#utilisation)
8. [Tests unitaires](#tests-unitaires)
9. [Migration depuis l'ancienne version](#migration-depuis-lancienne-version)
10. [Validation et tuning](#validation-et-tuning)

---

## 📖 Vue d'ensemble

### ⚠️ Notes Importantes

**Résolution ADC** : Cette implémentation utilise une résolution ADC de **14-bit (0-16383)** sur les deux variantes de boards :
- ✅ **Gentracker V1.0** : 14-bit (board principale pour Linkit)
- ✅ **Horizon V4.0** : 14-bit

Les deux boards utilisent le même nRF52840 et bénéficient donc de la même précision maximale. Cela garantit un comportement uniforme et une meilleure précision de détection.

**Différences entre boards** :
| Caractéristique | Gentracker V1.0 | Horizon V4.0 |
|----------------|-----------------|--------------|
| SWS_RECEIVER (ADC) | P0.02 (AIN0) | P0.02 (AIN0) |
| SWS_SENDER (GPIO) | P0.10 | P0.12 |
| Résolution ADC | 14-bit | 14-bit |
| Configuration ADC | Identique | Identique |

### Contexte

Le tracker Linkit V4 est utilisé pour le suivi long terme de tortues et autres espèces marines. La détection fiable de la position (surface vs sous l'eau) est **critique** pour :
- Optimiser la consommation énergétique (désactiver GNSS sous l'eau)
- Déclencher les transmissions Argos uniquement en surface
- Logger correctement les comportements de plongée

### Solution Implémentée

Le nouveau service **SWSAnalogService** remplace la détection digitale (ON/OFF) par une **détection analogique continue** avec :
- ✅ **Auto-calibration** : Le système apprend automatiquement les niveaux "air" et "eau"
- ✅ **Threshold dynamique** : S'adapte aux changements de salinité dans le temps
- ✅ **Hystérésis** : Évite les oscillations et faux positifs
- ✅ **Sécurités intégrées** : Timeouts et validation des mesures
- ✅ **Persistance** : Calibration sauvegardée en RAM (survit aux resets)

---

## 🎯 Problématique Résolue

### Ancien Système (Digital)

**Problème** : Détection binaire (pin HIGH/LOW) via conductivité de l'eau
```
Électrode 1 ----[eau salée]---- Électrode 2
         ↓                              ↓
     SWS_SENDER                    SWS_RECEIVER
```

**Limitations identifiées** :
1. ❌ **Dérive après 24h** : Dépôt de sel sur les électrodes → conductivité change
2. ❌ **Seuil fixe** : Ne s'adapte pas aux changements de salinité (0.5% → 3.5%)
3. ❌ **Faux positifs** : Éclaboussures, vagues, pluie peuvent déclencher
4. ❌ **Pas de validation** : Aucune vérification de cohérence temporelle

### Nouveau Système (Analog)

**Principe** : Mesure ADC analogique de la conductivité avec threshold adaptatif

```
┌─────────────────────────────────────┐
│  Auto-Calibration                   │
│  ─────────────────                  │
│  Air (sec)      : ADC = 200         │
│  Eau (salée)    : ADC = 2500        │
│  Threshold      : 200 + 40%(2500-200) = 1120 │
│  Hystérésis ±10%: [1008 - 1232]     │
└─────────────────────────────────────┘
          ↓
┌─────────────────────────────────────┐
│  Détection Continue                 │
│  ─────────────────                  │
│  Lecture ADC → Filtrage → Décision  │
│  avec adaptation continue du seuil  │
└─────────────────────────────────────┘
```

**Avantages** :
1. ✅ **Robuste au drift** : Recalibration automatique continue
2. ✅ **Adaptatif** : Apprend les nouveaux niveaux de salinité
3. ✅ **Filtré** : Moyenne mobile + confirmation temporelle
4. ✅ **Sécurisé** : Timeouts et validation des plages

---

## 🏗️ Architecture Technique

### Structure des Classes

```
UWDetectorService (classe de base)
        ↑
        │
        │ hérite
        │
SWSAnalogService
  │
  ├─ Calibration automatique
  ├─ Lecture ADC + filtrage
  ├─ Threshold dynamique + hystérésis
  ├─ Sécurités (timeouts)
  └─ Persistance (noinit RAM)
```

### Fichiers du Projet

| Fichier | Description | Lignes |
|---------|-------------|--------|
| `core/services/sws_analog_service.hpp` | Header avec documentation | 130 |
| `core/services/sws_analog_service.cpp` | Implémentation de l'algorithme | 360 |
| `tests/src/sws_analog_test.cpp` | 7 tests unitaires complets | 450 |

### Modifications Existantes

| Fichier | Modification | Impact |
|---------|--------------|--------|
| `core/protocol/base_types.hpp` | +6 nouveaux ParamID | Configuration |
| `core/protocol/dte_params.cpp` | Définitions UNP20-UNP25 | DTE protocol |
| `core/configuration/config_store.hpp` | Valeurs par défaut | Init config |
| `ports/nrf52840/bsp/horizon_v4.0/bsp.hpp` | +ADC_CHANNEL_1 | Hardware |
| `ports/nrf52840/bsp/horizon_v4.0/bsp.cpp` | Config ADC AIN0 (14-bit) | ADC setup |
| `ports/nrf52840/bsp/gentracker_v1.0/bsp.hpp` | +ADC_CHANNEL_1 | Hardware |
| `ports/nrf52840/bsp/gentracker_v1.0/bsp.cpp` | Config ADC AIN0 (14-bit) | ADC setup |
| `tests/fakes/bsp.hpp` | Support 2 canaux ADC | Tests |

---

## 🔬 Algorithme de Détection

### Phase 1 : Calibration Initiale (Air)

Au démarrage, le service effectue une calibration baseline :

```cpp
void calibrate_air_baseline() {
    // Prendre 10 échantillons en air sec
    for (int i = 0; i < 10; i++) {
        uint16_t value = read_analog_sws();
        sum += value;
        delay(100ms);
    }

    threshold_air = sum / 10;  // Moyenne

    // Initialiser threshold_water à 3× threshold_air (estimation)
    threshold_water = threshold_air × 3;

    // Calculer le threshold actif
    update_dynamic_threshold();

    // Sauvegarder en noinit RAM avec CRC
    save_calibration_with_crc();
}
```

**Exemple** :
```
Échantillons air : [195, 203, 198, 201, 200, 197, 202, 199, 204, 201]
→ threshold_air = 200
→ threshold_water = 600 (estimation initiale)
→ threshold_current = 200 + 40% × (600-200) = 360
```

### Phase 2 : Détection avec Threshold Dynamique

```cpp
bool detector_state() {
    // 1. Lecture ADC
    uint16_t raw = read_analog_sws();

    // 2. Validation
    if (raw < 50 || raw > 16300) {
        return previous_state;  // Valeur invalide, garder état
    }

    // 3. Filtrage (moyenne mobile sur 5 échantillons)
    uint16_t filtered = moving_average(raw);

    // 4. Calcul des seuils avec hystérésis
    uint16_t threshold_high = threshold_current + hysteresis;
    uint16_t threshold_low = threshold_current - hysteresis;

    // 5. Décision avec hystérésis
    if (filtered > threshold_high) {
        new_state = UNDERWATER;
        calibrate_water_baseline(filtered);  // Mise à jour continue
    } else if (filtered < threshold_low) {
        new_state = SURFACE;
    } else {
        new_state = previous_state;  // Zone d'hystérésis
    }

    // 6. Vérification sécurités
    if (check_safety_timeouts(new_state)) {
        new_state = SURFACE;  // Force surface si timeout
    }

    return new_state;
}
```

### Phase 3 : Adaptation Continue (Eau)

Lorsque l'animal plonge, le système met à jour progressivement le threshold eau :

```cpp
void calibrate_water_baseline(uint16_t value) {
    // Moyenne mobile exponentielle (α = 0.1)
    const float ALPHA = 0.1f;

    // Seulement si valeur significativement > air
    if (value > threshold_air × 1.5) {
        threshold_water = ALPHA × value + (1-ALPHA) × threshold_water;
        update_dynamic_threshold();
        save_calibration_with_crc();
    }
}
```

**Exemple d'adaptation** :
```
Première plongée (eau douce, 0.5% salinité) :
  ADC = 1500 → threshold_water converge vers 1500

Deuxième plongée (eau salée, 3.5% salinité) :
  ADC = 2800 → threshold_water converge vers 2800

Le système détecte correctement les deux cas !
```

### Diagramme d'État

```
┌──────────────┐
│   INIT       │
│ (Calibration)│
└──────┬───────┘
       │
       ↓
┌──────────────┐    ADC < threshold_low    ┌──────────────┐
│   SURFACE    │◄──────────────────────────┤  UNDERWATER  │
│ (ADC faible) │                           │ (ADC élevé)  │
└──────┬───────┘                           └──────┬───────┘
       │                                          │
       └─────────► ADC > threshold_high ◄─────────┘

       Zone d'hystérésis : Maintien état précédent
       [threshold_low ← threshold_current → threshold_high]
```

---

## ⚙️ Configuration Matérielle

### Connexions Hardware

Le système SWS analog est supporté sur les deux variantes de boards du Linkit V4, toutes deux basées sur le nRF52840.

**Linkit V4 - Gentracker V1.0** (board principale pour Linkit) :

| Signal | Pin nRF52840 | ADC | Description |
|--------|-------------|-----|-------------|
| **SWS_RECEIVER** | P0.02 | AIN0 | Mesure analogique de conductivité |
| **SWS_SENDER** | P0.10 | - | Génération du signal (GPIO output) |

**Linkit V4 - Horizon V4.0** :

| Signal | Pin nRF52840 | ADC | Description |
|--------|-------------|-----|-------------|
| **SWS_RECEIVER** | P0.02 | AIN0 | Mesure analogique de conductivité |
| **SWS_SENDER** | P0.12 | - | Génération du signal (GPIO output) |

**Configuration ADC (identique sur les deux boards)** :

```cpp
// Dans bsp.hpp (gentracker_v1.0 et horizon_v4.0)
#define SWS_ADC        BSP::ADC::ADC_CHANNEL_1

// Dans bsp.cpp - Configuration globale
.config = {
    .resolution = NRF_SAADC_RESOLUTION_14BIT,  // 14-bit pour les deux boards
    .oversample = NRF_SAADC_OVERSAMPLE_DISABLED,
    .interrupt_priority = INTERRUPT_PRIORITY_ADC,
    .low_power_mode = false
}

// Dans bsp.cpp - ADC_CHANNEL_1 configuration (identique pour les deux boards)
{
    .resistor_p = NRF_SAADC_RESISTOR_DISABLED,
    .resistor_n = NRF_SAADC_RESISTOR_DISABLED,
    .gain = NRF_SAADC_GAIN1_4,           // Gain 1/4
    .reference = NRF_SAADC_REFERENCE_INTERNAL,  // 0.6V
    .acq_time = NRF_SAADC_ACQTIME_10US,  // 10µs pour stabilisation
    .mode = NRF_SAADC_MODE_SINGLE_ENDED,
    .burst = NRF_SAADC_BURST_DISABLED,
    .pin_p = NRF_SAADC_INPUT_AIN0,       // P0.02
    .pin_n = NRF_SAADC_INPUT_DISABLED
}
```

**Plage de mesure (14-bit sur les deux boards)** :
```
Gain 1/4 × Référence 0.6V × 4 = 2.4V max
Résolution 14-bit = 0 - 16383
→ Résolution théorique : 2400mV / 16384 = 0.146 mV/count
→ Plage dynamique : ~84 dB
```

### Schéma de Fonctionnement

```
         VDD (3.3V)
            │
            │
       ┌────┴────┐
       │  MCU    │
       │         │
       │ P0.12───┼──────┐
       │ (EN)    │      │    R1
       │         │      ├────[10kΩ]────┐
       │ P0.02───┼──┐   │               │
       │ (AIN0)  │  │   │    Électrode  │  Eau salée  Électrode
       └─────────┘  │   └───────┤├──────[conductivité]───┤├─────┐
                    │                                              │
                    └──────────────────────────────────────────────┘
                                       ADC measure
```

---

## 📊 Paramètres de Configuration

### Nouveaux Paramètres Ajoutés

| ParamID | Code DTE | Type | Min | Max | Défaut | Description |
|---------|----------|------|-----|-----|--------|-------------|
| `SWS_ANALOG_THRESHOLD_MIN` | UNP20 | UINT | 50 | 16383 | 100 | ADC minimum valide (anti-saturation) |
| `SWS_ANALOG_THRESHOLD_MAX` | UNP21 | UINT | 50 | 16383 | 3000 | ADC maximum valide (limite eau) |
| `SWS_ANALOG_HYSTERESIS` | UNP22 | UINT | 0 | 50 | 10 | Hystérésis en % (anti-oscillation) |
| `SWS_ANALOG_CALIB_INTERVAL` | UNP23 | UINT | 60 | ∞ | 3600 | Intervalle recalibration (sec) |
| `UW_MAX_DIVE_TIME` | UNP24 | UINT | 0 | ∞ | 7200 | Temps max plongée (sec, 0=désactivé) |
| `UW_MIN_SURFACE_TIME` | UNP25 | UINT | 0 | ∞ | 10 | Temps min surface (sec, anti-splash) |

**Note** : Les paramètres THRESHOLD_MIN/MAX acceptent des valeurs jusqu'à 16383 (14-bit) sur les deux boards (gentracker et horizon).

### Paramètres Existants Réutilisés

| ParamID | Utilisation | Défaut |
|---------|-------------|--------|
| `UNDERWATER_EN` | Activer/désactiver détection | true |
| `UNDERWATER_DETECT_SOURCE` | Source (SWS ou SWS_GNSS) | SWS |
| `SAMPLING_UNDER_FREQ` | Période échantillonnage sous eau (sec) | Variable |
| `SAMPLING_SURF_FREQ` | Période échantillonnage surface (sec) | Variable |
| `UW_MAX_SAMPLES` | Nombre max échantillons pour confirmation | 5 |
| `UW_MIN_DRY_SAMPLES` | Nombre min échantillons "sec" pour surface | 3 |
| `UW_SAMPLE_GAP` | Délai entre échantillons (ms) | 1000 |
| `UW_PIN_SAMPLE_DELAY` | Délai stabilisation avant lecture ADC (ms) | 10 |

### Configuration Recommandée

**Configuration par défaut (déjà dans config_store.hpp)** :
```cpp
SWS_ANALOG_THRESHOLD_MIN:   100    // Évite bruit bas
SWS_ANALOG_THRESHOLD_MAX:   3000   // Plage normale eau salée
SWS_ANALOG_HYSTERESIS:      10     // 10% = ±40 counts (si range=400)
SWS_ANALOG_CALIB_INTERVAL:  3600   // Recalibration toutes les heures
UW_MAX_DIVE_TIME:           7200   // 2h max plongée (sécurité)
UW_MIN_SURFACE_TIME:        10     // 10s min surface (anti-splash)
```

**Pour environnement spécifique** :

| Environnement | THRESHOLD_MIN | THRESHOLD_MAX | HYSTERESIS |
|---------------|---------------|---------------|------------|
| Eau douce (faible salinité) | 100 | 1500 | 15% |
| Eau salée (océan normal) | 100 | 3000 | 10% |
| Eau très salée (mer morte) | 100 | 4000 | 8% |

---

## 🚀 Utilisation

### Intégration dans le Code Principal

**Option 1 : Remplacement direct de SWSService**

Dans votre fichier principal (ex: `main.cpp` ou `gentracker.cpp`) :

```cpp
// Ancien
// #include "sws_service.hpp"
// SWSService sws_detector;

// Nouveau
#include "sws_analog_service.hpp"
SWSAnalogService sws_detector;

// Le reste du code reste identique !
sws_detector.start([](ServiceEvent &event) {
    if (event.event_type == ServiceEventType::SERVICE_LOG_UPDATED) {
        bool is_underwater = std::get<bool>(event.event_data);

        if (is_underwater) {
            // Actions en plongée
            gnss_disable();
            argos_disable();
        } else {
            // Actions en surface
            gnss_enable();
            argos_enable();
        }
    }
});
```

**Option 2 : Sélection dynamique (coexistence)**

```cpp
#include "sws_service.hpp"
#include "sws_analog_service.hpp"

UWDetectorService *sws_detector;

// Sélection selon configuration ou hardware
if (use_analog_sws) {
    sws_detector = new SWSAnalogService();
} else {
    sws_detector = new SWSService();
}

sws_detector->start(callback);
```

### Configuration du Device

**Via commandes DTE** :

```bash
# Activer détection SWS analog
AT+UNP01=1              # UNDERWATER_EN = true
AT+UNP10=0              # UNDERWATER_DETECT_SOURCE = SWS

# Configurer thresholds (optionnel, défauts OK)
AT+UNP20=100            # SWS_ANALOG_THRESHOLD_MIN
AT+UNP21=3000           # SWS_ANALOG_THRESHOLD_MAX
AT+UNP22=10             # SWS_ANALOG_HYSTERESIS (10%)

# Configurer sécurités
AT+UNP24=7200           # UW_MAX_DIVE_TIME (2h)
AT+UNP25=10             # UW_MIN_SURFACE_TIME (10s)
```

**Via code** :

```cpp
configuration_store->write_param(ParamID::UNDERWATER_EN, true);
configuration_store->write_param(ParamID::UNDERWATER_DETECT_SOURCE,
                                  BaseUnderwaterDetectSource::SWS);
configuration_store->write_param(ParamID::SWS_ANALOG_THRESHOLD_MAX, 3000U);
configuration_store->write_param(ParamID::UW_MAX_DIVE_TIME, 7200U);
```

### Logs et Monitoring

**Niveaux de logs disponibles** :

```cpp
// À l'initialisation
[INFO]  SWSAnalog: Initialized - min=100 max=3000 hyst=10% calib_int=3600s
[INFO]  SWSAnalog: Air calibration complete - air=200 water=600 thresh=360

// Pendant le fonctionnement
[TRACE] SWSAnalog: raw=2450 filtered=2380 thresh=360 hyst=40
[INFO]  SWSAnalog: State change detected - new_state=1 value=2380
[TRACE] SWSAnalog: Updating water baseline 2000 -> 2100

// Recalibration périodique
[INFO]  SWSAnalog: Periodic recalibration triggered

// Warnings/Erreurs
[WARN]  SWSAnalog: Invalid ADC reading 16350, using previous state
[WARN]  SWSAnalog: Max dive time exceeded (7300s), forcing surface detection
[ERROR] SWSAnalog: ADC conversion failed with error 3
```

---

## 🧪 Tests Unitaires

### Suite de Tests Complète

**Fichier** : `tests/src/sws_analog_test.cpp`

| # | Test | Description | Validation |
|---|------|-------------|------------|
| 1 | `InitialAirCalibration` | Calibration au démarrage | 10 échantillons moyennés |
| 2 | `SurfaceDetection` | Détection surface (ADC bas) | ADC < threshold_low |
| 3 | `UnderwaterDetection` | Détection plongée (ADC haut) | ADC > threshold_high |
| 4 | `HysteresisPreventOscillation` | Anti-oscillation | Maintien état dans zone |
| 5 | `SalinityAdaptation` | Adaptation salinité variable | Détection avec 1500 puis 2800 |
| 6 | `MaxDiveTimeSafety` | Timeout sécurité plongée | Force surface après 5s |
| 7 | `InvalidADCValuesHandling` | Gestion valeurs invalides | Rejet saturation |

### Exécution des Tests

```bash
# Compiler les tests
cd /home/fourn/linkit-v4/linkit-v4-core/tests
make

# Exécuter tous les tests SWS Analog
./build/linkit_tests -g SWSAnalog

# Exécuter un test spécifique
./build/linkit_tests -n SurfaceDetection

# Exécuter avec verbose
./build/linkit_tests -g SWSAnalog -v
```

**Sortie attendue** :

```
TEST(SWSAnalog, InitialAirCalibration) - 12 ms
TEST(SWSAnalog, SurfaceDetection) - 8 ms
TEST(SWSAnalog, UnderwaterDetection) - 9 ms
TEST(SWSAnalog, HysteresisPreventOscillation) - 15 ms
TEST(SWSAnalog, SalinityAdaptation) - 22 ms
TEST(SWSAnalog, MaxDiveTimeSafety) - 11 ms
TEST(SWSAnalog, InvalidADCValuesHandling) - 7 ms

OK (7 tests, 7 ran, 15 checks, 0 ignored, 0 filtered out, 84 ms)
```

---

## 🔄 Migration depuis l'Ancienne Version

### Étape 1 : Vérification Matérielle

✅ **Vérifier la board utilisée et les pins correspondants** :

**Pour Gentracker V1.0** (board principale Linkit) :
```cpp
// Dans bsp.hpp gentracker_v1.0
#define SWS_SAMPLE_PIN BSP::GPIO::GPIO_SWS        // P0.02 (AIN0) ✓
#define SWS_ENABLE_PIN BSP::GPIO::GPIO_SLOW_SWS_SEND  // P0.10 ✓
```

**Pour Horizon V4.0** :
```cpp
// Dans bsp.hpp horizon_v4.0
#define SWS_SAMPLE_PIN BSP::GPIO::GPIO_SWS        // P0.02 (AIN0) ✓
#define SWS_ENABLE_PIN BSP::GPIO::GPIO_SLOW_SWS_SEND  // P0.12 ✓
```

**Important** : Les deux boards utilisent la même résolution ADC 14-bit et la même configuration de canal (AIN0 avec gain 1/4).

Si GPIO_SWS n'est pas sur P0.02, il faudra :
- Soit modifier le PCB
- Soit changer `NRF_SAADC_INPUT_AIN0` dans bsp.cpp vers le bon canal ADC disponible

### Étape 2 : Compilation

**CMakeLists.txt** : Ajouter les nouveaux fichiers

```cmake
# Dans core/services/CMakeLists.txt ou équivalent
set(SOURCES
    ...
    sws_service.cpp          # Ancienne version (garder pour compatibilité)
    sws_analog_service.cpp   # Nouvelle version
    ...
)
```

**Compiler** :

```bash
cd /home/fourn/linkit-v4/linkit-v4-core
mkdir -p build
cd build
cmake ..
make -j4
```

### Étape 3 : Migration du Code

**Changement minimal** :

```cpp
// Avant
#include "sws_service.hpp"
SWSService underwater_detector;

// Après
#include "sws_analog_service.hpp"
SWSAnalogService underwater_detector;

// Reste identique (même interface)
underwater_detector.start(callback);
```

### Étape 4 : Configuration Initiale

**Sur le premier déploiement** :

```bash
# 1. Flash le nouveau firmware
nrfjprog --program build/linkit_v4.hex --chiperase

# 2. Configurer via DTE (si nécessaire)
AT+UNP10=0    # Source = SWS (pas SWS_GNSS)
AT+UNP20=100  # Threshold min
AT+UNP21=3000 # Threshold max

# 3. Laisser le device se calibrer en air pendant ~2 minutes
#    (éviter de le plonger immédiatement)
```

### Étape 5 : Validation

**Test de fumée** :

1. **En air sec** :
   - Observer logs : `[INFO] SWSAnalog: Air calibration complete - air=XXX`
   - Vérifier que ADC ~200 (dépend de votre hardware)

2. **Première immersion** :
   - Tremper dans eau salée
   - Observer transition : `[INFO] State change detected - new_state=1`
   - Vérifier ADC > 1000

3. **Retour surface** :
   - Sortir de l'eau, sécher
   - Observer transition : `[INFO] State change detected - new_state=0`

---

## 🎛️ Validation et Tuning

### Phase 1 : Mesures ADC Baseline

**Objectif** : Déterminer les valeurs ADC réelles de votre hardware

**Procédure** :

```cpp
// Code de debug à ajouter temporairement dans detector_state()
DEBUG_INFO("SWS_ADC: raw=%u filtered=%u thresh=%u air=%u water=%u",
           raw_value, filtered_value, m_calib.threshold_current,
           m_calib.threshold_air, m_calib.threshold_water);
```

**Mesures à effectuer** :

| Condition | Procédure | ADC attendu (14-bit) |
|-----------|-----------|----------------------|
| **Air sec** | Électrodes sèches, en extérieur | 100-500 |
| **Air humide** | Électrodes humides (brouillard) | 300-800 |
| **Eau douce** | Électrodes dans eau robinet | 1000-2500 |
| **Eau salée 1%** | 10g sel / 1L eau | 2500-4000 |
| **Eau salée 3.5%** | 35g sel / 1L eau (océan) | 4000-8000 |

**Note** : Les valeurs attendues sont pour une résolution ADC de 14-bit (0-16383), utilisée sur les deux boards.

**Si les valeurs diffèrent significativement** :

```cpp
// Ajuster dans config_store.hpp
SWS_ANALOG_THRESHOLD_MIN: <votre_ADC_air_min>
SWS_ANALOG_THRESHOLD_MAX: <votre_ADC_eau_max>
```

### Phase 2 : Tuning de l'Hystérésis

**Test de stabilité** :

1. Placer le device à la limite air/eau (interface)
2. Observer le nombre de transitions pendant 5 minutes
3. Ajuster `SWS_ANALOG_HYSTERESIS` :

| Transitions | Action |
|-------------|--------|
| > 10 / min | ⬆️ Augmenter hystérésis (+5%) |
| 0-2 / min | ✅ Optimal |
| 0 (bloqué) | ⬇️ Réduire hystérésis (-5%) |

### Phase 3 : Validation Long Terme

**Déploiement test (7 jours)** :

```
Jour 1-2  : Test intensif (immersions multiples)
Jour 3-5  : Observation comportement normal
Jour 6-7  : Validation recalibration automatique
```

**Métriques à surveiller** :

| Métrique | Cible | Alarme si |
|----------|-------|-----------|
| Taux détection correcte | > 99% | < 95% |
| Faux positifs (splash) | < 1/jour | > 5/jour |
| Faux négatifs (rate plongée) | 0 | > 0 |
| Drift threshold_air | < ±10% / jour | > ±20% / jour |
| Drift threshold_water | < ±15% / jour | > ±30% / jour |

### Phase 4 : Optimisation Consommation

**Mesure courant de veille** :

```cpp
// Vérifier que ADC est bien uninit entre mesures
nrfx_saadc_uninit();  // Doit être appelé après chaque lecture
```

**Tuning fréquence d'échantillonnage** :

| Scénario | SAMPLING_SURF_FREQ | SAMPLING_UNDER_FREQ |
|----------|-------------------|---------------------|
| Économie max | 60s | 300s (5min) |
| Équilibré | 30s | 120s (2min) |
| Réactivité max | 10s | 60s (1min) |

### Cas Particuliers

**Si l'animal reste longtemps en surface sans bouger** :
```cpp
// Augmenter CALIB_INTERVAL pour éviter recalibrations inutiles
SWS_ANALOG_CALIB_INTERVAL: 7200  // 2h au lieu de 1h
```

**Si l'animal fait des plongées très courtes (< 10s)** :
```cpp
// Réduire MIN_SURFACE_TIME
UW_MIN_SURFACE_TIME: 3  // 3s au lieu de 10s
```

**Si l'animal fait des plongées extrêmement longues (> 2h)** :
```cpp
// Augmenter ou désactiver MAX_DIVE_TIME
UW_MAX_DIVE_TIME: 14400  // 4h
// ou
UW_MAX_DIVE_TIME: 0      // Désactivé (attention !)
```

---

## 📈 Monitoring et Debug

### Ajout de Logs Custom

Pour debug avancé, ajouter dans `detector_state()` :

```cpp
// Après chaque lecture ADC
DEBUG_INFO("SWS: raw=%u filt=%u th=%u hyst=%u state=%u time=%llu",
           raw_value, filtered_value,
           m_calib.threshold_current, m_calib.hysteresis_value,
           new_state, m_time_in_current_state);
```

### Export des Données

Pour analyse post-déploiement :

```cpp
// Créer un log détaillé dans la flash
struct SWSLogEntry {
    uint64_t timestamp;
    uint16_t adc_raw;
    uint16_t adc_filtered;
    uint16_t threshold;
    bool state;
};

// Logger à chaque transition + toutes les 10 mesures
```

---

## 🔍 Troubleshooting

### Problème 1 : ADC toujours saturé (16383)

**Cause** : Court-circuit ou connexion défectueuse

**Solution** :
1. Vérifier continuité électrodes
2. Vérifier que SWS_SENDER est bien sur P0.12
3. Réduire le temps d'acquisition : `NRF_SAADC_ACQTIME_3US`

### Problème 2 : Pas de détection sous l'eau

**Cause** : Threshold trop élevé ou électrodes sales

**Solution** :
1. Forcer recalibration : plonger/ressortir 3× rapidement
2. Vérifier ADC sous l'eau : doit être > 1000
3. Nettoyer électrodes (alcool isopropylique)
4. Réduire `THRESHOLD_MAX` temporairement

### Problème 3 : Oscillations permanentes

**Cause** : Hystérésis trop faible ou interférence électrique

**Solution** :
1. Augmenter `SWS_ANALOG_HYSTERESIS` à 20%
2. Augmenter `UW_MIN_DRY_SAMPLES` à 5
3. Vérifier masse commune MCU/électrodes

### Problème 4 : Calibration perdue après reset

**Cause** : Section noinit RAM effacée ou CRC incorrect

**Solution** :
1. Vérifier dans linker script : section `.noinit` existe
2. Ne pas utiliser `--chiperase` lors des flash (utiliser `--sectorerase`)
3. Ou accepter recalibration à chaque boot (~ 2 min)

---

## 📚 Références

### Documentation Technique

- **nRF52840 SAADC** : `nRF5_SDK_17.0.2/components/drivers_nrf/hal/nrf_saadc.h`
- **UWDetectorService** : `core/services/uwdetector_service.hpp`
- **Configuration BSP** : `ports/nrf52840/bsp/horizon_v4.0/`

### Algorithmes Utilisés

- **Moyenne mobile exponentielle** : `α × new + (1-α) × old`
- **Hystérésis de Schmitt** : Deux seuils pour éviter oscillations
- **CRC16-CCITT** : Validation intégrité calibration

### Contact et Support

Pour questions ou problèmes :
- Créer une issue sur le repo GitHub
- Vérifier les logs avec `DEBUG_TRACE` activé
- Inclure valeurs ADC mesurées et configuration

---

## 🎉 Conclusion

Le nouveau **SWSAnalogService** offre une détection robuste et adaptative de la position surface/sous-eau, essentielle pour le tracking long terme. L'auto-calibration et le threshold dynamique garantissent une fiabilité continue même avec des changements de salinité, résolvant les limitations critiques de l'ancienne implémentation digitale.

**Prochaines étapes recommandées** :
1. ✅ Compiler et tester en lab
2. ✅ Valider sur 3-5 devices pendant 1 semaine
3. ✅ Analyser les données, ajuster si nécessaire
4. ✅ Déploiement production sur toute la flotte

---

*Document créé le 2026-01-07*
*Dernière mise à jour : 2026-01-07*
*Version du firmware : Linkit V4*
