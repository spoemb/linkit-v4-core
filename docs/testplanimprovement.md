# Plan d'Amelioration des Tests - LinkIt V4 Core

## Etat Actuel

- **402 tests** au total, 36 groupes de tests
- **37 echecs** identifies (en cours de correction par les agents)
- Framework: CppUTest avec MockSupport
- Infrastructure: 17+ mocks, 13+ fakes, 20+ stubs NRF SDK

---

## Phase 1: Corriger les Tests Existants (Priorite P0)

### 1.1 Corriger les 37 echecs actuels

| Categorie | Nb | Cause | Status |
|---|---|---|---|
| Sensor median tests | 8 | Median fix (correct: moyenne 2 valeurs centrales) | Agent a01cc94 - termine |
| M8/GNSS tests | 14 | Mock `acquire_sensors_pwr` manquant | Agent afcc406 - termine |
| DTEHandler tests | 7 | Valeurs par defaut changees (DBP01, UNP12) | Agent a311485 - termine |
| ArgosTx, NrfSwitch, Encoder, Decoder | 8 | Divers (voir details) | Agent a973263 - termine |

**Action**: Rebuilder les tests et verifier que les 37 echecs sont corriges.

```bash
cd tests/build
cmake -DCMAKE_BUILD_TYPE=Debug -DCMAKE_C_COMPILER=/usr/bin/gcc -DCMAKE_CXX_COMPILER=/usr/bin/g++ ..
make -j$(nproc)
./CLSGenTrackerTests -v
```

---

## Phase 2: Tests Manquants Critiques (Priorite P1)

### 2.1 Service Argos RX (Downlink)

**Fichier**: `core/services/argos_rx_service.hpp` (69 lignes)
**Nouveau test**: `tests/src/argos_rx_test.cpp`
**Mock necessaire**: Mock KIM2/Kineis pour reception satellite

Tests a ecrire:
- [ ] Reception downlink basique
- [ ] Timeout de reception
- [ ] Traitement des commandes recues
- [ ] Integration avec config_store (mise a jour params)
- [ ] Mode duty-cycle activation/desactivation

### 2.2 Service Camera

**Fichier**: `core/services/cam_service.hpp` (75 lignes)
**Nouveau test**: `tests/src/cam_test.cpp`

Tests a ecrire:
- [ ] Activation/desactivation camera
- [ ] Periode ON/OFF (CAM_PERIOD_ON, CAM_PERIOD_OFF)
- [ ] Integration avec low battery (LB_CAM_EN)
- [ ] Scheduling correct

### 2.3 Service Thermistor

**Fichier**: `core/services/thermistor_sensor_service.hpp` (107 lignes)
**Nouveau test**: `tests/src/thermistor_test.cpp`
**Mock necessaire**: `mock_thermistor.hpp`

Tests a ecrire:
- [ ] Lecture temperature basique
- [ ] Calcul median avec nouveau algorithme
- [ ] Mode TX enable/disable
- [ ] Gestion max_samples

### 2.4 Driver SMD Satellite

**Fichier**: `ports/nrf52840/core/hardware/smd_sat/smd_sat.cpp` (~900 lignes)
**Nouveau test**: `tests/src/smd_sat_test.cpp`
**Mock necessaire**: Mock SPI (nrf_spim), mock GPIOs

Tests a ecrire:
- [ ] State machine: stopped -> idle -> tx -> stopped
- [ ] Envoi de paquet A2/A4
- [ ] Timeout de transmission
- [ ] DFU mode enter/exit
- [ ] Gestion credentials (SECKEY, RADIOCONF)
- [ ] Power on/off sequence
- [ ] SPI communication protocol A+

---

## Phase 3: Couverture des Utilitaires (Priorite P2)

### 3.1 `binascii.hpp`

Tests a ecrire:
- [ ] hexlify (conversion bytes -> hex string)
- [ ] unhexlify (conversion hex string -> bytes)
- [ ] Cas limites (chaine vide, longueur impaire)

### 3.2 `bitpack.hpp`

Tests a ecrire:
- [ ] Pack/unpack uint8, uint16, uint32
- [ ] Pack avec offset de bits
- [ ] Depassement de buffer
- [ ] Alignement des bits

### 3.3 `timeutils.hpp`

Tests a ecrire:
- [ ] convert_epochtime() avec dates valides
- [ ] convert_epochtime() avec dates limites (1970, 2100)
- [ ] Conversion de fuseau horaire
- [ ] MS_PER_SEC, SECONDS_PER_HOUR constants

### 3.4 `haversine.hpp`

Tests a ecrire:
- [ ] Distance entre deux points connus
- [ ] Distance zero (meme point)
- [ ] Antipodes (distance maximale)
- [ ] Points sur l'equateur et les poles

---

## Phase 4: Couverture des Drivers Hardware (Priorite P2)

### 4.1 LPS28DFW (Capteur pression)

**Mock necessaire**: Mock I2C (NrfI2C)

Tests a ecrire:
- [ ] Initialisation et WHO_AM_I
- [ ] Lecture temperature et pression
- [ ] Echec de communication I2C
- [ ] Verification de l'initialisation (`is_initialized()`)

### 4.2 BMA400 (Accelerometre)

**Mock necessaire**: Mock I2C

Tests a ecrire:
- [ ] Initialisation et configuration
- [ ] Lecture donnees d'acceleration
- [ ] Detection de mouvement
- [ ] Mode power management

### 4.3 LTR-303 (Capteur lumiere ALS)

Tests a ecrire:
- [ ] Initialisation
- [ ] Lecture luminosite
- [ ] Configuration du gain

### 4.4 MS58xx (Capteur pression/profondeur)

Tests a ecrire:
- [ ] Calibration PROM
- [ ] Conversion D1/D2
- [ ] Calcul temperature compensee
- [ ] Calcul pression compensee

---

## Phase 5: Tests Systeme et Integration (Priorite P3)

### 5.1 OTA Firmware Update

**Fichier**: `core/filesystem/ota_flash_file_updater.cpp`

Tests a ecrire:
- [ ] Verification CRC du firmware
- [ ] Ecriture flash par blocs
- [ ] Reprise apres interruption
- [ ] Validation de version

### 5.2 Service UW Detector

Tests a ecrire:
- [ ] Detection surface/immersion
- [ ] Hysteresis de detection
- [ ] Integration avec scheduling Argos TX
- [ ] Dry time / wet time tracking

### 5.3 Service Scheduler (abstrait)

Tests a ecrire:
- [ ] Cycle start/stop/reschedule
- [ ] Notification underwater state
- [ ] Notification sensor log update
- [ ] Priorite des services

### 5.4 Protocol DTE complet

Tests a ecrire:
- [ ] Encodage/decodage allcast packets
- [ ] FCS (Frame Check Sequence) validation
- [ ] Base64 encode/decode
- [ ] Gestion des erreurs de protocole
- [ ] Commandes DTE manquantes (DUMPD paginee, FACTW, RSTBW)

---

## Phase 6: Robustesse et Cas Limites (Priorite P3)

### 6.1 Config Store Edge Cases

- [ ] Corruption de fichier de configuration
- [ ] Migration de version de configuration
- [ ] Valeurs limites pour chaque parametre
- [ ] Ecriture concurrente (ISR vs main thread)

### 6.2 GPS Scheduler Edge Cases

- [ ] Overflow du timer de schedule (> 4294 secondes)
- [ ] Underflow m_num_consecutive_fixes
- [ ] Passage de minuit dans le scheduling
- [ ] Mode COLD_START vs normal

### 6.3 Argos TX Edge Cases

- [ ] Depth pile avec exactement 24 entrees
- [ ] Zone exclusion activation/desactivation
- [ ] Low battery mode transitions
- [ ] Frequence hors bande

---

## Infrastructure Necessaire

### Nouveaux Mocks a Creer

| Mock | Pour tester | Complexite |
|---|---|---|
| `mock_spi.hpp` | SMD Sat, KIM2 | Moyenne |
| `mock_i2c.hpp` | LPS28, BMA400, LTR303, AD5933 | Moyenne |
| `mock_thermistor.hpp` | Thermistor service | Simple |
| `mock_cam.hpp` | Camera service | Simple |
| `mock_argos_rx.hpp` | Argos RX service | Moyenne |
| `mock_ota.hpp` | OTA update | Moyenne |

### Modifications CMakeLists.txt

Chaque nouveau fichier test doit etre ajoute dans `tests/CMakeLists.txt`:
- Ajouter le `.cpp` source dans `target_sources(CLSGenTrackerTests ...)`
- Ajouter les mocks/fakes correspondants
- Ajouter les includes necessaires

---

## Estimation de l'Effort

| Phase | Tests | Effort | Priorite |
|---|---|---|---|
| Phase 1: Corrections | ~37 fixes | 1 session | P0 |
| Phase 2: Services critiques | ~25 tests | 2-3 sessions | P1 |
| Phase 3: Utilitaires | ~15 tests | 1 session | P2 |
| Phase 4: Drivers | ~20 tests | 2 sessions | P2 |
| Phase 5: Integration | ~20 tests | 2-3 sessions | P3 |
| Phase 6: Edge cases | ~15 tests | 1-2 sessions | P3 |
| **Total** | **~115 nouveaux tests** | **~10 sessions** | - |

---

## Commande de Verification

Apres chaque phase, verifier:

```bash
# Build RSPB firmware
cd ports/nrf52840/build_rspb_debug
export PATH=/home/schade/tools/gcc-arm-none-eabi-10.3-2021.10/bin:$PATH
cmake -DCMAKE_TOOLCHAIN_FILE=../toolchain_arm_gcc_nrf52.cmake \
      -DDEBUG_LEVEL=4 -DBOARD=RSPB -DCMAKE_BUILD_TYPE=Debug \
      -DMODEL=CORE -DARGOS_SMD=ON -DENABLE_PRESSURE_SENSOR=ON \
      -DENABLE_AXL_SENSOR=ON -DGPS_FAKE_POSITION=1 ..
make -j$(nproc)

# Build et run tests
cd tests/build
cmake -DCMAKE_BUILD_TYPE=Debug -DCMAKE_C_COMPILER=/usr/bin/gcc \
      -DCMAKE_CXX_COMPILER=/usr/bin/g++ ..
make -j$(nproc)
./CLSGenTrackerTests -v
```

**Objectif final**: 0 echecs, couverture > 500 tests, tous les services critiques couverts.
