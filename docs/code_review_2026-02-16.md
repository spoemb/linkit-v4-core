# Code Review Complet - LinkIt V4 Core

**Date:** 2026-02-16
**Branche:** `clean-repo`
**Fichiers analyses:** 28 fichiers modifies + 1 nouveau (`dte_handler.cpp`)
**Scope:** Fuites memoire, linting C++, algorithmique, robustesse, dead code, redondance, optimisation

---

## Table des matieres

1. [Bugs critiques (P0)](#1-bugs-critiques-p0)
2. [Fuites memoire](#2-fuites-memoire)
3. [Variables non initialisees](#3-variables-non-initialisees)
4. [Dead code](#4-dead-code)
5. [Redundance et optimisation de lignes](#5-redundance-et-optimisation-de-lignes)
6. [Robustesse](#6-robustesse)
7. [C++ best practices](#7-c-best-practices)
8. [CMake](#8-cmake)
9. [Tests](#9-tests)
10. [Actions recommandees par priorite](#10-actions-recommandees-par-priorite)

---

## 1. Bugs critiques (P0)

### 1.1 Copy-paste bug dans les tests
- **Fichier:** `tests/src/argos_tx_test.cpp:96-102`
- **Severite:** CRITICAL
- **Description:** `inject_gps_inactive()` envoie `ServiceEventType::SERVICE_ACTIVE` au lieu de `SERVICE_INACTIVE`. Corps identique a `inject_gps_active()`. Tous les appels a `inject_gps_inactive()` (lignes 1623, 1628, 1633, 1638) injectent le mauvais evenement.
- **Fix:** Changer ligne 98 : `e.event_type = ServiceEventType::SERVICE_INACTIVE;`

### 1.2 Bug de precedence d'operateur (UBX UART send)
- **Fichier:** `ports/nrf52840/core/hardware/m10qasync/ubx_comms.cpp:104-105`
- **Severite:** CRITICAL
- **Description:** `if ((ret = nrf_libuarte_async_tx(...) != NRF_SUCCESS))` evalue comme `ret = (func() != NRF_SUCCESS)`, stockant un bool (0/1) au lieu du code erreur reel. Le log a ligne 107 affiche une valeur incorrecte.
- **Fix:** `if ((ret = nrf_libuarte_async_tx(...)) != NRF_SUCCESS)`

### 1.3 Buffer overflow dans ubx_comms send()
- **Fichier:** `ports/nrf52840/core/hardware/m10qasync/ubx_comms.cpp:96`
- **Severite:** CRITICAL
- **Description:** `std::memcpy(m_tx_buffer, buffer, sz)` copie `sz` octets dans `m_tx_buffer[256]` sans verifier que `sz <= 256`.
- **Fix:** Ajouter `if (sz > sizeof(m_tx_buffer)) { DEBUG_ERROR(...); return; }` avant le memcpy.

### 1.4 Buffer overflow dans run_nav_filter()
- **Fichier:** `ports/nrf52840/core/hardware/m10qasync/ubx_comms.cpp:321-332`
- **Severite:** CRITICAL
- **Description:** `memcpy(&m_nav_report.pvt, msg->payload, sizeof(MSG_PVT))` suppose que le payload fait au moins 92 octets. Un message UBX malformed avec un payload plus court lit hors du buffer DMA. Le cas SAT (ligne 337-338) utilise correctement `std::min`, mais PVT, DOP et STATUS non.
- **Fix:** Ajouter `if (msg->msgLength >= sizeof(m_nav_report.pvt))` avant chaque memcpy.

### 1.5 Acces arg_list[] sans bounds check (6 occurrences)
- **Fichier:** `core/protocol/dte_handler.cpp`
- **Severite:** CRITICAL
- **Lignes:** 80-84 (PARMW), 106-107 (PROFW), 157 (RSTVW), 202-203 (DUMPM), 404-410 (SCALW), 431-434 (SCALR)
- **Description:** `arg_list[0]`, `arg_list[1]`, `arg_list[2]` accedes sans verifier `.size()`. Un message DTE malformed provoque un acces hors limites -> crash (HardFault sur Cortex-M4).
- **Fix:** Ajouter `if (arg_list.size() < N) return DTEEncoder::encode(..., (int)DTEError::MISSING_ARGUMENT);` avant chaque acces.

### 1.6 Undefined Behavior : va_arg avec types non-POD
- **Fichier:** `core/protocol/dte_protocol.hpp:625-745`
- **Severite:** CRITICAL
- **Description:** `DTEEncoder::encode(DTECommand, ...)` utilise `va_arg(args, std::string)` et `va_arg(args, BaseRawData)`. Passer des types avec constructeur/destructeur non-trivial via `...` est UB en C++. Fonctionne par chance sur GCC ARM actuel mais peut casser avec une mise a jour du compilateur ou des options d'optimisation differentes.
- **Fix:** Remplacer par un template variadic C++ ou des surcharges typees.

### 1.7 Race condition dans gnss_data_callback
- **Fichier:** `core/scheduling/gps_scheduler.cpp:296-302`
- **Severite:** CRITICAL
- **Description:** Le flag atomique `pending_data_logging` est set APRES l'ecriture des donnees (ligne 337). Fenetre de race : si le callback est re-invoque entre l'ecriture (ligne 299) et le set du flag, les donnees sont ecrasees.
- **Fix:** Set le flag AVANT l'ecriture des donnees :
```cpp
m_gnss_data.pending_data_logging = true;
m_gnss_data.data = data;
```

### 1.8 Race condition ISR sur m_pending_sat/nav
- **Fichier:** `ports/nrf52840/core/hardware/m10qasync/m10qasync.cpp:342, 376`
- **Severite:** CRITICAL
- **Description:** `react(UBXCommsEventSatReport&)` et `react(UBXCommsEventNavReport&)` sont appeles depuis un contexte ISR (callback UART DMA). Le `memcpy` sur `m_pending_sat`/`m_pending_nav` est hors du `InterruptLock`. Un second ISR entre le memcpy et le lock corrompt le buffer.
- **Fix:** Deplacer le memcpy dans le scope InterruptLock, ou utiliser un double-buffer.

### 1.9 Division par zero dans thermistor convert_temp()
- **Fichier:** `ports/nrf52840/core/hardware/thermistor/thermistor.cpp:82`
- **Severite:** CRITICAL
- **Description:** `R_BOTTOM * ((3.3 / v) - 1.0)` -- si `v == 0` (thermistance deconnectee, ADC lit 0), division par zero -> Infinity/NaN propage silencieusement. De plus si `v > 3.3` (bruit ADC), `r_therm < 0` et `log(negative)` = NaN.
- **Fix:**
```cpp
if (v < 0.001) { DEBUG_ERROR("Thermistor: ADC voltage near 0"); return NAN; }
if (v > 3.3) v = 3.3; // clamp
```

### 1.10 Flexible array member avec constructeur C++ (UB)
- **Fichier:** `ports/nrf52840/core/hardware/m10qasync/ubx.hpp:231-264`
- **Severite:** CRITICAL
- **Description:** `MSG_VALDEL`, `MSG_VALGET`, `MSG_VALSET` utilisent des flexible array members C99 (`uint32_t keys[]`, `uint32_t cfgData[]`) avec des constructeurs C++. Le constructeur ecrit via `reinterpret_cast<uint8_t*>(this + 1)` au-dela du struct. Si construit sur la stack sans espace supplementaire (ex: `setup_uart_port()` ligne 1205), ecrit dans la stack frame.
- **Fix:** Allouer dans un buffer correctement dimensionne ou serialiser dans un buffer pre-alloue.

### 1.11 Flag compilateur silencieusement ignore
- **Fichier:** `ports/nrf52840/CMakeLists.txt:532`
- **Severite:** CRITICAL
- **Description:** `$<$<COMPILE_LANGUAGE:C>:${-Wjump-misses-init}>` dereference une variable CMake inexistante `-Wjump-misses-init` -> expand en string vide. Le flag n'est jamais applique.
- **Fix:** `$<$<COMPILE_LANGUAGE:C>:-Wjump-misses-init>`

---

## 2. Fuites memoire

| # | Fichier | Ligne(s) | Description | Severite |
|---|---------|----------|-------------|----------|
| 1 | `tests/src/gps_test.cpp` | 56-63 | `fake_battery_mon` alloue avec `new` mais jamais `delete` dans teardown. Fuite a chaque test. | CRITICAL |
| 2 | `tests/src/config_store_test.cpp` | 148-164 | Si une assertion CppUTest echoue (longjmp), `delete store` est saute. | WARNING |
| 3 | `ports/nrf52840/main.cpp` | 685 | `pressure_sensor_devices[]` non initialise a nullptr -- risque de wild pointer. | CRITICAL |
| 4 | `ports/nrf52840/core/hardware/smd_sat/smd_sat.cpp` | multi | `m_nrf_spim` new/delete manuel dans 13+ emplacements. Leak si erreur dans certains chemins. | WARNING |
| 5 | `ports/nrf52840/core/hardware/smd_sat/smd_sat.cpp` | 2100-2157 | `dfu_enter()` return early sans cleanup SPI + sensor power. | WARNING |
| 6 | `ports/nrf52840/core/hardware/m10qasync/ubx_comms.hpp` | 33-35 | `UBXCommsEventDebug` stocke un pointeur brut vers un buffer DMA libere avant l'execution du task differe = **dangling pointer**. | CRITICAL |
| 7 | `tests/CMakeLists.txt` | 22-23 | `CPPUTEST_MEM_LEAK_DETECTION_DISABLED` desactive globalement la detection de fuites CppUTest. | WARNING |

---

## 3. Variables non initialisees

| Fichier | Variables | Severite |
|---------|-----------|----------|
| `core/configuration/config_store.hpp:347-349` | `m_battery_level`, `m_battery_voltage`, `m_is_battery_level_low` | CRITICAL |
| `core/services/gnss_detector_service.hpp:18-24` | `m_current_state`, `m_period_underwater_ms`, `m_period_surface_ms`, `m_min_num_dry_samples`, `m_num_dry_samples`, `m_dry_wet_threshold`, `m_gnss_only_detect_surfacing` | CRITICAL |
| `core/services/sensor_service.hpp:86-87` | `m_sample_number`, `m_sensor_background_active` | CRITICAL |
| `ports/nrf52840/core/hardware/m10qasync/m10qasync.cpp:45-53` | `m_num_nav_samples`, `m_num_sat_samples`, `m_fix_was_found`, `m_unrecoverable_error`, `m_database_overflow`, `m_uart_error_count`, `m_expected_dbd_messages`, `m_mga_ack_count`, `m_step`, `m_retries` | WARNING |
| `ports/nrf52840/core/hardware/thermistor/thermistor.cpp` | `m_is_init` pas dans la member initializer list | WARNING |

**Fix general :** Utiliser des in-class default member initializers :
```cpp
uint8_t  m_battery_level = 0;
uint16_t m_battery_voltage = 0;
bool     m_is_battery_level_low = false;
```

---

## 4. Dead code

| Fichier | Ligne(s) | Description |
|---------|----------|-------------|
| `config_store.hpp` | 356, 359 | `SECONDS_PER_DAY` et `HOURS_PER_DAY` jamais utilises |
| `config_store.hpp` | 191-195 | `#if MODEL_SB` / `#else` avec valeurs identiques |
| `config_store.hpp` | 788 | Assignation redondante `sensor_tx_enable = 0` (deja fait ligne 786) |
| `gps_scheduler.hpp` | 148 | `populate_gnss_data_and_callback()` declare, jamais defini |
| `gps_service.hpp` | 88 | `m_is_underwater` shadowed du parent, jamais utilise |
| `gps_service.hpp` | 106 | `populate_gnss_data_and_callback()` aussi declare ici, jamais defini |
| `m10qasync.cpp` | 561-563 | Code commente pour step 1 (MAX_BAUDRATE sync) |
| `m10qasync.cpp` | 1077-1083 | Bloc `else` inaccessible dans `state_sendofflinedatabase()` |
| `m10qasync.cpp` | 1673-1674 | `send_offline_database()` corps vide, jamais appele |
| `m10qasync.cpp` | 1215-1239 | `read_uart_port()` implemente, jamais appele |
| `m10qasync.hpp` | 42-43 | `Timeout::running` et `Timeout::end` jamais lus |
| `smd_sat.cpp` | 21-44 | `spistatus_string` et `kmacstatus_string` jamais references |
| `smd_sat.cpp` | 866-872 | `is_command_accepted()` et `is_firmware_ready()` toujours `true`, jamais appeles |
| `main.cpp` | 374 | `is_linkit_v3_v4_early` calcule mais jamais lu (2 appels PMU::hardware_version() gaspilles) |
| `main.cpp` | 883 | `return 0` apres `while(true)` |
| `CMakeLists.txt` (nrf52840) | 8 | `VERSION_NUMBER` set mais jamais utilise |
| `base_types.hpp` | multi | `break` apres `return` dans switch (~20 cas) |
| `base_types.hpp` | 3 | `#include <cstring>` inutilise |
| `dte_protocol.hpp` | 1-2 | `#include <ios>` et `#include <iomanip>` inutilises |
| `dte_protocol.hpp` | 249, 511 | `break` apres `throw` (inaccessible) |
| `argos_test.cpp` | 31, 49-50 | `linux_timer` alloue, jamais utilise |
| `argos_tx_test.cpp` | 317-452 | 135 lignes de tests commentes |
| `argos_tx_test.cpp` | 1087, 1165 | 2 tests `IGNORE_TEST` (~150 lignes inactives) |
| `encoder_test.cpp` | 33-36 | `setup()` et `teardown()` vides |

---

## 5. Redundance et optimisation de lignes

### 5.1 Tests (gain potentiel ~53%)

| Fichier | Lignes actuelles | Estimation apres | Gain | Technique |
|---------|:---:|:---:|:---:|-----------|
| `argos_test.cpp` | 4278 | ~1800 | ~58% | Helper pour config, GPS entry, mock expectations |
| `argos_tx_test.cpp` | 1651 | ~900 | ~45% | Helper config + supprimer tests commentes |
| `config_store_test.cpp` | 1513 | ~600 | ~60% | Template `test_param_persistence<T>(ParamID, T)` |
| `gps_test.cpp` | 784 | ~350 | ~55% | Helper config + parametriser dloc tests |
| `encoder_test.cpp` | 997 | ~700 | ~30% | Helper `check_encode()` |
| **Total** | **9223** | **~4350** | **~53%** | |

### 5.2 Code applicatif

| Zone | Description | Gain estime |
|------|-------------|:---:|
| `config_store.hpp:581-810` | 3 branches quasi-identiques dans `get_gnss_configuration()` et `get_argos_configuration()`. Extraire un helper pour les champs communs. | ~150 lignes |
| `gps_scheduler.cpp:225-255` + `gps_service.cpp:147-177` | Copie champ par champ `GNSSData -> GPSLogEntry` (30 lignes x2). Unifier `GNSSData`/`GPSInfo` ou utiliser memcpy. | ~55 lignes |
| `ubx.hpp:638-768` | `getParameterSize()` chaine de ~80 `if`. Remplacer par extraction de bits du key ID u-blox (bits 28-30). | ~130 lignes |
| `main.cpp:144-242` | 5 fault handlers avec blink pattern quasi-identique. Extraire un helper. | ~70 lignes |
| `tests/CMakeLists.txt:207-333` | Target `TurtleSimulation` duplique la config du target principal. Utiliser un INTERFACE library. | ~110 lignes |
| `ubx.hpp:557-623` | Enum `SIGATTCOMP_VALUES` enumere 0-63 explicitement. | ~67 lignes |
| `config_store.hpp` | ~60 casts redondants `(bool)true`, `(double)0`. | ~60 casts |
| `sensor_service.hpp:72-80` | Catch block avec logique dupliquee (les 2 branches appellent `service_complete`). | ~5 lignes |
| `dte_handler.cpp:160-175` | `save_params()` appele 3 fois dans RSTVW_REQ au lieu d'une fois apres le if-else. | ~4 lignes |

---

## 6. Robustesse

### 6.1 Pointeurs globaux extern sans null check
- **Fichiers:** `gps_scheduler.cpp`, `dte_handler.cpp`, `main.cpp`, `m10qasync.cpp`, `smd_sat.cpp`, `gnss_detector_service.hpp`
- **Description:** 6+ pointeurs globaux (`configuration_store`, `system_scheduler`, `sensor_log`, `battery_monitor`, `rtc`, `system_timer`) dereferences sans assert ni null check.
- **Fix:** Ajouter `assert(ptr != nullptr)` au point d'entree (ex: `start()`, `service_init()`).

### 6.2 `std::gmtime` non thread-safe
- **Fichiers:** `dte_handler.cpp:257`, `gps_scheduler.hpp:23`
- **Description:** Retourne un pointeur vers un buffer statique interne. Non-safe si appele depuis ISR ou multi-thread RTOS.
- **Fix:** Utiliser `gmtime_r()` avec une variable locale `struct tm`.

### 6.3 `using namespace std::string_literals` dans header
- **Fichier:** `config_store.hpp:20`
- **Description:** Pollue le namespace de tous les translation units qui incluent ce header.
- **Fix:** Deplacer dans le scope de la classe ou utiliser le prefix explicite.

### 6.4 `write_param(T&)` refuse les rvalues
- **Fichier:** `config_store.hpp:521`
- **Description:** `void write_param(ParamID, T& value)` ne compile pas avec `write_param(id, 42U)`.
- **Fix:** Changer en `const T& value`.

### 6.5 Securite : code d'acces hardcode 0x12345678
- **Fichier:** `core/protocol/dte_handler.cpp:125`
- **Description:** Fallback statique trivial a deviner pour la commande SECUR.
- **Fix:** Supprimer le fallback ou utiliser un secret derive par device.

### 6.6 Median incorrect pour taille paire
- **Fichier:** `core/services/sensor_service.hpp:93-98`
- **Description:** `compute_median_samples()` retourne `v[v.size()/2]` (element superieur). Pour `{1,2,3,4}` retourne 3 au lieu de 2.5.
- **Fix:**
```cpp
double compute_median_samples(std::vector<double>& v) {
    if (v.empty()) return 0.0;
    size_t n = v.size();
    auto mid = v.begin() + n / 2;
    std::nth_element(v.begin(), mid, v.end()); // O(n) au lieu de O(n log n)
    if (n % 2 == 0) {
        auto lower = std::max_element(v.begin(), mid);
        return (*mid + *lower) / 2.0;
    }
    return *mid;
}
```

### 6.7 FCS lu mais jamais valide
- **Fichier:** `core/protocol/dte_protocol.hpp:252-254`
- **Description:** Frame Check Sequence extrait des paquets Allcast puis `(void)fcs;`. Paquets corrompus acceptes silencieusement.
- **Fix:** Implementer la validation CRC-16.

### 6.8 Commentaires contradictoires sur `boot_count_check_modulo`
- **Fichier:** `core/configuration/config_store.hpp:856-872`
- **Description:** Le commentaire de la fonction dit "return true = should shutdown" mais le commentaire inline dit "return true = our turn to run". Semantique opposee.
- **Fix:** Clarifier la convention et corriger le mauvais commentaire.

### 6.9 `read_param` a des effets de bord
- **Fichier:** `core/configuration/config_store.hpp:406+`
- **Description:** `read_param` ecrit dans `m_params` pendant une "lecture" (appels I2C/ADC pour batterie, etc). Fonction non-const, non thread-safe, cout cache.
- **Fix:** Separer "refresh hardware values" de "read stored param".

### 6.10 Watchdog kick interval 14 minutes
- **Fichier:** `ports/nrf52840/main.cpp:432`
- **Description:** `14 * 60 * 1000` ms. Si le WDT timeout est < 14 min, le device reset avant le premier kick.
- **Fix:** Verifier que l'intervalle correspond au WDT timeout configure. Recommande : ~50% du timeout.

### 6.11 Overflow potentiel dans calcul de schedule GPS
- **Fichier:** `core/services/gps_service.cpp:44-51`
- **Description:** `(next_schedule - now) * MS_PER_SEC` retourne `unsigned int`. Si `aq_period > 4294` secondes (~71 min), overflow.
- **Fix:** Utiliser `uint64_t` pour la multiplication ou clamper le resultat.

### 6.12 Underflow `m_num_consecutive_fixes` si `min_num_fixes == 0`
- **Fichier:** `core/scheduling/gps_scheduler.cpp:322`
- **Description:** `--m_num_consecutive_fixes` sur unsigned = UINT_MAX si la valeur est 0.
- **Fix:** `if (m_num_consecutive_fixes > 0 && --m_num_consecutive_fixes > 0) return;`

### 6.13 `param_map` taille non validee a la compilation
- **Fichier:** `core/protocol/dte_params.cpp:10-236`
- **Description:** Si un dev ajoute un `ParamID` enum sans le param_map correspondant, indexation out-of-bounds silencieuse.
- **Fix:** `static_assert(sizeof(param_map)/sizeof(param_map[0]) == (size_t)ParamID::__PARAM_SIZE);`

### 6.14 Constructeur LPS28DFW echoue silencieusement
- **Fichier:** `ports/nrf52840/core/hardware/lps28dfw/lps28dfw.cpp:13`
- **Description:** `init()` appele dans le constructeur. Si `init()` retourne false (WHOAMI mismatch), l'objet est construit dans un etat invalide sans que l'appelant le sache.
- **Fix:** Throw une exception ou ajouter `is_initialized()`.

### 6.15 Unsigned underflow dans DUMPD_REQ
- **Fichier:** `core/protocol/dte_handler.cpp:331`
- **Description:** `total_entries - start_index` si `total_entries < start_index` -> underflow -> lecture massive.
- **Fix:** `unsigned int num_entries = (start_index < total_entries) ? std::min(...) : 0;`

---

## 7. C++ best practices

| Categorie | Fichiers | Description |
|-----------|----------|-------------|
| C-style casts partout | smd_sat, ubx_comms, config_store, dte_protocol | Utiliser `static_cast<>` / `reinterpret_cast` |
| `catch(...)` avale info | config_store (515, 527, 535), dte_handler, main | Catch types specifiques + DEBUG_ERROR |
| `catch(ErrorCode e)` par valeur | gps_scheduler.cpp:139 | Utiliser `const ErrorCode&` |
| `std::map<T*, bool>` pour EventEmitter | events.hpp:29 | `std::vector<T*>` (3 listeners max, meilleur cache) |
| `std::vector<uint8_t>` pour UBXParameter | ubx.hpp:93-116 | `std::array<uint8_t, 8>` (max 8 bytes) |
| `static inline std::map` dans header | dte_handler.hpp:62-94 | `constexpr` array ou `const char*[]` |
| Include guard `__GPS_SCHEDULER_HPP_` | gps_scheduler.hpp:1-2 | Identifiant reserve (`__`). Utiliser `#pragma once` |
| Comma operator au lieu de `;` | argos_tx_test.cpp:127-128, gps_test.cpp:92-93 | Remplacer par `;` |
| `#include <stdarg.h>` en C++ | dte_protocol.hpp:5 | Utiliser `#include <cstdarg>` |
| `#define MAX_CONFIG_ITEMS` macro | config_store.hpp:18 | Preferer `static constexpr unsigned int` |
| `(void)` dans signature | config_store.hpp:548, events.hpp | Style C, inutile en C++ |
| `virtual ~X() {}` redondant | gps_scheduler.hpp:72 | `= default` ou supprimer |
| Notification iteration copie | events.hpp:17, 23 | `const auto& m` au lieu de `auto m` |
| Modification de listeners pendant iteration | events.hpp:17-20 | UB si `react()` appelle `subscribe()`. Iterer sur une copie. |
| `gps_entry.header` wrapping struct inutile | gps_service.hpp:91-93 | `struct { GNSSData data; } m_gnss_data;` -> `GNSSData m_gnss_data;` |
| Identifiant reserve `_HAS_EXTERNAL_WAKEUP` | dte_params.cpp:5-8 | Renommer en `HAS_EXTERNAL_WAKEUP` |
| `static` functions dans header | ubx.hpp:41, 638 | Remplacer par `inline` |
| `%llu` pour `int64_t` signe | gps_scheduler.cpp:97 | Utiliser `%lld` ou `PRId64` |
| `memset` sur struct non-trivial | gps_scheduler.cpp:172,216, gps_service.cpp:116,137 | Utiliser `GPSLogEntry gps_entry{};` |
| Missing `#include <numeric>` pour `std::reduce` | sensor_service.hpp:91 | Ajouter l'include |
| `GNSSData` passe par valeur (112+ bytes) | gps_scheduler.hpp:147, gps_service.cpp:235 | Passer par `const GNSSData&` |
| Default arg dans definition pas declaration | gps_scheduler.cpp:187 | Deplacer dans le .hpp |

---

## 8. CMake

### ports/nrf52840/CMakeLists.txt

| # | Ligne(s) | Severite | Description |
|---|----------|----------|-------------|
| 1 | 532 | CRITICAL | `${-Wjump-misses-init}` -> `-Wjump-misses-init` (flag silencieusement ignore) |
| 2 | 1 | WARNING | `cmake_minimum_required(VERSION 3.1)` trop bas, `target_link_options` requiert 3.13+ |
| 3 | 18 | WARNING | `option(BOARD ...)` mauvaise utilisation pour une variable string. Utiliser `set(BOARD "LINKIT" CACHE STRING ...)` |
| 4 | 128-131 | INFO | `if(DEBUG_LEVEL)` empeche `DEBUG_LEVEL=0`. Utiliser `if(NOT DEFINED DEBUG_LEVEL)` |
| 5 | 343-344 | WARNING | `lps28dfw` gate sur `ENABLE_PRESSURE_SENSOR` mais aussi necessaire pour `ENABLE_CDT_SENSOR` |
| 6 | 341-349 | INFO | `ms58xx`, `bar100`, `ad5933`, `cdt`, `bma400`, `ltr_303` compiles meme si desactives |
| 7 | 451-452 | INFO | `Release=0`, `Debug=1` toujours definis quel que soit `CMAKE_BUILD_TYPE` |
| 8 | 659 | WARNING | `flash_softdev` depend du build app (inutile, c'est un hex SDK prebuilt) |
| 9 | 617-624 | WARNING | `create_wrapped_bin` utilise `stat -c` (Linux only), `crc32`, `xxd` (non portables) |
| 10 | 8 | INFO | `VERSION_NUMBER` set mais jamais utilise |
| 11 | multi | INFO | Mix `${PROJ_DIR}` (relatif) vs `${CMAKE_CURRENT_SOURCE_DIR}` (absolu) |

### tests/CMakeLists.txt

| # | Ligne(s) | Severite | Description |
|---|----------|----------|-------------|
| 1 | 22-23 | WARNING | Leak detection CppUTest desactivee globalement |
| 2 | 207-333 | WARNING | TurtleSimulation duplique ~130 lignes de config. Utiliser INTERFACE library |
| 3 | 216-220 | INFO | Sensor defines absents pour TurtleSimulation (present pour main target) |
| 4 | 193 | INFO | `src/main.cpp` non quote (inconsistant) |

---

## 9. Tests

### 9.1 Bugs dans les tests

| Fichier | Ligne(s) | Description |
|---------|----------|-------------|
| `argos_tx_test.cpp` | 96-102 | **CRITICAL** : `inject_gps_inactive()` envoie ACTIVE au lieu de INACTIVE |
| `gps_test.cpp` | 56-63 | **CRITICAL** : `fake_battery_mon` fuite memoire (delete manquant) |
| `argos_tx_test.cpp` | 127-128 | Comma operator au lieu de `;` |
| `gps_test.cpp` | 92-93 | Comma operator au lieu de `;` |
| `argos_test.cpp` | 313+ | `gps_entry.header.seconds` utilise sans etre initialise |
| `config_store_test.cpp` | 187, 376 | Buffers `clobber` non initialises pour tests de corruption |

### 9.2 Qualite des tests

- **`argos_test.cpp:4278 lignes`** : Le fichier le plus long. 12+ blocks de config de ~30 lignes repetes quasi-identiquement.
- **`gps_test.cpp`** : `GNSSDisabled` test n'a aucune assertion explicite (repose uniquement sur mock expectations).
- **`argos_tx_test.cpp:317-452`** : 135 lignes de tests commentes. 2 tests IGNORE_TEST (~150 lignes inactives).
- **Pattern general** : `new`/`delete` brut au lieu de `std::unique_ptr`. CppUTest longjmp sur assertion failure -> leaks.

---

## 10. Actions recommandees par priorite

### P0 - Immediat (bugs / crashes potentiels)

1. Fix `inject_gps_inactive()` dans `argos_tx_test.cpp:98`
2. Fix precedence operateur `ubx_comms.cpp:104`
3. Ajouter bounds check sur les 6 acces `arg_list[]` dans `dte_handler.cpp`
4. Ajouter bounds check `memcpy` dans `ubx_comms.cpp:96` et `:321`
5. Fix race condition `gnss_data_callback` dans `gps_scheduler.cpp:296-302`
6. Fix race condition ISR `m10qasync.cpp:342, 376`
7. Fix division par zero `thermistor.cpp:82`
8. Initialiser `pressure_sensor_devices[]` a nullptr dans `main.cpp:685`
9. Fix `${-Wjump-misses-init}` dans `CMakeLists.txt:532`
10. Ajouter `delete fake_battery_mon` dans `gps_test.cpp` teardown

### P1 - Court terme (robustesse)

1. Initialiser toutes les variables membres (config_store, gnss_detector, sensor_service, m10qasync)
2. Remplacer `va_arg` non-POD par template variadic dans `DTEEncoder::encode`
3. Utiliser `gmtime_r()` partout
4. Valider FCS des paquets Allcast
5. Utiliser `std::unique_ptr` pour `m_nrf_spim` dans smd_sat
6. Ajouter `static_assert` taille `param_map` vs `__PARAM_SIZE`
7. Fix median pour taille paire dans `sensor_service.hpp`
8. Fix commentaires contradictoires `boot_count_check_modulo`
9. Fix CMake `option(BOARD)` -> `set(BOARD ... CACHE STRING ...)`
10. Ajouter bounds check overflow GPS schedule (`gps_service.cpp:51`)

### P2 - Moyen terme (qualite / maintenance)

1. Factoriser les tests avec helpers (~53% reduction, ~4800 lignes gagnees)
2. Factoriser `get_gnss/argos_configuration()` dans config_store
3. Unifier `GNSSData` / `GPSInfo` pour eliminer la copie champ par champ
4. Supprimer tout le dead code identifie (section 4)
5. Remplacer `getParameterSize()` par extraction bits key ID (~130 lignes)
6. Activer la leak detection CppUTest
7. Remplacer `std::vector<uint8_t>` par `std::array<uint8_t,8>` dans UBXParameter
8. Remplacer `std::map<T*,bool>` par `std::vector<T*>` dans EventEmitter
9. Deplacer implementations non-template des headers vers .cpp (config_store, dte_protocol, gnss_detector, sensor_service)
10. Supprimer ~35 include directories BLE inutiles dans CMakeLists.txt
