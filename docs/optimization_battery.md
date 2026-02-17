# Optimisation Low-Power pour Mode TPL5111 (Bird Tracker RSPB)

**Date :** 2026-02-17
**Branche :** `clean-repo`
**Scope :** Analyse de consommation et plan d'optimisation pour le mode duty-cycle TPL5111

---

## Contexte

Le board RSPB utilise un TPL5111 (nano-power timer) qui reveille le nRF52840 toutes les **1h45 (6300s)**. Le cycle operationnel est :

```
TPL5111 wake -> boot MCU -> check modulo -> GNSS single fix -> lecture capteurs
-> transmission Argos (N messages espaces de X secondes) -> MCU_DONE -> TPL5111 coupe l'alimentation
```

Le code actuel a ete concu pour un fonctionnement **continu** (tracking marin surface/sous-marin) et contient de nombreuses fonctionnalites inutiles et couteuses pour ce mode duty-cycle oiseau :
- Detection sous-marine (SWS) inutile
- BLE init inutile (sauf demande reed switch)
- USB debug inutile en production
- LEDs invisibles sur un oiseau
- Timeouts GPS trop longs

L'objectif est de minimiser la consommation par cycle pour maximiser l'autonomie batterie (NCR18650 3400mAh).

---

## 1. Analyse de Consommation (par cycle de reveil)

| Composant | Courant | Duree | Energie (mAh) | Verdict |
|-----------|---------|-------|----------------|---------|
| Reed switch check + LED WHITE | ~10mA | 3s | 0.008 | INUTILE en TPL5111 |
| BootState -> PreOp delay | ~5mA | 1s | 0.001 | INUTILE |
| PreOp LED + delai 5s | ~8mA | 5s | 0.011 | INUTILE |
| BLE SoftDevice idle | ~0.3mA | continu | 0.015/cycle | INUTILE |
| GPS boot delay bloquant | ~70mA | 1s | 0.019 | EXCESSIF (30ms suffisent) |
| GPS configuration (14 etapes) | ~70mA | ~4s | 0.078 | OPTIMISABLE |
| **GPS acquisition (succes)** | **~70mA** | **30-120s** | **0.58-2.33** | **Principal consommateur** |
| **GPS acquisition (echec cold start)** | **~70mA** | **530s** | **10.3** | **CRITIQUE - trop long** |
| Sensor sampling pendant GPS | ~5mA | pendant GPS | 0.02-0.17 | OK |
| Argos TX par message | ~200mA | ~1s/msg | 0.056/msg | OK |
| **Argos RX (ecoute satellite)** | **~15mA** | **900s (15min)** | **3.75** | **EXCESSIF** |

### Top 3 des consommateurs

1. **GPS acquisition echouee (cold start 530s)** = 10.3 mAh - CRITIQUE
2. **Argos RX window (15 minutes d'ecoute)** = 3.75 mAh - EXCESSIF
3. **GPS acquisition reussie** = 0.58-2.33 mAh - incompressible mais optimisable

### Probleme specifique TPL5111

Le TPL5111 coupe **toute** l'alimentation entre les cycles. Consequences :
- **Pas de batterie backup GPS** : chaque boot est un cold start (pas d'ephemerides preservees)
- **Pas de RAM preservee** : tout reinitialise a chaque reveil
- **Le boot modulo check est commente** : chaque reveil fait un cycle complet meme quand il ne devrait pas

---

## 2. Optimisations par Priorite

### P0 - CRITIQUES (impact maximal)

#### P0-1 : Activer le Boot Modulo Check

**Fichier :** `ports/nrf52840/main.cpp:529-534`

**Probleme :** Le check modulo est commente (`// TODO: Re-enable for production`). Avec un TPL5111 qui reveille toutes les 1h45 et un modulo=4, l'objectif est de n'executer le cycle complet que toutes les ~7h. Actuellement les 3 reveils intermediaires font un cycle complet inutile.

**Action :** Decommenter les lignes 530-534 :
```cpp
if (boot_counter > 0 && !configuration_store->boot_count_check_modulo(boot_counter)) {
    DEBUG_INFO("EXTERNAL_WAKEUP: Not our turn to run (modulo check), powering down");
    PMU::powerdown();
}
```

**Gardes existantes :** `boot_count_check_modulo()` dans `config_store.hpp:847-863` gere deja :
- `modulo < 2` : warning + allow boot (pas de division par zero)
- `boot_counter == 0` : premier boot toujours autorise
- `boot_count_clear()` apres un cycle actif : reset du compteur

**Economie :** ~7.8 mAh par periode de 7h (3 cycles complets de ~2.6 mAh evites)

**Risque :** FAIBLE - le code est deja ecrit et teste

---

#### P0-2 : Reduire GNSS Cold Start Timeout (530s -> 180s)

**Fichier :** `core/configuration/config_store.hpp:165`

**Probleme :** `GNSS_COLD_ACQ_TIMEOUT = 530U` (8min50). Sans batterie backup GPS, chaque boot est un cold start. Si le fix echoue (canopee dense, nid), le GPS reste allume 530s a ~70mA = 10.3 mAh gaspilles.

Le u-blox M10Q cold start TTFF typique :
- Ciel ouvert : 26s
- Conditions moderees : 60s
- Avec AssistNow Offline : 15s

180s est largement suffisant. Si pas de fix en 3 min, les conditions sont trop mauvaises.

**Action :** Changer la valeur par defaut :
```cpp
/* GNSS_COLD_ACQ_TIMEOUT */ 180U,  // Etait 530U
```

**Alternative sans modif code :** Configurer via DTE `PARMW` avant deploiement.

**Economie :** ~6.8 mAh par acquisition echouee (10.3 - 3.5 mAh)

**Risque :** FAIBLE pour config DTE, MOYEN pour changement defaut (affecte tous les builds)

---

#### P0-3 : Activer SHUTDOWN_TIMER (securite anti-blocage)

**Fichier :** `core/configuration/config_store.hpp:218`

**Probleme :** `SHUTDOWN_TIMER = 0U` (desactive). Si le FSM bloque (exception non geree, deadlock scheduler, GPS qui ne repond plus), le device reste allume pendant tout l'intervalle TPL5111 (6300s a ~70mA = **122 mAh** gaspilles).

**Budget temps d'un cycle normal :**
- Boot + init : ~5s
- GPS cold start max : 180s (apres P0-2)
- Sensor reads : ~5s
- Argos TX (6 msg @ 60s interval) : ~360s
- Marge securite : ~50s
- **Total : ~600s (10 min)**

**Action :** Changer la valeur par defaut :
```cpp
#ifdef EXTERNAL_WAKEUP
/* [88] SHUTDOWN_TIMER */ 600U,    // 10 minutes max cycle
#else
/* [88] SHUTDOWN_TIMER */ 0U,
#endif
```

**Note :** Le mecanisme est deja implemente dans `main.cpp:848-861` :
```cpp
system_scheduler->post_task_prio([]() {
    PMU::powerdown();
}, "SHUTDOWN_TIMER", Scheduler::DEFAULT_PRIORITY, shutdown_timer * 1000);
```

**Economie :** Previent les drains catastrophiques (jusqu'a 122 mAh evites par blocage)

**Risque :** FAIBLE - mecanisme deja code, juste activer la valeur

---

#### P0-4 : Sauter le Reed Switch 3s Check en mode TPL5111

**Fichier :** `ports/nrf52840/main.cpp:368-469`

**Probleme :** A chaque Power On Reset (que le TPL5111 cause a chaque reveil), le code attend jusqu'a 3s en verifiant si le reed switch est maintenu. Pendant ce temps : LED WHITE allumee (~8mA) + buzzer actif (~20mA). Sur un oiseau, personne n'interagit avec le reed switch a chaque reveil.

**Action :** Ajouter un guard `#ifdef EXTERNAL_WAKEUP` au debut du bloc :
```cpp
#ifdef EXTERNAL_WAKEUP
    // TPL5111 mode: skip reed switch power-on validation
    // Le reed switch reste disponible via GenTracker FSM pour entrer en ConfigurationState
#else
#ifdef POWER_ON_RESET_REQUIRES_REED_SWITCH
    // ... code existant inchange ...
#endif
#endif
```

**Economie :** 3s de boot time + ~0.028 mAh/boot + 3s de budget SHUTDOWN_TIMER recuperees

**Risque :** FAIBLE - le reed switch reste fonctionnel via le FSM GenTracker pour entrer en ConfigurationState

---

### P1 - IMPORTANTS (economies significatives)

#### P1-1 : Differer l'Init BLE

**Fichier :** `ports/nrf52840/main.cpp:357`

**Probleme :** `BleInterface::get_instance().init()` est appele a chaque boot. Le SoftDevice nRF52840 consomme ~0.3mA idle et reserve ~12KB RAM. En mode TPL5111 autonome, le BLE n'est jamais utilise sauf demande explicite via reed switch.

**Action :**
```cpp
#ifndef EXTERNAL_WAKEUP
    DEBUG_TRACE("BLE...");
    BleInterface::get_instance().init();
#else
    // BLE init differe - sera fait si ConfigurationState est entre via reed switch
#endif
```

Et dans `gentracker.cpp` `ConfigurationState::entry()` (ligne 289), ajouter avant `ble_service->start()` :
```cpp
#ifdef EXTERNAL_WAKEUP
    BleInterface::get_instance().init();
#endif
```

**Verification :** `ble_service->start()` dans `OperationalState::entry()` (ligne 236) est deja garde par `debug_mode == BaseDebugMode::BLE_NUS` (ligne 234). Le RSPB utilise UART par defaut, donc ce chemin n'est pas pris.

**Economie :** ~0.3mA continu pendant tout le cycle + ~100ms boot time

**Risque :** MOYEN - tester que ConfigurationState fonctionne toujours avec init BLE differe

---

#### P1-2 : Sauter le Polling USB dans la Main Loop

**Fichier :** `ports/nrf52840/main.cpp:873`

**Probleme :** `NrfUSB::process()` est appele a chaque iteration de la boucle principale, meme sur RSPB ou le debug est en UART et USB n'est pas initialise.

**Action :**
```cpp
while (true) {
    try {
#ifndef EXTERNAL_WAKEUP
        NrfUSB::process();
#endif
        system_scheduler->run();
        PMU::run();
    } catch (ErrorCode e) {
```

**Economie :** ~0.1-0.5mA en overhead CPU par iteration de boucle

**Risque :** FAIBLE - RSPB utilise deja UART

---

#### P1-3 : Supprimer l'Etape GPS `setup_power_management()` (redondante)

**Fichier :** `ports/nrf52840/core/hardware/m10qasync/m10qasync.cpp:676-681`

**Probleme :** Dans la sequence de configuration GPS (14 etapes) :
- **Step 9** : `setup_power_management()` configure PSM-CT (cyclic tracking) - lignes 1217-1254
- **Step 10** : `setup_continuous_mode()` ecrase immediatement avec `CFG::PM::FULL` - lignes 1256-1275

Step 9 est du temps/energie gaspille : envoi VALSET via UART + attente ACK ~150ms a 75mA.

Pour le TPL5111 single-shot : FULL mode est correct (acquisition la plus rapide possible), donc step 9 n'a jamais d'utilite.

**Action :** Sauter step 9 :
```cpp
} else if (m_step == 9) {
    // Skip - step 10 (setup_continuous_mode) overrides to FULL mode anyway
    m_step++;
    m_op_state = OpState::IDLE;
```

**Economie :** ~150ms a 75mA = 0.003 mAh/boot (mineur, mais accelere la config)

**Risque :** FAIBLE - step 10 ecrase toujours step 9

---

#### P1-4 : Reduire le Delai GPS Boot de 1s a 500ms

**Fichier :** `ports/nrf52840/core/hardware/m10qasync/m10qasync.cpp:213`

**Probleme :** `PMU::delay_ms(1000)` est un delai bloquant de 1 seconde apres power-on GPS. Le M10Q demarre en ~30ms selon la datasheet. Le `sync_baud_rate()` qui suit a deja ses propres mecanismes de retry (6 tentatives).

**Action :**
```cpp
PMU::delay_ms(500); // M10Q boot ~30ms, 500ms marge conservative
```

Etape suivante : reduire a 250ms apres validation a 500ms.

**Economie :** 500ms a 75mA = 0.010 mAh/boot

**Risque :** MOYEN - a tester sur le hardware, le delai original peut inclure la stabilisation du rail VSENSORS

---

#### P1-5 : Supprimer LEDs + Delai PreOperationalState en mode TPL5111

**Fichiers :**
- `core/sm/gentracker.hpp:51` : `TRANSIT_PERIOD_MS = 5000`
- `core/sm/gentracker.cpp:119` : LED Boot
- `core/sm/gentracker.cpp:196-199` : LED PreOp
- `ports/nrf52840/main.cpp:330` : LED init WHITE

**Probleme :** Le FSM passe par BootState (LED + 1s delay) puis PreOperationalState (LED batterie + **5 secondes** de delai) avant d'atteindre OperationalState. Ces indicateurs visuels sont inutiles sur un oiseau.

**Action :**

Dans `gentracker.hpp`, TRANSIT_PERIOD conditionnel :
```cpp
#ifdef EXTERNAL_WAKEUP
    static inline const unsigned int TRANSIT_PERIOD_MS = 100;
#else
    static inline const unsigned int TRANSIT_PERIOD_MS = 5000;
#endif
```

Dans `BootState::entry()` :
```cpp
#ifndef EXTERNAL_WAKEUP
    led_handle::dispatch<SetLEDBoot>({});
#endif
```

Dans `PreOperationalState::entry()` :
```cpp
#ifndef EXTERNAL_WAKEUP
    if (battery_monitor->is_battery_low())
        led_handle::dispatch<SetLEDPreOperationalBatteryLow>({});
    else
        led_handle::dispatch<SetLEDPreOperationalBatteryNominal>({});
#endif
```

Dans `main.cpp:330`, LED OFF au boot :
```cpp
#ifdef EXTERNAL_WAKEUP
    NrfRGBLed nrf_status_led("STATUS", BSP::GPIO::GPIO_LED_RED,
        BSP::GPIO::GPIO_LED_GREEN, BSP::GPIO::GPIO_LED_BLUE, RGBLedColor::OFF);
#else
    NrfRGBLed nrf_status_led("STATUS", BSP::GPIO::GPIO_LED_RED,
        BSP::GPIO::GPIO_LED_GREEN, BSP::GPIO::GPIO_LED_BLUE, RGBLedColor::WHITE);
#endif
```

**Economie :** 5s de delai PreOp evites + ~0.02 mAh LED par boot

**Risque :** FAIBLE - pas d'observateur humain sur un oiseau

---

### P2 - CONFIGURATION (pas de changement code)

Parametres DTE a configurer via BLE avant deploiement sur chaque device :

| Parametre | Defaut | Recommande | Raison |
|-----------|--------|------------|--------|
| `GNSS_COLD_ACQ_TIMEOUT` | 530 | 180 | Reduire gaspillage sur echec acquisition |
| `GNSS_ACQ_TIMEOUT` | 120 | 90 | Fix plus rapide en warm start |
| `SHUTDOWN_TIMER` | 0 | 600 | Filet de securite anti-blocage |
| `BOOT_COUNTER_MODULO` | 2 | 4 | ~7h entre cycles actifs (4 x 1h45) |
| `UNDERWATER_EN` | false | false | Confirmer desactive (pas de SWS) |
| `ARGOS_RX_EN` | true | **false** | Desactiver 15min d'ecoute RX (-3.75 mAh/cycle) |
| `GNSS_DYN_MODEL` | PORTABLE | AIRBORNE_1G | Optimise pour dynamique de vol oiseau |
| `GNSS_HACCFILT_THR` | 5 | 10 | Accepter fix moins precis = TTFF plus court |
| `GNSS_MIN_NUM_FIXES` | 1 | 1 | Un seul fix suffit |
| `DRY_TIME_BEFORE_TX` | 1 | 0 | Pas de SWS, toujours en surface |
| `NTRY_PER_MESSAGE` | 6 | 3-4 | Reduire les repetitions TX |
| `LED_MODE` | HRS_24 | ALWAYS_OFF | Personne ne voit les LED sur un oiseau |

---

## 3. Estimation des Economies Totales

### Par periode de ~7h (4 reveils TPL5111, modulo=4)

| Changement | Economie (mAh) | Priorite |
|-----------|----------------|----------|
| P0-1: Boot modulo (3 cycles evites) | 7.8 | P0 |
| P0-2: Cold start 180s vs 530s (50% echec) | 3.4 | P0 |
| P0-3: Shutdown timer (previent blocages) | 0 a 122 | P0 |
| P0-4: Skip reed switch 3s | 0.028 | P0 |
| P1-1 a P1-5: BLE, USB, LED, GPS delays | ~0.15 | P1 |
| P2: ARGOS_RX_EN=false | 3.75 | P2 (config) |
| **TOTAL par 7h** | **~15 mAh** | |

### Autonomie estimee (NCR18650 3400mAh)

| Scenario | Conso/7h | Autonomie |
|----------|----------|-----------|
| **Avant optimisation** | ~45 mAh | **~22 jours** |
| **Apres P0 seulement** | ~25 mAh | **~40 jours** |
| **Apres P0 + P1 + P2** | ~15 mAh | **~66 jours** |

**Gain : ~3x l'autonomie** avec toutes les optimisations.

---

## 4. Fichiers Critiques a Modifier

| Fichier | Changements |
|---------|-------------|
| `ports/nrf52840/main.cpp` | Boot modulo (P0-1), reed switch skip (P0-4), BLE init guard (P1-1), USB guard (P1-2), LED OFF init (P1-5) |
| `core/configuration/config_store.hpp` | Cold timeout defaut (P0-2), shutdown timer defaut (P0-3) |
| `ports/nrf52840/core/hardware/m10qasync/m10qasync.cpp` | GPS step 9 skip (P1-3), boot delay reduction (P1-4) |
| `core/sm/gentracker.hpp` | TRANSIT_PERIOD_MS conditionnel (P1-5) |
| `core/sm/gentracker.cpp` | LED suppression boot/preop (P1-5), BLE init differe dans ConfigurationState (P1-1) |

---

## 5. Plan d'Implementation Progressive

### Phase 1 - Impact maximal, risque minimal
1. P0-1 : Decommenter boot modulo check (1 ligne)
2. P0-3 : Activer SHUTDOWN_TIMER=600 (1 ligne)
3. P2 : Appliquer config DTE recommandee (pas de code)

### Phase 2 - Boot rapide
4. P0-4 : Skip reed switch check (#ifdef guard)
5. P1-5 : LED OFF + PreOp delay reduit (#ifdef guards)
6. P1-6 : LED OFF au boot (#ifdef guard)

### Phase 3 - Peripheriques
7. P1-1 : BLE init differe (#ifdef guard + init dans ConfigurationState)
8. P1-2 : USB polling skip (#ifdef guard)

### Phase 4 - GPS
9. P0-2 : Cold start timeout 180s (defaut ou config DTE)
10. P1-3 : Skip step 9 GPS config
11. P1-4 : Reduire boot delay GPS a 500ms

---

## 6. Verification

1. **Build :** `cmake -DBOARD=RSPB` (active `EXTERNAL_WAKEUP`) + build standard pour verifier pas de regression
2. **Tests unitaires :** `cd tests && cmake .. && make && ./tests`
3. **Tests fonctionnels sur device :**
   - Boot modulo : 4 reveils successifs, seul le 4eme execute un cycle complet
   - SHUTDOWN_TIMER : bloquer le FSM, verifier powerdown apres 600s
   - GPS fix : obtenir un fix GNSS en conditions normales (<180s)
   - Argos TX : messages envoyes correctement apres fix
   - Reed switch : entree en ConfigurationState avec BLE init differe fonctionne
4. **Mesure courant :** Analyseur de puissance (PPK2) pour profil courant d'un cycle complet avant/apres
