# Audit Qualite & Robustesse -- linkit-v4-core (scenario RSPB)
**Date:** 2026-02-16
**Scope:** Protocole DTE, module satellite SMD, GNSS/GPS, sensors, scheduler, state machine, main
**Lignes analysees:** ~15,000

---

## CRITIQUES (crash/corruption memoire garantis)

### #1 - Buffer overflow dans le DFU satellite (stack corruption)
- **Fichiers:** `ports/nrf52840/core/hardware/smd_sat/smd_sat.cpp:85-118`, `:1600`
- **Status:** [ ] A fixer

Les buffers de frame SPI sont de 64 octets (`SPI_PROTOCOL_APLUS_FRAME_SIZE`), mais le chemin DFU envoie des payloads de 248 octets (chunks) et 256 octets (header). `build_aplus_frame` et `dfu_send_command` font un `memcpy` dans un buffer stack de 64 octets :

```cpp
uint8_t tx_buf[SPI_PROTOCOL_APLUS_FRAME_SIZE]; // 64 bytes
memcpy(&tx_buf[idx], data, data_len);          // data_len peut etre 248!
```

**Impact:** Corruption de stack a chaque ecriture DFU. Le DFU satellite ne peut pas fonctionner.

**Fix:** Utiliser `SPI_PROTOCOL_APLUS_MAX_FRAME_LEN` (255) pour les buffers dans `dfu_send_command` et `build_aplus_frame` quand le data_len depasse 58 octets.

---

### #2 - Overflow m_samples[5] avec l'accelerometre BMA400 (6 canaux)
- **Fichier:** `core/services/sensor_service.hpp:79`
- **Status:** [ ] A fixer

```cpp
std::vector<double> m_samples[5]; // tableau de 5
```

Mais `AXLSensorService::sensor_num_channels()` retourne 6. La boucle de lecture :

```cpp
for (unsigned int chan = 0; chan < sensor_num_channels(); chan++)
    m_samples[chan].push_back(...); // m_samples[5] = overflow!
```

**Impact:** Ecriture hors limites a chaque cycle de lecture accelerometre sur RSPB.

**Fix:** Changer `m_samples[5]` en `m_samples[6]`, ou mieux, dimensionner dynamiquement selon `sensor_num_channels()`.

---

### #3 - `operator==` de BasePassPredict inverse
- **Fichier:** `core/protocol/base_types.hpp:553-556`
- **Status:** [ ] A fixer

```cpp
static bool operator==(const BasePassPredict& lhs, const BasePassPredict& rhs) {
    return std::memcmp(&lhs, &rhs, sizeof(BasePassPredict)); // INVERSE!
}
```

`memcmp` retourne 0 quand egal, donc `operator==` retourne `false` quand les objets sont egaux et `true` quand ils sont differents.

**Fix:** `return std::memcmp(&lhs, &rhs, sizeof(BasePassPredict)) == 0;`

**Note:** De plus, `memcmp` sur un struct avec padding et floats est problematique. Envisager une comparaison champ par champ.

---

### #4 - Bypass securite SECUR -- action accordee meme en cas d'echec d'authentification
- **Fichier:** `core/protocol/dte_handler.hpp:1021`
- **Status:** [ ] A fixer

```cpp
case DTECommand::SECUR_REQ:
    resp = SECUR_REQ(error_code, arg_list);
    if (!error_code) action = DTEAction::SECUR;
    break;
```

`SECUR_REQ` gere l'erreur en interne mais ne modifie pas le `error_code` de `handle_dte_message`. L'action `DTEAction::SECUR` est toujours appliquee, meme si le code d'acces est mauvais.

**Fix:** `SECUR_REQ` doit setter `error_code` quand l'authentification echoue, ou le switch doit verifier le contenu de `resp`.

---

## HIGH (crash probable ou corruption de donnees)

### #5 - Data race ISR/scheduler dans les callbacks GNSS
- **Fichier:** `ports/nrf52840/core/hardware/m10qasync/m10qasync.cpp:341-372`
- **Status:** [ ] A fixer

Les callbacks `react(UBXCommsEventSatReport)` et `react(UBXCommsEventNavReport)` utilisent des `static` locales ecrasees par l'ISR UART avant que le scheduler n'execute la lambda.

```cpp
static UBXCommsEventSatReport sat;
std::memcpy(&sat, &s, sizeof(sat));
system_scheduler->post_task_prio([this]() {
    // ... uses sat ... // peut etre ecrasee entre-temps
}, "SatReport");
```

**Fix:** Copier les donnees dans la lambda par valeur (capture par copie) ou utiliser un ring buffer.

---

### #6 - `m_is_active` jamais reset sur GPSEventError
- **Fichier:** `core/services/gps_service.cpp:216-224`
- **Status:** [ ] A fixer

Contrairement aux autres handlers (`GPSEventMaxNavSamples`, `GPSEventPVT`), le handler d'erreur ne met pas `m_is_active = false`. Risque de double `power_off()` et double `service_complete()`.

**Fix:** Ajouter `m_is_active = false;` au debut du handler `react(GPSEventError)`.

---

### #7 - Busy-wait infini sans timeout sur le UART GNSS
- **Fichier:** `ports/nrf52840/core/hardware/m10qasync/ubx_comms.hpp:91,114,139`
- **Status:** [ ] A fixer

```cpp
while (m_is_send_busy);  // spin loop non borne
```

Si l'IRQ TX UART ne se declenche jamais, le systeme se bloque definitivement.

**Fix:** Ajouter un compteur/timeout et retourner une erreur si depasse.

---

### #8 - `std::get` sans verification de type/bounds sur arg_list DTE
- **Fichier:** `core/protocol/dte_handler.hpp` (multiples locations: 239, 260, 301, 316, 500, 725...)
- **Status:** [ ] A fixer

Les handlers DTE utilisent `std::get<T>(arg_list[N])` sans `std::holds_alternative` ni verification de `arg_list.size()`. Un payload mal forme declenche `std::bad_variant_access` non rattrape par le `catch (ErrorCode e)`.

**Fix:** Verifier `arg_list.size()` et utiliser `std::holds_alternative` ou `std::get_if` avant chaque acces.

---

### #9 - Dereferences de pointeurs globaux sans null-check
- **Fichier:** `core/protocol/dte_handler.hpp` (lignes 135, 167, 196, 210, 222, 242, 265, 347, 702, 838)
- **Status:** [ ] A fixer

`configuration_store->` et `memory_access->` utilises dans tous les handlers sans jamais verifier que le pointeur est non-null. Un hard fault garanti si appele avant initialisation.

**Fix:** Guard `if (!configuration_store) return error_response;` en entree de chaque handler, ou mieux, utiliser dependency injection.

---

### #10 - Double transition d'etat dans `state_transmit_pending`
- **Fichier:** `ports/nrf52840/core/hardware/smd_sat/smd_sat.cpp:1153-1166`
- **Status:** [ ] A fixer

```cpp
SMD_STATE_CHANGE(transmit_pending, idle);        // transition vers idle
if (--m_state_counter == 0) {
    SMD_STATE_CHANGE(transmit_pending, error);   // mais l'etat est deja idle!
}
```

Le mauvais exit handler est appele, et `m_state_counter` n'est pas initialise dans `state_transmit_pending_enter`.

**Fix:** Restructurer la logique -- ne pas transitionner vers idle si on va ensuite transitionner vers error. Initialiser `m_state_counter` dans `state_transmit_pending_enter()`.

---

### #11 - Exception dans `state_error_enter` du satellite
- **Fichier:** `ports/nrf52840/core/hardware/smd_sat/smd_sat.cpp:1000-1011`
- **Status:** [ ] A fixer

```cpp
void SmdSat::state_error_enter() {
    uint8_t status = 0;
    get_kmac_status(&status);        // SPI sur un bus potentiellement en panne!
    notify(KineisEventDeviceError({}));
    SMD_STATE_CHANGE(error, stopped);
}
```

Si l'erreur vient du bus SPI, `get_kmac_status()` va throw une exception non rattrapee.

**Fix:** Wrapper l'appel SPI dans un try-catch, ou supprimer l'appel SPI du error handler.

---

### #12 - TCXO warmup tronque 32-bit dans 8-bit
- **Fichier:** `ports/nrf52840/core/hardware/smd_sat/smd_sat.cpp:719-735`
- **Status:** [ ] A fixer

```cpp
void read_tcxo_warmup(uint8_t *time_ms) { // devrait etre uint32_t*
    *time_ms |= rx[i] << (8 * i);         // perd les octets hauts
}
```

**Fix:** Changer le type du parametre en `uint32_t*` et mettre a jour tous les call sites.

---

### #13 - Violation ODR -- `GPSNavSettings` defini 2 fois differemment
- **Fichiers:** `core/hardware/gps.hpp:8-22` vs `core/scheduling/gps_scheduler.hpp:64-68`
- **Status:** [ ] A fixer

Deux structs avec le meme nom mais 3 vs 13 champs. Si les deux headers sont inclus dans le meme TU = undefined behavior.

**Fix:** Renommer la version legacy de `gps_scheduler.hpp` (ex: `GPSNavSettingsLegacy`) ou la supprimer si inutilisee.

---

### #14 - LPS28DFW retourne 0 silencieusement en cas d'erreur I2C
- **Fichier:** `ports/nrf52840/core/hardware/lps28dfw/lps28dfw.cpp:36-57`
- **Status:** [ ] A fixer

```cpp
if (lps28dfw_trigger_sw(&m_ctx, &m_mode) != 0) {
    temperature = pressure = 0;
    return;   // Retourne 0 au lieu de throw
}
```

Le `PressureDetectorService` interprete 0 comme "pas sous l'eau" -> fausse detection de surface -> transmissions ARGOS sous l'eau.

**Fix:** `throw ErrorCode::SPI_COMMS_ERROR;` (ou un code I2C dedie) au lieu de retourner 0.

---

### #15 - Service constructor s'auto-enregistre avant construction de la classe derivee
- **Fichier:** `core/services/service.cpp:76`
- **Status:** [ ] A fixer

```cpp
Service::Service(...) {
    m_unique_id = ServiceManager::add(*this); // vtable incomplete!
}
```

`ServiceManager::add(*this)` est appele dans le constructeur de base, avant que le constructeur derive ne s'execute.

**Fix:** Deplacer l'enregistrement dans une methode `register_service()` appelee apres construction, ou dans `Service::start()`.

---

### #16 - `compute_oneshot_samples` throw `std::out_of_range` non rattrape
- **Fichier:** `core/services/sensor_service.hpp:91-93`
- **Status:** [ ] A fixer

```cpp
double compute_oneshot_samples(std::vector<double>& v) {
    return v.at(0); // throw std::out_of_range si vide
}
```

Le catch existant attrape `ErrorCode`, pas `std::out_of_range`.

**Fix:** Verifier `v.empty()` avant l'acces, ou ajouter un `catch (std::exception&)`.

---

### #17 - Erreur sensor background reste stuck
- **Fichier:** `core/services/sensor_service.hpp:71-75`
- **Status:** [ ] A fixer

```cpp
} catch (ErrorCode e) {
    if (!m_sensor_background_active)
        service_complete(nullptr, nullptr, reschedule);
    // Si m_sensor_background_active == true: rien ne se passe!
}
```

**Fix:** Ajouter un `else { m_sensor_background_active = false; service_complete(nullptr, nullptr, reschedule); }`.

---

### #18 - `read_credentials` fuite de ressources SPI/power sur exception
- **Fichier:** `ports/nrf52840/core/hardware/smd_sat/smd_sat.cpp:1500-1571`
- **Status:** [ ] A fixer

Pas de try-catch autour des appels SPI (`read_id`, `read_address`, `read_seckey`, `read_rconf_raw`). Si l'un d'eux throw, le `NrfSPIM` et le power rail restent alloues.

**Fix:** Wrapper dans un try-catch ou utiliser RAII pour la ressource SPI.

---

### #19 - Missing watchdog kick dans le DFU buffer-based
- **Fichier:** `ports/nrf52840/core/hardware/smd_sat/smd_sat.cpp:2230-2256`
- **Status:** [ ] A fixer

La boucle DFU file-based a `PMU::kick_watchdog()` (ligne 2361) mais pas la boucle buffer-based (~823 iterations pour 204KB).

**Fix:** Ajouter `PMU::kick_watchdog();` dans la boucle buffer-based.

---

## MEDIUM

### #20 - `BaseDebugMode` decode ne correspond pas a l'enum
- **Fichiers:** `core/protocol/dte_protocol.hpp:997-1006` vs `core/protocol/base_types.hpp:508-512`
- **Status:** [ ] A fixer

L'enum definit `UART=0, USB_CDC=1, BLE_NUS=2`, mais le decode mappe `"0"->USB_CDC, "1"->BLE_NUS`. UART n'est jamais decodable.

---

### #21 - `sscanf` format `"%1ud"` invalide
- **Fichier:** `core/protocol/dte_protocol.hpp:1524`
- **Status:** [ ] A fixer

Le `d` apres `u` n'est pas un format specifier -- `sscanf` essaie de matcher un caractere literal `d`.

**Fix:** `"%1u"`.

---

### #22 - `ParamID` enum shift selon `#ifdef` = ABI fragile
- **Fichier:** `core/protocol/base_types.hpp:54-265`
- **Status:** [ ] A evaluer

Les valeurs numeriques de `ParamID` changent selon les flags de compilation (`ENABLE_*_SENSOR`, `EXTERNAL_WAKEUP`, `ARGOS_SMD`). Si un composant persiste ces valeurs en flash, elles deviennent incompatibles entre builds.

**Fix:** Utiliser des valeurs fixes (ex: `PARAM_X = 100`) ou un schema de serialisation base sur les noms.

---

### #23 - `std::gmtime` peut retourner `nullptr`
- **Fichiers:** `core/protocol/dte_handler.hpp:356`, `core/protocol/dte_protocol.hpp:408`
- **Status:** [ ] A fixer

`std::gmtime()` peut retourner `nullptr` pour des valeurs `time_t` invalides. Le resultat est utilise directement dans `strftime`.

**Fix:** Verifier le retour de `gmtime` avant utilisation.

---

### #24 - UB: `uint8_t << 24` signed overflow
- **Fichier:** `ports/nrf52840/core/hardware/smd_sat/smd_sat.cpp:584`
- **Status:** [ ] A fixer

```cpp
uint32_t min_frequency = (rx[3] << 24) | (rx[2] << 16) | (rx[1] << 8) | rx[0];
```

`rx[3]` (uint8_t) est promu en `int` (32-bit signe). Si bit 7 est set, `<< 24` shift dans le bit de signe = UB.

**Fix:** `((uint32_t)rx[3] << 24)`.

---

### #25 - `power_off_immediate` appelle le mauvais exit handler
- **Fichier:** `ports/nrf52840/core/hardware/smd_sat/smd_sat.cpp:895-903`
- **Status:** [ ] A fixer

`SMD_STATE_CHANGE(idle, stopped)` appelle toujours `state_idle_exit()` quel que soit l'etat reel.

**Fix:** Ajouter un `switch` sur l'etat courant pour appeler le bon exit handler.

---

### #26 - Max nav check hors du state guard
- **Fichier:** `ports/nrf52840/core/hardware/m10qasync/m10qasync.cpp:375-454`
- **Status:** [ ] A fixer

Le check `GPSEventMaxNavSamples` est evalue en dehors du `while (STATE_EQUAL(receive))`, donc il peut se declencher meme si le receiver n'est plus en etat `receive`.

---

### #27 - `int` traite comme `bool` dans GNSSDetectorService notify_update
- **Fichier:** `core/services/gnss_detector_service.hpp:39-45`
- **Status:** [ ] A fixer

`m_current_state` est `int` (init a -1), passe en `bool` -> `-1` converti en `true`.

---

### #28 - `abs()` au lieu de `std::fabs()` pour la calibration thermistor
- **Fichier:** `ports/nrf52840/core/hardware/thermistor/thermistor.cpp:127`
- **Status:** [ ] A fixer

```cpp
double difference = abs(average_temperature - target_value); // troncature int!
```

**Fix:** `std::fabs(...)`.

---

### #29 - I2C platform callbacks LPS28DFW retournent toujours 0 (succes)
- **Fichier:** `ports/nrf52840/core/hardware/lps28dfw/lps28dfw.cpp:59-90`
- **Status:** [ ] A fixer

`platform_write()` et `platform_read()` retournent toujours 0, meme si `NrfI2C::write/read` echoue. Le driver ST ne detecte jamais les erreurs I2C.

---

### #30 - Race condition dans reschedule/deschedule
- **Fichier:** `core/services/service.cpp:232-280`
- **Status:** [ ] A evaluer

Si un timer callback fire entre `deschedule()` et le nouveau `post_task_prio()`, l'etat du service peut etre inconsistant.

---

### #31 - Migration de config perd tous les params sauf ARGOS IDs
- **Fichier:** `core/configuration/config_store_fs.hpp:238-288`
- **Status:** [ ] A evaluer

Quand le `config_version_code` change, seuls `ARGOS_DECID` et `ARGOS_HEXID` sont recuperes. Tous les autres parametres utilisateur reviennent aux defaults.

---

### #32 - Boot counter modulo check desactive (TODO en production)
- **Fichier:** `ports/nrf52840/main.cpp:530-535`
- **Status:** [ ] A evaluer

Le check modulo est commente avec un TODO. Le device boot pleinement a chaque wakeup TPL5111, gaspillant du flash (serialisation complete a chaque boot).

---

### #33 - EventEmitter::unsubscribe ne supprime pas l'entree (fuite memoire)
- **Fichier:** `core/hardware/events.hpp:13`
- **Status:** [ ] A fixer

`unsubscribe()` met la valeur a `false` mais ne retire pas l'entree du `std::map`. La map croit indefiniment.

**Fix:** Utiliser `m_listeners.erase(&m)` au lieu de `m_listeners[&m] = false`.

---

### #34 - Division par zero possible dans compute_mean_samples
- **Fichier:** `core/services/sensor_service.hpp:83-85`
- **Status:** [ ] A fixer

```cpp
return std::reduce(v.begin(), v.end()) / v.size(); // si v.empty() -> div/0
```

**Fix:** Verifier `v.empty()` avant.

---

## STYLE / QUALITE DE CODE

### #35 - Implementation complete dans un header (dte_handler.hpp)
- **Fichier:** `core/protocol/dte_handler.hpp` (1091 lignes)
- **Status:** [ ] Refactoring

Tout le DTEHandler est implemente dans un .hpp. Chaque TU qui l'inclut recompile 1091 lignes.

---

### #36 - `using namespace` dans des headers
- **Fichiers:** `core/protocol/dte_handler.hpp:22`, `core/protocol/dte_protocol.hpp:19`
- **Status:** [ ] A fixer

`using namespace std::literals::string_literals;` au scope fichier dans des headers pollue le namespace.

---

### #37 - DRY: `PARMR_REQ` et `STATR_REQ` quasi-identiques
- **Fichier:** `core/protocol/dte_handler.hpp:149-205`
- **Status:** [ ] Refactoring

Meme structure, seuls le filtre ('P' vs 'T') et la commande reponse different.

---

### #38 - Pattern null-check SMD repete 4 fois
- **Fichier:** `core/protocol/dte_handler.hpp:564, 651, 682, 744`
- **Status:** [ ] Refactoring

```cpp
if (!smd_sat_instance) {
    DEBUG_ERROR("...");
    return DTEEncoder::encode(DTECommand::XXX_RESP, (int)DTEError::INCORRECT_DATA);
}
```

---

### #39 - Faute de frappe: `LB_TRESHOLD` au lieu de `LB_THRESHOLD`
- **Fichier:** `core/protocol/dte_params.cpp:34`
- **Status:** [ ] A evaluer (breaking change protocole)

---

### #40 - FIXME params non implementes avec cles `XXX`
- **Fichier:** `core/protocol/dte_params.cpp:13,24`
- **Status:** [ ] A resoudre ou supprimer

```cpp
{ "AOP_STATUS", "XXX01", ... },     // FIXME: missing parameter key
{ "GPS_CONST_SELECT", "XXX02", ... }, // FIXME: missing parameter key
```

---

### #41 - Code de securite hardcode `0x12345678`
- **Fichier:** `core/protocol/dte_handler.hpp:228`
- **Status:** [ ] A evaluer

---

### #42 - `va_list` C-style au lieu de variadic templates
- **Fichier:** `core/protocol/dte_protocol.hpp:623`
- **Status:** [ ] Refactoring long terme

Type-unsafe. Si un `int` est passe ou un `unsigned int` est attendu = UB.

---

### #43 - Double `#include "debug.hpp"`
- **Fichier:** `ports/nrf52840/main.cpp:36,38`
- **Status:** [ ] A fixer

---

### #44 - Enum duplique: `FIXTYPE_NO = FIXTYPE_DEAD_RECKONING_ONLY = 0`
- **Fichier:** `ports/nrf52840/core/hardware/m10qasync/ubx.hpp:848-849`
- **Status:** [ ] A fixer

`FIXTYPE_DEAD_RECKONING_ONLY` devrait etre `1` (u-blox spec).

---

### #45 - Fonctions `static` dans header -> warnings "unused"
- **Fichier:** `core/protocol/base_types.hpp:367-577`
- **Status:** [ ] A fixer

Utiliser `inline` au lieu de `static` pour les fonctions definies dans les headers.

---

### #46 - Pointeurs globaux vers des objets stack-locaux
- **Fichier:** `ports/nrf52840/main.cpp`
- **Status:** [ ] A evaluer

Pattern fragile -- fonctionne car `main()` ne retourne jamais, mais dangereux si refactoring futur.

---

## RESUME STATISTIQUE

| Severite  | Nombre | Corrige |
|-----------|--------|---------|
| CRITIQUE  | 4      | 0       |
| HIGH      | 15     | 0       |
| MEDIUM    | 15     | 0       |
| LOW/STYLE | 12     | 0       |
| **TOTAL** | **46** | **0**   |

## TOP 5 ACTIONS PRIORITAIRES POUR RSPB

1. **#1** - Fixer le buffer overflow DFU satellite
2. **#2** - Fixer m_samples[5] -> m_samples[6] (ou dynamique)
3. **#3** - Fixer l'operator== de BasePassPredict
4. **#4** - Fixer le bypass securite SECUR
5. **#14** - Ajouter la gestion d'erreur dans LPS28DFW (eviter fausses detections surface)
