# Tests Unitaires - LinkIt V4 Core

Ce document explique comment utiliser les tests unitaires et le système de validation automatique par commits.

## 📋 Table des matières

- [Exécution manuelle des tests](#exécution-manuelle-des-tests)
- [Hook Pre-Commit](#hook-pre-commit)
- [Structure des tests](#structure-des-tests)
- [Adaptation après suppression d'Artic](#adaptation-après-suppression-dartic)

## 🧪 Exécution manuelle des tests

### Script rapide

Le moyen le plus simple d'exécuter les tests est d'utiliser le script `run_tests.sh`:

```bash
./run_tests.sh
```

### Options disponibles

```bash
./run_tests.sh --help         # Afficher l'aide
./run_tests.sh --clean        # Nettoyer et reconstruire complètement
./run_tests.sh --verbose      # Mode verbeux pour les tests
```

### Compilation manuelle

Si vous préférez compiler et exécuter manuellement:

```bash
# Créer le répertoire build pour les tests
mkdir -p tests/build && cd tests/build

# Configurer CMake
cmake .. -DCMAKE_BUILD_TYPE=Debug

# Compiler les tests
make -j$(nproc)

# Exécuter les tests
./CLSGenTrackerTests -v
```

## 🔒 Hook Pre-Commit

### Fonctionnement

Un hook Git pre-commit est automatiquement configuré dans `.git/hooks/pre-commit`.

**À chaque commit, le hook:**
1. ✅ Compile automatiquement les tests
2. ✅ Exécute tous les tests unitaires
3. ✅ Bloque le commit si des tests échouent

### Bypass du hook (déconseillé)

Si vous devez absolument commiter même avec des tests échouants:

```bash
git commit --no-verify -m "message"
```

⚠️ **Attention:** Utiliser cette option peut introduire des régressions. À utiliser avec précaution!

### Désactiver le hook

Si vous souhaitez désactiver temporairement le hook:

```bash
chmod -x .git/hooks/pre-commit
```

Pour le réactiver:

```bash
chmod +x .git/hooks/pre-commit
```

## 📁 Structure des tests

```
tests/
├── CMakeLists.txt              # Configuration CMake des tests
├── mocks/
│   ├── mock_kineis_device.hpp  # Mock pour KineisDevice
│   ├── mock_sensor.hpp
│   ├── mock_logger.hpp
│   └── ...
├── fakes/
│   ├── fake_rtc.hpp
│   ├── fake_config_store.hpp
│   └── ...
└── src/
    ├── argos_tx_test.cpp       # Tests service TX Argos/Kineis
    ├── argos_test.cpp           # Tests scheduler Argos
    ├── dte_handler_test.cpp    # Tests protocole DTE
    ├── main.cpp                 # Point d'entrée des tests
    └── ...
```

## 🔄 Adaptation après suppression d'Artic

### Changements effectués

Les tests ont été adaptés pour fonctionner avec le nouveau système KIM2/Kineis:

#### 1. Remplacement des mocks

- `MockArticDevice` → `MockKineisDevice`
- Nouveau fichier: `tests/mocks/mock_kineis_device.hpp`

#### 2. Mise à jour des types

- `ArticMode` → `KineisModulation`
  - `ArticMode::A2` → `KineisModulation::LDA2`
  - `ArticMode::A3` → `KineisModulation::LDA2`
  - `ArticMode::A4` → `KineisModulation::VLDA4`

- `ArticEventTxComplete` → `KineisEventTxComplete`

#### 3. Méthodes supprimées

Les appels à `set_tx_power()` ont été supprimés car cette méthode n'est plus utilisée dans le nouveau système.

### Fichiers de tests mis à jour

1. **argos_tx_test.cpp**
   - Utilise maintenant `MockKineisDevice`
   - Modes de modulation mis à jour

2. **dte_handler_test.cpp**
   - Suppression des références à `artic_device`
   - `DTEHandler` n'hérite plus de `ArticEventListener`

3. **tests/src/main.cpp**
   - Suppression de `extern ArticDevice *artic_device;`

## 📊 Interprétation des résultats

### Succès

```
✅ Tous les tests sont passés avec succès!
   Tests exécutés: 42
   Durée: 3s
```

### Échec

```
❌ Certains tests ont échoué!

Failures in TEST(ArgosTxService, test_name)
	/path/to/test.cpp:123: error: Expected <...> but was <...>

FAILURES!!! (42 tests, 1 failures, 0 ignored)
```

## 🛠️ Dépannage

### Les tests ne compilent pas

1. Vérifier que CMake est configuré avec `-DENABLE_TESTS=ON`
2. Nettoyer et reconstruire: `./run_tests.sh --clean`
3. Vérifier les dépendances CppUTest

### Le hook ne s'exécute pas

1. Vérifier les permissions: `ls -la .git/hooks/pre-commit`
2. Rendre exécutable si nécessaire: `chmod +x .git/hooks/pre-commit`

### Les tests échouent après modification

1. Vérifier que les mocks sont à jour
2. S'assurer que les interfaces correspondent
3. Exécuter en mode verbeux: `./run_tests.sh --verbose`

## 📚 Ressources

- [CppUTest Documentation](http://cpputest.github.io/)
- [Git Hooks Documentation](https://git-scm.com/book/en/v2/Customizing-Git-Git-Hooks)

## 🤝 Contribution

Lors de l'ajout de nouvelles fonctionnalités:

1. ✅ Écrire les tests unitaires correspondants
2. ✅ S'assurer que tous les tests passent
3. ✅ Le hook pre-commit validera automatiquement avant commit
4. ✅ Documenter les nouveaux mocks si nécessaires
