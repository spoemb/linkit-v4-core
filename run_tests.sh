#!/bin/bash
# Script pour exécuter les tests unitaires manuellement

set -e

# Couleurs pour l'affichage
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Répertoires
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TESTS_SRC_DIR="$PROJECT_DIR/tests"
BUILD_DIR="$TESTS_SRC_DIR/build"
TEST_EXECUTABLE="$BUILD_DIR/TrackerTests"

echo -e "${BLUE}╔════════════════════════════════════════╗${NC}"
echo -e "${BLUE}║   Tests Unitaires LinkIt V4 Core      ║${NC}"
echo -e "${BLUE}╚════════════════════════════════════════╝${NC}"
echo ""

# Fonction pour afficher les erreurs
error_exit() {
    echo -e "${RED}❌ $1${NC}" >&2
    exit 1
}

# Options
CLEAN_BUILD=false
VERBOSE=false

# Parser les arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -c|--clean)
            CLEAN_BUILD=true
            shift
            ;;
        -v|--verbose)
            VERBOSE=true
            shift
            ;;
        -h|--help)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  -c, --clean     Nettoyer et reconstruire complètement"
            echo "  -v, --verbose   Mode verbeux pour les tests"
            echo "  -h, --help      Afficher cette aide"
            exit 0
            ;;
        *)
            echo -e "${YELLOW}Option inconnue: $1${NC}"
            exit 1
            ;;
    esac
done

# Nettoyer si demandé
if [ "$CLEAN_BUILD" = true ]; then
    echo -e "${YELLOW}🧹 Nettoyage du répertoire build...${NC}"
    rm -rf "$BUILD_DIR"
fi

# Créer le répertoire build s'il n'existe pas
if [ ! -d "$BUILD_DIR" ]; then
    echo "📁 Création du répertoire build..."
    mkdir -p "$BUILD_DIR"
fi

cd "$BUILD_DIR" || error_exit "Impossible de naviguer vers $BUILD_DIR"

# Configuration CMake si nécessaire
if [ ! -f "CMakeCache.txt" ]; then
    echo "⚙️  Configuration CMake..."
    cmake "$TESTS_SRC_DIR" -DCMAKE_BUILD_TYPE=Debug || error_exit "Échec de la configuration CMake"
fi

# Compilation des tests
echo "🔨 Compilation des tests..."
START_TIME=$(date +%s)

if make -j$(nproc); then
    END_TIME=$(date +%s)
    COMPILE_TIME=$((END_TIME - START_TIME))
    echo -e "${GREEN}✅ Compilation réussie en ${COMPILE_TIME}s${NC}"
else
    echo -e "${RED}❌ Échec de la compilation${NC}"
    exit 1
fi

# Exécution des tests
echo ""
echo -e "${BLUE}════════════════════════════════════════${NC}"
echo -e "${BLUE}   Exécution des tests unitaires${NC}"
echo -e "${BLUE}════════════════════════════════════════${NC}"
echo ""

if [ -f "$TEST_EXECUTABLE" ]; then
    START_TIME=$(date +%s)

    # Exécuter les tests avec ou sans mode verbeux
    if [ "$VERBOSE" = true ]; then
        TEST_OUTPUT=$("$TEST_EXECUTABLE" -v 2>&1)
        TEST_RESULT=$?
    else
        TEST_OUTPUT=$("$TEST_EXECUTABLE" 2>&1)
        TEST_RESULT=$?
    fi

    END_TIME=$(date +%s)
    TEST_TIME=$((END_TIME - START_TIME))

    # Afficher la sortie
    echo "$TEST_OUTPUT"

    echo ""
    echo -e "${BLUE}════════════════════════════════════════${NC}"

    # Vérifier le résultat
    if [ $TEST_RESULT -eq 0 ]; then
        # Extraire les statistiques
        TESTS_RUN=$(echo "$TEST_OUTPUT" | grep -oP '\d+(?= tests run)' || echo "?")
        FAILURES=$(echo "$TEST_OUTPUT" | grep -oP '\d+(?= failures)' || echo "0")

        echo -e "${GREEN}✅ Tous les tests sont passés avec succès!${NC}"
        echo -e "${GREEN}   Tests exécutés: $TESTS_RUN${NC}"
        echo -e "${GREEN}   Durée: ${TEST_TIME}s${NC}"
        exit 0
    else
        echo -e "${RED}❌ Certains tests ont échoué!${NC}"
        echo -e "${RED}   Durée: ${TEST_TIME}s${NC}"
        exit 1
    fi
else
    echo -e "${RED}❌ Exécutable de tests non trouvé${NC}"
    exit 1
fi
