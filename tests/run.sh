#!/bin/sh
# Banc de simulation du sketch sur PC (temps accéléré, entrées/sorties modélisées).
#   tests/run.sh             -> tous les scénarios
#   tests/run.sh nofire      -> un seul scénario (ECHO=1 pour voir le journal série)
set -e
cd "$(dirname "$0")"
SK=../BoilerWithNextionLatest
mkdir -p build
# Même assemblage que l'IDE Arduino : fichier principal, puis les autres .ino par ordre alphabétique
{
  echo "#line 1 \"$SK/BoilerWithNextionLatest.ino\""
  cat "$SK/BoilerWithNextionLatest.ino"
  for f in $(cd "$SK" && ls *.ino | LC_ALL=C sort); do
    [ "$f" = BoilerWithNextionLatest.ino ] && continue
    echo "#line 1 \"$SK/$f\""
    cat "$SK/$f"
  done
} > build/sketch_all.cpp
${CXX:-c++} -std=c++17 -O2 -Wall -Wno-unused-function -Imock -I"$SK" -o build/sim sim.cpp
./build/sim "$@"
