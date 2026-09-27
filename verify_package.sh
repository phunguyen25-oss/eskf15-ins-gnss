#!/usr/bin/env sh
set -eu
ROOT=$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)
make -C "$ROOT/stm32/tests/native" test
if rg -n 'malloc\(|calloc\(|realloc\(|free\(' "$ROOT/stm32/Inc" "$ROOT/stm32/Src"; then
  echo "Dynamic allocation found in estimator path." >&2
  exit 1
fi
echo "Static checks passed. Run matlab/run_all_tests.m inside MATLAB for the desktop suite."
