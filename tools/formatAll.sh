#!/bin/bash

. "$(dirname "$0")/isTopLevel.sh"

for f in $(git ls-files | grep -E '.*\.(cpp|hpp)$'); do
  echo "formatting ${f}"
  clang-format -i "${f}"
done
