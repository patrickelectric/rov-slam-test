#!/bin/bash

cd bluesim

ARGUMENTS="--brpool"
# Define possible Godot binary locations
GODOT_BINARIES_LINUX=(
  "/usr/local/bin/godot"
  "/usr/bin/godot"
  "$HOME/bin/godot"
  "$HOME/.local/bin/godot"
)

GODOT_BINARIES_MACOS=(
  "/Applications/Godot.app/Contents/MacOS/Godot"
  "$HOME/Applications/Godot.app/Contents/MacOS/Godot"
)

# Function to find and run Godot binary
find_and_run_godot() {
  local binaries=("$@")
  for bin in "${binaries[@]}"; do
    if [ -x "$bin" ]; then
      echo "Found Godot binary: $bin"
      "$bin" $ARGUMENTS &
      exit 0
    fi
  done
  echo "Godot binary not found."
  exit 1
}

# Check the OS and set the appropriate binary locations
if [[ "$OSTYPE" == "linux-gnu"* ]]; then
  find_and_run_godot "${GODOT_BINARIES_LINUX[@]}"
elif [[ "$OSTYPE" == "darwin"* ]]; then
  find_and_run_godot "${GODOT_BINARIES_MACOS[@]}"
else
  echo "Unsupported OS: $OSTYPE"
  exit 1
fi
