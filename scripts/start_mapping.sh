#!/bin/bash

# Define the name of the virtual environment directory
VENV_DIR="venv"

# Check if the virtual environment directory exists
if [ -d "$VENV_DIR" ]; then
    echo "Activating the virtual environment..."
    source "$VENV_DIR/bin/activate"
else
    echo "Creating a virtual environment..."
    python3 -m venv "$VENV_DIR"

    echo "Activating the virtual environment..."
    source "$VENV_DIR/bin/activate"

    # Check if requirements.txt exists and install packages
    if [ -f "requirements.txt" ]; then
        echo "Installing requirements..."
        pip install -r requirements.txt
    else
        echo "requirements.txt not found. No packages installed."
    fi
fi

# Inform the user that the environment is ready
echo "Virtual environment is ready."

# Run the main.py script within the activated virtual environment
python navigation/main.py --vehicle tcp://10.40.1.129:5760
