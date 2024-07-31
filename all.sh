#!/bin/bash

# Define the name of the tmux session
SESSION_NAME="scmhitt"

# Define the scripts to be run in each window
SCRIPT1="./scripts/start_bluesim.sh"
SCRIPT2="./scripts/start_mavlink2rest.sh"
SCRIPT3="./scripts/start_mapping.sh"
# SCRIPT4="/path/to/your/script4.sh"

# Start a new tmux session with the first window
tmux new-session -d -s $SESSION_NAME

# Enable mouse support
tmux setw -g mouse on

# Split the first window into four panes
tmux split-window -h
tmux split-window -v
tmux select-pane -t 0
tmux split-window -v

# Run the scripts in each pane
tmux send-keys -t 0 "bash $SCRIPT1" C-m
tmux send-keys -t 1 "bash $SCRIPT2" C-m
tmux send-keys -t 2 "bash $SCRIPT3" C-m
# tmux send-keys -t 3 "bash $SCRIPT4" C-m

# Attach to the tmux session
tmux attach-session -t $SESSION_NAME
