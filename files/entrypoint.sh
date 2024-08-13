#!/bin/bash

echo "Starting.."
[ ! -e /var/run/nginx.pid ] && nginx&

# Create a new tmux session
tmux -f /etc/tmux.conf start-server
tmux new -d -s "BSLAMP"

# Split the screen into a 2x2 matrix
tmux split-window -v
tmux split-window -h
tmux select-pane -t 0
tmux split-window -h

tmux send-keys -t 0 "ls" Enter
tmux send-keys -t 1 "ls" Enter
tmux send-keys -t 2 "ls" Enter
tmux send-keys -t 3 "ls" Enter

function create_service {
    tmux new -d -s "$1" || true
    SESSION_NAME="$1:0"
    tmux send-keys -t $SESSION_NAME "$2" C-m
}

create_service 'ttyd' 'ttyd -p 89 sh -c "/usr/bin/tmux attach -t BSLAMP || /usr/bin/tmux new -s user_terminal"'

echo "Done!"
sleep infinity