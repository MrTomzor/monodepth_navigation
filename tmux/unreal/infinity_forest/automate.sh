#!/bin/bash

# Number of experiment runs
NUM_RUNS=5

# This "trap" ensures that if you press Ctrl+C to stop this script,
# it will also kill the current simulation session before exiting.
trap "echo -e '\n[!] Automation interrupted. Killing session...'; tmux -L mrs kill-session -t simulation; exit" INT

for i in $(seq 1 $NUM_RUNS)
do
    echo "========================================"
    echo "STARTING EXPERIMENT #$i OF $NUM_RUNS"
    echo "========================================"

    # 1. Cleanup
    rm -f /tmp/mission_done
    # Kill old processes to prevent Gazebo from "ghosting"
    killall -9 gzserver gzclient 2>/dev/null

    # 2. Launch simulation in a NEW terminal window
    # We use gnome-terminal to execute your start script.
    # The new window will attach to the tmux session automatically.
    gnome-terminal --window --title="Experiment #$i" -- bash -c "./start.sh"

    echo "Simulation window opened. Waiting for /tmp/mission_done flag..."

    # 3. Wait loop for evaluator.py to finish
    while [ ! -f /tmp/mission_done ]
    do
        sleep 5

        # Safety check: if the session was closed manually, stop waiting
        if ! tmux -L mrs has-session -t simulation 2>/dev/null; then
            echo "[!] Simulation session was closed manually or crashed."
            break
        fi
    done

    if [ -f /tmp/mission_done ]; then
        echo "Mission #$i SUCCESS! Closing session..."
    fi

    # 4. Kill the tmux session via the 'mrs' socket
    tmux -L mrs kill-session -t simulation

    # 5. Pause for full memory and port cleanup
    echo "System cooldown (15 sec)..."
    sleep 15
done

echo "All $NUM_RUNS experiments finished successfully!"