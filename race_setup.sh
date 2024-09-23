#!/bin/bash

SESH="tmux_session"

# Check for a valid directory provided
if [ -d $1 ] && [ ! -z "$1" ]; then
    # Directory valid and not null
    # Check to see if a session already exist.
    tmux has-session -t $SESH 2>/dev/null
    if [ $? != 0 ]; then
        # Found a valid directory, but no session, create one
        tmux new-session -d -s $SESH -n "nodes"

        # cd into the project directory
        tmux send-keys -t $SESH:nodes "cd $1" C-m
        # To connect to simulation
        tmux send-keys -t $SESH:nodes "echo http://localhost:8080/vnc.html" C-m
        tmux send-keys -t $SESH:nodes 'echo "Split Panels ctrl + B % (Vertical) or \" (Horizontal)"' C-m

        # Set colors for current session
        tmux set-window-option -t $SESH window-status-style fg=cyan,bg=black
        tmux set-window-option -t $SESH window-status-current-style fg=white

        ########## Create session for the simulation ##########
        tmux new-window -t $SESH -n "simulation"
        tmux send-keys -t $SESH:simulation "cd src" C-m     # Only go into the src folder
        
        # We will source here so that the lab can be started
        tmux send-keys -t $SESH:simulation "source /opt/ros/foxy/setup.bash" C-m
        tmux send-keys -t $SESH:simulation "source install/setup.bash" C-m

        # Now run the simulation
        tmux send-keys -t $SESH:simulation 'export PS1="(sim-1) $PS1"' C-m
        tmux send-keys -t $SESH:simulation "ros2 launch f1tenth_gym_ros gym_bridge_launch.py" C-m

        # Set color for current session
        tmux set-window-option -t $SESH window-status-style fg=cyan,bg=black
        tmux set-window-option -t $SESH window-status-current-style fg=white

        ########## Create window for keyboard ##########
        tmux new-window -t $SESH -n "keyboard"
        # Don't need to go to project directory for this one

        # Source so that ros2 will work
        tmux send-keys -t $SESH:keyboard "source /opt/ros/foxy/setup.bash" C-m
        tmux send-keys -t $SESH:keyboard "source install/setup.bash" C-m

        tmux send-keys -t $SESH:keyboard "ros2 run teleop_twist_keyboard teleop_twist_keyboard" C-m

        ### Attempt to create a split window for tmux. Not complete ###
        # tmux split-window -h -t $SESH:nodes
        # tmux send-keys -t 0 "source /opt/ros/foxy/setup.bash" C-m
        # tmux send-keys -t 0 "source install/setup.bash" C-m

        # tmux send-keys -t $SESH:keyboard "ros2 run teleop_twist_keyboard teleop_twist_keyboard" C-m

        ########## Color terminal ##########
        tmux set-option -t $SESH status on
        tmux set-option -t $SESH status-style fg=white,bg=black
        tmux set-option -t $SESH status-left "#[fg=green]Session: #[fg=yellow]"
        tmux set-option -t $SESH status-left-length 40
        tmux set-option -t $SESH status-right "#[fg=cyan]%d %b %R"

        # Set color for current session
        tmux set-window-option -t $SESH window-status-style fg=cyan,bg=black
        tmux set-window-option -t $SESH window-status-current-style fg=white

        # Focus on the launch nodes session first
        tmux select-window -t $SESH:nodes
    fi

    # If it does, just reattach
    tmux attach-session -t $SESH
else
    echo "Please provide a valid directory for the project"
fi

