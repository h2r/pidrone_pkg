#!/bin/sh
# make a new session and first window
tmux new-session -d -s 'Drone' -n 'roscore'
tmux send-keys "source setup.sh" ^M
tmux send-keys "roscore" ^M

# make flight window
tmux new-window -n 'flight'
tmux send-keys "source setup.sh" ^M
tmux send-keys "cd $(rospack find pidrone_pkg)/scripts" ^M
tmux send-keys "python state_controller.py"

# make vision window
tmux new-window -n 'vision'
tmux send-keys "source setup.sh" ^M
tmux send-keys "cd $(rospack find pidrone_pkg)/scripts" ^M
tmux send-keys "python flow_pub_transform.py" ^M

# make infrared window
tmux new-window -n 'infrared'
tmux send-keys "source setup.sh" ^M
tmux send-keys "cd $(rospack find pidrone_pkg)/scripts" ^M
tmux send-keys "python infrared_pub.py" ^M

# make joy window
tmux new-window -n 'joy_node'
tmux send-keys "source setup.sh" ^M
tmux send-keys "cd $(rospack find pidrone_pkg)/scripts" ^M
tmux send-keys "python joy_translation.py" ^M

# make rosbridge window
tmux new-window -n 'joy_node'
tmux send-keys "source setup.sh" ^M
tmux send-keys "sleep 10s && roslaunch rosbridge_server rosbridge_websocket.launch" ^M

# make web_video_server window
tmux new-window -n 'web_video_server'
tmux send-keys "source setup.sh" ^M
tmux send-keys "rosrun web_video_server web_video_server" ^M

# make free window
tmux new-window -n 'free'
tmux send-keys "source setup.sh" ^M

# attach to tmux session
tmux attach-session -d
