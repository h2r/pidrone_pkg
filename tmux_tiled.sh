# make a new session and name window
tmux new-session -d -s 'drone' -n 'drone'

# set up roscore
tmux send-keys "source setup.sh" ^M
tmux send-keys "roscore" ^M

# set up vision
tmux split-window -h
tmux send-keys "source setup.sh" ^M
tmux send-keys "cd $(rospack find pidrone_pkg)/scripts" ^M
tmux send-keys "python flow_pub_transform.py" ^M

# set up infrared
tmux split-window -h
tmux send-keys "source setup.sh" ^M
tmux send-keys "cd $(rospack find pidrone_pkg)/scripts" ^M
tmux send-keys "python infrared_pub.py" ^M

# set up joy node
tmux split-window -h
tmux send-keys "source setup.sh" ^M
tmux send-keys "cd $(rospack find pidrone_pkg)/scripts" ^M
tmux send-keys "python joy_translation.py" ^M

# set up rosbrigde
tmux select-pane -t 1
tmux split-window -v
tmux send-keys "source setup.sh" ^M
tmux send-keys "sleep 10s && roslaunch rosbridge_server rosbridge_websocket.launch" ^M

# set up web video server
tmux select-pane -t 3
tmux split-window -v
tmux send-keys "source setup.sh" ^M
tmux send-keys "rosrun web_video_server web_video_server" ^M

# set up flight
tmux select-pane -t 5
tmux split-window -v
tmux send-keys "source setup.sh" ^M
tmux send-keys "cd $(rospack find pidrone_pkg)/scripts" ^M
tmux send-keys "python state_controller.py"

# set up second free pane
tmux select-pane -t 7
tmux split-window -v
tmux send-keys "source setup.sh" ^M

tmux select-layout tiled

tmux attach-session -d
