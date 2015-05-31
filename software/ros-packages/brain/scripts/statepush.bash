#!/bin/bash
echo "1 is detect, 2 is incomplete, 3 is complete, 4 is detect obs, 5 is time expired"
echo "6 is avoid, 7 is find, 8 is rec, 9 is path";
while true; do
	read state
	if [ "$state" == "1" ]; then 
	rostopic pub /brain/state_change std_msgs/String "Detect Object" -1
	fi
	if [ "$state" == "2" ]; then 
	rostopic pub /brain/state_change std_msgs/String "Incomplete Objective" -1
	fi
	if [ "$state" == "3" ]; then 
	rostopic pub /brain/state_change std_msgs/String "Objective Complete" -1
	fi
	if [ "$state" == "4" ]; then 
	rostopic pub /brain/state_change std_msgs/String "Detect Obstacle" -1
	fi
	if [ "$state" == "5" ]; then 
	rostopic pub /brain/state_change std_msgs/String "Time Expired" -1
	fi
	if [ "$state" == "6" ]; then 
	rostopic pub /motor/commands/avoid_obstacle std_msgs/String "f10" -1
	fi
	if [ "$state" == "7" ]; then 
	rostopic pub /motor/commands/find_base std_msgs/String "f10" -1
	fi
	if [ "$state" == "8" ]; then 
	rostopic pub /motor/commands/object_recognition std_msgs/String "f10" -1
	fi
	if [ "$state" == "9" ]; then 
	rostopic pub /motor/commands/path_finding std_msgs/String "f10" -1
	fi
done




 

