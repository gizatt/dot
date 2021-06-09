
rosserial_setup:
	rosrun rosserial_arduino make_libraries.py src/due/build/

due_build: rosserial_setup
	arduino-cli compile --library $(realpath src/due/build/ros_lib) --fqbn arduino:sam:arduino_due_x_dbg -e src/due
due_flash:
	arduino-cli upload -p $(port) --fqbn arduino:sam:arduino_due_x_dbg -v src/due
due_reset:
	stty -F $(port) 1200
	sleep 1
due_update: rosserial_setup due_build due_reset due_flash

catkin:
	cd catkin && touch src/dot_msgs/CMakeLists.txt && catkin_make

sdf:
	xacro4sdf ./models/dot_control.sdf.xmacro

.PHONY: rosserial_setup catkin
