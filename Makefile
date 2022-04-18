
# Sentinel only rebuilds the message types when the files have changed.
build/rosserial_setup.sentinel: $(shell find catkin/src/dot_msgs -type f)
	mkdir -p build
	touch build/rosserial_setup.sentinel
rosserial_setup: build/rosserial_setup.sentinel
	rosrun rosserial_arduino make_libraries.py build/tmp catkin/src/dot_msgs
	rm -rf platformio/include/dot_msgs && cp -r build/tmp/ros_lib/dot_msgs platformio/include/dot_msgs

speck_build: build/rosserial_setup.sentinel
	cd platformio && pio run
speck_flash: build/rosserial_setup.sentinel
	cd platformio && pio run --target upload
speck_update: speck_flash

catkin:
	cd catkin && touch src/dot_msgs/CMakeLists.txt && catkin_make

sdf:
	xacro4sdf ./models/dot_control.sdf.xmacro

.PHONY: rosserial_setup catkin
