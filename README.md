# Dot (tiny Spot)

# Running

Necessary parts:
1) `roscore`
2) `rosrun rosserial_arduino serial_node.py _port:=/dev/ttyACM0 _baud:=250000` (may need to `sudo chmod 666 /dev/ttyACM0`)
3) e.g. interface `panel serve teleop_panel.py --allow-websocket-origin=192.168.0.142:5006`

Flash with `make due_update port=/dev/ttyACM0`.

Fun demos:
- `python locomotion_server.py`
- `python test_locomotion_com_commands.py -com_rel 0.0 0.0 0.25 -step 0.005` smooth moving to com positions
- `python test_locomotion_com_commands.py -com_spiral 1. 1.5 0.75 0.00 0.025 0.01 1.57 0. 0. -step 0.05` side to side bouncing
- `python test_locomotion_com_commands.py -com_spiral 5. 5. 3. 0.025 0.025 0.01 1.57 0. 0. -step 0.05` circles

## Notes

Currently fleshing out a trajectory-spooling server on the Arduino side. The primary limitation appears to be communication speed; I can run the Due up to 250000 baud but above that seems to be no go. (See https://forum.arduino.cc/t/arduino-due-serial-speed/129950/10.) With that rate, it's taking ~100ms to transit 16-point trajectories; borderline acceptable, but slow and important to be aware of. I'll need to trust the next level of motion generation code to produce those trajectories at ~10hz, and only transit a handful of points at a time, I think.

The delay floor (i.e. delay when sending empty trajectories, calculated by taking rostime - message timestamp at handling time on Arduino side) seems to be ~15ms. This may be the "empty" size of that type, or could be other parts of the system. Either way, trajectories of length ~4 seem like a reasonable compromise at 30-40ms. Trajectories of length 2 only shave of another 10ms from there.

To get those messages to transmit correctly, I need to modify `ros_lib/ros/node_handle.h` increase the INPUT_SIZE (I use 8192, as the Due has tons of RAM) and SERIAL_MSG_TIMEOUT (I use 500, to be super safe). I also decrease MAX_PUBLISHERS and MAX_SUBSCRIBERS to compensate for the extra memory consumption, since I know the number of channels we use. This patch is applied automatically after the Arduino ROS C files are autogenerated. (Maybe relevant: https://github.com/ros-drivers/rosserial/pull/338.)

# Deps

## Basic dependencies.

python3.6

http://wiki.ros.org/noetic/Installation/Ubuntu
`sudo apt install ros-noetic-rosserial-arduino bossa-cli`
`pip3 install panel`

## Udev

TODO


## Arm cross-compiler

Broken on apt...
https://itectec.com/ubuntu/ubuntu-how-to-install-arm-none-eabi-gdb-on-ubuntu-20-04-lts-focal-fossa/

```
wget https://armkeil.blob.core.windows.net/developer/Files/downloads/gnu-rm/9-2019q4/gcc-arm-none-eabi-9-2019-q4-major-aarch64-linux.tar.bz2
sudo tar xjvf [name] -C /usr/share/
export VERSION_STRING=gcc-arm-none-eabi-9-2019-q4-major
sudo ln -f -s /usr/share/$VERSION_STRING/bin/arm-none-eabi-gcc /usr/bin/arm-none-eabi-gcc 
sudo ln -f -s /usr/share/$VERSION_STRING/bin/arm-none-eabi-g++ /usr/bin/arm-none-eabi-g++
sudo ln -f -s /usr/share/$VERSION_STRING/bin/arm-none-eabi-gdb /usr/bin/arm-none-eabi-gdb
sudo ln -f -s /usr/share/$VERSION_STRING/bin/arm-none-eabi-gdb /usr/bin/arm-none-eabi-size
sudo apt install libncurses-dev
sudo ln -s /usr/lib/aarch64-linux-gnu/libncurses.so.6 /usr/lib/aarch64-linux-gnu/libncurses.so.5
sudo ln -s /usr/lib/aarch64-linux-gnu/libtinfo.so.6 /usr/lib/aarch64-linux-gnu/libtinfo.so.5
```

Confirm with
```
arm-none-eabi-gcc --version
arm-none-eabi-g++ --version
arm-none-eabi-gdb --version
arm-none-eabi-size --version
```

Install bossac and get it on path:
```
wget https://github.com/shumatech/BOSSA/archive/refs/tags/1.6.1-arduino.zip && unzip 1.6.1-arduino.zip
cd BOSSA-1.6.1-arduino && ./arduino/make_package.sh
sudo cp bin/bossac /usr/local/bin/bossac
```


## Arduino-cli
Install:
```
wget https://downloads.arduino.cc/arduino-cli/arduino-cli_latest_Linux_ARM64.tar.gz
tar -xf arduino-cli_latest_Linux_ARM64.tar.gz arduino-cli
rm arduino-cli_latest_Linux_ARM64.tar.gz
sudo mv arduino-cli /usr/local/bin
```

Setup following https://arduino.github.io/arduino-cli/latest/getting-started/:
```
arduino-cli config init
arduino-cli core update-index
```

Go into `~/.arduino15/package_index.json`, find the entry for `arduino:sam`, and remove
its dependencies -- we'll have to supply them ourselves if they're not available.

Then
```
arduino-cli core install arduino:sam@1.6.12 -v
```

Edit `~/.arduino15/packages/arduino/hardware/sam/1.6.12/platform.txt` and change `compiler.path`
to something like `compiler.path=/usr/share/gcc-arm-none-eabi-9-2020-q2-update/bin/`.

Check that it works:
```
arduino-cli sketch new /tmp/TestSketch
arduino-cli compile --fqbn arduino:sam:arduino_due_x_dbg -e /tmp/TestSketch
arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:sam:arduino_due_x_dbg /tmp/TestSketch
```

TODO: make a custom Due device library for arduino-cli so this stuff happens automatically.

## Arduino libraries

`arduino-cli lib install "Adafruit PWM Servo Driver Library"`

# Ref
http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup