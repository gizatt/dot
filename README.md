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

Currently fleshing out a trajectory-spooling server on the Arduino side. After switching to a Teensy 4.1, I've been able to get to 500KB/s trasmit (simply sending 1KB chunks as fast as possible, so they send at 500hz); it seems like the advertised as-fast-as-possible serial line is working, and is indeed fast. (I still do need to customize the node handle, which I'm doing with a surrogate `ros.h`.) For a 16-point 12-DOF trajectory of floats (say ~200 bytes), that's 2.5khz. That said, there is significant sending overhead; sending a 100byte message as fast as possible sees 40KB/s instead (also 500hz), so I may be saturating my send rate somehow. Either way, 500hz is plenty.

TODO: Delay floor?

# Deps

## Basic dependencies.

python3.6

http://wiki.ros.org/noetic/Installation/Ubuntu
`sudo apt install ros-noetic-rosserial-arduino bossa-cli`
`pip3 install panel`

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

## Teensy + Arduino + rosserial

# PlatformIO route:
```
python3 -c "$(curl -fsSL https://raw.githubusercontent.com/platformio/platformio/master/scripts/get-platformio.py)"
export PATH=$PATH:$HOME/.platformio/penv/bin # Put in bashrc
```

Build + upload:
```
pio run
pio run --target upload
```