# Dot (tiny Spot)

# Running

Necessary parts:
1) `roscore`
2) `rosrun rosserial_arduino serial_node.py _port:=/dev/ttyACM0 _baud:=115200` (may need to `sudo chmod 666 /dev/ttyACM0`)
3) e.g. interface `panel serve teleop_panel.py --allow-websocket-origin=192.168.0.142:5006`

Flash with `make due_update port=/dev/ttyACM0`.

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

`arduino-cli lib install "Rosserial Arduino Library"`
`arduino-cli lib install "Adafruit PWM Servo Driver Library"`

# Ref
http://wiki.ros.org/rosserial_arduino/Tutorials/Arduino%20IDE%20Setup