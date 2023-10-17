#!/bin/bash
sudo stty -parenb -F /dev/ttyUSB0 921600 cs8 -cstopb raw
#921600
sudo cat /dev/ttyUSB0
#sudo screen /dev/ttyACM0 921600
