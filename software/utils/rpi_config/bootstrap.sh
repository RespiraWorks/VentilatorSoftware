#!/bin/bash

# This script is designed for local unix usage.
# ./gui.sh --help

# Fail if any command fails
set -e
set -o pipefail

# Print each command as it executes
set -o xtrace

# Check if Linux
PLATFORM="$(uname -s)"
if [ $PLATFORM != "Linux" ]; then
  echo "Error: This script only supports 'Linux'. You have $PLATFORM."
  exit 1
fi

sudo apt-get update
sudo apt-get -y upgrade
sudo apt-get install guake
sudo apt-get install git-lfs

git clone https://github.com/RespiraWorks/Ventilator.git
cd Ventilator
git checkout issue_1028_general_deployment_scripts

### enable serial interface but not console
sudo raspi-config nonint do_serial 2

### configure USB permissions to deploy to Nucleo
sudo echo 'ATTRS{idVendor}=="0483", ATTRS{idProduct}=="374b", MODE="666"' > /etc/udev/rules.d/99-openocd.rules

### No screensaver, but guake
sudo /bin/cp -f software/utils/rpi_config/autostart /etc/xdg/lxsession/LXDE-pi/autostart

### Desktop shortcuts
cp software/utils/rpi_config/Github /home/pi/Desktop
cp software/utils/rpi_config/*.desktop /home/pi/Desktop

##open file manager here first to generate config file
pcmanfm &
sleep 2
kill %-

### Execute desktop shortcuts without bitching
sed -i 's/quick_exec=0/quick_exec=1/' /home/pi/.config/libfm/libfm.conf

### RW theme :)
pcmanfm --set-wallpaper /home/pi/Ventilator/manufacturing/images/rendering_full.jpg

sudo ./software/gui/gui.sh --install
./software/controller/controller.sh --install

#shutdown -r now
