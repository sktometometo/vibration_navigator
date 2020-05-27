#!/bin/sh

echo "# 
SUBSYSTEMS==\"usb\", ATTRS{serial}==\"01CDD81A\", GROUP=\"dialout\", MODE=\"666\", NAME=\"vibration_navigator_left\",  SYMLINK+=\"vibration_navigator_left\"
SUBSYSTEMS==\"usb\", ATTRS{serial}==\"01CDD4B8\", GROUP=\"dialout\", MODE=\"666\", NAME=\"vibration_navigator_right\", SYMLINK+=\"vibration_navigator_right\"" \
> /tmp/15-vibration-navigator.rules

sudo sh -c "cat /tmp/15-vibration-navigator.rules > /etc/udev/rules.d/15-vibration-navigator.rules"

sudo udevadm control --reload-rules
sudo udevadm trigger
sudo adduser $USER dialout
