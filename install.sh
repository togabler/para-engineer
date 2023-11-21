#!/bin/bash
# Update numpy library
sudo apt-get install libatlas-base-dev -y

# Install python dependencies
python3 -m pip install -r requirements.txt

# Create autostart directory
# sudo mkdir /home/pi/.config/autostart/ -p

# Copy file to autostart directory
# sudo cp ./mini_parallel.desktop /home/pi/.config/autostart/mini_parallel.desktop

