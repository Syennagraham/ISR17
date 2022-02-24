apt-get install python-tk screen
pip3 install pyserial
echo "alias compile="arduino-cli compile --fqbn arduino:avr:uno ""  >> ~/.bashrc 
echo 'alias upload="arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:uno "'  >> ~/.bashrc 
echo 'export DISPLAY=:1'  >> ~/.bashrc 

# connect BT keyboard and mouse - https://www.cnet.com/tech/computing/how-to-setup-bluetooth-on-a-raspberry-pi-3/ 
