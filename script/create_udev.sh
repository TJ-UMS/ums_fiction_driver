
echo "usb connection as /dev/ums_fiction  lidar as /dev/ums_lidar , check it using the command : ls -l /dev|grep ttyUSB"
echo "start copy ums.rules to  /etc/udev/rules.d/"
sudo cp ./ums.rules  /etc/udev/rules.d
echo " "
echo "Restarting udev"
echo ""
sudo udevadm control --reload-rules
sudo service udev restart
sudo udevadm trigger
echo "finish "