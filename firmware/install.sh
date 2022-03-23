firmware_dir=$(find ~/ -type d | grep /ebobot/firmware/main | xargs dirname)
echo "Found firmware directory - $firmware_dir"
cp -rf "$firmware_dir/FaBo_PWM_PCA9685" ~/Arduino/libraries
cp -rf "$firmware_dir/LiquidCrystal_I2C_V112" ~/Arduino/libraries
cp -rf "$firmware_dir/TimerMS" ~/Arduino/libraries
cp -rf "$firmware_dir/main" ~/Arduino/
echo "Libraries copied successfully"
cd ~/Arduino/libraries
rosrun rosserial_arduino make_libraries.py .
cd ~/Arduino/main


#avrdude -p m2560 -P /dev/ttyACM0 -c stk500v1 -U flash:w:$target_file
