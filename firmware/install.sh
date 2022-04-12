port="/dev/ttyACM0"
firmware_dir=$(find ~/ -type d | grep /ebobot/firmware/main | head -n 1| xargs dirname)
echo "Found firmware directory - $firmware_dir"
arduino_dir=$(find ~/ -type d | grep /Arduino/libraries | head -n 1 | xargs dirname)
if [ -d $arduino_dir ];then
echo "Found Arduino directory - $arduino_dir"
cp -rf "$firmware_dir/iarduino_MultiServo-1.1.3" "$arduino_dir/libraries"
cp -rf "$firmware_dir/LiquidCrystal_I2C_V112" "$arduino_dir/libraries"
cp -rf "$firmware_dir/TimerMs" "$arduino_dir/libraries"
cp -rf "$firmware_dir/main" "$arduino_dir/"
cp -rf "$firmware_dir/secondary" "$arduino_dir/"
cp -rf "$firmware_dir/lite_main" "$arduino_dir/"
echo "Libraries copied successfully"

cd "$arduino_dir/libraries"
rosrun rosserial_arduino make_libraries.py .
cd ..
if [  "$1" = "-port" ];then
    port=$2
    echo "Using port $port"
    else
    echo "Using default port ($port), change with '-port' option"
fi
echo "Using sketch main.ino in $(pwd)/main"
if [ "$3" = "-upload" ] || [ "$1" = "-upload" ];then
arduino --board arduino:avr:mega:cpu=atmega2560 --port $port --upload main/main.ino
else
echo "Not uploading, enable by '-upload' option"
fi
else
echo "Arduino dir not found!"
fi