# Turn BT chip on
```
echo 493 > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio493/direction
echo 1 > /sys/class/gpio/gpio493/value
```

# Turn BT chip off
```
echo 0 > /sys/class/gpio/gpio493/value
```

# Connect bluez

### Move firmware to where the driver expects
```
mkdir /lib/firmware/brcm
cp /etc/bluetooth/BCM4345C0.hcd /lib/firmware/brcm
```
### Attach bluez stack to uart
```
btattach -S 921600 -P bcm -B /dev/ttyS1 &
```
### Turn of rfkill (rfkill2 might change)
```
echo 1 > /sys/class/rfkill/rfkill2/state
```
### Bring interface up and start bluez5 daemon
```
hciconfig hci0 up
/usr/libexec/bluetooth/bluetoothd &
```

# Test microphone recording use the following command
```
arecord -D sysdefault:CARD=0 --format=S16_LE --duration=5 --rate=16000 --file-type=wav out.wav
```
### Download recording
```
adb pull out.wav .
```

# Mount debugfs
```
mount -t debugfs none /sys/kernel/debug
```
# Check pins
```
cat /sys/kernel/debug/pinctrl/pinctrl@ff634480//pinmux-pins
```

# Check ALS sensor (iio\:device0 might change)
```
cat /sys/bus/iio/devices/iio\:device0/in_distance_raw
cat /sys/bus/iio/devices/iio\:device0/in_illuminance_raw
cat /sys/bus/iio/devices/iio\:device0/in_proximity_raw
```
# Check accelerometer (iio\:device1 might change)
```
cat /sys/bus/iio/devices/iio\:device1/in_accel_z_raw
cat /sys/bus/iio/devices/iio\:device1/in_accel_y_raw
cat /sys/bus/iio/devices/iio\:device1/in_accel_x_raw
```
# Test buttons, rotary and touch
```
evtest
```
# Test lcd

### generic
```
echo panel > /sys/class/display/mode
echo 1 > /sys/class/lcd/power
echo 0 > /sys/class/backlight/aml-bl/brightness
export QT_LOGGING_RULES="*=true"
export QTWEBENGINE_CHROMIUM_FLAGS="--no-sandbox --ignore-gpu-blacklist"
fbset -g 480 800 480 1600 32 -t 43403 0 0 0 0 0 0
```
###opengl
```
export QT_QPA_PLATFORM=eglfs
export QT_QPA_EGLFS_INTEGRATION=eglfs_mali
export QMLSCENE_DEVICE=""
export QT_QUICK_BACKEND=""
export QT_QPA_EGLFS_PHYSICAL_WIDTH=51
export QT_QPA_EGLFS_PHYSICAL_HEIGHT=86

qt-superbird-app
```
