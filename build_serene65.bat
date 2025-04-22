@echo off
echo Building serene65 firmware with direct ZMK behavior hook...

cd C:\Users\mpamu\code\zmk\app

echo Cleaning build directory...
west build -b serene65 -t clean

echo Building firmware...
west build -p -b serene65 -S zmk-usb-logging -- -DZMK_EXTRA_MODULES="C:/Users/mpamu/zmk-config" -DZMK_CONFIG="C:/Users/mpamu/zmk-config/config" -DCONFIG_ZMK_KSCAN_MATRIX_DRIVER=y -DCONFIG_ZMK_HALL_EFFECT=y -DCONFIG_ZMK_HALL_EFFECT_CALIBRATION=y -DCONFIG_ADC=y -DCONFIG_SETTINGS=y -DCONFIG_SETTINGS_RUNTIME=y -DCONFIG_PRINTK=y -DCONFIG_ZMK_USB_LOGGING=y -DCONFIG_LOG=y -DCONFIG_KSCAN_LOG_LEVEL_DBG=y -DCONFIG_ZMK_LOG_LEVEL_DBG=y -DCONFIG_ZMK_USB_LOGGING=y -DKCONFIG_WARN_UNUSED=y

if %ERRORLEVEL% NEQ 0 (
  echo Build failed with error code %ERRORLEVEL%
  pause
  exit /b %ERRORLEVEL%
)

echo Build completed successfully!
echo Firmware is located at: C:\Users\mpamu\code\zmk\app\build\zephyr\zmk.bin

echo Preparing to flash firmware...
echo Please put your device in bootloader mode now!
pause

:: Add dfu-util to PATH for west flash
set PATH=%PATH%;C:\QMK_MSYS\mingw64\bin

echo Flashing firmware...
west flash

echo Connection via USB and test the keyboard
pause